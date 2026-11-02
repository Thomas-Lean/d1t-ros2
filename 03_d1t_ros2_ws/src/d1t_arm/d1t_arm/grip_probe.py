# grip_probe.py — GRIP-01 实验工具：臂停 HOME 不动，只动夹爪，反复「张开 → 闭合到目标」，
# 记录每帧 /d1t/joint_feedback 与每一步的停住值。
#
# 用途：G1 空夹基线（mode=empty，无人值守）；G2 夹物对比（mode=object，每次闭合前、张开前等 Enter）。
# 动作走 arm_driver 的 /move_to_joints（7 维目标，a0~a5 = HOME，第 7 维 = 夹爪），不绕过驱动的
# 互斥与限位。每一步发命令前查：反馈新鲜、a0~a5 离 HOME 不超过 HOME_TOL_DEG——不在 HOME 就停，
# 不把臂拉回去（任务书：实验期间臂轴不动）。
# arm_pose 参数（6 个浮点数，默认 HOME）把「HOME」换成指定臂姿态（Thomas 2026-09-17：夹取姿态下标定
# 夹爪间距）。程序不负责把臂送过去：臂必须已经在该姿态 HOME_TOL_DEG 内，否则中止。
#
# 参数：
#   targets            闭合目标列表（度，只许 [20, 50]），默认 [35.0, 30.0]
#   repeats            每个目标重复次数，默认 5。顺序轮换（35,30,35,30,...），不把同一目标连做，
#                      免得时间漂移（发热等）和目标值混在一起
#   open_deg           张开目标，默认 50.0（控制器限位上沿；实测张满停 49.7）
#   mode               empty / object
#   out_dir, label     输出到 <out_dir>/grip01_<label>_<时间>_{frames,steps}.csv；label 默认 = mode
#   speed_ratio        默认 0.3（与 /move_to_joints 默认一致）
#   settle_window_sec  1.0   停住判据：t_send + 规划时长之后，最近 window 秒内的夹爪读数
#   settle_band_deg    0.25    极差 <= band（反馈量化 0.1°）
#   settle_min_frames  5       且帧数 >= min_frames（反馈 ~9 Hz，会丢帧）
#   settle_max_wait_sec 10.0   从发命令起算，超时记 not_settled、照记最后一个窗口，不中止
#
# Ctrl-C 只停本程序，不停已经发出去的夹爪命令（急停只有断电）。

import csv
import os
import statistics
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from d1t_interfaces.action import MoveToJoints
from d1t_interfaces.msg import JointFeedback

from d1t_arm.arm_driver import HOME_DEG
from d1t_arm.grip_detect import last_window, settle_window  # 与 /grip 服务共用同一停住判据

# 任务书 GRIP-01 §3：实验目标值只在 [20, 50] 内选（控制器限位是 [-20, 50]）。
EXP_TARGET_MIN = 20.0
EXP_TARGET_MAX = 50.0
# a0~a5 离 HOME 的容许偏差。HOME 停放实测 a1≈-90.4、a2≈89.6，3° 足够、又能挡住「臂不在 HOME」。
HOME_TOL_DEG = 3.0
FEEDBACK_STALE_SEC = 0.5

STEP_FIELDS = ['trial', 'phase', 'target_deg', 't_send', 'reason', 'plan_T_sec',
               'settle_status', 'settle_after_sec', 'settled_raw', 'win_min', 'win_max',
               'win_n', 'err_deg', 'arm_max_dev_deg', 'note']


class ProbeAbort(Exception):
    pass


def bad_targets(targets, lo=EXP_TARGET_MIN, hi=EXP_TARGET_MAX):
    """[lo, hi] 之外的目标值。"""
    return [t for t in targets if not lo <= float(t) <= hi]


def arm_max_dev(deg7, ref=HOME_DEG):
    """a0~a5 离参考臂姿态 ref（默认 HOME）的最大偏差（度）。"""
    return max(abs(float(deg7[i]) - float(ref[i])) for i in range(6))


def summarize(step_rows, phase='close'):
    """按目标值汇总某一 phase 的停住值：{target: (n, mean, min, max, range)}。"""
    groups = {}
    for r in step_rows:
        if r['phase'] != phase or r['settled_raw'] == '':
            continue
        groups.setdefault(float(r['target_deg']), []).append(float(r['settled_raw']))
    out = {}
    for tgt, vals in groups.items():
        out[tgt] = (len(vals), statistics.mean(vals), min(vals), max(vals),
                    max(vals) - min(vals))
    return out


class GripProbe(Node):

    def __init__(self):
        super().__init__('grip_probe')
        self.targets = [float(v) for v in self.declare_parameter(
            'targets', [35.0, 30.0]).value]
        self.repeats = int(self.declare_parameter('repeats', 5).value)
        self.open_deg = float(self.declare_parameter('open_deg', 50.0).value)
        self.mode = self.declare_parameter('mode', 'empty').value
        out_dir = self.declare_parameter('out_dir', '/tmp').value
        label = self.declare_parameter('label', '').value or self.mode
        self.speed_ratio = float(self.declare_parameter('speed_ratio', 0.3).value)
        self.win = float(self.declare_parameter('settle_window_sec', 1.0).value)
        self.band = float(self.declare_parameter('settle_band_deg', 0.25).value)
        self.min_frames = int(self.declare_parameter('settle_min_frames', 5).value)
        self.max_wait = float(self.declare_parameter('settle_max_wait_sec', 10.0).value)
        self.arm_pose = [float(v) for v in self.declare_parameter(
            'arm_pose', [float(v) for v in HOME_DEG[:6]]).value]
        if len(self.arm_pose) != 6:
            raise ValueError('arm_pose 必须是 6 个数（a0~a5），收到 %d 个' % len(self.arm_pose))

        if self.mode not in ('empty', 'object'):
            raise ValueError('mode 只能是 empty / object，收到 %r' % self.mode)
        bad = bad_targets(self.targets + [self.open_deg])
        if bad:
            raise ValueError('目标值 %s 超出实验范围 [%.0f, %.0f]' % (
                bad, EXP_TARGET_MIN, EXP_TARGET_MAX))
        if self.repeats < 1:
            raise ValueError('repeats 必须 >= 1')

        stamp = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(out_dir, 'grip01_%s_%s' % (label, stamp))
        self.frames_path = base + '_frames.csv'
        self.steps_path = base + '_steps.csv'
        self._frames_f = open(self.frames_path, 'w', newline='')
        self._frames_w = csv.writer(self._frames_f)
        self._frames_w.writerow(['host_t', 'seq', 'angle0', 'angle1', 'angle2', 'angle3',
                                 'angle4', 'angle5', 'angle6_raw'])
        self._steps_f = open(self.steps_path, 'w', newline='')
        self._steps_w = csv.DictWriter(self._steps_f, fieldnames=STEP_FIELDS)
        self._steps_w.writeheader()
        self.step_rows = []

        self._lock = threading.Lock()
        self._samples = []          # [(host_t, angle6_raw)]
        self._last_deg = None
        self._last_t = 0.0

        jf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(JointFeedback, '/d1t/joint_feedback', self._on_fb, jf_qos)
        self._client = ActionClient(self, MoveToJoints, '/move_to_joints')

    def _on_fb(self, msg):
        t = time.time()
        deg7 = [msg.angle0, msg.angle1, msg.angle2, msg.angle3, msg.angle4, msg.angle5,
                msg.angle6_raw]
        with self._lock:
            self._samples.append((t, msg.angle6_raw))
            self._last_deg = deg7
            self._last_t = t
            self._frames_w.writerow(['%.3f' % t, msg.seq] + ['%.2f' % v for v in deg7])
            self._frames_f.flush()

    def close_files(self):
        with self._lock:
            self._frames_f.close()
        self._steps_f.close()

    # --- 一步：发一个夹爪目标，等驱动回结果，再等夹爪停住 ---
    def _step(self, trial, phase, target, note=''):
        with self._lock:
            deg7 = list(self._last_deg) if self._last_deg else None
            fresh = time.time() - self._last_t <= FEEDBACK_STALE_SEC
        if deg7 is None or not fresh:
            raise ProbeAbort('反馈不新鲜（>%.1f s 没收到 /d1t/joint_feedback）' % FEEDBACK_STALE_SEC)
        dev = arm_max_dev(deg7, self.arm_pose)
        if dev > HOME_TOL_DEG:
            raise ProbeAbort('臂不在 arm_pose %s：a0~a5 = %s，最大偏差 %.2f° > %.1f°，不动' % (
                self.arm_pose, ['%.2f' % v for v in deg7[:6]], dev, HOME_TOL_DEG))

        goal = MoveToJoints.Goal()
        goal.target_deg = list(self.arm_pose) + [float(target)]
        goal.timeout_sec = 10.0
        goal.speed_ratio = self.speed_ratio
        t_send = time.time()
        gh = self._wait(self._client.send_goal_async(goal), 5.0, 'goal 发送')
        if not gh.accepted:
            raise ProbeAbort('arm_driver 拒绝 goal（另有 goal/servo 在跑？）')
        result = self._wait(gh.get_result_async(), 15.0, 'goal 结果').result
        if result.reason != 'arrived':
            raise ProbeAbort('驱动结果 reason=%s（不是 arrived），final_deg=%s' % (
                result.reason, ['%.2f' % v for v in result.final_deg]))

        t_from = t_send + result.actual_duration_sec
        status = 'settled'
        while True:
            time.sleep(0.05)
            now = time.time()
            with self._lock:
                samples = list(self._samples)
            win = settle_window(samples, t_from, now, self.win, self.band, self.min_frames)
            if win is not None:
                break
            if now - t_send > self.max_wait:
                status = 'not_settled'
                win = last_window(samples, t_from, now, self.win)
                break
        if win is None:
            raise ProbeAbort('%.1f s 内没收到夹爪反馈' % self.max_wait)
        with self._lock:
            # 窗口结束时刻前的最后一帧 a0~a5：确认整步臂没被带动
            dev_end = arm_max_dev(self._last_deg, self.arm_pose)
            self._samples = [s for s in self._samples if s[0] >= now - 5.0]
        med, vmin, vmax, n = win
        row = {
            'trial': trial, 'phase': phase, 'target_deg': '%.1f' % target,
            't_send': '%.3f' % t_send, 'reason': result.reason,
            'plan_T_sec': '%.3f' % result.actual_duration_sec,
            'settle_status': status, 'settle_after_sec': '%.2f' % (now - t_send),
            'settled_raw': '%.2f' % med, 'win_min': '%.2f' % vmin, 'win_max': '%.2f' % vmax,
            'win_n': n, 'err_deg': '%+.2f' % (med - target),
            'arm_max_dev_deg': '%.2f' % max(dev, dev_end), 'note': note}
        self._steps_w.writerow(row)
        self._steps_f.flush()
        self.step_rows.append(row)
        print('  [%s] trial=%d 目标 %.1f → 停住 %.2f（窗口 %.2f~%.2f，%d 帧，%s，发令后 %.1f s）'
              ' 差 %+.2f' % (phase, trial, target, med, vmin, vmax, n, status,
                             now - t_send, med - target), flush=True)
        return row

    def _wait(self, future, timeout, what):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                raise ProbeAbort('%s 等待超时（%.0f s）' % (what, timeout))
            time.sleep(0.02)
        return future.result()

    def _prompt(self, text):
        ans = input(text).strip()
        if ans.lower() == 'q':
            raise ProbeAbort('操作者输入 q 结束')
        return ans

    def run(self):
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise ProbeAbort('/move_to_joints 不在（arm_driver 没起？）')
        t0 = time.time()
        while self._last_deg is None:
            if time.time() - t0 > 5.0:
                raise ProbeAbort('5 s 内没收到 /d1t/joint_feedback')
            time.sleep(0.05)
        total = self.repeats * len(self.targets)
        print('GRIP-01 grip_probe mode=%s targets=%s repeats=%d（共 %d 次闭合）open=%.1f arm_pose=%s' % (
            self.mode, self.targets, self.repeats, total, self.open_deg, self.arm_pose))
        print('  frames: %s\n  steps : %s' % (self.frames_path, self.steps_path), flush=True)

        self._step(0, 'open', self.open_deg)
        trial = 0
        for rep in range(self.repeats):
            for tgt in self.targets:
                trial += 1
                note = ''
                if self.mode == 'object':
                    self._prompt('\n第 %d/%d 次，目标 %.1f：托住瓶身放进两指中间（手指别伸进两指之间），'
                                 '按 Enter 夹紧；输入 q 回车结束 > ' % (trial, total, tgt))
                self._step(trial, 'close', tgt)
                if self.mode == 'object':
                    # 这里的 q 不能当场中止：夹爪正夹着物体，先张开再结束
                    note = input('托住瓶子，按 Enter 张开（有异常先打一句备注再回车，如「没对正」'
                                 '「滑了」；输入 q 回车 = 张开后结束）> ').strip()
                self._step(trial, 'open', self.open_deg, note=note)   # 备注记在本次 open 行
                if note.lower() == 'q':
                    raise ProbeAbort('操作者输入 q 结束（已张开）')
        self._print_summary()

    def _print_summary(self):
        print('\n===== 闭合停住值汇总（angle6_raw）=====')
        for tgt, (n, mean, vmin, vmax, rng) in sorted(summarize(self.step_rows).items(),
                                                     reverse=True):
            print('  目标 %.1f: n=%d 均值 %.2f 最小 %.2f 最大 %.2f 极差 %.2f 均值差 %+.2f' % (
                tgt, n, mean, vmin, vmax, rng, mean - tgt))
        opens = summarize(self.step_rows, phase='open')
        for tgt, (n, mean, vmin, vmax, rng) in opens.items():
            print('  张开 %.1f: n=%d 均值 %.2f 最小 %.2f 最大 %.2f 极差 %.2f' % (
                tgt, n, mean, vmin, vmax, rng))
        print('  steps: %s\n  frames: %s' % (self.steps_path, self.frames_path), flush=True)


def main():
    rclpy.init()
    node = GripProbe()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    rc = 0
    try:
        node.run()
    except ProbeAbort as exc:
        print('\n[中止] %s' % exc, flush=True)
        last = node._last_deg
        if last is not None and arm_max_dev(last, node.arm_pose) <= HOME_TOL_DEG:
            print('夹爪停在当前位置。要张开：ros2 action send_goal /move_to_joints '
                  'd1t_interfaces/action/MoveToJoints "{target_deg: [%s, 50.0]}"'
                  % ', '.join('%.1f' % v for v in node.arm_pose), flush=True)
        else:
            # 臂不在 arm_pose 时不给命令：那条命令会把臂拉到 arm_pose
            print('臂不在 arm_pose，不给张开命令（会拉动臂轴）', flush=True)
        rc = 1
    except (KeyboardInterrupt, EOFError):
        print('\n[中止] 操作者中断；已发出的夹爪命令不会被撤回', flush=True)
        rc = 130
    finally:
        node.close_files()
        executor.shutdown()
        spin.join(timeout=2.0)   # 不等 spin 线程退出就销毁节点，退出时偶发 core dump（仿真复现过）
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
