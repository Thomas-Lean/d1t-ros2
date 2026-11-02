#!/usr/bin/env python3
"""s4b3_run10.py — S4b-3 任务④：execution_mode=stream 下同一目标连跑 N 次。

先从当前位置定位到该目标的起点（不计轮次，照常打印），然后做 N 轮「起点→目标 去程 +
目标→起点 回程」的 plan+execute（MoveGroup /move_action，plan_only=False），每次打印 error_code / 规划点数 / 耗时 / 起止 unixtime（供事后与
arm_driver 日志按时间对齐，逐次找 STALL_AXIS）。失败不中断，跑满 N 次，最后打汇总。

只验「不卡」（判据 1 的一半：0 次超时、0 次无响应）。STALL / STALL_AXIS 次数从
arm_driver 与 trajectory_server 的日志里 grep，本脚本不读。

用法：
  ros2 run d1t_moveit_config s4b3_run10 --ros-args -p target:=bypass -p count:=10 \
      -p velocity_scaling:=0.5 -p acceleration_scaling:=0.5

target: t1 | t2 | t3 | bypass（默认 t1）
  t1/t2/t3：起点 HOME。
  bypass：起点 A=(J1=-70,J2=13,J3=54)，目标 B=(J1=+70,J2=20,J3=60)。B 与 s4b2_demo 相同；A 由
    s4b2 的 (J2=20,J3=60) 抬高（S4b-3 主窗口裁定 0146d33：原 A 按模型离桌面仅 5.3 mm，桌面碰撞盒
    外扩 30 mm 后原 A 直接判碰撞；J1 不变，取 J2/J3 最小改动使离桌面 ≥30 mm，按 s4b3_clearance
    同一算法得 32.6 mm）。第五轮裁定 784a847：目标点名义距离 ≥45 mm（外扩 30 mm + 到位误差），再抬到
    (-70,13,54)，同一算法 46.9 mm（J2 偏 +2° 时 38.4 mm，仍在外扩盒外）。
    A/B 之间 J1∈[-40,+30]（J2=20/J3=60 固定时）是碰撞封锁区，去程 A→B 与回程 B→A 两条腿
    都必须绕行，都算移动次数。HOME→B 不一定穿封锁区，所以不从 HOME 出发。
前置：move_group + trajectory_server(execution_mode:=stream) + arm_driver 已在跑。
"""
import datetime
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from d1t_interfaces.msg import JointFeedback

JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
HOME = [0.0, math.radians(-90.0), math.radians(90.0), 0.0, 0.0, 0.0]
# B 与 s4b2_demo 的 TARGET_B 同值；A 已抬高，见文件头
BYPASS_A = [math.radians(-70.0), math.radians(13.0), math.radians(54.0), 0.0, 0.0, 0.0]
BYPASS_B = [math.radians(70.0), math.radians(20.0), math.radians(60.0), 0.0, 0.0, 0.0]
# key -> (目标名, 目标, 起点名, 起点)
# 与 s4b1_target_demo 一致：T3=(0,7,55)（第五轮裁定 784a847：名义距离 ≥45 mm；(0,9,57) 实际停位离外扩盒不足 1 mm）
TARGETS = {
    't1': ('T1_仅Joint1', [math.radians(20.0), HOME[1], HOME[2], 0.0, 0.0, 0.0], 'HOME', HOME),
    't2': ('T2_J4J6都非零', [HOME[0], HOME[1], HOME[2], math.radians(20.0), 0.0, math.radians(-20.0)],
           'HOME', HOME),
    't3': ('T3_木板上方', [0.0, math.radians(7.0), math.radians(55.0), 0.0, 0.0, 0.0],
           'HOME', HOME),
    'bypass': ('绕行B', BYPASS_B, '绕行A', BYPASS_A),
}

ERR_NAME = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
            if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}

GOAL_TIMEOUT_SEC = 120.0
# 只对 INVALID_MOTION_PLAN（-2：规划出的路在最后校验被判碰撞，「Not executing」，臂不动）自动重试，
# 最多再试 2 次（S4b-3 主窗口裁定 3b8db6d）。重试前还核对反馈没动过，动过就不重试。
RETRY_CODES = (MoveItErrorCodes.INVALID_MOTION_PLAN,)
MAX_RETRIES = 2
MOVED_TOL_DEG = 1.0
JOINT_SIGNS = [1.0, 1.0, 1.0, -1.0, 1.0, -1.0]   # URDF -> 控制器角度，与 trajectory_server 一致       # 单次 plan+execute 的上限；stream 下抽稀后每段只几秒


class Run10(Node):
    def __init__(self):
        super().__init__('s4b3_run10')
        self.target_key = self.declare_parameter('target', 't1').value
        self.count = int(self.declare_parameter('count', 10).value)
        self.vel_scale = self.declare_parameter('velocity_scaling', 0.5).value
        self.acc_scale = self.declare_parameter('acceleration_scaling', 0.5).value
        # 每次移动的 MoveIt 规划轨迹落盘（控制器角度，格式同 trajectory_server 反馈 CSV），
        # 供 s4b3_clearance 算「规划路径本身离障碍多远」；空字符串 = 不落盘
        self.plan_csv_dir = self.declare_parameter('plan_csv_dir', '/tmp').value
        if self.target_key not in TARGETS:
            raise ValueError('target=%s 无效，可选 %s' % (self.target_key, list(TARGETS)))
        self._last_deg = None
        self.create_subscription(JointFeedback, '/d1t/joint_feedback', self._on_fb, 10)
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def _on_fb(self, msg):
        self._last_deg = [msg.angle0, msg.angle1, msg.angle2, msg.angle3,
                          msg.angle4, msg.angle5, msg.angle6_raw]

    def wait_ready(self, budget=20.0):
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline and self._last_deg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._client.wait_for_server(timeout_sec=budget) and self._last_deg is not None

    def goto(self, label, joint_positions):
        """一次移动，含对 INVALID_MOTION_PLAN 的自动重试。
        返回 (最终是否成功, 最后一次耗时, 首次是否成功, 共发了几次)。
        规划耗时不在这里算：MoveGroup 结果的 planning_time 在 plan+execute 下恒为 0（move_group
        不回填），改从 move_group 日志的时间戳算。"""
        first_ok = None
        for attempt in range(1 + MAX_RETRIES):
            before = list(self._last_deg) if self._last_deg else None
            tag = label if attempt == 0 else '%s [重试 %d/%d]' % (label, attempt, MAX_RETRIES)
            code, elapsed = self._goto_once(tag, joint_positions)
            ok = code == MoveItErrorCodes.SUCCESS
            if first_ok is None:
                first_ok = ok
            if ok or code not in RETRY_CODES or attempt == MAX_RETRIES:
                return ok, elapsed, first_ok, attempt + 1
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.1)
            if before is None or self._last_deg is None or any(
                    abs(a - b) > MOVED_TOL_DEG for a, b in zip(before[:6], self._last_deg[:6])):
                print('    反馈与发令前相差 >%.1f°（或无反馈），不重试' % MOVED_TOL_DEG)
                return ok, elapsed, first_ok, attempt + 1
            print('    %s：臂未动，自动重试' % ERR_NAME.get(code, code))
        return False, None, first_ok, 1 + MAX_RETRIES

    def _goto_once(self, label, joint_positions):
        """发一次 MoveGroup 规划+执行。返回 (error_code 或 None, 耗时或 None)。"""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        c = Constraints()
        for name, pos in zip(JOINTS, joint_positions):
            jc = JointConstraint(joint_name=name, position=pos,
                                  tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False

        t0 = time.monotonic()
        t0_epoch = time.time()
        send_fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            print('  %s: goal 被拒绝（另一个 goal/servo 在跑，或未就绪）' % label)
            return None, None
        result_fut = handle.get_result_async()
        deadline = time.monotonic() + GOAL_TIMEOUT_SEC
        while not result_fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not result_fut.done():
            print('  %s: 等待 %ds 无结果（超时/无响应），发取消' % (label, GOAL_TIMEOUT_SEC))
            cancel_fut = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=10.0)
            return None, None
        res = result_fut.result().result
        code = res.error_code.val
        n_pts = len(res.planned_trajectory.joint_trajectory.points)
        elapsed = time.monotonic() - t0
        print('  %s [t0=%.2f epoch] error_code=%d(%s) 规划点数=%d 耗时=%.3fs'
              % (label, t0_epoch, code, ERR_NAME.get(code, '?'), n_pts, elapsed))
        if n_pts and self.plan_csv_dir:
            print('    规划轨迹 CSV=%s' % self._write_plan_csv(res.planned_trajectory.joint_trajectory))
        if self._last_deg:
            print('    执行后反馈(controller-deg a0..a6)=%s'
                  % ['%.2f' % v for v in self._last_deg])
        return code, elapsed

    def at(self, joint_positions, tol_deg=3.0):
        """当前反馈是否在给定关节目标（URDF 弧度）的 tol_deg 以内，只比 J1~J6。"""
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_deg is None:
            return False
        return all(abs(self._last_deg[i] - math.degrees(joint_positions[i]) * JOINT_SIGNS[i]) <= tol_deg
                   for i in range(6))

    def _write_plan_csv(self, jt):
        idx = [list(jt.joint_names).index(j) for j in JOINTS]
        a6 = self._last_deg[6] if self._last_deg else 0.0
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(self.plan_csv_dir, 's4b3_plan_%s.csv' % stamp)
        with open(path, 'w') as f:
            f.write('t_rel_s,' + ','.join('a%d' % i for i in range(7)) + '\n')
            for pt in jt.points:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                deg = [math.degrees(pt.positions[k]) * JOINT_SIGNS[i] for i, k in enumerate(idx)]
                f.write('%.4f,' % t + ','.join('%.3f' % v for v in deg + [a6]) + '\n')
        return path


def main():
    rclpy.init()
    node = Run10()
    if not node.wait_ready():
        print('依赖未就绪：/move_action 或 /d1t/joint_feedback', file=sys.stderr)
        sys.exit(1)
    label, target_deg, start_label, start_deg = TARGETS[node.target_key]
    print('== %s 连跑 %d 轮（%s→%s 去程 + 回程各 1 次移动 = 1 轮），vel=%s acc=%s =='
          % (label, node.count, start_label, label, node.vel_scale, node.acc_scale))
    ok0 = node.goto('定位到%s（不计轮次）' % start_label, start_deg)
    if not ok0[0]:
        print('定位到%s 失败，不开始计轮' % start_label, file=sys.stderr)
        sys.exit(1)

    # 某条腿失败时臂留在原地：下一条腿的起点就不对（例如回程失败后「去程」变成 B→B，4~5 个点、
    # 不穿封锁区却算成功——S4b-3 仿真闸门实测踩过）。所以：去程失败就不发回程；每轮开头不在起点
    # 就先补位（不计轮次），补位失败这一轮整轮跳过。只数真正从正确起点出发的移动。
    ok_total = 0
    first_ok_total = 0
    retries_total = 0
    tried = 0
    skipped = 0
    elapsed_all = []
    for i in range(1, node.count + 1):
        print('-- 第 %d 轮 --' % i)
        if not node.at(start_deg):
            if not node.goto('补位到%s（不计轮次）' % start_label, start_deg)[0]:
                print('  第 %d 轮跳过：补位到%s 失败，起点不对' % (i, start_label))
                skipped += 2
                continue
        ok1, e1, f1, n1 = node.goto('%s 去程 %d/%d' % (label, i, node.count), target_deg)
        tried += 1
        ok_total += int(ok1)
        first_ok_total += int(f1)
        retries_total += n1 - 1
        if e1 is not None:
            elapsed_all.append(e1)
        if not ok1:
            print('  去程失败，本轮回程不发（起点不对），计跳过 1 次')
            skipped += 1
            continue
        ok2, e2, f2, n2 = node.goto('%s 回%s %d/%d' % (label, start_label, i, node.count),
                                         start_deg)
        tried += 1
        ok_total += int(ok2)
        first_ok_total += int(f2)
        retries_total += n2 - 1
        if e2 is not None:
            elapsed_all.append(e2)

    n_ok = ok_total
    total = node.count * 2
    print('== 汇总 ==')
    print('首次成功 %d/%d 次实际发出的移动（不含重试）' % (first_ok_total, tried))
    print('重试后成功 %d/%d 次实际发出的移动（INVALID_MOTION_PLAN 最多再试 %d 次，共重试 %d 次）'
          % (n_ok, tried, MAX_RETRIES, retries_total))
    print('因起点不对跳过 %d 次；计划 %d 次%s'
          % (skipped, total, '' if n_ok == total else '  ← 有失败或跳过'))
    if elapsed_all:
        srt = sorted(elapsed_all)
        print('耗时分布(去+回合计 %d 次移动): min=%.3f med=%.3f max=%.3f'
              % (len(elapsed_all), srt[0], srt[len(srt) // 2], srt[-1]))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if n_ok == total else 1)


if __name__ == '__main__':
    main()