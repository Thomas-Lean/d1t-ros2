#!/usr/bin/env python3
# traj_test_client.py — S4a-1 手测客户端：发一条多点轨迹给 /follow_joint_trajectory，
# 打印每段 feedback 和最终 result。命令行可调点数、摆动幅度、段时长、关节名（测
# 未知名场景）、取消延时。参数错误只打用法提示，不吐裸 traceback。
#
# 摆动方向永远朝"0"那一侧（不管起点本身是正是负），不会把已经贴近限位的轴推
# 过界——2026-09-14 实机复现：折叠姿态起点（a1≈-90°、a2≈90°，天然停放位，两个
# 轴本来就几乎贴着各自限位）下，旧版"全轴统一 +step"直接把 a2 推过硬件限位，
# 第一段就被拒 out_of_limits。见 build_trajectory() 里的 toward_center。
#
# joint_signs（S4b-0 交付 2 影响到的测试工具改动）：本客户端把 /d1t/joint_feedback
# 的原始控制器角度当"当前姿态"算起点，再拼成 FollowJointTrajectory 要求的 URDF
# 弧度发给 trajectory_server。J4、J6 这两轴控制器角度与 URDF 方向相反（S3 坑 1），
# 以前 trajectory_server 收到弧度后直接转度、不乘符号，相当于把这里的"假 URDF
# 弧度"又转回了原始控制器角度，两个方向不一致的问题相互抵消，测试碰巧能过
# （S4a-1 阶段文档已记录"用 traj_test_client 直接给控制器角度，所以没暴露"）。
# trajectory_server 修了符号以后，这里必须显式把 J4/J6 的控制器角度转成真正的
# URDF 弧度（乘 JOINT_SIGNS），否则这两轴发出去的目标会变成图反的、离谱的值。
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from d1t_arm.trajectory_server import JOINT_SIGNS_DEFAULT as JOINT_SIGNS
from d1t_interfaces.msg import JointFeedback

DEFAULT_JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
NAME_TO_AXIS = {'Joint%d' % (i + 1): i for i in range(6)}

USAGE = (
    '用法: ros2 run d1t_arm traj_test_client [选项]\n'
    '  --points N            轨迹点数，默认 3（朝远离限位方向渐进摆动，最后一点回'
    '起点）\n'
    '  --step-deg D          摆动幅度上限（度），默认 15.0（实际摆动方向由起点姿态'
    '决定，见文件头注释）\n'
    '  --seg-sec S           每段 time_from_start 增量（秒），默认 0.0'
    '（=0 时全轨迹 time_from_start 全零，退回节点 speed_ratio 兜底）\n'
    '  --joint-names a,b,c   覆盖关节名列表（逗号分隔），默认 Joint1..Joint6'
    '（测未知名场景用，如 Joint1,Joint_L）\n'
    '  --cancel-after-sec T  发出 goal 后等 T 秒发取消请求，默认不取消\n')


def parse_args(argv):
    opts = {'points': 3, 'step_deg': 15.0, 'seg_sec': 0.0,
            'joint_names': list(DEFAULT_JOINT_NAMES), 'cancel_after_sec': None}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('-h', '--help'):
            return None
        if a == '--points' and i + 1 < len(argv):
            opts['points'] = int(argv[i + 1]); i += 2; continue
        if a == '--step-deg' and i + 1 < len(argv):
            opts['step_deg'] = float(argv[i + 1]); i += 2; continue
        if a == '--seg-sec' and i + 1 < len(argv):
            opts['seg_sec'] = float(argv[i + 1]); i += 2; continue
        if a == '--joint-names' and i + 1 < len(argv):
            opts['joint_names'] = argv[i + 1].split(','); i += 2; continue
        if a == '--cancel-after-sec' and i + 1 < len(argv):
            opts['cancel_after_sec'] = float(argv[i + 1]); i += 2; continue
        return None   # 未知参数 -> 打用法提示
    if opts['points'] < 1:
        return None
    return opts


class TrajTestClient(Node):

    def __init__(self, opts):
        super().__init__('traj_test_client')
        self._opts = opts
        self._last_deg = None
        self.create_subscription(JointFeedback, '/d1t/joint_feedback', self._on_fb, 10)
        self._client = ActionClient(
            self, FollowJointTrajectory, '/follow_joint_trajectory')

    def _on_fb(self, msg):
        self._last_deg = [msg.angle0, msg.angle1, msg.angle2, msg.angle3,
                           msg.angle4, msg.angle5, msg.angle6_raw]

    def wait_feedback(self, budget=5.0):
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline and self._last_deg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._last_deg is not None

    def build_trajectory(self):
        names = self._opts['joint_names']
        base_rad = [0.0] * len(names)
        for k, name in enumerate(names):
            ax = NAME_TO_AXIS.get(name)
            if ax is not None:
                base_rad[k] = math.radians(self._last_deg[ax]) * JOINT_SIGNS[ax]

        n = self._opts['points']
        step_rad = math.radians(self._opts['step_deg'])
        # 每根轴摆动方向永远朝"0"那一侧（不管起点本身是正是负），不会把已经贴近
        # 限位的轴推过界。2026-09-14 实机复现：起点是折叠姿态（a1≈-90°、a2≈90°，
        # 两个轴本来就几乎贴着各自的限位——这是设计上的天然停放位，不是异常），旧版
        # "全轴统一 +step" 把 a2 直接推过了硬件限位，第一段就 out_of_limits 被拒。
        # 中间点朝这个方向渐进摆动（幅度线性爬升到接近 step_deg），最后一点回到起点，
        # 全程不会往限位那一侧多走一步。
        toward_center = [(-1.0 if b >= 0.0 else 1.0) for b in base_rad]
        traj = JointTrajectory()
        traj.joint_names = names
        t = 0.0
        for i in range(n):
            pt = JointTrajectoryPoint()
            frac = 0.0 if i == n - 1 else (i + 1) / n   # 最后一点回起点，测试可重复跑不用复位
            pt.positions = [b + toward_center[k] * step_rad * frac
                            for k, b in enumerate(base_rad)]
            t += self._opts['seg_sec']
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)
        return traj

    def run(self):
        if not self.wait_feedback():
            print('未收到 /d1t/joint_feedback，退出（先确认 arm_driver / fake_d1t 在跑）',
                  file=sys.stderr)
            return 1
        if not self._client.wait_for_server(timeout_sec=15.0):
            print('/follow_joint_trajectory 未上线，退出', file=sys.stderr)
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.build_trajectory()
        print('发出轨迹：joint_names=%s points=%d seg_sec=%.2f'
              % (goal.trajectory.joint_names, len(goal.trajectory.points),
                 self._opts['seg_sec']))
        send_fut = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            print('goal 被拒绝（另一条轨迹在跑？）', file=sys.stderr)
            return 1

        cancel_after = self._opts['cancel_after_sec']
        if cancel_after is not None:
            deadline = time.monotonic() + cancel_after
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            print('发送取消请求（当前段走完才停，不是急停）...')
            cancel_fut = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut)

        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        wrapped = result_fut.result()
        res = wrapped.result
        print('status=%d error_code=%d error_string=%s'
              % (wrapped.status, res.error_code, res.error_string))
        return 0 if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL else 1

    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        print('feedback: joint_names=%s actual=%s desired=%s error=%s'
              % (list(fb.joint_names),
                 ['%.4f' % v for v in fb.actual.positions],
                 ['%.4f' % v for v in fb.desired.positions],
                 ['%.4f' % v for v in fb.error.positions]))


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    opts = parse_args(argv)
    if opts is None:
        print(USAGE, file=sys.stderr)
        sys.exit(1)
    rclpy.init()
    node = None
    rc = 1
    try:
        node = TrajTestClient(opts)
        rc = node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
