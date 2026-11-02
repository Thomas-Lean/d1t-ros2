"""fake_d1t：D1-T 控制器的协议仿真节点。

订阅 /arm_Command，按 D1-T JSON 协议回 recv 回执，并以 10 Hz 在 /arm_Feedback
发布匀速直线运动的七轴角度与状态帧。只仿真协议时序，不仿真物理。
参数：drop_rate(整条丢弃概率)、speed_deg_per_s(匀速)、start_delay_sec(recv 到起动)、
      initial_angles(启动七轴)、grip_block_raw(夹爪被物体挡住的停止值，GRIP-01)。
只认 funcode=1（单关节，data={id,angle,delay_ms}）：只回 recv、不回 exec（真控制器
      exec 那行被注释掉了）；置 enable_status=1；delay_ms 不用于运动（仿真只保匀速
      时序，不仿真梯形曲线）。其它 funcode 一律 WARN 忽略——S-aux4（2026-09-11）
      随 funcode=2（全关节，带 mode）通路退役一并删除，那是它唯一的产生方。
未验证假设 1：回执的 seq 同时放在根和 data 两层，真实控制器放在哪层未验证。
未验证假设 2：运动中收到新命令即换目标、重置延迟状态，真实控制器是否如此未验证。
"""

import json
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_arm.msg import ArmString

TICK = 0.1          # 关节/状态各 10 Hz
HALF_TICK = 0.05    # 两类帧错开半拍发布，见 on_tick 注释


class FakeD1T(Node):

    def __init__(self):
        super().__init__('fake_d1t')
        self.drop_rate = self.declare_parameter('drop_rate', 0.0).value
        self.speed = self.declare_parameter('speed_deg_per_s', 11.5).value
        self.start_delay = self.declare_parameter('start_delay_sec', 0.3).value
        self.angles = list(
            self.declare_parameter(
                'initial_angles',
                [0.0, -90.4, 89.6, -0.3, 8.9, -0.3, 49.8]).value)
        # S4b-3 注入「命令回执但轴不动」：
        #   stall_axes         只忽略这些轴的第一条命令（回 recv、不动），之后恢复
        #   stall_axes_freeze  永远忽略这些轴（回 recv、永远不动）——验「重发后仍不动，
        #                       按原超时逻辑走、不无限重试」
        stall_axes = self.declare_parameter(
            'stall_axes', rclpy.Parameter.Type.INTEGER_ARRAY).value
        stall_freeze = self.declare_parameter(
            'stall_axes_freeze', rclpy.Parameter.Type.INTEGER_ARRAY).value
        self.stall_axes = [int(v) for v in (stall_axes or [])]
        self.stall_axes_freeze = [int(v) for v in (stall_freeze or [])]
        self._stall_once_done = set()
        # GRIP-01 注入「夹爪被物体挡住」：夹爪（angle6）从上方往下闭合时停在 grip_block_raw，
        # 不再往下走；张开不受影响。默认 -100（低于控制器限位 -20）= 不注入。每拍重读，
        # 可用 ros2 param set /fake_d1t grip_block_raw 36.7 运行中切换（必须写成浮点数）。
        self.declare_parameter('grip_block_raw', -100.0)

        self.enable_status = 0
        self.fb_seq = 0
        self.cmd_seq = None      # 当前命令的 seq
        self.target = None       # 当前目标七轴；None 表示无在途命令
        self.t_recv = 0.0
        self.moved = False       # MOVE 日志只打一次
        self.arrived = False

        # 与 d1t_feedback_node.cpp 的 /arm_Feedback 订阅端一致：RELIABLE/KEEP_LAST(1)/VOLATILE
        fb_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # 镜像真实控制器的命令 reader：BEST_EFFORT/KEEP_LAST(1)/VOLATILE
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(ArmString, '/arm_Feedback', fb_qos)
        self.sub = self.create_subscription(
            ArmString, '/arm_Command', self.on_command, cmd_qos)
        self.phase = False
        self.create_timer(HALF_TICK, self.on_tick)
        self.get_logger().info(
            'fake_d1t started: drop_rate=%.2f speed=%.2f deg/s start_delay=%.2f'
            % (self.drop_rate, self.speed, self.start_delay))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, address, funcode, data):
        frame = {'seq': data.get('seq', self.fb_seq), 'address': address,
                 'funcode': funcode, 'data': data}
        msg = ArmString()
        msg.data = json.dumps(frame)
        self.pub.publish(msg)

    def on_command(self, msg):
        try:
            root = json.loads(msg.data)
            funcode = int(root['funcode'])
            if funcode != 1:
                self.get_logger().warn('ignored funcode=%s' % root.get('funcode'))
                return
            seq = int(root['seq'])
            data = root['data']
            if random.random() < self.drop_rate:
                self.get_logger().warn('DROP seq=%d' % seq)
                return
            self.send(3, 1, {'seq': seq, 'recv_status': 1})
            self.cmd_seq = seq
            self.enable_status = 1

            # 单关节 + 到位时间。只回 recv、不回 exec。delay_ms 收下不用于运动。
            jid = int(data['id'])
            angle = float(data['angle'])
            self.get_logger().info(
                'RECV seq=%d funcode=1 id=%d angle=%.2f delay_ms=%s'
                % (seq, jid, angle, data.get('delay_ms')))
            # S4b-3 注入卡死：stall_axes 的轴忽略第一条命令，stall_axes_freeze 的轴
            # 永久忽略。recv 已在上方发过——与真机「全回执但轴不动」表现一致。
            if jid in self.stall_axes_freeze or (
                    jid in self.stall_axes and jid not in self._stall_once_done):
                self._stall_once_done.add(jid)
                self.get_logger().warn(
                    'STALL_INJECT jid=%d seq=%d（回执已发，目标不动）' % (jid, seq))
                return
            base = list(self.target) if self.target is not None else list(self.angles)
            base[jid] = angle
            new_target = base

            # 相同目标不重置状态机：只刷新 cmd_seq、照常回 recv，保留 t_recv / moved /
            # arrived，让运动按首帧的 start_delay 继续。这是仿真侧假设；真机「运动中
            # 收新命令是否立即换目标」仍未验证。
            if new_target != self.target:
                self.target = new_target
                self.t_recv = self.now()
                self.moved = False
                self.arrived = False
        except Exception as exc:  # 协议约定：不做字段校验，异常统一记为 WARN
            self.get_logger().warn('bad command: %r | raw=%s' % (exc, msg.data))

    def on_tick(self):
        # 订阅端 KEEP_LAST(1)：同一时刻连发关节帧和状态帧会互相顶掉，
        # 因此按 50 ms 交替发布，两类帧各保持 10 Hz。
        self.phase = not self.phase
        if not self.phase:
            self.send(2, 3, {'enable_status': self.enable_status,
                             'power_status': 0, 'error_status': 0})
            return

        if self.target is not None:
            elapsed = self.now() - self.t_recv
            if elapsed >= self.start_delay:
                if not self.moved:
                    self.moved = True
                    self.get_logger().info('MOVE seq=%d' % self.cmd_seq)
                step = self.speed * TICK
                block = float(self.get_parameter('grip_block_raw').value)
                for i in range(7):
                    err = self.target[i] - self.angles[i]
                    new = self.angles[i] + max(-step, min(step, err))
                    if i == 6 and self.angles[6] >= block > new:
                        new = block
                    self.angles[i] = new
                if not self.arrived and all(
                        abs(self.target[i] - self.angles[i]) < 0.05
                        for i in range(7)):
                    self.arrived = True
                    self.get_logger().info('ARRIVED seq=%d' % self.cmd_seq)

        self.fb_seq += 1
        self.send(2, 1, {'angle%d' % i: self.angles[i] for i in range(7)})


def main():
    rclpy.init()
    node = FakeD1T()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM 经 rclpy 信号处理转成 ExternalShutdownException：此时 rcl 已 shutdown，
        # 再调 rclpy.shutdown() 会抛 "rcl_shutdown already called"。故 finally 里加 rclpy.ok() 守卫。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
