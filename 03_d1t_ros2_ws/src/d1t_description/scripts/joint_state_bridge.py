#!/usr/bin/env python3
"""joint_state_bridge：/d1t/joint_feedback -> /joint_states 的真实数据通路组件。

angle0..angle5（度）经 deg2rad 和 joint_signs 符号翻转后填入 Joint1..Joint6；
angle6_raw 经 S3.1 实测线性拟合换算成爪间距，均分到 Joint_L、Joint_R 两个 prismatic。
source=static 时不订阅反馈、夹爪保持闭合，以 10 Hz 发布 static_angles（六个度数）预览臂姿态。
joint_signs、static_angles 用 dynamic_typing 声明，整数/浮点数组都接受，读入后统一转 float。
static_angles 长度不为 6 时构造期 warn 一次并归一到 6（多余截断、不足补 0）；不校验取值范围。
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState

from d1t_interfaces.msg import JointFeedback

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6',
               'Joint_L', 'Joint_R']
_ANY = ParameterDescriptor(dynamic_typing=True)


class JointStateBridge(Node):

    def __init__(self):
        super().__init__('joint_state_bridge')
        # 默认值固化 S3 步骤3 实测：Joint4、Joint6 实机方向与 URDF axis 相反，翻转符号
        self.signs = [float(x) for x in self.declare_parameter(
            'joint_signs', [1.0, 1.0, 1.0, -1.0, 1.0, -1.0], _ANY).value]
        source = self.declare_parameter('source', 'feedback').value
        raw = [float(x) for x in self.declare_parameter(
            'static_angles', [0.0] * 6, _ANY).value]
        if len(raw) != 6:
            self.get_logger().warn(
                'static_angles 长度为 %d，归一到 6：多余截断、不足补 0' % len(raw))
        self.static_angles = (raw + [0.0] * 6)[:6]
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        if source == 'static':
            self.create_timer(0.1, lambda: self._publish(self.static_angles))
        else:
            self.create_subscription(
                JointFeedback, '/d1t/joint_feedback', self._on_feedback, 10)

    def _publish(self, deg6, grip=0.0):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        # angle6_raw -> 爪间距 mm：S3.1 实测线性拟合，量程内 ±2mm，两端截断到机构极限
        gap = 0.0 if grip == 9999.0 else min(max(0.85 * grip + 19.8, 0.0), 62.0)
        # URDF Joint_L/R prismatic：FK 实测 position 0 = 最大张开约 66mm、0.033 = 闭合，
        # 与真实 angle6_raw 反向，故每爪 position = 0.033 - gap/2，截断到限位 [0, 0.033]
        pos = min(max(0.033 - gap / 2000.0, 0.0), 0.033)
        msg.position = [math.radians(deg6[i]) * self.signs[i]
                        for i in range(6)] + [pos, pos]
        self.pub.publish(msg)

    def _on_feedback(self, fb):
        self._publish([fb.angle0, fb.angle1, fb.angle2,
                       fb.angle3, fb.angle4, fb.angle5], fb.angle6_raw)


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
