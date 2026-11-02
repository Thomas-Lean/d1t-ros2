#!/usr/bin/env python3
"""set_desk_collision.py — S4b-1：给规划场景加桌面+木工夹碰撞体，一次性脚本，不是
常驻节点。调用 /apply_planning_scene（moveit_msgs/srv/ApplyPlanningScene），发一次
就退出。move_group 得先起来（这个服务由 move_group 提供）。

尺寸来源：Thomas 2026-09-16 实测（base_link 坐标系，米）：
  桌面 z = -0.020（与固定机械臂的木板厚度 20mm 吻合：base_link 在木板顶面，
  木板底面贴着桌面，二者共享同一个 z）。
  桌面范围：x∈[-0.200, 1.210]，y∈[-0.110, 0.700]，边界外悬空、不是桌面
  （Thomas 明确说了"其余部分都不是桌面，是悬空"，不再是占位假设）。
  木工夹（把固定臂的木板夹在桌面上）：位置 x=0.220 y=-0.110（正好在桌面 y
  下边界，符合"夹在木板边缘"的常识），尺寸 90x20x70mm，坐在桌面上（底面贴
  桌面 z=-0.020，往上凸 0.070）。长边(90mm)沿 Y、短边(20mm)沿 X——Thomas
  实测后改过，这版是他改的方向，不是猜的。

  固定臂的木板本身（S4b-1 首次实机测试漏建，a1 转正方向时夹爪顶到了木板，
  暴露这个洞）：440(沿X)x220(沿Y)x20mm，中心相对 base_link 在
  (x=+0.028, y=+0.010)，z 从 -0.020（贴桌面）到 0.000（base_link 落脚的
  顶面）。方向已跟 Thomas 确认（440mm 沿 X），不是猜的——第一版猜错方向、
  按 440x440 正方形建，误伤了 T3 那个合法目标（指尖离木板仅 0.5~2.6mm 就被
  判碰撞），改成精确长方形后解除。
"""
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# 桌面顶面高度（base_link 系），Thomas 实测
DESK_TOP_Z = -0.020
# 桌面范围（Thomas 实测，边界外悬空）
DESK_X_MIN, DESK_X_MAX = -0.200, 1.210
DESK_Y_MIN, DESK_Y_MAX = -0.110, 0.700
# 碰撞体往下的厚度（当"地板"用，不需要对应桌子实际厚度）
DESK_THICKNESS = 0.480

# 木工夹：位置是 Thomas 报的点（当中心处理），坐在桌面上往上凸
CLAMP_CENTER_X = 0.220
CLAMP_CENTER_Y = -0.110
# Thomas 实测后确认：长边(90mm)沿 Y、短边(20mm)沿 X
CLAMP_SIZE_X = 0.020
CLAMP_SIZE_Y = 0.090
CLAMP_HEIGHT = 0.070

# 固定臂的木板本身（z 从 -0.020 贴桌面到 0.000 base_link 落脚面）
BOARD_CENTER_X = 0.028
BOARD_CENTER_Y = 0.010
# Thomas 确认：440mm 沿 X、220mm 沿 Y
BOARD_SIZE_X = 0.440
BOARD_SIZE_Y = 0.220
BOARD_TOP_Z = 0.000
BOARD_THICKNESS = 0.020

# 安全余量（S4b-3 主窗口裁定，2026-09-16，START_HERE 3c8ed7a / 0146d33）：桌面、木工夹、木板
# 三个碰撞盒一律向四周和向上各扩 30 mm，底面不动，不设端点豁免。上面的尺寸常量保持实测值，
# 余量只在建碰撞体时加（s4b3_clearance 用 margin=0 取实测尺寸做硬指标）。
# 误差账：执行偏差实测最大 1.5°≈10 mm（指尖 0.4 m 处）+ 碰撞盒手量约 5 mm（估计）+
# stream 切角 zone_deg=2 约 14~17 mm。起因：不加余量时 RRTConnect 随机路径擦着障碍过，
# 实机指尖多次碰到木工夹（按模型逐帧算最近 −3.0 / 1.0 / 1.1 mm）。
SAFETY_MARGIN = 0.030


def _box_object(object_id, center_x, center_y, top_z, size_x, size_y, size_z):
    obj = CollisionObject()
    obj.header.frame_id = 'base_link'
    obj.id = object_id
    obj.operation = CollisionObject.ADD
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size_x, size_y, size_z])
    obj.primitives = [prim]
    pose = Pose()
    pose.position.x = center_x
    pose.position.y = center_y
    pose.position.z = top_z - size_z / 2.0
    pose.orientation.w = 1.0
    obj.primitive_poses = [pose]
    return obj


def build_desk_object(margin=SAFETY_MARGIN):
    size_x = DESK_X_MAX - DESK_X_MIN
    size_y = DESK_Y_MAX - DESK_Y_MIN
    center_x = (DESK_X_MAX + DESK_X_MIN) / 2.0
    center_y = (DESK_Y_MAX + DESK_Y_MIN) / 2.0
    return _box_object('desk', center_x, center_y, DESK_TOP_Z + margin,
                        size_x + 2 * margin, size_y + 2 * margin, DESK_THICKNESS + margin)


def build_clamp_object(margin=SAFETY_MARGIN):
    clamp_top_z = DESK_TOP_Z + CLAMP_HEIGHT
    return _box_object('desk_clamp', CLAMP_CENTER_X, CLAMP_CENTER_Y, clamp_top_z + margin,
                        CLAMP_SIZE_X + 2 * margin, CLAMP_SIZE_Y + 2 * margin,
                        CLAMP_HEIGHT + margin)


def build_board_object(margin=SAFETY_MARGIN):
    return _box_object('mount_board', BOARD_CENTER_X, BOARD_CENTER_Y, BOARD_TOP_Z + margin,
                        BOARD_SIZE_X + 2 * margin, BOARD_SIZE_Y + 2 * margin,
                        BOARD_THICKNESS + margin)


def main():
    rclpy.init()
    node = Node('set_desk_collision')
    cli = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('/apply_planning_scene 服务未上线（move_group 起来了吗？）', file=sys.stderr)
        sys.exit(1)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world = PlanningSceneWorld(
        collision_objects=[build_desk_object(), build_clamp_object(),
                            build_board_object()])

    req = ApplyPlanningScene.Request(scene=scene)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    print('apply_planning_scene success=%s' % (res.success if res else None))
    m = SAFETY_MARGIN
    print('安全余量 SAFETY_MARGIN=%.3f m（桌面、木工夹、木板四周与顶面外扩）' % m)
    print('desk: 实测 x=[%.3f,%.3f] y=[%.3f,%.3f] 顶面z=%.3f -> 碰撞盒 x=[%.3f,%.3f] y=[%.3f,%.3f] 顶面z=%.3f'
          % (DESK_X_MIN, DESK_X_MAX, DESK_Y_MIN, DESK_Y_MAX, DESK_TOP_Z,
             DESK_X_MIN - m, DESK_X_MAX + m, DESK_Y_MIN - m, DESK_Y_MAX + m, DESK_TOP_Z + m))
    print('desk_clamp: 中心=(%.3f,%.3f) 实测尺寸=(%.3f,%.3f,%.3f) 顶面z=%.3f -> 碰撞盒尺寸=(%.3f,%.3f,%.3f) 顶面z=%.3f'
          % (CLAMP_CENTER_X, CLAMP_CENTER_Y, CLAMP_SIZE_X, CLAMP_SIZE_Y,
             CLAMP_HEIGHT, DESK_TOP_Z + CLAMP_HEIGHT,
             CLAMP_SIZE_X + 2 * m, CLAMP_SIZE_Y + 2 * m, CLAMP_HEIGHT + m,
             DESK_TOP_Z + CLAMP_HEIGHT + m))
    print('mount_board: 中心=(%.3f,%.3f) 实测尺寸=(%.3f,%.3f,%.3f) 顶面z=%.3f -> 碰撞盒尺寸=(%.3f,%.3f,%.3f) 顶面z=%.3f'
          % (BOARD_CENTER_X, BOARD_CENTER_Y, BOARD_SIZE_X, BOARD_SIZE_Y,
             BOARD_THICKNESS, BOARD_TOP_Z,
             BOARD_SIZE_X + 2 * m, BOARD_SIZE_Y + 2 * m, BOARD_THICKNESS + m,
             BOARD_TOP_Z + m))

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (res and res.success) else 1)


if __name__ == '__main__':
    main()
