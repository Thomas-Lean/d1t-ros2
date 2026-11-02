#!/usr/bin/env python3
"""s4b3_clearance.py — S4b-3 绕障硬指标：逐帧算夹爪与碰撞体的模型最近距离，< 15 mm 判失败。

输入一个或多个关节角 CSV（表头 t_rel_s,a0..a6，控制器角度，单位度）：
  - trajectory_server stream 模式落盘的反馈 CSV（/tmp/sovl4_fb_*.csv，实际走过的姿态）；
  - s4b3_run10 落盘的规划轨迹 CSV（/tmp/s4b3_plan_*.csv，MoveIt 规划的姿态）。
每一帧：用 move_group 的 /compute_fk 求 Empty_Link_L、Empty_Link_R（两指）、Empty_Link6（夹爪本体）、
Empty_Link5（腕）的位姿，把各自碰撞网格表面的采样点变换到 base_link，算到三块碰撞体的最近距离，
负数 = 采样点在盒子里面。

碰撞体用实测尺寸（set_desk_collision 的常量，margin=0），不是规划用的外扩 30 mm 盒——
15 mm 门槛来自误差账（执行偏差 + 手量误差），要对着实物尺寸比。

精度：网格三角面按 ≤2 mm 间距撒点（只取顶点时，130 mm 长的三角边会把距离算大几厘米），
采样本身会把距离算大最多约 1.5 mm；反馈 10 Hz，两帧之间的姿态看不到。都是模型距离，不是实物测量。

用法（需要一个带同一份 URDF 的 move_group 在当前 ROS_DOMAIN_ID 上，仿真域 7 即可，不需要碰撞体）：
  ros2 run d1t_moveit_config s4b3_clearance /tmp/sovl4_fb_20260916_194211.csv ...
每个文件打一行 CLEARANCE（全体最近 + 每块碰撞体各自的最近），最后打一行 CLEARANCE_SUMMARY；
有任一文件 FAIL 退出码 1。
"""
import importlib.util
import math
import os
import struct
import sys

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState

FAIL_MM = 15.0
SAMPLE_M = 0.002
LINKS = ['Empty_Link_L', 'Empty_Link_R', 'Empty_Link6', 'Empty_Link5']
JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint_L', 'Joint_R']
JOINT_SIGNS = [1.0, 1.0, 1.0, -1.0, 1.0, -1.0]   # 控制器角度 -> URDF，与 trajectory_server 一致


def load_collision_boxes():
    """从 set_desk_collision.py 取实测尺寸（margin=0），返回 {id: (lo, hi)}，不留第二份常量。"""
    path = os.path.join(get_package_share_directory('d1t_moveit_config'),
                        'launch', 'set_desk_collision.py')
    spec = importlib.util.spec_from_file_location('set_desk_collision', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    boxes = {}
    for obj in (mod.build_desk_object(margin=0.0), mod.build_clamp_object(margin=0.0),
                mod.build_board_object(margin=0.0)):
        c = obj.primitive_poses[0].position
        half = np.array(obj.primitives[0].dimensions) / 2.0
        center = np.array([c.x, c.y, c.z])
        boxes[obj.id] = (center - half, center + half)
    return boxes


def box_distance(points, lo, hi):
    """点到轴对齐盒子的距离（米），盒内为负（到最近面的距离取负）。"""
    d = np.maximum(np.maximum(lo - points, points - hi), 0.0)
    outside = np.linalg.norm(d, axis=1)
    inside = np.all((points >= lo) & (points <= hi), axis=1)
    depth = np.min(np.minimum(points - lo, hi - points), axis=1)
    return np.where(inside, -depth, outside)


def surface_samples(stl_path, step=SAMPLE_M):
    """二进制 STL 三角面按 ≤step 间距做重心坐标网格采样，返回 (N,3) 点。"""
    raw = open(stl_path, 'rb').read()
    n = struct.unpack('<I', raw[80:84])[0]
    tri = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype(
        [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))['v'].astype(float)
    out = []
    for a, b, c in tri:
        edge = max(np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c))
        k = max(1, int(math.ceil(edge / step)))
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing='ij')
        m = (i + j) <= k
        u, v = i[m] / k, j[m] / k
        out.append(a + np.outer(u, b - a) + np.outer(v, c - a))
    return np.unique(np.round(np.vstack(out), 5), axis=0)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def gripper_position(a6_raw):
    """angle6_raw -> 每爪 prismatic 位置，与 joint_state_bridge 同一换算。"""
    gap = 0.0 if a6_raw == 9999.0 else min(max(0.85 * a6_raw + 19.8, 0.0), 62.0)
    return min(max(0.033 - gap / 2000.0, 0.0), 0.033)


def read_csv(path):
    rows = []
    for line in open(path):
        if not line[:1].isdigit():
            continue
        v = [float(x) for x in line.split(',')]
        rows.append((v[0], v[1:8]))
    return rows


class Clearance(Node):
    def __init__(self):
        super().__init__('s4b3_clearance')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        mesh_dir = os.path.join(get_package_share_directory('d1t_description'), 'meshes')
        self._samples = {k: surface_samples(os.path.join(mesh_dir, k + '.STL')) for k in LINKS}
        self._boxes = load_collision_boxes()

    def frame_min(self, deg7):
        """一帧里每块碰撞体的最近距离：{碰撞体: (mm, 连杆)}。"""
        g = gripper_position(deg7[6])
        js = JointState(name=JOINTS)
        js.position = [math.radians(deg7[i]) * JOINT_SIGNS[i] for i in range(6)] + [g, g]
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = LINKS
        req.robot_state = RobotState(joint_state=js)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError('/compute_fk 失败：%s' % (res.error_code.val if res else '无响应'))
        per = {oid: (float('inf'), None) for oid in self._boxes}
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p = ps.pose
            pts = self._samples[name] @ quat_to_matrix(p.orientation).T + np.array(
                [p.position.x, p.position.y, p.position.z])
            for oid, (lo, hi) in self._boxes.items():
                d = float(box_distance(pts, lo, hi).min()) * 1000.0
                if d < per[oid][0]:
                    per[oid] = (d, name)
        return per


def main():
    files = sys.argv[1:]
    if not files:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    rclpy.init()
    node = Clearance()
    if not node.fk.wait_for_service(timeout_sec=30.0):
        print('/compute_fk 未上线（需要 move_group）', file=sys.stderr)
        sys.exit(2)
    n_fail = 0
    worst = None
    for path in files:
        rows = read_csv(path)
        best = None
        obj_min = {}
        for t, deg7 in rows:
            for oid, (d, link) in node.frame_min(deg7).items():
                if oid not in obj_min or d < obj_min[oid][0]:
                    obj_min[oid] = (d, t)
                if best is None or d < best[0]:
                    best = (d, oid, link, t, deg7)
        if best is None:
            print('CLEARANCE file=%s frames=0 verdict=NO_DATA' % os.path.basename(path))
            continue
        verdict = 'FAIL' if best[0] < FAIL_MM else 'PASS'
        n_fail += verdict == 'FAIL'
        print('CLEARANCE file=%s frames=%d min_mm=%.1f object=%s link=%s t=%.2fs deg=%s '
              'verdict=%s(<%.0fmm判失败) per_object=%s'
              % (os.path.basename(path), len(rows), best[0], best[1], best[2], best[3],
                 ['%.1f' % v for v in best[4][:6]], verdict, FAIL_MM,
                 ','.join('%s:%.1fmm@%.2fs' % (k, v[0], v[1]) for k, v in sorted(obj_min.items()))))
        if worst is None or best[0] < worst[0]:
            worst = (best[0], os.path.basename(path))
    print('CLEARANCE_SUMMARY files=%d fail=%d worst_mm=%s worst_file=%s'
          % (len(files), n_fail, '%.1f' % worst[0] if worst else 'n/a',
             worst[1] if worst else 'n/a'))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == '__main__':
    main()
