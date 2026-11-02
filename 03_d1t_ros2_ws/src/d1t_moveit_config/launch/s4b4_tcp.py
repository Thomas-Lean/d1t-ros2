#!/usr/bin/env python3
"""s4b4_tcp.py — S4b-4：以 tcp_link（两指之间的夹持中心）为末端，下位姿目标、走直线。

取代 s4b2_demo（已删除）：那个演示的关节目标 A 在外扩 30 mm 碰撞盒下判碰撞，笛卡尔直线段
没有速度缩放；关节目标与绕行由 s4b3_run10 覆盖，直线由本脚本覆盖。

坐标与单位（全部 base_link 系）：位置 m；朝向二选一 —— --rpy R P Y（弧度，固定轴 X-Y-Z 顺序，
即 R = Rz(Y)·Ry(P)·Rx(R)）或 --quat QX QY QZ QW。
tcp_link 轴向：x = 接近方向（指尖朝向），y = 两指开合方向，z = 夹爪壳体厚的一侧。
「夹爪与桌面平行、水平接近、z 朝上」= --rpy 0 0 <接近方向偏航角>。

子命令：
  pose X Y Z (--rpy R P Y | --quat QX QY QZ QW)
      /compute_ik（避碰，外扩场景）求关节解 → 目标名义离障检查（≥45 mm，s4b3_clearance 同一算法）
      → /move_action 关节目标规划并执行 → 读反馈算 TCP 实际到达偏差。
  line DX DY DZ [--frame base|yaw]
      从当前 TCP 位姿走直线，朝向不变。frame=base：位移按 base_link 轴；frame=yaw：x 为当前 TCP
      接近方向在水平面的投影、z 竖直向上（「水平前伸」「竖直抬升」用它）。
      /compute_cartesian_path 覆盖率 < 0.999 → 不执行、退出码 1（不执行部分路径）。
  grasp X Y Z (--rpy ... | --quat ...) [--approach 0.09] [--lift 0.05]
      依次：pose 到预抓位（目标沿 yaw 接近方向后退 approach）→ line 水平前伸 approach → line 竖直抬升 lift。
      任一步失败即停。桌上不放东西时用（不开合夹爪）。

共用选项：--vel / --acc 速度、加速度缩放（默认 0.3，大于 0.3 拒绝）；--plan-only 只规划不执行；
--csv-dir 反馈 CSV 目录（默认 /tmp，文件 s4b4_fb_<时间>.csv，表头同 s4b3_clearance 输入）。
每次执行后对录到的反馈逐帧算最近距离（实测尺寸盒），< 15 mm 打 FAIL 并退出码 1。
这个判定在运动结束后才做，不是急停；运动中要碰，断电。

前置：move_group + arm_driver + trajectory_server（execution_mode:=point, max_points:=400）在跑，
碰撞体已加（set_desk_collision）。
用法示例：
  ros2 run d1t_moveit_config s4b4_tcp pose 0.55 0.0 0.053 --rpy 0 0 0
  ros2 run d1t_moveit_config s4b4_tcp line 0.09 0 0 --frame yaw
  ros2 run d1t_moveit_config s4b4_tcp grasp 0.55 0.0 0.053 --rpy 0 0 0 --approach 0.09 --lift 0.05
"""
import argparse
import datetime
import importlib.util
import math
import os
import sys
import time

from pathlib import Path

import numpy as np

JOINTS = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
JOINT_SIGNS = [1.0, 1.0, 1.0, -1.0, 1.0, -1.0]   # URDF -> 控制器角度，与 trajectory_server 一致
TCP_LINK = 'tcp_link'
GROUP = 'arm'
MAX_SCALING = 0.3            # 实机速度、加速度缩放上限（任务书 S4b-4 §1）
TARGET_CLEARANCE_MM = 45.0   # 目标点名义距离下限（S4b-3 第五轮裁定）
FRAME_FAIL_MM = 15.0         # 逐帧最近距离失败线（S4b-3 裁定）
MIN_FRACTION = 0.999
CART_MAX_STEP_M = 0.005
# 相对跳变阈值：某一步关节变化 > 5 × 平均步长就截断路径（覆盖率随之 < 1，按失败处理）
CART_JUMP_THRESHOLD = 5.0
CART_MAX_POINTS = 11         # 点模式每点停一次：100 mm 直线约 10 mm 一段
IK_TIMEOUT_SEC = 0.3
FK_CHECK_POS_M = 0.001
FK_CHECK_ANG_RAD = 0.01
GOAL_TIMEOUT_SEC = 300.0


# ---------- 纯函数（test/test_s4b4_tcp.py 覆盖） ----------

def quat_from_rpy(r, p, y):
    """固定轴 X-Y-Z（R = Rz(y)·Ry(p)·Rx(r)）→ 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-9:
        raise ValueError('四元数模长为 0')
    return tuple(v / n for v in q)


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_angle(q1, q2):
    """两个朝向之间的转角（rad，0~π）。"""
    d = abs(sum(a * b for a, b in zip(quat_normalize(q1), quat_normalize(q2))))
    return 2.0 * math.acos(min(1.0, d))


def heading_yaw(q):
    """TCP x 轴（接近方向）在水平面投影的偏航角（rad）。x 轴竖直时无定义，抛异常。"""
    ax = quat_to_matrix(q)[:, 0]
    if math.hypot(ax[0], ax[1]) < 1e-3:
        raise ValueError('接近方向接近竖直，水平投影无定义')
    return math.atan2(ax[1], ax[0])


def z_tilt_deg(q):
    """TCP z 轴与竖直向上的夹角（度）。0 = 夹爪与桌面平行且 z 朝上。"""
    return math.degrees(math.acos(max(-1.0, min(1.0, quat_to_matrix(q)[2, 2]))))


def line_offset(frame, d, q):
    """直线位移 d=(dx,dy,dz) 换到 base_link。frame=base 原样；frame=yaw 按当前接近方向的水平偏航旋转。"""
    d = np.asarray(d, dtype=float)
    if frame == 'base':
        return d
    if frame == 'yaw':
        yaw = heading_yaw(q)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
    raise ValueError('frame 只能是 base 或 yaw')


def downsample_indices(n, max_points):
    """等间隔取下标，含首尾，最多 max_points 个。"""
    if n <= max_points:
        return list(range(n))
    return sorted({round(i * (n - 1) / (max_points - 1)) for i in range(max_points)})


def time_scale_factor(vel, acc):
    """/compute_cartesian_path（Humble）不收缩放参数，返回的轨迹按关节限位 100% 计时。
    把时间拉长 1/k 倍：速度 ×k、加速度 ×k²，取 k = min(vel, sqrt(acc)) 同时满足两个缩放。"""
    return min(vel, math.sqrt(acc))


def point_line_distance(p, a, b):
    """点 p 到线段 ab 的距离（m）。"""
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    L2 = float(ab @ ab)
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, float((p - a) @ ab) / L2))
    return float(np.linalg.norm(p - (a + t * ab)))


def urdf_to_controller_deg(q_rad):
    return [math.degrees(q_rad[i]) * JOINT_SIGNS[i] for i in range(6)]


def controller_deg_to_urdf(deg6):
    return [math.radians(deg6[i]) * JOINT_SIGNS[i] for i in range(6)]


def check_scaling(vel, acc):
    for name, v in (('--vel', vel), ('--acc', acc)):
        if not (0.0 < v <= MAX_SCALING):
            raise ValueError('%s=%.3f 超出 (0, %.1f]' % (name, v, MAX_SCALING))


def parse_orientation(args):
    if (args.rpy is None) == (args.quat is None):
        raise ValueError('朝向必须二选一：--rpy R P Y（弧度）或 --quat QX QY QZ QW')
    if args.rpy is not None:
        return quat_from_rpy(*args.rpy)
    return quat_normalize(args.quat)


def build_parser():
    p = argparse.ArgumentParser(prog='s4b4_tcp', description='tcp_link 位姿目标 / 直线（S4b-4）')
    sub = p.add_subparsers(dest='cmd', required=True)

    def common(sp):
        sp.add_argument('--vel-scaling', '--vel', dest='vel', type=float, default=MAX_SCALING, help='速度缩放 (0,0.3]')
        sp.add_argument('--acc-scaling', '--acc', dest='acc', type=float, default=MAX_SCALING, help='加速度缩放 (0,0.3]')
        sp.add_argument('--plan-only', action='store_true', help='只规划、打印，不执行')
        sp.add_argument('--csv-dir', default='/tmp', help='反馈 CSV 目录')

    def orient(sp):
        sp.add_argument('--rpy', type=float, nargs=3, metavar=('R', 'P', 'Y'),
                        help='朝向 RPY，弧度，固定轴 X-Y-Z')
        sp.add_argument('--quat', type=float, nargs=4, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='朝向四元数')

    sp = sub.add_parser('teach', help='读取当前姿态并保存示教点（只读，不动臂）')
    sp.add_argument('label', choices=['grasp', 'release'])
    sp.add_argument('--teach-dir', default='forlearning/S4b-4-tcp-pose/logs/real/teach')
    sp.add_argument('--vel-scaling', '--vel', dest='vel', type=float, default=MAX_SCALING)
    sp.add_argument('--acc-scaling', '--acc', dest='acc', type=float, default=MAX_SCALING)

    sp = sub.add_parser('pickplace', help='按 grasp/release 示教点执行抓放')
    sp.add_argument('--approach', type=float, default=0.09, help='预抓/预放后退距离 m')
    sp.add_argument('--retreat', type=float, default=0.05, help='抓取后/释放后直线后退距离 m')
    sp.add_argument('--vel-scaling', '--vel', dest='vel', type=float, default=MAX_SCALING)
    sp.add_argument('--acc-scaling', '--acc', dest='acc', type=float, default=MAX_SCALING)
    sp.add_argument('--plan-only', action='store_true', help='只规划，不执行、不调用夹爪')
    sp.add_argument('--teach-dir', default='forlearning/S4b-4-tcp-pose/logs/real/teach')
    sp.add_argument('--csv-dir', default='/tmp', help='反馈 CSV 目录')

    sp = sub.add_parser('pose', help='TCP 位姿目标')
    sp.add_argument('x', type=float)
    sp.add_argument('y', type=float)
    sp.add_argument('z', type=float)
    orient(sp)
    common(sp)

    sp = sub.add_parser('line', help='TCP 直线')
    sp.add_argument('dx', type=float)
    sp.add_argument('dy', type=float)
    sp.add_argument('dz', type=float)
    sp.add_argument('--frame', choices=['base', 'yaw'], default='base')
    common(sp)

    sp = sub.add_parser('grasp', help='预抓位 → 水平前伸 → 竖直抬升（不开合夹爪）')
    sp.add_argument('x', type=float)
    sp.add_argument('y', type=float)
    sp.add_argument('z', type=float)
    orient(sp)
    sp.add_argument('--approach', type=float, default=0.09, help='水平前伸距离 m')
    sp.add_argument('--lift', type=float, default=0.05, help='竖直抬升距离 m')
    common(sp)
    return p


# ---------- ROS 部分 ----------

def _load_clearance_module():
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('d1t_moveit_config'), 'launch', 's4b3_clearance.py')
    spec = importlib.util.spec_from_file_location('s4b3_clearance', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main(argv=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.utilities import remove_ros_args
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
    from d1t_interfaces.srv import Grip
    from sensor_msgs.msg import JointState
    from d1t_interfaces.msg import JointFeedback

    argv = sys.argv if argv is None else argv
    args = build_parser().parse_args(remove_ros_args(argv)[1:])
    try:
        check_scaling(args.vel, args.acc)
        target_q = parse_orientation(args) if args.cmd in ('pose', 'grasp') else None
    except ValueError as e:
        print('参数错误：%s' % e, file=sys.stderr)
        return 2

    err_name = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
                if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}

    # 节点复用 s4b3_clearance.Clearance（构造时写死节点名），这里改名以免与距离工具重名
    rclpy.init(args=list(argv) + ['--ros-args', '-r', '__node:=s4b4_tcp'])
    cl_mod = _load_clearance_module()
    node = cl_mod.Clearance()   # 复用 s4b3_clearance：/compute_fk 客户端 + 网格撒点 + 实测尺寸盒
    state = {'fb': None, 'rec': None}

    def on_fb(msg):
        deg7 = [msg.angle0, msg.angle1, msg.angle2, msg.angle3, msg.angle4, msg.angle5, msg.angle6_raw]
        state['fb'] = deg7
        if state['rec'] is not None:
            state['rec'].append((time.monotonic(), deg7))

    node.create_subscription(JointFeedback, '/d1t/joint_feedback', on_fb, 10)
    ik_cli = node.create_client(GetPositionIK, '/compute_ik')
    cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    move_ac = ActionClient(node, MoveGroup, '/move_action')
    exec_ac = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    grip_cli = node.create_client(Grip, '/grip')

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(cli, req, timeout=15.0):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    def ready():
        deadline = time.monotonic() + 20.0
        while state['fb'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = state['fb'] is not None
        ok = ok and node.fk.wait_for_service(timeout_sec=20.0) and ik_cli.wait_for_service(timeout_sec=20.0)
        if args.cmd == 'teach':
            return ok and node.fk.wait_for_service(timeout_sec=20.0)
        ok = ok and cart_cli.wait_for_service(timeout_sec=20.0)
        ok = ok and move_ac.wait_for_server(timeout_sec=20.0) and exec_ac.wait_for_server(timeout_sec=20.0)
        if args.cmd == 'pickplace':
            ok = ok and grip_cli.wait_for_service(timeout_sec=20.0)
        return ok

    def fk_tcp(q_rad):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [TCP_LINK]
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=list(q_rad)))
        res = call(node.fk, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('/compute_fk 失败')
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def current_q():
        spin_for(0.3)
        return controller_deg_to_urdf(state['fb'][:6])

    def clearance_mm(q_rad):
        per = node.frame_min(urdf_to_controller_deg(q_rad) + [state['fb'][6]])
        return min((d, oid, link) for oid, (d, link) in per.items())

    def fmt_pose(p, q):
        return 'p=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)' % (tuple(p) + tuple(q))

    def ik(p, q, avoid):
        seeds = []
        az = -math.atan2(p[1], p[0])   # Joint1 轴是 -z：正角把 +x 转向 -y
        seeds.append([az, 1.0, 0.5, 0.0, -1.5, 0.0])
        seeds.append([az, 0.3, 0.9, 0.0, -1.2, 0.0])
        seeds.append(current_q())
        for seed in seeds:
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name = GROUP
            r.ik_link_name = TCP_LINK
            r.avoid_collisions = avoid
            r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=seed))
            ps = PoseStamped()
            ps.header.frame_id = 'base_link'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in p)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = q
            r.pose_stamped = ps
            r.timeout.nanosec = int(IK_TIMEOUT_SEC * 1e9)
            res = call(ik_cli, req)
            if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        return None

    def wait_result(handle, label):
        fut = handle.get_result_async()
        t0 = time.monotonic()
        last = t0
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - t0 > GOAL_TIMEOUT_SEC:
                print('%s: %ds 无结果，发取消（取消不是急停）' % (label, GOAL_TIMEOUT_SEC))
                cfut = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cfut, timeout_sec=10.0)
                return None
            if now - last >= 5.0:
                print('  ...%s 执行中，已 %.0fs' % (label, now - t0))
                last = now
        return fut.result().result

    def start_rec():
        state['rec'] = []

    def finish_rec(label):
        rows, state['rec'] = state['rec'] or [], None
        if not rows:
            print('S4B4_CLEARANCE label=%s frames=0 verdict=NO_DATA' % label)
            return False
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(args.csv_dir, 's4b4_fb_%s.csv' % stamp)
        t0 = rows[0][0]
        with open(path, 'w') as f:
            f.write('t_rel_s,' + ','.join('a%d' % i for i in range(7)) + '\n')
            for t, deg7 in rows:
                f.write('%.4f,' % (t - t0) + ','.join('%.3f' % v for v in deg7) + '\n')
        worst = None
        for t, deg7 in rows:
            for oid, (d, link) in node.frame_min(deg7).items():
                if worst is None or d < worst[0]:
                    worst = (d, oid, link, t - t0)
        verdict = 'FAIL' if worst[0] < FRAME_FAIL_MM else 'PASS'
        print('S4B4_CLEARANCE label=%s frames=%d min_mm=%.1f object=%s link=%s t=%.2fs verdict=%s'
              '(<%.0fmm判失败) csv=%s' % (label, len(rows), worst[0], worst[1], worst[2], worst[3],
                                        verdict, FRAME_FAIL_MM, path))
        return verdict == 'PASS'

    def report_arrival(label, p_cmd, q_cmd):
        spin_for(1.0)
        q_act = controller_deg_to_urdf(state['fb'][:6])
        p_act, o_act = fk_tcp(q_act)
        d = (p_act - np.asarray(p_cmd)) * 1000.0
        print('S4B4_ARRIVAL label=%s cmd=(%.4f, %.4f, %.4f) act=(%.4f, %.4f, %.4f) '
              'dx=%.1f dy=%.1f dz=%.1f dist_mm=%.1f ang_deg=%.2f z_tilt_deg=%.2f fb_deg=%s'
              % (label, p_cmd[0], p_cmd[1], p_cmd[2], p_act[0], p_act[1], p_act[2], d[0], d[1], d[2],
                 float(np.linalg.norm(d)), math.degrees(quat_angle(o_act, q_cmd)), z_tilt_deg(o_act),
                 ['%.2f' % v for v in state['fb'][:7]]))

    def teach_point(label, teach_dir):
        q = current_q()
        p, o = fk_tcp(q)
        rpy = matrix_to_rpy(quat_to_matrix(o))
        out = Path(teach_dir)
        out.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = out / ('%s_%s.yaml' % (label, stamp))
        def seq(vals):
            return '[' + ', '.join('%.12g' % float(v) for v in vals) + ']'
        path.write_text('label: %s\npos: %s\nrpy: %s\njoints: %s\nframe: base_link\n'
                        % (label, seq(p), seq(rpy), seq(q)))
        print('S4B4_TEACH label=%s pos=%s rpy=%s joints=%s file=%s' %
              (label, seq(p), seq(rpy), seq(q), path))
        return True

    def matrix_to_rpy(R):
        # 与 quat_from_rpy 相反：R=Rz(y) Ry(p) Rx(r)，返回 (r,p,y) 弧度。
        p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        if abs(abs(p) - math.pi / 2) < 1e-7:
            r, y = 0.0, math.atan2(-R[0, 1], R[1, 1])
        else:
            r, y = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
        return r, p, y

    def read_teach(label, teach_dir):
        paths = sorted(Path(teach_dir).glob(label + '_*.yaml'))
        if not paths:
            raise ValueError('找不到示教文件：%s/%s_*.yaml' % (teach_dir, label))
        vals = {}
        for line in paths[-1].read_text().splitlines():
            key, value = line.split(':', 1)
            if key in ('pos', 'rpy', 'joints'):
                vals[key] = [float(x.strip()) for x in value.strip()[1:-1].split(',')]
        if set(vals) != {'pos', 'rpy', 'joints'}:
            raise ValueError('示教文件字段不完整：%s' % paths[-1])
        return np.array(vals['pos']), quat_from_rpy(*vals['rpy']), vals['joints'], paths[-1]

    def do_pose(label, p, q):
        p = np.asarray(p, dtype=float)
        print('%s: 目标 %s z_tilt=%.2f° heading=%.1f°' % (
            label, fmt_pose(p, q), z_tilt_deg(q),
            math.degrees(heading_yaw(q)) if z_tilt_deg(q) < 89 else float('nan')))
        sol = ik(p, q, avoid=True)
        if sol is None:
            sol_nc = ik(p, q, avoid=False)
            reason = 'IN_COLLISION（不避碰时有解，外扩碰撞场景里判碰撞）' if sol_nc else 'UNREACHABLE（不避碰也无逆解）'
            print('S4B4_POSE label=%s result=REJECTED reason=%s；不发任何运动' % (label, reason))
            return False
        p_fk, q_fk = fk_tcp(sol)
        e_pos, e_ang = float(np.linalg.norm(p_fk - p)), quat_angle(q_fk, q)
        cmin = clearance_mm(sol)
        print('%s: 逆解 URDF(deg)=%s 控制器(deg)=%s' % (
            label, ['%.2f' % math.degrees(v) for v in sol], ['%.2f' % v for v in urdf_to_controller_deg(sol)]))
        print('%s: 逆解 FK 复核 pos_err=%.2f mm ang_err=%.3f°；目标名义最近距离 %.1f mm（%s / %s）'
              % (label, e_pos * 1000, math.degrees(e_ang), cmin[0], cmin[1], cmin[2]))
        if e_pos > FK_CHECK_POS_M or e_ang > FK_CHECK_ANG_RAD:
            print('S4B4_POSE label=%s result=REJECTED reason=IK_FK_MISMATCH' % label)
            return False
        if cmin[0] < TARGET_CLEARANCE_MM:
            print('S4B4_POSE label=%s result=REJECTED reason=TARGET_CLEARANCE %.1f<%.0f mm；不发任何运动'
                  % (label, cmin[0], TARGET_CLEARANCE_MM))
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = args.vel
        req.max_acceleration_scaling_factor = args.acc
        c = Constraints()
        for name, pos in zip(JOINTS, sol):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=pos, tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        req.goal_constraints = [c]
        goal.planning_options.plan_only = args.plan_only
        t0 = time.monotonic()
        send = move_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('S4B4_POSE label=%s result=REJECTED reason=GOAL_NOT_ACCEPTED' % label)
            return False
        if not args.plan_only:
            start_rec()
        res = wait_result(handle, label)
        ok_clear = True if args.plan_only else finish_rec(label)
        if res is None:
            print('S4B4_POSE label=%s result=TIMEOUT' % label)
            return False
        code = res.error_code.val
        n_pts = len(res.planned_trajectory.joint_trajectory.points)
        print('S4B4_POSE label=%s result=%s error_code=%d(%s) points=%d elapsed=%.2fs vel=%.2f acc=%.2f plan_only=%s'
              % (label, 'OK' if code == MoveItErrorCodes.SUCCESS else 'FAILED', code, err_name.get(code, '?'),
                 n_pts, time.monotonic() - t0, args.vel, args.acc, args.plan_only))
        if code != MoveItErrorCodes.SUCCESS:
            return False
        if not args.plan_only:
            report_arrival(label, p, q)
        return ok_clear

    def do_line(label, d, frame, start_q=None):
        q0 = current_q() if start_q is None else list(start_q)
        p0, o0 = fk_tcp(q0)
        off = line_offset(frame, d, o0)
        p1 = p0 + off
        print('%s: 起点 %s；frame=%s 位移(base)=(%.4f, %.4f, %.4f) 长度 %.1f mm'
              % (label, fmt_pose(p0, o0), frame, off[0], off[1], off[2], float(np.linalg.norm(off)) * 1000))
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.start_state = RobotState(is_diff=True)
        if start_q is not None:
            req.start_state.joint_state = JointState(name=JOINTS, position=q0)
            req.start_state.is_diff = False
        req.group_name = GROUP
        req.link_name = TCP_LINK
        target = Pose()
        target.position.x, target.position.y, target.position.z = (float(v) for v in p1)
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = o0
        req.waypoints = [target]
        req.max_step = CART_MAX_STEP_M
        req.jump_threshold = CART_JUMP_THRESHOLD
        req.avoid_collisions = True
        res = call(cart_cli, req)
        if res is None:
            print('S4B4_LINE label=%s result=REJECTED reason=NO_RESPONSE' % label)
            return False
        jt = res.solution.joint_trajectory
        print('S4B4_LINE_PLAN label=%s fraction=%.4f raw_points=%d error_code=%d'
              % (label, res.fraction, len(jt.points), res.error_code.val))
        if res.fraction < MIN_FRACTION or not jt.points:
            print('S4B4_LINE label=%s result=REJECTED reason=FRACTION %.4f<%.3f；不执行部分路径'
                  % (label, res.fraction, MIN_FRACTION))
            return False
        idx = [list(jt.joint_names).index(j) for j in JOINTS]
        end_q = [jt.points[-1].positions[k] for k in idx]
        cmin = clearance_mm(end_q)
        print('%s: 终点名义最近距离 %.1f mm（%s / %s）' % (label, cmin[0], cmin[1], cmin[2]))
        if cmin[0] < TARGET_CLEARANCE_MM:
            print('S4B4_LINE label=%s result=REJECTED reason=TARGET_CLEARANCE %.1f<%.0f mm；不发任何运动'
                  % (label, cmin[0], TARGET_CLEARANCE_MM))
            return False
        n_raw = len(jt.points)
        keep = downsample_indices(n_raw, CART_MAX_POINTS)
        k = time_scale_factor(args.vel, args.acc)
        pts = []
        for i in keep:
            pt = jt.points[i]
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) / k
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
            pt.velocities = [v * k for v in pt.velocities]
            pt.accelerations = [a * k * k for a in pt.accelerations]
            pts.append(pt)
        jt.points = pts
        # 直线度：保留点 + 相邻保留点关节线性插值中点的 FK 离理想线段的距离
        dev = 0.0
        prev = None
        for pt in pts:
            qk = [pt.positions[j] for j in idx]
            dev = max(dev, point_line_distance(fk_tcp(qk)[0], p0, p1))
            if prev is not None:
                mid = [(a + b) / 2 for a, b in zip(prev, qk)]
                dev = max(dev, point_line_distance(fk_tcp(mid)[0], p0, p1))
            prev = qk
        dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        print('%s: 抽稀 %d→%d 点，时间拉长 1/%.2f 倍，总时长 %.2fs；模型直线度（保留点与中点离理想线段）最大 %.2f mm'
              % (label, n_raw, len(pts), k, dur, dev * 1000))
        if args.plan_only:
            print('S4B4_LINE label=%s result=PLAN_ONLY fraction=%.4f' % (label, res.fraction))
            return True
        t0 = time.monotonic()
        start_rec()
        send = exec_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=res.solution))
        rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            state['rec'] = None
            print('S4B4_LINE label=%s result=REJECTED reason=GOAL_NOT_ACCEPTED' % label)
            return False
        eres = wait_result(handle, label)
        ok_clear = finish_rec(label)
        if eres is None:
            print('S4B4_LINE label=%s result=TIMEOUT' % label)
            return False
        code = eres.error_code.val
        print('S4B4_LINE label=%s result=%s error_code=%d(%s) fraction=%.4f points=%d elapsed=%.2fs'
              % (label, 'OK' if code == MoveItErrorCodes.SUCCESS else 'FAILED', code, err_name.get(code, '?'),
                 res.fraction, len(pts), time.monotonic() - t0))
        if code != MoveItErrorCodes.SUCCESS:
            return False
        report_arrival(label, p1, o0)
        return ok_clear


    def do_grip(label, target):
        req = Grip.Request(target_deg=float(target), speed_ratio=args.vel, timeout_sec=0.0)
        fut = grip_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            print('S4B4_GRIP label=%s target=%.0f result=FAILED reason=no_response' % (label, target))
            return None
        print('S4B4_GRIP label=%s target=%.0f success=%s state=%s reason=%s start_raw=%.2f settled_raw=%.2f err_deg=%.2f' %
              (label, target, res.success, res.state, res.reason, res.start_raw, res.settled_raw, res.err_deg))
        if res.reason in ('not_moved', 'no_recv', 'rejected_recv'):
            print('S4B4_GRIP_STOP label=%s reason=%s（停止并报告，不自行归因）' % (label, res.reason))
        return res

    def do_home(label='home'):
        # HOME 不是抓放目标点；单独以关节目标规划，避免套用抓取区的 45 mm 目标点门槛。
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = args.vel
        req.max_acceleration_scaling_factor = args.acc
        # Joint3（=控制器轴 a2）2026-09-23 从 90° 改成 89°：TEACH-01 任务把 a2 正限位
        # 放宽到 Thomas 实测行程时定的上界是 89（不是对称猜测值 90），90° 现在已经
        # 超出 URDF 新上界（d1t_description/urdf/d1t.urdf Joint3 upper≈89.2°），
        # 跟 d1t.srdf 的 "home" group_state、d1t_arm.arm_driver.HOME_DEG 同步改。
        home = [0.0, -math.pi / 2, math.radians(89.0), 0.0, 0.0, 0.0]
        c = Constraints()
        for name, pos in zip(JOINTS, home):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=pos,
                                                       tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        req.goal_constraints = [c]
        goal.planning_options.plan_only = args.plan_only
        send = move_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
        h = send.result()
        if h is None or not h.accepted:
            print('S4B4_HOME label=%s result=REJECTED' % label); return False
        res = wait_result(h, label)
        if res is None:
            print('S4B4_HOME label=%s result=TIMEOUT' % label); return False
        ok = res.error_code.val == MoveItErrorCodes.SUCCESS
        print('S4B4_HOME label=%s result=%s error_code=%d(%s) plan_only=%s' %
              (label, 'OK' if ok else 'FAILED', res.error_code.val, err_name.get(res.error_code.val, '?'), args.plan_only))
        return ok

    def do_pickplace():
        gp, go, gj, gf = read_teach('grasp', args.teach_dir)
        rp, ro, rj, rf = read_teach('release', args.teach_dir)
        print('S4B4_PICKPLACE_TEACH grasp=%s release=%s approach=%.3f retreat=%.3f vel=%.2f acc=%.2f plan_only=%s' %
              (gf, rf, args.approach, args.retreat, args.vel, args.acc, args.plan_only))
        gpre = gp - line_offset('yaw', (args.approach, 0, 0), go)
        rpre = rp - line_offset('yaw', (args.approach, 0, 0), ro)
        if args.plan_only:
            # plan-only 也规划全段；显式传入上一段的理论末端关节状态，避免“不执行”时
            # 后续直线错误地从当前反馈重复规划。
            gpre_q = ik(gpre, go, avoid=True)
            grasp_q = ik(gp, go, avoid=True)
            rpre_q = ik(rpre, ro, avoid=True)
            release_q = ik(rp, ro, avoid=True)
            named_q = [('grasp_pre', gpre_q), ('grasp', grasp_q),
                       ('release_pre', rpre_q), ('release', release_q)]
            missing = [name for name, q in named_q if q is None]
            if missing:
                print('S4B4_PICKPLACE result=REJECTED reason=无避碰逆解 points=%s' % ','.join(missing))
                return False
            checks = [
                ('pick_pre', gpre, go, None),
                ('pick_approach', gp, go, gpre_q),
                ('pick_retreat', gp - line_offset('yaw', (args.retreat, 0, 0), go), go, grasp_q),
                ('place_pre', rpre, ro, None),
                ('place_approach', rp, ro, rpre_q),
                ('place_retreat', rp - line_offset('yaw', (args.retreat, 0, 0), ro), ro, release_q),
            ]
            for label, point, orient, start_q in checks:
                if start_q is None:
                    if not do_pose(label, point, orient): return False
                else:
                    delta = point - fk_tcp(start_q)[0]
                    if not do_line(label, delta, 'base', start_q=start_q): return False
            if not do_home('home'):
                return False
            print('S4B4_PICKPLACE result=PLAN_ONLY grip_calls=0')
            return True

        if not do_pose('pick_pre', gpre, go): return False
        pre_open = do_grip('pre_open', 50)
        if pre_open is None or pre_open.reason != 'ok': return False
        if not do_line('pick_approach', (args.approach, 0, 0), 'yaw'): return False
        close_res = do_grip('close', 29)
        if close_res is None or close_res.reason != 'ok' or close_res.state != 'grasped':
            print('S4B4_PICKPLACE_ABORT reason=close_not_grasped state=%s reason=%s' %
                  (close_res.state if close_res else '', close_res.reason if close_res else 'no_response'))
            do_line('abort_retreat', (-args.retreat, 0, 0), 'yaw')
            return False
        if not do_line('pick_retreat', (-args.retreat, 0, 0), 'yaw'): return False
        if not do_pose('place_pre', rpre, ro): return False
        if not do_line('place_approach', (args.approach, 0, 0), 'yaw'): return False
        release_open = do_grip('release_open', 50)
        if release_open is None or release_open.reason != 'ok': return False
        if not do_line('place_retreat', (-args.retreat, 0, 0), 'yaw'): return False
        return do_home('home')

    rc = 1
    try:
        if not ready():
            print('依赖未就绪：/d1t/joint_feedback /compute_fk /compute_ik /compute_cartesian_path '
                  '/move_action /execute_trajectory', file=sys.stderr)
            return 1
        if args.cmd == 'teach':
            ok = teach_point(args.label, args.teach_dir)
        elif args.cmd == 'pickplace':
            ok = do_pickplace()
        elif args.cmd == 'pose':
            ok = do_pose('pose', (args.x, args.y, args.z), target_q)
        elif args.cmd == 'line':
            ok = do_line('line', (args.dx, args.dy, args.dz), args.frame)
        else:
            p = np.array([args.x, args.y, args.z])
            pre = p - line_offset('yaw', (args.approach, 0.0, 0.0), target_q)
            ok = (do_pose('grasp_pre', pre, target_q)
                  and do_line('grasp_approach', (args.approach, 0.0, 0.0), 'yaw')
                  and do_line('grasp_lift', (0.0, 0.0, args.lift), 'yaw'))
        print('S4B4_DONE cmd=%s ok=%s' % (args.cmd, ok))
        rc = 0 if ok else 1
    except ValueError as e:
        print('S4B4_DONE cmd=%s ok=False error=%s' % (args.cmd, e), file=sys.stderr)
        rc = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
