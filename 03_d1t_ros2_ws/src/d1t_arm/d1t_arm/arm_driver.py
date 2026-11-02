# arm_driver.py — d1t_arm 常驻驱动：action 收七轴目标，驱动机械臂到位；主线常驻节点（MoveIt → trajectory_server → 本节点），ros2_control 路线 2026-09-11 已放弃，不会退役。
# 到位：angle0~5 的 |actual-target| 全部 <= tolerance_deg，连续 stable_frames 帧成立。
# 参数：recv_timeout_sec (1.0)    funcode=1 每次发送后等 recv 的窗口
#       max_sends (3)             funcode=1 每个目标最多发送轮数（含首发），每次新 seq
#       tolerance_deg (3.0)       angle0~5 到位容差——不是精度指标，是「大于机械臂自身
#                                 静差、小于会掩盖真失败的值」；静差与位姿强相关，实测
#                                 跨度 0.06°~1.3°（含 2026-09-14 最大臂展 a1=1.2° 导致
#                                 旧 1.0° 带下 arrival_timeout 的那次，S-aux4c 放宽）
#       stable_frames (3)         连续合格帧数
#       default_timeout_sec (20.0) goal 未指定时的到位超时，从第一次发送起算
#       funcode1_gap_ms (0.0)     funcode=1 一轮 7 条命令的帧间隔；Step 0 实测需要再调
#
# 唯一通路：funcode=1 单点规划通路（S4a-1b）。上位机用 trapezoid_planner 算出全臂统一
#   总时长 T 和统一 delay_ms，发 7 条 funcode=1 单关节命令（每轴一条、同一 delay_ms），
#   控制器按「实际到位时间 = COMP × delay_ms」（COMP 实机标定 1.27）执行 → 七轴同时到达。
#   funcode=1 报文无 mode 字段，只回 recv、无 exec 回执（§7 坑1）——到位只看
#   /d1t/joint_feedback；enable_status 被 funcode=1 置 1 但那只是软件标志（§7 坑3），
#   不作证据。speed_ratio / accel_ratio 随 goal 给——两个倍率都只影响总时长，都不传
#   加减速形状，形状由控制器收到 delay_ms 后自行重算（S-aux4c，2026-09-14 实机证实
#   accel_ratio 端到端有效，此前「无效」的标签只是没说清是形状不传，不是倍率不生效）。
#   S-aux4（2026-09-11）退役原 funcode=2 mode0/mode1 通路：Thomas 裁定统一走本通路，
#   sequence_runner 是其唯一用户，已先行迁移并在 test_s2_fake.sh 重验多段序列（见该
#   commit），确认站得住后才删——`goal.mode` / `goal.use_planner` 两个字段随之从
#   action 移除。
#
# service /set_override (SetOverride)：全局速度倍率，节点级状态，不随 goal 走，不受 goal 门控
#   （手动 jog 时随时可改）。对每个 goal 都生效（S-aux4 前是「仅 use_planner=true」，
#   现在只剩这一条路）。
#
# service /set_servo (SetServo)：上/下伺服。复用同一 /arm_Command 发布者，与 /move_to_joints
# goal 互斥（有 goal 在跑直接返回 reason="busy"）。回调在独立 ReentrantCallbackGroup +
# MultiThreadedExecutor 上跑，阻塞等待时不挡 action 线程。
#   enable=true ：先读一帧新鲜反馈，读数超实测硬件行程 = 传感器异常拒绝。再按 in_folded_region 分两支：
#     折叠区内（a1≈-90 & a2≈90，±FOLD_TOL）→ 主动规划回精确 HOME [0,-90,89,0,0,0,0]（位移小，
#       安全；到位判定复用 _tick/_tick_planned/_on_feedback，见 _run_planned_blocking）；
#     折叠区外 → 零位移 funcode=1 原地使能（只 6 根臂轴，不碰夹爪；下发目标 clamp 到控制器限位
#       CTRL_LIMIT，clamp 偏差 = 会突跳多少，> 2° 拒绝——任意远处直线插值回 HOME 有碰撞风险）。
#     两支都是等 enable_status 变 1 后打印可粘贴位置。
#   enable=false：只在折叠区就地发 funcode=5 mode=0 卸力，不主动运动。
#     远离折叠区 → reason="not_folded" 拒绝（卸力会砸 + 直线插值回位有碰撞风险，正解是 web
#     示教器；当前唯一可行是先 /move_to_joints 走过去）。
#
# service /set_stiffness (SetStiffness)：发一帧 funcode=5 全关节刚度命令，测刚度档位用。
#   mode<0 或 mode>1000 → out_of_range，不发命令（实测量程就是 0~1000，1000 以上饱和）；
#   mode<HOLD_MODE_MIN 视同卸力、走折叠区检查；mode>=HOLD_MODE_MIN 打「带力命令」告警
#   （满档也撑不住自重，不能用来保持位置）。与 goal 同样互斥。
#
# service /grip (Grip)：GRIP-01 夹爪动作 + 夹取判定。只发 jid=6 一条 funcode=1（不碰臂轴），等 recv、
#   等夹爪停稳，按 grip_detect 判 empty / grasped / uncertain / not_moved。起动自救与 action 同一套
#   参数：发令 startup_check_sec 后夹爪应动未动（行程 >= grip_move_expect_deg、离开起点 < startup_stall_deg）
#   → 单独重发一次；仍不动由 not_moved 报出。与 goal 同样互斥。

import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from unitree_arm.msg import ArmString
from d1t_interfaces.action import MoveToJoints
from d1t_interfaces.msg import ArmStatus, CommandAck, JointFeedback
from d1t_interfaces.srv import Grip, SetOverride, SetServo, SetStiffness

from d1t_arm.grip_detect import GripConfig, evaluate_grip_wait, grip_target_ok
# Retain the historical import for task-local adapters that imported it here.
from d1t_arm.grip_detect import evaluate_grip as evaluate_grip
from d1t_arm.trapezoid_planner import (
    A_PLAN, V_MAX, HW_LIMIT_MAX, HW_LIMIT_MIN, LIMIT_MAX as CTRL_LIMIT_MAX,
    LIMIT_MIN as CTRL_LIMIT_MIN, clamp_ratio, clamp_to_ctrl, hw_out_of_limits, plan)

# 三层限位原则（S-aux4，2026-09-11 统一 action 与 /set_servo 到同一套表/函数，见
# trapezoid_planner.CTRL_LIMIT/HW_LIMIT 与 clamp_to_ctrl/hw_out_of_limits）：
#   超出 HW_LIMIT      → 拒绝 out_of_limits（读数/目标本身不可达）
#   在 HW_LIMIT 内、    → 截到 CTRL_LIMIT 照常执行，result 里如实回报截断量
#     超出 CTRL_LIMIT     （控制器本来就会自己截，我们拒绝等于拒绝一件它乐意做的事）
#   在 CTRL_LIMIT 内   → 原样执行
# 取代了旧的单表 reject-only 判据（LIMIT_ABS ±{135,90,90,135,90,135}、A6_RANGE[0,65]——
# 后者两头都错，见 START_HERE「夹爪 a6 的范围三处不一致」，随本次改动一并解决）。

# MoveToJoints.action goal 里 speed_ratio / accel_ratio 的 IDL 默认值。
# 也用作 /set_servo 原地使能零位移规划的默认倍率（见 _servo_on_in_place）。
SPEED_RATIO_DEFAULT = 0.3
ACCEL_RATIO_DEFAULT = 1.0

# HOME 位（度）：接近自然掉电姿态。见 manifest/START_HERE.md。
# a2 原是 90.0，2026-09-23 Thomas 把 a2 正限位放宽到实测行程时上界定的是 89（不是 97 那组
# 对称猜测值），90.0 因此超出新控制器限位、也超出新 URDF 上界——原样留着会让折叠区内的
# /set_servo 上电（走这个 HOME_DEG 规划）直接报 MoveIt 目标越界。改成新限位的边界值 89.0，
# 语义仍是「折叠姿态」，只是贴着新的合法上界，不是重新设计的姿态。
HOME_DEG = (0.0, -90.0, 89.0, 0.0, 0.0, 0.0, 0.0)

# HW_LIMIT 判定统一用的宽松量：/set_servo 的「传感器异常」判定用它（a0~a5 读数超出
# 实测硬件行程 HW_MARGIN 度以上才算异常），action 的三层限位第一档也用它（2026-09-11
# 实机改用，原 margin=0 直接把 a1≈-90.4 自然停放位这个目标本身拒了——HW_LIMIT 表就是
# 「一次采样、±几度不确定度」，margin=0 等于假装那张表精确，不成立）。留 margin 还因为
# 官方软限位（如 a2=90）本就略超实测值（89.6）。
HW_MARGIN = 3.0

# 原地使能的偏差阈值（度）：控制器把下发目标截多远（= 机械臂会突跳多少）。
# 超过它原来拒绝上电，Thomas 2026-09-23 裁定改成只告警不拦（见 _servo_on_in_place），
# 现在它只决定日志与 reason 里报不报「这次上电会突跳」。
ENABLE_CLAMP_TOL = 2.0

# 折叠区（下伺服 / 低刚度判据）：a1、a2 是受重力的两轴，落在 HOME 邻域 ±FOLD_TOL 内 =
# 卸力就地不会大幅跌落。a0/a3/a5 滚转、a4/a6 卸力本就自然下垂，都不进判据。
# 折叠区外卸力会砸下来：/set_stiffness 的低刚度仍据此拒绝；/set_servo 下电不再拒绝，
# 改走「先阻尼再卸力」两段式（Thomas 2026-09-23 裁定，见 _servo_off）。
FOLD_TOL = 15.0

# 「不足以保持」的刚度上界：低于此 /set_stiffness 视同卸力，走折叠区检查。
# START_HERE：enable_status 真判据是 mode>=1000；低于此控制器不认为在使能。
# 「>=1000 是否真能保持」已实测：不能，满档也撑不住自重（阻尼只加阻力不驱动）——
# 仍打「带力命令」告警，不是放行，见 _on_set_stiffness。
HOLD_MODE_MIN = 1000

# 折叠区外下伺服时，阻尼档与真正卸力之间等多久（秒）。这段时间里臂带阻尼下坠。
# 2026-09-23 Thomas 裁定的两段式下电用；实机坠落耗时未标定，先取 2 s。
DAMPED_RELEASE_SEC = 2.0

# /set_stiffness 的合法量程上限（2026-09-11 Thomas 实测盖棺）：0~1000 之间阻尼手感
# 有梯度，1000 以上（含 65536、80000）全部饱和、与满档手感无异，官方文档 0~80000
# 在实机上不成立。数值上与 HOLD_MODE_MIN 相同（两者恰好是同一条线——控制器自己的
# 判据也是 mode<1000 时 enable_status=0），但概念不同：那是使能判据，这是硬件已知的
# 有效范围上限，若未来分开标定二者可能不再相等，不要合并成一个常量。
STIFFNESS_MODE_MAX = 1000


def stiffness_out_of_range(mode):
    """/set_stiffness 的量程判据：0~STIFFNESS_MODE_MAX 之外拒绝。"""
    return mode < 0 or mode > STIFFNESS_MODE_MAX


def planned_axis_displacement(stroke_deg, elapsed_sec, duration_sec, axis_min_sec,
                              speed_limit_deg_s):
    """按 plan() 的最短梯形/三角形曲线，算轴在 elapsed_sec 时应离开起点多少度。

    plan() 先算每轴最短曲线，再由最慢轴、duration_sec 或 override 把整条曲线等比拉长到
    duration_sec。这里做同一件时间缩放；因此 delay_ms 很长、起步本来就慢时，期望位移
    会自然变小，不会被固定的「0.5 s 必须动 1°」误判。
    """
    s = abs(float(stroke_deg))
    t_min = float(axis_min_sec)
    t_total = float(duration_sec)
    v_limit = float(speed_limit_deg_s)
    if s <= 0.0 or t_min <= 0.0 or t_total <= 0.0 or elapsed_sec <= 0.0:
        return 0.0

    # 把全臂统一总时长映射回该轴的最短曲线时间。
    t = min(t_min, float(elapsed_sec) * t_min / t_total)
    if t >= t_min:
        return s

    # 三角形峰值速度 = 2s/T；未撞 v_limit 时加速段占一半。撞速度上限时，
    # s = v_limit * (T - t_acc)，由此反解 t_acc。
    if 2.0 * s / t_min <= v_limit + 1e-9:
        t_acc = t_min * 0.5
    else:
        t_acc = t_min - s / v_limit
    t_acc = max(1e-12, min(t_min * 0.5, t_acc))
    accel = s / (t_acc * (t_min - t_acc))
    v_peak = accel * t_acc

    if t <= t_acc:
        return 0.5 * accel * t * t
    if t <= t_min - t_acc:
        return 0.5 * accel * t_acc * t_acc + v_peak * (t - t_acc)
    left = t_min - t
    return s - 0.5 * accel * left * left


def startup_stalled_axes(start_deg, current_deg, target_deg, per_axis_min_sec,
                          elapsed_sec, duration_sec, speed_limit_deg_s,
                          expected_min_deg, moved_max_deg):
    """返回「规划曲线此刻本应移动 ≥expected_min_deg、反馈离开起点却 <moved_max_deg」的轴。

    两个阈值分开（S4b-3 主窗口裁定 3b8db6d）：原先共用 1.0°，实机慢速回 HOME 时 a2 规划应走
    1.22°、实际已走 0.90°，被误判卡死重发。实机三次真卡死的位移都是 0，所以「没动」只认
    <moved_max_deg（默认 0.3°，高于反馈 0.1° 量化加噪声）。"""
    stalled = []
    for axis in range(7):
        expected = planned_axis_displacement(
            target_deg[axis] - start_deg[axis], elapsed_sec, duration_sec,
            per_axis_min_sec[axis], speed_limit_deg_s)
        observed = abs(current_deg[axis] - start_deg[axis])
        if expected >= expected_min_deg and observed < moved_max_deg:
            stalled.append(axis)
    return stalled


def servo_home_timeout(plan_dur_sec):
    """折叠区内规划回 HOME 的到位超时：规划时长 ×1.5 + 2 s，20 s 作下限。位移受
    FOLD_TOL 约束、通常很小，20 s 下限在绝大多数情况下就是实际生效值。"""
    return max(20.0, plan_dur_sec * 1.5 + 2.0)


class _ServoGoalStub:
    """/set_servo 触发的规划移动没有真 goal_handle。_tick 只查它的 is_cancel_requested、
    _on_feedback 只调它的 publish_feedback——给一个两者都满足的空壳，好让那两段代码原样复用。"""

    is_cancel_requested = False

    def publish_feedback(self, _fb):
        pass


_SERVO_STUB = _ServoGoalStub()


def build_stiffness_json(seq, mode):
    """funcode=5 全关节刚度帧。mode=0 即完全卸力。"""
    return json.dumps(
        {'seq': seq, 'address': 1, 'funcode': 5, 'data': {'mode': int(mode)}})


def build_disable_json(seq):
    """下使能帧 = 刚度 mode=0。"""
    return build_stiffness_json(seq, 0)


def hw_limit_violation(deg7):
    """a0~a5 读数超出实测硬件行程 HW_MARGIN 度以上的轴（/set_servo 传感器异常判据）。
    返回 [(轴号, 值), ...]，空 = 都正常。a6 夹爪不参与上伺服判定。"""
    return hw_out_of_limits(deg7, range(6), margin=HW_MARGIN)


def clamp_for_enable(deg7):
    """原地使能的「下发目标」：a0~a5 clamp 到控制器限位 CTRL_LIMIT（funcode=1 通路控制器
    无条件截断的就是这套）。返回 (下发用 7 元素列表, a0~a5 的 clamp 偏差列表，取绝对值)。

    偏差 = |控制器 clamp 后 - 当前读数| = 控制器会产生的那段突跳，交调用方按 2° 判：
      a1=-90.4 → 截到 -90.0，偏差 0.4° → 放行；a0=160 → 截到 135，偏差 25° → 拒绝。
    a6 不下发（只发 6 根臂轴），原样带过供打印。
    """
    out, devs = clamp_to_ctrl(deg7, range(6))
    return out, [abs(d) for d in devs]


def in_folded_region(deg7):
    """机械臂是否处于折叠区：a1 在 -90±FOLD_TOL、a2 在 90±FOLD_TOL 内。"""
    return (abs(float(deg7[1]) - HOME_DEG[1]) <= FOLD_TOL
            and abs(float(deg7[2]) - HOME_DEG[2]) <= FOLD_TOL)


class ArmDriver(Node):

    def __init__(self):
        super().__init__('d1t_arm')
        self.recv_timeout = self.declare_parameter('recv_timeout_sec', 1.0).value
        self.max_sends = self.declare_parameter('max_sends', 3).value
        self.tolerance = self.declare_parameter('tolerance_deg', 3.0).value  # 2026-10-27 Thomas：实验验证后 3° 定为默认（原 2.0）
        self.stable_frames = self.declare_parameter('stable_frames', 3).value
        self.default_timeout = self.declare_parameter('default_timeout_sec', 20.0).value
        # 反馈新鲜度阈值：规划前必须有一帧不早于此的 /d1t/joint_feedback，否则 not_ready。
        # 反馈周期实测 111ms（8.98Hz），默认 0.5s 留约 4 帧余量，正常抖动不会误触发。
        # 同时挡住反馈节点被停掉后驱动仍照常规划、把命令发进空气里的那类事故。
        self.feedback_stale_sec = self.declare_parameter('feedback_stale_sec', 0.5).value
        # S4b-3 起动自救默认开启：正常实机 bag 中命令后 0.135s 已移动 1.0~3.2°；取
        # 0.5s 留出约 4 帧反馈余量。是否「本该移动 1°」不一刀切，而按 plan() 的曲线逐轴
        # 计算；慢命令在 0.5s 时规划位移不足 startup_expected_deg 就不检查该轴。判「没动」
        # 用另一个阈值 startup_stall_deg（0.3°）：实机真卡死位移都是 0，0.3° 高于反馈 0.1°
        # 量化加噪声；与 1.0° 共用时慢速起步（应走 1.22°、实走 0.90°）被误判过。
        # 只重发一次，之后沿用原超时。
        self.startup_retry_enabled = bool(self.declare_parameter(
            'startup_retry_enabled', True).value)
        self.startup_check_sec = float(self.declare_parameter(
            'startup_check_sec', 0.5).value)
        self.startup_expected = float(self.declare_parameter(
            'startup_expected_deg', 1.0).value)
        self.startup_stall = float(self.declare_parameter(
            'startup_stall_deg', 0.3).value)
        if min(self.startup_check_sec, self.startup_expected, self.startup_stall) <= 0.0:
            raise ValueError('startup_check_sec、startup_expected_deg、startup_stall_deg 必须 > 0')
        # funcode=1 一轮 7 条命令的帧间隔。/arm_Command 读端是 BEST_EFFORT/KEEP_LAST(1)，
        # 无间隔连发会被读端覆盖掉中间帧（缺 recv 的轴走 _tick 重发兜底）。默认 2 ms 是
        # 任务书 §4 的建议起步值；真机需要的确切值由 Step 0 闸门（probe_cmd_rate）实测。
        self._f1_gap = self.declare_parameter('funcode1_gap_ms', 2.0).value / 1000.0
        # /grip 判定参数，默认值与依据见 grip_detect.DEFAULT_CONFIG
        self.grip_cfg = GripConfig(
            empty_max_err_deg=float(self.declare_parameter('grip_empty_max_err_deg', 1.0).value),
            grasped_min_err_deg=float(self.declare_parameter(
                'grip_grasped_min_err_deg', 2.0).value),
            move_expect_deg=float(self.declare_parameter('grip_move_expect_deg', 2.0).value),
            moved_min_deg=float(self.declare_parameter('grip_moved_min_deg', 0.5).value),
            settle_window_sec=float(self.declare_parameter('grip_settle_window_sec', 1.0).value),
            settle_band_deg=float(self.declare_parameter('grip_settle_band_deg', 0.25).value),
            settle_min_frames=int(self.declare_parameter('grip_settle_min_frames', 5).value))
        self.grip_timeout = float(self.declare_parameter('grip_timeout_sec', 10.0).value)
        # Opt-in until the feedback-only strategy is validated for the field setup.
        self.grip_early_confirm = self.declare_parameter('grip_early_confirm', False).value
        self.grip_open_confirm_sec = float(
            self.declare_parameter('grip_open_confirm_sec', 0.3).value)
        self.grip_feedback_max_gap_sec = float(
            self.declare_parameter('grip_feedback_max_gap_sec', 0.25).value)
        if (not math.isfinite(self.grip_open_confirm_sec) or self.grip_open_confirm_sec <= 0.
                or not math.isfinite(self.grip_feedback_max_gap_sec)
                or self.grip_feedback_max_gap_sec <= 0.):
            raise ValueError('grip confirmation intervals must be finite and positive')

        # /arm_Command 与真实 reader 一致：BEST_EFFORT / KEEP_LAST(1) / VOLATILE。
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # 派生话题与 d1t_feedback_node 的发布 QoS 对齐。
        jf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        ack_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        self._cmd_pub = self.create_publisher(ArmString, '/arm_Command', cmd_qos)
        self.create_subscription(
            JointFeedback, '/d1t/joint_feedback', self._on_feedback, jf_qos)
        self.create_subscription(
            CommandAck, '/d1t/command_ack', self._on_ack, ack_qos)
        self.create_subscription(
            ArmStatus, '/d1t/arm_status', self._on_status, jf_qos)

        self._seq = 0            # 进程内单调递增
        self._active = False     # 已有 goal 或 servo 操作在执行
        self._active_lock = threading.Lock()   # 守 _active 的读改写（goal 回调与 service 回调跨线程）
        # Services can build/reset a goal while executor callbacks are queued.
        # Keep one native timer alive; serialize the state it is allowed to advance.
        self._motion_lock = threading.RLock()
        self._motion_timer = None
        self._motion_timer_active = False
        self._next_motion_tick = 0.0
        self._enable_status = None  # 最近一帧 /d1t/arm_status.enable_status；判断使能只看这个
        self._last_fb_time = 0.0    # 最近一帧关节反馈到达时刻，供 service / 规划器起点判断新鲜度
        # 节点状态（机械臂现在在哪），只由 _on_feedback 更新，永不随 goal 重置——
        # 曾经错放在 _reset_goal_state() 里，每个 goal 结束就被清零成 [0]*7，导致
        # 下一个 goal 在反馈到达前（111ms）就已把「原点」当成规划起点，见 START_HERE
        # 「delay_ms=1 满速冲」根因记录。与 _last_fb_time 本该是同一类状态，现在对齐。
        self._last_deg = [0.0] * 7
        self._override = 1.0        # 全局速度倍率（/set_override 改），对每个 goal 都生效
        self._reset_goal_state()
        self._motion_timer = self.create_timer(0.1, self._tick)
        self._motion_timer.cancel()
        self._motion_timer_active = False

        self._server = ActionServer(
            self, MoveToJoints, '/move_to_joints',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _req: CancelResponse.ACCEPT)
        self._servo_srv = self.create_service(
            SetServo, '/set_servo', self._on_set_servo,
            callback_group=ReentrantCallbackGroup())
        self._override_srv = self.create_service(
            SetOverride, '/set_override', self._on_set_override,
            callback_group=ReentrantCallbackGroup())
        self._stiffness_srv = self.create_service(
            SetStiffness, '/set_stiffness', self._on_set_stiffness,
            callback_group=ReentrantCallbackGroup())
        self._grip_srv = self.create_service(
            Grip, '/grip', self._on_grip, callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            'd1t_arm ready: action /move_to_joints, services /set_servo /set_override '
            '/set_stiffness /grip, resident publisher /arm_Command')

    def _reset_goal_state(self):
        with self._motion_lock:
            self._cancel_motion_timer()
            self._reset_goal_state_locked()

    def _cancel_motion_timer(self):
        self._motion_timer_active = False
        self._next_motion_tick = 0.0
        if self._motion_timer is not None:
            self._motion_timer.cancel()
        self._timer = None

    def _reset_goal_state_locked(self):
        self._target = None
        self._phase = 'sending'
        self._timeout = self.default_timeout
        self._send_count = 0
        self._recv_deadline = 0.0
        self._t_first = 0.0
        self._stable = 0
        self._timer = None
        self._done = None
        self._goal_handle = None
        self._target_clamp_deg = [0.0] * 7   # result.target_clamp_deg（三层限位截断量）
        # --- funcode=1 规划通路 ---
        self._planned = False            # 本 goal 走 funcode=1 规划通路
        self._plan_dur = 0.0             # 规划出的总时长 T，回进 result.actual_duration_sec
        self._plan_limited = False       # 倍率被 clamp / delay_ms 撞上限，回进 result.speed_limited
        self._plan_delay_ms = 0          # 7 条 funcode=1 共用的 delay_ms（重发时复用）
        self._exec_path = 'none'         # result.executed_path
        self._planned_seqs = {}          # jid -> 当前该轴的 seq
        self._planned_recv = set()       # 已收到 recv=1 的 jid
        self._planned_recv_bad = False   # 任一 jid 收到 recv=0
        self._plan_start_deg = [0.0] * 7
        self._plan_per_axis_sec = [0.0] * 7
        self._plan_speed_limit = 0.0
        self._startup_checked = False
        self._startup_retried_axes = set()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- action ---
    def _on_goal(self, _request):
        with self._active_lock:
            if self._active:
                self.get_logger().warn('goal rejected: another goal/servo is running')
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        # _active 的清除挪到 _reset_goal_state 之后、且加锁：/set_servo 回调在别的线程
        # 也读改 _active，若先放行再复位，等到的 servo 可能撞进半复位的 goal 状态。
        try:
            result = await self._run(goal_handle)
            if result.reason == 'arrived':
                goal_handle.succeed()
            elif result.reason == 'canceled':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self._reset_goal_state()
            return result
        finally:
            with self._active_lock:
                self._active = False

    async def _run(self, goal_handle):
        req = goal_handle.request
        target = [float(v) for v in req.target_deg]
        if self._cmd_pub.get_subscription_count() == 0:
            return self._result(False, 'not_ready')

        # 三层限位（模块头注释）：超 HW_LIMIT 拒绝；HW_LIMIT 内、超 CTRL_LIMIT 则截断
        # 照走并回报截断量；CTRL_LIMIT 内原样执行。margin=HW_MARGIN，与 /set_servo 的
        # 传感器异常判定用同一个宽松量——HW_LIMIT 表本身就是「一次采样、±几度不确定度」
        # （trapezoid_planner.py 注释），margin=0 曾在实机上把「把机械臂送回它刚读到的
        # 自然停放位」这个目标本身直接拒了（2026-09-11 实测：a1=-90.4000015 差 HW_LIMIT_
        # MIN[1]=-90.4 仅约 1.5e-6°，纯浮点噪声；a2=89.7 差 89.6 约 0.1°，真实测量误差）
        # ——这正是三层限位要修的那个场景，margin=0 等于没修。
        hw_bad = hw_out_of_limits(target, range(7), margin=HW_MARGIN)
        if hw_bad:
            self.get_logger().warn(
                '目标超出实测硬件行程 %s：拒绝 out_of_limits'
                % ['a%d=%.1f' % (i, v) for i, v in hw_bad])
            return self._result(False, 'out_of_limits')
        # 查新鲜度、不只查「有没有过」：从未收到反馈，或最近一帧太旧（反馈节点被停/
        # 掉线），都不能拿 _last_deg 当规划起点——旧版本这里查的是 _last_fb_time==0.0，
        # 但从不重置 _last_fb_time 的是 _reset_goal_state()，一次没被清过，这道守卫
        # 因此从未真正生效过。见 START_HERE「delay_ms=1 满速冲」根因记录。
        if (self._last_fb_time == 0.0
                or (self._now() - self._last_fb_time) > self.feedback_stale_sec):
            return self._result(False, 'not_ready')

        clamped, clamp_devs = clamp_to_ctrl(target, range(7))
        self._target_clamp_deg = clamp_devs
        if any(d != 0.0 for d in clamp_devs):
            self.get_logger().info(
                '目标超出控制器软限位 CTRL_LIMIT，将截断照走：偏差(deg)=[%s]'
                % ', '.join('%.2f' % d for d in clamp_devs))

        self._target = clamped
        self._timeout = req.timeout_sec if req.timeout_sec > 0.0 else self.default_timeout
        self._goal_handle = goal_handle
        self._done = Future()
        return await self._run_planned(req, clamped)

    async def _run_planned(self, req, target):
        """funcode=1 规划通路。起点取最近一帧反馈；发 7 条单关节命令、同一 delay_ms。"""
        start = list(self._last_deg[:7])
        pr = plan(start, target, req.speed_ratio, req.accel_ratio, self._override,
                  duration_sec=req.duration_sec)
        # timeout_sec<=0（未指定）时用规划时长推一个更合适的默认值；否则 _run() 已把
        # self._timeout 设成 req.timeout_sec，原样沿用。
        timeout = (self._timeout if req.timeout_sec > 0.0
                   else max(self.default_timeout, pr.duration_sec * 1.5 + 5.0))
        # 起点和行程是这类「规划起点算错了」事故的关键量（S-aux4 delay_ms=1 满速冲
        # 根因：_last_deg 被清零，规划起点变成 [0]*7），必须打出来，事后才查得出来。
        stroke_max = max(abs(target[i] - start[i]) for i in range(7))
        log_message = (
            'PLAN funcode=1 start=%s target=%s stroke_max=%.2fdeg T=%.3fs delay_ms=%d '
            'override=%.2f speed_ratio=%.2f accel_ratio=%.2f limited=%s' % (
                ['%.2f' % v for v in start], ['%.2f' % v for v in target], stroke_max,
                pr.duration_sec, pr.delay_ms, self._override, req.speed_ratio,
                req.accel_ratio, pr.speed_limited))
        speed_ratio, _ = clamp_ratio(req.speed_ratio)
        self._setup_planned(
            target, pr, timeout, self._goal_handle, log_message,
            start=start, speed_limit=V_MAX * speed_ratio)
        return await self._done

    def _setup_planned(self, target, pr, timeout, goal_handle, log_message,
                       start=None, speed_limit=None):
        with self._motion_lock:
            self._cancel_motion_timer()
            self._setup_planned_locked(target, pr, timeout, goal_handle, log_message,
                                       start=start, speed_limit=speed_limit)

    def _setup_planned_locked(self, target, pr, timeout, goal_handle, log_message,
                              start=None, speed_limit=None):
        """规划通路建态，_run_planned（action）与 _run_planned_blocking（/set_servo）
        共用一份，不留两份——规划通路以后新增任何字段，只用改这一处（上一版两处各自
        初始化，已登记为缺陷：漏改一处就是这类 bug）。

        plan() 调用与 timeout 的计算策略两边不同，留给各自调用方；「等 _done 完成」
        的方式（await 还是轮询 + _SERVO_STUB）也保持各自的，那是仅存的真实差异。
        """
        self._target = list(target)
        self._timeout = timeout
        self._goal_handle = goal_handle
        self._done = Future()
        self._planned = True
        self._plan_dur = pr.duration_sec
        self._plan_limited = pr.speed_limited
        self._plan_delay_ms = pr.delay_ms
        self._exec_path = 'funcode1_planner'
        self._phase = 'waiting_recv'
        self._planned_seqs = {}
        self._planned_recv = set()
        self._planned_recv_bad = False
        self._plan_start_deg = list(self._last_deg[:7] if start is None else start)
        self._plan_per_axis_sec = list(pr.per_axis_sec)
        self._plan_speed_limit = float(V_MAX if speed_limit is None else speed_limit)
        self._startup_checked = not self.startup_retry_enabled
        self._startup_retried_axes = set()
        self._stable = 0
        self._send_count = 1
        self._t_first = self._now()
        self.get_logger().info(log_message)
        self._send_planned(self._target, pr.delay_ms)     # 首轮 7 条
        self._recv_deadline = self._now() + self.recv_timeout
        self._timer = self._motion_timer
        self._next_motion_tick = self._now() + 0.1
        self._motion_timer_active = True
        self._timer.reset()

    def _send_planned(self, target, delay_ms, ids=None):
        """funcode=1：每根轴一条单关节命令，7 条共用同一个 delay_ms。
        ids 给定时只发这些轴（_tick 里对缺 recv 的轴用新 seq 重发）。"""
        for jid in (range(7) if ids is None else ids):
            seq = self._seq
            self._seq += 1
            msg = ArmString()
            msg.data = json.dumps({
                'seq': seq, 'address': 1, 'funcode': 1,
                'data': {'id': jid, 'angle': float(target[jid]),
                         'delay_ms': int(delay_ms)}})
            self._cmd_pub.publish(msg)
            self._planned_seqs[jid] = seq
            self.get_logger().info(
                'SEND funcode=1 jid=%d seq=%d delay_ms=%d' % (jid, seq, delay_ms))
            if self._f1_gap > 0.0:
                time.sleep(self._f1_gap)

    # --- 发送与推进 ---
    def _tick(self):
        with self._motion_lock:
            now = self._now()
            # A timer take queued for the previous goal may arrive after reset.
            # It must not advance partially initialized state or run the new
            # goal before its first 100-ms tick. No old goal state is retained.
            if not self._motion_timer_active or now < self._next_motion_tick:
                return
            self._next_motion_tick = 0.0  # native timer keeps the normal cadence after this first gate
            self._tick_locked(now)

    def _tick_locked(self, now):
        if self._done is None or self._done.done():
            return
        if self._goal_handle.is_cancel_requested:
            self._finish(False, 'canceled')
            return
        if now - self._t_first >= self._timeout:
            self._finish(False, 'arrival_timeout')
            return
        self._tick_planned(now)

    def _tick_planned(self, now):
        # funcode=1：等 7 条都回 recv -> moving（到位由 _on_feedback 判）；
        # 任一 recv=0 -> rejected_recv；recv_deadline 内没凑齐 -> 对缺的轴用新 seq 重发。
        if self._planned_recv_bad:
            self._finish(False, 'rejected_recv')
            return
        if self._phase == 'moving':
            if not self._startup_checked and now - self._t_first >= self.startup_check_sec:
                self._startup_checked = True
                stalled = startup_stalled_axes(
                    self._plan_start_deg, self._last_deg, self._target,
                    self._plan_per_axis_sec, now - self._t_first, self._plan_dur,
                    self._plan_speed_limit, self.startup_expected, self.startup_stall)
                stalled = [a for a in stalled if a not in self._startup_retried_axes]
                if stalled:
                    self._send_planned(self._target, self._plan_delay_ms, ids=stalled)
                    self._startup_retried_axes.update(stalled)
                    for axis in stalled:
                        self.get_logger().warn(
                            'STALL_AXIS axis=%d start=%.2f feedback=%.2f retry_seq=%d'
                            % (axis, self._plan_start_deg[axis], self._last_deg[axis],
                               self._planned_seqs[axis]))
                    # 单轴重发也要等新 seq 的 recv；不占缺回执 max_sends 轮数。收不到时仍由
                    # 原 arrival timeout 终止，不再重发第二次。
                    for axis in stalled:
                        self._planned_recv.discard(axis)
                    self._recv_deadline = now + self.recv_timeout
                    self._phase = 'waiting_retry_recv'
            return
        required = (set(self._startup_retried_axes)
                    if self._phase == 'waiting_retry_recv' else set(range(7)))
        if required.issubset(self._planned_recv):
            self._phase = 'moving'
            return
        if now >= self._recv_deadline:
            if self._send_count >= self.max_sends:
                self._finish(False, 'no_recv')
                return
            missing = [j for j in range(7) if j not in self._planned_recv]
            self.get_logger().warn(
                'funcode=1 第 %d 轮缺 recv 轴 %s，重发' % (self._send_count, missing))
            self._send_planned(self._target, self._plan_delay_ms, ids=missing)
            self._send_count += 1
            self._recv_deadline = now + self.recv_timeout

    def _on_ack(self, msg):
        with self._motion_lock:
            self._on_ack_locked(msg)

    def _on_ack_locked(self, msg):
        kind = 'recv' if msg.ack_type == CommandAck.RECV else 'exec'
        self.get_logger().info(
            'ACK %s seq=%d status=%d' % (kind, msg.seq, msg.status))
        if msg.ack_type == CommandAck.RECV and self._planned:
            for jid, s in list(self._planned_seqs.items()):
                if msg.seq == s:
                    if msg.status == 1:
                        self._planned_recv.add(jid)
                    else:
                        self._planned_recv_bad = True

    def _on_status(self, msg):
        self._enable_status = msg.enable_status

    # --- service /set_override ---
    def _on_set_override(self, request, response):
        # 节点级全局倍率，不受 _active 门控：手动 jog 时要能随时改（§8）。
        # 已在跑的 funcode=1 goal 是单发，不重规划——改动对下一个 goal 生效。
        ratio, limited = clamp_ratio(request.ratio)
        self._override = ratio
        self.get_logger().info(
            'set_override: ratio=%.3f -> applied=%.3f%s'
            % (request.ratio, ratio, ' (clamped)' if limited else ''))
        response.success = True
        response.applied_ratio = ratio
        return response

    # --- service /set_servo ---
    def _on_set_servo(self, request, response):
        with self._active_lock:
            if self._active:
                return self._servo_response(response, False, 'busy')
            self._active = True
        try:
            if request.enable:
                return self._servo_on(response)
            return self._servo_off(response)
        finally:
            # 复位 goal 状态字段：servo 借用了 _target / _planned* / _send_count / _done 等，
            # 不复位会让紧随其后的 goal 撞进半初始化的规划状态。
            self._reset_goal_state()
            with self._active_lock:
                self._active = False

    def _servo_on(self, response):
        """上伺服分两支：折叠区附近 -> 主动规划回精确 HOME（位移小，已经在折叠邻域内，
        安全）；折叠区外 -> 零位移原地使能（任意远处直线插值回 HOME 有碰撞风险，见
        clamp_for_enable）。

        Thomas 2026-09-23 裁定：任意姿态都能上电。唯一还会拒的是读数超实测硬件行程——
        那不是「臂在某个位置」，那是读数不可信，此时下发的目标同样不可信，而且这种异常
        肉眼看不出来，只能靠这道检查。
        """
        if self._cmd_pub.get_subscription_count() == 0:
            return self._servo_response(response, False, 'not_ready')
        if not self._wait_fresh_feedback(budget=0.5):
            return self._servo_response(response, False, 'not_ready')  # 取不到新鲜反馈

        cur = list(self._last_deg[:7])
        hw_bad = hw_limit_violation(cur)
        if hw_bad:
            self.get_logger().warn(
                '[set_servo] 读数超实测硬件行程 %s：疑似传感器异常，拒绝上伺服'
                % ['a%d=%.1f' % (i, v) for i, v in hw_bad])
            return self._servo_response(response, False, 'out_of_limits')

        if in_folded_region(cur):
            return self._servo_on_home(response)
        return self._servo_on_in_place(response, cur)

    def _servo_on_home(self, response):
        """折叠区内：规划回精确 HOME（复用 _tick/_tick_planned/_on_feedback 的到位判定，
        不写第二份），到位后读 enable_status。位移小（受 FOLD_TOL 约束），风险低。"""
        arrived, reason = self._run_planned_blocking(HOME_DEG, 0.1)
        if not arrived:
            hint = 'timeout' if reason == 'arrival_timeout' else 'move_failed'
            return self._servo_response(response, False, hint)
        enabled = self._enable_status == 1
        if enabled:
            self._log_pose_for_paste(list(self._last_deg[:7]))
        return self._servo_response(response, enabled, 'ok' if enabled else 'move_failed')

    def _servo_on_in_place(self, response, cur):
        """折叠区外：零位移 funcode=1，只发 6 根臂轴（不碰夹爪）。使能是 funcode=1 的副作用
        （START_HERE：无独立上使能命令）。"""
        clamped, devs = clamp_for_enable(cur)
        max_dev = max(devs)
        self.get_logger().info(
            '[set_servo] 原地使能：控制器 clamp 偏差/轴(deg) = [%s]，max=%.2f'
            % (', '.join('%.2f' % d for d in devs), max_dev))
        # Thomas 2026-09-23 裁定：任意姿态都要能上电。原来这里 max_dev > 2° 就拒绝，
        # 现在只告警不拦——突跳幅度算出来如实报出去，由人决定按不按。
        jumpy = max_dev > ENABLE_CLAMP_TOL
        if jumpy:
            self.get_logger().warn(
                '[set_servo] 控制器会把下发目标截 %.2f°（funcode=1 无条件截到官方软限位），'
                '上电瞬间机械臂会突跳这么多：照发，不拦'
                % max_dev)

        if self._enable_status == 1:
            self.get_logger().info('[set_servo] enable_status 已为 1，不重复发命令')
            self._log_pose_for_paste(clamped)
            return self._servo_response(response, True, 'already')

        # 零位移：只发 6 根臂轴（不碰夹爪 a6）。plan() 只用来取一个合法 delay_ms（零行程→1）。
        pr = plan(cur, clamped, SPEED_RATIO_DEFAULT, ACCEL_RATIO_DEFAULT, self._override)
        self._planned = True
        self._exec_path = 'funcode1_planner'
        self._send_planned(clamped, pr.delay_ms, ids=range(6))
        deadline = self._now() + 3.0
        while self._now() < deadline:
            if self._enable_status == 1:
                self._log_pose_for_paste(clamped)
                # reason 分开报，界面要看得出这次上电有没有带突跳
                return self._servo_response(
                    response, True, 'ok_clamped_%.1fdeg' % max_dev if jumpy else 'ok')
            time.sleep(0.05)
        return self._servo_response(response, False, 'move_failed')

    def _servo_off(self, response):
        """下伺服：折叠区内直接卸力；折叠区外先加阻尼再卸力，不自动运动回 HOME。

        Thomas 2026-09-23 裁定，推翻「折叠区外拒绝下伺服」：任意姿态都要能下电。
        折叠区外走两段——先发 funcode=5 mode=HOLD_MODE_MIN（阻尼档，只加阻力不驱动，
        撑不住自重，臂从这一刻起就开始下坠，但是**带阻尼的下坠**），等
        DAMPED_RELEASE_SEC 让它坠完，再发 mode=0 真正卸力。
        臂会跌落，这是已知且被接受的代价（下方必须清空，手放在断电处）。
        """
        if self._cmd_pub.get_subscription_count() == 0:
            return self._servo_response(response, False, 'not_ready')
        if not self._wait_fresh_feedback():
            return self._servo_response(response, False, 'not_ready')
        cur = list(self._last_deg[:7])
        if not in_folded_region(cur):
            self.get_logger().warn(
                '[set_servo] 机械臂不在折叠区（a1=%.1f a2=%.1f）：先发阻尼档 mode=%d 让它'
                '带阻尼下坠 %.1f s，再卸力。臂会跌落，确认下方已清空'
                % (cur[1], cur[2], HOLD_MODE_MIN, DAMPED_RELEASE_SEC))
            self._send_stiffness(HOLD_MODE_MIN)
            time.sleep(DAMPED_RELEASE_SEC)
        was_disabled = self._enable_status == 0
        self._send_disable()
        time.sleep(0.5)
        if self._enable_status == 0:
            return self._servo_response(
                response, True, 'already' if was_disabled else 'ok')
        return self._servo_response(response, False, 'move_failed')

    def _wait_fresh_feedback(self, budget=2.0):
        """等一帧比进入时刻更新的关节反馈，供 servo 读当前位姿。"""
        t0 = self._now()
        while self._now() - t0 < budget:
            if self._last_fb_time > t0:
                return True
            time.sleep(0.05)
        return False

    def _run_planned_blocking(self, target, speed_ratio):
        """在 service 回调线程里跑一次 funcode=1 规划通路回 target。建态与 _run_planned
        共用 _setup_planned（不写第二份），推进与到位判定复用 _tick / _tick_planned /
        _on_feedback——只把 action 的 `await self._done` 换成轮询，并塞一个 _SERVO_STUB
        当 goal_handle。返回 (arrived, reason)；reason 直接是 _result 的 reason
        （arrived / arrival_timeout / ...）。"""
        start = list(self._last_deg[:7])
        pr = plan(start, list(target), speed_ratio, ACCEL_RATIO_DEFAULT, self._override)
        timeout = servo_home_timeout(pr.duration_sec)
        log_message = (
            '[set_servo] 规划回 HOME：T=%.2fs delay_ms=%d timeout=%.1fs speed_ratio=%.2f'
            % (pr.duration_sec, pr.delay_ms, timeout, speed_ratio))
        self._setup_planned(
            target, pr, timeout, _SERVO_STUB, log_message,
            start=start, speed_limit=V_MAX * clamp_ratio(speed_ratio)[0])
        while not self._done.done():
            time.sleep(0.05)
        res = self._done.result()
        return res.arrived, res.reason

    # --- service /set_stiffness ---
    def _on_set_stiffness(self, request, response):
        """funcode=5 刚度档位测试用。
        mode < 0 或 mode > STIFFNESS_MODE_MAX → reason='out_of_range'，不发命令
                              （2026-09-11 实测盖棺：有效量程就是 0~1000，1000 以上
                              全部饱和，官方文档 0~80000 在实机上不成立；mode 最终塞进
                              uint16，超界还会静默回绕、造出「enable_status=1 但舵机
                              全松」的假使能状态）。
        mode < HOLD_MODE_MIN → 视同卸力，走折叠区检查（在展开姿态卸力会砸下来）；不在折叠区
                              → not_folded 拒绝。
        mode >= HOLD_MODE_MIN → 「带力命令」，打 WARN——但阻尼只加阻力、不朝任何目标驱动，
                              满档也撑不住自重，不能用来保持位置（2026-09-11 实测）。
        """
        with self._active_lock:
            if self._active:
                return self._stiffness_response(response, False, 0, 'busy')
            self._active = True
        try:
            mode = int(request.mode)
            if stiffness_out_of_range(mode):
                self.get_logger().warn(
                    '[set_stiffness] mode=%d 超出实测量程 0~%d（%d 以上已饱和，无实际意义）：'
                    '拒绝，不发命令' % (mode, STIFFNESS_MODE_MAX, STIFFNESS_MODE_MAX))
                return self._stiffness_response(response, False, mode, 'out_of_range')
            if self._cmd_pub.get_subscription_count() == 0:
                return self._stiffness_response(response, False, mode, 'not_ready')
            if mode < HOLD_MODE_MIN:
                if not self._wait_fresh_feedback():
                    return self._stiffness_response(response, False, mode, 'not_ready')
                cur = list(self._last_deg[:7])
                if not in_folded_region(cur):
                    self.get_logger().warn(
                        '[set_stiffness] mode=%d < %d 不足以保持，机械臂不在折叠区'
                        '（a1=%.1f a2=%.1f）：拒绝，卸力会砸下来。先用 /move_to_joints 走到折叠姿态'
                        % (mode, HOLD_MODE_MIN, cur[1], cur[2]))
                    return self._stiffness_response(response, False, mode, 'not_folded')
            else:
                self.get_logger().warn(
                    '[set_stiffness] mode=%d >= %d：带力命令，仅加阻力不驱动，满档也撑不住'
                    '自重、不能用来保持位置——确认起姿 / 从小值起步 / 手在断电处'
                    % (mode, HOLD_MODE_MIN))
            self._send_stiffness(mode)
            time.sleep(0.5)
            return self._stiffness_response(response, True, mode, 'ok')
        finally:
            self._reset_goal_state()
            with self._active_lock:
                self._active = False

    # --- service /grip ---
    def _on_grip(self, request, response):
        response.start_raw = response.settled_raw = response.err_deg = math.nan
        with self._active_lock:
            if self._active:
                return self._grip_response(response, 'busy')
            self._active = True
        try:
            return self._grip(request, response)
        finally:
            self._reset_goal_state()
            with self._active_lock:
                self._active = False

    def _grip(self, request, response):
        target = float(request.target_deg)
        if not grip_target_ok(target):
            self.get_logger().warn('[grip] target=%.2f 超出 [20, 50]：拒绝，不发命令' % target)
            return self._grip_response(response, 'out_of_range')
        if self._cmd_pub.get_subscription_count() == 0:
            return self._grip_response(response, 'not_ready')
        if not self._wait_fresh_feedback(budget=0.5):
            return self._grip_response(response, 'not_ready')

        start7 = list(self._last_deg[:7])
        start = start7[6]
        response.start_raw = float(start)
        tgt7 = list(start7)
        tgt7[6] = target          # 只有夹爪有行程；只发 jid=6，臂轴读数不下发
        pr = plan(start7, tgt7, request.speed_ratio, ACCEL_RATIO_DEFAULT, self._override)
        timeout = request.timeout_sec if request.timeout_sec > 0.0 else self.grip_timeout
        cfg = self.grip_cfg

        # ACK callbacks use this same lock. No ACK can cross the reset/send
        # boundary and mark a new sequence received on behalf of an old one.
        with self._motion_lock:
            # 借用 funcode=1 的 recv 记账（_on_ack 只在 _planned 时认 seq）
            self._planned = True
            self._exec_path = 'funcode1_planner'
            self._planned_seqs = {}
            self._planned_recv = set()
            self._planned_recv_bad = False
            self.get_logger().info(
                '[grip] start=%.2f target=%.2f T=%.3fs delay_ms=%d timeout=%.1fs'
                % (start, target, pr.duration_sec, pr.delay_ms, timeout))
            t0 = self._now()
            self._send_planned(tgt7, pr.delay_ms, ids=[6])
            sends = 1
            recv_deadline = t0 + self.recv_timeout
            retried = False
            samples = []
            last_fb = self._last_fb_time
        while True:
            time.sleep(0.02)
            with self._motion_lock:
                now = self._now()
                feedback_time, feedback_value = self._last_fb_time, self._last_deg[6]
                if feedback_time > last_fb:
                    last_fb = feedback_time
                    samples.append((feedback_time, feedback_value))
                if self._planned_recv_bad:
                    return self._grip_response(response, 'rejected_recv')
                if 6 not in self._planned_recv:
                    # Absolute service budget starts at the first send and is never
                    # renewed by either transport retransmits or startup rescue.
                    if now >= t0 + timeout:
                        return self._grip_response(response, 'no_recv')
                    if now >= recv_deadline:
                        if sends >= self.max_sends:
                            return self._grip_response(response, 'no_recv')
                        self.get_logger().warn('[grip] 第 %d 轮缺 recv，重发' % sends)
                        self._send_planned(tgt7, pr.delay_ms, ids=[6])
                        sends += 1
                        recv_deadline = self._now() + self.recv_timeout
                    continue
                if (self.startup_retry_enabled and not retried and now < t0 + timeout
                        and now - t0 >= self.startup_check_sec
                        and abs(target - start) >= cfg.move_expect_deg
                        and abs(self._last_deg[6] - start) < self.startup_stall):
                    retried = True
                    # One startup rescue is independent of max_sends (which counts
                    # the initial send plus missing-RECV transport sends). Both are
                    # bounded by the same absolute service deadline.
                    self._planned_recv.discard(6)
                    if self.grip_early_confirm:
                        samples.clear()
                    self._send_planned(tgt7, pr.delay_ms, ids=[6])
                    recv_deadline = self._now() + self.recv_timeout
                    self.get_logger().warn(
                        'STALL_AXIS axis=6 start=%.2f feedback=%.2f retry_seq=%d（/grip）'
                        % (start, self._last_deg[6], self._planned_seqs[6]))
                    continue  # await the new sequence's RECV in either wait mode
                out = evaluate_grip_wait(
                    samples, target, start, t0, t0 + pr.duration_sec, now, t0 + timeout, cfg,
                    early_confirm=self.grip_early_confirm,
                    open_confirm_sec=self.grip_open_confirm_sec,
                    feedback_max_gap_sec=self.grip_feedback_max_gap_sec)
                if out is None:
                    continue
                reason, state, settled = out
                response.state = state
                response.settled_raw = float(settled)
                response.err_deg = float(settled - target)
                self.get_logger().info(
                    '[grip] reason=%s state=%s start=%.2f target=%.2f settled=%.2f err=%+.2f '
                    'after=%.2fs retried=%s' % (reason, state, start, target, settled,
                                                settled - target, now - t0, retried))
                return self._grip_response(response, reason)

    def _grip_response(self, response, reason):
        response.reason = reason
        response.success = reason == 'ok' and response.state in ('empty', 'grasped')
        if reason != 'ok':
            response.state = ''
        return response

    def _stiffness_response(self, response, success, mode, reason):
        response.success = bool(success)
        response.applied_mode = int(mode)
        response.enabled = self._enable_status == 1
        response.reason = reason
        return response

    def _log_pose_for_paste(self, deg7):
        """上伺服成功后打印当前位置，格式能直接粘进 send_goal 的 goal。"""
        body = ', '.join('%.2f' % v for v in deg7[:7])
        self.get_logger().info(
            '[set_servo] 已使能。当前位置：\n'
            '  target_deg: [%s]\n'
            '  （可直接粘进 ros2 action send_goal /move_to_joints 的 goal）' % body)

    def _send_disable(self):
        seq = self._seq
        self._seq += 1
        msg = ArmString()
        msg.data = build_disable_json(seq)
        self._cmd_pub.publish(msg)
        self.get_logger().info(
            'SEND disable seq=%d (address=1 funcode=5 mode=0)' % seq)

    def _send_stiffness(self, mode):
        seq = self._seq
        self._seq += 1
        msg = ArmString()
        msg.data = build_stiffness_json(seq, mode)
        self._cmd_pub.publish(msg)
        self.get_logger().info(
            'SEND stiffness seq=%d (address=1 funcode=5 mode=%d)' % (seq, int(mode)))

    def _servo_response(self, response, success, reason):
        response.success = bool(success)
        response.enabled = self._enable_status == 1
        response.final_deg = [float(v) for v in self._last_deg[:7]]
        response.reason = reason
        return response

    def _on_feedback(self, msg):
        with self._motion_lock:
            self._on_feedback_locked(msg)

    def _on_feedback_locked(self, msg):
        self._last_deg = [msg.angle0, msg.angle1, msg.angle2, msg.angle3,
                          msg.angle4, msg.angle5, msg.angle6_raw]
        self._last_fb_time = self._now()
        if self._done is None or self._done.done() or self._target is None:
            return
        errs = [abs(self._last_deg[i] - self._target[i]) for i in range(6)]
        max_err = max(errs)
        self._stable = self._stable + 1 if max_err <= self.tolerance else 0
        fb = MoveToJoints.Feedback()
        fb.current_deg = [float(v) for v in self._last_deg]
        fb.max_err_deg = float(max_err)
        fb.phase = self._phase
        self._goal_handle.publish_feedback(fb)
        # funcode=1 通路在 7 条 recv 凑齐（_phase=='moving'）之前不latch到位，
        # 避免零行程 goal 绕过 recv 闸门。其它通路行为不变。
        if self._stable >= self.stable_frames and (
                not self._planned or self._phase == 'moving'):
            self._finish(True, 'arrived')

    def _finish(self, arrived, reason):
        with self._motion_lock:
            if self._done is None or self._done.done():
                return
            self._finish_locked(arrived, reason)

    def _finish_locked(self, arrived, reason):
        # A queued MultiThreadedExecutor take can outlive completion. Cancel the
        # one persistent handle instead of destroying it while still queued.
        # Reuse (rather than create+cancel per goal) keeps the timer count bounded.
        # arrival_timeout 曾经什么都不打，reason 里看不出是哪根轴卡住，只能自己拿
        # final_deg 减目标——跟 PLAN 日志当初缺 start= 是同一类病：关键量没打出来，
        # 事后查不出来（S-aux4c，2026-09-14 最大臂展 a1 偏差 1.2° 那次超时就是这样）。
        if reason == 'arrival_timeout' and self._target is not None:
            errs = [abs(self._last_deg[i] - self._target[i]) for i in range(6)]
            over = [i for i in range(6) if errs[i] > self.tolerance]
            self.get_logger().warn(
                'arrival_timeout 逐轴偏差(deg)=[%s] 超带轴=%s（tolerance_deg=%.1f）'
                % (', '.join('a%d=%.2f' % (i, errs[i]) for i in range(6)),
                   over, self.tolerance))
        self._cancel_motion_timer()
        if self._done is not None and not self._done.done():
            self._done.set_result(self._result(arrived, reason))

    def _result(self, arrived, reason):
        r = MoveToJoints.Result()
        r.arrived = arrived
        r.final_deg = [float(v) for v in self._last_deg]
        r.reason = reason
        r.actual_duration_sec = float(self._plan_dur)
        r.speed_limited = bool(self._plan_limited)
        r.executed_path = self._exec_path
        r.target_clamp_deg = [float(v) for v in self._target_clamp_deg]
        r.target_clamped = bool(any(v != 0.0 for v in self._target_clamp_deg))
        return r


def main():
    rclpy.init()
    node = ArmDriver()
    # MultiThreadedExecutor：/set_servo 回调阻塞等待运动/卸力时，action 与订阅回调
    # 仍在别的线程推进。已有 action/timer/订阅仍在默认 MutuallyExclusive 组，彼此串行不变。
    # 用 rclpy.spin(node, executor) 而非 executor.spin()：前者带 context.ok() 循环守卫，
    # SIGTERM 下干净退出；后者会在 shutdown 竞态里抛 RCLError traceback。
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Drain worker callbacks before Node destroys its persistent timer handle.
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
