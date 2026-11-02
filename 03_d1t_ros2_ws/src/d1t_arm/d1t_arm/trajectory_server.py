# trajectory_server.py — S4a-1：给 funcode=1 梯形规划通路套一个标准 FollowJointTrajectory
# action 服务端，节点名 d1t_trajectory，提供 /follow_joint_trajectory。
#
# 内部是 /move_to_joints 的 action 客户端：逐段把 trajectory.points 转成
# MoveToJoints goal 顺序发出去。理由（任务书裁定 1）：/arm_Command 只能有一个发布者，
# 这次是给 S2 sequence_runner 已经跑通的模式套标准接口，代价是每段多一次 action
# 往返（几十毫秒），可接受——段间本来就必须停（舵机接口单段、始末速度为零）。
#
# 只接 Joint1~Joint6（裁定 2）：夹爪是夹持指令不是位置指令（2026-09-11 裁定，见
# manifest/START_HERE.md），真去夹物体会把物体夹变形，指令位置本不该被反馈复现；
# URDF 里它还是两个 prismatic 关节 Joint_L/Joint_R，单位米，与 a6 的角度指令不是
# 一回事。goal.joint_names 含这两个名字或任何未知名字 -> INVALID_JOINTS。
#
# 单位转换（裁定 3）：FollowJointTrajectory 是弧度，我们整条栈是度。Joint1..Joint6
# -> a0..a5，弧度 × 180/π = 度，再乘 joint_signs[ax]（S4b-0 交付 2：J4/J6 与
# joint_state_bridge 的翻转同源，命令方向、feedback 方向都要乘，见文件尾
# joint_signs 说明）。a6 恒填 goal 开始时的反馈快照原样发回，不参与轨迹、
# 不被任何段改动。goal.joint_names 未列出的 a0~a5 轴同样保持这份起始快照不变——
# 我们没有那些轴的轨迹数据，唯一站得住的选择是让它们在整条轨迹执行期间不动。
#
# 段时长（裁定 4）：取 max(轨迹要求的时长, 规划器算出的最短可行时间)，实现在
# trapezoid_planner.plan() 的 duration_sec 参数里，不在本文件复制规划数学。
# 要求时长 = 相邻两点 time_from_start 之差；有要求时长的段必须把 speed_ratio=1.0
# 传给 plan()，否则"最短可行时间"这个地板是按 speed_ratio 算的，节点默认 0.3 会把
# 地板抬高、把本来硬件做得到的更快请求错误拉长（见 SPEED_RATIO_FOR_DURATION_SEC）。
# 全轨迹 time_from_start 全零/缺失时退回节点参数 speed_ratio（默认 0.3）。
# /set_override 是全局旋钮，本节点不用它做单段调速（那是 arm_driver 的职责，本节点
# 只把 duration_sec 传下去）。
#
# 取消（裁定 5）：当前段走完才停，不是急停——舵机收到 setRawAngle 发出去就会走完，
# 中途改发当前位置去刹车没有定义的补偿行为，禁止。收到 cancel 后不发下一段，等
# 当前段的 /move_to_joints 结果回来，报 canceled。**这条链上没有急停**，
# /set_stiffness mode=0 在展开姿态下会让机械臂砸下来，不是急停手段；硬要停只有断电
# ——这是登记的限制，不是本阶段要解决的问题。
#
# tolerance（裁定 6）：path_tolerance 要求一条连续参考轨迹，我们是点到点分段执行，
# 没有那个参考，不实现也不假装实现，收到非空值只记日志。goal_tolerance 不做每次
# 调用可变，沿用 arm_driver 固定的 1.0 度（tolerance_deg 参数）。goal_time_tolerance
# 做 clamp+记日志（下限 1s，S4a-0 实测整体时滞 0.45~0.86s），但不接到每段
# timeout_sec 上——arm_driver 的默认超时公式（1.5×规划时长+5s，下限 20s）已经比
# 任何合理的 goal_time_tolerance 更宽松，见 forlearning 阶段文档 §8。
#
# feedback 节奏：直接转发 /move_to_joints 的 feedback（本身由硬件反馈话题
# /d1t/joint_feedback 到达驱动，见 arm_driver._on_feedback），不新造定时器。
#
# joint_signs（S4b-0 交付 2）：joint_state_bridge 对 Joint4、Joint6 乘 -1
# （S3 坑 1，真机 J4/J6 转动方向与 URDF <axis> 相反），把控制器度数翻成 URDF
# 弧度给 RViz/MoveIt 看。这里做反向翻译——MoveIt 按 URDF 方向发来的 J4/J6
# 弧度目标，转度数命令给 /move_to_joints 前要乘回同一个符号，否则 a3/a5
# 会反着转；publish_feedback 回给 MoveIt 的角度同样要乘回去，两个方向都要处理，
# 否则命令对了、反馈方向却和目标反着，MoveIt 会一直判不到位。
# 默认值、下标含义（0..5 对应 Joint1..Joint6/a0..a5）都跟 joint_state_bridge.py
# 的 joint_signs 参数保持一致，符号是"同一件事"，不是两个独立标定。
#
# ==== S-ovl4（2026-09-16）：execution_mode 参数，point（默认）/ stream ====
#
# point：上面全部行为不变（逐点调 /move_to_joints、每点等到位），只多打每段实际耗时
#   与结尾 SUMMARY（落盘复盘用，不改执行）。
#
# stream：把 S-ovl3 gate_demo 实机验过的「不插值 + 进区域切换」搬进来，经由点不停车。
#   1. 执行侧抽稀（flyby.thin_indices，参数 thin_tol_deg，默认 1.0°）：关节空间接近
#      直线的连串点合成一段，只留拐点；被删点离保留弦线最大单轴偏差 ≤ thin_tol_deg。
#      保留点之间的段时长 = 两点 time_from_start 之差，总时长不变。≤0 关闭抽稀。
#      没改 MoveIt 的 resample_dt：那是 d1t_moveit_config 的配置内容（本阶段边界不许改），
#      而且它只改时间间隔，慢档照样出 100+ 点，笛卡尔路径也不走它。
#   2. 除最后一点外：按**名义路点**差（≥0.5°）判本段动轴，不看反馈；每根动轴发一条
#      funcode=1，delay_ms 由 trapezoid_planner.plan() 给（起点=当前反馈，段时长照上）；
#      发完等「本段动轴全部进 zone_deg」或「1.27×delay_ms+0.5 s 兜底」，先到先走，
#      立即发下一点。兜底触发且有动轴离起点不到 1° -> 记 STALL，照样继续。
#   3. 最后一点仍走 /move_to_joints 等到位（保 goal tolerance），超时 = 规划 T + 3 s；
#      超时记 STALL、同目标重发一次，再失败才 abort（S-ovl3 §D 同款）。
#   4. MoveIt 兼容：feedback 照发（经由段由本节点反馈回调直接发，最后一段转发
#      /move_to_joints 的）；收到 cancel 后不再发下一点——当前经由点进区域/兜底后返回
#      canceled，此时臂还在走向该经由点的最后 ≤zone_deg，不是急停；错误码映射不变。
#   5. 每条 goal 录反馈到 csv_dir/sovl4_fb_<时间戳>.csv，结尾 SUMMARY 报每段耗时分布、
#      每个经由点切换时刻 ±0.5 s 全轴最大速度（S-ovl3 口径）、STALL 次数。
#
# **入口独占（stream 专属，硬规矩）**：机械臂只有 /arm_Command 一个命令入口，两个节点
# 同时发，控制器每收到一条就丢掉旧目标重新规划，臂走到哪不可预测。所以：
#   a. 启动时等 discovery_wait_sec（默认 2 s）让 DDS 发现完成，再查 /arm_Command 的
#      发布者；除本节点（d1t_trajectory）和 arm_driver（d1t_arm，最后一点要用它）外
#      还有任何发布者 -> 打出是谁、拒绝启动（进程退出码 1）。检查通过前一律拒 goal。
#   b. 每条 goal 开始时、每个经由点发送前再查一次；查到外来发布者 -> 不再发任何命令，abort。
#   c. 运行期间：本节点只接一条 FollowJointTrajectory（原有）；经由段期间订阅
#      /move_to_joints 的 action 状态，发现有别人的 goal 在执行 -> 不再发命令，abort。
#   d. 做不到的：本节点无法让 arm_driver 拒绝别人的 /move_to_joints 或 /set_servo
#      （那要改 arm_driver，本阶段边界不许），只能「发现即停发」，不能事前拦住。
#      所以 stream 运行时**人不要手动调 /move_to_joints、/set_servo，不要开 gate_demo、
#      traj_test_client 之外的任何发命令节点**。

import datetime
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_action_status_default)
from rclpy.task import Future
from rcl_interfaces.msg import ParameterDescriptor

from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from unitree_arm.msg import ArmString

from d1t_interfaces.action import MoveToJoints
from d1t_interfaces.msg import JointFeedback

from d1t_arm import flyby
from d1t_arm.trapezoid_planner import clamp_to_ctrl, plan

# Joint1..Joint6（URDF 命名）-> a0..a5（控制器轴号）。夹爪 Joint_L/Joint_R 不在表内，
# 任何不在这张表里的名字都被 validate_joint_names 拒绝。
JOINT_AXIS = {'Joint%d' % (i + 1): i for i in range(6)}

# 与 joint_state_bridge.py 的 joint_signs 默认值同源（S3 坑 1）：J4、J6 反向。
JOINT_SIGNS_DEFAULT = [1.0, 1.0, 1.0, -1.0, 1.0, -1.0]
_ANY = ParameterDescriptor(dynamic_typing=True)

ACCEL_RATIO_DEFAULT = 1.0   # funcode=1 本阶段无效，随 goal 传给 plan() 只为将来生效
SEGMENT_DURATION_FLOOR_SEC = 1e-9   # 不是"要求的时长"，只是让 plan() 走"要求时间"分支的占位
GOAL_TIME_TOLERANCE_FLOOR_SEC = 1.0   # S4a-0 实测整体时滞 0.45~0.86s，低于 1s 会误判超时

SPEED_RATIO_FOR_DURATION_SEC = 1.0   # 见 speed_ratio_for_segment()：地板必须是硬件真最短时间

EXECUTION_MODES = ('point', 'stream')
# stream 入口独占：/arm_Command 上只允许这两个节点是发布者（本节点 + arm_driver）。
ALLOWED_CMD_PUBLISHERS = ('d1t_trajectory', 'd1t_arm')
F1_GAP_SEC = 0.002      # 同一点内各轴命令间隔，同 arm_driver funcode1_gap_ms 默认值
_ACTIVE_GOAL_STATES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                       GoalStatus.STATUS_CANCELING)


def validate_joint_names(names):
    """goal.trajectory.joint_names -> (axes, error)。

    axes[k] 是 names[k] 对应的机械臂轴号 0~5（Joint1->0 .. Joint6->5），与
    positions[k] 一一对应。含 Joint_L/Joint_R 或任何其它未知名字 -> (None, 人话说明
    第一个不认识的名字)。空列表不算错，返回 ([], None)——点数/长度校验在别处做。
    """
    axes = []
    for n in names:
        if n not in JOINT_AXIS:
            return None, (
                "未知关节名 '%s'：本接口只接 Joint1~Joint6，夹爪（Joint_L/Joint_R）"
                "是夹持指令不是位置指令，不进轨迹接口" % n)
        axes.append(JOINT_AXIS[n])
    return axes, None


def point_to_deg7(base_deg7, axes, positions_rad, signs):
    """base_deg7：7 元素度值，goal 开始时的反馈快照——未被 axes 覆盖的轴（含 a6）
    原样保留。axes/positions_rad：与 goal.joint_names 顺序一致，只覆盖 axes 指到的
    那几个轴，弧度转度再乘 signs[ax]（J4/J6 那两轴翻回控制器方向，见文件头
    joint_signs 说明；其余轴 signs=1 不影响）。"""
    out = list(base_deg7)
    for ax, rad in zip(axes, positions_rad):
        out[ax] = math.degrees(rad) * signs[ax]
    return out


def duration_to_sec(sec, nanosec):
    return float(sec) + float(nanosec) * 1e-9


def all_zero_time(time_from_start_pairs):
    """time_from_start_pairs：[(sec, nanosec), ...]。全部为 (0, 0) -> True，
    判定退回节点参数 speed_ratio 兜底（裁定 4「全零或缺失」）。"""
    return all(sec == 0 and nsec == 0 for sec, nsec in time_from_start_pairs)


def segment_durations_sec(time_from_start_pairs):
    """相邻两点 time_from_start 之差（秒）；第 0 点相对基准 0 秒。
    不做正负号判断——负值/零值交给 duration_sec_for_segment 处理。"""
    out = []
    prev = 0.0
    for sec, nsec in time_from_start_pairs:
        t = duration_to_sec(sec, nsec)
        out.append(t - prev)
        prev = t
    return out


def duration_sec_for_segment(seg_duration_raw, fallback):
    """fallback=True（整条轨迹 time_from_start 全零/缺失）-> 0.0，交给
    trapezoid_planner.plan() 用节点默认 speed_ratio 计算时长。

    否则：seg_duration_raw 为正照用；<=0（时间戳不增或重复，通常是调用方笔误）
    视为"能有多快就多快"，钳到一个不可感知的极小正数——这样 plan() 仍会把它当
    "有时长要求"来跟硬件最短时间比较、按硬件最短时间执行并标记 speed_limited，
    不会静默改判成"没给时长"从而落回默认 speed_ratio（丢失"这段本该多快"的意图，
    等于悄悄放慢——裁定 4 明确禁止）。
    """
    if fallback:
        return 0.0
    return seg_duration_raw if seg_duration_raw > 0.0 else SEGMENT_DURATION_FLOOR_SEC


def speed_ratio_for_segment(dur, node_speed_ratio):
    """给某一段 /move_to_joints goal 的 speed_ratio：

    dur>0（这段有 duration_sec 要求，即 duration_sec_for_segment 的非 fallback 输出）
    -> SPEED_RATIO_FOR_DURATION_SEC（1.0）。plan() 里 duration_sec 的地板是
    max(duration_sec, t_base)，t_base 按传给 plan() 的 speed_ratio 算——传节点默认
    的 node_speed_ratio 会把地板抬高到"那个倍率下的最短时间"而不是硬件真正的最短
    时间，调用方给一个硬件做得到、只是比节点默认倍率更快的 duration_sec 就会被
    错误拉长并标 speed_limited，与裁定 4「不许悄悄放慢」冲突。

    dur==0（time_from_start 全零/缺失的兜底，duration_sec_for_segment 只在这种
    fallback 情形下返回 0.0）-> node_speed_ratio，退回节点参数原有行为，不受影响。
    """
    return SPEED_RATIO_FOR_DURATION_SEC if dur > 0.0 else node_speed_ratio


def thinned_segment_durations(time_from_start_pairs, keep):
    """抽稀后每段的原始时长：相邻两个保留点 time_from_start 之差，第一个保留点相对 0 秒。
    被删的点时长并进下一个保留点，所以总时长不变。"""
    out = []
    prev = 0.0
    for idx in keep:
        t = duration_to_sec(*time_from_start_pairs[idx])
        out.append(t - prev)
        prev = t
    return out


def clamp_goal_time_tolerance(sec):
    """goal.goal_time_tolerance 转出的秒数 -> (clamped, was_raised)。
    下限 GOAL_TIME_TOLERANCE_FLOOR_SEC，见任务书裁定 6。"""
    if sec < GOAL_TIME_TOLERANCE_FLOOR_SEC:
        return GOAL_TIME_TOLERANCE_FLOOR_SEC, True
    return float(sec), False


def map_error_code(reason):
    """/move_to_joints result.reason -> control_msgs 标准错误码。

    该枚举没有『硬件未回执』『尚未就绪』这类档位，只能选最接近的近似（登记为
    简化，见阶段文档 §8）：out_of_limits（目标本身超硬件行程）像是目标不合法
    -> INVALID_GOAL；其余失败（no_recv/rejected_recv/arrival_timeout/not_ready 等）
    都是"没能在预期内到位" -> GOAL_TOLERANCE_VIOLATED，是现成枚举里最接近的档。
    """
    if reason == 'out_of_limits':
        return FollowJointTrajectory.Result.INVALID_GOAL
    return FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED


class TrajectoryServer(Node):

    def __init__(self):
        super().__init__('d1t_trajectory')
        self._max_points = self.declare_parameter('max_points', 64).value
        self._speed_ratio = self.declare_parameter('speed_ratio', 0.3).value
        self._joint_signs = [float(x) for x in self.declare_parameter(
            'joint_signs', JOINT_SIGNS_DEFAULT, _ANY).value]
        self._mode = str(self.declare_parameter('execution_mode', 'point').value)
        if self._mode not in EXECUTION_MODES:
            raise ValueError("execution_mode=%r 非法，只能是 'point' 或 'stream'" % self._mode)
        self._zone_deg = float(self.declare_parameter(
            'zone_deg', 2.0).value)  # 2026-10-27 Thomas：2° 定为默认（原 5.0）
        self._thin_tol_deg = float(self.declare_parameter(
            'thin_tol_deg', flyby.THIN_TOL_DEG_DEFAULT).value)
        self._csv_dir = str(self.declare_parameter('csv_dir', '/tmp').value)
        self.discovery_wait_sec = float(self.declare_parameter(
            'discovery_wait_sec', 2.0).value)
        if self._zone_deg <= 0.0:
            raise ValueError('zone_deg=%.3f 必须 >0' % self._zone_deg)

        self._last_deg = None   # 最近一帧关节反馈（7 元素度）
        self._desired = None    # 当前在跑的段的目标（7 元素度），feedback 转换要用
        self._active = False
        self._active_lock = threading.Lock()
        # point 模式直接放行；stream 要等 main() 里的入口独占检查通过才放行。
        self.entry_ok = (self._mode == 'point')
        self._seg_elapsed = []      # 本条 goal 每段实际耗时（秒），SUMMARY 用
        self._stall_count = 0
        # --- stream 专属状态 ---
        self._seq = 0
        self._zone_wait = None      # (axes, target7, Future)：经由点等进区域
        self._stream_fb = None      # (goal_handle, names, axes)：经由段由反馈回调直接发 FJT feedback
        self._recording = None      # 本条 goal 的反馈录制 [(t_mono, deg7), ...]；None=不录
        self._mtj_active_foreign = False   # /move_to_joints 上有不是本节点发的 goal 在跑
        self._own_mtj_goal_id = None

        jf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            JointFeedback, '/d1t/joint_feedback', self._on_feedback, jf_qos)

        self._client = ActionClient(self, MoveToJoints, '/move_to_joints')
        if self._mode == 'stream':
            cmd_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self._cmd_pub = self.create_publisher(ArmString, '/arm_Command', cmd_qos)
            self.create_subscription(
                GoalStatusArray, '/move_to_joints/_action/status', self._on_mtj_status,
                qos_profile_action_status_default)
        self._server = ActionServer(
            self, FollowJointTrajectory, '/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT)
        self.get_logger().info(
            'd1t_trajectory ready: action /follow_joint_trajectory '
            '(client of /move_to_joints), max_points=%d speed_ratio=%.2f '
            'joint_signs=%s execution_mode=%s zone_deg=%.1f thin_tol_deg=%.2f'
            % (self._max_points, self._speed_ratio, self._joint_signs, self._mode,
               self._zone_deg, self._thin_tol_deg))

    def _on_feedback(self, msg):
        self._last_deg = [msg.angle0, msg.angle1, msg.angle2, msg.angle3,
                           msg.angle4, msg.angle5, msg.angle6_raw]
        if self._recording is not None:
            self._recording.append((time.monotonic(), list(self._last_deg)))
        if self._stream_fb is not None and self._desired is not None:
            gh, names, axes = self._stream_fb
            self._publish_fjt_feedback(gh, names, axes, self._last_deg)
        zw = self._zone_wait
        if zw is not None and not zw[2].done() and flyby.in_zone(
                self._last_deg, zw[1], zw[0], self._zone_deg):
            zw[2].set_result(True)

    def _on_mtj_status(self, msg):
        own = self._own_mtj_goal_id
        self._mtj_active_foreign = any(
            st.status in _ACTIVE_GOAL_STATES
            and (own is None or bytes(st.goal_info.goal_id.uuid) != own)
            for st in msg.status_list)

    def foreign_cmd_publishers(self):
        infos = self.get_publishers_info_by_topic('/arm_Command')
        return flyby.foreign_publishers(
            [(i.node_namespace, i.node_name) for i in infos], ALLOWED_CMD_PUBLISHERS)

    def _on_goal(self, _request):
        if not self.entry_ok:
            self.get_logger().warn('goal rejected: stream 入口独占检查尚未通过')
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                self.get_logger().warn('goal rejected: another trajectory is running')
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        self._seg_elapsed = []
        self._stall_count = 0
        t0 = time.monotonic()
        result = None
        try:
            result, status = await self._run(goal_handle)
        finally:
            self._zone_wait = None
            self._stream_fb = None
            self._own_mtj_goal_id = None
            n, tot, mn, med, mx = flyby.elapsed_stats(self._seg_elapsed)
            self.get_logger().info(
                'SUMMARY mode=%s error_code=%s total=%.3fs segs=%d seg_sum=%.3fs '
                'seg_min=%.3fs seg_median=%.3fs seg_max=%.3fs STALL count=%d'
                % (self._mode, result.error_code if result is not None else 'exception',
                   time.monotonic() - t0, n, tot, mn, med, mx, self._stall_count))
            self.get_logger().info(
                'SUMMARY seg_elapsed=[%s]' % ', '.join('%.3f' % v for v in self._seg_elapsed))
            with self._active_lock:
                self._active = False
        if status == 'succeeded':
            goal_handle.succeed()
        elif status == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    async def _run(self, goal_handle):
        goal = goal_handle.request
        traj = goal.trajectory
        names = list(traj.joint_names)
        points = traj.points

        if goal.path_tolerance or goal.component_path_tolerance:
            self.get_logger().info(
                'FJT: goal 带 path_tolerance，本实现不支持（点到点分段执行，没有连续'
                '参考轨迹可比对），已忽略')

        if not points:
            return self._abort('trajectory.points 为空，无点可执行',
                                FollowJointTrajectory.Result.INVALID_GOAL)

        if len(points) > self._max_points:
            return self._abort(
                'points 数 %d 超过 max_points=%d，拒绝' % (len(points), self._max_points),
                FollowJointTrajectory.Result.INVALID_GOAL)

        axes, err = validate_joint_names(names)
        if err is not None:
            return self._abort(err, FollowJointTrajectory.Result.INVALID_JOINTS)

        for i, p in enumerate(points):
            if len(p.positions) != len(names):
                return self._abort(
                    'points[%d].positions 长度 %d 与 joint_names 长度 %d 不符'
                    % (i, len(p.positions), len(names)),
                    FollowJointTrajectory.Result.INVALID_GOAL)

        if self._last_deg is None:
            return self._abort('尚无关节反馈，无法确定起点（not_ready）',
                                FollowJointTrajectory.Result.INVALID_GOAL)

        gtt_sec = duration_to_sec(
            goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec)
        gtt_clamped, gtt_raised = clamp_goal_time_tolerance(gtt_sec)
        if gtt_raised:
            self.get_logger().info(
                'FJT: goal_time_tolerance=%.3fs 低于 %.1fs 下限（S4a-0 实测整体时滞 '
                '0.45~0.86s），抬到 %.3fs（仅记录，未接入每段超时——arm_driver 默认'
                '超时公式已更宽松）' % (gtt_sec, GOAL_TIME_TOLERANCE_FLOOR_SEC, gtt_clamped))

        raw_tfs = [(p.time_from_start.sec, p.time_from_start.nanosec) for p in points]
        fallback = all_zero_time(raw_tfs)
        if fallback:
            self.get_logger().info(
                'FJT: 轨迹 time_from_start 全零/缺失，逐段退回节点参数 speed_ratio=%.2f'
                % self._speed_ratio)
        seg_raw = segment_durations_sec(raw_tfs)

        base = list(self._last_deg)
        if self._mode == 'stream':
            return await self._run_stream(goal_handle, names, axes, points, raw_tfs,
                                          fallback, base)
        n = len(points)
        limited = 0
        for i, p in enumerate(points):
            if goal_handle.is_cancel_requested:
                return self._canceled(i, n, limited)

            target = point_to_deg7(base, axes, list(p.positions), self._joint_signs)
            dur = duration_sec_for_segment(seg_raw[i], fallback)

            mv_goal = MoveToJoints.Goal()
            mv_goal.target_deg = [float(v) for v in target]
            mv_goal.timeout_sec = 0.0
            mv_goal.speed_ratio = speed_ratio_for_segment(dur, self._speed_ratio)
            mv_goal.accel_ratio = ACCEL_RATIO_DEFAULT
            mv_goal.duration_sec = dur

            self._desired = target
            self.get_logger().info(
                'FJT: segment[%d/%d] target=%s duration_sec=%.3f%s'
                % (i, n, ['%.2f' % v for v in target], dur,
                   ' (fallback)' if fallback else ''))
            t_seg = time.monotonic()
            send_fut = self._client.send_goal_async(
                mv_goal, feedback_callback=self._on_move_feedback_factory(
                    goal_handle, names, axes))
            handle = await send_fut
            if handle is None or not handle.accepted:
                return self._abort(
                    'segment[%d]/%d 被 /move_to_joints 拒绝（另一个 goal 在跑，或未'
                    '就绪）' % (i, n),
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)

            result_fut = handle.get_result_async()
            mv_result = (await result_fut).result
            self._seg_elapsed.append(time.monotonic() - t_seg)
            self.get_logger().info(
                'SEG %d/%d point elapsed=%.3fs reason=%s'
                % (i, n, self._seg_elapsed[-1], mv_result.reason))
            if bool(mv_result.speed_limited):
                limited += 1
            if mv_result.reason != 'arrived':
                return self._abort(
                    'segment[%d]/%d 失败，/move_to_joints reason=%s（原样搬运，未'
                    '翻译）' % (i, n, mv_result.reason),
                    map_error_code(mv_result.reason))

        return self._success(n, limited)

    # ---------------- stream（S-ovl4，见文件头）----------------
    async def _run_stream(self, goal_handle, names, axes, points, raw_tfs, fallback, base):
        foreign = self.foreign_cmd_publishers()
        if foreign:
            return self._abort(
                'stream 入口独占：/arm_Command 上有外来发布者 %s，未发任何命令' % foreign,
                FollowJointTrajectory.Result.INVALID_GOAL)

        targets = [point_to_deg7(base, axes, list(p.positions), self._joint_signs)
                   for p in points]
        keep = flyby.thin_indices([t[:6] for t in targets], base[:6], self._thin_tol_deg)
        seg_raw = thinned_segment_durations(raw_tfs, keep)
        n = len(keep)
        # S4b-3 任务③：采数用，供 PATH_DEV 对比实际路径与 MoveIt 规划路径的偏差。
        self._stream_planned_dense = [t[:6] for t in targets]
        self._stream_kept = [targets[k][:6] for k in keep]
        self._stream_start = base[:6]
        self.get_logger().info(
            'THIN points_in=%d kept=%d thin_tol_deg=%.2f kept_idx=%s'
            % (len(points), n, self._thin_tol_deg, keep))

        self._recording = []
        t_goal0 = time.monotonic()
        switch_times = []
        limited = 0
        prev_nominal = base[:6]
        try:
            for k, idx in enumerate(keep):
                if goal_handle.is_cancel_requested:
                    return self._canceled(k, n, limited)
                target = targets[idx]
                dur = duration_sec_for_segment(seg_raw[k], fallback)
                sr = speed_ratio_for_segment(dur, self._speed_ratio)
                self._desired = target
                t_seg = time.monotonic()

                if k == n - 1:
                    res = await self._stream_final(goal_handle, names, axes, k, n, target, dur, sr)
                    self._seg_elapsed.append(time.monotonic() - t_seg)
                    self.get_logger().info(
                        'SEG %d/%d stream-final elapsed=%.3fs dur_req=%.3fs reason=%s'
                        % (k, n, self._seg_elapsed[-1], dur,
                           res.reason if res is not None else 'rejected'))
                    if res is None:
                        return self._abort(
                            'segment[%d]/%d 被 /move_to_joints 拒绝（另一个 goal 在跑，或未'
                            '就绪）' % (k, n),
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
                    if bool(res.speed_limited):
                        limited += 1
                    if res.reason != 'arrived':
                        return self._abort(
                            'segment[%d]/%d 失败，/move_to_joints reason=%s（原样搬运，未'
                            '翻译）' % (k, n, res.reason),
                            map_error_code(res.reason))
                    break

                err = self._stream_guard()
                if err is not None:
                    return self._abort(
                        'segment[%d]/%d 前入口独占检查失败：%s，不再发命令' % (k, n, err),
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
                pr = await self._stream_via(goal_handle, names, axes, k, n,
                                            prev_nominal, target, dur, sr)
                prev_nominal = target[:6]
                self._seg_elapsed.append(time.monotonic() - t_seg)
                if pr is not None:
                    switch_times.append(time.monotonic())
                    if pr.speed_limited:
                        limited += 1
        finally:
            rows = self._recording
            self._recording = None
            self._stream_fb = None
            self._report_stream(rows, switch_times, t_goal0)
        return self._success(n, limited)

    def _stream_guard(self):
        foreign = self.foreign_cmd_publishers()
        if foreign:
            return '/arm_Command 上出现外来发布者 %s' % foreign
        if self._mtj_active_foreign:
            return '/move_to_joints 上有别人的 goal 在执行'
        return None

    async def _stream_via(self, goal_handle, names, axes, k, n, prev_nominal, target, dur, sr):
        """经由点：名义动轴各发一条 funcode=1，等进区域或兜底。返回 plan 结果；
        名义无位移（没发命令）返回 None。"""
        start = list(self._last_deg)
        mov = flyby.moving_axes(prev_nominal, target[:6])
        if not mov:
            self.get_logger().info('SEG %d/%d stream-via 名义无位移，跳过发送' % (k, n))
            return None
        pr = plan(start, target, sr, ACCEL_RATIO_DEFAULT, 1.0, duration_sec=dur)
        sent, _ = clamp_to_ctrl(target, mov)
        t_send = time.monotonic()
        for i, ax in enumerate(mov):
            if i:
                time.sleep(F1_GAP_SEC)
            msg = ArmString()
            msg.data = flyby.f1_command_json(self._seq, ax, sent[ax], pr.delay_ms)
            self._seq += 1
            self._cmd_pub.publish(msg)

        self._stream_fb = (goal_handle, names, axes)
        fut = Future()
        wait_sec = flyby.fallback_timeout_sec(pr.delay_ms)
        self._zone_wait = (mov, target, fut)
        timer = self.create_timer(wait_sec, lambda: fut.done() or fut.set_result(False))
        try:
            entered = await fut
        finally:
            self._zone_wait = None
            self.destroy_timer(timer)

        self.get_logger().info(
            'SEG %d/%d stream-via elapsed=%.3fs entered=%s dur_req=%.3fs T=%.3fs delay_ms=%d '
            'fallback=%.3fs axes=%s start=%s target=%s'
            % (k, n, time.monotonic() - t_send, entered, dur, pr.duration_sec, pr.delay_ms,
               wait_sec, mov, ['%.2f' % v for v in start[:6]],
               ['%.2f' % v for v in target[:6]]))
        if not entered:
            stalled = flyby.stalled_axes(start, self._last_deg, mov)
            if stalled:
                self._stall_count += 1
                self.get_logger().warn(
                    'STALL seg=%d/%d(via) axes=%s（兜底 %.3fs 内未离开起点 %.1f°）'
                    % (k, n, stalled, wait_sec, flyby.STALL_AXIS_EPS_DEG))
        return pr

    async def _stream_final(self, goal_handle, names, axes, k, n, target, dur, sr):
        """最后一点：/move_to_joints 等到位，超时 = 规划 T + 3 s；超时记 STALL 重发一次。"""
        self._stream_fb = None
        start = list(self._last_deg)
        timeout = (plan(start, target, sr, ACCEL_RATIO_DEFAULT, 1.0, duration_sec=dur)
                   .duration_sec + flyby.MOVE_TIMEOUT_MARGIN_SEC)
        res = await self._send_move(goal_handle, names, axes, target, dur, sr, timeout)
        if res is None or res.arrived or res.reason != 'arrival_timeout':
            return res
        mov = flyby.moving_axes(start[:6], target[:6], flyby.STALL_AXIS_EPS_DEG)
        self._stall_count += 1
        self.get_logger().warn(
            'STALL seg=%d/%d(final) axes=%s（%.3fs 超时内未离开起点 %.1f°），同目标重发一次'
            % (k, n, flyby.stalled_axes(start, list(res.final_deg), mov), timeout,
               flyby.STALL_AXIS_EPS_DEG))
        return await self._send_move(goal_handle, names, axes, target, dur, sr, timeout)

    async def _send_move(self, goal_handle, names, axes, target, dur, sr, timeout):
        mv_goal = MoveToJoints.Goal()
        mv_goal.target_deg = [float(v) for v in target]
        mv_goal.timeout_sec = float(timeout)
        mv_goal.speed_ratio = sr
        mv_goal.accel_ratio = ACCEL_RATIO_DEFAULT
        mv_goal.duration_sec = dur
        handle = await self._client.send_goal_async(
            mv_goal, feedback_callback=self._on_move_feedback_factory(goal_handle, names, axes))
        if handle is None or not handle.accepted:
            return None
        self._own_mtj_goal_id = bytes(handle.goal_id.uuid)
        try:
            return (await handle.get_result_async()).result
        finally:
            self._own_mtj_goal_id = None

    def _report_stream(self, rows, switch_times, t0):
        rel = [(t - t0, d) for t, d in (rows or [])]
        sw = [t - t0 for t in switch_times]
        # S4b-3 任务③：实际路径相对规划路径的最大单轴偏差（dense=抽稀前 / kept=抽稀后）。
        # 纯观测，不改执行行为。用最近一帧反馈做对比，10Hz 采样会低估真峰值——口径如实报。
        if rows and getattr(self, '_stream_planned_dense', None):
            dev_dense, dev_kept = flyby.path_deviation(
                rows, self._stream_planned_dense, self._stream_kept,
                start_deg=self._stream_start)
            self.get_logger().info(
                'PATH_DEV max_single_axis_deg: vs_planned(dense)=%.2f vs_thinned(kept)=%.2f '
                '(n_rows=%d n_dense=%d n_kept=%d)'
                % (dev_dense, dev_kept, len(rows), len(self._stream_planned_dense),
                   len(self._stream_kept)))
        speeds = flyby.corner_speeds(rel, sw)
        for i, (ts, v) in enumerate(zip(sw, speeds)):
            self.get_logger().info(
                'CORNER %d t=%.3fs max_axis_speed=%s'
                % (i, ts, '%.1f deg/s' % v if v is not None else 'n/a(窗口内不足两帧)'))
        valid = [v for v in speeds if v is not None]
        csv_path = ''
        if self._csv_dir and rel:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(self._csv_dir, 'sovl4_fb_%s.csv' % stamp)
            with open(csv_path, 'w') as f:
                f.write('t_rel_s,' + ','.join('a%d' % i for i in range(7)) + '\n')
                for t, d in rel:
                    f.write('%.4f,' % t + ','.join('%.3f' % v for v in d) + '\n')
                f.write('# switch_t_rel_s=' + ';'.join('%.4f' % t for t in sw) + '\n')
        self.get_logger().info(
            'CORNER_SUMMARY corners=%d min_max_axis_speed=%s below_1deg_s=%d frames=%d csv=%s'
            % (len(sw), '%.1f' % min(valid) if valid else 'n/a',
               sum(1 for v in valid if v < 1.0), len(rel), csv_path or '(未写)'))

    def _on_move_feedback_factory(self, goal_handle, names, axes):
        def _cb(fb_msg):
            self._publish_fjt_feedback(goal_handle, names, axes, fb_msg.feedback.current_deg)
        return _cb

    def _publish_fjt_feedback(self, goal_handle, names, axes, current_deg):
        signs = self._joint_signs
        actual_rad = [math.radians(current_deg[ax]) * signs[ax] for ax in axes]
        desired_rad = [math.radians(self._desired[ax]) * signs[ax] for ax in axes]
        error_rad = [a - d for a, d in zip(actual_rad, desired_rad)]
        out = FollowJointTrajectory.Feedback()
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(names)
        out.actual = JointTrajectoryPoint(positions=actual_rad)
        out.desired = JointTrajectoryPoint(positions=desired_rad)
        out.error = JointTrajectoryPoint(positions=error_rad)
        goal_handle.publish_feedback(out)

    def _success(self, n, limited):
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        r.error_string = (
            '轨迹执行完成：共 %d 段，%d 段被拉长（比要求时间还快，已按硬件最短时间'
            '执行）' % (n, limited))
        return r, 'succeeded'

    def _abort(self, msg, code):
        r = FollowJointTrajectory.Result()
        r.error_code = code
        r.error_string = msg
        self.get_logger().warn('FJT abort: %s' % msg)
        return r, 'aborted'

    def _canceled(self, done_n, total_n, limited):
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        r.error_string = (
            '已取消：收到取消请求时已完成 %d/%d 段，当前段走完后停止（不是急停，'
            '不会中途改发位置刹车），不再发下一段；已完成段中 %d 段被拉长'
            % (done_n, total_n, limited))
        self.get_logger().info('FJT canceled: %s' % r.error_string)
        return r, 'canceled'


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryServer()
    rc = 0
    try:
        if not node.entry_ok:
            # stream 入口独占（见文件头）：先让 DDS 发现跑完，再查 /arm_Command 发布者。
            end = time.monotonic() + node.discovery_wait_sec
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            foreign = node.foreign_cmd_publishers()
            if foreign:
                node.get_logger().error(
                    '拒绝启动 stream：/arm_Command 上除 d1t_trajectory、d1t_arm 外还有发布者 %s。'
                    '两个节点同时发命令，控制器每收到一条就丢旧目标重新规划，臂走到哪不可预测。'
                    '先关掉它们再启动。' % foreign)
                rc = 1
            else:
                node.entry_ok = True
                node.get_logger().info(
                    'stream 入口独占检查通过（/arm_Command 发布者只有 d1t_trajectory/d1t_arm），'
                    '开始接 goal')
        if rc == 0:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
