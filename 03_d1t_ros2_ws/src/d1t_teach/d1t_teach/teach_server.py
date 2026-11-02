#!/usr/bin/env python3
"""teach_server.py — TEACH-01：Web 手动控制界面（示教器第一版）的后端。

一个 ROS2 节点 + 一个标准库 http.server。浏览器打开页面 → 轮询 /api/state 取状态，
POST /api/* 下命令。**不新增任何 apt / pip 依赖**：HTTP 用 Python 标准库，前端是
可直接打开的静态文件、无 npm 构建链。

命令链（与 MoveIt 主干一致，不绕过碰撞检查）：
  关节点动 / 绝对定位 / 预设点位 → /move_action（MoveIt 规划 + 碰撞检查 + 执行）
  笛卡尔增量 → /compute_cartesian_path（avoid_collisions=True）→ /execute_trajectory
  夹爪 → /grip（GRIP-01 已入库的判定，不另写一套）
  伺服 → /set_servo（只做服务外壳，reason 原样显示）
  倍率 → /set_override
执行模式（point / stream）是 trajectory_server 的启动参数、运行中改不了（OPERATIONS.md F），
所以本节点自己把 trajectory_server 作为子进程拉起，切模式 = 空闲时重起它。已经有别人
起好的 trajectory_server 时不抢，改为只读显示、禁用切换。

安全：界面上没有急停按钮。cancel、Ctrl-C、停止发布都不是急停，唯一能立即停的是断电。
服务端只允许绑定回环或直连网段（bind_host 参数有白名单）。
"""

import datetime
import json
import math
import os
import posixpath
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import (GetCartesianPath, GetPlanningScene, GetPositionFK,
                             GetPositionIK)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_arm.msg import ArmString

from d1t_arm import flyby

from d1t_interfaces.msg import JointFeedback
from d1t_interfaces.srv import Grip, SetOverride, SetServo, SetStiffness

from . import gating, presets, units

GROUP = 'arm'
BASE_FRAME = 'base_link'
# SRDF 里规划组 arm 的 chain tip_link；逆解与笛卡尔路径一律对它求
IK_TIP_LINK = 'tcp_link'
JOINTS = units.URDF_JOINT_NAMES

# 笛卡尔直线参数：与 s4b4_tcp 同值（任务书点名照它的做法）
CART_MAX_STEP_M = 0.005
CART_JUMP_THRESHOLD = 5.0
CART_MAX_POINTS = 11
MIN_FRACTION = 0.999

GOAL_TIMEOUT_SEC = 120.0
SERVICE_TIMEOUT_SEC = 40.0
EXTERNAL_HOLD_SEC = 2.0
# 空闲时「是不是别人在动臂」不看单帧差，看一段时间里的**净位移**。
# 原因（2026-09-23 实机）：臂停在某些姿态上伺服会自己抖，反馈上表现为每帧 0.6~0.7°
# 来回摆、净位移接近 0。按单帧差判会被一直判成外部运动，界面永久锁死。
# 别人真在动臂是单向走的，净位移会持续累积；抖动不会。
EXTERNAL_WIN_SEC = 2.0
EXTERNAL_NET_DEG = 3.0        # 空闲时 EXTERNAL_WIN_SEC 内任一轴净走超过它 = 别人在动臂
# 一条命令返回之后臂还在停稳，这段时间里的运动算自己的，不算外部运动。
# /set_servo 在折叠区内会自己规划回 HOME 并把夹爪送到 0，夹爪走完比服务返回晚得多，
# 所以服务类给的窗口比运动类长。
SETTLE_MOTION_SEC = 2.0
SETTLE_SERVICE_SEC = 8.0
# /move_to_joints 目标按 goal_id 认人：本界面执行期间出现过的 id 记下来算自己的，
# 其余一律算外人。不用时间窗口——外人的目标可能在几百毫秒里发出、走完、结束，
# 时间窗口会整个漏掉（实测：停稳窗口内发的外部目标一次都没报出来）。
OWN_GOAL_ID_CAP = 512
TRAJ_NODE_NAME = 'd1t_trajectory'
# 执行模式固定 stream（Thomas 2026-09-22 裁定：示教器只用 stream，不做 point、不给切换）。
# point 每个路点都经 /move_to_joints 等到位，一次几毫米的 jog 要等好几秒，示教器没法用；
# 代价是 stream 不校验中间点是否走到（trajectory_server._stream_via 超时只记日志），
# 界面为此常驻一行提示，靠近物体由操作员自己判断。
# 真要 point：用 -p manage_trajectory_server:=false 起本节点，自己按 OPERATIONS A.3.6 起
# trajectory_server，本界面只跟着用、不管它是什么模式。
EXEC_MODE = 'stream'

# 起点限制：主窗口 2026-09-22 W4·2d 只放行「折叠姿态 → 0 点」；
# Thomas 2026-09-23 裁定取消，任意姿态都可以按 0 点（二次确认保留）。
ZERO_PRESET_ID = 'zero'
ZERO_REQUIRES_FOLD_START = False
TRAJ_RESTART_TIMEOUT_SEC = 25.0

# ---------- 按住即走（Thomas 2026-09-23 裁定） ----------
# 这条通路**不经 MoveIt 规划、也不做任何碰撞检查**（Thomas 2026-09-23 取消了任务书
# 「所有臂轴运动必须经 MoveIt 规划并通过碰撞检查」）。按住时按固定周期直接发 funcode=1，
# 松手停发。松手不是刹车：funcode=1 是「走到目标」，停发只是不再给新目标，
# 臂会走完最后一条的剩余行程。唯一能立即停的仍然是断电。
# 发送周期与 delay_ms 做成启动参数：臂上控制器「运动中收到新 funcode=1 是否立即换目标」
# **未验证**（fake_d1t 的注释原话）。仿真里 fake_d1t 每收一条新目标就重置起动延时
# （start_delay_sec 默认 0.3 s），10 Hz 发命令会让它整段不动、停发后才走一次——
# 仿真验不了连续追踪，实机第一次按住必须盯着反馈看它是不是在按住期间就连续变化。
# 发段频率：臂跟得上时每秒最多发这么多段。不是硬时钟——上一段没走进区域就不发下一段，
# 这是 stream 流畅的根本原因（trajectory_server._stream_via：发完等进 zone_deg 再发下一点）。
# 原来固定 10 Hz 硬发会顿挫，根因在厂商 marm_controller_node.cpp 的 subServoAngle_callback：
# 每条命令都 servo[id].setRawAngle(angle, testdelay_ms) 重开一条加减速曲线，且加速度被
# 截在 maxalpha = 286.4789/4 ≈ 71.6 °/s²。2° / delay_ms=79 算出来 testdelay_ms ≈ 129 ms，
# 比 100 ms 的发送周期长——臂每次刚起步就被下一条命令打断重新起步，于是一顿一顿。
# 默认值 2.0 / 60.0 是 Thomas 实机试出来更顺的值（2026-09-23 裁定定为默认，原 4.0 / 35.0）。
HOLD_HZ = 2.0                   # 名义段频率（段长 0.5 s）；实际发段由「进区域」触发
HOLD_MIN_SEG_SEC = 0.08         # 两段之间最短间隔：臂到得再快也不把命令发成一串碎片
HOLD_MAX_LOOKAHEAD_SEC = 0.5    # 单段最长前瞻：臂卡住时目标不会跑得太远（松手滑行的上界）
HOLD_POLL_SEC = 0.01            # 到点前的轮询间隔（越粗，发段时刻越容易冲过头）
# 下一段在「上一段预计走完」的这个比例处发出。必须 < 1：等它走完再发 = 每段之间停一次车，
# 那就是顿挫。实测（2026-09-23，probe 抓 /arm_Command 原始帧）：
#   关节 a0：单段 3.20°、delay_ms=127 → 控制器要走 198 ms，实际每 162 ms 发一段
#            → 重叠 36 ms，不停车，Thomas 说这条顺
#   笛卡尔 z：最大轴 3.06°、delay_ms=134 → 要走 205 ms，实际每 211 ms 才发
#            → 空档 6 ms，停住再起步，Thomas 说这条顿
# 关节那条重叠 198−162 = 36 ms，所以直接把「目标重叠」定成 35 ms，笛卡尔按同一个标准走。
# 「上一段要走多久」用 flyby.controller_move_sec 按厂商源码公式算，不是拿 delay_ms 当时长。
#
# 不用固定比例的原因：从「该发了」到「命令真出去」之间还有本进程自己的开销——读反馈、
# 算逆解、6 条帧按 2 ms 间隔排队。实测关节约 30 ms、笛卡尔 55~60 ms，随姿态还会变，
# 固定比例压不住。改成量出来的延迟自己补（指数平均），重叠就能稳定在目标值附近。
# 实机嫌顿就把这个数调大（更早发下一段、重叠更多、松手滑行更远）：启动参数 hold_overlap_ms。
HOLD_OVERLAP_MS = 60.0
HOLD_LAT_EMA = 0.3              # 发送延迟的指数平均系数
HOLD_LAG_FRAC = 2.5             # 臂落后超过本段行程的这个倍数就先等它，别把目标越拉越远
F1_MIN_DEG = 0.05               # 小于它的轴这一段不发（攒到下一段），上界就是它
HOLD_WATCHDOG_SEC = 0.25        # 2.5 个周期收不到心跳就停发（浏览器卡了 / 网络断了）
HOLD_JUMP_LIMIT_DEG = 10.0      # 单步任一关节超它就判失败停发（Thomas 定；实测正常 0.63°）
HOLD_JOINT_DEG_PER_S = 20.0     # 关节按住的基准速度，再乘速度缩放
HOLD_CART_MM_PER_S = 50.0       # 笛卡尔平移基准速度
HOLD_CART_DEG_PER_S = 20.0      # 笛卡尔姿态基准角速度
# 每条命令要求臂上花的运动时间（秒）→ delay_ms = round(T*1000/COMP)，COMP=1.27。
# 取一个周期：命令之间正好首尾相接，靠命令重叠连续追踪（S-ovl1 实测成立）。
HOLD_SEG_SEC = 1.0 / HOLD_HZ
HOLD_DELAY_MS = int(round(HOLD_SEG_SEC * 1000.0 / 1.27))
F1_GAP_SEC = 0.002              # 同一周期内各轴命令间隔，同 trajectory_server

TCPS = [
    {'id': 'grip_center', 'name': '夹持中心', 'link': 'tcp_link', 'enabled': True,
     'note': 'URDF tcp_joint，距 Empty_Link6 (96.6, -0.4, -0.4) mm'},
    {'id': 'flange', 'name': '法兰', 'link': 'Empty_Link6', 'enabled': True,
     'note': 'URDF 规划组末端 link'},
    {'id': 'fingertip', 'name': '指尖', 'link': 'fingertip_link', 'enabled': True,
     'note': '模型推算 x = 128.6 mm（S4b-4 报告 §4），未实物核实'},
    {'id': 'camera', 'name': '相机光心', 'link': None, 'enabled': False,
     'note': '等 VISION-01 验收；没有可用数值，不给占位数'},
]

SAFETY_TEXT = {
    'estop': '本界面没有急停按钮。cancel、Ctrl-C、停止发布都不是急停，'
             '唯一能立即停的是断电——手放在断电处。',
    'stream': '执行模式固定 stream：经由点没走到也会继续发下一点（只记日志，不中止）。'
              '靠近物体、障碍、下探时把步长调小、手放在断电处——界面不替你判断。',
    # Thomas 2026-09-23 裁定：任意姿态都能上电，原来「clamp 偏差 > 2° 拒绝」改成只告警。
    'servo_on': '折叠区（a1≈-90 且 a2≈90）内上电会自动规划回 HOME，并把夹爪送到约 20 mm 开口；'
                '折叠区外是零位移原地使能，但超出控制器软限位的轴会被截回限位，'
                '上电瞬间臂会突跳那么多（幅度见事件页 reason=ok_clamped_X.Xdeg）。',
    # Thomas 2026-09-23 裁定：任意姿态都能下电，arm_driver 改走「先阻尼再卸力」。
    'servo_off': '折叠区外下电：先发阻尼档（mode=1000）让臂带阻尼下坠约 2 s，再真正卸力。'
                 '臂一定会跌落——先确认臂下方已清空、手放在断电处。',
    # Thomas 2026-09-22 加：/set_servo 回 HOME 那段是 arm_driver 内部的梯形规划，
    # 不经 MoveIt 碰撞检查，界面也挡不住（按任务书伺服按钮只做服务外壳）。
    'servo_path': '上电前确认臂到 HOME 的路径上没有遮挡：这段回 HOME 是 arm_driver 内部规划，'
                  '不经 MoveIt 碰撞检查，界面挡不住。',
    # 刚度不是「保持力度」：2026-09-11 实测，满档 1000 也撑不住自重。
    'stiffness': '刚度档位 0~1000（funcode=5 全关节）。它只加阻力、不朝任何目标驱动，'
                 '满档 1000 也撑不住自重，不能用来保持位置。低于 1000 控制器判为没使能，'
                 '等同卸力：不在折叠区时 arm_driver 会拒（reason=not_folded）。',
    'grip_mm':'括号里的 mm 是空载估算：GRIP-01 实测 raw 33~38 段（夹住物体时）'
               '换算偏大 3.4 mm 以上，不可用于判断夹住没夹住。',
    # Thomas 2026-09-22 裁定：示教不加载桌面碰撞体（人机协同，障碍由操作员目视避免）。
    # 界面必须把「现在没有防护」显式写出来，不能让人以为 MoveIt 在替他看桌子。
    'scene_empty': '规划场景里没有世界碰撞体：MoveIt 只检查臂自碰撞与关节限位，'
                   '桌子、支架、瓶子一概不知道。障碍由你目视避免。',
    'scene_has': '规划场景里有世界碰撞体，MoveIt 会绕开它们；'
                 '臂一旦停进碰撞盒里，MoveIt 会拒绝一切规划（只能手发 /move_to_joints 挪出来）。',
}


# MoveIt 失败码 → 一句人话。示教器上只报 error_code=99999 没用，操作员要知道下一步怎么办。
# 码的含义见 manifest/OPERATIONS.md ·「I.3 MoveGroup 常见 error_code」。
ERR_HINT = {
    99999: '规划失败。最常见的原因是目标姿态臂会撞到自己'
           '（MoveIt 只检查臂自碰撞与关节限位）——换个中间姿态分两步走。'
           '折叠姿态本身在 MoveIt 眼里就是自碰撞状态（Empty_Link1 贴 Empty_Link6），'
           '从折叠姿态出发时先把 a2 打开再动 a1 往往就能过。',
    -1: '规划失败：规划器没找到路径。同上，先换中间姿态分步走。',
    -2: '规划结果无效（通常是起点在限位外或姿态奇异）。换个起点再试。',
    -4: '执行器拒绝或执行失败：看 trajectory_server 与 arm_driver 的日志。',
    -10: '起点就判碰撞：臂现在的姿态 MoveIt 认为自碰撞，先手动点动挪开。',
    -12: '目标点判碰撞：换目标。',
}


def _err_names():
    return {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
            if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}


def _allowed_bind(host):
    """只允许本机回环或直连网段 192.168.123.0/24。一个网页按钮能让真臂动，权限面必须收窄。"""
    if host in ('127.0.0.1', 'localhost', '::1'):
        return True
    parts = host.split('.')
    return len(parts) == 4 and parts[:3] == ['192', '168', '123'] and parts[3].isdigit()


class TeachServer(Node):

    def _float_param(self, name, default):
        """声明一个浮点参数，但命令行写 `-p x:=1` 这种整数也收。

        rclpy 默认按「默认值的类型」锁死参数类型：默认是 1.0（DOUBLE），命令行传 60
        就是 INTEGER，直接抛 InvalidParameterTypeException 起不来。实机现场谁都会
        顺手少打那个小数点（Thomas 2026-09-23 就这么被挡了一次），所以关掉类型锁、
        自己转 float。
        """
        desc = ParameterDescriptor(dynamic_typing=True)
        return float(self.declare_parameter(name, float(default), desc).value)

    def __init__(self):
        super().__init__('d1t_teach')
        self.cbg = ReentrantCallbackGroup()
        self.host = str(self.declare_parameter('bind_host', '127.0.0.1').value)
        self.port = int(self.declare_parameter('bind_port', 8175).value)
        if not _allowed_bind(self.host):
            raise ValueError(
                'bind_host=%r 不在允许范围：只能绑本机回环或直连网段 192.168.123.x。'
                '从别的机器用，请走 SSH 端口转发。' % self.host)
        # max_scaling：速度 / 加速度缩放的上限，默认 1.0 = MoveIt 关节限速的 100%
        # （J1~J3 60.2 °/s、J4~J6 99.1 °/s，OPERATIONS.md H.2）。
        # 任务书原来写死 0.3（S4b-3 / S4b-4 实机连跑用值），Thomas 2026-09-23 裁定
        # 取消「所有臂轴运动必须经 MoveIt 规划并通过碰撞检查」并要求界面直接调这两个值，
        # 上限随之放到 1.0；要压回去就启动时传 -p max_scaling:=0.3。
        self.max_scaling = self._float_param('max_scaling', 1.0)
        if not (0.0 < self.max_scaling <= 1.0):
            raise ValueError('max_scaling=%s 超出 (0, 1.0]' % self.max_scaling)
        self.vel = self._float_param('vel_scaling', units.MAX_SCALING)
        self.acc = self._float_param('acc_scaling', units.MAX_SCALING)
        err = (units.check_scaling(self.vel, self.max_scaling)
               or units.check_scaling(self.acc, self.max_scaling))
        if err:
            raise ValueError('%s（上限由 max_scaling 参数给，现在是 %s）'
                             % (err, self.max_scaling))
        self.presets_path = str(self.declare_parameter(
            'presets_path', presets.DEFAULT_PATH).value)
        # launch 传进来的是字符串 'false'，bool('false') 是 True，所以字符串单独解析
        raw_manage = self.declare_parameter('manage_trajectory_server', True).value
        self.manage_traj = (raw_manage if isinstance(raw_manage, bool)
                            else str(raw_manage).strip().lower() in ('1', 'true', 'yes'))
        self.csv_dir = str(self.declare_parameter('csv_dir', '/tmp').value)
        # 按住即走的两个硬件相关旋钮，见 HOLD_HZ 上方的说明
        # hold_hz 现在只定「名义段长 = 1/hold_hz」，实际什么时候发下一段由「上一段进区域」
        # 决定；hold_delay_ms 也不再固定，每段按自己的时长算（见 _hold_loop）。两个参数留着
        # 是为了实机上能不改代码地调：臂跟不上就把 hold_hz 调小（段更长、发得更稀）。
        self.hold_hz = self._float_param('hold_hz', HOLD_HZ)
        if not (1.0 <= self.hold_hz <= 50.0):
            raise ValueError('hold_hz=%s 超出 [1, 50]' % self.hold_hz)
        self.hold_max_seg = max(HOLD_MIN_SEG_SEC, min(1.0 / self.hold_hz,
                                                      HOLD_MAX_LOOKAHEAD_SEC))
        self.hold_delay_ms = int(self.declare_parameter(
            'hold_delay_ms', HOLD_DELAY_MS).value)
        if not (1 <= self.hold_delay_ms <= 32767):
            raise ValueError('hold_delay_ms=%s 超出 [1, 32767]' % self.hold_delay_ms)
        self.hold_overlap_ms = self._float_param('hold_overlap_ms', HOLD_OVERLAP_MS)
        if not (0.0 <= self.hold_overlap_ms <= 500.0):
            raise ValueError('hold_overlap_ms=%s 超出 [0, 500]' % self.hold_overlap_ms)
        self.traj_zone_deg = self._float_param('zone_deg', 2.0)
        self.traj_max_points = int(self.declare_parameter('max_points', 400).value)
        self.err_names = _err_names()
        self.lock = threading.Lock()
        self.data = presets.load(self.presets_path)
        self.state = {
            'fb_deg7': None, 'fb_time': None, 'fb_count': 0,
            'busy': False, 'busy_label': '', 'exec_in_flight': False,
            'external_until': 0.0, 'external_why': '', 'settle_until': 0.0,
            'tcp': TCPS[0]['id'], 'frame': 'tool',
            'override': 1.0, 'pose': None, 'tip_pose': None, 'pose_err': '',
            'stiffness': None,
            'events': [], 'last_result': None, 'rec': None, 'scene_objects': None,
            'hold': None,
        }
        self._own_goal_ids = set()
        self._idle_ref = None       # (时刻, 6 轴角度)：空闲净位移的比较基准
        self._hold_pub = None       # /arm_Command 发布者，只在按住期间存在（见 _hold_loop）
        self._hold_seq = 90000      # 与 arm_driver / trajectory_server 的 seq 分开
        self._cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._fb_window = []
        self._last_idle_deg = None
        self._cmd_lock = threading.Lock()
        self._traj_proc = None

        self.create_subscription(JointFeedback, '/d1t/joint_feedback',
                                 self._on_feedback, 10, callback_group=self.cbg)
        self.create_subscription(GoalStatusArray, '/move_to_joints/_action/status',
                                 self._on_goal_status, 10, callback_group=self.cbg)
        self.fk_cli = self.create_client(GetPositionFK, '/compute_fk', callback_group=self.cbg)
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path',
                                           callback_group=self.cbg)
        self.scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene',
                                            callback_group=self.cbg)
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cbg)
        self.grip_cli = self.create_client(Grip, '/grip', callback_group=self.cbg)
        self.servo_cli = self.create_client(SetServo, '/set_servo', callback_group=self.cbg)
        self.stiff_cli = self.create_client(SetStiffness, '/set_stiffness',
                                            callback_group=self.cbg)
        self.override_cli = self.create_client(SetOverride, '/set_override',
                                               callback_group=self.cbg)
        self.move_ac = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cbg)
        self.exec_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory',
                                    callback_group=self.cbg)

        self.create_timer(0.3, self._tick_pose, callback_group=self.cbg)
        self.create_timer(2.0, self._tick_publishers, callback_group=self.cbg)
        self.create_timer(2.0, self._tick_scene, callback_group=self.cbg)

        self.web_dir = os.path.join(get_package_share_directory('d1t_teach'), 'web')
        self.httpd = None
        self._log_event('info', '后端启动：%s' % SAFETY_TEXT['estop'], True)

    # ---------- 反馈与外部运动 ----------

    def _on_feedback(self, msg):
        deg7 = [msg.angle0, msg.angle1, msg.angle2, msg.angle3,
                msg.angle4, msg.angle5, msg.angle6_raw]
        now = time.monotonic()
        with self.lock:
            self.state['fb_deg7'] = deg7
            self.state['fb_time'] = now
            self.state['fb_count'] += 1
            if self.state['rec'] is not None:
                self.state['rec'].append((now, list(deg7)))
            self._fb_window.append(now)
            if self.state['busy'] or self._settling():
                self._idle_ref = None
            else:
                ref = self._idle_ref
                verdict = gating.idle_drift_verdict(
                    None if ref is None else ref[1], deg7[:6],
                    0.0 if ref is None else now - ref[0],
                    EXTERNAL_WIN_SEC, EXTERNAL_NET_DEG)
                if verdict != 'keep':
                    self._idle_ref = (now, list(deg7[:6]))
                if verdict == 'flag':
                    net = max(abs(deg7[i] - ref[1][i]) for i in range(6))
                    self._flag_external('空闲时关节在走（%.1f s 内净走 %s°），不是本界面发的'
                                        % (now - ref[0], units.fmt1(net)))

    def _on_goal_status(self, msg):
        active = [bytes(s.goal_info.goal_id.uuid) for s in msg.status_list if s.status in (1, 2)]
        with self.lock:
            if self.state['busy']:
                # 本界面在跑：point / stream 模式下这些目标是 trajectory_server 替我们发的
                if len(self._own_goal_ids) > OWN_GOAL_ID_CAP:
                    self._own_goal_ids.clear()
                self._own_goal_ids.update(active)
                return
            if any(gid not in self._own_goal_ids for gid in active):
                self._flag_external('/move_to_joints 上有别人的目标在跑')

    def _tick_scene(self):
        """查一次规划场景里有哪些世界碰撞体，显示在界面上。

        示教默认不加载桌面碰撞体（Thomas 2026-09-22 裁定），所以这一行通常是「没有」。
        它存在的意义是：不让人误以为 MoveIt 在替他看桌子；以及臂真停进碰撞盒、
        MoveIt 开始拒绝一切规划时，一眼能看出是被哪个碰撞体挡住的。
        """
        if not self.scene_cli.service_is_ready():
            with self.lock:
                self.state['scene_objects'] = None
            return
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(
            components=PlanningSceneComponents.WORLD_OBJECT_NAMES)
        fut = self.scene_cli.call_async(req)
        fut.add_done_callback(self._on_scene)

    def _on_scene(self, fut):
        res = fut.result()
        with self.lock:
            self.state['scene_objects'] = (
                None if res is None
                else sorted(o.id for o in res.scene.world.collision_objects))

    def _tick_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic('/arm_Command')
        except Exception:
            return
        extra = gating.external_publisher_names([i.node_name for i in infos])
        if extra:
            with self.lock:
                self._flag_external('/arm_Command 上有外来发布者：%s' % '、'.join(extra))

    def _flag_external(self, why):
        """调用方必须已经持有 self.lock。"""
        fresh = not self._external_now()   # 只在从「没有」变成「有」时记一条
        self.state['external_until'] = time.monotonic() + EXTERNAL_HOLD_SEC
        self.state['external_why'] = why
        if fresh:
            self._log_event('warn', why, False, locked=True)

    def _settling(self):
        """调用方必须已经持有 self.lock。"""
        return time.monotonic() < self.state['settle_until']

    def _external_now(self):
        return time.monotonic() < self.state['external_until']

    def _fb_hz(self):
        """最近 3 s 的到帧率。在 snapshot 里现算，反馈一断就会自己掉下去，
        不会停在断开前的那个数（调用方必须已经持有 self.lock）。"""
        now = time.monotonic()
        while self._fb_window and now - self._fb_window[0] > 3.0:
            self._fb_window.pop(0)
        if len(self._fb_window) < 2:
            return 0.0
        span = self._fb_window[-1] - self._fb_window[0]
        return (len(self._fb_window) - 1) / span if span > 0 else 0.0

    def _fb_age(self):
        t = self.state['fb_time']
        return None if t is None else time.monotonic() - t

    # ---------- 位姿显示 ----------

    def _tick_pose(self):
        with self.lock:
            deg7 = self.state['fb_deg7']
            tcp = self._tcp_by_id(self.state['tcp'])
            stale = gating.feedback_is_stale(self._fb_age(), units.FEEDBACK_STALE_SEC)
        if deg7 is None or stale or tcp['link'] is None or not self.fk_cli.service_is_ready():
            with self.lock:
                self.state['pose'] = None
                self.state['tip_pose'] = None
                self.state['pose_err'] = '反馈已断' if stale else '/compute_fk 未就绪'
            return
        links = [tcp['link']] if tcp['link'] == IK_TIP_LINK else [tcp['link'], IK_TIP_LINK]
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.fk_link_names = links
        req.robot_state = RobotState(joint_state=JointState(
            name=JOINTS, position=units.controller_deg_to_urdf_rad(deg7[:6])))
        fut = self.fk_cli.call_async(req)
        fut.add_done_callback(lambda f, ls=links: self._on_fk(f, ls))

    @staticmethod
    def _pose_of(msg):
        p = msg.pose
        return ([p.position.x, p.position.y, p.position.z],
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def _on_fk(self, fut, links):
        res = fut.result()
        with self.lock:
            if (res is None or res.error_code.val != MoveItErrorCodes.SUCCESS
                    or len(res.pose_stamped) != len(links)):
                self.state['pose'] = None
                self.state['tip_pose'] = None
                self.state['pose_err'] = '/compute_fk 失败'
                return
            poses = dict(zip(links, [self._pose_of(m) for m in res.pose_stamped]))
            p, q = poses[links[0]]
            r, pitch, y = units.quat_to_rpy(q)
            self.state['pose'] = {
                'link': links[0], 'frame': BASE_FRAME,
                'x_mm': units.m2mm(p[0]), 'y_mm': units.m2mm(p[1]), 'z_mm': units.m2mm(p[2]),
                'rx_deg': units.rad2deg(r), 'ry_deg': units.rad2deg(pitch),
                'rz_deg': units.rad2deg(y),
                'sel_pose': (p, q)}
            self.state['tip_pose'] = poses[IK_TIP_LINK]
            self.state['pose_err'] = ''

    def _current_pose(self):
        """选中 TCP 与规划组末端 link 的当前位姿（米、四元数）。取不到返回 (None, None)。"""
        with self.lock:
            pose, tip = self.state['pose'], self.state['tip_pose']
        if pose is None or tip is None:
            return None, None
        return pose['sel_pose'], tip

    # ---------- 事件日志 ----------

    def _log_event(self, level, text, ok, locked=False):
        entry = {'t': time.strftime('%H:%M:%S'), 'level': level, 'text': text, 'ok': bool(ok)}

        def append():
            self.state['events'].append(entry)
            del self.state['events'][:-40]
        if locked:
            append()
        else:
            with self.lock:
                append()
        if ok:
            self.get_logger().info('TEACH %s' % text)
        else:
            self.get_logger().warn('TEACH %s' % text)
        return entry

    # ---------- 依赖与状态 ----------

    def _missing_deps(self):
        missing = []
        for name, ready in (('/compute_fk', self.fk_cli.service_is_ready()),
                            ('/compute_cartesian_path', self.cart_cli.service_is_ready()),
                            ('/move_action', self.move_ac.server_is_ready()),
                            ('/execute_trajectory', self.exec_ac.server_is_ready())):
            if not ready:
                missing.append(name)
        return missing

    def _tcp_by_id(self, tcp_id):
        for tcp in TCPS:
            if tcp['id'] == tcp_id:
                return tcp
        return TCPS[0]

    def _gate(self, kind):
        with self.lock:
            missing = self._missing_deps()
            if kind == gating.GRIP and not self.grip_cli.service_is_ready():
                missing = missing + ['/grip']
            return gating.check_command(
                kind,
                busy=self.state['busy'],
                feedback_stale=gating.feedback_is_stale(self._fb_age(),
                                                        units.FEEDBACK_STALE_SEC),
                external_motion=self._external_now(),
                missing_deps=missing if kind == gating.MOTION else
                (['/grip'] if kind == gating.GRIP and '/grip' in missing else []),
                exec_in_flight=self.state['exec_in_flight'])

    def snapshot(self):
        with self.lock:
            deg7 = self.state['fb_deg7']
            age = self._fb_age()
            stale = gating.feedback_is_stale(age, units.FEEDBACK_STALE_SEC)
            margins = None
            if deg7 is not None:
                margins = [{'axis': units.AXIS_NAMES[i],
                            'deg': units.round1(deg7[i]),
                            'to_min': units.round1(m[0]), 'to_max': units.round1(m[1])}
                           for i, m in enumerate(units.limit_margins(deg7[:6]))]
            pose = self.state['pose']
            out = {
                'ok': True,
                'feedback': {
                    'stale': stale,
                    'age_s': None if age is None else round(age, 2),
                    'hz': round(self._fb_hz(), 2),
                    'joints_deg': None if deg7 is None else [units.round1(v) for v in deg7[:6]],
                    'grip_raw': None if deg7 is None else units.round1(deg7[6]),
                    'grip_gap_mm': None if deg7 is None else units.round1(
                        units.raw_to_gap_mm(deg7[6])),
                    'margins': margins,
                },
                'pose': None if pose is None else {
                    k: (units.round1(v) if k.endswith('_mm') or k.endswith('_deg') else v)
                    for k, v in pose.items() if k != 'sel_pose'},
                'pose_err': self.state['pose_err'],
                'busy': self.state['busy'], 'busy_label': self.state['busy_label'],
                'external': self._external_now(), 'external_why': self.state['external_why'],
                'holding': self.state['hold'] is not None,
                'hold_hz': self.hold_hz, 'hold_delay_ms': self.hold_delay_ms,
                'hold_overlap_ms': self.hold_overlap_ms,
                'hold_steps': (self.state['hold'] or {}).get('steps', 0),
                'settling': self._settling(),
                'missing_deps': self._missing_deps(),
                'tcp': self.state['tcp'], 'frame': self.state['frame'],
                'exec_mode': EXEC_MODE,
                'traj_managed': self._traj_proc is not None,
                'exec_mode_note': '' if self._traj_proc is not None
                                  else 'trajectory_server 不是本界面起的，它是什么模式本界面不知道；'
                                       '界面按 stream 提示',
                'override': self.state['override'],
                'stiffness': self.state['stiffness'],
                'stiffness_limits': [units.STIFFNESS_MIN, units.STIFFNESS_MAX,
                                     units.STIFFNESS_HOLD_MIN],
                'vel': self.vel, 'acc': self.acc, 'max_scaling': self.max_scaling,
                'grip_values': dict(self.data['grip']),
                'presets': presets.list_presets(self.data),
                'tcps': [{k: v for k, v in t.items()} for t in TCPS],
                'events': list(self.state['events'][-20:]),
                'last_result': self.state['last_result'],
                'limits': {'joint': units.UI_LIMIT_DEG,
                           'grip': [units.GRIP_CMD_MIN, units.GRIP_CMD_MAX],
                           'max_scaling': units.MAX_SCALING},
                'steps': {'joint': units.JOINT_STEPS_DEG, 'pos': units.POS_STEPS_MM,
                          'rot': units.ROT_STEPS_DEG},
                'scene_objects': self.state['scene_objects'],
                'safety': SAFETY_TEXT,
                'presets_path': self.presets_path,
            }
        return out

    # ---------- 执行：都在单独线程里跑，一次只有一条 ----------

    def _run_async(self, label, kind, fn):
        """接受命令 → 起线程执行。返回 (accepted, reason)。"""
        reason = self._gate(kind)
        if reason:
            with self.lock:
                self._log_event('reject', '%s：%s' % (label, reason), False, locked=True)
            return False, reason
        if not self._cmd_lock.acquire(blocking=False):
            return False, '上一条动作还没结束，不接受新命令'
        with self.lock:
            if self.state['busy']:
                self._cmd_lock.release()
                return False, '上一条动作还没结束，不接受新命令'
            self.state['busy'] = True
            self.state['busy_label'] = label
            self.state['last_result'] = None

        def worker():
            try:
                try:
                    result = fn()
                except Exception as exc:                  # noqa: BLE001 - 兜底不让线程静默死
                    result = {'ok': False, 'text': '%s 异常：%s' % (label, exc)}
                with self.lock:
                    self.state['busy'] = False
                    self.state['busy_label'] = ''
                    self.state['exec_in_flight'] = False
                    self.state['last_result'] = result
                    self._log_event('done' if result.get('ok') else 'fail',
                                    result.get('text', label), result.get('ok'), locked=True)
            finally:
                with self.lock:
                    self.state['busy'] = False
                    self.state['busy_label'] = ''
                    self.state['settle_until'] = time.monotonic() + (
                        SETTLE_SERVICE_SEC if kind == gating.SERVICE else SETTLE_MOTION_SEC)
                self._cmd_lock.release()

        threading.Thread(target=worker, name='d1t_teach_cmd', daemon=True).start()
        return True, ''

    def _start_rec(self):
        with self.lock:
            self.state['rec'] = []

    def _finish_rec(self, label):
        """落盘一份反馈 CSV，表头与 s4b3_clearance / s4b4_tcp 的输入一致，
        可以直接喂给 s4b3_clearance 逐帧算最近距离。"""
        with self.lock:
            rows, self.state['rec'] = self.state['rec'] or [], None
        if not rows:
            return '；反馈 CSV frames=0（没录到帧）'
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(self.csv_dir, 'teach01_fb_%s.csv' % stamp)
        t0 = rows[0][0]
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write('t_rel_s,' + ','.join('a%d' % i for i in range(7)) + '\n')
                for t, deg7 in rows:
                    f.write('%.4f,' % (t - t0) + ','.join('%.3f' % v for v in deg7) + '\n')
        except OSError as exc:
            return '；反馈 CSV 写不出来：%s' % exc
        return '；反馈 CSV frames=%d %s' % (len(rows), path)

    def _wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end and rclpy.ok():
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _move_joint_target(self, label, deg6, note=''):
        err = units.check_joint_target(deg6)
        if err:
            return {'ok': False, 'text': '%s：%s；不下发任何命令' % (label, err)}
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.acc
        c = Constraints()
        for name, pos in zip(JOINTS, units.controller_deg_to_urdf_rad(deg6)):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=pos, tolerance_above=0.01,
                tolerance_below=0.01, weight=1.0))
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False
        t0 = time.monotonic()
        send = self.move_ac.send_goal_async(goal)
        handle = self._wait(send, 15.0)
        if handle is None or not handle.accepted:
            return {'ok': False, 'text': '%s：/move_action 未接受目标' % label}
        with self.lock:
            self.state['exec_in_flight'] = True
        self._start_rec()
        res = self._wait(handle.get_result_async(), GOAL_TIMEOUT_SEC)
        note = note + self._finish_rec(label)
        if res is None:
            return {'ok': False, 'text': '%s：%.0fs 无结果（取消不是急停，未发取消）'
                                         % (label, GOAL_TIMEOUT_SEC)}
        code = res.result.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        return {'ok': ok,
                'text': '%s：%s error_code=%d(%s) 目标 a0~a5=%s 耗时 %.2fs 模式=%s%s%s'
                        % (label, 'OK' if ok else 'FAILED', code,
                           self.err_names.get(code, '?'),
                           [units.fmt1(v) for v in deg6], time.monotonic() - t0,
                           EXEC_MODE, note,
                           '' if ok else '；' + ERR_HINT.get(code, '原因看 move_group 日志')),
                'elapsed_s': round(time.monotonic() - t0, 2)}

    def _move_cartesian(self, label, p_new, q_new, link):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.start_state = RobotState(is_diff=True)
        req.group_name = GROUP
        req.link_name = link
        target = Pose()
        target.position.x, target.position.y, target.position.z = (float(v) for v in p_new)
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = q_new
        req.waypoints = [target]
        req.max_step = CART_MAX_STEP_M
        req.jump_threshold = CART_JUMP_THRESHOLD
        req.avoid_collisions = True
        t0 = time.monotonic()
        res = self._wait(self.cart_cli.call_async(req), 20.0)
        if res is None:
            return {'ok': False, 'text': '%s：/compute_cartesian_path 无响应，臂不动' % label}
        jt = res.solution.joint_trajectory
        if res.fraction < MIN_FRACTION or not jt.points:
            return {'ok': False,
                    'text': '%s：规划覆盖率 %.4f < %.3f，不执行部分路径，臂不动'
                            % (label, res.fraction, MIN_FRACTION)}
        # /compute_cartesian_path（Humble）不收缩放参数，返回按 100% 计时；这里拉长时间
        # 让速度 ×k、加速度 ×k²，与 s4b4_tcp.time_scale_factor 同一算法。
        k = min(self.vel, math.sqrt(self.acc))
        n_raw = len(jt.points)
        keep = (list(range(n_raw)) if n_raw <= CART_MAX_POINTS else
                sorted({round(i * (n_raw - 1) / (CART_MAX_POINTS - 1))
                        for i in range(CART_MAX_POINTS)}))
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
        send = self.exec_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=res.solution))
        handle = self._wait(send, 15.0)
        if handle is None or not handle.accepted:
            return {'ok': False, 'text': '%s：/execute_trajectory 未接受目标，臂不动' % label}
        with self.lock:
            self.state['exec_in_flight'] = True
        self._start_rec()
        eres = self._wait(handle.get_result_async(), GOAL_TIMEOUT_SEC)
        csv_note = self._finish_rec(label)
        if eres is None:
            return {'ok': False, 'text': '%s：%.0fs 无结果' % (label, GOAL_TIMEOUT_SEC)}
        code = eres.result.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        return {'ok': ok,
                'text': '%s：%s error_code=%d(%s) fraction=%.4f points=%d→%d 耗时 %.2fs 模式=%s%s'
                        % (label, 'OK' if ok else 'FAILED', code,
                           self.err_names.get(code, '?'), res.fraction, n_raw, len(pts),
                           time.monotonic() - t0, EXEC_MODE, csv_note),
                'elapsed_s': round(time.monotonic() - t0, 2)}

    # ---------- 各个命令 ----------

    @staticmethod
    def _hold_target(deg7):
        """其余轴「保持不动」的目标：当前读数钳进控制器限位（见 units.clamp_to_limit）。"""
        target, notes = units.clamp_to_limit(deg7[:6])
        note = ''
        if notes:
            note = '；保持轴读数在限位外、已钳回：' + '，'.join(
                '%s %s→%s' % n for n in notes)
        return target, note

    def cmd_jog_joint(self, axis, delta_deg):
        with self.lock:
            deg7 = self.state['fb_deg7']
        if deg7 is None:
            return False, '还没收到反馈'
        target, note = self._hold_target(deg7)
        target[axis] = units.round1(deg7[axis] + float(delta_deg))
        err = units.check_joint_deg(axis, target[axis])
        if err:
            return False, err
        label = '关节点动 %s %+.1f°' % (units.AXIS_NAMES[axis], float(delta_deg))
        return self._run_async(label, gating.MOTION,
                               lambda: self._move_joint_target(label, target, note))

    def cmd_goto_joint(self, axis, deg):
        err = units.check_joint_deg(axis, deg)
        if err:
            return False, err
        with self.lock:
            deg7 = self.state['fb_deg7']
        if deg7 is None:
            return False, '还没收到反馈'
        target, note = self._hold_target(deg7)
        target[axis] = units.round1(deg)
        label = '绝对定位 %s → %s°' % (units.AXIS_NAMES[axis], units.fmt1(deg))
        return self._run_async(label, gating.MOTION,
                               lambda: self._move_joint_target(label, target, note))

    def cmd_jog_cart(self, frame, axis, value):
        err = units.check_cart_frame(frame) or units.check_cart_axis(axis)
        if err:
            return False, err
        with self.lock:
            tcp = self._tcp_by_id(self.state['tcp'])
        if tcp['link'] is None:
            return False, '当前 TCP 不可用'
        sel_now, tip_now = self._current_pose()
        if sel_now is None:
            return False, '当前 TCP 位姿未知（反馈或 /compute_fk 未就绪）'
        sel_target = units.cart_target(frame, axis, value, sel_now[0], sel_now[1])
        # 逆解只对规划组末端 link 求（见 units.retarget_to_link）
        tip_p, tip_q = units.retarget_to_link(sel_now, tip_now, sel_target)
        unit = 'mm' if axis in ('x', 'y', 'z') else '°'
        label = '笛卡尔 %s系 %s %+.1f%s（TCP=%s）' % (
            '工具' if frame == 'tool' else '基座', axis, float(value), unit, tcp['name'])
        return self._run_async(
            label, gating.MOTION,
            lambda: self._move_cartesian(label, tip_p, tip_q, IK_TIP_LINK))

    def cmd_preset(self, preset_id, confirm=False):
        with self.lock:
            preset = presets.find_preset(self.data, preset_id)
            deg7 = self.state['fb_deg7']
        if preset is None:
            return False, '预设 %s 不存在' % preset_id
        if preset['empty']:
            return False, '预设「%s」还没记录过位置' % preset['name']
        if preset.get('confirm') and not confirm:
            return False, ('预设「%s」是整臂大幅度运动，要二次确认才下发：%s'
                           % (preset['name'], preset['note']))
        if (preset_id == ZERO_PRESET_ID and ZERO_REQUIRES_FOLD_START
                and deg7 is not None and not units.in_fold_zone(deg7[:6])):
            return False, ('0 点只放行「折叠姿态 → 0 点」这一条（主窗口 2026-09-22 W4·2d）；'
                           '当前 a1=%s a2=%s 不在折叠区（a1≈-90、a2≈90，±15°）。'
                           '先点「折叠姿态」，首次实跑回报后由主窗口放宽起点'
                           % (units.fmt1(deg7[1]), units.fmt1(deg7[2])))
        label = '预设点位「%s」' % preset['name']
        grip = preset.get('grip')
        pose = preset.get('pose') if preset.get('kind') == 'cart' else None
        if pose is not None:
            with self.lock:
                cur_tcp = self.state['tcp']
            if pose['tcp'] != cur_tcp:
                return False, ('预设「%s」记的是 %s 的位姿，当前选的是 %s。'
                               '先在笛卡尔页把 TCP 切回 %s 再到达'
                               % (preset['name'], pose['tcp'], cur_tcp, pose['tcp']))

        def run():
            deg6 = preset['deg6']
            if pose is not None:
                deg6, err = self._ik_from_recorded_pose(pose, preset['deg6'])
                if err:
                    return {'ok': False, 'text': '%s：%s；不下发任何命令' % (label, err)}
            result = self._move_joint_target(label, deg6)
            if not result['ok'] or grip is None:
                return result
            if units.check_grip_raw(grip) is not None:
                result['text'] += '；夹爪记录值 %s 不在命令域 [20, 50]，没下发夹爪命令' \
                                  % units.fmt1(grip)
                return result
            grip_res = self._call_grip(grip)
            result['ok'] = result['ok'] and grip_res['ok']
            result['text'] += '；' + grip_res['text']
            return result

        return self._run_async(label, gating.MOTION, run)

    def _ik_from_recorded_pose(self, pose, seed_deg6):
        """笛卡尔槽位：把记下的 TCP 位姿解成关节目标。返回 (deg6, 错误原因)。

        种子用记录时的关节值，不是当前关节值——同一位姿可以有多支解，用当初那组
        种子才会解回当初那一支，避免到达时臂整个翻过去。
        逆解判碰撞（avoid_collisions=True）：这是经 MoveIt 规划的整臂运动，
        与「按住即走」那条不检查的通路不是一回事。
        """
        sel_now, tip_now = self._current_pose()
        if sel_now is None:
            return None, '算不出当前位姿（/compute_fk 没结果），无法换算逆解末端'
        target = ([float(v) for v in pose['p_m']], tuple(float(v) for v in pose['q']))
        tip_target = units.retarget_to_link(sel_now, tip_now, target)
        seed = units.controller_deg_to_urdf_rad(seed_deg6)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = IK_TIP_LINK
        r.avoid_collisions = True
        r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=seed))
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) for v in tip_target[0])
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = tip_target[1]
        r.pose_stamped = ps
        r.timeout.nanosec = int(0.2 * 1e9)
        res = self._wait(self.ik_cli.call_async(req), SERVICE_TIMEOUT_SEC)
        if res is None:
            return None, '/compute_ik 无响应'
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, ('这个位姿解不出来（/compute_ik 返回 %s）'
                          % self.err_names.get(res.error_code.val, res.error_code.val))
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        deg6 = [units.round1(v) for v in units.urdf_rad_to_controller_deg(
            [sol[j] for j in JOINTS])]
        err = units.check_joint_target(deg6)
        if err:
            return None, '逆解结果超限：%s' % err
        return deg6, None

    def _call_grip(self, target_raw):
        err = units.check_grip_raw(target_raw)
        if err:
            return {'ok': False, 'text': '夹爪：%s；不下发命令' % err}
        req = Grip.Request(target_deg=float(target_raw), speed_ratio=self.vel, timeout_sec=0.0)
        res = self._wait(self.grip_cli.call_async(req), SERVICE_TIMEOUT_SEC)
        if res is None:
            return {'ok': False, 'text': '夹爪 target=%s：/grip 无响应' % units.fmt1(target_raw)}
        return {'ok': bool(res.success),
                'text': '夹爪 target=%s：success=%s state=%s reason=%s '
                        'start_raw=%s settled_raw=%s err_deg=%s'
                        % (units.fmt1(target_raw), res.success, res.state, res.reason,
                           units.fmt1(res.start_raw), units.fmt1(res.settled_raw),
                           units.fmt1(res.err_deg)),
                'grip': {'state': res.state, 'reason': res.reason,
                         'target': units.round1(target_raw),
                         'start_raw': units.round1(res.start_raw),
                         'settled_raw': units.round1(res.settled_raw),
                         'err_deg': units.round1(res.err_deg)}}

    def cmd_grip(self, which):
        with self.lock:
            values = dict(self.data['grip'])
        if which not in ('open', 'close'):
            return False, '夹爪命令只能是 open 或 close'
        target = values['open_raw'] if which == 'open' else values['close_raw']
        label = '夹爪%s' % ('开' if which == 'open' else '关')
        return self._run_async(label, gating.GRIP,
                               lambda: dict(self._call_grip(target),
                                            **{'label': label}))

    def cmd_servo(self, enable):
        label = '伺服%s' % ('上电' if enable else '下电')

        def run():
            res = self._wait(self.servo_cli.call_async(SetServo.Request(enable=bool(enable))),
                             SERVICE_TIMEOUT_SEC)
            if res is None:
                return {'ok': False, 'text': '%s：/set_servo 无响应' % label}
            return {'ok': bool(res.success),
                    'text': '%s：success=%s enabled=%s reason=%s final_deg=%s'
                            % (label, res.success, res.enabled, res.reason,
                               [units.fmt1(v) for v in res.final_deg]),
                    'servo': {'reason': res.reason, 'success': bool(res.success),
                              'enabled': bool(res.enabled)}}

        if not self.servo_cli.service_is_ready():
            return False, '依赖未就绪：/set_servo'
        return self._run_async(label, gating.SERVICE, run)

    def cmd_stiffness(self, mode):
        """/set_stiffness：发一帧 funcode=5 全关节刚度档位。

        这不是「保持力度」旋钮：实测 1000（满档）也撑不住自重，阻尼只加阻力、
        不朝任何目标驱动。低于 1000 控制器直接判为没使能，arm_driver 会按卸力处理
        （不在折叠区就拒绝）。界面把这两条都写在确认框里，不替人做判断。
        """
        err = units.check_stiffness(mode)
        if err:
            return False, err
        mode = int(mode)
        label = '刚度档位 → %d' % mode

        def run():
            res = self._wait(self.stiff_cli.call_async(SetStiffness.Request(mode=mode)),
                             SERVICE_TIMEOUT_SEC)
            if res is None:
                return {'ok': False, 'text': '%s：/set_stiffness 无响应' % label}
            if res.success:
                with self.lock:
                    self.state['stiffness'] = int(res.applied_mode)
            hint = ''
            if res.reason == 'not_folded':
                hint = ('；低于 %d 控制器判为卸力，不在折叠区会砸下来，所以被拒。'
                        '要在展开姿态卸力，用伺服「下电」——那条会先加阻尼再卸力'
                        % units.STIFFNESS_HOLD_MIN)
            return {'ok': bool(res.success),
                    'text': '%s：success=%s applied=%s enabled=%s reason=%s%s'
                            % (label, res.success, res.applied_mode, res.enabled,
                               res.reason, hint),
                    'stiffness': {'mode': int(res.applied_mode), 'reason': res.reason,
                                  'success': bool(res.success)}}

        if not self.stiff_cli.service_is_ready():
            return False, '依赖未就绪：/set_stiffness'
        return self._run_async(label, gating.SERVICE, run)

    def cmd_override(self, ratio):
        err = units.check_override(ratio)
        if err:
            return False, err
        label = '倍率 → %.2f' % float(ratio)

        def run():
            res = self._wait(
                self.override_cli.call_async(SetOverride.Request(ratio=float(ratio))),
                SERVICE_TIMEOUT_SEC)
            if res is None:
                return {'ok': False, 'text': '%s：/set_override 无响应' % label}
            with self.lock:
                self.state['override'] = float(res.applied_ratio)
            return {'ok': True, 'text': '%s：applied_ratio=%.2f' % (label, res.applied_ratio)}

        if not self.override_cli.service_is_ready():
            return False, '依赖未就绪：/set_override'
        return self._run_async(label, gating.SERVICE, run)


    # ---------- 按住即走 ----------

    def _hold_rates(self):
        """按住时的速度（每秒）：基准速度 × 速度缩放。

        步长不再是「每周期固定一段」，而是 速度 × 距上一段发出的实际间隔——
        发段由「臂进区域」触发，间隔是变的；按固定段长算，臂到得快就等于加速，
        实测仿真里跑成了设定值的两倍。
        """
        return (HOLD_JOINT_DEG_PER_S * self.vel,
                HOLD_CART_MM_PER_S * self.vel,
                HOLD_CART_DEG_PER_S * self.vel)

    def cmd_hold(self, kind, axis, direction, frame=None):
        """按住：浏览器按下后按周期反复打这个接口，每次刷新看门狗。

        第一次调用启动发送线程；之后每一次只是「我还按着」。
        """
        if kind not in ('joint', 'cart'):
            return False, '按住点动只支持 joint / cart'
        direction = 1.0 if float(direction) >= 0 else -1.0
        if kind == 'joint':
            if units.check_joint_deg(int(axis), 0.0):
                return False, '轴号 %s 不存在' % axis
            key = ('joint', int(axis), direction, None)
        else:
            err = units.check_cart_axis(str(axis)) or units.check_cart_frame(str(frame))
            if err:
                return False, err
            key = ('cart', str(axis), direction, str(frame))

        now = time.monotonic()
        with self.lock:
            h = self.state['hold']
            if h is not None:
                if h['key'] != key:
                    return False, '已经在按另一个方向了，先松手'
                h['until'] = now + HOLD_WATCHDOG_SEC
                return True, ''

        reason = self._gate(gating.MOTION)
        if reason:
            with self.lock:
                self._log_event('reject', '按住点动：%s' % reason, False, locked=True)
            return False, reason
        if kind == 'cart' and not self.ik_cli.service_is_ready():
            return False, '依赖未就绪：/compute_ik'
        if not self._cmd_lock.acquire(blocking=False):
            return False, '上一条动作还没结束，不接受新命令'

        with self.lock:
            deg7 = self.state['fb_deg7']
            if deg7 is None:
                self._cmd_lock.release()
                return False, '还没收到反馈'
            label = '按住点动 %s %s%s' % (
                'a%d' % axis if kind == 'joint' else str(axis),
                '+' if direction > 0 else '-',
                '' if kind == 'joint' else '（%s系）' % ('工具' if frame == 'tool' else '基座'))
            self.state['hold'] = {
                'key': key, 'kind': kind, 'axis': axis, 'dir': direction, 'frame': frame,
                'until': now + HOLD_WATCHDOG_SEC, 'target': list(deg7[:6]),
                'pose': None, 'seed': None, 'steps': 0, 'capped': 0, 'late': 0,
                # sent = 在飞的那一段 (轴, 目标, 发出时刻, 进区域半径, 兜底秒)；
                # last_sent = 每根轴最后一次真发出去的值，慢轴按它攒够再发
                'sent': None, 'last_sent': list(deg7[:6]), 'lat': 0.0,
                't_last': now - HOLD_MIN_SEG_SEC, 'stop': '',
            }
            self.state['busy'] = True
            self.state['busy_label'] = label
            self.state['last_result'] = None

        threading.Thread(target=self._hold_loop, args=(label,),
                         name='d1t_teach_hold', daemon=True).start()
        return True, ''

    def cmd_hold_stop(self):
        """松手。停发新目标——这不是刹车，臂会走完最后一条命令的剩余行程。"""
        with self.lock:
            if self.state['hold'] is not None:
                self.state['hold']['until'] = 0.0
        return True, ''

    def _hold_publisher_gone(self, timeout=3.0):
        """等 /arm_Command 上看不见自己为止：trajectory_server 见到外来发布者会拒绝 stream。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                names = [i.node_name for i in self.get_publishers_info_by_topic('/arm_Command')]
            except Exception:                                   # noqa: BLE001
                return True
            if 'd1t_teach' not in names:
                return True
            time.sleep(0.1)
        return False

    def _hold_send(self, target6, moved_axes, delay_ms):
        """按 funcode=1 发这一段的目标。delay_ms 按本段时长算，控制器据此定速度与加速度。"""
        for i, ax in enumerate(moved_axes):
            if i:
                time.sleep(F1_GAP_SEC)
            msg = ArmString()
            msg.data = flyby.f1_command_json(
                self._hold_seq, ax, target6[ax], delay_ms)
            self._hold_seq += 1
            self._hold_pub.publish(msg)

    def _hold_next_joint(self, h, d_joint):
        """关节：在上一周期的目标上继续加，目标始终领先臂一个周期，动作才连续。"""
        target = list(h['target'])
        axis = int(h['axis'])
        target[axis] = units.round1(target[axis] + d_joint * h['dir'])
        err = units.check_joint_deg(axis, target[axis])
        if err:
            return None, None, err
        return target, [axis], None

    def _hold_next_cart(self, h, d_mm, d_deg, cap):
        """笛卡尔：位姿累加一步 → 逆解 → 关节目标，并按关节速度上限压步长。

        压步长这一步是 2026-09-23 加的，根因见 §「各轴不协调」：笛卡尔步长是按 TCP
        速度定的，但同样的 TCP 位移在不同姿态下要的关节行程差几十倍（实测工具系 rz
        转 8.4° 要 a5 走 48°）。步长不压，某些周期会要求某根轴在 100 ms 里走好几度，
        臂做不到就各轴各自落后不同的量——看上去就是「不同步、很诡异」。
        这里把任一轴每周期的行程压到与关节按住同一个上限（HOLD_JOINT_DEG_PER_S）。
        """
        step = d_mm if h['axis'] in ('x', 'y', 'z') else d_deg
        # 按名义步长解一次，拿本姿态的增益（关节行程 / 笛卡尔步长），再线性修正到 cap 上。
        # 修正只降不弹、每次都从实测增益重算，所以不会像旧写法那样在 cap 附近走锯齿
        # （见 units.cart_step_for_cap）。姿态越接近奇异，增益越非线性，一次不一定够，
        # 最多迭代 3 次——rz 这类方向实测一次压过头会把步长压到发不出命令。
        capped = False
        for _ in range(3):
            out = self._cart_ik_step(h, step)
            if out[-1]:                          # 逆解无解 / 超限：原样上报
                return None, None, out[-1]
            need = max(abs(out[0][i] - h['target'][i]) for i in range(6))
            if need <= cap:
                break
            step = units.cart_step_for_cap(step, need, cap)
            capped = True
        # 跳解判定放在压步长之后：先把步长压下去再看还跳不跳，否则「这个方向陡」会被
        # 当成「换了一支解」一律停发——实测工具系 rz 第一步就被这么挡掉。
        if out[4] > HOLD_JUMP_LIMIT_DEG:
            return None, None, ('逆解跳解 %.1f° > %.0f°，停发（KDL 换了一支解，再走臂会猛跳）'
                                % (out[4], HOLD_JUMP_LIMIT_DEG))
        if need > cap * 1.5:
            return None, None, ('这个方向关节要走太快（本周期需 %.1f°，上限 %.1f°），停发'
                                % (need, cap))
        target, q_new, sel_target, tip_target = out[:4]
        h['pose'], h['seed'], h['tip'] = (sel_target[0], sel_target[1]), q_new, tip_target
        if capped:
            h['capped'] += 1
        # 六根轴每周期全发：只发「动了的轴」会让慢轴攒够 0.05° 才收到一条命令，
        # 于是慢轴一顿一顿走、快轴匀速走，合成出来就是各轴不同步。
        return target, list(range(6)), None

    def _cart_ik_step(self, h, step):
        """按 step 走一步的逆解。

        返回 (关节目标, 解, 选中TCP目标位姿, 末端目标位姿, 相对种子的最大单轴跳变, 错误)。
        跳变只算出来，判不判由调用方定——步长压过之后再判才准。
        """
        sel_p, sel_q = h['pose']
        sel_target = units.cart_target(h['frame'], h['axis'], step * h['dir'], sel_p, sel_q)
        tip_target = units.retarget_to_link((sel_p, sel_q), h['tip'], sel_target)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = IK_TIP_LINK
        r.avoid_collisions = False      # Thomas 2026-09-23：这条通路不做任何碰撞检查
        r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=h['seed']))
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) for v in tip_target[0])
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = tip_target[1]
        r.pose_stamped = ps
        r.timeout.nanosec = int(0.02 * 1e9)
        res = self._wait(self.ik_cli.call_async(req), 0.5)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, None, None, None, 0.0, '逆解无解（多半是走到奇异方向了），停发'
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q_new = [sol[j] for j in JOINTS]
        jump = max(abs(units.rad2deg(q_new[i] - h['seed'][i])) for i in range(6))
        # 不 round1：0.1° 的量化对每周期只走零点几度的慢轴是 50% 的误差，
        # 攒起来就是抖。限位检查用四舍五入后的值判，下发用原值。
        target = list(units.urdf_rad_to_controller_deg(q_new))
        err = units.check_joint_target([units.round1(v) for v in target])
        if err:
            return None, None, None, None, jump, err
        return target, q_new, sel_target, tip_target, jump, None

    def _hold_loop(self, label):
        """按住期间的发送线程：自己建 /arm_Command 发布者，松手后销毁并等它从话题上消失。"""
        stop, t0 = '', time.monotonic()
        try:
            self._hold_pub = self.create_publisher(ArmString, '/arm_Command', self._cmd_qos)
            time.sleep(0.15)        # 等 DDS 配对，否则第一条必丢（BEST_EFFORT）
            self._start_rec()
            while rclpy.ok():
                cycle = time.monotonic()
                with self.lock:
                    h = self.state['hold']
                    if h is None:
                        break
                    if cycle > h['until']:
                        stop = stop or ('松手' if h['until'] == 0.0 else
                                        '看门狗：%.0f ms 没收到心跳' % (HOLD_WATCHDOG_SEC * 1000))
                        break
                    deg7 = self.state['fb_deg7']
                    stale = gating.feedback_is_stale(self._fb_age(), units.FEEDBACK_STALE_SEC)
                if stale or deg7 is None:
                    stop = '反馈已断'
                    break
                if h['kind'] == 'cart' and h['pose'] is None:
                    sel, tip = self._current_pose()
                    if sel is None:
                        stop = '当前 TCP 位姿未知'
                        break
                    h['pose'], h['tip'] = sel, tip
                    h['seed'] = units.controller_deg_to_urdf_rad(deg7[:6])

                # 发下一段的两个条件：距上一段够久了，且上一段已经走进区域（或兜底到点）。
                # 「进区域再发下一段」是 stream 流畅的根本原因，也是固定 10 Hz 硬发会
                # 顿挫的原因（见文件头「按住即走」段）。
                now_m = time.monotonic()
                if now_m - h['t_last'] < HOLD_MIN_SEG_SEC:
                    time.sleep(HOLD_POLL_SEC)
                    continue
                if h['sent'] is not None:
                    axes, tgt, due, lag_limit, counted = h['sent']
                    lag = max(abs(deg7[a] - tgt[a]) for a in axes)
                    if lag > lag_limit:
                        # 臂已经落后本段行程的 2.5 倍：再往前发只会让目标离臂越来越远，
                        # 松手时滑行也越长。先等它追。
                        if not counted:
                            h['late'] += 1
                            h['sent'] = (axes, tgt, due, lag_limit, True)
                        time.sleep(HOLD_POLL_SEC)
                        continue
                    if now_m < due:
                        time.sleep(HOLD_POLL_SEC)
                        continue

                # 段长取固定的名义值，不跟实际间隔走。跟着走会互相追：间隔短→步长小→
                # 控制器走得快→间隔更短……一路缩到「本进程发一条命令要多久」为止，
                # 重叠归零（实测笛卡尔缩到 107 ms / 1.26°，重叠 1 ms）。
                # 固定段长之后，自适应的是**发送时刻**，不是步长。
                seg_sec = self.hold_max_seg
                delay_ms = max(1, min(32767, int(round(seg_sec * 1000.0 / 1.27))))
                v_joint, v_mm, v_deg = self._hold_rates()
                if h['kind'] == 'joint':
                    target, moved, err = self._hold_next_joint(h, v_joint * seg_sec)
                else:
                    target, moved, err = self._hold_next_cart(
                        h, v_mm * seg_sec, v_deg * seg_sec, v_joint * seg_sec)
                if err:
                    stop = err
                    break
                # 只发离「上次真发给这根轴的值」超过 F1_MIN_DEG 的轴：慢轴攒够了再发，
                # 攒的量有上界（F1_MIN_DEG），不会像按上一段目标比那样越攒越顿。
                moved = [a for a in moved if abs(target[a] - h['last_sent'][a]) >= F1_MIN_DEG]
                # 这一段比「该发的时刻」晚了多少：下一段的 due 提前这么多补回来
                if h['sent'] is not None:
                    late_by = max(0.0, time.monotonic() - h['sent'][2])
                    h['lat'] = (1.0 - HOLD_LAT_EMA) * h['lat'] + HOLD_LAT_EMA * late_by
                h['t_last'] = time.monotonic()
                if moved:
                    self._hold_send(target, moved, delay_ms)
                    span = max(abs(target[a] - h['last_sent'][a]) for a in moved)
                    for a in moved:
                        h['last_sent'][a] = target[a]
                    # 控制器走完本段要多久（厂商公式）→ 下一段要在它走完前 HOLD_OVERLAP_MS 发出，
                    # 再把本进程自己的发送延迟提前补掉，才是真的重叠而不是「打算重叠」
                    move_sec = flyby.controller_move_sec(span, delay_ms)
                    due = (h['t_last'] + move_sec
                           - self.hold_overlap_ms / 1000.0 - h['lat'])
                    h['sent'] = (moved, list(target),
                                 max(due, h['t_last'] + HOLD_MIN_SEG_SEC),
                                 max(1.0, HOLD_LAG_FRAC * span), False)
                    h['steps'] += 1
                h['target'] = target
        except Exception as exc:                                # noqa: BLE001
            stop = '异常：%s' % exc
        finally:
            steps, capped, late = 0, 0, 0
            with self.lock:
                h = self.state['hold']
                if h is not None:
                    steps, capped, late = h['steps'], h['capped'], h['late']
                self.state['hold'] = None
            if self._hold_pub is not None:
                self.destroy_publisher(self._hold_pub)
                self._hold_pub = None
            gone = self._hold_publisher_gone()
            csv_note = self._finish_rec(label)
            result = {'ok': True,
                      'text': '%s：%d 段 / %.1f s 停发（%s）%s%s%s%s'
                              % (label, steps, time.monotonic() - t0, stop or '结束',
                                 '' if not capped else
                                 '；其中 %d 段因关节要走太快被压了步长（TCP 走得比设定慢）'
                                 % capped,
                                 '' if not late else
                                 '；有 %d 段是等兜底超时才发下一段（臂没跟上，段长或速度要调）'
                                 % late,
                                 csv_note,
                                 '' if gone else '；/arm_Command 上还看得见本节点，'
                                                 '下一条 MoveIt 运动可能被 stream 入口拒')}
            with self.lock:
                self.state['busy'] = False
                self.state['busy_label'] = ''
                self.state['settle_until'] = time.monotonic() + SETTLE_MOTION_SEC
                self.state['last_result'] = result
                self._log_event('done', result['text'], True, locked=True)
            self._cmd_lock.release()

    # ---------- trajectory_server 子进程与执行模式 ----------

    def _traj_node_present(self):
        return any(name == TRAJ_NODE_NAME for name, _ in self.get_node_names_and_namespaces())

    def start_trajectory_server(self):
        if not self.manage_traj:
            self._log_event('info', 'manage_trajectory_server=false，不托管 trajectory_server', True)
            return
        if self._traj_node_present():
            self._log_event('warn', '已有 %s 在跑，不抢；它是什么执行模式本界面不知道'
                                    % TRAJ_NODE_NAME, True)
            return
        self._spawn_traj()

    def _spawn_traj(self):
        cmd = ['ros2', 'run', 'd1t_arm', 'trajectory_server', '--ros-args',
               '-p', 'execution_mode:=%s' % EXEC_MODE,
               '-p', 'max_points:=%d' % self.traj_max_points,
               '-p', 'zone_deg:=%.1f' % self.traj_zone_deg]
        self._traj_proc = subprocess.Popen(cmd, start_new_session=True)
        deadline = time.monotonic() + TRAJ_RESTART_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._traj_node_present():
                self._log_event('info', 'trajectory_server 已起：execution_mode=%s '
                                        'max_points=%d zone_deg=%.1f'
                                        % (EXEC_MODE, self.traj_max_points,
                                           self.traj_zone_deg), True)
                return True
            time.sleep(0.3)
        self._log_event('fail', 'trajectory_server 启动后 %.0fs 没出现在节点列表里'
                                % TRAJ_RESTART_TIMEOUT_SEC, False)
        return False

    def _stop_traj(self):
        proc = self._traj_proc
        self._traj_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            proc.terminate()
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        deadline = time.monotonic() + 10.0
        while self._traj_node_present() and time.monotonic() < deadline:
            time.sleep(0.2)

    # ---------- 只改界面状态的配置项 ----------

    def cmd_set_tcp(self, tcp_id):
        tcp = next((t for t in TCPS if t['id'] == tcp_id), None)
        if tcp is None:
            return False, 'TCP %s 不存在' % tcp_id
        if not tcp['enabled']:
            return False, '「%s」本期不可选：%s' % (tcp['name'], tcp['note'])
        with self.lock:
            self.state['tcp'] = tcp_id
            self.state['pose'] = None
            self._log_event('info', '选中 TCP：%s（%s）' % (tcp['name'], tcp['link']), True,
                            locked=True)
        return True, ''

    def cmd_set_speed(self, vel=None, acc=None):
        """界面的速度 / 加速度两个滑块：改 MoveIt 的缩放，下一条运动生效。

        两个分开调（Thomas 2026-09-23）。为什么不是改 /set_override：stream 模式下
        override 对经由点不生效（trajectory_server._stream_via 把 override 写死成 1.0），
        一条 jog 轨迹里除最后一点全是经由点，拖 override 基本不改 jog 快慢。
        """
        for name, value in (('速度', vel), ('加速度', acc)):
            if value is None:
                continue
            err = units.check_scaling(value, self.max_scaling)
            if err:
                return False, '%s缩放：%s；上限由启动参数 max_scaling 给（现在 %s）' % (
                    name, err, self.max_scaling)
        if vel is None and acc is None:
            return False, '没给要改的值'
        with self.lock:
            if self.state['busy']:
                return False, '执行中不改速度，等这条走完'
            if vel is not None:
                self.vel = float(vel)
            if acc is not None:
                self.acc = float(acc)
            self._log_event('info', '缩放 → 速度 %.2f / 加速度 %.2f（MoveIt，下一条运动生效）'
                                    % (self.vel, self.acc), True, locked=True)
        return True, ''

    def cmd_clear_external(self):
        """手动解除「外部运动」锁定。

        判定只能看反馈，看不出抖动是伺服自己抖还是别人在动臂；人站在臂前面看得出来。
        这个按钮就是把这个判断交回给人：他确认没别人动臂，点一下继续用。
        """
        with self.lock:
            if not self._external_now():
                return True, '当前没有外部运动锁定'
            why = self.state['external_why']
            self.state['external_until'] = 0.0
            self.state['external_why'] = ''
            self._idle_ref = None
            self._log_event('warn', '人工解除外部运动锁定（原因曾是：%s）' % why, True, locked=True)
        return True, '已解除，若真有别人在动臂会立刻再次报出来'

    def cmd_set_frame(self, frame):
        err = units.check_cart_frame(frame)
        if err:
            return False, err
        with self.lock:
            self.state['frame'] = frame
        return True, ''

    def cmd_set_grip_values(self, open_raw, close_raw):
        with self.lock:
            err = presets.set_grip_values(self.data, open_raw, close_raw)
            if err:
                return False, err
            presets.save(self.data, self.presets_path)
            self._log_event('info', '夹爪开值 %s / 关值 %s（raw）已存盘'
                                    % (units.fmt1(open_raw), units.fmt1(close_raw)),
                            True, locked=True)
        return True, ''

    def cmd_record_preset(self, slot_id, name=None, kind='joint'):
        """kind='joint' 只记关节；kind='cart' 同时记当前选中 TCP 的位姿（Thomas 2026-09-23）。"""
        if kind not in ('joint', 'cart'):
            return False, '记录类型只能是 joint 或 cart，收到 %s' % kind
        pose = None
        if kind == 'cart':
            sel, _tip = self._current_pose()
            if sel is None:
                with self.lock:
                    why = self.state['pose_err'] or '还没算出当前位姿'
                return False, '记不了笛卡尔位姿：%s' % why
            with self.lock:
                tcp_id = self.state['tcp']
            pose = {'tcp': tcp_id, 'p_m': list(sel[0]), 'q': list(sel[1])}
        with self.lock:
            deg7 = self.state['fb_deg7']
            stale = gating.feedback_is_stale(self._fb_age(), units.FEEDBACK_STALE_SEC)
            if deg7 is None or stale:
                return False, '反馈已断，不能记录当前位置'
            # HOME 的 a1 读数本来就在控制器限位外（约 -90.4），照抄存不进去；
            # 与点动时保持轴的处理一致：钳回边界并如实报出钳了多少。
            clamped, notes = units.clamp_to_limit(deg7[:6])
            err = presets.set_slot(self.data, slot_id, clamped + [deg7[6]], name, pose=pose)
            if err:
                return False, err
            presets.save(self.data, self.presets_path)
            extra = ('；读数在限位外、已钳回：' + '，'.join('%s %s→%s' % n for n in notes)
                     if notes else '')
            if pose is not None:
                extra += ('；同时记下 %s 的位姿 (%s, %s, %s) mm'
                          % (pose['tcp'], units.fmt1(units.m2mm(pose['p_m'][0])),
                             units.fmt1(units.m2mm(pose['p_m'][1])),
                             units.fmt1(units.m2mm(pose['p_m'][2]))))
            self._log_event('info', '槽位 %s 记录当前位置 %s%s'
                                    % (slot_id, [units.fmt1(v) for v in clamped + [deg7[6]]],
                                       extra), True, locked=True)
        return True, ''

    def cmd_clear_preset(self, slot_id):
        with self.lock:
            err = presets.clear_slot(self.data, slot_id)
            if err:
                return False, err
            presets.save(self.data, self.presets_path)
        return True, ''

    # ---------- HTTP ----------

    def serve_forever(self):
        handler = _make_handler(self)
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.httpd.daemon_threads = True
        self.get_logger().info(
            'TEACH 界面在 http://%s:%d/ ；%s' % (self.host, self.port, SAFETY_TEXT['estop']))
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def shutdown(self):
        if self.httpd is not None:
            self.httpd.shutdown()
        self._stop_traj()


_STATIC = {'/': 'index.html', '/index.html': 'index.html',
           '/app.js': 'app.js', '/style.css': 'style.css'}
_MIME = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
         '.css': 'text/css; charset=utf-8'}


def _make_handler(server):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype):
            data = body if isinstance(body, bytes) else body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, ensure_ascii=False), 'application/json; charset=utf-8')

        def do_GET(self):
            path = posixpath.normpath(self.path.split('?')[0])
            if path == '/api/state':
                return self._json(server.snapshot())
            name = _STATIC.get(path)
            if name is None:
                return self._json({'ok': False, 'reason': '没有这个地址'}, 404)
            try:
                with open(os.path.join(server.web_dir, name), 'rb') as f:
                    body = f.read()
            except OSError:
                return self._json({'ok': False, 'reason': '静态文件缺失：%s' % name}, 500)
            return self._send(200, body, _MIME[os.path.splitext(name)[1]])

        def do_POST(self):
            path = posixpath.normpath(self.path.split('?')[0])
            try:
                length = int(self.headers.get('Content-Length') or 0)
                payload = json.loads(self.rfile.read(length) or b'{}')
            except (ValueError, TypeError):
                return self._json({'ok': False, 'reason': '请求体不是合法 JSON'}, 400)
            if not isinstance(payload, dict):
                return self._json({'ok': False, 'reason': '请求体必须是 JSON 对象'}, 400)
            try:
                ok, reason = _dispatch(server, path, payload)
            except (ValueError, TypeError, KeyError) as exc:
                return self._json({'ok': False, 'reason': '参数错误：%s' % exc}, 400)
            return self._json({'ok': ok, 'reason': reason}, 200 if ok else 409)

    return Handler


def _dispatch(server, path, p):
    if path == '/api/jog_joint':
        return server.cmd_jog_joint(int(p['axis']), float(p['delta_deg']))
    if path == '/api/goto_joint':
        return server.cmd_goto_joint(int(p['axis']), float(p['deg']))
    if path == '/api/jog_cart':
        return server.cmd_jog_cart(str(p['frame']), str(p['axis']), float(p['value']))
    if path == '/api/grip':
        return server.cmd_grip(str(p['which']))
    if path == '/api/grip_values':
        return server.cmd_set_grip_values(float(p['open_raw']), float(p['close_raw']))
    if path == '/api/tcp':
        return server.cmd_set_tcp(str(p['tcp']))
    if path == '/api/hold':
        return server.cmd_hold(str(p['kind']), p['axis'], p['dir'], p.get('frame'))
    if path == '/api/hold_stop':
        return server.cmd_hold_stop()
    if path == '/api/stiffness':
        return server.cmd_stiffness(int(p['mode']))
    if path == '/api/clear_external':
        return server.cmd_clear_external()
    if path == '/api/speed':
        return server.cmd_set_speed(
            None if p.get('vel') is None else float(p['vel']),
            None if p.get('acc') is None else float(p['acc']))
    if path == '/api/frame':
        return server.cmd_set_frame(str(p['frame']))
    if path == '/api/override':
        return server.cmd_override(float(p['ratio']))
    if path == '/api/servo':
        return server.cmd_servo(bool(p['enable']))
    if path == '/api/preset/goto':
        return server.cmd_preset(str(p['id']), bool(p.get('confirm', False)))
    if path == '/api/preset/record':
        return server.cmd_record_preset(str(p['id']), p.get('name'),
                                        str(p.get('kind', 'joint')))
    if path == '/api/preset/clear':
        return server.cmd_clear_preset(str(p['id']))
    return False, '没有这个接口：%s' % path


def main():
    rclpy.init()
    node = None
    try:
        node = TeachServer()
        node.start_trajectory_server()
        node.serve_forever()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
