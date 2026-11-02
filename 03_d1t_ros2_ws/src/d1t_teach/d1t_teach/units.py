#!/usr/bin/env python3
"""units.py — TEACH-01 界面层的单位换算、限位判定与增量合成（纯函数，无 ROS 依赖）。

约定（TEACH-01 任务书 §1「已批准约束」，Thomas 2026-09-22）：
- 界面层用度、mm，显示与输入统一 1 位小数；程序内部一律用弧度、米。
- 关节角一律用控制器轴 a0~a5；URDF 的 Joint4、Joint6 与 a3、a5 差一个负号。
- 夹爪 angle6_raw 是无量纲原始值，是单位规则的例外；命令域 [20, 50]。

数值来源全部取 manifest/OPERATIONS.md ·「H.1 逐轴表」「H.4 单位与方向换算」，
不在本文件另填一套。
"""

import math

AXIS_NAMES = ['a0', 'a1', 'a2', 'a3', 'a4', 'a5']
URDF_JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

# OPERATIONS.md H.1「方向符号」：a3/a5（URDF Joint4/Joint6）实机转向与 URDF axis 相反
JOINT_SIGNS = [1.0, 1.0, 1.0, -1.0, 1.0, -1.0]

# 控制器限位（截断线），(下界, 上界)。超了不拒绝、静默截断照走。
#
# 2026-09-23 Thomas 裁定放宽到他实测的安全行程，原值是厂商的 ±135/±90/±90/±135/±90/±135。
# **这张表只是臂上那张表的镜像**：真正截断的是臂上 marm_controller_node.cpp:176-177 的
# maxangle/minangle 两个数组。臂上不改，这里改了也没用——命令照样被截回 ±135。
# 两边必须同时改、改成同一组数，见 forlearning/TEACH-01-web-jog 的限位操作清单。
CTRL_LIMIT_DEG = [(-163.0, 163.0), (-90.3, 97.0), (-97.0, 89.0),
                  (-166.0, 163.0), (-96.0, 113.0), (-166.0, 162.0)]
# URDF 限位（MoveIt 规划用的上下界）。URDF 比控制器各放宽 0.2°：URDF 窄一点会把边界姿态
# 挡在规划外（原来 ±89.95 比控制器 ±90 窄，折叠姿态 a1=-90 就卡在边界上）。
URDF_MARGIN_DEG = 0.2
URDF_LIMIT_DEG = [(lo - URDF_MARGIN_DEG, hi + URDF_MARGIN_DEG) for lo, hi in CTRL_LIMIT_DEG]
# 界面判定线 = 控制器限位。超了 arm_driver 会静默截断照走，这是必须挡住的那条线。
UI_LIMIT_DEG = list(CTRL_LIMIT_DEG)
# arm_driver 拒绝线（实测硬件行程 ±3°）：只用于状态条提示，不作为界面判定线
DRIVER_REJECT_DEG = [(-167.0, 168.0), (-93.4, 101.0), (-99.5, 92.6),
                     (-169.0, 166.0), (-100.3, 116.0), (-170.2, 165.9)]

# 夹爪命令域（Grip.srv：越界 reason='out_of_range'、不发命令）
# 上沿 2026-09-23 从 50 放宽到 64.5（Thomas 裁定，见 d1t_arm.grip_detect.GRIP_TARGET_MAX
# 的同一条注释）：50 只是当时的控制器限位，不是夹爪本身量程。下沿未改。
# 下沿 2026-09-24 从 20 放宽到 -30（Thomas 裁定，同 d1t_arm.grip_detect.GRIP_TARGET_MIN）。
GRIP_CMD_MIN = -30.0
GRIP_CMD_MAX = 64.5
# OPERATIONS.md C.3：爪间距(mm) = clamp(0.85 × angle6_raw + 19.8, 0, 62)
#
# 这条公式的 6 个标定点全部落在 raw∈[0, ~50]（S3 阶段实测）；raw 上沿放宽到 64.5 后，
# raw∈(50, 64.5] 这一段是线性外推，没有真实标定点支持。GRIP_GAP_MAX_MM 从 62 改到 70
# （Thomas 实测的物理最大张开），只是不让显示提前封顶在旧上限——不代表这段区间的 mm
# 读数是准的，只能当粗略参考，跟折叠区的关节限位边界一样「未验证，走到了留意听声音」。
GRIP_GAP_SLOPE = 0.85
GRIP_GAP_INTERCEPT = 19.8
GRIP_GAP_MAX_MM = 70.0

# TEACH-01 §1：实机速度与加速度缩放上限 0.3。
# 这不是硬件上限——硬件侧 MoveIt 关节限速 J1~J3 60.2 °/s、J4~J6 99.1 °/s，
# 控制器削顶 114.59 °/s（OPERATIONS.md H.2）。0.3 是 S4b-3 / S4b-4 实机连跑用值，
# 由任务书写死。teach_server 把它做成启动参数 max_scaling，默认仍是 0.3。
MAX_SCALING = 0.3
# OPERATIONS.md D.1：/set_override 域 0.01~1.0
OVERRIDE_MIN = 0.01
OVERRIDE_MAX = 1.0

# /set_stiffness 档位量程（arm_driver 的 STIFFNESS_MODE_MAX 同值，2026-09-11 实测盖棺）。
# STIFFNESS_HOLD_MIN 是控制器的 enable_status 判据：低于它控制器不认为在使能，
# arm_driver 会当成卸力、走折叠区检查。
STIFFNESS_MIN = 0
STIFFNESS_MAX = 1000
STIFFNESS_HOLD_MIN = 1000

# 增量步长档位（TEACH-01 §1 建议值）
JOINT_STEPS_DEG = [0.1, 1.0, 5.0, 10.0]
POS_STEPS_MM = [1.0, 5.0, 10.0, 50.0]
ROT_STEPS_DEG = [0.5, 1.0, 5.0, 10.0]

# 反馈超过这个岁数就算「反馈已断」，不得继续当实时值显示。
# 实机 /arm_Feedback 约 8.98 Hz 且出现过 0.41~0.61 s 的洞（OPERATIONS.md C.1），
# 取 0.8 s：比已知最大丢帧洞略宽，又远小于「臂在动而界面不知道」的危险时长。
FEEDBACK_STALE_SEC = 0.8


# ---------- 显示与基本换算 ----------

def round1(value):
    """界面显示与输入统一保留 1 位小数（与反馈真实分辨率 0.1° 一致）。"""
    return round(float(value) + 0.0, 1)


def fmt1(value):
    return '%.1f' % round1(value)


def deg2rad(deg):
    return math.radians(float(deg))


def rad2deg(rad):
    return math.degrees(float(rad))


def mm2m(mm):
    return float(mm) / 1000.0


def m2mm(m):
    return float(m) * 1000.0


def controller_deg_to_urdf_rad(deg6):
    """控制器轴 a0~a5（度）→ URDF Joint1~Joint6（弧度）。"""
    if len(deg6) != 6:
        raise ValueError('需要 6 个控制器轴角度，收到 %d 个' % len(deg6))
    return [math.radians(float(deg6[i])) * JOINT_SIGNS[i] for i in range(6)]


def urdf_rad_to_controller_deg(rad6):
    """URDF Joint1~Joint6（弧度）→ 控制器轴 a0~a5（度）。"""
    if len(rad6) != 6:
        raise ValueError('需要 6 个 URDF 关节角，收到 %d 个' % len(rad6))
    return [math.degrees(float(rad6[i])) * JOINT_SIGNS[i] for i in range(6)]


def raw_to_gap_mm(raw):
    """夹爪 angle6_raw → 爪间距 mm。**空载估算**：GRIP-01 实测 raw 33~38 段
    （夹住物体时）该公式偏大 3.4 mm 以上，不可用于判断夹住没夹住。"""
    return min(max(GRIP_GAP_SLOPE * float(raw) + GRIP_GAP_INTERCEPT, 0.0), GRIP_GAP_MAX_MM)


def gap_mm_to_raw(gap_mm):
    """爪间距 mm → angle6_raw（raw_to_gap_mm 在未截断区间的反函数）。"""
    return (float(gap_mm) - GRIP_GAP_INTERCEPT) / GRIP_GAP_SLOPE


# ---------- 限位与参数判定：越界返回中文原因，调用方据此报错且不下发 ----------

def check_joint_deg(axis, deg):
    """单轴目标角判定。返回 None = 可以下发；返回字符串 = 报错原因，不得下发。"""
    if axis < 0 or axis > 5:
        return '轴号 %s 不存在，只有 a0~a5' % axis
    try:
        value = float(deg)
    except (TypeError, ValueError):
        return '目标角不是数字'
    if not math.isfinite(value):
        return '目标角不是有限数'
    lo_lim, hi_lim = UI_LIMIT_DEG[axis]
    if value < lo_lim or value > hi_lim:
        lo, hi = DRIVER_REJECT_DEG[axis]
        extra = ('；并且超过 arm_driver 拒绝线 [%s, %s]' % (fmt1(lo), fmt1(hi))
                 if value < lo or value > hi else
                 '；超控制器限位会被 arm_driver 静默截断照走，所以界面这里就挡掉')
        return '%s 目标 %s° 超出控制器限位 [%s, %s]%s' % (
            AXIS_NAMES[axis], fmt1(value), fmt1(lo_lim), fmt1(hi_lim), extra)
    return None


def check_joint_target(deg6):
    """整组六轴目标判定，返回第一个越界轴的原因或 None。"""
    if len(deg6) != 6:
        return '关节目标需要 6 个值，收到 %d 个' % len(deg6)
    for axis in range(6):
        err = check_joint_deg(axis, deg6[axis])
        if err:
            return err
    return None


def check_grip_raw(raw):
    """夹爪命令值判定（无量纲 raw，不是角度也不是 mm）。"""
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return '夹爪命令值不是数字'
    if not math.isfinite(value):
        return '夹爪命令值不是有限数'
    if value < GRIP_CMD_MIN or value > GRIP_CMD_MAX:
        return '夹爪命令值 %s 超出命令域 [%s, %s]（raw，不是 mm）' % (
            fmt1(value), fmt1(GRIP_CMD_MIN), fmt1(GRIP_CMD_MAX))
    return None


def check_scaling(value, max_scaling=MAX_SCALING):
    """MoveIt 速度 / 加速度缩放判定。上限默认 0.3，由 teach_server 的 max_scaling 参数给。"""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return '缩放值不是数字'
    if not math.isfinite(v) or v <= 0.0 or v > float(max_scaling):
        return '缩放 %s 超出 (0, %s]' % (v, max_scaling)
    return None


def check_stiffness(value):
    """/set_stiffness 档位判定。量程 0~1000 是 2026-09-11 实测盖棺的值：

    1000 以上（含 65536、80000）全部饱和、与满档无异，官方文档的 0~80000 在实机上
    不成立；而且 mode 最终塞进 uint16，超界会静默回绕成「enable_status=1 但舵机全松」
    的假使能。所以界面先挡，不靠服务去截。
    """
    try:
        v = int(value)
    except (TypeError, ValueError):
        return '刚度档位不是整数'
    if v < STIFFNESS_MIN or v > STIFFNESS_MAX:
        return '刚度档位 %s 超出 [%d, %d]' % (value, STIFFNESS_MIN, STIFFNESS_MAX)
    return None


def check_override(value):
    """/set_override 倍率判定。服务自己会截断，界面先挡掉，不靠它截。"""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return '倍率不是数字'
    if not math.isfinite(v) or v < OVERRIDE_MIN or v > OVERRIDE_MAX:
        return '倍率 %s 超出 [%s, %s]' % (v, OVERRIDE_MIN, OVERRIDE_MAX)
    return None


def cart_step_for_cap(step, need, cap, margin=0.95):
    """笛卡尔步长按关节速度上限修正，一次到位、不带状态。

    step  本段试探用的笛卡尔步长（mm 或 °）
    need  这个步长解出来的最大单轴关节行程（°）
    cap   每段允许的最大单轴关节行程（°）

    返回该用的步长。need <= cap 就原样返回。

    为什么不能「压一点、不够再压、宽松了再弹回来」：那是个 bang-bang，会在 cap 附近
    自己振荡。实测（2026-09-23，Thomas 实机反馈笛卡尔顿挫）旧写法带 1.3 倍回弹，
    对平滑变化的姿态给出锯齿状的步长——压下去、爬上来、再掉回去，周期约 3 段、
    段间跳 9%；换算到他的 2.7 段/秒就是约 1.1 s 一次的速度波动，与反馈 CSV 里量到的
    1.5~2 s 波动同量级。这里改成按实测增益一次线性修正：
    关节行程与笛卡尔步长在一小步内近似成正比，need/step 就是本姿态的增益，
    直接解出让 need 落在 cap 上的步长。姿态是连续变的，增益也连续，步长就连续。
    """
    if need <= cap or need <= 0.0:
        return step
    return step * (cap / need) * margin


def clamp_to_limit(deg6):
    """把「本次不动、只是要保持住」的轴钳进控制器限位。

    为什么需要：HOME 的 a1 实际读数约 -90.2 ~ -90.5，本来就在控制器限位 ±90 外
    （OPERATIONS.md H.1 脚注）。点动一根轴时，其余五轴要作为 MoveIt 关节目标一起下发，
    照抄读数会被判越界，所以钳回边界并如实报出钳了多少——不是静默截断用户输入的目标，
    用户真正要动的那根轴仍然是超了就拒绝。

    返回 (钳后的六个值, [(轴名, 原值, 钳后值), ...])。
    """
    out, notes = [], []
    for axis in range(6):
        lo_lim, hi_lim = UI_LIMIT_DEG[axis]
        value = float(deg6[axis])
        fixed = min(max(value, lo_lim), hi_lim)
        out.append(round1(fixed))
        if abs(fixed - value) > 1e-9:
            notes.append((AXIS_NAMES[axis], round1(value), round1(fixed)))
    return out, notes


# 折叠区判定：只看 a1、a2，±15°——与 arm_driver 的 /set_servo 同一条线
# （OPERATIONS.md ·「D.2 /set_servo」：|a1 + 90| <= 15 且 |a2 - 90| <= 15）。
FOLD_A1_DEG = -90.0
FOLD_A2_DEG = 90.0
FOLD_TOL_DEG = 15.0


def in_fold_zone(deg6):
    """当前是不是在折叠区。定义照 /set_servo，不另立一条线。"""
    return (abs(float(deg6[1]) - FOLD_A1_DEG) <= FOLD_TOL_DEG
            and abs(float(deg6[2]) - FOLD_A2_DEG) <= FOLD_TOL_DEG)


def limit_margins(deg6):
    """状态条用：每轴距界面限位上下界还有多少度（负值 = 已经在界外）。"""
    out = []
    for axis in range(6):
        lo_lim, hi_lim = UI_LIMIT_DEG[axis]
        value = float(deg6[axis])
        out.append((value - lo_lim, hi_lim - value))
    return out


# ---------- 姿态：四元数与增量合成 ----------
# 约定与 d1t_moveit_config/launch/s4b4_tcp.py 一致：固定轴 X-Y-Z，R = Rz(y)·Ry(p)·Rx(r)，
# 四元数顺序 (x, y, z, w)。test/test_units.py 直接拿 s4b4_tcp 的实现做交叉核对。

def quat_from_rpy(r, p, y):
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


def quat_mul(q1, q2):
    """先转 q2 再转 q1（矩阵意义上的 R(q1)·R(q2)）。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def matrix_to_rpy(R):
    """quat_from_rpy 的逆：R = Rz(y)·Ry(p)·Rx(r) → (r, p, y) 弧度。"""
    p = math.asin(max(-1.0, min(1.0, -R[2][0])))
    if abs(abs(p) - math.pi / 2) < 1e-7:
        return 0.0, p, math.atan2(-R[0][1], R[1][1])
    return math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])


def quat_to_rpy(q):
    return matrix_to_rpy(quat_to_matrix(quat_normalize(q)))


def quat_angle(q1, q2):
    """两个朝向之间的转角（弧度，0~π）。"""
    d = abs(sum(a * b for a, b in zip(quat_normalize(q1), quat_normalize(q2))))
    return 2.0 * math.acos(min(1.0, d))


def quat_conj(q):
    x, y, z, w = quat_normalize(q)
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    R = quat_to_matrix(quat_normalize(q))
    return [sum(R[row][k] * float(v[k]) for k in range(3)) for row in range(3)]


def pose_mul(a, b):
    """位姿相乘：a·b，位姿写成 (位置 m 三元组, 四元数)。"""
    pa, qa = a
    pb, qb = b
    rotated = quat_rotate(qa, pb)
    return ([pa[i] + rotated[i] for i in range(3)], quat_normalize(quat_mul(qa, qb)))


def pose_inv(a):
    p, q = a
    qi = quat_conj(q)
    rotated = quat_rotate(qi, p)
    return ([-v for v in rotated], qi)


def retarget_to_link(sel_now, tip_now, sel_target):
    """把「选中 TCP 要到的位姿」换算成「规划组末端 link 要到的位姿」。

    候选 TCP 都是固定关节挂在同一段刚体上，两者之间的偏移是常量：
    X = inv(选中 TCP 当前位姿) · (末端 link 当前位姿)，末端目标 = 选中目标 · X。
    这样不管选哪个 TCP，逆解永远只对规划组自己的末端 link 求，
    绕开「MoveIt 只接受与求解末端固定相连（且方向向下）的 tip」这条限制。
    """
    return pose_mul(sel_target, pose_mul(pose_inv(sel_now), tip_now))


CART_AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz']
CART_FRAMES = ['tool', 'base']


def check_cart_axis(axis):
    if axis not in CART_AXES:
        return '笛卡尔轴只能是 %s' % ' / '.join(CART_AXES)
    return None


def check_cart_frame(frame):
    if frame not in CART_FRAMES:
        return '参考系只能是 tool（工具系）或 base（基座系 base_link）'
    return None


def cart_target(frame, axis, value, p_m, q):
    """把一次笛卡尔增量合成为新的 TCP 目标位姿。

    frame='tool'：沿 / 绕选中 TCP 自身的轴；frame='base'：沿 / 绕 base_link 的轴。
    axis x/y/z 的 value 单位是 mm；rx/ry/rz 的 value 单位是度。
    姿态增量绕 TCP 点转，位置不变。
    返回 (新位置 m 三元组, 新朝向四元数)。
    """
    err = check_cart_frame(frame) or check_cart_axis(axis)
    if err:
        raise ValueError(err)
    q = quat_normalize(q)
    p_m = [float(v) for v in p_m]
    if axis in ('x', 'y', 'z'):
        idx = 'xyz'.index(axis)
        d_local = [0.0, 0.0, 0.0]
        d_local[idx] = mm2m(value)
        if frame == 'base':
            d = d_local
        else:
            R = quat_to_matrix(q)
            d = [sum(R[row][k] * d_local[k] for k in range(3)) for row in range(3)]
        return [p_m[i] + d[i] for i in range(3)], q
    idx = 'xyz'.index(axis[1])
    rpy = [0.0, 0.0, 0.0]
    rpy[idx] = deg2rad(value)
    dq = quat_from_rpy(*rpy)
    # 工具系：在 TCP 自身坐标里右乘；基座系：在 base_link 里左乘
    return p_m, quat_normalize(quat_mul(q, dq) if frame == 'tool' else quat_mul(dq, q))
