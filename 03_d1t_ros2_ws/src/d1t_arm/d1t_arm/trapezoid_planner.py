# trapezoid_planner.py — 时间同步梯形速度规划器（纯函数，无 ROS 依赖，可单测）。
#
# 输入起点/终点/三个 0~1 倍率，输出全臂统一的总时长 T 与一个统一的 delay_ms。
# 用在 arm_driver 的 funcode=1 通路：7 条单关节命令都带同一个 delay_ms，控制器按
# 「实际到位时间 = COMP × delay_ms」（行程无关、轴无关，见下）执行 → 七轴同时到达，
# 不需要额外同步逻辑。
#
# 常量全部从机械臂控制器源码读出（manifest/START_HERE.md「机械臂控制器内部实现」节），
# 不要自己猜：
#   V_MAX = 114.59 °/s   ← 源码 maxomega = 28.64789 * 4
#   A_MAX = 71.62  °/s²  ← 源码 maxalpha = 286.4789 / 4
#
# 补偿系数 COMP = 1.27（S4a-1b 实机标定，2026-09-10；验收 B 档——常数但不等于推导值）：
#   控制器不照我们给的 delay_ms 走，它拿 omega=s/T、alpha=s/T² 代回自己带 *1800 魔数的
#   公式重算 testdelay，行程 s 完全约掉 → 到位时间 = COMP·delay_ms，与行程/轴无关。
#   所以 delay_ms = T·1000 / COMP。
#   主窗口纸面推导 (sqrt(3)-1)·1.8 = 1.318。probe_comp_ratio 实机四点（j0，40° 行程，
#   delay_ms 1600~3600）线性拟合 measured_sec ≈ 1.27·(delay_ms/1000) + 0.23，残差 < 15 ms。
#   斜率取 1.27。与 1.318 差 3.6%，机制未查明——整型截断、单精度都解释不了这个量级。
#   +0.23 s 偏置疑为到位检测延迟（3 帧在带内 ≈ 0.3 s），未做仿射补偿（归因未定）。
#   要重新标定只改这一个常量。
#
# 成立前提：控制器的 alpha 不被 clamp 到 A_MAX。见下方 A_PLAN/clamp_threshold_sec。
#
# S-aux4c（2026-09-14）：上面这条"成立前提"曾经被规划器自己违反——它拿 A_MAX 本身
# 当加速度画曲线，画出来的三角形时间恒为控制器 clamp 阈值的 2 倍，比控制器实际能给
# 的最快还慢 57%（ratios=1.0 时 delay_ms 是门槛的 1.575 倍）。A_MAX/V_MAX 现在只作
# 控制器削顶门槛用；规划器画曲线单独用 A_PLAN（v 仍用 V_MAX，见 plan() 里的注释——
# 这也是为什么低速档几乎不受这次改动影响，只有高速档明显变快）。

import math
from collections import namedtuple

V_MAX = 114.59          # °/s   —— 控制器削顶门槛（也是规划器巡航速度上限，未拆分）
A_MAX = 71.62           # °/s²  —— 控制器削顶门槛，只用来算 clamp_threshold_sec，不进 plan()
COMP = 1.27             # 实际到位时间 / delay_ms，S4a-1b 实机标定（推导 1.318，差 3.6% 未解）
DELAY_MS_MAX = 32767    # 消息里 delay_ms 是 int16_t（§7 坑 2）
DELAY_MS_MIN = 1
RATIO_MIN = 0.01        # 倍率下限：防 T→∞ 与除零

# CLAMP_MARGIN：规划时长换算出的 delay_ms 相对「控制器真正会削顶的门槛」要留的安全
# 倍数，1.05 表示留 5%。A_PLAN 的取值就是让 ratios=1.0 时算出的 delay_ms 恰好落在
# 门槛的 CLAMP_MARGIN 倍处（S-aux4c）：
#   令三角形区（小行程，规划器与门槛公式都是 sqrt 支）：
#     2·sqrt(s/A_PLAN)/COMP == CLAMP_MARGIN · sqrt(s/A_MAX)
#   行程 s 两边完全约掉，解出：
#     A_PLAN = 4·A_MAX / (CLAMP_MARGIN·COMP)²   ≈ 161.09 °/s²
# 大行程（趋于纯速度受限）时规划器与门槛公式都收敛到同一个 s/V_MAX，A_PLAN 这个对
# 齐只对加速度受限区有效——见 clamp_threshold_sec 下面硬下限的说明，270° 附近这条
# 下限会在没有任何标定漂移的情况下被真实触发，这是设计的自然结果，不是 bug。
CLAMP_MARGIN = 1.05
A_PLAN = 4 * A_MAX / (CLAMP_MARGIN * COMP) ** 2   # 规划器画曲线用的加速度，≈161.09 °/s²

# 控制器侧真实限位（度，出处 marm_controller_node.cpp:176-177）：set_servo_angle
# （funcode=1 通路）收到的目标无条件截断到这套官方软限位，超出的机械臂会截到边界照走，
# 不是拒绝理由。规划器本身不做限位判定，这两张表和下面的截断/越界函数供 arm_driver
# （action 三层限位判定 + /set_servo 原地使能 clamp）统一 import，不留两份。
# 2026-09-23 Thomas 裁定放宽到他实测的安全行程（原厂值 ±135/±90/±90/±135/±90/±135）。
# **这两行必须与臂上 marm_controller_node.cpp:176-177 的 maxangle/minangle 逐位相同**：
# 那两个数组才是真正截断命令的地方，这里只是镜像，用来提前算出「控制器会截多少」。
# 臂上不改、只改这里 = 界面放行到 163°，臂停在 135° 且不报错。夹爪轴 a6 未改动。
# a6（夹爪）上沿 2026-09-23 从 50.0 放宽到 64.5（Thomas 裁定，与 grip_detect.GRIP_TARGET_MAX、
# 臂上 maxangle[6] 同步）。下沿 -20.0 未改——只放宽张开方向。
LIMIT_MAX = (163.0, 97.0, 89.0, 163.0, 113.0, 162.0, 64.5)
# a6 下沿 2026-09-24 从 -20.0 放宽到 -30.0（Thomas 裁定，与 grip_detect.GRIP_TARGET_MIN、
# 臂上 minangle[6] 同步；实测硬件行程下沿是 -30.4，-30 在其内）。
LIMIT_MIN = (-163.0, -90.3, -97.0, -166.0, -96.0, -166.0, -30.0)

# 实测硬件行程（度，Thomas 2026-09-10 手动测量，一次采样、±几度不确定）。超出它说明
# 读数/目标本身不可达（传感器异常，或调用方给了个机械臂到不了的值）——这是唯一该拒绝
# 的一层；落在 [HW_LIMIT, CTRL_LIMIT) 之间的目标，控制器自己会截到 CTRL_LIMIT 照走，
# 不该拒绝，只该照实回报截断量（见 clamp_to_ctrl）。
HW_LIMIT_MIN = (-164.0, -90.4, -96.5, -166.0, -97.3, -167.2, -30.4)
HW_LIMIT_MAX = (165.0, 98.0, 89.6, 163.0, 113.0, 162.9, 64.5)


def clamp_to_ctrl(deg7, axes):
    """把 deg7 中给定 axes 截到 CTRL_LIMIT（LIMIT_MIN/MAX）。

    返回 (7 元素列表：axes 内已截断、axes 外原样保留，
          按 axes 顺序排列的偏差列表：截断后 - 截断前，未截为 0.0)。
    偏差符号即"控制器会把目标往哪边推"：a1=-90.4 → 截到 -90.0 → 偏差 +0.4；
    a0=160 → 截到 135 → 偏差 -25.0。
    """
    out = [float(v) for v in deg7]
    devs = []
    for i in axes:
        c = max(LIMIT_MIN[i], min(LIMIT_MAX[i], out[i]))
        devs.append(c - out[i])
        out[i] = c
    return out, devs


def hw_out_of_limits(deg7, axes, margin=0.0):
    """给定 axes 中超出 HW_LIMIT ± margin 的轴。返回 [(轴号, 值), ...]，空 = 都在行程内。"""
    bad = []
    for i in axes:
        v = float(deg7[i])
        if v < HW_LIMIT_MIN[i] - margin or v > HW_LIMIT_MAX[i] + margin:
            bad.append((i, v))
    return bad


PlanResult = namedtuple(
    'PlanResult', ['duration_sec', 'delay_ms', 'speed_limited', 'per_axis_sec'])


def clamp_threshold_sec(s_max):
    """控制器会把这条指令削顶的最短允许时间（S-aux4c）。s_max 是本次规划里最大的
    单轴行程（度，7 轴共用一个 delay_ms，行程最大的那根轴决定会不会削顶）。

    alpha 撞 A_MAX 对应 sqrt(s_max/A_MAX)，omega 撞 V_MAX 对应 s_max/V_MAX，取更严
    （更大）的一个——delay_ms 换算出的时间低于它，传给控制器就会被削顶，此后
    delay_ms 再怎么变、结果都一样，speed_ratio/accel_ratio 静默失效，且从日志上
    看不出来（照常 SEND，只是不起作用）。
    """
    if s_max <= 0.0:
        return 0.0
    return max(math.sqrt(s_max / A_MAX), s_max / V_MAX)


def clamp_ratio(x):
    """把一个倍率 clamp 到 [RATIO_MIN, 1.0]。返回 (值, 是否被截断)。"""
    v = float(x)
    if v < RATIO_MIN:
        return RATIO_MIN, True
    if v > 1.0:
        return 1.0, True
    return v, False


def axis_min_time(stroke, v, a):
    """单轴走完 stroke（度，非负）在速度上限 v、加速度上限 a 下的最短时间。

    加速到 v 需要距离 d = v²/(2a)。stroke >= 2d 时是梯形（有匀速段），
    否则是三角形（还没加到 v 就要减速）。见任务书 §5.3。
    """
    if stroke <= 0.0:
        return 0.0
    d = v * v / (2.0 * a)
    if stroke >= 2.0 * d:
        return 2.0 * v / a + (stroke - 2.0 * d) / v
    return 2.0 * math.sqrt(stroke / a)


def plan(start_deg, target_deg, speed_ratio, accel_ratio, override, duration_sec=0.0):
    """规划一段时间同步梯形轨迹。

    start_deg / target_deg：7 元素序列，单位度。
    speed_ratio / accel_ratio / override：0~1 小数，越界按 [RATIO_MIN, 1.0] clamp。
    duration_sec：>0 时把 t_base（由 speed_ratio/accel_ratio 算出的最短可行时间）当
      地板，取 max(duration_sec, t_base) 作为总时长——**不是**忽略 speed_ratio/
      accel_ratio，地板本身仍由它们决定；调用方要让地板等于硬件真正的最短时间，
      必须自己把 speed_ratio/accel_ratio 都传 1.0（见 trajectory_server.
      speed_ratio_for_segment，S4a-1 复核修复）。比地板还快会被拉长；override 仍
      叠加。<=0（默认）走原逻辑：由 speed_ratio/accel_ratio 算出的最短时间即总时长。

    返回 PlanResult：
      duration_sec   全臂统一总时长 T
      delay_ms       统一下发值 = round(T*1000/COMP)，clamp 到 [1, 32767]，再按
                     clamp_threshold_sec() 兜一道硬下限（不越过控制器削顶门槛，
                     duration_sec 拉长出来的值同样要过这道下限）
      speed_limited  任一倍率被 clamp、delay_ms 撞 32767 上限、duration_sec 被
                     拉长、或撞了硬下限 → True
      per_axis_sec   每轴最短时间 T_i（诊断用）
    """
    sr, l1 = clamp_ratio(speed_ratio)
    ar, l2 = clamp_ratio(accel_ratio)
    ov, l3 = clamp_ratio(override)

    v = V_MAX * sr
    a = A_PLAN * ar   # 画曲线用规划加速度，不是控制器削顶门槛 A_MAX（S-aux4c，见文件头）

    per_axis = []
    s_max = 0.0
    for i in range(7):
        s_i = abs(float(target_deg[i]) - float(start_deg[i]))
        if s_i > s_max:
            s_max = s_i
        per_axis.append(axis_min_time(s_i, v, a))

    # 全臂统一总时长；1e-6 下限避免零行程时后续除法/取整异常。
    t_base = max(max(per_axis), 1e-6)

    # duration_sec>0：调用方（FollowJointTrajectory 逐段执行）要求一个具体时长——
    # 够宽就用它，比 t_base 还快就拉长到 t_base 并标记 speed_limited。这个 t_base
    # 地板仍由 speed_ratio/accel_ratio 决定（不是"忽略"两者，见上方 docstring）。
    l_dur = False
    if duration_sec > 0.0:
        t_req = max(float(duration_sec), t_base)
        l_dur = t_req > float(duration_sec) + 1e-9
    else:
        t_req = t_base

    # override 只是一行除法：位移不变，速度自动 ×k、加速度自动 ×k²，轨迹几何形状不变、
    # 多轴协调自动保持（与把 override 折进 v、override² 折进 a 数学等价，见 §8）。
    t = t_req / ov

    delay_ms = round(t * 1000.0 / COMP)
    l4 = False
    if delay_ms > DELAY_MS_MAX:
        delay_ms = DELAY_MS_MAX
        l4 = True
    if delay_ms < DELAY_MS_MIN:
        delay_ms = DELAY_MS_MIN

    # 硬下限：A_PLAN 只把三角形区（小到中等行程）对准到门槛的 CLAMP_MARGIN 倍；大行程
    # 趋于纯速度受限时规划器与门槛公式都收敛到同一个 s/V_MAX，这条对齐失效，此外 COMP
    # 一旦漂移也可能把 delay_ms 顶进削顶区——两种情况都在这一道兜住，且从不减小
    # delay_ms（只在算出来的值不够时才抬高，不影响正常范围内的时长）。
    thr_s = clamp_threshold_sec(s_max)
    delay_floor_ms = thr_s * 1000.0 * CLAMP_MARGIN
    l5 = False
    if delay_ms < delay_floor_ms:
        delay_ms = int(math.ceil(delay_floor_ms))
        l5 = True

    return PlanResult(
        duration_sec=t,
        delay_ms=int(delay_ms),
        speed_limited=(l1 or l2 or l3 or l4 or l5 or l_dur),
        per_axis_sec=per_axis)
