# flyby.py — S-ovl4：trajectory_server 的 stream 模式（经由点不停车）用到的纯函数，
# 无 ROS 依赖，可单测。
#
# 来源：S-ovl3 gate_demo 里已在实机验证过的「不插值 + 进区域切换」逻辑（moving_axes、
# _wait_zone 的判据、STALL 判轴、funcode=1 报文格式、1.27×delay+0.5 s 兜底）。
# gate_demo 是已验收的证据源，不动它、也不 import 它——需要的纯函数在这里另存一份，
# 与 gate_demo 的版本保持同义（改这里不影响 gate_demo，反之亦然）。
#
# 新增（S-ovl4 才需要）：
#   thin_indices    执行侧轨迹抽稀（MoveIt 轨迹 46~193 点 -> 只留拐点）
#   corner_speeds   拐角处全轴最大速度，算法照 S-ovl3 主窗口复核口径
#   elapsed_stats   每段实际耗时分布
#   foreign_publishers  /arm_Command 入口独占检查

import json
import math

from d1t_arm.trapezoid_planner import COMP

# 厂商控制器对单条 funcode=1 的削顶值，照抄 marm_controller_node.cpp subServoAngle_callback：
#   float maxalpha = 286.4789 / 4;   float maxomega = 28.64789 * 4;
CTRL_MAX_OMEGA_DEG_S = 28.64789 * 4     # 114.59 °/s
CTRL_MAX_ALPHA_DEG_S2 = 286.4789 / 4    # 71.62 °/s²
CTRL_TIME_FACTOR = 1.8                  # 源码里那个 × 1800（毫秒→秒后是 ×1.8）

NOMINAL_MOVE_EPS_DEG = 0.5      # 按名义路点差判「本段动轴」的阈值（S-ovl3 §A）
ZONE_DEG_DEFAULT = 5.0          # 经由点「进区域」半径（S-ovl3 裁定起始值，实机 5 圈验过）
FALLBACK_MARGIN_SEC = 0.5       # 兜底 = COMP × delay_ms + 这个余量
STALL_AXIS_EPS_DEG = 1.0        # 超时后「没离开起点」的判轴阈值（S-ovl3 §D）
MOVE_TIMEOUT_MARGIN_SEC = 3.0   # 最后一点 /move_to_joints 超时 = 规划 T + 这个余量（S-ovl3 §D）
THIN_TOL_DEG_DEFAULT = 1.0      # 抽稀容差：被删的点离保留弦线的最大单轴偏差
CORNER_HALF_WINDOW_SEC = 0.5    # 拐角速度窗口 = 切换时刻 ±0.5 s（S-ovl3 验收口径）


def controller_move_sec(delta_deg, delay_ms):
    """一条 funcode=1 发下去，控制器实际会走多久（秒）。

    照抄 marm_controller_node.cpp 的 subServoAngle_callback：

        omega = |Δ| / delay_ms * 1000;            omega = min(omega, maxomega)
        alpha = omega / delay_ms * 1000;          alpha = min(alpha, maxalpha)
        testdelay_ms = ((sqrt(omega² + 2·|Δ|·alpha) - omega) / alpha) * 1800
        servo[id].setRawAngle(angle, testdelay_ms)

    为什么要算它：delay_ms **不是**这条命令的实际时长。加速度被削到 71.62 °/s² 之后，
    实际时长可以比 delay_ms 长 50% 以上（实测 3.2° / delay_ms=127 → 198 ms）。
    「按住即走」要想不顿挫，必须在上一条**走完之前**发下一条（重叠），
    所以得先知道上一条要走多久——这是那个判据的依据。
    """
    d = abs(float(delta_deg))
    delay = max(1, int(delay_ms))
    if d <= 0.0:
        return 0.0
    omega = min(d / delay * 1000.0, CTRL_MAX_OMEGA_DEG_S)
    alpha = min(omega / delay * 1000.0, CTRL_MAX_ALPHA_DEG_S2)
    if alpha <= 0.0:
        return 0.0
    return ((math.sqrt(omega * omega + 2.0 * d * alpha) - omega) / alpha) * CTRL_TIME_FACTOR


def moving_axes(start6, target6, eps=NOMINAL_MOVE_EPS_DEG):
    """位移超过 eps 的轴号列表（0..5）。stream 模式传的是名义路点，不是反馈。"""
    return [i for i in range(6) if abs(target6[i] - start6[i]) > eps]


def in_zone(cur_deg, target_deg, axes, zone_deg):
    """axes 里每根轴的 |反馈 − 目标| 都 ≤ zone_deg。"""
    return all(abs(cur_deg[a] - target_deg[a]) <= zone_deg for a in axes)


def fallback_timeout_sec(delay_ms):
    """没进区域时的兜底等待：控制器实际走完 = COMP × delay_ms，再加 0.5 s。"""
    return COMP * delay_ms / 1000.0 + FALLBACK_MARGIN_SEC


def stalled_axes(start_deg, cur_deg, axes, eps=STALL_AXIS_EPS_DEG):
    """axes 里离开起点不到 eps 的轴——超时后用来记 STALL，不做判定。"""
    return [a for a in axes if abs(cur_deg[a] - start_deg[a]) < eps]


def f1_command_json(seq, axis, angle_deg, delay_ms):
    """funcode=1 单关节报文，字段与 arm_driver._send_planned 一致。"""
    return json.dumps({
        'seq': int(seq), 'address': 1, 'funcode': 1,
        'data': {'id': int(axis), 'angle': float(angle_deg), 'delay_ms': int(delay_ms)}})


def _chord_deviation(a, b, q):
    """点 q 到线段 a→b 的偏差：先在关节空间把 q 投影到线段上（t 夹到 [0,1]），
    再取投影点与 q 的最大单轴差（度）。a==b 时退化为 q 到 a 的最大单轴差。"""
    d = [bb - aa for aa, bb in zip(a, b)]
    dd = sum(x * x for x in d)
    if dd <= 0.0:
        t = 0.0
    else:
        t = sum((qq - aa) * x for aa, qq, x in zip(a, q, d)) / dd
        t = max(0.0, min(1.0, t))
    return max(abs(qq - (aa + t * x)) for aa, qq, x in zip(a, q, d))


def thin_indices(points_deg, start_deg, tol_deg):
    """执行侧抽稀：返回要保留的点下标（升序），最后一点永远保留。

    贪心：从锚点（起点是反馈快照）往后延伸，只要锚点到候选点这根弦线能把中间所有点
    都罩在 tol_deg 以内，就继续延伸；罩不住了，就把上一个点定为新锚点（保留）。
    结果：关节空间里接近直线的一串点合成一段，拐点留下。
    tol_deg <= 0 -> 不抽稀，全保留。
    """
    n = len(points_deg)
    if n == 0:
        return []
    if tol_deg <= 0.0:
        return list(range(n))
    kept = []
    anchor = list(start_deg)
    first = 0          # 当前弦线覆盖的第一个点下标
    j = 0
    while j < n:
        ok = all(_chord_deviation(anchor, points_deg[j], points_deg[k]) <= tol_deg
                 for k in range(first, j))
        if ok:
            j += 1
            continue
        kept.append(j - 1)
        anchor = list(points_deg[j - 1])
        first = j
        # 不 j += 1：用新锚点重判 j
    kept.append(n - 1)
    return kept


def path_deviation(rows, dense_path, kept_path, start_deg=None):
    """实际反馈路径相对规划折线的最大单轴偏差（度）。S4b-3 任务③采数用。

    rows: [(t_sec, deg7), ...] 实机反馈采样；dense_path: 抽稀前全部规划路点（deg6，
    控制器坐标系）；kept_path: 抽稀后保留路点（deg6）；start_deg: 执行起点（deg6，
    缺省时 kept_path 不加起点锚点）。对每个实际点 q，取 q 到折线最近一段的最大单轴
    偏差；全轨迹取最大。单点路径退化为点到点的最大单轴差。

    dense_path 是 MoveIt 碰撞检查过的路径，kept_path 是实际执行折线（执行时是
    起点→kept[0]→kept[1]→…，所以折线要含起点锚点）——两个数分开报，才能分辨偏差
    来自抽稀还是来自进区域切角 + 臂自身跟踪误差。
    """
    def max_dev(poly):
        if not poly or not rows:
            return 0.0
        if len(poly) == 1:
            return max(max(abs(q[a] - poly[0][a]) for a in range(6)) for _, q in rows)
        best = 0.0
        for _, q in rows:
            d = min(_chord_deviation(a, b, q[:6]) for a, b in zip(poly, poly[1:]))
            best = max(best, d)
        return best
    if start_deg is not None:
        kept_path = [list(start_deg)] + list(kept_path)
    return max_dev(dense_path), max_dev(kept_path)


def corner_speeds(rows, switch_times, half_window=CORNER_HALF_WINDOW_SEC):
    """rows：[(t_sec, deg6), ...] 按时间升序的反馈。switch_times：每个经由点的切换时刻。

    对每个切换时刻，在 [t−half_window, t+half_window] 内取相邻两帧算各轴速度
    |Δ角度/Δt|，返回窗口内全轴最大值（°/s）；窗口里不足两帧 -> None。
    口径照 S-ovl3 主窗口复核（「切换前后 ±0.5 s 窗口内全轴最大速度」）。
    """
    out = []
    for ts in switch_times:
        win = [r for r in rows if ts - half_window <= r[0] <= ts + half_window]
        best = None
        for (t0, p0), (t1, p1) in zip(win, win[1:]):
            dt = t1 - t0
            if dt <= 0.0:
                continue
            v = max(abs(b - a) for a, b in zip(p0[:6], p1[:6])) / dt
            best = v if best is None else max(best, v)
        out.append(best)
    return out


def elapsed_stats(values):
    """(段数, 合计, 最小, 中位, 最大)；空列表全 0。"""
    if not values:
        return 0, 0.0, 0.0, 0.0, 0.0
    s = sorted(values)
    n = len(s)
    med = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
    return n, float(sum(s)), s[0], med, s[-1]


def foreign_publishers(endpoints, allowed_node_names):
    """endpoints：[(node_namespace, node_name), ...]，来自 get_publishers_info_by_topic。
    返回不在 allowed_node_names 里的发布者，格式 '<ns>/<name>'（去重、保持顺序）。"""
    out = []
    for ns, name in endpoints:
        if name in allowed_node_names:
            continue
        full = (ns.rstrip('/') + '/' + name) if ns and ns != '/' else '/' + name
        if full not in out:
            out.append(full)
    return out

