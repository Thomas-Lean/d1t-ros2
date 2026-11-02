# grip_detect.py — GRIP-01 夹取判定纯函数（无 ROS 依赖，可单测）：只靠夹爪位置反馈 angle6_raw，
# 比较「命令目标」与「稳定后停住值」判 空夹 / 夹住 / 说不准 / 夹爪没动。
#
# 依据（实机，2026-09-17，logs 见 forlearning/GRIP-01-grasp-detect/logs/real/）：
#   空夹：停住值 - 目标 ∈ [-0.3, +0.5]（HOME 位 6 个目标各 5 次闭合 + 31 次张开；夹取姿态目标 29
#         5 次闭合 + 6 次张开；同目标极差 ≤ 0.1）
#   夹娃哈哈瓶身（夹取姿态）：目标 29 → +4.0/+4.1；31 → +2.8/+3.0；33 → +2.1/+2.2；35 → +1.3/+1.7；
#         37 → +0.6/+1.0（37 与空夹区间重叠：目标给得太松，位置信号分不开）
#   夹爪没动：张开命令回执、起动自救重发后仍停在原处（夹取姿态标定第 6 次）——不查这个，
#         「从 49.7 闭合到 29 却卡在 49.7」会被当成误差 +20.7 判成「夹住」。
# 阈值只在娃哈哈、目标 29~33 上有数据支撑；其它物品、其它目标未验证。

import math
import statistics
from collections import namedtuple

GripConfig = namedtuple('GripConfig', [
    'empty_max_err_deg',     # |停住 - 目标| <= 此值 → empty（空夹 / 张开到位）
    'grasped_min_err_deg',   # 停住 - 目标 >= 此值 → grasped（被物体挡住）
    'move_expect_deg',       # 命令行程 >= 此值才检查「动没动」
    'moved_min_deg',         # 实际离开起点 < 此值 → not_moved
    'settle_window_sec', 'settle_band_deg', 'settle_min_frames'])

# 1.0：空夹实测最大 |误差| 0.5，留 0.5；2.0：娃哈哈目标 29 实测 +4.0，留 2.0，33 的 +2.1 刚过线。
# 0.5：反馈量化 0.1，真卡死实测位移 0。settle 三项与 grip_probe 实机采数时一致（61+11+22 步全部判出停住）。
DEFAULT_CONFIG = GripConfig(
    empty_max_err_deg=1.0, grasped_min_err_deg=2.0,
    move_expect_deg=2.0, moved_min_deg=0.5,
    settle_window_sec=1.0, settle_band_deg=0.25, settle_min_frames=5)

# 夹爪目标只许 [-30, 64.5]：上沿 = 控制器限位上沿（Thomas 2026-09-23 从 50 放宽到 64.5，
# 与 trapezoid_planner.LIMIT_MAX[6]、臂上 marm_controller_node.cpp maxangle[6] 三处同步——
# 原 50 是当时的控制器限位，不是夹爪本身的量程；HW_LIMIT_MAX[6]=64.5 早就测出舵机能转到
# 这里，只是命令层面一直被 50 卡住没人试过）；20 以下没做过实验（百岁山合到 20 Thomas 判太紧），
# 这次没改，只放宽张开方向。
# 下沿 2026-09-24 从 20 放宽到 -30（Thomas 裁定：夹爪开合完全由他自定义，不要软件替他挡）。
# -30 已经低于臂上原控制器下沿 -20，必须同步改臂上 maxangle/minangle 第 7 个元素，
# 否则命令被静默截回 -20（见 ARM-LIMITS-checklist.md 第 11 节）。负区间没有标定与实验，
# 「夹住/没夹住」判定是按 20~50 段调的，负区间下不保证准。
GRIP_TARGET_MIN = -30.0
GRIP_TARGET_MAX = 64.5


def grip_target_ok(target):
    return GRIP_TARGET_MIN <= float(target) <= GRIP_TARGET_MAX


def settle_window(samples, t_from, t_now, window_sec, band_deg, min_frames):
    """停住判据。samples = [(t, angle6_raw)] 按时间递增。

    只看 [max(t_from, t_now - window_sec), t_now] 内的样本。t_now - t_from 不足一个窗口、
    帧数不足 min_frames、或极差 > band_deg，都返回 None；否则返回
    (中位数, 最小, 最大, 帧数)。"""
    if t_now - t_from < window_sec:
        return None
    lo = t_now - window_sec
    vals = [v for t, v in samples if lo <= t <= t_now]
    if len(vals) < min_frames or max(vals) - min(vals) > band_deg:
        return None
    return statistics.median(vals), min(vals), max(vals), len(vals)


def last_window(samples, t_from, t_now, window_sec):
    """超时兜底：返回同一窗口的 (中位数, 最小, 最大, 帧数)，不看极差；没有样本返回 None。"""
    lo = max(t_from, t_now - window_sec)
    vals = [v for t, v in samples if lo <= t <= t_now]
    if not vals:
        return None
    return statistics.median(vals), min(vals), max(vals), len(vals)


def classify_grip(target, start, settled, cfg=DEFAULT_CONFIG):
    """停住后分类：not_moved / empty / grasped / uncertain。"""
    if (abs(target - start) >= cfg.move_expect_deg
            and abs(settled - start) < cfg.moved_min_deg):
        return 'not_moved'
    err = settled - target
    if abs(err) <= cfg.empty_max_err_deg:
        return 'empty'
    if err >= cfg.grasped_min_err_deg:
        return 'grasped'
    return 'uncertain'


def evaluate_grip(samples, target, start, t_from, t_now, t_deadline, cfg=DEFAULT_CONFIG):
    """每拍调用一次。还要等 → None；否则 (reason, state, settled)：
      ('ok', not_moved/empty/grasped/uncertain, 停住值)
      ('not_settled', '', 最后一个窗口中位数)  到 t_deadline 仍没停稳
      ('no_feedback', '', nan)                 到 t_deadline 窗口里一帧反馈都没有
    """
    win = settle_window(samples, t_from, t_now, cfg.settle_window_sec,
                        cfg.settle_band_deg, cfg.settle_min_frames)
    if win is not None:
        return 'ok', classify_grip(target, start, win[0], cfg), win[0]
    if t_now < t_deadline:
        return None
    lw = last_window(samples, t_from, t_now, cfg.settle_window_sec)
    if lw is None:
        return 'no_feedback', '', math.nan
    return 'not_settled', '', lw[0]


def _fresh_stable_window(samples, t_from, t_now, window_sec, cfg, max_gap_sec):
    """A full observed stable interval: finite, ordered, fresh and without gaps.

    Include the sample immediately before the interval boundary, so a high-rate
    burst shorter than window_sec cannot substitute for elapsed stable feedback.
    """
    frames = [(t, v) for t, v in samples if t >= t_from]
    if (not frames or any(not math.isfinite(x) for row in frames for x in row)
            or any(b[0] <= a[0] for a, b in zip(frames, frames[1:]))
            or frames[-1][0] > t_now or t_now - frames[-1][0] > max_gap_sec):
        return None
    boundary = frames[-1][0] - window_sec
    before = [i for i, (t, _) in enumerate(frames) if t <= boundary]
    if not before:
        return None
    tail = frames[before[-1]:]
    if (len(tail) < max(5, cfg.settle_min_frames)
            or tail[-1][0] - tail[0][0] < window_sec
            or any(b[0] - a[0] > max_gap_sec for a, b in zip(tail, tail[1:]))):
        return None
    values = [v for _, v in tail]
    if max(values) - min(values) > cfg.settle_band_deg:
        return None
    return statistics.median(values), min(values), max(values)


def evaluate_grip_wait(samples, target, start, command_start, planned_end, now,
                       deadline, cfg=DEFAULT_CONFIG, *, early_confirm=False,
                       open_confirm_sec=0.3, feedback_max_gap_sec=0.25):
    """Optional feedback-driven wait; disabled preserves evaluate_grip exactly.

    RECV acceptance remains the driver's responsibility. A position plateau is
    evidence of blocked closure, not a force sensor or proof of secure holding.
    """
    if not early_confirm:
        return evaluate_grip(samples, target, start, planned_end, now, deadline, cfg)
    window_sec = open_confirm_sec if target > start else cfg.settle_window_sec
    window = _fresh_stable_window(samples, command_start, now, window_sec, cfg,
                                  feedback_max_gap_sec)
    if now < deadline and window is not None:
        settled, low, high = window
        state = classify_grip(target, start, settled, cfg)
        if target > start and state == 'empty':
            # All stable frames must be in band, not merely their median.
            if max(abs(low - target), abs(high - target)) <= cfg.empty_max_err_deg:
                return 'ok', 'empty', settled
        if (target < start and start - settled >= max(cfg.move_expect_deg, cfg.moved_min_deg)
                and state == 'grasped'):
            return 'ok', 'grasped', settled
    out = evaluate_grip(samples, target, start, planned_end, now, deadline, cfg)
    # The opt-in strategy must never succeed from a stale/sparse/invalid window,
    # including when it falls back to empty/not_moved/uncertain classification.
    full = _fresh_stable_window(samples, planned_end, now, cfg.settle_window_sec,
                                cfg, feedback_max_gap_sec)
    if out is not None and out[0] == 'ok' and full is not None:
        return out
    if now < deadline:
        return None
    fresh = [(t, v) for t, v in samples if max(command_start, now - feedback_max_gap_sec) <= t <= now
             and math.isfinite(t) and math.isfinite(v)]
    if not fresh:
        return 'no_feedback', '', math.nan
    finite_samples = [(t, v) for t, v in samples if math.isfinite(t) and math.isfinite(v)]
    last = last_window(finite_samples, command_start, now, cfg.settle_window_sec)
    if last is None:
        return 'no_feedback', '', math.nan
    return 'not_settled', '', last[0]
