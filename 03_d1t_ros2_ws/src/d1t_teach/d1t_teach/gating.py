#!/usr/bin/env python3
"""gating.py — 「现在这条命令能不能下发」的判定（纯函数，无 ROS 依赖）。

分出来单独一份是为了能被 colcon test 直接测（TEACH-01 W8）：并发、反馈断开、
外部运动这几种拒绝行为不需要起 ROS 就能验。

判定顺序固定，先命中的先报，保证同一状态下理由唯一。
"""

# 命令类别
MOTION = 'motion'      # 臂轴运动：关节点动、绝对定位、笛卡尔增量、预设点位
GRIP = 'grip'          # 夹爪：/grip
SERVICE = 'service'    # /set_servo、/set_override
CONFIG = 'config'      # 只改界面状态：选 TCP、改步长、改开关值、记录/清空槽位

# /arm_Command 上允许出现的发布者：arm_driver、trajectory_server，
# 外加本界面自己——「按住即走」期间 teach_server 会自己建一个发布者直接发 funcode=1，
# 那是我们自己发的，不算外来运动（trajectory_server 那边的 ALLOWED_CMD_PUBLISHERS
# 不含 d1t_teach，所以按住期间它会拒绝 stream goal，这是有意的：两条通路互斥）。
ARM_COMMAND_PUBLISHERS = ('d1t_arm', 'd1t_trajectory', 'd1t_teach')


def check_command(kind, *, busy, feedback_stale, external_motion, missing_deps,
                  exec_in_flight=False):
    """返回 None = 可以下发；返回字符串 = 拒绝理由，界面明确报错且不下发命令。

    busy            本界面已有一条命令在飞（单条在飞，不排队）
    feedback_stale  最近一帧反馈太旧或从未收到
    external_motion 发现别的发布者 / 别人的目标 / 空闲时臂自己在动
    missing_deps    还没就绪的依赖名字列表
    exec_in_flight  轨迹正在执行（此时不许调 /set_servo、/set_override、/set_stiffness）
    """
    if kind == CONFIG:
        return None
    if busy:
        return '上一条动作还没结束，不接受新命令（单条在飞，不排队）'
    if feedback_stale:
        return '反馈已断：没有新鲜的 /d1t/joint_feedback，不下发任何命令'
    if external_motion:
        return '发现本界面之外的运动或发布者在动臂，拒绝下发（不得与其它运动发布者并发）'
    if kind in (SERVICE,) and exec_in_flight:
        return '轨迹执行期间不得调用 /set_servo、/set_override、/set_stiffness'
    if kind == MOTION and missing_deps:
        return '依赖未就绪：%s' % '、'.join(missing_deps)
    if kind == GRIP and missing_deps and '/grip' in missing_deps:
        return '依赖未就绪：/grip'
    return None


def idle_drift_verdict(ref_deg, cur_deg, ref_age_sec, win_sec, net_deg):
    """空闲时「臂是不是被别人在动」的判定，看净位移不看单帧差。

    ref_deg 为 None = 还没有基准。返回 'reset'（把当前帧设成新基准）、
    'flag'（判定为外部运动，同时换基准）、'keep'（继续用老基准比）。

    2026-09-23 实机：臂停住时伺服会自己抖，单帧差 0.6~0.7° 一直有，净位移接近 0。
    别人真在动臂是单向走的，净位移会累积——这就是两者的区别。
    """
    if ref_deg is None or ref_age_sec > win_sec:
        return 'reset'
    net = max(abs(cur_deg[i] - ref_deg[i]) for i in range(len(ref_deg)))
    return 'flag' if net > net_deg else 'keep'


def external_publisher_names(publisher_node_names):
    """/arm_Command 上除 arm_driver、trajectory_server 之外的发布者名字。"""
    return sorted({name for name in publisher_node_names
                   if name.lstrip('/').split('/')[-1] not in ARM_COMMAND_PUBLISHERS})


def feedback_is_stale(age_sec, stale_after_sec):
    """age_sec 为 None = 一帧都没收到，同样算断。"""
    return age_sec is None or age_sec > stale_after_sec
