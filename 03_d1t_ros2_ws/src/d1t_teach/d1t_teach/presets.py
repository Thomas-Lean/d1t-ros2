#!/usr/bin/env python3
"""presets.py — TEACH-01 关节预设点位与夹爪开 / 关值的持久化（无 ROS 依赖）。

文件：默认 ~/.d1t_teach/presets.json，JSON，重启后还在。
内容：3 组用户槽位 + 夹爪开值 / 关值。
两个固定预设（0 点、折叠姿态）写死在代码里，不进文件、不可改。

每个用户槽位都存 7 个控制器轴值（a0~a5 + angle6_raw）；记笛卡尔位姿时额外存 pose
（Thomas 2026-09-23 裁定第三条）。两者同时存不是冗余：到达笛卡尔槽位要解逆解，
记录时的关节值就是那次逆解的种子，用它才能回到**当初那一支解**（同一位姿可以有
肘上肘下等多支解，换一支解路径完全不同）。
"""

import json
import os

from . import units

DEFAULT_PATH = os.path.join(os.path.expanduser('~'), '.d1t_teach', 'presets.json')

# 固定预设：deg6 是 a0~a5；grip=None 表示夹爪保持当前值、不下发夹爪命令
# confirm=True 的预设要二次确认才下发（主窗口 2026-09-22 裁定 W4·2c）：
# 这两个都是大幅度整臂运动，误点一下代价太大。用户槽位不要求。
FIXED_PRESETS = [
    {'id': 'zero', 'name': '0 点', 'deg6': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'grip': None,
     'confirm': True,
     'note': '全轴 0，接近满臂展。起点不限（Thomas 2026-09-23 取消「只许从折叠姿态出发」）'},
    # a2 原是 90.0，Thomas 2026-09-23 把 a2 正限位放宽到实测行程时定的上界是 89（与
    # HOME_DEG 同步改，理由见 arm_driver.py 的 HOME_DEG 注释）：90.0 已超出新限位，
    # 90.0 会被 check_joint_target 直接拒绝、这个预设点不了。改成新限位的边界值 89.0。
    {'id': 'fold', 'name': '折叠姿态', 'deg6': [0.0, -90.0, 89.0, 0.0, 0.0, 0.0], 'grip': None,
     'confirm': True,
     'note': '夹爪保持当前值。折叠区内 a4 只能走负方向'},
]

USER_SLOT_IDS = ['slot1', 'slot2', 'slot3']
DEFAULT_SLOT_NAMES = {'slot1': '预抓取', 'slot2': '用户槽位 2', 'slot3': '用户槽位 3'}

# 开值默认取命令域上沿：2026-09-23 命令域从 50 放宽到 64.5 之后，50 只是「旧上限」，
# 没有别的含义了。注意这只是**新文件**的默认值——已经存在的 presets.json 里存的还是
# 旧值，要在界面「预设点位」页的「夹爪开合值」里改，或者删掉文件重新生成。
DEFAULT_GRIP_OPEN_RAW = 64.5
DEFAULT_GRIP_CLOSE_RAW = 29.0


def default_data():
    return {
        'version': 1,
        'grip': {'open_raw': DEFAULT_GRIP_OPEN_RAW, 'close_raw': DEFAULT_GRIP_CLOSE_RAW},
        'slots': {sid: {'name': DEFAULT_SLOT_NAMES[sid], 'deg7': None, 'pose': None}
                  for sid in USER_SLOT_IDS},
    }


def clean_pose(raw):
    """校验并规整一个笛卡尔位姿记录。非法一律返回 None（当成没记过笛卡尔）。

    形如 {'tcp': 'grip_center', 'p_m': [x, y, z], 'q': [x, y, z, w]}，
    位置单位米、四元数已归一化——都是内部 SI 域的值，界面显示时才换成 mm / 度。
    """
    if not isinstance(raw, dict):
        return None
    tcp = raw.get('tcp')
    if not isinstance(tcp, str) or not tcp.strip():
        return None
    try:
        p = [float(v) for v in raw['p_m']]
        q = [float(v) for v in raw['q']]
    except (TypeError, ValueError, KeyError):
        return None
    if len(p) != 3 or len(q) != 4:
        return None
    if not all(abs(v) < 10.0 for v in p):          # 米：超过 10 m 一定是坏数据
        return None
    if sum(v * v for v in q) < 1e-9:
        return None
    return {'tcp': tcp.strip()[:32], 'p_m': p, 'q': list(units.quat_normalize(q))}


def _clean_slot(sid, raw):
    slot = {'name': DEFAULT_SLOT_NAMES[sid], 'deg7': None, 'pose': None}
    if not isinstance(raw, dict):
        return slot
    slot['pose'] = clean_pose(raw.get('pose'))
    name = raw.get('name')
    if isinstance(name, str) and name.strip():
        slot['name'] = name.strip()[:24]
    deg7 = raw.get('deg7')
    if isinstance(deg7, list) and len(deg7) == 7:
        try:
            values = [float(v) for v in deg7]
        except (TypeError, ValueError):
            return slot
        if units.check_joint_target(values[:6]) is None:
            slot['deg7'] = [units.round1(v) for v in values]
    return slot


def load(path=DEFAULT_PATH):
    """读预设文件。文件不存在、损坏、字段非法都退回默认值，不抛异常。"""
    data = default_data()
    try:
        with open(path, 'r', encoding='utf-8') as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return data
    if not isinstance(raw, dict):
        return data
    grip = raw.get('grip')
    if isinstance(grip, dict):
        for key, default in (('open_raw', DEFAULT_GRIP_OPEN_RAW),
                             ('close_raw', DEFAULT_GRIP_CLOSE_RAW)):
            value = grip.get(key, default)
            data['grip'][key] = units.round1(value) if units.check_grip_raw(value) is None else default
    slots = raw.get('slots') if isinstance(raw.get('slots'), dict) else {}
    for sid in USER_SLOT_IDS:
        data['slots'][sid] = _clean_slot(sid, slots.get(sid))
    return data


def save(data, path=DEFAULT_PATH):
    """原子写：先写同目录临时文件再 rename，断电不会留下半截 JSON。"""
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write('\n')
    os.replace(tmp, path)
    return path


def set_slot(data, slot_id, deg7, name=None, pose=None):
    """把当前位置记进某个用户槽位。越界或字段不对返回错误原因字符串。

    pose 为 None = 只记关节；给了就同时记笛卡尔位姿（关节值当逆解种子留着）。
    """
    if slot_id not in USER_SLOT_IDS:
        return '槽位 %s 不存在' % slot_id
    if len(deg7) != 7:
        return '记录位置需要 7 个控制器轴值（a0~a5 + angle6_raw），收到 %d 个' % len(deg7)
    try:
        values = [float(v) for v in deg7]
    except (TypeError, ValueError):
        return '记录位置里有非数字'
    err = units.check_joint_target(values[:6])
    if err:
        return '当前位置不能记录：%s' % err
    clean = None
    if pose is not None:
        clean = clean_pose(pose)
        if clean is None:
            return '笛卡尔位姿记录不合法（要 tcp + 3 个位置米值 + 4 个四元数分量）'
    slot = data['slots'][slot_id]
    slot['deg7'] = [units.round1(v) for v in values]
    slot['pose'] = clean
    if name is not None and str(name).strip():
        slot['name'] = str(name).strip()[:24]
    return None


def clear_slot(data, slot_id):
    if slot_id not in USER_SLOT_IDS:
        return '槽位 %s 不存在' % slot_id
    data['slots'][slot_id]['deg7'] = None
    data['slots'][slot_id]['pose'] = None
    return None


def set_grip_values(data, open_raw, close_raw):
    for label, value in (('开值', open_raw), ('关值', close_raw)):
        err = units.check_grip_raw(value)
        if err:
            return '夹爪%s：%s' % (label, err)
    data['grip']['open_raw'] = units.round1(open_raw)
    data['grip']['close_raw'] = units.round1(close_raw)
    return None


def list_presets(data):
    """固定预设 + 用户槽位，给界面用的统一列表。

    kind='cart' 只在既有位姿又有关节种子时成立——没有种子就解不回当初那一支解，
    这种槽位退回按关节到达（正常流程不会出现，是手改坏了 JSON 的兜底）。
    """
    out = [dict(p, fixed=True, empty=False, kind='joint', pose=None) for p in FIXED_PRESETS]
    for sid in USER_SLOT_IDS:
        slot = data['slots'][sid]
        deg7 = slot['deg7']
        pose = slot.get('pose') if deg7 else None
        out.append({'id': sid, 'name': slot['name'], 'fixed': False, 'confirm': False,
                    'empty': deg7 is None,
                    'kind': 'cart' if pose else 'joint',
                    'pose': pose,
                    'deg6': deg7[:6] if deg7 else None,
                    'grip': deg7[6] if deg7 else None,
                    'note': '' if deg7 else '未记录'})
    return out


def find_preset(data, preset_id):
    for preset in list_presets(data):
        if preset['id'] == preset_id:
            return preset
    return None
