"""Versioned measurement bundles. JSON metadata + lossless NPY images, no pickle."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid
import numpy as np
from .geometry import (ValidationError, camera_model, finite, frame, positive,
                       read_transform, required_text)

ARM_JOINTS = [f'Joint{i}' for i in range(1, 7)]
CAMERA_STREAMS = ('color', 'depth', 'color_info', 'depth_info')


def json_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()


def digest(value):
    return hashlib.sha256(json_bytes(value)).hexdigest()


def read_json(path):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError) as exc:
        raise ValidationError(f'cannot read JSON {path}: {exc}') from exc


def write_new_json(path, value):
    """Complete serialization before a no-clobber, atomic link into place."""
    data = json_bytes(value)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.pending-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.link(name, path)  # fails if ANY prior output exists
    finally:
        os.unlink(name)


def stamp(value):
    if type(value) is not int or value <= 0:
        raise ValidationError('nonzero integer ROS nanosecond timestamp required')
    return value


def validate_board(board, test_only=False):
    if board['kind'] != 'charuco' or board['dictionary'] != 'DICT_4X4_50':
        raise ValidationError('supported board: ChArUco DICT_4X4_50')
    for key in ('squares_x', 'squares_y'):
        if type(board[key]) is not int or not 3 <= board[key] <= 10:
            raise ValidationError('board squares must be integers in [3,10]')
    if board['squares_x'] * board['squares_y'] // 2 > 50:
        raise ValidationError('board needs more markers than dictionary contains')
    for key in ('square_length_m', 'marker_length_m', 'nominal_square_length_m', 'print_scale'):
        positive(board[key], key)
    if board['marker_length_m'] >= board['square_length_m']:
        raise ValidationError('marker must fit inside square')
    if not np.isclose(board['square_length_m'], board['nominal_square_length_m'] * board['print_scale'], rtol=.01):
        raise ValidationError('measured square/nominal square/print scale differ by over 1%; remeasure')
    if board['origin'] != 'outer_top_left_x_right_y_down_z_cross':
        raise ValidationError('unsupported board coordinate convention')
    if type(board['legacy_pattern']) is not bool or type(board['measured']) is not bool:
        raise ValidationError('board legacy_pattern and measured must be booleans')
    required_text(board['board_id'], 'board physical ID')
    required_text(board['measurement_note'], 'board measurement source')
    if not test_only and board['measured'] is not True:
        raise ValidationError('real samples require physically verified board dimensions')


def board_points(board, ids):
    ids = np.asarray(ids)
    n = (board['squares_x']-1) * (board['squares_y']-1)
    if ids.ndim != 1 or ids.dtype.kind not in 'iu' or len(set(ids.tolist())) != len(ids) or np.any((ids < 0) | (ids >= n)):
        raise ValidationError('invalid/duplicate ChArUco corner IDs')
    s = board['square_length_m']
    return np.column_stack(((ids % (board['squares_x']-1)+1)*s,
                            (ids // (board['squares_x']-1)+1)*s, np.zeros(len(ids))))


def info_signature(info):
    return {k: v for k, v in info.items() if k not in ('stamp_ns', 'received_ns', 'stamp_semantics')}


def validate_policy(policy):
    for key in ('max_age_ms', 'max_camera_skew_ms', 'max_joint_skew_ms',
                'max_joint_gap_ms', 'stationary_window_ms', 'stationary_tolerance_rad'):
        positive(policy[key], key)
    if policy['stationary_window_ms'] < 2 * policy['max_joint_gap_ms']:
        raise ValidationError('stationary window must span at least two allowed feedback gaps')


def joint_units(names):
    known = dict.fromkeys(ARM_JOINTS, 'rad') | {'Joint_L': 'm', 'Joint_R': 'm'}
    if any(n not in known for n in names):
        raise ValidationError('unknown joint type/unit: only this D1-T URDF joints are supported')
    return [known[n] for n in names]


def joint_vector(joint):
    names = joint['names']
    if not isinstance(names, list) or any(not isinstance(n, str) for n in names) or len(set(names)) != len(names):
        raise ValidationError('joint names missing/duplicated')
    values = finite(joint['position'], (len(names),), 'actual joint positions')
    if joint['position_units'] != joint_units(names):
        raise ValidationError('joint position units must match URDF: revolute rad, prismatic m')
    if not set(ARM_JOINTS).issubset(names):
        raise ValidationError('actual Joint1..Joint6 required')
    return values[[names.index(n) for n in ARM_JOINTS]]


def check_time(record, now_ns, policy):
    t, received = stamp(record['stamp_ns']), stamp(record['received_ns'])
    required_text(record['stamp_semantics'], 'timestamp semantics')
    age = policy['max_age_ms'] * 1e6
    if not 0 <= now_ns-t <= age or not 0 <= now_ns-received <= age:
        raise ValidationError('stale/future data or incompatible ROS clocks')
    return t


def validate_timing(meta, now_ns=None):
    policy = meta['pairing_policy']
    validate_policy(policy)
    now = stamp(meta['capture_ros_ns']) if now_ns is None else stamp(now_ns)
    clock = meta['clock']
    if clock['same_ros_clock_verified'] is not True:
        raise ValidationError('image/joint ROS clock correspondence must be verified')
    required_text(clock['evidence'], 'clock verification evidence')
    times = [check_time(meta[s], now, policy) for s in CAMERA_STREAMS]
    if max(times)-min(times) > policy['max_camera_skew_ms']*1e6:
        raise ValidationError('color/depth/CameraInfo timestamp mismatch')
    t_image = times[0]
    joint = meta['joint_state']
    tj = check_time(joint, now, policy)
    selected = joint_vector(joint)
    if abs(t_image-tj) > policy['max_joint_skew_ms']*1e6:
        raise ValidationError('joint/image timestamp mismatch')
    history = meta['joint_history']
    if len(history) < 3:
        raise ValidationError('at least three actual feedback messages required for stationary check')
    ts = [check_time(j, now, policy) for j in history]
    if any(b <= a or b-a > policy['max_joint_gap_ms']*1e6 for a, b in zip(ts, ts[1:])):
        raise ValidationError('duplicate/out-of-order feedback or feedback gap')
    half = policy['stationary_window_ms']*1e6/2
    if ts[0] > t_image-half or ts[-1] < t_image+half:
        raise ValidationError('feedback must bracket image for entire stationary window')
    positions = np.array([joint_vector(j) for j in history])
    if np.max(np.ptp(positions, axis=0)) > policy['stationary_tolerance_rad']:
        raise ValidationError('robot not stationary within configured joint tolerance')
    if not any(j == joint for j in history) or not np.isfinite(selected).all():
        raise ValidationError('selected actual state is not in recorded history')
    if meta['fk']['joint_state_sha256'] != digest(joint) or meta['fk']['stamp_ns'] != tj:
        raise ValidationError('FK must be computed from this exact recorded JointState')
    required_text(meta['fk']['source'], 'FK source')
    required_text(meta['joint_source_evidence'], 'actual feedback source evidence')


def validate_sample(meta, color, depth, now_ns=None):
    try:
        if meta['schema'] != 'd1t_vision.sample.v1' or type(meta['test_only']) is not bool:
            raise ValidationError('sample schema/test_only flag missing')
        for key in ('sample_id', 'session_id', 'device_id', 'mount_id', 'board_fixed_evidence'):
            required_text(meta[key], key)
        optical = frame(meta['optical_frame'])
        if optical in ('camera_link', 'tcp_link', 'Empty_Link6', 'base_link'):
            raise ValidationError('actual COLOR optical frame required')
        if meta['base_frame'] != 'base_link' or meta['ee_frame'] != 'Empty_Link6':
            raise ValidationError('this package uses base_link and Empty_Link6')
        validate_board(meta['board'], meta['test_only'])
        validate_timing(meta, now_ns)
        for stream in CAMERA_STREAMS:
            if meta[stream]['frame_id'] != optical:
                raise ValidationError('color/aligned depth/CameraInfo optical frame mismatch')
        for stream in ('color_info', 'depth_info'):
            camera_model(meta[stream], meta['image_geometry'])
        if info_signature(meta['color_info']) != info_signature(meta['depth_info']):
            raise ValidationError('aligned depth and color intrinsics must describe same pixel grid')
        required_text(meta['depth']['scale_evidence'], 'depth encoding/scale evidence')
        positive(meta['depth']['scale_m'], 'depth scale')
        positive(meta['depth']['min_m'], 'minimum depth')
        positive(meta['depth']['max_m'], 'maximum depth')
        if meta['depth']['min_m'] >= meta['depth']['max_m']:
            raise ValidationError('depth range must be increasing')
        expected = {'16UC1': np.dtype('uint16'), '32FC1': np.dtype('float32')}.get(meta['depth']['encoding'])
        if expected is None or depth.dtype != expected:
            raise ValidationError('depth array dtype/encoding mismatch')
        if meta['depth']['encoding'] == '32FC1' and meta['depth']['scale_m'] != 1.0:
            raise ValidationError('32FC1 must explicitly use metres')
        shape = (meta['color_info']['height'], meta['color_info']['width'])
        if color.dtype != np.uint8 or color.shape != (*shape, 3) or depth.shape != shape:
            raise ValidationError('image/depth/CameraInfo dimension or dtype mismatch')
        if meta['color']['encoding'] not in ('rgb8', 'bgr8'):
            raise ValidationError('unsupported color encoding')
        read_transform(meta['fk']['transform'], 'Empty_Link6', 'base_link')
        obs = meta['observation']
        read_transform(obs['transform'], 'calibration_board', optical)
        ids = obs['corner_ids']
        xyz = board_points(meta['board'], ids)
        uv = finite(obs['pixels_uv'], (len(ids), 2), 'detected corners')
        if len(ids) < 6 or np.linalg.matrix_rank(xyz[:, :2]-xyz[:, :2].mean(0)) < 2:
            raise ValidationError('need >=6 noncollinear ChArUco corners')
        if np.any(uv < 0) or np.any(uv[:, 0] >= shape[1]) or np.any(uv[:, 1] >= shape[0]):
            raise ValidationError('detected corners outside image')
        error = obs['reprojection_rms_px']
        limit = positive(obs['max_reprojection_rms_px'], 'reprojection threshold')
        if not isinstance(error, (int, float)) or not np.isfinite(error) or not 0 <= error <= limit:
            raise ValidationError('invalid/excessive board reprojection residual')
        required_text(obs['library'], 'observation library provenance')
    except (KeyError, TypeError, IndexError) as exc:
        raise ValidationError(f'missing/malformed sample field: {exc}') from exc
    return meta


def save_sample(root, meta, color, depth):
    validate_sample(meta, color, depth)
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    # The storage name never uses unsanitized measurement labels.
    temporary = Path(tempfile.mkdtemp(prefix='.pending-', dir=root))
    try:
        np.save(temporary/'color.npy', color, allow_pickle=False)
        np.save(temporary/'depth.npy', depth, allow_pickle=False)
        (temporary/'sample.json').write_bytes(json_bytes(meta))
        checksums = {n: hashlib.sha256((temporary/n).read_bytes()).hexdigest()
                     for n in ('sample.json', 'color.npy', 'depth.npy')}
        (temporary/'checksums.json').write_bytes(json_bytes(checksums))
        final = root/str(uuid.uuid4())
        temporary.rename(final)
        return final
    except Exception:
        shutil.rmtree(temporary)
        raise


def load_sample(path):
    path = Path(path)
    try:
        hashes = read_json(path/'checksums.json')
        for name in ('sample.json', 'color.npy', 'depth.npy'):
            if hashlib.sha256((path/name).read_bytes()).hexdigest() != hashes[name]:
                raise ValidationError(f'sample checksum mismatch: {name}')
        meta = read_json(path/'sample.json')
        color = np.load(path/'color.npy', allow_pickle=False)
        depth = np.load(path/'depth.npy', allow_pickle=False)
        validate_sample(meta, color, depth)  # use capture time, not today's wall clock
        return meta, color, depth
    except (OSError, KeyError, ValueError) as exc:
        raise ValidationError(f'invalid sample {path}: {exc}') from exc


def sample_paths(root):
    result = sorted(p.parent for p in Path(root).glob('*/sample.json') if not p.parent.name.startswith('.'))
    if not result:
        raise ValidationError('no complete sample directories')
    return result
