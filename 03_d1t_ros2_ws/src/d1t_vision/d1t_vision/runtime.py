"""Pure input selection and TF attachment guards shared by ROS wrappers/tests."""
from copy import deepcopy
import numpy as np
from .geometry import (ValidationError, camera_model, frame, inverse, positive,
                       required_text, rigid, transform_record)
from .samples import CAMERA_STREAMS, validate_board, validate_policy, stamp, check_time, joint_vector


def validate_config(cfg):
    try:
        if cfg['schema'] != 'd1t_vision.runtime.v1':
            raise ValidationError('unsupported runtime config schema')
        for name in ('session_id', 'device_id', 'mount_id', 'board_fixed_evidence', 'joint_source_evidence'):
            required_text(cfg[name], name)
        optical = frame(cfg['optical_frame'])
        if optical in ('camera_link', 'tcp_link', 'Empty_Link6', 'base_link'):
            raise ValidationError('actual COLOR optical frame required')
        for name in (*CAMERA_STREAMS, 'joint_state'):
            required_text(cfg['topics'][name], f'topic: {name}')
        if len(set(cfg['topics'].values())) != 5 or any(not v.startswith('/') for v in cfg['topics'].values()):
            raise ValidationError('five distinct absolute input topics required')
        required_text(cfg['fk_service'], 'read-only FK service')
        publisher = required_text(cfg['joint_state_publisher_node'], 'actual JointState publisher node')
        if not publisher.startswith('/') or publisher.endswith('/'):
            raise ValidationError('JointState publisher node must be an absolute node name')
        required_text(cfg['output_directory'], 'sample output directory')
        required_text(cfg['clock']['evidence'], 'clock evidence')
        if cfg['clock']['same_ros_clock_verified'] is not True:
            raise ValidationError('verify camera and robot header clocks before capture')
        for name in (*CAMERA_STREAMS, 'joint_state'):
            required_text(cfg['stamp_semantics'][name], f'{name} stamp semantics')
        if cfg['image_geometry'] not in ('raw', 'rectified'):
            raise ValidationError('configure raw or rectified color pixel geometry')
        validate_board(cfg['board'])
        validate_policy(cfg['pairing_policy'])
        for name in ('scale_m', 'min_m', 'max_m'):
            positive(cfg['depth'][name], f'depth {name}')
        if cfg['depth']['encoding'] not in ('16UC1', '32FC1') or cfg['depth']['min_m'] >= cfg['depth']['max_m']:
            raise ValidationError('configure supported depth encoding and usable range')
        if cfg['depth']['encoding'] == '32FC1' and cfg['depth']['scale_m'] != 1:
            raise ValidationError('32FC1 requires explicit metre scale=1')
        required_text(cfg['depth']['scale_evidence'], 'depth scale evidence')
        positive(cfg['max_reprojection_rms_px'], 'board reprojection gate')
        positive(cfg['fk_timeout_s'], 'FK response timeout')
    except (KeyError, TypeError) as exc:
        raise ValidationError(f'incomplete runtime config: {exc}') from exc
    return cfg


def choose_inputs(buffers, cfg, now_ns, newer_than_ns=0):
    """Newest color with a complete stationary window; never silently try older
    frames after a chosen frame fails board/timing checks. Items=(metadata,array).
    """
    now_ns = stamp(now_ns)
    if any(not buffers.get(s) for s in (*CAMERA_STREAMS, 'joint_state')):
        raise ValidationError('missing camera/CameraInfo/actual joint stream')
    joints = [j[0] for j in buffers['joint_state']]
    half = cfg['pairing_policy']['stationary_window_ms']*1e6/2
    eligible = [item for item in buffers['color'] if newer_than_ns < item[0]['stamp_ns'] <= joints[-1]['stamp_ns']-half]
    if not eligible:
        raise ValidationError('no new image bracketed by a full stationary feedback window')
    color_meta, color = max(eligible, key=lambda item: item[0]['stamp_ns'])
    t = color_meta['stamp_ns']
    before = [j for j in joints if j['stamp_ns'] <= t-half]
    after = [j for j in joints if j['stamp_ns'] >= t+half]
    if not before or not after:
        raise ValidationError('joint history does not bracket image')
    start, end = before[-1]['stamp_ns'], after[0]['stamp_ns']
    history = [j for j in joints if start <= j['stamp_ns'] <= end]
    joint = min(history, key=lambda j: abs(j['stamp_ns']-t))
    result = {'color': deepcopy(color_meta), 'joint_state': deepcopy(joint),
              'joint_history': deepcopy(history), 'capture_ros_ns': now_ns}
    depth = None
    for name in ('depth', 'color_info', 'depth_info'):
        meta, array = min(buffers[name], key=lambda item: abs(item[0]['stamp_ns']-t))
        result[name] = deepcopy(meta)
        if name == 'depth':
            depth = array
    for name in ('color_info', 'depth_info'):
        camera_model(result[name], cfg['image_geometry'])
    return result, color.copy(), depth.copy()


def mount_transform(calibration, root_frame, root_to_optical, observed_edges):
    """Attach ONLY the driver's root; optical keeps its existing parent chain.

    observed_edges: child -> {(parent, publisher_identity), ...}, both TF topics.
    """
    from .geometry import read_transform
    root_frame = frame(root_frame)
    optical = calibration['contract']['optical_frame']
    if root_frame in (optical, 'base_link', 'Empty_Link6', 'tcp_link'):
        raise ValidationError('camera root must be distinct from optical/robot frames')
    if observed_edges.get(root_frame):
        raise ValidationError('camera root already has a parent/publisher; refuse duplicate static TF')
    current, visited = optical, set()
    while current != root_frame:
        if current in visited:
            raise ValidationError('cycle in camera TF graph')
        visited.add(current)
        parents = observed_edges.get(current, set())
        if len(parents) != 1:
            raise ValidationError('camera internal TF chain missing or has duplicate publishers/parents')
        current = next(iter(parents))[0]
    t_e_c = read_transform(calibration['transform'], optical, 'Empty_Link6')
    return transform_record(t_e_c@inverse(rigid(root_to_optical)), root_frame, 'Empty_Link6')


def quaternion_xyzw(matrix):
    """Rotation matrix to unit quaternion via a symmetric eigenproblem."""
    r = rigid(matrix)[:3, :3]
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    _, v = np.linalg.eigh(k)
    q = v[:, -1]
    return q if q[3] >= 0 else -q


def check_point_joint_bracket(history, image_ns, now_ns, policy):
    """TF interpolation is permitted only between fresh actual feedback messages."""
    stamp(image_ns)
    stamp(now_ns)
    validate_policy(policy)
    before = [j for j in history if j['stamp_ns'] <= image_ns]
    after = [j for j in history if j['stamp_ns'] >= image_ns]
    if not before or not after:
        raise ValidationError('image timestamp lacks actual JointState bracket; no latest-state fallback')
    a, b = before[-1], after[0]
    for j in (a, b):
        check_time(j, now_ns, policy)
        joint_vector(j)
    if b['stamp_ns']-a['stamp_ns'] > policy['max_joint_gap_ms']*1e6:
        raise ValidationError('feedback gap too large for point-time interpolation')
    if min(image_ns-a['stamp_ns'], b['stamp_ns']-image_ns) > policy['max_joint_skew_ms']*1e6:
        raise ValidationError('point/actual feedback timestamps too far apart')
