"""VISION-02 installation and reviewed-plan contracts.

The module deliberately contains no ROS or motion side effects.  It turns the
new direct-table installation into a complete, reviewable contract and refuses
to turn incomplete drawing values into a plan.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .geometry import ValidationError, finite, rigid


INSTALLATION_SCHEMA = 'd1t_vision.vision02.installation.v1'
PLAN_SCHEMA = 'd1t_vision.vision02.plan.v1'
ARM_JOINTS = [f'Joint{i}' for i in range(1, 7)]
OBSOLETE_SCENE_IDS = ('mount_board', 'desk_clamp')
CAMERA_CENTER_IN_EMPTY_LINK6_M = np.array([0.065, 0.0, 0.045])
BOARD_PATTERN_SIZE_M = 0.15
# VISION-01 field CameraInfo baseline; the live planner records that the actual
# CameraInfo must be rechecked before execution/capture.
CAMERA_FX_FY = (906.0022583007812, 905.3165893554688)
CAMERA_IMAGE_SIZE = (1280, 720)


def canonical_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()


def sha256(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ValidationError(f'cannot read VISION-02 JSON: {exc}') from exc


def _positive(value, name):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValidationError(f'{name} must be a positive finite number')
    return float(value)


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f'{name} must be recorded text, not an assumption')
    return value


def validate_installation(cfg, require_confirmed=True):
    """Validate all scene facts needed to safely construct the new desk box.

    ``desk_to_base`` is T_base_desk: it maps desk-frame metres to base_link
    metres.  Requiring a rigid explicit transform prevents a translation-only
    shortcut from silently assuming the base axes match the drawing axes.
    """
    try:
        if cfg['schema'] != INSTALLATION_SCHEMA:
            raise ValidationError('unsupported VISION-02 installation schema')
        if require_confirmed and cfg['field_confirmation']['confirmed'] is not True:
            raise ValidationError('installation is not field-confirmed; scene application is forbidden')
        _text(cfg['installation_id'], 'installation_id')
        field = cfg['field_confirmation']
        for key in ('operator', 'recorded_at', 'base_axis_evidence', 'base_contact_evidence',
                    'board_fixed_evidence'):
            _text(field[key], 'field_confirmation.' + key)
        desk = cfg['desk']
        if desk['frame'] != 'desk_frame' or desk['units'] != 'm':
            raise ValidationError('desk must be specified in desk_frame metres')
        _positive(desk['size_x_m'], 'desk.size_x_m')
        _positive(desk['size_y_m'], 'desk.size_y_m')
        _positive(desk['thickness_m'], 'desk.thickness_m')
        if not isinstance(desk['top_z_m'], (int, float)) or not math.isfinite(desk['top_z_m']):
            raise ValidationError('desk.top_z_m must be a finite desk-frame height')
        margin = _positive(cfg['safety_margin_m'], 'safety_margin_m')
        if margin < 0.03:
            raise ValidationError('VISION-02 keeps at least the existing 30 mm safety margin')
        t = np.asarray(cfg['desk_to_base']['matrix'], dtype=float)
        if t.shape != (4, 4):
            raise ValidationError('desk_to_base.matrix must be 4x4')
        rigid(t)
        if cfg['desk_to_base'].get('source_frame') != 'desk_frame' or cfg['desk_to_base'].get('target_frame') != 'base_link':
            raise ValidationError('desk_to_base must explicitly be desk_frame -> base_link')
        envelope = cfg['camera_mount_envelope']
        if envelope['frame'] != 'Empty_Link6' or envelope['source'] == '':
            raise ValidationError('camera envelope needs Empty_Link6 frame and stated source')
        extents = finite(envelope['box_size_m'], (3,), 'camera envelope dimensions')
        if np.any(extents <= 0):
            raise ValidationError('camera envelope dimensions must be positive')
        center = finite(envelope['box_center_m'], (3,), 'camera envelope centre')
        if not np.isfinite(center).all():
            raise ValidationError('camera envelope centre must be finite')
        board = cfg['board']
        if board['frame'] not in ('desk_frame', 'base_link') or board['units'] != 'm':
            raise ValidationError('board estimate must use desk_frame or base_link metres')
        _positive(board['outer_size_x_m'], 'board.outer_size_x_m')
        _positive(board['outer_size_y_m'], 'board.outer_size_y_m')
        # Board pose is not required by hand-eye calibration: the solver estimates
        # the fixed board pose.  It becomes mandatory only for independent local
        # absolute-position validation, which has a separate gate.
        if board.get('center_m') is not None:
            finite(board['center_m'], (3,), 'board centre')
    except ValidationError:
        raise
    except (KeyError, TypeError, ValueError) as exc:
        raise ValidationError(f'incomplete/malformed VISION-02 installation: {exc}') from exc
    return cfg


def validate_local_reference(cfg):
    """Additional facts needed only for independent desk/local validation."""
    validate_installation(cfg)
    board = cfg['board']
    if board.get('center_m') is None:
        raise ValidationError('local validation requires an independently measured board centre')
    finite(board['center_m'], (3,), 'board centre')
    _text(board.get('orientation_evidence'), 'board.orientation_evidence')
    _text(cfg['field_confirmation'].get('board_axis_evidence'),
          'field_confirmation.board_axis_evidence')
    return cfg


def desk_collision_spec(cfg):
    """A serialisable rigid pose for the new desk collision object."""
    validate_installation(cfg)
    desk, margin = cfg['desk'], float(cfg['safety_margin_m'])
    base_from_desk = np.asarray(cfg['desk_to_base']['matrix'], dtype=float)
    pose = base_from_desk.copy()
    local_center = np.array([desk['size_x_m'] / 2, desk['size_y_m'] / 2,
                             desk['top_z_m'] - desk['thickness_m'] / 2 + margin / 2, 1.0])
    pose[:, 3] = base_from_desk @ local_center
    return {
        'id': 'desk',
        'frame_id': 'base_link',
        'size_m': [desk['size_x_m'] + 2 * margin, desk['size_y_m'] + 2 * margin,
                    desk['thickness_m'] + margin],
        'matrix': pose.tolist(),
        'physical_top_z_desk_m': desk['top_z_m'],
        'safety_margin_m': margin,
        'source': cfg['installation_id'],
    }


def camera_mount_collision_spec(cfg):
    """Serialise the reviewed rigid camera/mount envelope attached to link 6.

    The envelope is deliberately a simple box.  By the recorded field scope,
    the flexible USB plug/cable is excluded rather than represented here.
    """
    validate_installation(cfg)
    envelope = cfg['camera_mount_envelope']
    return {
        'id': 'vision_assembly',
        'frame_id': 'Empty_Link6',
        'center_m': finite(envelope['box_center_m'], (3,),
                           'camera envelope centre').tolist(),
        'size_m': finite(envelope['box_size_m'], (3,),
                         'camera envelope dimensions').tolist(),
        'touch_links': ['Empty_Link6'],
        'usb_cable_modelled': False,
        'usb_cable_in_scope': False,
        'source': envelope['source'],
    }


def obsolete_scene_removals(existing_ids):
    """Return only obsolete objects that are present in the current scene."""
    existing = set(existing_ids)
    return [object_id for object_id in OBSOLETE_SCENE_IDS if object_id in existing]


def _unit(vector, name):
    value = finite(vector, (3,), name)
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        raise ValidationError(name + ' must be nonzero')
    return value / norm


def vision02_view_definitions():
    """Frozen finite camera-view set; no robot motion or IK occurs here."""
    raw = [
        ('train_01', 'train', [0.00, 0.00, 1.00], .32,   0.),
        ('train_02', 'train', [0.24, 0.00, 0.971], .34,  0.),
        ('train_03', 'train', [-0.24, 0.00, 0.971], .34, 0.),
        ('train_04', 'train', [0.00, 0.24, 0.971], .36, 12.),
        ('train_05', 'train', [0.00, -0.24, 0.971], .36, -12.),
        ('train_06', 'train', [0.18, 0.18, 0.967], .38, 20.),
        ('train_07', 'train', [-0.18, 0.18, 0.967], .38, -20.),
        ('train_08', 'train', [0.18, -0.18, 0.967], .40, 28.),
        ('held_out_01', 'held_out', [-0.18, -0.18, 0.967], .36, -28.),
        ('held_out_02', 'held_out', [0.30, 0.12, 0.946], .42, 10.),
        ('held_out_03', 'held_out', [-0.30, -0.12, 0.946], .42, -10.),
    ]
    return [{'id': ident, 'role': role,
             'board_to_camera_direction': _unit(direction, ident + ' direction').tolist(),
             'optical_distance_m': distance, 'roll_about_optical_axis_deg': roll}
            for ident, role, direction, distance, roll in raw]


def camera_view_geometry(board_center_m, view, camera_center_in_link=None):
    """Return desired Empty_Link6 pose and analytic board-in-view evidence.

    Mount convention inferred from the three field photos: camera optical
    forward is Empty_Link6 +X and image-horizontal is Empty_Link6 +Y.  This is
    a planning/review convention, not a solved hand-eye transform.
    """
    board = finite(board_center_m, (3,), 'board centre')
    distance = _positive(view['optical_distance_m'], 'optical distance')
    if not .28 <= distance <= .50:
        raise ValidationError('optical distance must remain within 0.28-0.50 m')
    outward = _unit(view['board_to_camera_direction'], 'board-to-camera direction')
    camera = board + distance * outward
    x_axis = -outward  # camera/link forward points from camera toward the board
    reference = np.array([1., 0., 0.])
    if abs(float(reference @ x_axis)) > .92:
        reference = np.array([0., 1., 0.])
    y_axis = _unit(reference - x_axis * float(reference @ x_axis), 'image horizontal')
    z_axis = np.cross(x_axis, y_axis)
    roll = math.radians(float(view['roll_about_optical_axis_deg']))
    y_rot = math.cos(roll) * y_axis + math.sin(roll) * z_axis
    z_rot = -math.sin(roll) * y_axis + math.cos(roll) * z_axis
    rotation = np.column_stack((x_axis, y_rot, z_rot))
    rigid(np.block([[rotation, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
    offset = (CAMERA_CENTER_IN_EMPTY_LINK6_M if camera_center_in_link is None
              else finite(camera_center_in_link, (3,), 'camera centre in Empty_Link6'))
    link_origin = camera - rotation @ offset
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = link_origin

    half = BOARD_PATTERN_SIZE_M / 2
    board_corners = np.array([[board[0] + dx, board[1] + dy, board[2]]
                              for dx in (-half, half) for dy in (-half, half)])
    local = (board_corners - camera) @ rotation
    depth = local[:, 0]
    tan_h = (CAMERA_IMAGE_SIZE[0] / 2) / CAMERA_FX_FY[0]
    tan_v = (CAMERA_IMAGE_SIZE[1] / 2) / CAMERA_FX_FY[1]
    margins = np.column_stack((tan_h - np.abs(local[:, 1] / depth),
                               tan_v - np.abs(local[:, 2] / depth)))
    fov_pass = bool(np.all(depth > 0) and np.all(margins >= 0))
    return {
        'camera_center_base_m': camera.tolist(),
        'empty_link6_matrix': matrix.tolist(),
        'optical_axis_base': x_axis.tolist(),
        'optical_distance_m': distance,
        'pattern_fov_pass_analytic': fov_pass,
        'pattern_fov_min_tangent_margin': float(np.min(margins)),
        'boundary': ('Analytic 150 mm pattern/FOV check using historical CameraInfo; '
                     'live 16-corner detection remains mandatory.'),
    }


def validate_review_plan(plan, installation):
    """Validate a fully materialised motion plan before any executor sees it."""
    validate_installation(installation)
    try:
        if plan['schema'] != PLAN_SCHEMA:
            raise ValidationError('unsupported VISION-02 plan schema')
        if plan['installation_sha256'] != sha256(installation):
            raise ValidationError('plan does not match the reviewed installation config')
        if plan['execution']['point_mode'] is not True:
            raise ValidationError('VISION-02 allows point mode only')
        for key in ('velocity_scaling', 'acceleration_scaling'):
            value = _positive(plan['execution'][key], 'execution.' + key)
            if value > 0.3:
                raise ValidationError('VISION-02 execution scaling exceeds 0.3')
        poses = plan['poses']
        if len(poses) < 8:
            raise ValidationError('fewer than eight reviewed training poses')
        ids = set()
        for item in poses:
            ident = _text(item['id'], 'pose id')
            if ident in ids:
                raise ValidationError('duplicate pose id')
            ids.add(ident)
            if item['role'] not in ('train', 'held_out'):
                raise ValidationError('pose role must be train or held_out')
            finite(item['goal_joint_rad'], (6,), 'goal_joint_rad')
            if not isinstance(item.get('trajectory_ros_message'), dict):
                raise ValidationError('each pose requires a verbatim MoveIt trajectory message')
            trajectory = item['trajectory']
            if trajectory['joint_names'] != ARM_JOINTS or len(trajectory['points']) < 2:
                raise ValidationError('each pose needs a complete six-joint trajectory')
            last = -1.0
            for index, point in enumerate(trajectory['points']):
                finite(point['positions_rad'], (6,), 'trajectory positions')
                finite(point['velocities_rad_s'], (6,), 'trajectory velocities')
                finite(point['accelerations_rad_s2'], (6,), 'trajectory accelerations')
                t = point['time_from_start_s']
                if not isinstance(t, (int, float)) or not math.isfinite(t) or t < 0:
                    raise ValidationError('trajectory time must be finite and nonnegative')
                if index > 0 and t <= 0:
                    raise ValidationError('only the first trajectory point may be at time zero')
                if t <= last:
                    raise ValidationError('trajectory timestamps must strictly increase')
                last = float(t)
            if not isinstance(item['minimum_clearance_mm'], (int, float)) or item['minimum_clearance_mm'] < 45:
                raise ValidationError('reviewed pose clearance is below 45 mm')
        if sum(p['role'] == 'train' for p in poses) < 8 or sum(p['role'] == 'held_out' for p in poses) < 3:
            raise ValidationError('plan needs >=8 train and >=3 held-out poses')
    except ValidationError:
        raise
    except (KeyError, TypeError, ValueError) as exc:
        raise ValidationError(f'incomplete/malformed reviewed plan: {exc}') from exc
    return plan
