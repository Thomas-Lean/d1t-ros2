"""AX=XB solver and held-out depth checks, independent of ROS and OpenCV."""
from datetime import datetime, timezone
from pathlib import Path
import itertools
import numpy as np
from .geometry import (ValidationError, apply, deproject, inverse, positive,
                       read_transform, rigid, rotation_angle, transform_record, finite)
from .samples import digest, info_signature, load_sample, read_json, write_new_json, validate_board, stamp


def _proper_rotation(matrix):
    u, _, vh = np.linalg.svd(matrix)
    return u @ np.diag([1., 1., np.linalg.det(u@vh)]) @ vh


def solve_hand_eye(base_ee, camera_board, max_translation_residual_m, max_rotation_residual_rad):
    """G_i X C_i = Y. X=T_E_C. All unordered pairs supply A X = X B.

    Rotation: null vector of (I⊗R_A - R_B^T⊗I), column-major vec(R_X).
    Translation: (R_A-I)t_X = R_X t_B-t_A. SVD ranks test observability.
    Residual gates are caller-supplied diagnostics, never grasp tolerances.
    """
    positive(max_translation_residual_m, 'translation residual limit')
    positive(max_rotation_residual_rad, 'rotation residual limit')
    if len(base_ee) != len(camera_board) or len(base_ee) < 8:
        raise ValidationError('at least 8 paired samples required; 12 recommended')
    g, c = [rigid(t) for t in base_ee], [rigid(t) for t in camera_board]
    pairs = [(inverse(g[i])@g[j], c[i]@inverse(c[j])) for i, j in itertools.combinations(range(len(g)), 2)]
    system = np.vstack([np.kron(np.eye(3), a[:3, :3])-np.kron(b[:3, :3].T, np.eye(3)) for a, b in pairs])
    _, s, vh = np.linalg.svd(system, full_matrices=False)
    # Numerical diversity floor, not a physical accuracy claim. Pure translation,
    # duplicated poses, single-axis rotation and very narrow sweeps fail here.
    if s[0] < 1e-8 or s[-2]/s[0] < 1e-3 or s[-1]/s[-2] > .25:
        raise ValidationError('degenerate/inconsistent rotation sampling: require independent axes and spread')
    raw = vh[-1].reshape(3, 3, order='F')
    if np.linalg.det(raw) < 0:
        raw = -raw
    rx = _proper_rotation(raw)
    lhs = np.vstack([a[:3, :3]-np.eye(3) for a, _ in pairs])
    rhs = np.concatenate([rx@b[:3, 3]-a[:3, 3] for a, b in pairs])
    tx, _, rank, st = np.linalg.lstsq(lhs, rhs, rcond=1e-3)
    if rank != 3 or st[-1]/st[0] < 1e-3:
        raise ValidationError('degenerate translation solve')
    x = np.eye(4)
    x[:3, :3], x[:3, 3] = rx, tx
    rigid(x)
    ys = [gi@x@ci for gi, ci in zip(g, c)]
    mean = np.eye(4)
    mean[:3, :3] = _proper_rotation(sum(t[:3, :3] for t in ys))
    mean[:3, 3] = np.mean([t[:3, 3] for t in ys], axis=0)
    residuals = [{'translation_m': float(np.linalg.norm(t[:3, 3]-mean[:3, 3])),
                  'rotation_rad': rotation_angle(mean[:3, :3].T@t[:3, :3])} for t in ys]
    if any(r['translation_m'] > max_translation_residual_m or r['rotation_rad'] > max_rotation_residual_rad for r in residuals):
        raise ValidationError('fixed-board residual gate failed; inspect every sample, no automatic outlier deletion')
    return x, mean, residuals, {'rotation_singular_values': s.tolist(), 'translation_singular_values': st.tolist(),
                               'relative_rank_floor': 1e-3, 'nullspace_ratio_limit': .25}


def contract(meta):
    return {key: meta[key] for key in ('session_id', 'device_id', 'mount_id', 'board', 'optical_frame',
                                      'base_frame', 'ee_frame', 'image_geometry', 'test_only')} | {
        'camera_model': info_signature(meta['color_info']),
        'depth_model': {k: meta['depth'][k] for k in ('encoding', 'scale_m', 'min_m', 'max_m')}}


def solve_dataset(paths, output, max_translation_residual_m, max_rotation_residual_rad):
    paths = [Path(p) for p in paths]
    bundles = [load_sample(p) for p in paths]
    metas = [b[0] for b in bundles]
    if len(metas) < 8:
        raise ValidationError('at least 8 paired samples required; 12 recommended')
    reference = contract(metas[0])
    if any(contract(m) != reference for m in metas):
        raise ValidationError('mixed camera/board/session/mount/frame/depth calibration data')
    ids = [m['sample_id'] for m in metas]
    stamps = [m['color']['stamp_ns'] for m in metas]
    if len(set(ids)) != len(ids) or len(set(stamps)) != len(stamps):
        raise ValidationError('repeated sample IDs or image timestamps')
    gs = [read_transform(m['fk']['transform'], 'Empty_Link6', 'base_link') for m in metas]
    cs = [read_transform(m['observation']['transform'], 'calibration_board', m['optical_frame']) for m in metas]
    x, y, residuals, diagnostics = solve_hand_eye(gs, cs, max_translation_residual_m, max_rotation_residual_rad)
    result = {'schema': 'd1t_vision.calibration.v1', 'test_only': reference['test_only'],
              'status': 'TEST_ONLY' if reference['test_only'] else 'UNVALIDATED_REAL_CALIBRATION',
              'created_utc': datetime.now(timezone.utc).isoformat(), 'contract': reference,
              'transform': transform_record(x, reference['optical_frame'], 'Empty_Link6'),
              'board_pose_training_mean': transform_record(y, 'calibration_board', 'base_link'),
              'solver': 'numpy AX=XB Kronecker SVD + translation least squares', 'numpy_version': np.__version__,
              'source_samples': [{'sample_id': m['sample_id'], 'metadata_sha256': digest(m),
                                  'image_stamp_ns': m['color']['stamp_ns'], 'checksums': read_json(p/'checksums.json')}
                                 for m, p in zip(metas, paths)],
              'residuals': [dict(sample_id=m['sample_id'], **r) for m, r in zip(metas, residuals)],
              'limits': {'translation_m': max_translation_residual_m, 'rotation_rad': max_rotation_residual_rad},
              'diagnostics': diagnostics, 'absolute_accuracy': '未验证',
              'restriction': 'Observation only. No motion interface. Synthetic data is never a real calibration.'}
    write_new_json(output, result)
    return result


def load_calibration(path, allow_test_data=False):
    c = read_json(path)
    try:
        if c['schema'] != 'd1t_vision.calibration.v1' or type(c['test_only']) is not bool:
            raise ValidationError('invalid calibration schema/test flag')
        expected = 'TEST_ONLY' if c['test_only'] else 'UNVALIDATED_REAL_CALIBRATION'
        if c['status'] != expected or c['contract']['test_only'] != c['test_only']:
            raise ValidationError('calibration status/test provenance mismatch')
        if c['test_only'] and not allow_test_data:
            raise ValidationError('TEST_ONLY calibration forbidden in runtime')
        read_transform(c['transform'], c['contract']['optical_frame'], 'Empty_Link6')
        validate_board(c['contract']['board'], test_only=c['test_only'])
        if c['contract']['base_frame'] != 'base_link' or c['contract']['ee_frame'] != 'Empty_Link6':
            raise ValidationError('calibration robot frame mismatch')
        sources = c['source_samples']
        if len(sources) < 8 or len(c['residuals']) != len(sources):
            raise ValidationError('calibration needs >=8 sources and one residual for each')
        ids = [s['sample_id'] for s in sources]
        stamps = [stamp(s['image_stamp_ns']) for s in sources]
        if len(set(ids)) != len(ids) or len(set(stamps)) != len(stamps):
            raise ValidationError('duplicate calibration source provenance')
        for name in ('translation_m', 'rotation_rad'):
            positive(c['limits'][name], 'calibration residual limit')
        for source, residual in zip(sources, c['residuals']):
            if source['sample_id'] != residual['sample_id']:
                raise ValidationError('calibration source/residual mismatch')
            for name in ('translation_m', 'rotation_rad'):
                value = residual[name]
                if not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 <= value <= c['limits'][name]:
                    raise ValidationError('invalid/excessive calibration residual')
    except (KeyError, TypeError) as exc:
        raise ValidationError(f'incomplete calibration: {exc}') from exc
    return c


def sample_point(meta, depth, calibration, pixel_uv):
    if contract(meta) != calibration['contract']:
        raise ValidationError('sample/calibration contract mismatch')
    uv = finite(pixel_uv, (2,), 'pixel (u,v)')
    if np.any(uv < 0) or uv[0] >= depth.shape[1] or uv[1] >= depth.shape[0]:
        raise ValidationError('pixel outside depth image')
    # Sample the nearest pixel; use THAT pixel's ray, not the subpixel feature ray.
    u, v = np.floor(uv + .5).astype(int)
    if u >= depth.shape[1] or v >= depth.shape[0]:
        raise ValidationError('rounded pixel outside depth image')
    d = meta['depth']
    pc = deproject([u, v], depth[v, u], meta['color_info'], meta['image_geometry'],
                   d['encoding'], d['scale_m'], d['min_m'], d['max_m'])
    x = read_transform(calibration['transform'], meta['optical_frame'], 'Empty_Link6')
    g = read_transform(meta['fk']['transform'], 'Empty_Link6', 'base_link')
    return {'pixel_uv_used': [int(u), int(v)], 'depth_raw': float(depth[v, u]),
            'camera_point_m': pc.tolist(), 'base_point_m': apply(g@x, pc).tolist(),
            'stamp_ns': meta['color']['stamp_ns'], 'frame_id': 'base_link', 'test_only': meta['test_only']}


def check_held_out(paths, calibration, corner_ids, independent_reference=None):
    if len(paths) < 3 or len(corner_ids) < 3 or len(set(corner_ids)) != len(corner_ids):
        raise ValidationError('held-out check requires >=3 new poses and >=3 distinct board corners')
    training = calibration['source_samples']
    train_ids = {s['sample_id'] for s in training}
    train_stamps = {s['image_stamp_ns'] for s in training}
    train_images = {s['checksums']['color.npy'] for s in training}
    measurements, seen, seen_stamps, seen_images = [], set(), set(), set()
    for path in paths:
        m, _, depth = load_sample(path)
        if (m['sample_id'] in train_ids or m['color']['stamp_ns'] in train_stamps
                or read_json(path/'checksums.json')['color.npy'] in train_images):
            raise ValidationError('held-out observation reuses training data')
        image_hash = read_json(path/'checksums.json')['color.npy']
        if m['sample_id'] in seen or m['color']['stamp_ns'] in seen_stamps or image_hash in seen_images:
            raise ValidationError('duplicate held-out sample/image')
        seen.add(m['sample_id'])
        seen_stamps.add(m['color']['stamp_ns'])
        seen_images.add(image_hash)
        ids, pixels = m['observation']['corner_ids'], m['observation']['pixels_uv']
        if not set(corner_ids).issubset(ids):
            raise ValidationError('requested reference corners not visible in all held-out images')
        measurements.append([sample_point(m, depth, calibration, pixels[ids.index(i)])['base_point_m'] for i in corner_ids])
    points = np.array(measurements)
    mean = points.mean(axis=0)
    distances = np.linalg.norm(points-mean, axis=2)
    report = {'schema': 'd1t_vision.held_out.v1', 'test_only': calibration['test_only'],
              'corner_ids': corner_ids, 'base_points_m_by_pose': points.tolist(),
              'cross_pose_consistency': {'reference': 'held-out mean, NOT absolute ground truth',
                                         'rms_m': float(np.sqrt(np.mean(distances**2))), 'max_m': float(distances.max())},
              'absolute_position_error': '未验证: no independent base_link reference provided',
              'sampling': 'nearest depth pixel and its own ray; raw pixels are undistorted when needed'}
    if independent_reference is not None:
        ref = independent_reference
        if (ref.get('independent_of_this_calibration') is not True or ref.get('frame_id') != 'base_link'
                or ref.get('unit') != 'm' or ref.get('corner_ids') != corner_ids
                or ref.get('board_id') != calibration['contract']['board']['board_id']
                or ref.get('session_id') != calibration['contract']['session_id']
                or not ref.get('measurement_method') or not ref.get('source_record')):
            raise ValidationError('independent reference must include matching board/session/IDs, method and original source record')
        truth = finite(ref['base_points_m'], (len(corner_ids), 3), 'independent surveyed points')
        if np.linalg.matrix_rank(truth-truth.mean(0)) < 2:
            raise ValidationError('independent reference points must not be collinear')
        error = np.linalg.norm(points-truth[None, :, :], axis=2)
        report['absolute_position_error'] = {'independent_reference': ref, 'per_pose_per_corner_m': error.tolist(),
                                             'rms_m': float(np.sqrt(np.mean(error**2))), 'max_m': float(error.max()),
                                             'acceptance': 'not assessed; no grasp accuracy guarantee'}
    return report
