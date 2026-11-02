"""Optional OpenCV ChArUco observation; never supplies invented observations."""
import numpy as np
from .geometry import ValidationError, camera_model, positive, transform_record
from .samples import board_points, validate_board


def opencv_board(spec):
    validate_board(spec, test_only=True)
    try:
        import cv2
        aruco = cv2.aruco
    except (ImportError, AttributeError) as exc:
        raise ValidationError('OpenCV with cv2.aruco is required; no detector fallback') from exc
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    args = (spec['squares_x'], spec['squares_y'], spec['square_length_m'], spec['marker_length_m'], dictionary)
    if hasattr(aruco, 'CharucoBoard_create'):
        board = aruco.CharucoBoard_create(*args)
    else:
        board = aruco.CharucoBoard(args[:2], *args[2:])
    if hasattr(board, 'setLegacyPattern'):
        board.setLegacyPattern(spec['legacy_pattern'])
    elif not spec['legacy_pattern']:
        # Unknown intermediate APIs must not silently generate a different printed pattern.
        raise ValidationError('this OpenCV API requires legacy_pattern=true; verify printed board')
    actual = np.asarray(board.getChessboardCorners() if hasattr(board, 'getChessboardCorners') else board.chessboardCorners)
    expected = board_points(spec, list(range((spec['squares_x']-1)*(spec['squares_y']-1))))
    if not np.allclose(actual, expected, atol=1e-7, rtol=1e-6):
        raise ValidationError('OpenCV board corner coordinates differ from saved board convention')
    return cv2, aruco, dictionary, board


def render_board(spec, pixels_per_square=160):
    cv2, _, _, board = opencv_board(spec)
    if type(pixels_per_square) is not int or pixels_per_square < 40:
        raise ValidationError('pixels_per_square must be integer >=40')
    size = (spec['squares_x']*pixels_per_square, spec['squares_y']*pixels_per_square)
    image = board.generateImage(size) if hasattr(board, 'generateImage') else board.draw(size)
    return cv2, image


def detect_board(color, encoding, info, image_geometry, spec, max_rms_px):
    cv2, aruco, dictionary, board = opencv_board(spec)
    positive(max_rms_px, 'max board reprojection RMS')
    if encoding not in ('rgb8', 'bgr8') or color.dtype != np.uint8 or color.shape != (info['height'], info['width'], 3):
        raise ValidationError('board detection requires correctly sized rgb8/bgr8 image')
    k, d = camera_model(info, image_geometry)
    gray = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY if encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < 2:
        raise ValidationError('board not visible: fewer than two markers detected')
    count, uv, corner_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board, cameraMatrix=k, distCoeffs=d)
    if count is None or count < 6 or corner_ids is None:
        raise ValidationError('fewer than six ChArUco corners detected')
    corner_ids = corner_ids.reshape(-1).astype(int).tolist()
    xyz = board_points(spec, corner_ids)
    if np.linalg.matrix_rank(xyz[:, :2]-xyz[:, :2].mean(0)) < 2:
        raise ValidationError('collinear ChArUco observation')
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(xyz, uv, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValidationError('board solvePnP failed')
    rotation = cv2.Rodrigues(rvec)[0]
    if np.any((xyz @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
        raise ValidationError('board pose behind optical plane')
    projected = cv2.projectPoints(xyz, rvec, tvec, k, d)[0].reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum((projected-uv)**2, axis=1))))
    if not np.isfinite(error) or error > max_rms_px:
        raise ValidationError(f'board reprojection RMS {error} exceeds {max_rms_px} px')
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation, tvec.reshape(3)
    return {'transform': transform_record(t, 'calibration_board', info['frame_id']),
            'corner_ids': corner_ids, 'pixels_uv': uv.tolist(), 'reprojection_rms_px': error,
            'max_reprojection_rms_px': max_rms_px, 'library': f'OpenCV {cv2.__version__} ChArUco + solvePnP ITERATIVE'}
