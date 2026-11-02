"""ROS-independent geometry; T_target_source maps p_source into p_target."""
import math
import numpy as np


class ValidationError(ValueError):
    """Invalid or insufficient measurement; never substitute a stale value."""


def finite(value, shape, label):
    try:
        a = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValidationError(f'{label}: numeric data required') from exc
    if a.shape != shape or not np.isfinite(a).all():
        raise ValidationError(f'{label}: expected finite shape {shape}, got {a.shape}')
    return a


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f'{label}: positive number required')
    if not math.isfinite(value) or value <= 0:
        raise ValidationError(f'{label}: positive finite number required')
    return value


def required_text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f'{label}: nonempty text required')
    return value


def frame(value):
    required_text(value, 'frame')
    if value.startswith('/') or any(c.isspace() for c in value):
        raise ValidationError('frame: no leading slash or whitespace')
    return value


def rigid(matrix):
    t = finite(matrix, (4, 4), 'rigid transform')
    r = t[:3, :3]
    if (not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-6, rtol=0)):
        raise ValidationError('transform must contain a proper rotation and homogeneous last row')
    return t


def transform_record(matrix, source, target):
    if frame(source) == frame(target):
        raise ValidationError('source and target frames must differ')
    return {'source_frame': source, 'target_frame': target, 'translation_unit': 'm',
            'angle_unit': 'rad', 'direction': 'p_target = T_target_source * p_source',
            'matrix': rigid(matrix).tolist()}


def read_transform(record, source=None, target=None):
    if (record.get('translation_unit') != 'm' or record.get('angle_unit') != 'rad'
            or record.get('direction') != 'p_target = T_target_source * p_source'):
        raise ValidationError('transform units/direction missing or unsupported')
    a, b = frame(record['source_frame']), frame(record['target_frame'])
    if a == b or (source is not None and a != source) or (target is not None and b != target):
        raise ValidationError('transform frame/direction mismatch')
    return rigid(record['matrix'])


def inverse(matrix):
    t = rigid(matrix)
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def apply(matrix, point):
    t = rigid(matrix)
    p = finite(point, (3,), 'point (m)')
    return t[:3, :3] @ p + t[:3, 3]


def rotation_angle(rotation):
    # atan2 remains well behaved near identity and pi.
    r = finite(rotation, (3, 3), 'rotation')
    sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]) / 2
    return float(np.arctan2(sine, np.clip((np.trace(r)-1)/2, -1, 1)))


def pose_matrix(position, quaternion_xyzw):
    q = finite(quaternion_xyzw, (4,), 'quaternion xyzw')
    if not np.isclose(np.linalg.norm(q), 1., atol=1e-5, rtol=0):
        raise ValidationError('quaternion must be unit length')
    x, y, z, w = q / np.linalg.norm(q)
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = finite(position, (3,), 'position (m)')
    return t


def camera_model(info, image_geometry):
    """Require a complete CameraInfo; no ROI/binning or stereo projection guess."""
    frame(info['frame_id'])
    for name in ('width', 'height'):
        if type(info[name]) is not int or info[name] <= 0:
            raise ValidationError('CameraInfo dimensions must be positive integers')
    k = finite(info['k'], (9,), 'K').reshape(3, 3)
    d = np.asarray(info['d'], dtype=float)
    r = finite(info['r'], (9,), 'R').reshape(3, 3)
    p = finite(info['p'], (12,), 'P').reshape(3, 4)
    if d.ndim != 1 or not np.isfinite(d).all():
        raise ValidationError('invalid distortion coefficients')
    roi = info['roi']
    if (info['binning_x'] not in (0, 1) or info['binning_y'] not in (0, 1)
            or any(roi[x] != 0 for x in ('x_offset', 'y_offset', 'width', 'height'))
            or roi['do_rectify']):
        raise ValidationError('ROI/binning CameraInfo requires explicit upstream normalization')
    model = info['distortion_model']
    if model not in ('plumb_bob', 'rational_polynomial'):
        raise ValidationError(f'unsupported distortion model: {model}')
    if len(d) not in ((0, 5) if model == 'plumb_bob' else (8,)):
        raise ValidationError('distortion coefficient count mismatch')
    if image_geometry == 'rectified':
        if not np.allclose(r, np.eye(3), atol=1e-9) or not np.allclose(p[:, 3], 0):
            raise ValidationError('nonidentity rectification/stereo offset is unsupported')
        k, d = p[:, :3], np.zeros(5)
    elif image_geometry != 'raw':
        raise ValidationError('image_geometry must explicitly be raw or rectified')
    if (k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
            or k[0, 1] != 0 or k[1, 0] != 0):
        raise ValidationError('uncalibrated/unsupported pinhole intrinsics')
    return k, d


def depth_metres(raw, encoding, scale_m, min_m, max_m):
    positive(scale_m, 'depth scale (m per stored unit)')
    positive(min_m, 'minimum usable depth')
    positive(max_m, 'maximum usable depth')
    if min_m >= max_m:
        raise ValidationError('depth range must be increasing')
    if encoding not in ('16UC1', '32FC1'):
        raise ValidationError(f'unsupported depth encoding: {encoding}')
    if encoding == '32FC1' and scale_m != 1.0:
        raise ValidationError('32FC1 requires explicitly confirmed metres, scale_m=1')
    if not np.isscalar(raw) or not np.isfinite(raw) or raw <= 0:
        raise ValidationError('invalid depth: zero, negative, NaN or infinity')
    if encoding == '16UC1' and (raw > 65535 or float(raw) != int(raw)):
        raise ValidationError('16UC1 value outside unsigned 16-bit integer range')
    z = float(raw) * scale_m
    if not math.isfinite(z) or not min_m <= z <= max_m:
        raise ValidationError('depth outside configured usable range')
    return z


def deproject(pixel_uv, raw_depth, info, image_geometry, encoding, scale_m, min_m, max_m):
    u, v = finite(pixel_uv, (2,), 'pixel (u,v)')
    k, d = camera_model(info, image_geometry)
    if not (0 <= u < info['width'] and 0 <= v < info['height']):
        raise ValidationError('pixel outside image')
    z = depth_metres(raw_depth, encoding, scale_m, min_m, max_m)
    if np.any(d):
        try:
            import cv2
        except ImportError as exc:
            raise ValidationError('OpenCV required to undistort raw nonzero-distortion pixels') from exc
        ray = cv2.undistortPoints(np.array([[[u, v]]]), k, d).reshape(2)
    else:
        ray = np.array([(u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1]])
    return finite([ray[0]*z, ray[1]*z, z], (3,), 'deprojected point')


def decode_image(data, width, height, step, encoding, is_bigendian):
    """Lossless ROS Image buffer decoding, including row padding and byte order."""
    types = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if encoding not in types or any(type(x) is not int or x <= 0 for x in (width, height, step)):
        raise ValidationError('unsupported image encoding/dimensions')
    kind, channels = types[encoding]
    dtype = np.dtype(('>' if is_bigendian else '<') + kind)
    row = width * channels * dtype.itemsize
    if step < row or len(data) != step * height:
        raise ValidationError('Image data length/step mismatch')
    shape = (height, width, channels) if channels == 3 else (height, width)
    strides = (step, channels*dtype.itemsize, dtype.itemsize) if channels == 3 else (step, dtype.itemsize)
    return np.ndarray(shape, dtype=dtype, buffer=bytes(data), strides=strides).astype(dtype.newbyteorder('='), copy=True)
