"""Lossless PointCloud2 field decoding. No ROS dependency or assumed PCL layout."""
from __future__ import annotations
import hashlib
import numpy as np

POINT_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def decode_cloud(metadata, raw):
    """Return all original structured fields, finite float64 XYZ and source indices.

    Raw payload bytes, including row/point padding and NaNs, are preserved by
    the caller. XYZ conversion is explicit; structured fields keep original
    float widths, integer fields, vector counts and byte order.
    """
    w, h = int(metadata['width']), int(metadata['height'])
    step, row = int(metadata['point_step']), int(metadata['row_step'])
    if min(w, h, step) <= 0 or row < w * step or len(raw) != row * h:
        raise ValueError('invalid PointCloud2 dimensions or payload length')
    names, formats, offsets = [], [], []
    endian = '>' if metadata['is_bigendian'] else '<'
    occupied = []
    for f in metadata['fields']:
        name, count, offset, kind = f['name'], int(f['count']), int(f['offset']), int(f['datatype'])
        if not name or name in names or kind not in POINT_TYPES or count < 1 or offset < 0:
            raise ValueError('invalid or duplicate PointField')
        dt = np.dtype(endian + POINT_TYPES[kind]); end = offset + dt.itemsize * count
        if end > step or any(offset < b and end > a for a, b in occupied):
            raise ValueError('PointField overlap/out of point_step')
        occupied.append((offset, end)); names.append(name); offsets.append(offset)
        formats.append(dt if count == 1 else (dt, (count,)))
    by_name = {f['name']: f for f in metadata['fields']}
    if any(k not in by_name or int(by_name[k]['count']) != 1 or int(by_name[k]['datatype']) not in (7, 8)
           for k in ('x', 'y', 'z')):
        raise ValueError('XYZ must be scalar FLOAT32/FLOAT64 fields')
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': step})
    # Strides retain organized-cloud row padding rather than mistaking it for a point.
    view = np.ndarray((h, w), dtype=dtype, buffer=raw, strides=(row, step))
    # NumPy structured copy may leave padding uninitialized; explicitly copy
    # every original point byte, then view it with the original field layout.
    point_bytes = np.ndarray((h, w, step), dtype=np.uint8, buffer=raw,
                             strides=(row, step, 1)).copy(order='C').reshape(-1)
    fields = point_bytes.view(dtype)
    xyz = np.column_stack([fields[n].astype(np.float64) for n in ('x', 'y', 'z')])
    valid = np.isfinite(xyz).all(axis=1)
    return fields, np.ascontiguousarray(xyz[valid]), np.flatnonzero(valid).astype(np.int64)


def metadata_from_message(msg):
    return {'width': int(msg.width), 'height': int(msg.height), 'point_step': int(msg.point_step),
            'row_step': int(msg.row_step), 'is_bigendian': bool(msg.is_bigendian), 'is_dense': bool(msg.is_dense),
            'fields': [{'name': f.name, 'offset': int(f.offset), 'datatype': int(f.datatype), 'count': int(f.count)}
                       for f in msg.fields]}


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(float(q @ q) - 1.) > .02:
        raise ValueError('SLAM quaternion invalid')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class GravityCalibration:
    """Freeze up from stationary actual accelerometer + SLAM IMU rotation.

    Does not use the IMU message orientation, simulator gravity, world yaw, SDF
    or an assumed camera_init Z. Dynamic samples cannot update the frozen axis.
    """
    def __init__(self, min_samples=50, min_span_ns=500_000_000, min_pose_samples=4):
        self.min_samples = min_samples; self.min_span_ns = min_span_ns
        self.min_pose_samples = min_pose_samples; self.samples = []; self.result = None

    def observe(self, imu, imu_pose, body_pose):
        if self.result is not None:
            return self.result
        stamp = imu['original_stamp_ns']
        if (imu.get('frame_id')!='imu_link' or imu_pose.get('frame_id')!='camera_init'
                or imu_pose.get('child_frame_id')!='aft_mapped' or body_pose.get('frame_id')!='camera_init'
                or body_pose.get('child_frame_id')!='demo_slam_body'
                or abs(stamp-imu_pose['original_stamp_ns']) > 150_000_000
                or abs(stamp-body_pose['original_stamp_ns']) > 150_000_000
                or not 0<=imu['received_monotonic_wall']-imu_pose['received_monotonic_wall']<.3
                or not 0<=imu['received_monotonic_wall']-body_pose['received_monotonic_wall']<.3
                or max(body_pose.get('twist_covariance',[1e6]*36)[7*k]for k in range(3))>=1.
                or not np.isfinite(imu['angular_velocity']).all()
                or np.linalg.norm(imu['angular_velocity']) > .1
                or np.linalg.norm(body_pose['body_linear_velocity']) > .05):
            return None
        accel = np.asarray(imu['linear_acceleration'], dtype=float)
        norm = float(np.linalg.norm(accel))
        if not np.isfinite(accel).all() or not 9.0 <= norm <= 10.6:
            return None
        if self.samples and stamp <= self.samples[-1][0]:
            return None
        up = rotation(imu_pose['quaternion']) @ accel / norm
        self.samples.append((stamp, up, int(imu_pose['original_stamp_ns'])))
        if len(self.samples) > 2000:
            self.samples.pop(0)
        if (len(self.samples) < self.min_samples or self.samples[-1][0]-self.samples[0][0] < self.min_span_ns
                or len({s[2] for s in self.samples}) < self.min_pose_samples):
            return None
        mean = np.mean([s[1] for s in self.samples], axis=0); mean /= np.linalg.norm(mean)
        deviations = np.arccos(np.clip([float(s[1] @ mean) for s in self.samples], -1, 1))
        if float(np.max(deviations)) > np.deg2rad(3):
            return None
        self.result = {'schema': 'actual_slam_accelerometer_up/v1', 'up_camera_init': mean.tolist(),
                       'sources': ['/livox/imu linear_acceleration', '/aft_mapped_to_init SLAM IMU quaternion',
                                   '/demo/slam/body_odom stationary velocity gate'],
                       'first_stamp_ns': self.samples[0][0], 'last_stamp_ns': self.samples[-1][0],
                       'samples': len(self.samples), 'unique_slam_imu_pose_samples': len({s[2] for s in self.samples}),
                       'maximum_axis_deviation_rad': float(np.max(deviations)), 'frozen_once': True,
                       'calibration_sample_receipts': [{'imu_stamp_ns':s[0],'slam_imu_pose_stamp_ns':s[2],
                           'transformed_up_camera_init':s[1].tolist()}for s in self.samples],
                       'imu_orientation_field_used': False, 'ground_truth_used': False}
        return self.result
