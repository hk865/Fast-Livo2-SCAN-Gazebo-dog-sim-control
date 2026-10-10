# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""Static scenario-axis calibration from time-matched IMU and SLAM attitudes.

This module has no ROS subscriptions, truth input, or pose correction. Callers
must explicitly declare the IMU orientation reference in world coordinates.
The resulting yaw is frozen once, and used only to transform prescribed scene
waypoint offsets into camera_init. Runtime localization remains FAST-LIVO2.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_rotation(value):
    q=np.asarray(value,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:
        raise ValueError('orientation requires finite nonzero xyzw quaternion')
    return Rotation.from_quat(q).as_matrix()


def yaw_rotation(yaw):
    if not math.isfinite(yaw):raise ValueError('yaw must be finite')
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]],dtype=float)


def scene_heading_sample(imu_quaternion, slam_body_quaternion, *,
                         imu_reference_world_quaternion, body_imu_quaternion):
    """Return R_camera_init_world yaw from one simultaneous attitude pair.

    IMU message: R_ref_imu. The mandatory known reference: R_world_ref.
    IMU mounting: R_body_imu. SLAM message: R_camera_init_body.
      R_world_body = R_world_ref R_ref_imu R_body_imu.T
      R_camera_init_world = R_camera_init_body R_world_body.T
    Both world and camera_init must be gravity aligned; residual roll/pitch is
    returned for validation. Full 3D body/IMU mount rotations cancel correctly.
    """
    rwr=quaternion_rotation(imu_reference_world_quaternion)
    rri=quaternion_rotation(imu_quaternion)
    rbi=quaternion_rotation(body_imu_quaternion)
    rcb=quaternion_rotation(slam_body_quaternion)
    rwb=rwr@rri@rbi.T
    rcw=rcb@rwb.T
    roll,pitch,yaw=Rotation.from_matrix(rcw).as_euler('xyz')
    return {'yaw_camera_init_from_world':float(yaw),
            'gravity_axis_residual_rad':float(math.hypot(roll,pitch)),
            'world_body_yaw':float(math.atan2(rwb[1,0],rwb[0,0])),
            'slam_body_yaw':float(math.atan2(rcb[1,0],rcb[0,0]))}


def calibrate_scene_heading(pairs, *, imu_reference_world_quaternion,
                            body_imu_quaternion, reference_description,
                            max_stamp_difference_s=.02, min_samples=10,
                            min_span_s=.8, max_spread_rad=math.radians(1),
                            max_gravity_residual_rad=math.radians(3)):
    """Validate a stable collection of actual, timestamp-paired attitudes.

    Pair keys: imu_stamp, slam_stamp, imu_quaternion, slam_body_quaternion.
    Invalid orientation, missing source reference, clock reversal, stale pairing
    or inconsistent estimates fail. The caller enforces stationary initialization
    and freezes this result; this function never follows later robot yaw.
    """
    if not isinstance(reference_description,str) or not reference_description.strip():
        raise ValueError('the IMU world-reference assumption must be recorded explicitly')
    observations=[];stamps=[];stamp_errors=[]
    for pair in pairs:
        ti,ts=float(pair['imu_stamp']),float(pair['slam_stamp'])
        if not all(map(math.isfinite,[ti,ts])):raise ValueError('non-finite sensor timestamps')
        if abs(ti-ts)>max_stamp_difference_s:raise ValueError('IMU and SLAM timestamps differ by more than pairing tolerance')
        if stamps and ts<=stamps[-1]:raise ValueError('SLAM calibration timestamps must increase')
        observation=scene_heading_sample(pair['imu_quaternion'],pair['slam_body_quaternion'],
            imu_reference_world_quaternion=imu_reference_world_quaternion,body_imu_quaternion=body_imu_quaternion)
        observations.append(observation);stamps.append(ts);stamp_errors.append(abs(ti-ts))
    if len(observations)<min_samples or stamps[-1]-stamps[0]<min_span_s:
        raise ValueError('not enough time-matched stable heading samples')
    angles=np.array([r['yaw_camera_init_from_world'] for r in observations])
    mean=float(math.atan2(np.sin(angles).mean(),np.cos(angles).mean()))
    errors=np.arctan2(np.sin(angles-mean),np.cos(angles-mean))
    spread=float(np.max(np.abs(errors)))
    tilt=max(r['gravity_axis_residual_rad'] for r in observations)
    if spread>max_spread_rad:raise ValueError('heading estimates are not consistent during initialization')
    if tilt>max_gravity_residual_rad:raise ValueError('world and camera_init gravity axes are inconsistent')
    return {'yaw_camera_init_from_world':mean,'rotation_camera_init_from_world':yaw_rotation(mean).tolist(),
            'sample_count':len(observations),'first_stamp':stamps[0],'last_stamp':stamps[-1],
            'max_stamp_difference_s':max(stamp_errors),'max_heading_spread_rad':spread,
            'max_gravity_residual_rad':tilt,'imu_reference_description':reference_description,
            'imu_reference_world_quaternion':list(imu_reference_world_quaternion),
            'body_imu_quaternion':list(body_imu_quaternion),'ground_truth_used':False,
            'purpose':'single frozen scene-waypoint axis calibration; no SLAM pose correction'}


def scene_waypoints_in_slam(world_axis_offsets, slam_origin, yaw_camera_init_from_world):
    """p_camera_init = initial_body_position + Rz(theta) scene_world_offset.

    z is a relative floor elevation and is not tilted with the robot. Absolute
    world coordinates must first have the known scene anchor subtracted.
    """
    offsets=np.asarray(world_axis_offsets,dtype=float);origin=np.asarray(slam_origin,dtype=float)
    if offsets.ndim!=2 or offsets.shape[1]!=3 or origin.shape!=(3,)\
            or not np.isfinite(offsets).all() or not np.isfinite(origin).all():
        raise ValueError('waypoints must be finite N×3 offsets and origin a finite 3-vector')
    return (offsets@yaw_rotation(yaw_camera_init_from_world).T+origin).tolist()
