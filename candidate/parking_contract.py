"""Explicit approved R80 first-qualified-zero-ACK parking contract; no IO."""
import copy
import math
import numpy as np
from parking_zero_contract import validate_zero_ack,integer

MODE='ordinary_zero_ack_fixed5'
SPEC=dict(schema='ordinary_terminal_zero_ack_fixed5/v1',enabled=True,
    T0='first_core_precise_pose_and_actual_zero_input_ACK_qualification_clock',
    duration_ns=5_000_000_000,reset_window=False,keep_legacy_arrival_window=True,
    capture_timeout='original_active_goal_timeout_s_from_separate_capture_begin',
    capture_xy_m=.025,capture_yaw_rad=.035,capture_speed_mps=.03,capture_yawdot_radps=.06,
    capture_dwell_ns=600_000_000,xy_drift_m=.05,yaw_drift_rad=.1,
    native_speed_mps=.08,native_wz_radps=.1,native_yawdot_radps=.1,source_age_ns=300_000_000)

def enabled(profile):
    value=profile.get('ordinary_terminal_parking_contract')
    if value is None:return False
    if value!=SPEC:raise ValueError('Approved ordinary parking contract differs')
    return True

def qualification(status,run_id,request_id,now_ns):
    cascade=status.get('cascade_parking')or{}
    owner=cascade.get('ordinary_parking')or{}
    proof=owner.get('first_actor_zero_ack')
    if proof is None:return None
    if (cascade.get('mode')!='ordinary_zero_hold' or cascade.get('protection_active') or
        cascade.get('failure_latched') or owner.get('failure') or
        owner.get('stage')not in ('handoff_zero','zero_hold')):
        raise ValueError('Ordinary zero qualification is protected/failed')
    marker=cascade['ordinary_terminal_zero_event']
    if marker['run_id']!=run_id or marker['request_id']!=request_id:
        raise ValueError('Foreign ordinary parking qualification')
    if marker['goal_sha256']!=cascade['fixed_goal_sha256'] or marker['path_sha256']!=cascade['path_sha256']:
        raise ValueError('Ordinary qualification goal/path changed')
    if marker.get('schema')=='ordinary_terminal_zero_event/v2':
        from r105_terminal_reference import validate_fields
        record=validate_fields(cascade,run_id)
        if record is None or marker.get('reference_sha256')!=cascade.get('reference_sha256'):
            raise ValueError('v2 first qualification lost terminal reference')
        frame=status.get('known_scene_localization')
        from r105_terminal_reference import digest
        if (not isinstance(frame,dict) or frame.get('generation')!=record['generation']
            or digest([frame.get('generation'),frame.get('C_world_odom'),frame.get('T_navigation_world')])!=record['coordinate_frame_sha256']):
            raise ValueError('v2 first qualification changed coordinate generation')
    elif any(k in cascade for k in ('reference_kind','terminal_reference','reference_sha256')):
        raise ValueError('v1 qualification cannot borrow terminal identity')
    ack=dict(proof['ack_source_fields'],ordinary_zero_before_actor_observation=proof)
    validate_zero_ack(ack,marker,now_ns)
    start=integer(proof['qualification_clock_ns']);stamp=integer(proof['qualification_pose_stamp_ns'])
    if not proof['ack_native_clock_ns']<=start<=integer(now_ns) or now_ns-start>300_000_000 or not 0<=start-stamp<=300_000_000:
        raise ValueError('Ordinary qualification clock is stale/future')
    origin=proof['qualification_origin'];position=np.asarray(origin['position'],float)
    q=np.asarray(origin['quaternion_xyzw'],float);goal=np.asarray(cascade['fixed_goal']['position_world_xyz'],float)
    if (position.shape!=(3,) or q.shape!=(4,) or not np.isfinite(position).all() or not np.isfinite(q).all()
        or not .98<=float(q@q)<=1.02 or origin.get('navigation_ground_truth_used')is not False
        or origin.get('source_topic')!='/demo/slam/body_odom' or origin['stamp_ns']!=stamp):
        raise ValueError('No exact original SLAM qualification origin')
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    error=math.atan2(math.sin(cascade['fixed_goal']['heading_rad']-yaw),math.cos(cascade['fixed_goal']['heading_rad']-yaw))
    if (np.linalg.norm(position[:2]-goal[:2])>.025 or abs(error)>.035
        or np.linalg.norm(np.asarray(origin['body_velocity'])[:2])>=.03
        or abs(proof['qualification_source_yawdot_radps'])>=.06
        or proof['ready_pose_stamp_ns']!=owner['ready_pose_stamp_ns']):
        raise ValueError('First zero ACK lacks its actual precise low-speed source')
    return dict(contract=MODE,start_stamp_ns=start,end_stamp_ns=start+5_000_000_000,
        origin=copy.deepcopy(origin),event=copy.deepcopy(marker),proof=copy.deepcopy(proof),
        first_qualification_immutable=True,formal_window_never_reset=True,navigation_ground_truth_used=False)
