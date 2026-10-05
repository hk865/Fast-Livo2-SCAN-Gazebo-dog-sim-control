"""Only add prospective active parking to a hash-verified frozen v1 profile.

The original analytic path, driving gains, speed, native model and feedback
cadence are preserved. No launch, policy inference or original writes.
"""
import hashlib
import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('go2_active_parking_core',Path(__file__).with_name('core.py'))
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parking_contract(path):
    target=core.parking_target_record(path)
    return dict(schema='fixed_endpoint_active_parking/v1',candidate='weak_hold_p1',
        target_kind='prospective_fixed_path_endpoint',target=target,target_sha256=core.parameter_hash(target),
        position_kp=.18,position_kd=.20,yaw_kp=.65,yaw_kd=.18,
        position_deadband_m=.003,yaw_deadband_rad=.005,
        reference_xy_norm_limit_mps=.025,reference_yaw_limit_radps=.07,
        command_limits_body=[.06,.04,.10],capture_xy_entry_m=.025,capture_xy_hold_m=.03,
        capture_yaw_entry_rad=.035,capture_yaw_hold_rad=.045,
        capture_actual_xy_speed_mps=.03,capture_actual_yawrate_radps=.06,capture_fresh_dwell_s=.6,
        target_loss_radius_m=.3,target_loss_heading_rad=.6,
        first_fixed_parking_window_s=5.,maximum_xy_drift_m=.05,maximum_yaw_drift_rad=.1,
        maximum_native_xy_speed_mps=.08,maximum_native_Euler_yawrate_radps=.1,maximum_native_body_wz_radps=.1,
        inner_PI='unchanged v1 gains, separate parking integral reset exactly once at capture',
        filters='preserve drive-filter states, original .2s/.1s; fresh native measurements only',
        command_slew_per_s=[.6,.6,.8],feedback_TTL_sim_and_wall_s=.3,
        capture_window_source='Strictly increasing fresh native feedback stamps; no held-row dwell extension',
        hold_window_selection='First declared active_hold native state timestamp; never choose a later quiet window',
        actual_command_must_be_zero=False,continuous_Teacher_inference_required=True,
        action_zero_is_parking=False,SLAM_verified=False,real_robot_verified=False)


def from_v1(source_path,expected_sha256,trial_role='pilot',repetition=1):
    """Read exact v1 profile, then derive one new candidate without drive edits.

    Root subsequently supplies the complete v2 source freeze before launch.
    Only parking configuration, revision/provenance and trial labels change.
    """
    source_path=Path(source_path).resolve()
    if sha(source_path)!=expected_sha256:raise ValueError('Frozen v1 profile SHA mismatch')
    old=json.loads(source_path.read_text())
    if old['schema']!='truth_teacher_static_curve_profile/v1':raise ValueError('Frozen v1 profile required')
    if trial_role not in ('pilot','confirmation') or repetition not in (1,2,3):
        raise ValueError('One pilot and three prospective confirmation repeats only')
    path=core.StaticPath(old['path'])
    if old['path_parameters_sha256']!=path.sha256:raise ValueError('Original path parameter SHA mismatch')
    profile=json.loads(json.dumps(old,allow_nan=False))
    profile.update(schema='truth_teacher_static_curve_active_hold_profile/v2',
        scenario=old['scenario']+'_active_hold_weak_p1',design='curvature_cascade_pi_active_hold',
        controller_revision='v3 numerical target binding; weak_p1 active parking; frozen v1 drive preserved',
        campaign_phase='active_parking_pilot' if trial_role=='pilot' else 'active_parking_confirmation',
        trial_role=trial_role,repetition=repetition,
        active_parking=parking_contract(path),
        v1_parent_profile=dict(path=str(source_path),sha256=expected_sha256,
            original_scenario=old['scenario'],original_feedback_hz=old['feedback_hz']),
        v1_frozen_source_hashes=profile.pop('frozen_source_hashes',{}),
        frozen_source_hashes={},source_freeze_pending=True,
        actual_active_parking_verified=False,counts_as_old_zero_command_parking=False)
    profile['v1_drive_fields_sha256']=core.parameter_hash({k:old[k] for k in
        ('path','path_parameters_sha256','path_length_m','maximum_abs_curvature_1pm','desired_speed',
         'feedback_hz','duration_s','post_completion_s','command_limits','gains','base_com_offset','spawn')})
    return profile


def minimal_tight25_campaign(source_path,expected_sha256):
    """Four immutable prospective candidates; no file writing or launches."""
    source=json.loads(Path(source_path).read_text())
    if (source['scenario']!='S_tight' or source['feedback_hz']!=25 or source['desired_speed']!=.3 or
        source['path']['kind']!='s_curve' or source['path']['curve_length_m']!=6. or
        source['path']['heading_amplitude_rad']!=1.):
        raise ValueError('The original exact tight S 25Hz case is required')
    return [from_v1(source_path,expected_sha256,'pilot',1)]+[
        from_v1(source_path,expected_sha256,'confirmation',r) for r in (1,2,3)]
