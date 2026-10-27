"""Explicit isolated PCT/SCAN/Student experiment scope; no inherited R83 pass."""
from project_config import loads as _project_loads
from pathlib import Path
import hashlib
import json
import math
import shutil

SESSION_SCHEMA='r85_pct_scan_student_session/v1'
TASK_SCHEMA='r85_pct_scan_student_task/v1'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def validate_turn_brake_reference(profile):
    cascade=profile.get('cascade',{})
    if not isinstance(cascade,dict) or 'r85_turn_brake_reference'not in cascade:return None
    from cascade_core import turn_brake_reference_configuration
    kind=turn_brake_reference_configuration(cascade['r85_turn_brake_reference'])
    if (profile.get('r85_active_turn_braking')!={'enabled':True}
            or type(profile['r85_active_turn_braking']['enabled'])is not bool
            or cascade.get('external_heading_gate')is not True):
        raise ValueError('Explicit turn-brake reference requires active braking and external heading ownership')
    return kind

def validate_student_yaw_pd_fade(profile):
    value=profile.get('r85_student_yaw_pd_fade')
    if value is None:return None
    if type(value)is not dict or type(value.get('enabled'))is not bool:raise ValueError('Strict yaw PD fade option required')
    if value=={'enabled':False}:return None
    expected={'enabled':True,'kind':'accepted_pre_zero_event_yaw_PD_fade_v1','maximum_alpha_rate_per_s':1.0}
    if value!=expected or any(type(value.get(k))is not type(v)for k,v in expected.items()):raise ValueError('Unselected yaw PD fade law')
    from pre_zero_integral_unload import configuration
    if configuration(profile)is None:raise ValueError('Yaw PD fade requires original real prezero event authority')
    if profile.get('r85_stance_support_handoff')!={'enabled':True,'kind':'locked_heading_PD_stance_support_v1'}:
        raise ValueError('R42 preserves the selected R41 stance handoff')
    from runtime_limits import configuration as resource_configuration
    if resource_configuration(profile)is None:raise ValueError('R42 requires explicit renewed resource authority')
    return dict(value)

SHORT_TIMING_KEY='r74_short_timing_horizon'
SHORT_TIMING_SCHEMA='r74_native180_task770_timing_resource_window/v1'


def validate_short_timing_horizon(p):
    """Default-off resource window only; never shorten the task or grant PASS."""
    if SHORT_TIMING_KEY not in p:return False
    value=p[SHORT_TIMING_KEY]
    if (type(value)is not dict or set(value)!={'enabled','schema'}
            or type(value['enabled'])is not bool or value['enabled']is not True
            or type(value['schema'])is not str or value['schema']!=SHORT_TIMING_SCHEMA):
        raise ValueError('Exact R74 native180 task770 timing resource selector required')
    from minimal_simulation_baseline import enabled as minimal_enabled
    from r74_teacher_substage_timing import enabled as timing_enabled
    if not minimal_enabled(p) or not timing_enabled(p):
        raise ValueError('Short timing horizon requires existing minimal and timing opt-ins')
    diagnostic=p.get('diagnostic_observe_only')
    formal=p.get('r85_formal_regression')
    task=p.get('r85_task')
    if (type(diagnostic)is not dict or diagnostic.get('enabled')is not True
            or diagnostic.get('simulation_only')is not True
            or diagnostic.get('formal_acceptance_pass_allowed')is not False
            or diagnostic.get('navigation_truth_injection')is not False
            or diagnostic.get('recording_requested')is not False
            or diagnostic.get('full_evidence_after_endpoint')is not False
            or type(diagnostic.get('maximum_physics_rounds'))is not int
            or diagnostic['maximum_physics_rounds']!=1
            or type(formal)is not dict or set(formal)!={'enabled'}
            or type(formal['enabled'])is not bool or formal['enabled']is not False
            or p.get('navigation_ground_truth_used')is not False
            or p.get('controller_kind')!='student' or p.get('actor_kind')!='student'
            or type(p.get('duration_s'))is not int or p['duration_s']!=180
            or type(task)is not dict or task.get('kind')!='f1_to_f3'
            or type(task.get('maximum_task_sim_s'))is not int or task['maximum_task_sim_s']!=770):
        raise ValueError('Short timing is single-run simulation-only unverified native180/task770')
    return True


def validate_profile(p,here):
    """Validate current interfaces and numeric safety, not a historical version."""
    from diagnostic_observation import strict_formal_enabled
    from runtime_limits import configuration
    from clock_hold import CONTRACT
    from parking_contract import SPEC
    from navigation_stop_window import SPEC as SETTLE
    from parking_pose10hz import configuration as parking_configuration
    strict_formal_enabled(p)
    configuration(p)
    if p.get('navigation_ground_truth_used')is not False or p.get('actor_kind')!='student' or p.get('controller_kind')!='student':raise ValueError('Student/actual-SLAM navigation required')
    if p.get('mission46_required')is not False:raise ValueError('Single full F1-F3 task required')
    from safety_configuration import clock,parking,settle
    clock(p.get('control_clock_contract'));parking(p.get('ordinary_terminal_parking_contract'));settle(p.get('navigation_settle_statistics_contract'))
    cascade=p['cascade']
    if p.get('pose_cloud_timeout_s')!=.3 or cascade.get('feedback_ttl_sim_and_wall_s')!=.3 or cascade.get('maximum_gyro_pair_gap_s')!=.02:raise ValueError('Source/IMU freshness limits invalid')
    if not parking_configuration(cascade.get('parking_pose10hz')):raise ValueError('10Hz fixed parking configuration required')
    if cascade.get('command_limits')!=[.6,.2,.6] or cascade.get('command_slew_per_s')!=[.6,.6,.8]:raise ValueError('Command safety envelope invalid')
    if cascade.get('navigation_ground_truth_used')is not False or cascade.get('feedback_expected_hz')!=p['sensor_sampling']['camera_hz']:raise ValueError('Actual sensor feedback required')
    if p.get('raw_storage_budget')!=dict(max_run_bytes=170000000000,minimum_start_available_bytes=107374182400,minimum_runtime_available_bytes=107374182400):raise ValueError('Filesystem minimums invalid')
    duration=float(p['duration_s']);task=p['r85_task']
    if not math.isfinite(duration)or not 30<=duration<=1500:raise ValueError('Finite simulation duration required')
    if task.get('kind')!='f1_to_f3' or task.get('progress_window_sim_s')!=90 or task.get('minimum_progress_m')!=.5 or not 0<task['maximum_task_sim_s']<=duration:raise ValueError('Task/progress horizon invalid')
    route=_project_loads(Path(task['typed_route_source_file']).read_text())
    if route.get('schema')!='r85_prepared_typed_global_reference/v1' or route.get('physical_static_allowed')is not True:raise ValueError('Typed route format invalid')
    if p['spawn']!=[0.,0.,.3,0.]or p['expected_asset']['spawn']!=p['spawn']:raise ValueError('Physical spawn mismatch')
    from student_lower_loop_ack import configuration as ack_configuration
    ack_configuration(cascade.get('r85_student_lower_loop_ack'))
    from r109_speed_tiers import configuration as speed_configuration
    speed_configuration(p)
    validate_turn_brake_reference(p);validate_student_yaw_pd_fade(p)
    return duration,p['overview_pose']

def prepare_task_inputs(run,p,here):
    run=Path(run);here=Path(here)
    original=Path(p['r85_task']['typed_route_source_file']).resolve()
    source=_project_loads(original.read_text())
    prior_path=Path(source['physical_sources']['known_scene_reference_file'])
    prior=_project_loads(prior_path.read_text());actual=_project_loads((run/'known_scene_reference.json').read_text())
    if len(prior['boxes'])!=43 or prior['boxes']!=actual['boxes'] or prior['excluded_models']!=actual['excluded_models']:
        raise ValueError('Generated actual world static geometry differs from PCT physical certificate')
    with (run/'r85_scene_geometry_equivalence.json').open('x')as f:
        json.dump(dict(schema='r85_actual_static_geometry_equivalence/v1',run_id=run.name,
            actual_world_file=str(run/'world.sdf'),actual_world_sha256=sha(run/'world.sdf'),
            actual_reference_sha256=sha(run/'known_scene_reference.json'),prior_reference_file=str(prior_path),
            prior_reference_sha256=sha(prior_path),static_boxes_exact_equal=True,box_count=43,
            dynamic_actor_is_separately_observed=True,actual_robot_pose_not_used=True),f,indent=2);f.write('\n')
    for name,src in [('r85_typed_route_source.json',original),('r85_physical_route_admission.json',Path(source['physical_certificate_file']))]:
        dst=run/name
        with dst.open('xb')as f:
            f.write((json.dumps(_project_loads(src.read_bytes()),indent=2)+'\n').encode())
    task=dict(schema=TASK_SCHEMA,run_id=run.name,kind=p['r85_task']['kind'],
        navigation_ground_truth_used=False,known_map_assistance=True,
        actual_SLAM_canonical_registration_required=True,actual_SCAN_curve_physical_certificate_required=True,
        source_typed_route_file=str(run/'r85_typed_route_source.json'),
        source_typed_route_sha256=sha(run/'r85_typed_route_source.json'),
        source_static_certificate_file=str(run/'r85_physical_route_admission.json'),
        source_static_certificate_sha256=sha(run/'r85_physical_route_admission.json'),
        original_goal_protocol=p['original_scenario']['goal_region_profile'],
        original_F3_goal=p['original_scenario']['route_goals']['navigation_f1_f3'][-1],
        automatic_path_is_not_original_46_region_sequence=True,
        timing=dict(whole_task_maximum_sim_s=p['r85_task']['maximum_task_sim_s'],
            progress_window_sim_s=90,minimum_forward_progress_m=.5,
            temporal_scope_change='Single automatic global goal uses a 90s progress watchdog plus original 1500s overall cap; original per-region timeout is retained as historical evidence, not silently reused as one 35m deadline'),
        parking_contract=p['ordinary_terminal_parking_contract'],
        simulation_only=True,real_robot=False,training=False,
        raw_route_previous_rejections_retained=True)
    if task['kind']=='business_chain_prefix':
        task.update(prefix_length_m=float(p['r85_task']['prefix_length_m']),
            short_chain_validation_only=True,full_F3_PASS_allowed=False,dynamic_required=False)
    else:
        task.update(short_chain_validation_only=False,full_F3_PASS_allowed=True,dynamic_required=True)
    with (run/'r85_task_contract.json').open('x')as f:json.dump(task,f,indent=2,allow_nan=False);f.write('\n')

def runtime_stop_decision(run,observed_native):
    # Original native fault events remain hard and precede every endpoint or
    # worker completion observation. No native pose enters navigation here.
    if observed_native.get('fault_record') is not None:
        return dict(stop=True,reason='actual_original_native_fault',
            actual_fault_record=observed_native['fault_record'],navigation_PASS=False)
    run=Path(run)
    diagnostic_path=run/'r85_diagnostic_result.json'
    if diagnostic_path.exists():
        from diagnostic_observation import enabled
        profile=_project_loads((run/'input_profile.json').read_text())
        result=_project_loads(diagnostic_path.read_text())
        if (not enabled(profile) or result.get('run_id')!=run.name
                or result.get('formal_full_route_pass')is not False):
            raise ValueError('Foreign or non-diagnostic terminal result')
        return dict(stop=True,reason='actual_R85_diagnostic_terminated',diagnostic_result=result)
    p=run/'r85_task_result.json'
    if p.exists():
        d=_project_loads(p.read_text())
        if d.get('run_id')!=run.name:raise ValueError('Foreign R85 task result')
        if d.get('state')=='failed':return dict(stop=True,reason='actual_R85_task_failed',task_result=d)
    for name,key in [('navigation_status.json','state'),('r85_task_status.json','state')]:
        p=run/name
        if p.exists():
            d=_project_loads(p.read_text())
            if d.get(key)=='failed':return dict(stop=True,reason='actual_R85_'+name+'_failed',actual_status=d)
    return dict(stop=False,reason='bounded_task_active')
