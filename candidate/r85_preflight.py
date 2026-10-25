"""Explicit isolated PCT/SCAN/Student experiment scope; no inherited R83 pass."""
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
    from r85_resource_renewal import configuration as resource_configuration
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
    if 'r74_protocol_observation_python' in p:
        from r74_protocol_observation_python import enabled as protocol_observation_enabled
        protocol_observation_enabled(p)
    if 'r74_callback_cost_witness' in p:
        from r74_callback_cost_witness import enabled as callback_cost_enabled
        callback_cost_enabled(p)
    if 'r74_corrected_metadata_latest_only' in p:
        from r74_corrected_metadata_qos import subscription_depth as corrected_metadata_depth
        corrected_metadata_depth(p)
    if 'r74_corrected_delivery_witness' in p:
        from r74_corrected_delivery_witness import enabled as corrected_delivery_witness_enabled
        corrected_delivery_witness_enabled(p)
    if 'r74_sensor_hold_witness' in p:
        from r74_sensor_hold_witness import enabled as sensor_hold_witness_enabled
        sensor_hold_witness_enabled(p)
    if 'r74_dynamic_causal_intake' in p:
        from r74_dynamic_causal_intake import enabled as causal_intake_enabled
        causal_intake_enabled(p)
    if 'r74_cooperative_executor' in p:
        from r74_cooperative_executor import enabled as cooperative_executor_enabled
        cooperative_executor_enabled(p)
    from diagnostic_observation import strict_formal_enabled
    strict_formal_enabled(p)  # Exact formal/diagnostic mode validation; no execution permission.
    turn_brake_reference=validate_turn_brake_reference(p)
    from r85_resource_renewal import retained_hard_bytes
    retained_cap=retained_hard_bytes(p)
    validate_student_yaw_pd_fade(p)
    measurement_p_retention=None
    if ('r85_student_yaw_measurement_p_retention' in p or
            isinstance(p.get('cascade',{}).get('r85_student_lower_loop_ack'),dict) and
            'measurement_p_retention_required' in p['cascade']['r85_student_lower_loop_ack']):
        from student_yaw_measurement_feedback import configuration as measurement_p_configuration
        measurement_p_retention=measurement_p_configuration(p)
    if 'r85_teacher_pre_zero_heading_p_taper' in p:
        from cascade_core import teacher_pre_zero_heading_p_taper_configuration
        teacher_pre_zero_heading_p_taper_configuration(p)
    here=Path(here)
    s=p.get('r85_session',{})
    from r85_session_guard import session_deadline_utc
    required=dict(enabled=True,schema=SESSION_SCHEMA,deadline_utc=session_deadline_utc(p),
        simulation_only=True,maximum_retained_bytes=retained_cap,maximum_round_bytes=10000000000)
    if any(s.get(k)!=v for k,v in required.items()):raise ValueError('Explicit overnight isolated R85 session contract differs')
    if p.get('controller_kind')!='student' or p.get('actor_kind')!='student' or p.get('navigation_ground_truth_used')is not False:
        raise ValueError('Actual SLAM/SCAN + fixed Student scope required')
    if p.get('mission46_required')is not False or p.get('continuous_route_contract',{}).get('enabled')is not False:
        raise ValueError('R85 automatic goal task must not masquerade as original46')
    if p.get('review_basis')!='explicit_user_R85_PCT_SCAN_Student_bounded_simulation_session':
        raise ValueError('New R85 experimental scope missing')
    from clock_hold import CONTRACT
    from parking_contract import SPEC
    from navigation_stop_window import SPEC as SETTLE
    if p['control_clock_contract']!=CONTRACT or p['ordinary_terminal_parking_contract']!=SPEC or p['navigation_settle_statistics_contract']!=SETTLE:
        raise ValueError('Original clock/first-zero-ACK fixed5/settle contracts changed')
    if p['pose_cloud_timeout_s']!=.3 or p['cascade']['feedback_ttl_sim_and_wall_s']!=.3 or p['cascade']['maximum_gyro_pair_gap_s']!=.02:
        raise ValueError('Source/causal pair freshness changed')
    if p['diagnostic_scope']['detail_window_sim_s']!=[1600,1601]:raise ValueError('Dense diagnostics outside 1500s only')
    if p['raw_storage_budget']!={'max_run_bytes':retained_cap,'minimum_start_available_bytes':107374182400,'minimum_runtime_available_bytes':107374182400}:
        raise ValueError('Original filesystem/current retained reserve changed')
    diagnostic=p.get('diagnostic_observe_only')
    if diagnostic is not None:
        if (not isinstance(diagnostic,dict) or type(diagnostic.get('enabled'))is not bool
                or diagnostic.get('schema')!='r85_existing_diagnostic_business_observation/v1'
                or diagnostic.get('simulation_only')is not True
                or diagnostic.get('formal_acceptance_pass_allowed')is not False
                or diagnostic.get('maximum_physics_rounds')!=1
                or diagnostic.get('navigation_truth_injection')is not False):
            raise ValueError('Explicit single simulation diagnostic observation contract required')
    duration=float(p['duration_s'])
    if not math.isfinite(duration)or not 30<=duration<=1500:raise ValueError('Finite simulation duration required')
    task=p['r85_task']
    if task['kind'] not in ('business_chain_prefix','f1_to_f3') or task['progress_window_sim_s']!=90 or task['minimum_progress_m']!=.5:
        raise ValueError('Explicit goal/progress deadline protocol required')
    short_timing=validate_short_timing_horizon(p)
    if task['maximum_task_sim_s']>1500 or (task['maximum_task_sim_s']>duration and not short_timing):raise ValueError('Task exceeds original finite horizon')
    typed=Path(task['typed_route_source_file']).resolve()
    if sha(typed)!=task['typed_route_source_sha256']:raise ValueError('PCT typed source bytes changed')
    document=json.loads(typed.read_text())
    if document['schema']!='r85_prepared_typed_global_reference/v1' or document['physical_static_allowed']is not True or document['ready_for_control']is not False:
        raise ValueError('Missing genuine static typed route, runtime remains unverified')
    for role in ('world','geometry_metadata','known_scene_reference','limits'):
        b=document['physical_sources']
        if sha(b[role+'_file'])!=b[role+'_sha256']:raise ValueError('Physical route input bytes changed '+role)
    from scan_workspace import scan_contract
    scan_contract(p)
    from policy_selection import selection,student_sources
    selection();student_sources()
    from automatic_reference_runtime import enabled
    if not enabled(p):raise ValueError('Persistent mode3 reference is required')
    from r109_speed_tiers import configuration as speed_configuration,split_ack
    speed_tier=speed_configuration(p)
    baseline=json.loads((here/'profiles/engineering_nav46.json').read_text())
    from student_lower_loop_ack import (CONFIG as STUDENT_LOWER_ACK_CONFIG,
        MEASUREMENT_CONFIG as MEASUREMENT_LOWER_ACK_CONFIG, configuration as lower_ack_configuration)
    original_cascade=dict(p['cascade'])
    if turn_brake_reference is not None:original_cascade.pop('r85_turn_brake_reference')
    lower_ack_configuration(original_cascade.pop('r85_student_lower_loop_ack',None))
    expected_ack=(MEASUREMENT_LOWER_ACK_CONFIG if measurement_p_retention is not None else STUDENT_LOWER_ACK_CONFIG)
    selected_ack=p['cascade'].get('r85_student_lower_loop_ack')
    if isinstance(selected_ack,dict)and 'xy_stop_takeover_enabled'in selected_ack:
        flag=selected_ack['xy_stop_takeover_enabled']
        if type(flag)is not bool or measurement_p_retention is None:
            raise ValueError('XY stop mode requires a typed flag and the original selected yaw proof')
        expected_ack=dict(expected_ack,xy_stop_takeover_enabled=flag)
    selected_ack,_=split_ack(selected_ack)
    if selected_ack!=expected_ack:
        raise ValueError('Explicit same-frame Student lower-loop ACK semantics required')
    if speed_tier is not None:original_cascade['desired_speed']=.2
    if original_cascade!=baseline['cascade']:
        raise ValueError('Unchanged baseline navigation/safety cascade differs beyond explicit ACK semantics')
    for key in ('warmup_history','teacher_transition','sensor_sampling','control_clock_contract',
                'ordinary_terminal_parking_contract','navigation_settle_statistics_contract','known_scene_localization_contract',
                'pose_cloud_timeout_s','max_speed_mps','max_yaw_rate_radps','max_lateral_speed_mps','clear_guard_gap_max_sim_s'):
        expected=(speed_tier['cruise_mps'] if speed_tier is not None and key=='max_speed_mps' else baseline.get(key))
        if p.get(key)!=expected:raise ValueError('Unchanged baseline navigation/safety field differs: '+key)
    cascade=p['cascade']
    if cascade['command_limits']!=[.6,.2,.6] or cascade['command_slew_per_s']!=[.6,.6,.8] or cascade['desired_speed']!=(.2 if speed_tier is None else speed_tier['cruise_mps']):
        raise ValueError('Existing outer controller envelope changed')
    if p['spawn']!=[0.,0.,.3,0.] or p['expected_asset']['spawn']!=p['spawn']:
        raise ValueError('Original scene physical spawn required')
    if cascade['navigation_ground_truth_used']is not False or cascade['feedback_expected_hz']!=p['sensor_sampling']['camera_hz']:
        raise ValueError('Actual navigation feedback contract differs')
    return duration,p['overview_pose']

def prepare_task_inputs(run,p,here):
    run=Path(run);here=Path(here)
    original=Path(p['r85_task']['typed_route_source_file']).resolve()
    if sha(original)!=p['r85_task']['typed_route_source_sha256']:raise ValueError('PCT source changed before archive')
    source=json.loads(original.read_text())
    prior_path=Path(source['physical_sources']['known_scene_reference_file'])
    if sha(prior_path)!=source['physical_sources']['known_scene_reference_sha256']:
        raise ValueError('Static geometry source changed')
    prior=json.loads(prior_path.read_text());actual=json.loads((run/'known_scene_reference.json').read_text())
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
        with dst.open('xb')as f:f.write(src.read_bytes())
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
        profile=json.loads((run/'input_profile.json').read_text())
        result=json.loads(diagnostic_path.read_text())
        if (not enabled(profile) or result.get('run_id')!=run.name
                or result.get('formal_full_route_pass')is not False):
            raise ValueError('Foreign or non-diagnostic terminal result')
        return dict(stop=True,reason='actual_R85_diagnostic_terminated',diagnostic_result=result)
    p=run/'r85_task_result.json'
    if p.exists():
        d=json.loads(p.read_text())
        if d.get('run_id')!=run.name:raise ValueError('Foreign R85 task result')
        if d.get('state')=='failed':return dict(stop=True,reason='actual_R85_task_failed',task_result=d)
    for name,key in [('navigation_status.json','state'),('r85_task_status.json','state')]:
        p=run/name
        if p.exists():
            d=json.loads(p.read_text())
            if d.get(key)=='failed':return dict(stop=True,reason='actual_R85_'+name+'_failed',actual_status=d)
    return dict(stop=False,reason='bounded_task_active')
