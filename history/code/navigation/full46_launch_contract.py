"""V33 exact profiles: original V32 mission and safety, declared engineering edits."""
from pathlib import Path
import copy
import json

PROFILE_NAMES=('engineering_ipc60.json','engineering_nav60.json','engineering_nav270.json','engineering_nav46.json')
PROFILE_NAME=PROFILE_NAMES[0]
DURATIONS=dict(zip(PROFILE_NAMES,(60.,60.,270.,1500.)))
SURFACE_PARAMETERS=dict(minimum_support_points=6,minimum_mid_max_ratio=.05,
    maximum_normalized_rss_per_dof=4.,normal_variance_floor_m2=1e-8,
    original_min_eigenvalue_threshold_m2=.005)
SURFACE_DIAGNOSTIC=dict(windows='0:15,209:215',bounds='14:18,-1:3,0:1.5',
    max_bytes=67108864,source_kinds=[104,105],raw_truth_navigation_used=False)
CHANGE=('V33 simulation engineering: independent IPC stage timing, bounded optional worker evidence; '
    'switchable current-segment SCAN guides and same-trajectory monotonic projection with '
    'original heading rejection; shadow submap interface has no control authority. '
    'Frozen V32 SLAM build reused read-only with surface validity0. Original46 goals, '
    'arrival/dwell,90s deadline,300ms freshness,200ms IPC and native physics unchanged. '
    'Continuous registered-cloud/raw-source archives deliberately omitted; their replay '
    'acceptance is UNVERIFIED, never zero or historical PASS. Whole session4GiB budget.')
DECLARED_FIELDS=('controller_selector','profile_version','prospective_change',
    'mission46_required_source_files','duration_s','diagnostic_scope','raw_budget_bytes',
    'minimum_available_storage_bytes','raw_storage_budget',
    'raw_budget_is_limit_not_permission_to_drop_evidence')

def baseline_profile_path(here,name=PROFILE_NAME):
    if name not in PROFILE_NAMES:raise ValueError('Unknown V33 profile')
    base='repair_original46.json' if name==PROFILE_NAMES[-1] else 'repair_prefix270.json'
    return Path(here).resolve().parent/'map_surface_v32/profiles'/base

def profile_name(profile):
    name=profile.get('engineering_contract',{}).get('profile_name')
    if name not in PROFILE_NAMES:raise ValueError('Missing exact V33 profile identity')
    return name

def normal_form(profile,here):
    baseline=json.loads(baseline_profile_path(here,profile_name(profile)).read_text())
    result=copy.deepcopy(profile)
    for key in DECLARED_FIELDS:result[key]=copy.deepcopy(baseline[key])
    for key in ('engineering_contract','global_route_reference_contract','engineering_recording'):
        result.pop(key,None)
    return result

def expected_profile(here,name=PROFILE_NAME):
    from mission46_profile import required_source_files
    from storage_guard import HARD_BYTES,HOME_RESERVE
    if name not in PROFILE_NAMES:raise ValueError('Unknown V33 profile')
    result=json.loads(baseline_profile_path(here,name).read_text())
    result.update(controller_selector='engineering_v33',profile_version='engineering_v33',
        prospective_change=CHANGE,mission46_required_source_files=[str(p.resolve()) for p in required_source_files()],
        duration_s=DURATIONS[name],raw_budget_bytes=HARD_BYTES,
        minimum_available_storage_bytes=HOME_RESERVE,
        raw_budget_is_limit_not_permission_to_drop_evidence=False,
        raw_storage_budget=dict(max_run_bytes=HARD_BYTES,minimum_start_available_bytes=HOME_RESERVE,
            minimum_runtime_available_bytes=HOME_RESERVE))
    result['diagnostic_scope'].update(finite_horizon_s=DURATIONS[name],detail_window_sim_s=[1600,1601],
        route_controller_gates_unchanged=True,
        purpose='V33 bounded timing/navigation experiment; omitted cloud/raw archives explicitly UNVERIFIED')
    result['engineering_contract']=dict(schema='go2_engineering_v33_launch/v1',profile_name=name,
        simulation_only=True,runtime_cutoff_sim_elapsed_s=DURATIONS[name],cutoff_is_success=False,
        original_mission_global_cap_s=1500.,surface_validity_flag=0,
        SLAM_build_reused_from='map_surface_v32/slam_ws',SLAM_CPP_changed=False,
        backend_control_adoption=False,shadow_localization_is_navigation_PASS=False,
        original_46_region_definitions_and_arrival_gates_unchanged=True,
        historical_pass_inherited=False,actual_navigation_verified=False)
    result['global_route_reference_contract']=dict(enabled=name!=PROFILE_NAME,guide_spacing_m=1.,
        guide_start_ahead_m=.35,direction_audit_arc_m=.8,max_guide_points=64)
    result['engineering_recording']=dict(registered_cloud_archive='disabled',
        raw_source_capture=False,overview_archive_stride=30,omitted_cloud_replay_acceptance='UNVERIFIED',
        dense_solver_payload_collected=False,dense_solver_payload_acceptance='UNVERIFIED',
        state_stage_timing_kept=True,dense_window_outside_original1500s_cap=True,
        optional_IPC_payload_max_bytes=36*1024**2,live_SLAM_inputs_unchanged=True,
        current_run_RGB_map_save_required=True,whole_session_budget_bytes=HARD_BYTES)
    return result

def validate_full46(profile,here):
    from mission46_profile import validate_profile
    validate_profile(profile);name=profile_name(profile)
    if profile!=expected_profile(here,name):raise ValueError('Only exact V33 engineering profiles allowed')
    if normal_form(profile,here)!=json.loads(baseline_profile_path(here,name).read_text()):
        raise ValueError('Undeclared mission/controller/safety change')
    if (profile.get('mission46_required') is not True or profile.get('pose_cloud_timeout_s')!=.3
        or profile['cascade']['feedback_ttl_sim_and_wall_s']!=.3):
        raise ValueError('Original mission freshness required')
    return profile
