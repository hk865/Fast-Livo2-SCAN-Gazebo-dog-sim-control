"""Exact V34 continuous-route, known-scene assisted simulation profiles.
Ordinary passage is a NEW versioned outcome, not the old46 dwell pass.
"""
from pathlib import Path
import copy,json
PROFILE_NAMES=('engineering_ipc60.json','engineering_nav30.json','engineering_nav60.json','engineering_nav270.json','engineering_nav46.json')
PROFILE_NAME=PROFILE_NAMES[0]
DURATIONS=dict(zip(PROFILE_NAMES,(60.,30.,60.,270.,1500.)))
SURFACE_PARAMETERS=dict(minimum_support_points=6,minimum_mid_max_ratio=.05,
    maximum_normalized_rss_per_dof=4.,normal_variance_floor_m2=1e-8,
    original_min_eigenvalue_threshold_m2=.005)
SURFACE_DIAGNOSTIC=dict(windows='0:15,209:215',bounds='14:18,-1:3,0:1.5',
    max_bytes=67108864,source_kinds=[104,105],raw_truth_navigation_used=False)
VERSION='engineering_v34_continuous_known_map'
CHANGE=('V34 actual simulation implementation: fixed phase route with ordered ordinary-region passage, '
        'original connector and terminal dwell/terrain acknowledgments; known-scene43-box map-assisted '
        'navigation after independent-frame/first-hit/timing/stage checks. No truth pose navigation; '
        'no independent sensor-map or old all-region dwell PASS claim. Native physics, Actor checkpoint, '
        '300ms fresh inputs,200ms IPC and original safety/collision gates retained. '
         'R20 selects the existing serial/0 ingress mode to commit each admitted callback before mapping; '
         '64MiB/512 bounds and original message contents/headers are unchanged. Upstream ROS stall is unresolved.')
def baseline_profile_path(here,name=PROFILE_NAME):
    if name not in PROFILE_NAMES:raise ValueError('Unknown V34 profile')
    return Path(here).resolve().parent/'engineering_v33/profiles'/name
def profile_name(profile):
    name=profile.get('engineering_contract',{}).get('profile_name')
    if name not in PROFILE_NAMES:raise ValueError('Missing exact V34 profile identity')
    return name
def expected_profile(here,name=PROFILE_NAME):
    from mission46_profile import required_source_files
    result=json.loads(baseline_profile_path(here,name).read_text())
    # Exact user-authorized shared cumulative budget; all behavioral fields preserved.
    result['raw_budget_bytes']=80_000_000_000
    result['raw_storage_budget']['max_run_bytes']=80_000_000_000
    result['pipeline']['mode']='serial'
    result.update(controller_selector=VERSION,profile_version=VERSION,prospective_change=CHANGE,
        # Preserve the original profile bytes, including its historical source
        # path metadata. The new candidate's actual sources are independently
        # archived/hashed by pid_scope.runtime_files and explicit runtime plan.
        mission46_required_source_files=[str((Path(here).resolve().with_name('engineering_v34_continuous_known_map')/p.relative_to(Path(here).resolve())).resolve()) if p.is_relative_to(Path(here).resolve()) else str(p.resolve()) for p in required_source_files()],
        route_prior_source='Given original scene43box geometry; axes of actual matched canonical map/world→navigation frame, measured initial SLAM origin; no IMU world orientation')
    result['engineering_contract'].update(schema='go2_engineering_v34_launch/v1',
        backend_control_adoption=True,backend_control_scope='navigation frame only; frontend state never reset',
        original_46_region_definitions_and_arrival_gates_unchanged=False,
        original_46_region_definitions_unchanged=True,ordinary_arrival_semantics='ordered_route_pass/v34',
        connector_and_terminal_dwell_gates_unchanged=True,
        independent_sensor_submaps_implemented=False,reference_kind='provided_simulation_scene43box_geometry')
    result['global_route_reference_contract']['enabled']=True
    result['continuous_route_contract']=dict(enabled=True,version='continuous_route/v34',
        ordered_original_regions=46,ordinary_semantics='pass_original_control_region',
        hard_stops=['exploration:11','exploration:17','return_origin:5','return_origin:13',
                    'navigation_f1_f3:7','navigation_f1_f3:13'],
        region_progress_deadline_s=90.,original_dwell_acceptance_claimed=False,
        local_planning_horizon_m=.65,local_horizon_reason='native Euclidean horizon must not span initial1m loop')
    result['known_scene_localization_contract']=dict(enabled=True,
        schema='known_scene_map_assisted_navigation/v1',reference_kind='provided_simulation_scene43box_geometry',
        independent_sensor_map=False,robot_truth_pose_used=False,IMU_world_orientation_used=False,
        maximum_source_age_s=.3,maximum_match_age_s=3.,matcher_interval_s=1.,
        translation_rate_mps=.04,rotation_rate_radps=.02,
        startup_correction_commit_requires_navigation_idle=True,
        route_axis_registration=dict(version='known_scene_canonical_axes_v34r17',
            source='actual_known_scene_canonical_frame',yaw_navigation_from_world_rad=0.,
            frozen_initial_commit_and_two_actual_source_matches_required=True,
            IMU_world_orientation_used=False,raw_IMU_use='rates/acceleration for stationary initialization only'),
        quality_diagnostic_capture=dict(version='known_scene_complete_CDR_diagnostic_v34r16',maximum_groups=2,
            maximum_logical_and_allocated_bytes=8*1024*1024,maximum_whole_raw_adapted_CDR_bytes=1024*1024,
            maximum_whole_registered_CDR_bytes=1536*1024,final_receipt_requires_worker_drained=True,
            trigger='top_candidate_only_ray_p95_rejected',quality_gates_changed=False,formal_R6_capture_complete=False),
        navigation_world_transform_source='original_scenario.slam_origin_in_world',
        frontend_velocity_or_bias_repaired=False,
        correction_trigger=dict(version='known_scene_body_effect_v34r15',translation_m=.04,rotation_rad=.025,
            translation_reference='same_source_corrected_body_position',
            exact_worker_captured_body_required=True,origin_translation_is_diagnostic_only=True),
        correction_release=dict(version='known_scene_frozen_body_convergence_v34r17',
            frozen_same_source_body_residual_m=.005,original_origin_residual_m=.005,
            original_rotation_residual_rad=.003,fresh_measured_zero_stop_required=True),
        bounded_foreground_surface_policy=dict(version='known_scene_unmapped_foreground_v34r14',
            maximum_fraction=.05,negative_static_first_hit_threshold_m=.12,
            minimum_static_surface_distance_m=.1,
            original_all_point_coverage_and_ray_gates_retained=True,
            original_full_capped_RMS_ranking_retained=True))
    result['diagnostic_scope'].update(route_controller_gates_unchanged=False,
        purpose='V34 actual continuous route and explicitly known-scene assisted navigation experiment')
    return result
def normal_form(profile,here):
    base=json.loads(baseline_profile_path(here,profile_name(profile)).read_text())
    result=copy.deepcopy(profile)
    for key in ('controller_selector','profile_version','prospective_change',
                'mission46_required_source_files','engineering_contract','diagnostic_scope',
                'global_route_reference_contract','route_prior_source','pipeline'):
        result[key]=copy.deepcopy(base[key])
    result.pop('continuous_route_contract',None);result.pop('known_scene_localization_contract',None)
    result['raw_budget_bytes']=base['raw_budget_bytes']
    result['raw_storage_budget']['max_run_bytes']=base['raw_storage_budget']['max_run_bytes']
    return result
def validate_full46(profile,here):
    from mission46_profile import validate_profile
    validate_profile(profile);name=profile_name(profile)
    if profile!=expected_profile(here,name):raise ValueError('Only exact reviewed V34 profiles allowed')
    if normal_form(profile,here)!=json.loads(baseline_profile_path(here,name).read_text()):
        raise ValueError('Undeclared inherited safety/profile change')
    if profile['pose_cloud_timeout_s']!=.3 or profile['cascade']['feedback_ttl_sim_and_wall_s']!=.3:
        raise ValueError('Original freshness changed')
    return profile
