"""Exact V34 continuous-route, known-scene assisted simulation profiles.
Ordinary passage is a NEW versioned outcome, not the old46 dwell pass.
"""
from pathlib import Path
import copy,json
PROFILE_NAMES=('engineering_ipc60.json','engineering_nav30.json','engineering_nav60.json','engineering_nav270.json','engineering_nav46.json')
PROFILE_NAME=PROFILE_NAMES[0]
DURATIONS=dict(zip(PROFILE_NAMES,(60.,30.,60.,270.,1500.)))
VERSION='engineering_v34_continuous_known_map'
CHANGE=('V34 actual simulation implementation: fixed phase route with ordered ordinary-region passage, '
        'original connector and terminal dwell/terrain acknowledgments; known-scene43-box map-assisted '
        'navigation after independent-frame/first-hit/timing/stage checks. No truth pose navigation; '
        'no independent sensor-map or old all-region dwell PASS claim. Native physics, Actor checkpoint, '
        '300ms fresh inputs,200ms IPC and original safety/collision gates retained.')
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
    result.update(controller_selector=VERSION,profile_version=VERSION,prospective_change=CHANGE,
        mission46_required_source_files=[str(p.resolve()) for p in required_source_files()])
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
        region_progress_deadline_s=90.,original_dwell_acceptance_claimed=False)
    result['known_scene_localization_contract']=dict(enabled=True,
        schema='known_scene_map_assisted_navigation/v1',reference_kind='provided_simulation_scene43box_geometry',
        independent_sensor_map=False,robot_truth_pose_used=False,IMU_world_orientation_used=False,
        maximum_source_age_s=.3,maximum_match_age_s=3.,matcher_interval_s=1.,
        translation_rate_mps=.04,rotation_rate_radps=.02,
        startup_correction_commit_requires_navigation_idle=True,
        navigation_world_transform_source='original_scenario.slam_origin_in_world',
        frontend_velocity_or_bias_repaired=False)
    result['diagnostic_scope'].update(route_controller_gates_unchanged=False,
        purpose='V34 actual continuous route and explicitly known-scene assisted navigation experiment')
    return result
def normal_form(profile,here):
    base=json.loads(baseline_profile_path(here,profile_name(profile)).read_text())
    result=copy.deepcopy(profile)
    for key in ('controller_selector','profile_version','prospective_change',
                'mission46_required_source_files','engineering_contract','diagnostic_scope',
                'global_route_reference_contract'):
        result[key]=copy.deepcopy(base[key])
    result.pop('continuous_route_contract',None);result.pop('known_scene_localization_contract',None)
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
