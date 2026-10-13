"""Exact V32 map-quality profiles; V28 navigation math/safety remain frozen."""
from pathlib import Path
import copy
import json

PROFILE_NAMES = ('repair_prefix270.json', 'repair_original46.json')
PROFILE_NAME = PROFILE_NAMES[0]
DURATIONS = {'repair_prefix270.json': 270., 'repair_original46.json': 1500.}
SURFACE_PARAMETERS = dict(minimum_support_points=6, minimum_mid_max_ratio=.05,
    maximum_normalized_rss_per_dof=4., normal_variance_floor_m2=1e-8,
    original_min_eigenvalue_threshold_m2=.005)
SURFACE_DIAGNOSTIC = dict(windows='0:15,209:215', bounds='14:18,-1:3,0:1.5',
    max_bytes=67108864, source_kinds=[104,105], raw_truth_navigation_used=False)
CHANGE = ('V32 independently built SLAM surface-support validity 0/1 intervention. '
    'Original V28 controller, gains, SCAN, Teacher, 46-region geometry, arrivals, '
    'dwell,90s region deadlines,300ms freshness and200ms native IPC remain frozen. '
    'State/P/IMU/stage/timing remain enabled; detail[209,215], bounded early '
    'support evidence[0,15] and bounded async binary/worker/raw-cloud recording '
    'are explicit diagnostic/storage changes. Validity0/1 is frozen in the '
    'runtime plan before launch, never a postselected profile variant. No '
    'historical navigation/height acceptance is inherited; cutoff is not success.')


def baseline_profile_path(here, name=PROFILE_NAME):
    if name not in PROFILE_NAMES:
        raise ValueError('Unknown exact V28 baseline profile')
    return Path(here).resolve().parent/'path_admission_v28_capture/profiles'/name


def profile_name(profile):
    name = profile.get('map_surface_contract', {}).get('profile_name')
    if name not in PROFILE_NAMES:
        raise ValueError('Missing exact V32 profile identity')
    return name


def normal_form(profile, here):
    """Restore only declared V32 identity/evidence fields to the same V28 profile."""
    baseline = json.loads(baseline_profile_path(here, profile_name(profile)).read_text())
    result = copy.deepcopy(profile)
    for key in ('controller_selector', 'profile_version', 'prospective_change',
                'mission46_required_source_files'):
        result[key] = copy.deepcopy(baseline[key])
    for key in ('detail_window_sim_s', 'purpose'):
        result['diagnostic_scope'][key] = copy.deepcopy(baseline['diagnostic_scope'][key])
    result.pop('map_surface_contract', None)
    return result


def expected_profile(here, name=PROFILE_NAME):
    if name not in PROFILE_NAMES:
        raise ValueError('V32 has only two exact source-bound profiles')
    from mission46_profile import required_source_files
    here = Path(here).resolve()
    result = copy.deepcopy(json.loads(baseline_profile_path(here,name).read_text()))
    result.update(controller_selector='map_surface_v32',profile_version='map_surface_v32',
        prospective_change=CHANGE,
        mission46_required_source_files=[str(p.resolve()) for p in required_source_files()])
    result['diagnostic_scope'].update(detail_window_sim_s=[209,215],
        purpose='V32 support validity A/B with unchanged V28 navigation; dense[209,215], bounded early support[0,15]; finite source/math evidence never proves height or46 success')
    result['map_surface_contract'] = dict(schema='map_surface_v32_launch/v1',
        profile_name=name, simulation_only=True, runtime_cutoff_sim_elapsed_s=DURATIONS[name],
        cutoff_is_success=False,prefix_only=name==PROFILE_NAME,full46_verified=False,
        actual_navigation_verified=False,historical_pass_inherited=False,
        SLAM_workspace='map_surface_v32/slam_ws',independent_build_required=True,
        actual_loaded_binary_witness_required=True,
        algorithm_change='Original lambda_min threshold AND normalized surface support validity when runtime flag1; diagnostics/storage bounded in botharms',
        surface_parameters=copy.deepcopy(SURFACE_PARAMETERS),
        surface_validity_input='run.py --surface-validity {0,1}; frozen runtime_plan field, never profile variant',
        support_diagnostic=copy.deepcopy(SURFACE_DIAGNOSTIC),
        backend_connected=False,original_46_region_definitions_and_arrival_gates_unchanged=True,
        original_mission_global_cap_s=1500.)
    return result


def validate_full46(profile, here):
    from mission46_profile import validate_profile, required_source_files
    validate_profile(profile)
    name=profile_name(profile)
    if (profile != expected_profile(here,name)
        or normal_form(profile,here) != json.loads(baseline_profile_path(here,name).read_text())
        or 'original46_prefix_regions' in profile or 'bounded_prefix_contract' in profile
        or profile.get('mission46_required') is not True
        or profile.get('duration_s') != DURATIONS[name]
        or profile.get('expected_region_count') != 46
        or profile.get('expected_stage_region_counts') != {'exploration':18,'return_origin':14,'navigation_f1_f3':14}
        or profile.get('pose_cloud_timeout_s') != .3
        or profile['cascade']['feedback_ttl_sim_and_wall_s'] != .3
        or profile.get('mission46_required_source_files') != [str(p.resolve()) for p in required_source_files()]):
        raise ValueError('V32 permits only exact270s/1500s profiles normalized to unchanged V28 navigation; cutoff never proves success')
    return profile
