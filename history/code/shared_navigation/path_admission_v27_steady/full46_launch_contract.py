"""Exact V27 profiles: path admission repair only; original46 gates stay active."""
from pathlib import Path
import copy
import json

PROFILE_NAMES = ('repair_prefix270.json', 'repair_original46.json')
PROFILE_NAME = PROFILE_NAMES[0]
DURATIONS = {'repair_prefix270.json': 270., 'repair_original46.json': 1500.}
CHANGE = ('V27 independent steady-recording comparison; five production repair '
          'files remain byte-identical to frozen V27. Detail window[0,0] retains '
          'all-time state/IMU/stage/timing recording and omits dense H/R/plane/pixel '
          'details, whose absence is UNVERIFIED. This isolates recording load; '
          'the R1 IPC failure root cause is not established. Original V25 route, '
          'regions, gains, stops, TTL, Actor, observations, frontend and planner '
          'stay unchanged. Both270/1500s profiles retain48GiB cap/90GiB start '
          'reserve/30GiB runtime reserve and original1500s mission global cap. '
          'Cutoff is never success; no historical navigation result is adopted.')


def baseline_profile_path(here):
    return Path(here).resolve().parent/'corridor_tracking_v25_terrain_event/profiles/curvature_original46_on.json'


def normal_form(profile, here):
    """Restore only the declared identity/recording/source-list extension to V25."""
    baseline = json.loads(baseline_profile_path(here).read_text())
    result = copy.deepcopy(profile)
    for key in ('controller_selector', 'profile_version', 'prospective_change',
                'duration_s', 'raw_budget_bytes', 'minimum_available_storage_bytes',
                'raw_storage_budget', 'mission46_required_source_files'):
        result[key] = copy.deepcopy(baseline[key])
    for key in ('finite_horizon_s', 'detail_window_sim_s', 'purpose'):
        result['diagnostic_scope'][key] = copy.deepcopy(baseline['diagnostic_scope'][key])
    result.pop('path_admission_contract', None)
    return result


def expected_profile(here, name=PROFILE_NAME):
    if name not in PROFILE_NAMES:
        raise ValueError('V27 has only two exact source-bound profiles')
    from mission46_profile import required_source_files
    here = Path(here).resolve()
    duration = DURATIONS[name]
    result = copy.deepcopy(json.loads(baseline_profile_path(here).read_text()))
    result.update(controller_selector='path_admission_v27',
                  profile_version='path_admission_v27', prospective_change=CHANGE,
                  duration_s=duration, raw_budget_bytes=48*1024**3,
                  minimum_available_storage_bytes=90*1024**3)
    result['raw_storage_budget'] = {
        'max_run_bytes': 48*1024**3,
        'minimum_start_available_bytes': 90*1024**3,
        'minimum_runtime_available_bytes': 30*1024**3}
    result['mission46_required_source_files'] = [str(p.resolve()) for p in required_source_files()]
    result['diagnostic_scope'].update(finite_horizon_s=duration, detail_window_sim_s=[0, 0],
        purpose='V27 byte-identical path admission with steady state/IMU/stage/timing recording; detail window[0,0] omits dense H/R/plane/pixel evidence (UNVERIFIED), isolating recording load without an I/O root-cause claim')
    result['path_admission_contract'] = {
        'schema': 'path_admission_v27_launch/v1',
        'profile_name': name,
        'simulation_only': True,
        'runtime_cutoff_sim_elapsed_s': duration,
        'cutoff_is_success': False,
        'prefix_only': name == PROFILE_NAME,
        'full46_verified': False,
        'actual_navigation_verified': False,
        'historical_pass_inherited': False,
        'algorithm_change': 'SCAN path admission only',
        'backend_connected': False,
        'original_46_region_definitions_and_arrival_gates_unchanged': True,
        'original_mission_global_cap_s': 1500.}
    return result


def profile_name(profile):
    name = profile.get('path_admission_contract', {}).get('profile_name')
    if name not in PROFILE_NAMES:
        raise ValueError('Missing exact V27 profile identity')
    return name


def validate_full46(profile, here):
    """Both profiles retain full original46 definitions; neither asserts a pass."""
    from mission46_profile import validate_profile, required_source_files
    validate_profile(profile)
    name = profile_name(profile)
    if (profile != expected_profile(here, name)
            or normal_form(profile, here) != json.loads(baseline_profile_path(here).read_text())
            or 'original46_prefix_regions' in profile or 'bounded_prefix_contract' in profile
            or profile.get('mission46_required') is not True
            or profile.get('expected_region_count') != 46
            or profile.get('expected_stage_region_counts') != {'exploration': 18, 'return_origin': 14, 'navigation_f1_f3': 14}
            or profile.get('mission46_required_source_files') != [str(p.resolve()) for p in required_source_files()]):
        raise ValueError('V27 permits only exact270s repair prefix or1500s original46 repair; cutoff never equals navigation success')
    return profile
