"""Exact V26 diagnostic contract; all original 46-region gates remain active."""
from pathlib import Path
import copy
import json

PROFILE_NAME = 'height_fault270.json'
CHANGE = ('V26 independent height-failure recording only: 270s diagnostic cutoff, '
          '214-226s detailed solver window, 48GiB recording cap/90GiB startup '
          'reserve/30GiB runtime reserve and independent source identity. '
          'Original V25 64x480 LiDAR30/RGB30, 46 regions, scenario/global mission '
          'cap 1500s, geometry, gains, observations, arrival gates, 300ms freshness, '
          'parking and sole actuator protections are unchanged. A cutoff is never '
          'navigation success; no historical runtime pass is inherited.')


def baseline_profile_path(here):
    return Path(here).resolve().parent/'corridor_tracking_v25_terrain_event/profiles/curvature_original46_on.json'


def normal_form(profile, here):
    """Restore only the explicitly allowed identity/recording fields to V25."""
    baseline = json.loads(baseline_profile_path(here).read_text())
    result = copy.deepcopy(profile)
    for key in ('controller_selector', 'profile_version', 'prospective_change',
                'duration_s', 'raw_budget_bytes', 'minimum_available_storage_bytes',
                'raw_storage_budget'):
        result[key] = copy.deepcopy(baseline[key])
    result['mission46_required_source_files'] = [
        str(path).replace('/height_diagnostic_v26/', '/corridor_tracking_v25_terrain_event/')
        for path in result['mission46_required_source_files']]
    for key in ('finite_horizon_s', 'detail_window_sim_s', 'purpose'):
        result['diagnostic_scope'][key] = copy.deepcopy(baseline['diagnostic_scope'][key])
    result.pop('height_diagnostic_contract', None)
    return result


def expected_profile(here):
    here = Path(here).resolve()
    result = copy.deepcopy(json.loads(baseline_profile_path(here).read_text()))
    result.update(controller_selector='height_diagnostic_v26',
                  profile_version='height_diagnostic_v26', prospective_change=CHANGE,
                  duration_s=270., raw_budget_bytes=48*1024**3,
                  minimum_available_storage_bytes=90*1024**3)
    result['raw_storage_budget'] = {
        'max_run_bytes': 48*1024**3,
        'minimum_start_available_bytes': 90*1024**3,
        'minimum_runtime_available_bytes': 30*1024**3}
    result['mission46_required_source_files'] = [
        str(path).replace('/corridor_tracking_v25_terrain_event/', '/height_diagnostic_v26/')
        for path in result['mission46_required_source_files']]
    result['diagnostic_scope'].update(finite_horizon_s=270., detail_window_sim_s=[214, 226],
        purpose='Current V25 height-failure evidence: IMU/LIO/VIO state, covariance, residual/plane and real motion; recording-only diagnostic, no algorithm repair')
    result['height_diagnostic_contract'] = {
        'schema': 'height_diagnostic_v26_cutoff/v1',
        'simulation_only': True,
        'cutoff_sim_elapsed_s': 270.,
        'cutoff_is_success': False,
        'full46_verified': False,
        'actual_navigation_verified': False,
        'historical_pass_inherited': False,
        'original_46_region_definitions_and_arrival_gates_unchanged': True,
        'original_mission_global_cap_s': 1500.}
    return result


def validate_full46(profile, here):
    """Keep the shared call name, but authorize this fixed diagnostic only."""
    from mission46_profile import validate_profile, required_source_files
    validate_profile(profile)
    if (profile != expected_profile(here)
            or normal_form(profile, here) != json.loads(baseline_profile_path(here).read_text())
            or 'original46_prefix_regions' in profile or 'bounded_prefix_contract' in profile
            or profile.get('mission46_required') is not True
            or profile.get('expected_region_count') != 46
            or profile.get('expected_stage_region_counts') != {'exploration': 18, 'return_origin': 14, 'navigation_f1_f3': 14}
            or profile.get('mission46_required_source_files') != [str(p.resolve()) for p in required_source_files()]):
        raise ValueError('V26 permits only the exact recording-only 270s diagnostic with original46 definitions; never a prefix or navigation pass')
    return profile
