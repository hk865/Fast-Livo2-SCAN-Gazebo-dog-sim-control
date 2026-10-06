"""The first nine original regions are a partial test, never original46 PASS."""
from mission46_profile import validate_original_scenario


def validate_prefix(profile):
    if profile.get('original46_prefix_regions') is None:
        return
    validate_original_scenario(profile['original_scenario'])
    original = profile['original_scenario']['route_goals']['exploration'][:9]
    expected = [dict(goal_id=g['goal_id'], xyz=g['center']) for g in original]
    if (type(profile.get('original46_prefix_regions')) is not int or profile['original46_prefix_regions'] != 9
            or profile.get('route_world_points') != expected or profile.get('spawn') != [0., 0., .3, 0.]
            or profile.get('mission46_required') is not False or profile.get('return_to_origin') is not False
            or profile.get('expected_region_count') != 9 or profile.get('duration_s') != 600.
            or profile.get('navigation_ground_truth_used') is not False or profile.get('pose_cloud_timeout_s') != .3):
        raise ValueError('The V20 prefix must preserve the original first-nine regions, spawn and source protection')
    contract = profile.get('bounded_prefix_contract', {})
    if (contract.get('schema') != 'original46_exploration_prefix9/v1'
            or contract.get('source_region_indices') != list(range(9))
            or contract.get('expected_active_hold_first_window_s') != 5.
            or contract.get('clock_cutoff_is_success') is not False or contract.get('full46_verified') is not False
            or contract.get('region_geometry_or_timeout_changed') is not False):
        raise ValueError('Prefix completion must require actual arrivals and first5s hold, never cutoff or full46 PASS')
