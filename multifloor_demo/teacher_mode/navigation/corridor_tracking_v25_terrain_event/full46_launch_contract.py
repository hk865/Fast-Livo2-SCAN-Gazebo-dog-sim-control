"""Exact full46 launch contract; a prefix pass never authorizes this attempt."""
from pathlib import Path
import copy
import json

PROFILE_NAME = 'curvature_original46_on.json'
CHANGE = 'V25 full original46 curvature ON: fix privileged Actor terrain-event duplicate-key serialization, retaining authoritative event sequence/local time/navigation false. Original map rays, source/causal transitions, control mathematics, geometry, deadlines and protections unchanged; no failed V24 navigation result inherited.'


def baseline_profile_path(here):
    return Path(here).resolve().parent/'corridor_tracking_v20/profiles/pipeline_staged_original46.json'


def expected_profile(here):
    here = Path(here).resolve()
    profile = copy.deepcopy(json.loads(baseline_profile_path(here).read_text()))
    profile['controller_selector'] = 'v25_terrain_event_full46'
    profile['profile_version'] = 'corridor_tracking_v25_terrain_event'
    profile['prospective_change'] = CHANGE
    profile['mission46_required_source_files'] = [
        str(path).replace('/corridor_tracking_v20/', '/corridor_tracking_v25_terrain_event/')
        for path in profile['mission46_required_source_files']]
    spatial = profile['cascade']['spatial_reference']
    spatial['curvature_feedforward_enabled'] = True
    spatial['curvature_speed_limit_enabled'] = True
    return profile


def validate_full46(profile, here):
    from mission46_profile import validate_profile, required_source_files
    validate_profile(profile)
    if (profile != expected_profile(here) or 'original46_prefix_regions' in profile
            or 'bounded_prefix_contract' in profile
            or profile.get('mission46_required') is not True
            or profile.get('expected_region_count') != 46
            or profile.get('expected_stage_region_counts') != {'exploration': 18, 'return_origin': 14, 'navigation_f1_f3': 14}
            or profile.get('registered_route_fence') != expected_profile(here)['registered_route_fence']
            or profile.get('mission46_required_source_files') != [str(p.resolve()) for p in required_source_files()]):
        raise ValueError('V25 requires the exact full original46 curvature ON contract, never a prefix authorization')
    return profile
