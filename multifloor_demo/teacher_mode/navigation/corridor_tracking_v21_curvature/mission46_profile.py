"""Pure, fail-closed profile adapter for the original 46-region mission.

This module starts no ROS, simulator, controller or policy process. It retains
the authoritative original scenario, rather than synthesizing a 46-point path
from the previously tested 32-region route. Runtime integration is mandatory.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
DEMO = HERE.parents[2]
if str(DEMO) not in sys.path:
    sys.path.insert(0, str(DEMO))
from mission.route_regions import ROUTES, validate_route_regions

FROZEN_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
ORIGINAL_SCENARIO_SHA = '6111db95f08adba2a16c086d74e85c5135823ac1e2b7380f8c882538fd441804'
ROUTE_COUNTS = {'exploration': 18, 'return_origin': 14, 'navigation_f1_f3': 14}
SPAWN = [0., 0., .3, 0.]
SIMULATION_MAX_S = 1500.
RAW_BUDGET_BYTES = 120 * 1024**3
MIN_FREE_BYTES = 150 * 1024**3


def canonical_sha(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def required_source_files():
    """Root must archive/hash these, in addition to its existing runtime set."""
    return [HERE / name for name in ('mission46.py', 'mission46_profile.py', 'mission46_contract.py',
        'mission46_runtime.py', 'mission46_runtime_evidence.py', 'mission46_obstacle_runtime.py')] + [
        DEMO / name for name in ('simulation/scenario.json', 'simulation/obstacle_trigger.py',
                                 'mission/state_machine.py', 'mission/route_regions.py',
                                 'navigation/goal_regions.py', 'slam/map_archive.py')]


def load_original_scenario():
    path = DEMO / 'simulation/scenario.json'
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != ORIGINAL_SCENARIO_SHA:
        raise ValueError('Original scenario bytes changed; require new source review')
    scenario = json.loads(raw)
    validate_original_scenario(scenario)
    return scenario


def validate_original_scenario(scenario):
    """Require original points, all region geometry and all stage definitions."""
    original_raw = (DEMO / 'simulation/scenario.json').read_bytes()
    if hashlib.sha256(original_raw).hexdigest() != ORIGINAL_SCENARIO_SHA:
        raise ValueError('Original scenario source identity changed')
    original = json.loads(original_raw)
    validate_route_regions(scenario)
    for name, count in ROUTE_COUNTS.items():
        if len(scenario.get(name, [])) != count or scenario[name] != original[name]:
            raise ValueError(name + ' is not the original route')
    for key in ('route_goals', 'goal_region_profile', 'spawn', 'frame_id',
                'waypoint_reference', 'waypoint_semantics', 'dynamic_obstacle',
                'floor_elevations'):
        if scenario.get(key) != original.get(key):
            raise ValueError('Original mission field differs: ' + key)
    if sum(map(len, scenario['route_goals'].values())) != 46:
        raise ValueError('Exactly 46 original regions required')
    if scenario['spawn'] != dict(x=0, y=0, z=.3, yaw=0):
        raise ValueError('Original origin must not be replaced by the tested ramp spawn')


def scenario_for_teacher():
    result = copy.deepcopy(load_original_scenario())
    result['mode'] = 'teacher_original_mission46'
    result['require_dynamic_obstacle'] = True
    result['stage_timeout_s'] = SIMULATION_MAX_S
    result['sensors']['lidar'].update(rate=30, samples=[480, 64])
    result['sensors']['camera']['rate'] = 30
    # Original IMU geometry/rate and every route/goal/dynamic field are retained.
    return result


def build_profile(base_profile=None):
    """Return a candidate profile; this does not authorize the old run entrypoint.

    The existing single-request route.py cannot consume this profile. Root must
    integrate TeacherMission46 and its explicit runtime actions before prepare.
    """
    if base_profile is None:
        base_profile = json.loads((HERE / 'profiles/pipeline_staged_600.json').read_text())
    result = copy.deepcopy(base_profile)
    scenario = scenario_for_teacher()
    result.update(experiment='teacher_original46_actual_SLAM_SCAN_corridor_v20',
                  selector='original_46_region_mission_v20', duration_s=SIMULATION_MAX_S,
                  spawn=SPAWN, navigation_ground_truth_used=False,
                  return_to_origin=True, mission46_required=True,
                  original_scenario_sha256=ORIGINAL_SCENARIO_SHA,
                  original_mission_definition_sha256=canonical_sha({
                      key: scenario[key] for key in (*ROUTES, 'route_goals',
                          'goal_region_profile', 'dynamic_obstacle', 'spawn')}),
                  checkpoint_sha256=FROZEN_SHA,
                  expected_region_count=46, expected_stage_region_counts=ROUTE_COUNTS.copy(),
                  max_simulation_duration_s=SIMULATION_MAX_S,
                  raw_budget_bytes=RAW_BUDGET_BYTES, minimum_available_storage_bytes=MIN_FREE_BYTES,
                  raw_budget_is_limit_not_permission_to_drop_evidence=True,
                  original_scenario=scenario,
                  requires_original_origin_initialization_receipt=True,
                  requires_current_run_rgb_save=True,
                  requires_original_dynamic_obstacle=True,
                  requires_bidirectional_layer_switches=True,
                  mission46_required_source_files=[str(p.resolve()) for p in required_source_files()],
                  navigation_is_verified=False, status='SOURCE_PREPARED_UNVERIFIED')
    # A single uniform disc contract cannot express the original ramps/sphere.
    for key in ('route_world_points', 'route_goal_ids', 'route_surface_ids',
                'arrival_radius_m', 'arrival_control_radius_m',
                'arrival_height_half_span_m', 'dwell_sim_s', 'terrain_layer_switch',
                'distance_m', 'fence', 'ramp_segments'):
        result.pop(key, None)
    result['expected_asset'] = dict(result.get('expected_asset', {}), spawn=SPAWN)
    result['excluded'] = ['real_robot', 'true_stairs', 'fully_sensor_replaced_actor']
    result['mission46_runtime_interfaces'] = [
        'multi_request_controller', 'whole_mission_worker', 'current_rgb_map_save',
        'original_dynamic_obstacle', 'bidirectional_terrain_provider']
    result['mission46_final_hold_sim_s'] = 5.
    result['mission46_source_timeout_s'] = .3
    result['diagnostic_scope'] = dict(result.get('diagnostic_scope', {}),
        finite_horizon_s=SIMULATION_MAX_S,
        arrival_gate_source='original46 three_platform_body_arrival_v2, not inherited32 uniform discs')
    result['policy_observations'] = (
        'Teacher CPU single-thread: 232 privileged simulator-state/height dimensions '
        'and 15 command/previous-action dimensions; actual SLAM/IMU/SCAN only for navigation')
    validate_profile(result)
    return result


def validate_profile(profile):
    validate_original_scenario(profile['original_scenario'])
    if (profile.get('checkpoint_sha256') != FROZEN_SHA or
            profile.get('navigation_ground_truth_used') is not False or
            profile.get('mission46_required') is not True or
            profile.get('expected_region_count') != 46 or
            profile.get('expected_stage_region_counts') != ROUTE_COUNTS or
            profile.get('spawn') != SPAWN):
        raise ValueError('Mission46 profile identity/source contract differs')
    duration = profile.get('duration_s')
    if (type(duration) not in (int, float) or not math.isfinite(duration)
            or duration != SIMULATION_MAX_S or profile.get('max_simulation_duration_s') != SIMULATION_MAX_S):
        raise ValueError('Original46 candidate must retain the 1500s global simulation cap')
    if (profile.get('raw_budget_bytes') != RAW_BUDGET_BYTES or
            profile.get('minimum_available_storage_bytes') != MIN_FREE_BYTES):
        raise ValueError('Original46 storage contract differs')
    if profile['original_scenario']['sensors']['lidar']['samples'] != [480, 64] or any(
            profile['original_scenario']['sensors'][key]['rate'] != 30 for key in ('lidar', 'camera')):
        raise ValueError('Mission46 requires the declared 64x480 LiDAR30/RGB30 experiment')
    if profile.get('status') != 'SOURCE_PREPARED_UNVERIFIED' or profile.get('navigation_is_verified') is not False:
        raise ValueError('Prepared profile cannot claim runtime validation')


if __name__ == '__main__':
    print(json.dumps(build_profile(), indent=2, ensure_ascii=False, allow_nan=False))
