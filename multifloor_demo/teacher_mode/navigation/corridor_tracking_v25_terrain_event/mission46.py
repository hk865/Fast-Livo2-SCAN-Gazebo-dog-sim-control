"""Transport-free Teacher adapter for the unmodified original mission phases.

Root integrates the actions into its sole Teacher controller/worker. This file
never starts the historical CHAMP/JTC stack and never reads simulator pose for
navigation. Receipts here are structurally checked after the runtime has
verified their archived sources (``binding_verified``); this pure module is
not an independent file/physical verifier and cannot award a formal PASS.
"""
from __future__ import annotations

import copy
import math
import re

from mission46_profile import (FROZEN_SHA, ORIGINAL_SCENARIO_SHA, ROUTE_COUNTS,
                              SIMULATION_MAX_S, SPAWN, canonical_sha,
                              scenario_for_teacher, validate_original_scenario)
from mission.state_machine import Mission, ObstacleEvidence, ACTIVE_STAGES
from navigation.goal_regions import contains_control, parse_goal
from simulation.obstacle_trigger import SceneTrigger

SHA = re.compile(r'^[0-9a-f]{64}$')
INTERFACES = {'multi_request_controller', 'whole_mission_worker',
              'current_rgb_map_save', 'original_dynamic_obstacle',
              'bidirectional_terrain_provider'}
INIT_CHECKS = {'spawn_matches_original', 'exclusive_teacher_actuator',
               'physical_standing', 'stationary_imu_initialization', 'actual_slam_ready'}
MAP_CHECKS = {'actual_saved_rgb_file', 'current_run_rgb_provenance',
              'fresh_sensor_evidence', 'not_capacity_truncated'}
DYNAMIC_CHECKS = {'actual_entity_motion', 'actual_slam_scene_trigger',
                  'fresh_scan_pre_roll', 'obstacle_stop', 'clearance', 'recovery'}
TERRAIN_CHECKS = {'actual_slam_arrival', 'shared_landing_187_rays_equal',
                  'causal_provider_application', 'privileged_actor_source_declared',
                  'frozen_sdf_geometry'}
PARK_CHECKS = {'first_five_seconds', 'actual_slam_hold_drift', 'native_motion_limits',
               'source_freshness', 'continuous_teacher_control', 'no_protection'}
TRANSITIONS = {
    'exploring': ('exploration:11', 'lower12', 'upper23'),
    'returning': ('return_origin:5', 'upper23', 'lower12'),
    'navigating': ('navigation_f1_f3:7', 'lower12', 'upper23'),
}


class MissionDeadlineExceeded(RuntimeError):
    """Mission has already entered failed; runtime must retain Teacher hold."""


class TeacherMission46:
    """Compose the original Mission; optional inputs never imply success.

    Times are explicitly separated: ``wall_s`` is monotonic wall time for the
    original mission startup/stage deadlines; ``sim_ns`` is /clock for source
    freshness, region dwell and the hard 1500s total simulation cap. Caller
    must preserve the independent controller/bridge 300ms watchdog.
    """

    def __init__(self, scenario=None):
        self.scenario = copy.deepcopy(scenario if scenario is not None else scenario_for_teacher())
        validate_original_scenario(self.scenario)
        self.original = Mission()
        self.trigger = SceneTrigger()
        self.obstacle_evidence = ObstacleEvidence()
        self.interfaces = {}
        self.initialization = None
        self.layer = 'lower12'
        self.terrain_pending = None
        self.terrain_receipts = []
        self.phase_receipts = []
        self.rgb_receipt = None
        self.dynamic_receipt = None
        self.parking_receipt = None
        self.pending_final = None
        self.last_navigation = None
        self.start_sim_ns = None
        self.last_sim_ns = None
        self.last_wall_s = None
        self.completed = False

    def bind_runtime_interfaces(self, implementations):
        if not isinstance(implementations, dict) or set(implementations) != INTERFACES:
            raise ValueError('All five explicit Teacher46 runtime interfaces are required')
        for name, record in implementations.items():
            if (not isinstance(record, dict) or record.get('supported') is not True
                    or not SHA.fullmatch(str(record.get('implementation_sha256', '')))
                    or not isinstance(record.get('module'), str) or not record['module']):
                raise ValueError('Missing implemented runtime interface: ' + name)
        self.interfaces = copy.deepcopy(implementations)

    def start(self, run_id, *, wall_s, sim_ns):
        if not isinstance(run_id, str) or not run_id:
            raise ValueError('A unique run ID is required')
        if self.original.stage in ACTIVE_STAGES:
            raise ValueError('Mission already active')
        # Reset evidence when restarting, preserving only the source declarations.
        interfaces = copy.deepcopy(self.interfaces)
        self.__init__(self.scenario)
        self.interfaces = interfaces
        self._time(wall_s, sim_ns)
        self.start_sim_ns = sim_ns
        self.original.start(run_id, self.scenario, now=wall_s)
        return [{'kind': 'hold_navigation', 'reason': 'original_origin_initialization_pending',
                 'continuous_teacher_required': True}]

    def _time(self, wall_s, sim_ns):
        if (type(wall_s) not in (int, float) or not math.isfinite(wall_s) or wall_s < 0
                or type(sim_ns) is not int or sim_ns < 0):
            raise ValueError('Finite monotonic wall time and integer ROS sim ns required')
        if self.last_wall_s is not None and wall_s < self.last_wall_s:
            raise ValueError('Mission wall clock moved backward')
        if self.last_sim_ns is not None and sim_ns < self.last_sim_ns:
            raise ValueError('Mission simulation clock moved backward')
        self.last_wall_s, self.last_sim_ns = wall_s, sim_ns
        if (self.start_sim_ns is not None and self.original.stage in ACTIVE_STAGES
                and sim_ns - self.start_sim_ns > round(SIMULATION_MAX_S * 1e9)):
            self._fail(wall_s, 'global 1500s simulation limit exceeded')
            raise MissionDeadlineExceeded('global 1500s simulation limit exceeded')

    def _fail(self, wall_s, reason):
        self.pending_final = None
        self.completed = False
        return self.original.fail(wall_s, 'Teacher46: ' + reason)

    def _bound(self, receipt, schema, required, request_id=None):
        if (not isinstance(receipt, dict) or receipt.get('schema') != schema
                or receipt.get('run_id') != self.original.run_id
                or receipt.get('binding_verified') is not True
                or not SHA.fullmatch(str(receipt.get('source_evidence_sha256', '')))
                or receipt.get('passed') is not True):
            raise ValueError('Missing or unbound ' + schema + ' receipt')
        if request_id is not None and receipt.get('request_id') != request_id:
            raise ValueError('Receipt belongs to another phase request')
        checks = receipt.get('checks')
        if (not isinstance(checks, dict) or not required.issubset(checks)
                or not all(isinstance(c, dict) and c.get('passed') is True
                           and c.get('status') == 'passed' for c in checks.values())):
            raise ValueError('Required receipt checks missing, failed or unverified')
        return receipt

    def accept_initialization(self, receipt):
        self._bound(receipt, 'teacher_original_origin_initialization/v1', INIT_CHECKS)
        if (self.original.stage != 'waiting_sensors' or receipt.get('spawn') != SPAWN
                or receipt.get('checkpoint_sha256') != FROZEN_SHA
                or receipt.get('original_scenario_sha256') != ORIGINAL_SCENARIO_SHA
                or receipt.get('navigation_ground_truth_used') is not False):
            raise ValueError('Original spawn/Teacher/source initialization identity differs')
        self.initialization = copy.deepcopy(receipt)

    def set_heading_alignment(self, alignment):
        if (not isinstance(alignment, dict) or alignment.get('ground_truth_used') is not False
                or alignment.get('source') != 'actual_slam_and_imu'
                or not SHA.fullmatch(str(alignment.get('source_evidence_sha256', '')))
                or not isinstance(alignment.get('yaw_camera_init_from_world'), (int, float))
                or not math.isfinite(alignment['yaw_camera_init_from_world'])):
            raise ValueError('Heading must be frozen from actual SLAM/IMU registration')
        if self.original.heading_alignment is not None and alignment != self.original.heading_alignment:
            raise ValueError('Frozen heading alignment cannot be changed between phases')
        self.original.heading_alignment = copy.deepcopy(alignment)

    def _pose(self, observation, wall_s, sim_ns):
        if (not isinstance(observation, dict) or observation.get('source') != 'actual_slam_body'
                or observation.get('frame_id') != self.scenario['frame_id']
                or observation.get('ground_truth_used') is not False
                or type(observation.get('stamp_ns')) is not int):
            raise ValueError('Navigation requires an actual SLAM body pose')
        received = observation.get('received_wall_s')
        if (type(received) not in (int, float) or not math.isfinite(received)
                or not 0 <= wall_s - received <= .3
                or not 0 <= sim_ns - observation['stamp_ns'] <= 300_000_000):
            raise ValueError('Actual SLAM pose exceeds unchanged 300ms source timeout')
        pose = observation.get('position')
        if (not isinstance(pose, (list, tuple)) or len(pose) != 3
                or not all(type(v) in (int, float) and math.isfinite(v) for v in pose)):
            raise ValueError('Invalid actual SLAM position')
        return list(pose)

    def _actions(self, actions):
        result = copy.deepcopy(actions)
        for action in result:
            action['mission_run_id'] = self.original.run_id
            action['mission_stage'] = self.original.stage
            if action['kind'] == 'route':
                action['goals_definition_sha256'] = self.original.current_goals_sha256
                action['navigation_ground_truth_used'] = False
                action['reset_cascade_and_phase_parking'] = True
                action['route_success_is_whole_mission_success'] = False
                action['frozen_origin'] = copy.deepcopy(self.original.origin)
                action['frozen_heading_alignment'] = copy.deepcopy(self.original.heading_alignment)
                action['required_layer'] = self.layer
            elif action['kind'] == 'save_map':
                action['request_id'] = self.original.current_request
                action['current_run_only'] = True
                action['required_receipt_schema'] = 'teacher_mission46_rgb_save/v1'
        return result

    def tick(self, *, wall_s, sim_ns, sensors_ok, observation, map_points=0):
        try:
            self._time(wall_s, sim_ns)
        except MissionDeadlineExceeded:
            return self._actions([{'kind': 'stop_navigation'}])
        if self.original.stage not in ACTIVE_STAGES:
            return []
        try:
            pose = self._pose(observation, wall_s, sim_ns)
        except ValueError:
            pose, sensors_ok = None, False
        ready = (sensors_ok is True and self.initialization is not None and set(self.interfaces) == INTERFACES)
        actions = self.original.tick(wall_s, ready, pose, map_points=map_points)
        if self.original.stage == 'failed':
            return self._actions(actions)
        if not ready or self.terrain_pending is not None:
            actions.append({'kind': 'hold_navigation', 'continuous_teacher_required': True,
                            'reason': 'missing_interface_initialization_or_fresh_source'
                            if not ready else 'terrain_switch_ack_pending'})
        return self._actions(actions)

    def _valid_prefix(self, status, count):
        # Reuse every original region gate on a prefix without changing receipts.
        candidate = copy.deepcopy(self.original)
        candidate.current_goals = candidate.current_goals[:count]
        if not candidate.current_goals:
            return False
        candidate.goal_region = candidate.current_goals[-1]
        receipts = status.get('region_arrivals', [])
        if len(receipts) != count:
            return False
        view = copy.deepcopy(status)
        view.update(goals_definitions=candidate.current_goals, total=count, waypoint_index=count)
        return candidate.region_completion_valid(view, receipts[-1].get('raw_position'))

    def terrain_intent(self, status, *, wall_s, sim_ns, observation):
        """Root must hold at the connector before releasing the next goal."""
        self._time(wall_s, sim_ns)
        pose = self._pose(observation, wall_s, sim_ns)
        if self.terrain_pending is not None:
            return [{'kind': 'hold_navigation', 'reason': 'terrain_switch_ack_pending',
                     'continuous_teacher_required': True}]
        transition = TRANSITIONS.get(self.original.stage)
        if not transition or status.get('request_id') != self.original.current_request:
            return []
        goal_id, from_layer, to_layer = transition
        phase = self.original.stage
        if any(r['phase'] == phase for r in self.terrain_receipts):
            return []
        index = next(i for i, g in enumerate(self.original.current_goals) if g['goal_id'] == goal_id)
        receipts = status.get('region_arrivals', [])
        if len(receipts) <= index:
            return []
        if (status.get('state') != 'running' or status.get('waypoint_index') != index + 1
                or status.get('total') != len(self.original.current_goals)
                or status.get('goals_definitions') != self.original.current_goals
                or self.layer != from_layer or not self._valid_prefix(status, index + 1)
                or not contains_control(parse_goal(self.original.current_goals[index]), pose)):
            return self._fail(wall_s, 'terrain connector receipt/order is invalid or controller ran ahead')
        self.terrain_pending = dict(phase=phase, run_id=self.original.run_id,
            request_id=self.original.current_request, goal_id=goal_id,
            goals_definition_sha256=self.original.current_goals_sha256,
            from_layer=from_layer, to_layer=to_layer,
            arrival_receipt=copy.deepcopy(receipts[index]))
        return [{'kind': 'hold_navigation', 'reason': 'terrain_switch_ack_pending',
                 'continuous_teacher_required': True},
                dict(kind='terrain_switch', **copy.deepcopy(self.terrain_pending))]

    def accept_terrain(self, receipt, *, wall_s, sim_ns):
        self._time(wall_s, sim_ns)
        pending = self.terrain_pending
        if pending is None:
            raise ValueError('No authorized terrain transition pending')
        self._bound(receipt, 'teacher_mission46_terrain_switch/v1', TERRAIN_CHECKS, pending['request_id'])
        for key in ('phase', 'goal_id', 'goals_definition_sha256', 'from_layer', 'to_layer', 'arrival_receipt'):
            if receipt.get(key) != pending[key]:
                raise ValueError('Terrain ack differs from actual SLAM connector intent: ' + key)
        applied = receipt.get('effective_sim_ns')
        if (type(applied) is not int or not pending['arrival_receipt']['stamp_ns'] <= applied <= sim_ns
                or receipt.get('navigation_ground_truth_used') is not False
                or receipt.get('ray_count') != 187
                or type(receipt.get('maximum_ray_difference_m')) not in (int, float)
                or not 0 <= receipt['maximum_ray_difference_m'] <= 1e-9):
            raise ValueError('Terrain ack lacks causal identical privileged ray evidence')
        self.layer = pending['to_layer']
        self.terrain_receipts.append(copy.deepcopy(receipt))
        self.terrain_pending = None
        return [{'kind': 'release_navigation', 'reason': 'verified_terrain_ack',
                 'request_id': self.original.current_request}]

    def navigation_result(self, status, *, wall_s, sim_ns, observation):
        self._time(wall_s, sim_ns)
        if self.original.stage not in {'exploring', 'returning', 'navigating'}:
            return []
        if status.get('request_id') != self.original.current_request:
            return []
        pose = self._pose(observation, wall_s, sim_ns)
        self.last_navigation = copy.deepcopy(status)
        if status.get('state') != 'succeeded':
            return self._actions(self.original.navigation_result(status, pose, wall_s))
        if (self.terrain_pending is not None or not any(
                r['phase'] == self.original.stage for r in self.terrain_receipts)):
            return self._fail(wall_s, 'phase succeeded without its required terrain switch ack')
        if not self.original.region_completion_valid(status, pose):
            return self._fail(wall_s, 'original complete region/dwell evidence failed')
        if self.original.stage == 'navigating':
            if self.dynamic_receipt is None or not self.obstacle_evidence.verified:
                return self._fail(wall_s, 'original dynamic obstacle stop/recovery evidence missing')
            if self.pending_final is None:
                self.pending_final = dict(status=copy.deepcopy(status), pose=pose,
                                          parking_start_sim_ns=sim_ns)
            return [{'kind': 'final_active_hold', 'duration_sim_s': 5.,
                     'request_id': self.original.current_request,
                     'start_stamp_ns': self.pending_final['parking_start_sim_ns'],
                     'continuous_teacher_required': True}]
        self.phase_receipts.append(dict(stage=self.original.stage, status=copy.deepcopy(status)))
        return self._actions(self.original.navigation_result(status, pose, wall_s))

    def map_saved(self, receipt, *, wall_s, sim_ns):
        self._time(wall_s, sim_ns)
        self._bound(receipt, 'teacher_mission46_rgb_save/v1', MAP_CHECKS, self.original.current_request)
        if self.original.stage != 'saving_map' or self.layer != 'lower12':
            raise ValueError('RGB saving is allowed only after complete original return to F1')
        ages = receipt.get('sensor_ages_s', {})
        if (receipt.get('point_count', 0) < 500 or receipt.get('unique_colors', 0) < 8
                or receipt.get('observed_rgb_samples', 0) < 500
                or receipt.get('capacity_rejections') != 0
                or receipt.get('ground_truth_used') is not False
                or receipt.get('reference_map_loaded') is not False
                or receipt.get('source_topic') != '/cloud_registered'
                or '/camera/image_color' not in str(receipt.get('color_source'))
                or not SHA.fullmatch(str(receipt.get('saved_file_sha256', '')))
                or not isinstance(receipt.get('saved_file'), str) or not receipt['saved_file']
                or not all(type(ages.get(k)) in (int, float) and math.isfinite(ages[k])
                           and 0 <= ages[k] < 2 for k in ('odom', 'lidar', 'imu', 'full_cloud', 'camera', 'colored_cloud'))):
            return self._fail(wall_s, 'actual current RGB save/provenance/freshness evidence failed')
        self.rgb_receipt = copy.deepcopy(receipt)
        return self._actions(self.original.map_saved(True, receipt['point_count'], wall_s))

    def dynamic_trigger(self, status, *, wall_s, sim_ns, observation):
        self._time(wall_s, sim_ns)
        pose = self._pose(observation, wall_s, sim_ns)
        state = self.original.snapshot()
        state['navigation'] = status
        self.trigger.observe(state)
        return dict(ready=self.trigger.ready, activate=self.trigger.nearby(pose[:2]),
                    point=copy.deepcopy(self.trigger.point), error=self.trigger.error,
                    run_id=self.original.run_id, request_id=self.original.current_request,
                    navigation_ground_truth_used=False)

    def observe_obstacle(self, *, held, actual_speed_mps, obstacle):
        if self.original.stage != 'navigating':
            raise ValueError('Obstacle evidence must come from this navigation phase')
        if type(held) is not bool or not math.isfinite(actual_speed_mps) or actual_speed_mps < 0:
            raise ValueError('Actual hold and speed observations required')
        if obstacle.get('run_id') != self.original.run_id or obstacle.get('request_id') != self.original.current_request:
            raise ValueError('Foreign obstacle observation')
        self.obstacle_evidence.observe(held, actual_speed_mps, obstacle)

    def accept_dynamic(self, receipt):
        self._bound(receipt, 'teacher_mission46_dynamic_obstacle/v1', DYNAMIC_CHECKS,
                    self.original.current_request)
        if self.original.stage != 'navigating' or not self.obstacle_evidence.verified:
            raise ValueError('Actual original hold/recovery observations are incomplete')
        if receipt.get('navigation_ground_truth_used') is not False:
            raise ValueError('Ground-truth navigation cannot verify dynamic integration')
        self.dynamic_receipt = copy.deepcopy(receipt)

    def parking_complete(self, receipt, *, wall_s, sim_ns, observation):
        self._time(wall_s, sim_ns)
        if self.pending_final is None:
            raise ValueError('Original final navigation has not completed')
        self._bound(receipt, 'teacher_mission46_final_parking/v1', PARK_CHECKS,
                    self.original.current_request)
        start = self.pending_final['parking_start_sim_ns']
        if (receipt.get('start_stamp_ns') != start or receipt.get('end_stamp_ns') != start + 5_000_000_000
                or sim_ns < receipt['end_stamp_ns'] or len(self.terrain_receipts) != 3
                or self.rgb_receipt is None or self.dynamic_receipt is None):
            return self._fail(wall_s, 'complete first-five-second parking/mission evidence missing')
        pose = self._pose(observation, wall_s, sim_ns)
        status = copy.deepcopy(self.pending_final['status'])
        status['dynamic_obstacle_verified'] = True
        actions = self.original.navigation_result(status, pose, wall_s)
        if self.original.stage != 'completed':
            return self._actions(actions)
        self.phase_receipts.append(dict(stage='navigating', status=status))
        self.parking_receipt = copy.deepcopy(receipt)
        self.pending_final = None
        self.completed = True
        return self._actions(actions)

    def snapshot(self):
        result = self.original.snapshot()
        result.update(schema='teacher_original_mission46_state/v1',
                      adapter_stage='final_active_hold' if self.pending_final else self.original.stage,
                      functional_sequence_completed=self.completed,
                      navigation_is_verified=False,
                      formal_acceptance='independent_actual_run_evaluation_required',
                      expected_region_count=46, completed_region_count=sum(
                          len(r['status']['region_arrivals']) for r in self.phase_receipts),
                      layer=self.layer, terrain_pending=copy.deepcopy(self.terrain_pending),
                      terrain_switch_count=len(self.terrain_receipts),
                      missing_runtime_interfaces=sorted(INTERFACES - set(self.interfaces)),
                      original_scenario_sha256=ORIGINAL_SCENARIO_SHA,
                      navigation_ground_truth_used=False,
                      actor_privileged_dimensions=232, source_contract_sha256=canonical_sha(self.scenario))
        return result

    def stop(self, *, wall_s, sim_ns):
        self._time(wall_s, sim_ns)
        self.original.stop(wall_s)
        self.pending_final = None
        self.completed = False
        return self._actions([{'kind': 'stop_navigation'}, {'kind': 'obstacle', 'enabled': False},
                              {'kind': 'hold_navigation', 'continuous_teacher_required': True,
                               'reason': 'mission_stopped'}])
