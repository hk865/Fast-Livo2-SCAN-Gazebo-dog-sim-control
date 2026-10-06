"""Serializable transport integration contract; no runtime feature is implied.

Root must implement and freeze each interface, and verify every receipt's
source files before passing binding_verified=True. These declarations and the
finite pure tests are not permission to bypass runtime/safety/source gates.
"""
from __future__ import annotations

from mission46 import (INTERFACES, INIT_CHECKS, MAP_CHECKS, TERRAIN_CHECKS,
                       DYNAMIC_CHECKS, PARK_CHECKS)
from mission46_profile import (FROZEN_SHA, ORIGINAL_SCENARIO_SHA, SIMULATION_MAX_S,
                              RAW_BUDGET_BYTES, MIN_FREE_BYTES)


def contract():
    return {
        'schema': 'teacher_original_mission46_runtime_contract/v1',
        'status': 'SOURCE_PREPARED_UNVERIFIED',
        'checkpoint_sha256': FROZEN_SHA,
        'original_scenario_sha256': ORIGINAL_SCENARIO_SHA,
        'simulation_only': True,
        'max_simulation_s': SIMULATION_MAX_S,
        'raw_budget_bytes': RAW_BUDGET_BYTES,
        'minimum_available_storage_bytes': MIN_FREE_BYTES,
        'required_interfaces': sorted(INTERFACES),
        'interface_declaration': {
            'module': 'absolute source path archived in this run scope',
            'implementation_sha256': 'exact frozen implementation file SHA256',
            'supported': 'true only after implementation plus tests; declarations are not attestations'},
        'time_contract': {
            'wall_s': 'monotonic wall time, original startup/stage/save timeouts',
            'sim_ns': 'integer actual /clock ns, dwell/source ages/global cap',
            'source_timeout_s': .3,
            'controller_and_bridge_watchdogs': 'unchanged 300ms, independently enforced',
            'duplicate_clock': 'original clock-hold protection; never refresh pose timestamps'},
        'pose_observation': {
            'source': 'actual_slam_body', 'frame_id': 'camera_init',
            'ground_truth_used': False, 'position': 'three finite actual SLAM body coordinates',
            'stamp_ns': 'original source header, not receive-time replacement',
            'received_wall_s': 'actual receive monotonic clock'},
        'heading_alignment': {
            'source': 'actual_slam_and_imu', 'ground_truth_used': False,
            'yaw_camera_init_from_world': 'one fixed measured scene-axis registration',
            'source_evidence_sha256': 'archived real registration evidence digest',
            'must_remain_identical': True},
        'common_receipt_fields': {
            'run_id': 'this actual mission run',
            'request_id': 'this stage request, except initialization',
            'binding_verified': 'runtime source-file verifier result, never guessed',
            'source_evidence_sha256': 'real archived evidence receipt SHA256',
            'passed': 'all checks passed',
            'checks': 'all required names present; each has status=passed and passed=true'},
        'required_receipt_checks': {
            'teacher_original_origin_initialization/v1': sorted(INIT_CHECKS),
            'teacher_mission46_rgb_save/v1': sorted(MAP_CHECKS),
            'teacher_mission46_terrain_switch/v1': sorted(TERRAIN_CHECKS),
            'teacher_mission46_dynamic_obstacle/v1': sorted(DYNAMIC_CHECKS),
            'teacher_mission46_final_parking/v1': sorted(PARK_CHECKS)},
        'phase_order': ['waiting_sensors', 'exploring:18', 'returning:14', 'saving_map',
                        'navigating:14', 'final_active_hold:5sim_s', 'completed'],
        'request_format': '<run_id>:<original_route_name>:<monotonic_route_counter>',
        'request_transport': {
            'topic': 'root existing actual SCAN navigation-request transport',
            'payload': 'original schema2 goals, goal IDs, rotated geometry, .4s dwell and full definitions SHA',
            'reset_on_new_request': ['cascade_key', 'PI state', 'derivative state',
                                     'slew history per original controller reset contract',
                                     'final_bundle', 'phase parking scope'],
            'do_not_reset': ['frozen initial SLAM origin', 'frozen heading alignment',
                             'whole mission worker lifetime', 'source/watchdog protections'],
            'after_intermediate_success': 'measured parking/transition hold until next request accepted',
            'at_layer_connector': 'hold before next goal; wait for actual terrain-provider ack'},
        'terrain_maps': {
            'lower12': ['floor_1', 'ramp_12', 'floor_2'],
            'upper23': ['floor_2', 'ramp_23', 'floor_3'],
            'navigation_selector': 'actual SLAM arrival geometry and original goal ID only',
            'actor_only_privileged_verifier': 'native pose checks same187rays at sharedfloor2 landing',
            'maximum_ray_difference_m': 1e-9,
            'required_sequence': [
                ['exploration:11', 'lower12', 'upper23'],
                ['return_origin:5', 'upper23', 'lower12'],
                ['navigation_f1_f3:7', 'lower12', 'upper23']],
            'native_safety_map': 'retain original all-static map independently'},
        'rgb_save': {
            'when': 'after all14 return_origin regions including original origin sphere',
            'source_topic': '/cloud_registered', 'color_source_contains': '/camera/image_color',
            'minimum_points': 500, 'minimum_unique_rgb_colors': 8,
            'minimum_observed_rgb_samples': 500, 'capacity_rejections': 0,
            'ground_truth_used': False, 'reference_map_loaded': False,
            'all_sensor_ages_strictly_less_s': 2.,
            'fresh_sources': ['odom', 'lidar', 'imu', 'full_cloud', 'camera', 'colored_cloud'],
            'saved_file_and_sha': 'must be checked against actual saved RGB PCD, not metadata claim'},
        'dynamic_obstacle': {
            'entity': 'moving_obstacle', 'world_service': 'root actual Teacher Gazebo world, not old demo service',
            'original_scene_trigger': '[1,2,0] transformed by frozen actualSLAM/IMU registration',
            'stage': 'navigating', 'activation_navigation_waypoint_index': 1,
            'matching_request_state': 'running', 'planar_activation_radius_m': 1.4,
            'original_motion': 'y4→y2 at .25m/s; block20sim_s; y2→y0 at .25m/s',
            'proof': 'actual SetEntityPose acks, fresh actual SCAN, hold, clearance and actual recovery',
            'functional_obstacle_evidence_alone_is_formal_pass': False},
        'final_hold': {
            'duration_sim_s': 5., 'start': 'first final-route succeeded observation sim stamp',
            'policy': 'continuous Teacher and existing active hold; never action=0',
            'receipt': 'exact first5s interval with actualSLAM drift/native motion/source checks',
            'formal_gate': 'retain existing .05m XY/.1rad yaw and native speed limits'},
        'root_required_changes': {
            'controller': 'multiple stage request reset, connector hold/ack gate and explicit final hold',
            'worker': 'end only on whole mission completion/failure/global cap, not intermediate route success',
            'terrain_provider': 'initial lower12 plus three bidirectional causal switches, exact archived goal geometry',
            'request_or_new_mission_node': 'drive pure module with actual stamped sensor/SCAN status; publish and archive actions',
            'RGB_saver': 'actual current-run map save and fresh evidence receipt',
            'obstacle_node': 'original pure SceneTrigger and original motion in owned Teacher world',
            'scope_and_evaluator': 'archive all source files; original46 geometry/dwell/stages/map/dynamic and bothramps2up1down'},
        'acceptance_boundary': (
            'functional_sequence_completed is coordinator state only; formal interface/motion/Sim2Sim/navigation '
            'requires independent new-run evidence. Historic32 passes and finite pure tests cannot substitute.')}


if __name__ == '__main__':
    import json
    print(json.dumps(contract(), indent=2, ensure_ascii=False, allow_nan=False))
