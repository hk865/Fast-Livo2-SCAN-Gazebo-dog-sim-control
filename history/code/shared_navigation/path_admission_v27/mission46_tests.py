"""Finite pure Mission46 tests; synthetic receipts are not runtime evidence."""
from __future__ import annotations

import copy
import math
import unittest

from mission46 import (TeacherMission46, MissionDeadlineExceeded, INTERFACES, INIT_CHECKS, MAP_CHECKS,
                       TERRAIN_CHECKS, DYNAMIC_CHECKS, PARK_CHECKS, TRANSITIONS)
from mission46_profile import (build_profile, scenario_for_teacher, validate_profile,
                              FROZEN_SHA, ORIGINAL_SCENARIO_SHA, SPAWN,
                              SIMULATION_MAX_S, validate_original_scenario)
from navigation.goal_regions import parse_goal, contains_control

TEST_SHA = '1' * 64


def receipt(m, schema, checks, **fields):
    return dict(schema=schema, run_id=m.original.run_id,
                request_id=m.original.current_request, binding_verified=True,
                passed=True, source_evidence_sha256=TEST_SHA,
                checks={k: dict(passed=True, status='passed') for k in checks}, **fields)


def pose(position=(0., 0., 0.), wall=0., sim=0, **fields):
    result = dict(source='actual_slam_body', frame_id='camera_init',
                  ground_truth_used=False, position=list(position),
                  stamp_ns=sim, received_wall_s=wall)
    result.update(fields)
    return result


def interfaces():
    return {k: dict(supported=True, module='synthetic_fixture_only',
                    implementation_sha256=TEST_SHA) for k in INTERFACES}


def initialization(m):
    return receipt(m, 'teacher_original_origin_initialization/v1', INIT_CHECKS,
                   spawn=SPAWN, checkpoint_sha256=FROZEN_SHA,
                   original_scenario_sha256=ORIGINAL_SCENARIO_SHA,
                   navigation_ground_truth_used=False)


def started(yaw=0., origin=(0., 0., 0.)):
    m = TeacherMission46()
    m.bind_runtime_interfaces(interfaces())
    m.start('synthetic_test', wall_s=0., sim_ns=0)
    m.accept_initialization(initialization(m))
    m.set_heading_alignment(dict(ground_truth_used=False, source='actual_slam_and_imu',
        source_evidence_sha256=TEST_SHA, yaw_camera_init_from_world=yaw))
    m.tick(wall_s=0., sim_ns=0, sensors_ok=True, observation=pose(origin), map_points=100)
    m.tick(wall_s=3., sim_ns=3_000_000_000, sensors_ok=True,
           observation=pose(origin, 3., 3_000_000_000), map_points=100)
    return m


def status(m, count=None, state='succeeded', start=10_000_000_000):
    goals = m.original.current_goals
    count = len(goals) if count is None else count
    arrivals = []
    for i, raw in enumerate(goals[:count]):
        g = parse_goal(raw)
        begin = start + i * 500_000_000
        dwell = round(g.dwell_sim_s * 1e9)
        item = dict(goal_id=g.goal_id, request_id=m.original.current_request,
                    goals_definition_sha256=m.original.current_goals_sha256,
                    stamp_ns=begin+dwell, start_stamp_ns=begin, dwell_ns=dwell,
                    region_inside=True, protected=False, arrival_definition=g.definition()['arrival'],
                    reason='arrived', max_observation_gap_ns=200_000_000,
                    raw_position=list(g.center), control_region_inside=True,
                    control_arrival_definition=g.control_arrival_definition())
        arrivals.append(item)
    return dict(request_id=m.original.current_request, state=state,
                goals_definition_sha256=m.original.current_goals_sha256,
                goals_definitions=copy.deepcopy(goals), waypoint_index=count,
                total=len(goals), region_arrivals=arrivals)


def terrain_ack(m):
    pending = copy.deepcopy(m.terrain_pending)
    pending.pop('run_id')
    pending.pop('request_id')
    return receipt(m, 'teacher_mission46_terrain_switch/v1', TERRAIN_CHECKS,
                   **pending, effective_sim_ns=m.last_sim_ns,
                   navigation_ground_truth_used=False, ray_count=187,
                   maximum_ray_difference_m=0.)


def cross_terrain(m, wall, sim):
    goal_id = TRANSITIONS[m.original.stage][0]
    index = next(i for i, g in enumerate(m.original.current_goals) if g['goal_id'] == goal_id)
    st = status(m, count=index+1, state='running', start=sim-(index+1)*500_000_000)
    observation = pose(m.original.current_goals[index]['center'], wall, sim)
    actions = m.terrain_intent(st, wall_s=wall, sim_ns=sim, observation=observation)
    assert actions[-1]['kind'] == 'terrain_switch'
    m.accept_terrain(terrain_ack(m), wall_s=wall, sim_ns=sim)


def complete_phase(m, wall, sim):
    st = status(m, start=sim-len(m.original.current_goals)*500_000_000)
    return m.navigation_result(st, wall_s=wall, sim_ns=sim,
        observation=pose(m.original.current_goals[-1]['center'], wall, sim))


def rgb(m):
    return receipt(m, 'teacher_mission46_rgb_save/v1', MAP_CHECKS,
        point_count=600, unique_colors=8, observed_rgb_samples=600,
        capacity_rejections=0, ground_truth_used=False, reference_map_loaded=False,
        source_topic='/cloud_registered', color_source='/camera/image_color actual samples',
        saved_file='colored_map.pcd', saved_file_sha256=TEST_SHA,
        sensor_ages_s={k: .1 for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud')})


def navigation_phase():
    m = started()
    cross_terrain(m, 20., 20_000_000_000)
    complete_phase(m, 25., 25_000_000_000)
    cross_terrain(m, 40., 40_000_000_000)
    complete_phase(m, 45., 45_000_000_000)
    m.map_saved(rgb(m), wall_s=46., sim_ns=46_000_000_000)
    return m


class ProfileTests(unittest.TestCase):
    def test_original_46_profile(self):
        p = build_profile()
        self.assertEqual(p['expected_stage_region_counts'], {'exploration':18,'return_origin':14,'navigation_f1_f3':14})
        self.assertEqual(p['duration_s'], 1500.)
        self.assertNotIn('route_world_points', p)
        self.assertNotIn('arrival_radius_m', p)
        self.assertFalse(p['navigation_is_verified'])

    def test_every_original_region_retained(self):
        s = scenario_for_teacher()
        self.assertEqual(sum(len(r) for r in s['route_goals'].values()), 46)
        for name, goals in s['route_goals'].items():
            for i, goal in enumerate(goals):
                self.assertEqual(goal['goal_id'], f'{name}:{i}')
                self.assertEqual(goal['arrival']['dwell_sim_s'], .4)
                self.assertEqual(goal['timeout_sim_s'], 90.)
        self.assertEqual(s['route_goals']['return_origin'][-1]['arrival']['type'], 'sphere')

    def test_32_replacement_rejected(self):
        s = scenario_for_teacher()
        s['exploration'] = s['exploration'][:4]
        with self.assertRaises(ValueError): validate_original_scenario(s)

    def test_unregistered_geometry_rejected(self):
        s = scenario_for_teacher()
        s['route_goals']['exploration'][0]['arrival']['radius_m'] = .4
        with self.assertRaises(ValueError): validate_original_scenario(s)

    def test_original_spawn_required(self):
        s = scenario_for_teacher()
        s['spawn']['x'] = 1.2
        with self.assertRaises(ValueError): TeacherMission46(s)

    def test_budget_and_duration_not_relaxed(self):
        for key, value in [('duration_s', 1501), ('minimum_available_storage_bytes', 0), ('raw_budget_bytes', 1)]:
            p = build_profile();p[key] = value
            with self.assertRaises(ValueError): validate_profile(p)


class SourceInitializationTests(unittest.TestCase):
    def test_missing_interfaces_blocks_route(self):
        m=TeacherMission46();m.start('test', wall_s=0., sim_ns=0)
        m.accept_initialization(initialization(m))
        m.set_heading_alignment(dict(ground_truth_used=False, source='actual_slam_and_imu',
            source_evidence_sha256=TEST_SHA, yaw_camera_init_from_world=0.))
        a=m.tick(wall_s=4.,sim_ns=4_000_000_000,sensors_ok=True,
                 observation=pose(wall=4.,sim=4_000_000_000),map_points=1000)
        self.assertEqual(m.original.stage,'waiting_sensors')
        self.assertEqual(a[-1]['kind'],'hold_navigation')

    def test_declared_false_interface_rejected(self):
        values=interfaces();values['current_rgb_map_save']['supported']=False
        with self.assertRaises(ValueError): TeacherMission46().bind_runtime_interfaces(values)

    def test_missing_init_blocks(self):
        m=TeacherMission46();m.bind_runtime_interfaces(interfaces());m.start('test',wall_s=0.,sim_ns=0)
        m.tick(wall_s=4.,sim_ns=4_000_000_000,sensors_ok=True,
               observation=pose(wall=4.,sim=4_000_000_000),map_points=1000)
        self.assertEqual(m.original.stage,'waiting_sensors')

    def test_failed_actual_origin_init_rejected(self):
        m=TeacherMission46();m.start('test',wall_s=0.,sim_ns=0)
        r=initialization(m);r['checks']['physical_standing']['passed']=False
        with self.assertRaises(ValueError): m.accept_initialization(r)

    def test_wrong_checkpoint_init_rejected(self):
        m=TeacherMission46();m.start('test',wall_s=0.,sim_ns=0)
        r=initialization(m);r['checkpoint_sha256']='0'*64
        with self.assertRaises(ValueError): m.accept_initialization(r)

    def test_sensor_heading_cannot_change(self):
        m=started();v=copy.deepcopy(m.original.heading_alignment);v['yaw_camera_init_from_world']=.1
        with self.assertRaises(ValueError): m.set_heading_alignment(v)

    def test_truth_pose_rejected(self):
        m=started();o=pose(wall=4.,sim=4_000_000_000,ground_truth_used=True)
        with self.assertRaises(ValueError): m._pose(o,4.,4_000_000_000)

    def test_300ms_age_exact_boundary(self):
        m=started();o=pose(wall=3.7,sim=3_700_000_000)
        self.assertEqual(m._pose(o,4.,4_000_000_000),[0.,0.,0.])
        o['stamp_ns']-=1
        with self.assertRaises(ValueError): m._pose(o,4.,4_000_000_000)

    def test_initial_drift_restarts_original_stability(self):
        m=TeacherMission46();m.bind_runtime_interfaces(interfaces());m.start('test',wall_s=0.,sim_ns=0)
        m.accept_initialization(initialization(m));m.set_heading_alignment(dict(ground_truth_used=False,
            source='actual_slam_and_imu',source_evidence_sha256=TEST_SHA,yaw_camera_init_from_world=0.))
        for wall, x in [(0.,0.),(2.,.09),(3.,.09)]:
            m.tick(wall_s=wall,sim_ns=int(wall*1e9),sensors_ok=True,
                   observation=pose((x,0.,0.),wall,int(wall*1e9)),map_points=100)
        self.assertEqual(m.original.stage,'waiting_sensors')
        self.assertEqual(m.original.origin,[.09,0.,0.])

    def test_fixed_sensor_frame_transform(self):
        m=started(math.pi/2,(10.,20.,30.));g=m.original.current_goals[0]
        self.assertAlmostEqual(g['center'][0],11.)
        self.assertAlmostEqual(g['center'][1],20.)
        self.assertAlmostEqual(g['center'][2],30.)
        self.assertTrue(contains_control(parse_goal(g),g['center']))

    def test_global_simulation_limit(self):
        m=started();sim=round((SIMULATION_MAX_S+.001)*1e9)
        a=m.tick(wall_s=4.,sim_ns=sim,sensors_ok=True,observation=pose(wall=4.,sim=sim),map_points=100)
        self.assertEqual(m.original.stage,'failed');self.assertEqual(a[0]['kind'],'stop_navigation')

    def test_non_tick_callback_cannot_bypass_global_limit(self):
        m=started();sim=round((SIMULATION_MAX_S+.001)*1e9)
        with self.assertRaises(MissionDeadlineExceeded):
            m.navigation_result(status(m),wall_s=4.,sim_ns=sim,
                observation=pose(wall=4.,sim=sim))
        self.assertEqual(m.original.stage,'failed');self.assertFalse(m.completed)

    def test_clock_regression_rejected(self):
        m=started()
        with self.assertRaises(ValueError):
            m.tick(wall_s=4.,sim_ns=2_999_999_999,sensors_ok=True,
                   observation=pose(wall=4.,sim=2_999_999_999),map_points=100)

    def test_stop_retains_teacher_hold_requirement(self):
        m=started();a=m.stop(wall_s=4.,sim_ns=4_000_000_000)
        self.assertEqual(m.original.stage,'stopped');self.assertFalse(m.completed)
        self.assertEqual(a[-1]['kind'],'hold_navigation')
        self.assertTrue(a[-1]['continuous_teacher_required'])


class TerrainTests(unittest.TestCase):
    def test_bidirectional_sequence(self):
        m=navigation_phase()
        cross_terrain(m,60.,60_000_000_000)
        self.assertEqual([(r['from_layer'],r['to_layer']) for r in m.terrain_receipts],
                         [('lower12','upper23'),('upper23','lower12'),('lower12','upper23')])

    def test_wrong_connector_id_rejected(self):
        m=started();s=status(m,12,'running',start=10_000_000_000)
        s['region_arrivals'][-1]['goal_id']='connector_mid'
        a=m.terrain_intent(s,wall_s=20.,sim_ns=20_000_000_000,
            observation=pose(m.original.current_goals[11]['center'],20.,20_000_000_000))
        self.assertEqual(a[0]['kind'],'stop_navigation');self.assertEqual(m.original.stage,'failed')

    def test_not_inside_connector_now_rejected(self):
        m=started();s=status(m,12,'running',start=10_000_000_000)
        m.terrain_intent(s,wall_s=20.,sim_ns=20_000_000_000,observation=pose(wall=20.,sim=20_000_000_000))
        self.assertEqual(m.original.stage,'failed')

    def test_controller_ran_ahead_rejected(self):
        m=started();s=status(m,13,'running',start=10_000_000_000)
        m.terrain_intent(s,wall_s=20.,sim_ns=20_000_000_000,
            observation=pose(m.original.current_goals[11]['center'],20.,20_000_000_000))
        self.assertEqual(m.original.stage,'failed')

    def test_no_phase_success_without_terrain_ack(self):
        m=started();complete_phase(m,20.,20_000_000_000)
        self.assertEqual(m.original.stage,'failed')

    def test_bad_ray_ack_cannot_release(self):
        m=started();s=status(m,12,'running',start=10_000_000_000)
        m.terrain_intent(s,wall_s=20.,sim_ns=20_000_000_000,
            observation=pose(m.original.current_goals[11]['center'],20.,20_000_000_000))
        r=terrain_ack(m);r['maximum_ray_difference_m']=.01
        with self.assertRaises(ValueError): m.accept_terrain(r,wall_s=20.,sim_ns=20_000_000_000)
        self.assertIsNotNone(m.terrain_pending);self.assertEqual(m.layer,'lower12')


class PhaseAndMapTests(unittest.TestCase):
    def test_exact_original_phase_progress(self):
        m=started();cross_terrain(m,20.,20_000_000_000)
        a=complete_phase(m,25.,25_000_000_000)
        self.assertEqual(m.original.stage,'returning');self.assertEqual(len(a[0]['goals']),14)
        self.assertTrue(a[0]['reset_cascade_and_phase_parking'])
        self.assertFalse(a[0]['route_success_is_whole_mission_success'])
        cross_terrain(m,40.,40_000_000_000);a=complete_phase(m,45.,45_000_000_000)
        self.assertEqual(m.original.stage,'saving_map');self.assertEqual(a[0]['kind'],'save_map')
        self.assertEqual(m.snapshot()['completed_region_count'],32)

    def test_dwell_399ms_cannot_pass_400ms(self):
        m=started();cross_terrain(m,20.,20_000_000_000);s=status(m,start=20_000_000_000)
        s['region_arrivals'][0]['dwell_ns']=399_000_000
        s['region_arrivals'][0]['stamp_ns']=20_399_000_000
        m.navigation_result(s,wall_s=30.,sim_ns=30_000_000_000,
            observation=pose(m.original.current_goals[-1]['center'],30.,30_000_000_000))
        self.assertEqual(m.original.stage,'failed')

    def test_foreign_request_does_not_advance(self):
        m=started();s=status(m);s['request_id']='foreign'
        self.assertEqual(m.navigation_result(s,wall_s=4.,sim_ns=4_000_000_000,
            observation=pose(wall=4.,sim=4_000_000_000)),[])
        self.assertEqual(m.original.stage,'exploring')

    def test_actual_map_8_colors_and_new_navigation(self):
        m=navigation_phase();self.assertEqual(m.original.stage,'navigating')
        self.assertEqual(m.original.current_goals[0]['goal_id'],'navigation_f1_f3:0')
        self.assertEqual(m.layer,'lower12');self.assertEqual(len(m.original.current_goals),14)

    def test_stale_saved_map_cannot_advance(self):
        m=started();cross_terrain(m,20.,20_000_000_000);complete_phase(m,25.,25_000_000_000)
        cross_terrain(m,40.,40_000_000_000);complete_phase(m,45.,45_000_000_000)
        r=rgb(m);r['sensor_ages_s']['camera']=2.
        m.map_saved(r,wall_s=46.,sim_ns=46_000_000_000)
        self.assertEqual(m.original.stage,'failed')

    def test_reference_map_cannot_be_used(self):
        m=started();cross_terrain(m,20.,20_000_000_000);complete_phase(m,25.,25_000_000_000)
        cross_terrain(m,40.,40_000_000_000);complete_phase(m,45.,45_000_000_000)
        r=rgb(m);r['reference_map_loaded']=True
        m.map_saved(r,wall_s=46.,sim_ns=46_000_000_000)
        self.assertEqual(m.original.stage,'failed')

    def test_repeated_return_success_during_save_is_ignored(self):
        m=started();cross_terrain(m,20.,20_000_000_000);complete_phase(m,25.,25_000_000_000)
        cross_terrain(m,40.,40_000_000_000)
        st=status(m,start=38_000_000_000)
        p=pose(m.original.current_goals[-1]['center'],45.,45_000_000_000)
        m.navigation_result(st,wall_s=45.,sim_ns=45_000_000_000,observation=p)
        self.assertEqual(m.original.stage,'saving_map')
        self.assertEqual(m.navigation_result(st,wall_s=46.,sim_ns=46_000_000_000,
            observation=pose(p['position'],46.,46_000_000_000)),[])
        self.assertEqual(m.original.stage,'saving_map')


class DynamicAndFinalTests(unittest.TestCase):
    def test_original_dynamic_trigger_index_and_distance(self):
        m=navigation_phase();s=status(m,1,'running',start=46_000_000_000)
        p=pose((1.,2.,0.),50.,50_000_000_000)
        t=m.dynamic_trigger(s,wall_s=50.,sim_ns=50_000_000_000,observation=p)
        self.assertTrue(t['activate']);self.assertEqual(t['point'],[1.,2.,0.])
        s['waypoint_index']=0
        self.assertFalse(m.dynamic_trigger(s,wall_s=50.,sim_ns=50_000_000_000,observation=p)['activate'])
        s['waypoint_index']=1;s['request_id']='foreign'
        self.assertFalse(m.dynamic_trigger(s,wall_s=50.,sim_ns=50_000_000_000,observation=p)['activate'])

    def test_dynamic_unverified_receipt_rejected(self):
        m=navigation_phase()
        r=receipt(m,'teacher_mission46_dynamic_obstacle/v1',DYNAMIC_CHECKS,navigation_ground_truth_used=False)
        with self.assertRaises(ValueError): m.accept_dynamic(r)

    def test_missing_dynamic_evidence_blocks_final(self):
        m=navigation_phase();cross_terrain(m,60.,60_000_000_000)
        complete_phase(m,70.,70_000_000_000)
        self.assertEqual(m.original.stage,'failed');self.assertFalse(m.completed)

    def test_full_sequence_requires_actual_final_hold(self):
        m=navigation_phase();cross_terrain(m,60.,60_000_000_000)
        obstacle=dict(run_id=m.original.run_id,request_id=m.original.current_request,
                      active=True,visible_phase='blocking',gazebo_updates=10)
        m.observe_obstacle(held=True,actual_speed_mps=0.,obstacle=obstacle)
        m.observe_obstacle(held=False,actual_speed_mps=.05,obstacle=obstacle)
        m.accept_dynamic(receipt(m,'teacher_mission46_dynamic_obstacle/v1',DYNAMIC_CHECKS,
                                 navigation_ground_truth_used=False))
        a=complete_phase(m,70.,70_000_000_000)
        self.assertEqual(a[0]['kind'],'final_active_hold');self.assertFalse(m.completed)
        self.assertEqual(m.original.stage,'navigating')
        r=receipt(m,'teacher_mission46_final_parking/v1',PARK_CHECKS,
                  start_stamp_ns=70_000_000_000,end_stamp_ns=75_000_000_000)
        m.parking_complete(r,wall_s=75.,sim_ns=75_000_000_000,
            observation=pose(m.original.current_goals[-1]['center'],75.,75_000_000_000))
        self.assertTrue(m.completed);self.assertEqual(m.original.stage,'completed')
        self.assertEqual(m.snapshot()['completed_region_count'],46)
        self.assertFalse(m.snapshot()['navigation_is_verified'])

    def test_missing_parking_source_key_rejected(self):
        m=navigation_phase();cross_terrain(m,60.,60_000_000_000)
        obstacle=dict(run_id=m.original.run_id,request_id=m.original.current_request,
                      active=True,visible_phase='blocking',gazebo_updates=10)
        m.observe_obstacle(held=True,actual_speed_mps=0.,obstacle=obstacle)
        m.observe_obstacle(held=False,actual_speed_mps=.05,obstacle=obstacle)
        m.accept_dynamic(receipt(m,'teacher_mission46_dynamic_obstacle/v1',DYNAMIC_CHECKS,
                                 navigation_ground_truth_used=False))
        complete_phase(m,70.,70_000_000_000)
        r=receipt(m,'teacher_mission46_final_parking/v1',PARK_CHECKS,
                  start_stamp_ns=70_000_000_000,end_stamp_ns=75_000_000_000)
        del r['checks']['native_motion_limits']
        with self.assertRaises(ValueError): m.parking_complete(r,wall_s=75.,sim_ns=75_000_000_000,
            observation=pose(m.original.current_goals[-1]['center'],75.,75_000_000_000))


if __name__ == '__main__':
    unittest.main(verbosity=2)
