"""Synthetic acceptance regressions; no robot, ROS or physics success claim."""
import copy
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tarfile
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('region_evaluator', ROOT/'scripts/evaluate_run.py')
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)
from mission.route_regions import add_route_regions, transformed_route_goals
from navigation.goal_regions import definitions_sha256


class RegionAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='region_evaluator_fixture_')
        self.path = Path(self.temp.name)
        self.scenario = add_route_regions(dict(waypoint_reference='relative_world_axes',
            exploration=[[0, 2, 0], [5, 2, .3]], return_origin=[[0, 0, 0]],
            navigation_f1_f3=[[0, 7, 2.4]]),
            profile=getattr(self, 'profile', 'three_platform_body_arrival_v1'))
        self.mission = dict(run_id='synthetic', origin=[3., -2., .1],
                            heading_alignment=dict(yaw_camera_init_from_world=.7))
        self.rows = []
        self.nav = []
        self.R = Rotation.from_euler('xyz', [.02, -.03, .8])
        self.offset = np.array([10., -4., .315])
        self.time_base = 0
        for ns in range(0, 1_000_000_000, 100_000_000):
            self.pose_pair(ns, self.mission['origin'], 'waiting_sensors')
        start = 2_000_000_000
        for counter, (stage, name) in enumerate(ev.ROUTE_KEYS.items(), 1):
            goals = transformed_route_goals(self.scenario, name, self.mission['origin'], self.mission['heading_alignment'])
            h = definitions_sha256(goals)
            rid = 'synthetic:'+name+':'+str(counter)
            receipts = []
            for i, g in enumerate(goals):
                for ns in range(start, start+400_000_001, 100_000_000):
                    self.pose_pair(ns, g.center, stage)
                receipts.append(dict(request_id=rid, goal_id=g.goal_id, waypoint_index=i,
                    goals_definition_sha256=h, stamp_ns=start+400_000_000, start_stamp_ns=start,
                    dwell_ns=400_000_000, raw_position=list(g.center), center_error_m=0.,
                    region_inside=True, protected=False, reason='arrived',
                    max_observation_gap_ns=200_000_000, arrival_definition=g.definition()['arrival']))
                if g.control_extents is not None:
                    receipts[-1].update(control_region_inside=True,
                        control_arrival_definition=g.control_arrival_definition())
                start += 600_000_000
            self.nav.append(dict(stage=stage, current_request=rid, status=dict(
                request_id=rid, state='succeeded', total=len(goals), waypoint_index=len(goals),
                goals_definition_sha256=h, goals_definitions=[g.definition() for g in goals],
                region_arrivals=receipts)))
            if stage == 'returning':
                self.pose_pair(start-100_000_000, self.mission['origin'], 'saving_map')
        self.pose_pair(start-100_000_000, goals[-1].center, 'completed')
        self.freeze_scenario()

    def tearDown(self):
        self.temp.cleanup()

    def pose_pair(self, ns, p, stage):
        p = np.asarray(p)
        self.rows.extend([dict(source='slam', stamp=ns/1e9, stamp_ns=ns, p=p.tolist(),
                              q=[0, 0, 0, 1], stage=stage),
                          dict(source='truth', stamp=ns/1e9, stamp_ns=ns,
                               p=(self.R.apply(p)+self.offset).tolist(),
                               q=self.R.as_quat().tolist(), stage=stage)])

    def freeze_scenario(self):
        data = json.dumps(self.scenario).encode()
        (self.path/'source_manifest.json').write_text(json.dumps(dict(sha256={
            'simulation/scenario.json': hashlib.sha256(data).hexdigest()})))
        with tarfile.open(self.path/'source_snapshot.tar.gz', 'w:gz') as a:
            item = tarfile.TarInfo('simulation/scenario.json'); item.size = len(data)
            a.addfile(item, io.BytesIO(data))

    def evaluate(self):
        (self.path/'pose_audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        (self.path/'navigation_audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.nav))
        samples, errors = ev.read_poses(self.path/'pose_audit.jsonl')
        self.assertFalse(errors)
        trajectory, r, t, truth = ev.compare_trajectory(samples)
        self.assertEqual(trajectory['initial_pair_stage'], 'waiting_sensors')
        return ev.ordered_region_evidence(self.path, self.scenario, self.mission, samples, r, t, truth)

    def reject(self):
        self.assertFalse(self.evaluate()['passed'])

    def test_once_initial_SE3_all_stages_and_heading_axes_pass(self):
        r = self.evaluate()
        self.assertTrue(r['passed'])
        self.assertTrue(all(x['passed'] for x in r['stages'].values()))
        self.assertEqual(r['truth_full_pose_coverage']['unpaired_samples'], 0)

    def test_wrong_floor_at_same_XY_rejects(self):
        for row in self.rows:
            if row['source'] == 'truth' and row['stage'] == 'exploring':
                row['p'] = (np.array(row['p'])+self.R.apply([0, 0, 1.2])).tolist()
        self.reject()

    def test_center_error_over_point_tolerance_can_pass_declared_region(self):
        stage = self.nav[0]
        g = transformed_route_goals(self.scenario, 'exploration', self.mission['origin'], self.mission['heading_alignment'])[0]
        p = np.array(g.center)+np.array(g.axes).T@np.array([.34, 0, .09])
        receipt = stage['status']['region_arrivals'][0]
        for row in self.rows:
            if row['stage'] == 'exploring' and receipt['start_stamp_ns'] <= row['stamp_ns'] <= receipt['stamp_ns']:
                row['p'] = p.tolist() if row['source']=='slam' else (self.R.apply(p)+self.offset).tolist()
        receipt['raw_position']=p.tolist();receipt['center_error_m']=float(np.linalg.norm(p-g.center))
        self.assertGreater(receipt['center_error_m'], .30)
        self.assertTrue(self.evaluate()['passed'])

    def test_truth_outside_during_first_window_cannot_use_later_window(self):
        receipt = self.nav[0]['status']['region_arrivals'][0]
        for row in self.rows:
            if row['source']=='truth' and row['stamp_ns']==receipt['start_stamp_ns']:
                row['p'] = (np.array(row['p'])+self.R.apply([0, 0, .11])).tolist()
        self.reject()

    def test_missing_active_tail_truth_is_not_silently_skipped(self):
        last = max(r['stamp_ns'] for r in self.rows if r['stage']=='navigating')
        self.rows = [r for r in self.rows if not (r['source']=='truth' and r['stamp_ns']==last)]
        r = self.evaluate();self.assertFalse(r['passed'])
        self.assertEqual(r['truth_full_pose_coverage']['unpaired_samples'], 1)

    def test_completed_shutdown_tail_is_not_an_active_motion_claim(self):
        last = max(r['stamp_ns'] for r in self.rows)
        self.rows = [r for r in self.rows if not (r['source']=='truth' and r['stamp_ns']==last)]
        self.assertTrue(self.evaluate()['passed'])

    def test_missing_truth_bracket_over_150ms_fails_coverage(self):
        ns = self.nav[0]['status']['region_arrivals'][0]['start_stamp_ns']+100_000_000
        self.rows = [r for r in self.rows if not (r['source']=='truth' and r['stamp_ns']==ns)]
        self.reject()

    def test_missing_actual_integer_stamp_fails(self):
        next(r for r in self.rows if r['stage']=='exploring' and r['source']=='slam').pop('stamp_ns')
        self.reject()

    def test_receipt_for_other_request_rejected(self):
        self.nav[0]['status']['region_arrivals'][0]['request_id']='foreign'
        self.reject()

    def test_receipt_wrong_goal_index_or_hash_rejected(self):
        original = copy.deepcopy(self.nav)
        for field, value in [('goal_id', 'foreign'), ('waypoint_index', 4), ('goals_definition_sha256', 'wrong')]:
            self.nav = copy.deepcopy(original);self.nav[0]['status']['region_arrivals'][0][field]=value
            self.reject()

    def test_same_request_changed_definition_rejected(self):
        self.nav[0]['status']['goals_definitions'][0]['arrival']['radius_m']=.5
        self.reject()

    def test_missing_success_callback_is_not_inferred_from_index(self):
        self.nav[0]['status']['state']='running'
        self.reject()

    def test_failed_later_goal_retains_only_real_joint_prefix(self):
        self.nav[0]['status']['state']='failed'
        self.nav[0]['status']['region_arrivals']=self.nav[0]['status']['region_arrivals'][:1]
        self.nav[0]['status']['waypoint_index']=1
        r=self.evaluate();self.assertFalse(r['passed'])
        self.assertEqual(r['stages']['exploring']['reached_waypoints'], 1)
        self.assertFalse(r['stages']['returning']['passed'])

    def test_bad_receipt_position_not_actual_same_stamp_rejected(self):
        receipt=self.nav[0]['status']['region_arrivals'][0]
        receipt['raw_position'][0]+=.01;receipt['center_error_m']=.01
        self.reject()

    def test_centermetric_false_receipt_rejected(self):
        self.nav[0]['status']['region_arrivals'][0]['center_error_m']=1.
        self.reject()

    def test_protected_receipt_or_short_dwell_rejected(self):
        original=copy.deepcopy(self.nav)
        for field,value in [('protected',True),('dwell_ns',399_999_999),('start_stamp_ns',False),('reason','dwell')]:
            self.nav=copy.deepcopy(original);self.nav[0]['status']['region_arrivals'][0][field]=value
            self.reject()

    def test_observation_gap_above_200ms_rejected(self):
        receipt=self.nav[0]['status']['region_arrivals'][0]
        self.rows=[r for r in self.rows if r['stamp_ns'] not in [receipt['start_stamp_ns']+100_000_000, receipt['start_stamp_ns']+200_000_000]]
        self.reject()

    def test_exact_200ms_window_gap_passes(self):
        receipt=self.nav[0]['status']['region_arrivals'][0]
        self.rows=[r for r in self.rows if r['stamp_ns'] != receipt['start_stamp_ns']+100_000_000]
        self.assertTrue(self.evaluate()['passed'])

    def test_reverse_receipts_and_skipped_earlier_goal_block_prefix(self):
        self.nav[0]['status']['region_arrivals'].reverse()
        r=self.evaluate();self.assertFalse(r['passed'])
        self.assertFalse(r['stages']['returning']['passed'])

    def test_window_cannot_reuse_previous_arrival_stamp(self):
        self.nav[0]['status']['region_arrivals'][1]['start_stamp_ns']=self.nav[0]['status']['region_arrivals'][0]['stamp_ns']
        self.reject()

    def test_preregistered_bounds_or_origin_center_changed_rejected(self):
        self.scenario['route_goals']['exploration'][0]['arrival']['radius_m']=.5
        self.reject()

    def test_missing_or_modified_prelaunch_source_manifest_rejected(self):
        (self.path/'source_manifest.json').write_text('{}')
        self.reject()

    def test_epoch_ns_dwell_is_preserved_without_float_roundtrip(self):
        offset=1_790_000_000_123_456_789
        for row in self.rows:
            row['stamp_ns']+=offset;row['stamp']=row['stamp_ns']/1e9
        for nav in self.nav:
            for receipt in nav['status']['region_arrivals']:
                receipt['stamp_ns']+=offset;receipt['start_stamp_ns']+=offset
        self.assertTrue(self.evaluate()['passed'])

    def full_artifacts(self):
        # These payloads are synthetic evaluator fixtures, never physical
        # evidence. Exercise the production evaluate() branch and endpoint gate.
        self.evaluate()
        self.scenario.update(floor_elevations=[0,1.2,2.4],simulation={'kind':'gazebo_physics'})
        reference=dict(description='synthetic declared IMU world reference',
                       world_quaternion=[0,0,0,1],body_imu_quaternion=[0,0,0,1])
        self.scenario['sensors']={'imu':{'body_rpy':[0,0,0],'orientation_reference':reference}}
        waiting=[row for row in self.rows if row['source']=='slam' and row['stage']=='waiting_sensors']
        pairs=[dict(slam_stamp=r['stamp'],imu_stamp=r['stamp'],slam_body_quaternion=r['q'],
                    imu_quaternion=Rotation.from_euler('z',-.7).as_quat().tolist()) for r in waiting]
        heading=ev.calibrate_scene_heading(pairs,imu_reference_world_quaternion=[0,0,0,1],
                    body_imu_quaternion=[0,0,0,1],reference_description=reference['description'])
        heading.update(yaw_camera_init_from_world=.7,rotation_camera_init_from_world=Rotation.from_euler('z',.7).as_matrix().tolist(),attitude_pairs=pairs)
        self.mission['heading_alignment']=heading
        self.mission.update(run_id=self.path.name,stage='completed',completed_stages=ev.STAGES,
            events=[dict(stage=s) for s in ['waiting_sensors']+ev.STAGES+['completed']],
            goal=list(transformed_route_goals(self.scenario,'navigation_f1_f3',self.mission['origin'],heading)[-1].center),
            navigation=dict(feedback_source='/demo/slam/body_odom',frame_id='camera_init',
                counts={'odom':100,'cloud':100,'bspline':10,'commands':100},obstacle_stops=1,obstacle_resumes=1),
            counts={'body_contact_sensor':1},obstacle={'gazebo_updates':21,'failed_updates':0},
            validation=dict(obstacle_hold_events=1,resumed_after_hold=True,body_contact_events=[],max_abs_tilt_rad=0.,
                obstacle_history=[dict(active=True,position=[1,4,1],visible_phase='entering',gazebo_updates=1,failed_updates=0,stamp=1),
                                  dict(active=True,position=[1,2,1],visible_phase='blocking',gazebo_updates=11,failed_updates=0,stamp=2),
                                  dict(active=True,position=[1,0,1],visible_phase='clear',gazebo_updates=21,failed_updates=0,stamp=3)]))
        for nav in self.nav:
            rid=nav['current_request'].replace('synthetic:',self.path.name+':')
            nav['current_request']=nav['status']['request_id']=rid
            for receipt in nav['status']['region_arrivals']:receipt['request_id']=rid
        data=np.zeros(600,dtype=ev.MAP_DTYPE);data['xyz'][:,0]=np.arange(600)*.01
        data['rgb'][:,0]=np.arange(600)%255;data['rgb'][:,1]=123
        (self.path/'colored_map.bin').write_bytes(data.tobytes())
        packed=np.empty(600,dtype=[('xyz','<f4',(3,)),('rgb','<u4')]);packed['xyz']=data['xyz']
        packed['rgb']=(data['rgb'][:,0].astype('u4')<<16)|(123<<8)
        (self.path/'colored_map.pcd').write_bytes(b'FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nPOINTS 600\nDATA binary\n'+packed.tobytes())
        meta=dict(run_id=self.path.name,binary_filename='colored_map.bin',point_count=600,rgb_points=600,
            frame_id='camera_init',ground_truth_used=False,reference_map_loaded=False,source_topic='/cloud_registered',
            color_source='/camera/image_color',observed_rgb_samples=600,capacity_rejections=0,
            counts={k:100 for k in ('camera','lidar','imu','odom','colored_cloud')},
            save=dict(complete=True,filename='colored_map.pcd',point_count=600,
                healthy_sensor_evidence=dict(slam_healthy=True,camera_healthy=True,error=None,
                    ages={k:.1 for k in ('odom','lidar','imu','full_cloud','camera','colored_cloud')})))
        self.freeze_scenario()
        for name,obj in [('scenario',self.scenario),('mission',self.mission),('map_metadata',meta)]:
            (self.path/(name+'.json')).write_text(json.dumps(obj))
        (self.path/'navigation_audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.nav))

    def test_full_evaluate_region_main_and_both_endpoints_pass(self):
        self.full_artifacts();result=ev.evaluate(self.path)
        self.assertTrue(result['passed'],result['failed_checks'])
        self.assertTrue(result['endpoints']['return_origin']['joint_region_arrival'])
        self.assertTrue(result['endpoints']['floor3']['joint_region_arrival'])
        self.assertNotIn('floor3_truth_error',result['failed_checks'])

    def test_full_region_endpoint_over_30cm_uses_same_region(self):
        goal=transformed_route_goals(self.scenario,'navigation_f1_f3',self.mission['origin'],self.mission['heading_alignment'])[-1]
        p=np.array(goal.center)+np.array(goal.axes).T@np.array([.34,0,.09])
        for row in self.rows:
            if row['stage'] in ('navigating','completed'):
                row['p']=p.tolist() if row['source']=='slam' else (self.R.apply(p)+self.offset).tolist()
        receipt=self.nav[-1]['status']['region_arrivals'][0]
        receipt.update(raw_position=p.tolist(),center_error_m=float(np.linalg.norm(p-goal.center)))
        self.full_artifacts();result=ev.evaluate(self.path)
        self.assertTrue(result['passed'],result['failed_checks'])
        self.assertGreater(result['endpoints']['floor3']['truth_error_m'],.30)


class InnerRegionAcceptanceTests(unittest.TestCase):
    """Preregistered control policy; physical acceptance stays at outer bounds."""
    def setUp(self):
        self.f = RegionAcceptanceTests('test_once_initial_SE3_all_stages_and_heading_axes_pass')
        self.f.profile = 'three_platform_body_arrival_v2'
        self.f.setUp()

    def tearDown(self):
        self.f.tearDown()

    def goal(self, stage=0, index=0):
        name = list(ev.ROUTE_KEYS.values())[stage]
        return transformed_route_goals(self.f.scenario, name,
            self.f.mission['origin'], self.f.mission['heading_alignment'])[index]

    def set_window(self, stage, index, source, local_position):
        goal = self.goal(stage, index)
        point = np.array(goal.center)+np.array(goal.axes).T@np.array(local_position)
        receipt = self.f.nav[stage]['status']['region_arrivals'][index]
        for row in self.f.rows:
            if row['source'] == source and receipt['start_stamp_ns'] <= row['stamp_ns'] <= receipt['stamp_ns']:
                row['p'] = point.tolist() if source == 'slam' else (self.f.R.apply(point)+self.f.offset).tolist()
        if source == 'slam':
            receipt.update(raw_position=point.tolist(), center_error_m=float(np.linalg.norm(point-goal.center)))

    def test_v2_all_stages_and_canonical_control_geometry_pass(self):
        result = self.f.evaluate()
        self.assertTrue(result['passed'])
        self.assertIn('control_arrival_rule', result)
        for stage in result['stages'].values():
            for point in stage['waypoints']:
                self.assertTrue(all(x['slam_control_inside'] for x in point['window_observations']))
                outer = point['region_definition']; inner = point['control_region_definition']
                self.assertEqual(outer['center'], inner['center'])
                self.assertEqual(outer['arrival']['control_band'], inner['arrival'])

    def test_raw_only_inside_outer_fails_for_flat_ramp_and_origin(self):
        original_rows, original_nav = copy.deepcopy(self.f.rows), copy.deepcopy(self.f.nav)
        for stage, index, local in [(0,0,[.29,0,0]), (0,1,[.26,0,0]), (1,0,[.18,0,0])]:
            with self.subTest(stage=stage, index=index):
                self.f.rows, self.f.nav = copy.deepcopy(original_rows), copy.deepcopy(original_nav)
                self.set_window(stage, index, 'slam', local)
                self.f.reject()

    def test_truth_can_be_outside_inner_while_still_inside_outer(self):
        for stage,index,raw,gt in [(0,0,[.24,0,.09],[.34,0,.09]),
                (0,1,[.24,.19,.069],[.34,.29,.099]),
                (1,0,[.169,0,0],[.219,0,0]),
                (2,0,[.24,0,.09],[.34,0,.09])]:
            self.set_window(stage,index,'slam',raw)
            self.set_window(stage,index,'truth',gt)
        self.assertTrue(self.f.evaluate()['passed'])

    def test_truth_outer_bound_is_not_relaxed_for_any_geometry(self):
        original = copy.deepcopy(self.f.rows)
        for stage,index,local in [(0,0,[.351,0,0]), (0,1,[.351,0,0]), (1,0,[.221,0,0])]:
            with self.subTest(stage=stage, index=index):
                self.f.rows = copy.deepcopy(original)
                self.set_window(stage,index,'truth',local)
                self.f.reject()

    def test_one_raw_outside_inner_cannot_be_rescued_by_later_window(self):
        receipt = self.f.nav[0]['status']['region_arrivals'][0]
        goal = self.goal()
        point = np.array(goal.center)+np.array(goal.axes).T@np.array([.26,0,0])
        row = next(r for r in self.f.rows if r['source']=='slam' and r['stamp_ns']==receipt['start_stamp_ns'])
        row['p'] = point.tolist()
        self.f.reject()

    def test_missing_false_or_changed_control_receipt_is_rejected(self):
        original = copy.deepcopy(self.f.nav)
        for change in ('missing','false','geometry'):
            with self.subTest(change=change):
                self.f.nav = copy.deepcopy(original)
                receipt = self.f.nav[0]['status']['region_arrivals'][0]
                if change=='missing':receipt.pop('control_region_inside')
                elif change=='false':receipt['control_region_inside']=False
                else:receipt['control_arrival_definition']['radius_m']=.26
                self.f.reject()

    def test_control_axes_receipt_must_match_frozen_scene_transform(self):
        self.f.nav[0]['status']['region_arrivals'][0]['control_arrival_definition']['axes'] = np.eye(3).tolist()
        self.f.reject()

    def test_actual_NAV_control_definition_cannot_change_with_same_hash(self):
        self.f.nav[0]['status']['goals_definitions'][0]['arrival']['control_band']['radius_m']=.24
        self.f.reject()

    def test_control_declaration_mutation_after_freeze_is_rejected(self):
        self.f.scenario['route_goals']['exploration'][0]['arrival']['control_band']['radius_m']=.26
        self.f.reject()

    def test_v2_profile_cannot_omit_control_band_even_in_source_archive(self):
        self.f.scenario['route_goals']['exploration'][0]['arrival'].pop('control_band')
        self.f.freeze_scenario()
        self.f.reject()

    def test_closed_flat_inner_boundary_passes_with_unchanged_outer(self):
        self.set_window(0,0,'slam',[.25,0,.10])
        self.assertTrue(self.f.evaluate()['passed'])

    def test_full_v2_endpoint_truth_over_30cm_stays_outer_region_contract(self):
        self.set_window(2,0,'slam',[.24,0,.09])
        self.set_window(2,0,'truth',[.34,0,.09])
        self.f.full_artifacts()
        result = ev.evaluate(self.f.path)
        self.assertTrue(result['passed'],result['failed_checks'])
        self.assertGreater(result['endpoints']['floor3']['truth_error_m'], .30)

    def test_integer_epoch_receipt_keeps_v2_control_window(self):
        offset = 1_790_000_000_123_456_789
        for row in self.f.rows:
            row['stamp_ns']+=offset;row['stamp']=row['stamp_ns']/1e9
        for nav in self.f.nav:
            for receipt in nav['status']['region_arrivals']:
                receipt['stamp_ns']+=offset;receipt['start_stamp_ns']+=offset
        self.assertTrue(self.f.evaluate()['passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
