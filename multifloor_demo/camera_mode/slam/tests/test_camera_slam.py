"""Small non-ROS safety/evidence contracts; no sensor-SLAM accuracy claim."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from _shared import ROOT, verify_sources
from startup_policy import CameraStaticImuWindow, ObservationHealth
from lidar_relay import valid_cloud
spec = importlib.util.spec_from_file_location('camera_slam_evaluator_test', HERE/'evaluate_run.py')
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


class CameraSlamTests(unittest.TestCase):
    def safety(self):
        return dict(schema=2, mode='camera', state='hold', imu_fresh=True,
                    requested=[0.]*6, safe=[0.]*6)

    def test_original_three_second_static_gate_without_robot(self):
        p = CameraStaticImuWindow()
        for ns in range(10_000_000_000, 13_000_000_001, 1_000_000):
            t = ns/1e9
            p.update_bridge(self.safety(), t)
            p.update(t, t, (0,0,0,1), (0,0,0), (0,0,9.81))
        self.assertTrue(p.ready(13., 13.))
        self.assertEqual(p.report()['quadruped_prerequisites'], [])
        self.assertEqual(p.WINDOW_SECONDS, 3.)
        self.assertEqual(p.WARM_SECONDS, 10.)
        p.update(13.001, 13.001, (0,0,0,1), (0,0,.04), (0,0,9.81))
        self.assertFalse(p.ready(13.001, 13.001))

    def test_camera_safety_requires_all_six_axes_zero_and_original_freshness(self):
        p = CameraStaticImuWindow()
        p.update_bridge(self.safety(), 0.)
        self.assertTrue(p.bridge_ready(.3))
        self.assertFalse(p.bridge_ready(.300001))
        for mutation in (dict(mode='go2'), dict(imu_fresh=False), dict(state='failed'),
                         dict(requested=[0,0,.01,0,0,0]), dict(safe=[0,0,0,0,0,.01]),
                         dict(requested=[0,0,0])):
            p.update_bridge(dict(self.safety(), **mutation), 1.)
            self.assertFalse(p.bridge_ready(1.))

    def test_duplicate_headers_do_not_refresh_wall_or_future_age(self):
        h = ObservationHealth()
        self.assertTrue(h.observe('imu', 1_000_000_000, 1.))
        self.assertFalse(h.observe('imu', 1_000_000_000, 3.))
        self.assertFalse(h.fresh('imu', 3_000_000_000, 3.))
        self.assertFalse(h.fresh('imu', 900_000_000, 1.1))

    def test_passthrough_lidar_has_no_go2_mask_and_rejects_wrong_layout(self):
        fields = [SimpleNamespace(name=n, datatype=7, count=1, offset=i*4) for i,n in enumerate(('x','y','z'))]
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='velodyne'), fields=fields,
                              width=2, height=1, point_step=16, row_step=32, data=bytes(range(32)))
        before = msg.data
        self.assertTrue(valid_cloud(msg, 'velodyne'))
        self.assertEqual(msg.data, before)
        self.assertFalse(valid_cloud(msg, 'base_link'))
        msg.row_step = 31
        self.assertFalse(valid_cloud(msg, 'velodyne'))
        source = (HERE/'lidar_relay.py').read_text()
        self.assertIn('self.publisher.publish(msg)', source)
        self.assertNotIn('self_mask(', source)

    def test_declared_camera_noise_gravity_ablation_keeps_extrinsics_and_initialization(self):
        verify_sources(include_binaries=True)
        old = yaml.safe_load((ROOT/'slam/fastlivo.yaml').read_text())['/**']['ros__parameters']
        new = yaml.safe_load((HERE/'fastlivo.yaml').read_text())['/**']['ros__parameters']
        old['common']['lid_topic'] = '/camera_demo/slam/lidar'
        old['evo']['seq_name'] = 'Camera_Multifloor_LIVO'
        # Camera-specific declared ablations; original Go2 config remains pinned.
        old['imu']['gravity_est_en'] = False
        old['imu']['ba_bg_est_en'] = False
        old['imu']['gyr_cov'] = 4e-8
        old['vio']['img_point_cov'] = 100000000
        self.assertEqual(old, new)
        rig = ET.parse(ROOT/'camera_mode/simulation/generated/camera_rig.sdf')
        noise = rig.findall(".//sensor[@type='imu']/imu/angular_velocity/*/noise/stddev")
        self.assertEqual(len(noise), 3)
        self.assertTrue(all(abs(float(n.text)**2-new['imu']['gyr_cov']) < 1e-20 for n in noise))
        self.assertIs(type(new['imu']['gyr_cov']), float)
        self.assertIs(type(new['vio']['img_point_cov']), int)
        self.assertLess(new['vio']['img_point_cov'], 2**31)
        self.assertEqual(new['common']['img_en'], 1)
        self.assertEqual(new['vio']['max_iterations'], 5)
        self.assertEqual((HERE/'camera.yaml').read_bytes(), (ROOT/'slam/camera.yaml').read_bytes())

    def build_region_fixture(self, directory):
        scenario = json.loads((ROOT/'camera_mode/simulation/scenario.json').read_text())
        mission = dict(run_id='fixture', origin=[1.,2.,.1], heading_alignment=dict(yaw_camera_init_from_world=.4))
        samples = dict(slam=[], truth=[])
        rot = Rotation.from_euler('xyz', [.02,-.03,.8])
        R, offset = rot.as_matrix(), np.array([10.,-4.,.75])
        def pair(ns, p, stage):
            p = np.asarray(p)
            samples['slam'].append(dict(stamp=ns/1e9, stamp_ns=ns, p=p, q=np.array([0.,0.,0.,1.]), stage=stage))
            samples['truth'].append(dict(stamp=ns/1e9, stamp_ns=ns, p=R@p+offset, q=rot.as_quat(), stage=stage))
        for ns in range(0, 1_000_000_000, 100_000_000): pair(ns, mission['origin'], 'waiting_sensors')
        rows, start = [], 2_000_000_000
        for counter, (stage, name) in enumerate(camera.SHARED.ROUTE_KEYS.items(), 1):
            goals = camera.SHARED.transformed_route_goals(scenario, name, mission['origin'], mission['heading_alignment'])
            digest = camera.SHARED.definitions_sha256(goals)
            request = f'fixture:{name}:{counter}'
            receipts = []
            for i,g in enumerate(goals):
                for ns in range(start, start+400_000_001, 100_000_000): pair(ns, g.center, stage)
                receipts.append(dict(request_id=request, goal_id=g.goal_id, waypoint_index=i,
                    goals_definition_sha256=digest, stamp_ns=start+400_000_000, start_stamp_ns=start,
                    dwell_ns=400_000_000, raw_position=list(g.center), center_error_m=0., region_inside=True,
                    protected=False, reason='arrived', max_observation_gap_ns=200_000_000,
                    arrival_definition=g.definition()['arrival'], control_region_inside=True,
                    control_arrival_definition=g.control_arrival_definition()))
                start += 600_000_000
            rows.append(dict(stage=stage, current_request=request, status=dict(mode='camera', request_id=request,
                state='succeeded', total=len(goals), waypoint_index=len(goals), goals_definition_sha256=digest,
                goals_definitions=[g.definition() for g in goals], region_arrivals=receipts)))
        archived = directory/'sources/camera_mode/simulation/scenario.json'
        archived.parent.mkdir(parents=True)
        encoded = json.dumps(scenario).encode()
        archived.write_bytes(encoded)
        manifest = directory/'source_manifest.json'
        manifest.write_text(json.dumps(dict(mode='camera', files=[dict(path='camera_mode/simulation/scenario.json',
                                     sha256=hashlib.sha256(encoded).hexdigest())])))
        (directory/'source_manifest.sha256').write_text(hashlib.sha256(manifest.read_bytes()).hexdigest())
        (directory/'navigation_audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        return scenario, mission, samples, R, offset, rows

    def test_all_original_46_regions_same_initial_transform_pass(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            s,m,p,R,t,rows = self.build_region_fixture(d)
            report = camera.SHARED.ordered_region_evidence(d,s,m,p,R,t,p['truth'])
            self.assertTrue(report['passed'])
            self.assertEqual(sum(x['reached_waypoints'] for x in report['stages'].values()), 46)
            self.assertEqual(report['truth_full_pose_coverage']['unpaired_samples'], 0)

    def test_wrong_floor_first_window_cannot_use_later_window(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            s,m,p,R,t,rows = self.build_region_fixture(d)
            first = rows[0]['status']['region_arrivals'][0]['start_stamp_ns']
            x = next(x for x in p['truth'] if x['stamp_ns'] == first)
            x['p'] += R@np.array([0.,0.,1.2])
            report = camera.SHARED.ordered_region_evidence(d,s,m,p,R,t,p['truth'])
            self.assertFalse(report['passed'])
            self.assertFalse(report['stages']['exploring']['waypoints'][0]['reached_in_order'])

    def test_missing_active_tail_truth_and_modified_declaration_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            d = Path(folder)
            s,m,p,R,t,rows = self.build_region_fixture(d)
            p['truth'].pop()
            report = camera.SHARED.ordered_region_evidence(d,s,m,p,R,t,p['truth'])
            self.assertFalse(report['passed'])
            self.assertEqual(report['truth_full_pose_coverage']['unpaired_samples'], 1)
            s['route_goals']['exploration'][0]['arrival']['radius_m'] = 1.
            self.assertFalse(camera.preregistered_camera_regions(d,s)['passed'])

    def test_header_gap_is_preserved_and_never_filled(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'sensor_audit.jsonl'
            rows = [dict(source=n, stamp_ns=t) for n,stamps in (
                ('imu',[0,1_000_000,3_000_000]), ('camera',[0,100_000_000]), ('lidar',[0,100_000_000])) for t in stamps]
            p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            report = camera.sensor_header_evidence(p)
            self.assertTrue(report['passed'])
            self.assertFalse(report['complete_nominal_sampling'])
            self.assertEqual(report['streams']['imu']['gaps'], [dict(before_ns=1_000_000,after_ns=3_000_000,delta_ns=2_000_000)])

    def test_camera_runtime_list_requires_both_actual_selected_paths_and_sha(self):
        declaration = json.loads((HERE/'source_contract.json').read_text())
        rows = [dict(path=str((ROOT/key).resolve()), sha256=sha)
                for key,sha in declaration['isolated_binaries'].items()]
        runtime = dict(mode='camera', artifacts=rows)
        self.assertTrue(camera.camera_runtime_artifact_evidence(runtime, declaration)['passed'])
        wrong = copy.deepcopy(runtime)
        wrong['artifacts'][0]['path'] = '/tmp/libfast_livo2_core.so'
        self.assertFalse(camera.camera_runtime_artifact_evidence(wrong, declaration)['passed'])
        wrong = copy.deepcopy(runtime)
        wrong['artifacts'][1]['sha256'] = '0'*64
        self.assertFalse(camera.camera_runtime_artifact_evidence(wrong, declaration)['passed'])


if __name__ == '__main__':
    unittest.main()
