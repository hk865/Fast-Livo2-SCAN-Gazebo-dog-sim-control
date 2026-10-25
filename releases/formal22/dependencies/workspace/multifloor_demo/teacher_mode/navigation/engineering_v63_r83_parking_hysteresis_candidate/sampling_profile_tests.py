"""Actual XML / registration-capacity profile checks; temporary files only."""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation
from request import SceneAxisRegistration
from sampling import override_sensors, shape


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEMO = ROOT.parent
BASELINE = ROOT / 'runs/20261005_192400_closed_loop_cascade_lidar64_30hz_rgb10_r1_f728'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Writer:
    def __init__(self): self.rows = []
    def append(self, path, row): self.rows.append(row)


class SamplingProfileChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.run = Path(self.temp.name)
        shutil.copy2(BASELINE/'world_before_sensor_override.sdf', self.run/'world.sdf')
        contract = json.loads((BASELINE/'sensor_contract.json').read_text())
        contract['lidar'].update(rate_hz=10, samples={'vertical': 32, 'horizontal': 480})
        contract['camera']['rate_hz'] = 10
        (self.run/'sensor_contract.json').write_text(json.dumps(contract)+'\n')
    def tearDown(self): self.temp.cleanup()

    def test_20Hz_actual_XML_and_only_four_sensor_leaves(self):
        p = json.loads((HERE/'profiles/l64_r20_c20_210.json').read_text())
        original = ET.parse(self.run/'world.sdf').getroot().find('world')
        receipt = override_sensors(self.run, p)
        changed = ET.parse(self.run/'world.sdf').getroot().find('world')
        lidar = changed.find("model[@name='go2']/.//sensor[@type='gpu_lidar']")
        camera = changed.find("model[@name='go2']/.//sensor[@name='demo_rgb']")
        self.assertEqual(lidar.findtext('lidar/scan/vertical/samples'), '64')
        self.assertEqual(lidar.findtext('lidar/scan/horizontal/samples'), '480')
        self.assertEqual(lidar.findtext('update_rate'), '20')
        self.assertEqual(camera.findtext('update_rate'), '20')
        ol = original.find("model[@name='go2']/.//sensor[@type='gpu_lidar']")
        oc = original.find("model[@name='go2']/.//sensor[@name='demo_rgb']")
        for new, old in ((lidar.find('lidar/scan/vertical/samples'), ol.find('lidar/scan/vertical/samples')),
                         (lidar.find('lidar/scan/horizontal/samples'), ol.find('lidar/scan/horizontal/samples')),
                         (lidar.find('update_rate'), ol.find('update_rate')),
                         (camera.find('update_rate'), oc.find('update_rate'))):
            new.text = old.text
        self.assertEqual(shape(changed), shape(original))
        actual = json.loads((self.run/'sensor_contract.json').read_text())
        self.assertEqual(actual['lidar']['rate_hz'], 20)
        self.assertEqual(actual['camera']['rate_hz'], 20)
        self.assertTrue(receipt['only_allowed_sensor_leaves_changed'])

    def test_20Hz_registration_capacity18_fresh_span850ms_original_gates(self):
        p = json.loads((HERE/'profiles/l64_r20_c20_210.json').read_text())
        override_sensors(self.run, p)
        source = DEMO/'slam/heading_alignment.py'
        snapshot = self.run/'sources/slam/heading_alignment.py'
        snapshot.parent.mkdir(parents=True)
        shutil.copy2(source, snapshot)
        snapshots = self.run/'navigation_source_snapshots.json'
        snapshots.write_text(json.dumps({str(source): {'snapshot': str(snapshot), 'sha256': digest(source)}})+'\n')
        refs = {str(path.resolve()): digest(path) for path in
                (source, snapshot, snapshots, self.run/'sensor_contract.json')}
        registration = SceneAxisRegistration(self.run, {'references': refs}, Writer())
        self.assertEqual(registration.pairs.maxlen, 18)
        imu_rotation = (Rotation.from_quat(registration.reference['world_quaternion']).inv() *
                        Rotation.from_quat(registration.reference['body_imu_quaternion']))
        for i in range(18):
            stamp = 1_000_000_000 + i*50_000_000
            wall = 10_000_000_000 + stamp
            self.assertTrue(registration.observe_imu(dict(stamp_ns=stamp-10_000_000,
                callback_ros_clock_ns=stamp+5_000_000, received_wall_ns=wall-10_000_000,
                orientation_xyzw=imu_rotation.as_quat().tolist(), angular_velocity_sensor=[0.,0.,0.],
                orientation_covariance_0=0., frame_id=registration.imu_config['frame'])))
            self.assertTrue(registration.observe_pose(dict(stamp_ns=stamp,
                callback_ros_clock_ns=stamp+5_000_000, received_wall_ns=wall,
                position=[0.,0.,.32], quaternion_xyzw=[0.,0.,0.,1.],
                body_velocity=[0.,0.,0.]), True, wall+5_000_000))
            if i==15:
                self.assertIsNone(registration.try_freeze(stamp+5_000_000, wall+5_000_000))
        receipt = registration.try_freeze(stamp+5_000_000, wall+5_000_000)
        self.assertIsNotNone(receipt)
        self.assertEqual(len(receipt['exact_paired_sample_records']), 18)
        pairs = registration.pairs
        self.assertEqual(pairs[-1]['slam_stamp_ns']-pairs[0]['slam_stamp_ns'], 850_000_000)
        self.assertEqual(receipt['minimum_samples'], 10)
        self.assertEqual(receipt['minimum_span_ns'], 800_000_000)
        self.assertEqual(receipt['maximum_pair_gap_ns'], 20_000_000)

    def test_low_log30_profile_changes_only_scope_name_prospective_text_and_window(self):
        original = json.loads((HERE/'profiles/l64_r30_c30_210.json').read_text())
        reduced = json.loads((HERE/'profiles/l64_r30_c30_detail3s_210.json').read_text())
        self.assertEqual(reduced['diagnostic_scope']['detail_window_sim_s'], [115,118])
        reduced['diagnostic_scope']['detail_window_sim_s'] = [115,165]
        reduced['experiment'] = original['experiment']
        reduced['prospective_change'] = original['prospective_change']
        self.assertEqual(reduced, original)

    def test_20Hz600_preserves_original600_mission_and_gains(self):
        original = json.loads((HERE/'profiles/l64_r30_c30_600.json').read_text())
        changed = json.loads((HERE/'profiles/l64_r20_c20_600.json').read_text())
        self.assertEqual(changed['duration_s'], 600)
        for key in ('lidar_hz','camera_hz','scope'):
            changed['sensor_sampling'][key] = original['sensor_sampling'][key]
        changed['cascade']['feedback_expected_hz'] = original['cascade']['feedback_expected_hz']
        for key in ('experiment','prospective_change'):
            changed[key] = original[key]
        self.assertEqual(changed, original)


if __name__ == '__main__': unittest.main()
