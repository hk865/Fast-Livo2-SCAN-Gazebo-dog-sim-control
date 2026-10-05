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
            navigation_f1_f3=[[0, 7, 2.4]]))
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
        last = max(r['stamp_ns'] for r in self.rows)
        self.rows = [r for r in self.rows if not (r['source']=='truth' and r['stamp_ns']==last)]
        r = self.evaluate();self.assertFalse(r['passed'])
        self.assertEqual(r['truth_full_pose_coverage']['unpaired_samples'], 1)

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


if __name__ == '__main__':
    unittest.main(verbosity=2)
