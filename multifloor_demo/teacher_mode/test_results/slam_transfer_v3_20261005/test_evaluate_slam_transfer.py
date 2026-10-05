#!/usr/bin/env python3
"""Offline source-integrity negative examples; no ROS or simulator imports."""
import copy
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('private_slam_transfer_validator',HERE/'evaluate_slam_transfer.py')
ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)


def pose(ns,wall=100.,position=(0.,0.,0.),yaw=0.):
    return {'stamp_ns':ns,'accepted':True,'received_monotonic_wall':wall,'position':list(position),
        'quaternion_xyzw':[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],'origin_linear_velocity_body':[0.,0.,0.]}


def imu(ns,wall=100.):
    return {'stamp_ns':ns,'accepted':True,'received_monotonic_wall':wall,'angular_velocity_sensor':[0.,0.,0.]}


class SourceAcceptance(unittest.TestCase):
    def source(self):
        p=pose(1_000_000_000);g=imu(995_000_000)
        e={'pose_stamp_ns':p['stamp_ns'],'gyro_stamp_ns':g['stamp_ns'],'control_pose_stamp_ns':p['stamp_ns'],
            'pose_received_monotonic_wall':100.,'gyro_received_monotonic_wall':100.,
            'clock_ns':1_020_000_000,'monotonic_wall':100.02}
        return e,{p['stamp_ns']:p},{g['stamp_ns']:g}
    def test_causal_join_original_identity(self):
        e,p,g=self.source();_,_,_,ages=ev.bind_original(e,p,g,1_040_000_000,100.04)
        self.assertAlmostEqual(ages['pose_sim_s'],.04)
        self.assertAlmostEqual(ages['gyro_pair_gap_s'],.005)
    def test_future_gyro_cannot_fill_past_pose(self):
        e,p,g=self.source();e['gyro_stamp_ns']=1_005_000_000;g[e['gyro_stamp_ns']]=imu(e['gyro_stamp_ns'])
        with self.assertRaisesRegex(ValueError,'causal'):ev.bind_original(e,p,g,1_040_000_000,100.04)
    def test_future_pose_at_actual_Teacher_read_is_rejected(self):
        e,p,g=self.source()
        with self.assertRaisesRegex(ValueError,'Future SLAM'):ev.bind_original(e,p,g,999_999_999,100.04)
    def test_producer_clock_future_tolerance_is_separate(self):
        e,p,g=self.source();e['clock_ns']=1_090_000_000
        _,_,_,a=ev.bind_original(e,p,g,1_040_000_000,100.04)
        self.assertEqual(a['producer_clock_sim_s'],-.05)
        e['clock_ns']+=1
        with self.assertRaisesRegex(ValueError,'producer'):ev.bind_original(e,p,g,1_040_000_000,100.04)
    def test_heartbeat_cannot_refresh_source_received_wall(self):
        e,p,g=self.source();e['pose_received_monotonic_wall']=100.03
        with self.assertRaisesRegex(ValueError,'refreshed'):ev.bind_original(e,p,g,1_040_000_000,100.04)
    def test_wall_TTL_is_not_replaced_by_fresh_sim_time(self):
        e,p,g=self.source()
        with self.assertRaisesRegex(ValueError,'300ms'):ev.bind_original(e,p,g,1_040_000_000,100.301)
    def test_sim_TTL_is_not_replaced_by_fresh_wall(self):
        e,p,g=self.source()
        with self.assertRaisesRegex(ValueError,'300ms'):ev.bind_original(e,p,g,1_305_000_000,100.04)
    def test_source_absence_is_unverified(self):
        e,p,g=self.source()
        with self.assertRaises(ev.MissingEvidence):ev.bind_original(e,{},g,1_040_000_000,100.04)
    def test_duplicate_accepted_pose_does_not_advance_dwell(self):
        p=pose(1_000_000_000)
        with self.assertRaisesRegex(ValueError,'Duplicate'):ev.original_index([p,p],'SLAM')
    def test_unaccepted_duplicate_is_not_selected(self):
        p=pose(1_000_000_000);q=copy.deepcopy(p);q.update(accepted=False,received_monotonic_wall=101.)
        indexed,n=ev.original_index([p,q],'SLAM')
        self.assertEqual(n,1);self.assertEqual(indexed[p['stamp_ns']]['received_monotonic_wall'],100.)
    def test_bool_timestamp_not_integer_header(self):
        with self.assertRaises(ValueError):ev.integer(True,'stamp')


class RouteAndDwell(unittest.TestCase):
    def anchor(self,yaw=.7,translation=(2.,-3.,.32)):
        p=pose(1_000_000_000,position=translation,yaw=yaw);c,s=math.cos(yaw),math.sin(yaw)
        R=np.array([[c,-s,0],[s,c,0],[0,0,1]]);rel=np.array([[0,0,0],[6,0,0]])
        a={'source':'/demo/slam/body_odom','original_stamp_ns':p['stamp_ns'],'navigation_ground_truth_used':False,
            'route_reference_changes_after_anchor':0,'original_position_camera_init':p['position'],
            'original_quaternion_xyzw':p['quaternion_xyzw'],'received_monotonic_wall':p['received_monotonic_wall'],
            'horizontal_rotation':R.tolist(),'heading_rad':yaw,'relative_points_xyz':rel.tolist(),
            'fixed_route_camera_init_xyz':(rel@R.T+translation).tolist()}
        return a,{p['stamp_ns']:p},rel
    def test_anchor_rotation_translation_from_actual_SLAM_only(self):
        a,p,rel=self.anchor();route,_=ev.validate_anchor(a,p,rel)
        self.assertAlmostEqual(np.linalg.norm(route[1,:2]-route[0,:2]),6.)
        self.assertAlmostEqual(math.atan2(*(route[1,:2]-route[0,:2])[::-1]),.7)
    def test_scene_axes_or_later_pose_cannot_replace_anchor(self):
        a,p,rel=self.anchor();a['fixed_route_camera_init_xyz'][1][1]+=1.
        with self.assertRaisesRegex(ValueError,'exactly derived'):ev.validate_anchor(a,p,rel)
    def test_truth_anchor_provenance_rejected(self):
        a,p,rel=self.anchor();a['navigation_ground_truth_used']=True
        with self.assertRaisesRegex(ValueError,'provenance'):ev.validate_anchor(a,p,rel)
    def dwell(self,stamps,updated=True,gap_violation=False):
        ps={ns:pose(ns,position=(6.,0.,.32)) for ns in stamps};gs={ns:imu(ns) for ns in stamps}
        details=[{'envelope':{'control_pose_stamp_ns':ns,'gyro_stamp_ns':ns},
            'core_original_row':{'controller_updated':updated,'mode':'parking' if i==len(stamps)-1 else 'goal_dwell'}}
            for i,ns in enumerate(stamps)]
        return details,ps,gs
    def test_distinct_actual_header_dwell_completes(self):
        ns=list(range(1_000_000_000,1_700_000_000,100_000_000));d,p,g=self.dwell(ns)
        r=ev.dwell_evidence(d,p,g,np.array([[0,0,.32],[6,0,.32]]),np.eye(3),ns[-1])
        self.assertTrue(r['passed']);self.assertEqual(r['distinct_header_count'],7)
    def test_timer_holds_cannot_advance_dwell(self):
        ns=list(range(1_000_000_000,1_700_000_000,100_000_000));d,p,g=self.dwell(ns,False)
        self.assertFalse(ev.dwell_evidence(d,p,g,np.array([[0,0,.32],[6,0,.32]]),np.eye(3),ns[-1])['passed'])
    def test_duplicate_header_cannot_forge_dwell(self):
        ns=[1_000_000_000]*8;d,p,g=self.dwell(ns)
        with self.assertRaisesRegex(ValueError,'Duplicate'):ev.dwell_evidence(d,p,g,np.array([[0,0,.32],[6,0,.32]]),np.eye(3),ns[-1])
    def test_gap_resets_dwell_without_reusing_old_pose(self):
        ns=[1_000_000_000,1_100_000_000,1_200_000_000,1_500_000_000,1_600_000_000];d,p,g=self.dwell(ns)
        self.assertFalse(ev.dwell_evidence(d,p,g,np.array([[0,0,.32],[6,0,.32]]),np.eye(3),ns[-1])['passed'])
    def test_nonupdated_completion_cannot_forge_dwell(self):
        ns=list(range(1_000_000_000,1_700_000_000,100_000_000));d,p,g=self.dwell(ns);d[-1]['core_original_row']['controller_updated']=False
        self.assertFalse(ev.dwell_evidence(d,p,g,np.array([[0,0,.32],[6,0,.32]]),np.eye(3),ns[-1])['passed'])


class Lifecycle(unittest.TestCase):
    def test_all_child_exit_zero_from_original_log(self):
        with tempfile.TemporaryDirectory() as name:
            log=Path(name)/'SLAM.log'
            log.write_text('\n'.join(f'process started with pid [{i}]\nprocess has finished cleanly [pid {i}]' for i in range(10,16)))
            self.assertTrue(ev.child_cleanup(log,6)['passed'])
            log.write_text(log.read_text()+'\nprocess has died [pid 12, exit code -2]')
            self.assertFalse(ev.child_cleanup(log,6)['passed'])
    def test_missing_run_evidence_not_success(self):
        with tempfile.TemporaryDirectory() as name:
            r=ev.evaluate(Path(name),write=False)
            self.assertEqual(r['status'],'unverified');self.assertFalse(r['actual_SLAM_fixed_route_verified']);self.assertIsNone(r['score'])
    def test_append_only_receipt_no_old_report_replacement(self):
        with tempfile.TemporaryDirectory() as name:
            run=Path(name);r=ev.evaluate(run);old=(run/'summary_slam_transfer_independent.json').read_bytes()
            with self.assertRaisesRegex(ValueError,'overwrite'):ev.evaluate(run)
            self.assertEqual((run/'summary_slam_transfer_independent.json').read_bytes(),old)
            ev.evaluate(run,suffix='second_diagnostic')
            self.assertTrue((run/'summary_slam_transfer_independent.second_diagnostic.json').is_file())


if __name__=='__main__':
    unittest.main()
