#!/usr/bin/env python3
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from trajectory_contract import TrajectoryAssociation,payload


def spline(identity=1):
    return NS(order=3,traj_id=identity,start_time=NS(sec=100,nanosec=0),
              pos_pts=[NS(x=x,y=0.,z=0.) for x in [0.,.3,.7,1.]],knots=[0.]*4+[5.]*4)


class ContractTest(unittest.TestCase):
    def setUp(self):
        self.contract=TrajectoryAssociation(4)
        self.contract.request([99,0],[0.,2.,0.])
        self.msg=spline()
        self.metadata=dict(schema=1,reference_stamp=[99,0],body_goal=[0.,2.,0.],trajectory=payload(self.msg))
    def test_both_orders(self):
        for first in ['spline','metadata']:
            self.setUp()
            calls=[lambda:self.contract.add_spline(self.msg),lambda:self.contract.add_metadata(self.metadata)]
            if first=='metadata':calls.reverse()
            self.assertIsNone(calls[0]());self.assertEqual(calls[1]()[0],self.msg)
    def test_no_reference_rejects_previous_goal(self):
        self.contract.reset()
        self.assertIsNone(self.contract.add_spline(self.msg));self.assertIsNone(self.contract.add_metadata(self.metadata))
    def test_freeze_clock_newer_cannot_relabel_old_reference(self):
        self.metadata['reference_stamp']=[98,0]
        self.msg.start_time.sec=500;self.metadata['trajectory']=payload(self.msg)
        self.assertIsNone(self.contract.add_spline(self.msg));self.assertIsNone(self.contract.add_metadata(self.metadata))
    def test_failed_reference_has_no_matching_metadata(self):
        self.contract.request([100,0],[0.,2.,0.])
        self.assertIsNone(self.contract.add_spline(self.msg));self.assertIsNone(self.contract.add_metadata(self.metadata))
    def test_same_identity_wrong_payload_rejected(self):
        self.metadata['trajectory']['pos_pts'][1][0]=100.
        self.contract.add_spline(self.msg);self.assertIsNone(self.contract.add_metadata(self.metadata))
        self.assertIn('payload differ',self.contract.last_rejected)
    def test_goal_mismatch_rejected(self):
        self.metadata['body_goal']=[1.,0.,0.]
        self.assertIsNone(self.contract.add_metadata(self.metadata))
    def test_adjusted_goal_separate_from_requested(self):
        self.metadata['adjusted_body_goal']=[.2,1.8,0.]
        self.contract.add_metadata(self.metadata);self.assertIsNotNone(self.contract.add_spline(self.msg))
    def test_cache_bounded_and_restart_empty(self):
        for identity in range(20):self.contract.add_spline(spline(identity))
        self.assertEqual(len(self.contract.splines),4)
        self.assertIsNone(TrajectoryAssociation().add_metadata(self.metadata))
    def test_geometry_is_not_decided_by_provenance(self):
        self.msg.pos_pts[-1].x=5.
        self.metadata['trajectory']=payload(self.msg)
        self.contract.add_metadata(self.metadata);self.assertIsNotNone(self.contract.add_spline(self.msg))
    def test_duplicate_delivery_only_once(self):
        self.contract.add_spline(self.msg);self.assertIsNotNone(self.contract.add_metadata(self.metadata))
        self.contract.add_spline(self.msg);self.assertIsNone(self.contract.add_metadata(self.metadata))


if __name__=='__main__':unittest.main()
