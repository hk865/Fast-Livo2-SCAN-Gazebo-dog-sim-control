"""Finite enablement tests: pure core plus extracted production wrapper method.

These do not run ROS, Actor, Gazebo or recorded-trajectory replay. They do not
certify terrain, plant response or original route-fence candidate admission.
"""
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np

V20=Path(__file__).resolve().parents[2]/'navigation/corridor_tracking_v20'
sys.path.insert(0,str(V20))
from cascade_core import Controller
from pure_tests import sources
from spatial_reference import configuration,speed_budget
from spatial_reference_tests import circle
from controller_reference_tests import wrapper_method
from pid_core import slew

FLAGS=dict(curvature_feedforward_enabled=True,curvature_speed_limit_enabled=True)

def curved(sign=1.):
    p=circle();p[:,1]*=sign
    return p

def make(path=None):
    c=Controller(dict(desired_speed=.3,external_heading_gate=True,spatial_reference=FLAGS),[3.,3.,.32],0.,'goal')
    c.set_path(curved()if path is None else path,'first',0,0)
    return c

def steady(c,last=900_000_000):
    yaw=c.preview_reference([0.,0.,.32])['heading_rad']
    q=(math.cos(yaw/2),0.,0.,math.sin(yaw/2))
    for stamp in range(100_000_000,last+1,100_000_000):
        command,row=c.update(*sources(stamp,quaternion=q),mode_override='drive')
    return command,row,q

class CurvatureEnableChecks(unittest.TestCase):
    def test_joint_flags_effective_feedforward_and_budget_match(self):
        c=make();previous=0.
        yaw=c.preview_reference([0.,0.,.32])['heading_rad'];q=(math.cos(yaw/2),0.,0.,math.sin(yaw/2))
        for stamp in range(100_000_000,1_100_000_000,100_000_000):
            command,row=c.update(*sources(stamp,quaternion=q),mode_override='drive')
            if not row['controller_updated']:continue
            b=row['curvature_speed_supervisor'];k=row['spatial_reference']['curvature_per_m'];v=b['limited_speed_mps']
            self.assertTrue(b['reference_budget_feasible']);self.assertEqual(row['mode'],'drive')
            self.assertAlmostEqual(row['yaw_PD']['curvature_FF'],v*k,places=12)
            expected=1.15*v*k+row['yaw_PD']['P']-.15*c.filtered[2]
            self.assertAlmostEqual(row['reference_COM_velocity_body'][2],expected,places=12)
            self.assertLessEqual(abs(1.15*v*k),b['yaw_rate_budget_radps']+1e-10)
            self.assertLessEqual(abs(1.15*v*k-previous),.6*row['header_dt_s']+1e-10)
            self.assertTrue(np.isfinite(command).all());self.assertTrue(np.all(abs(command)<=[.6,.2,.6]))
            previous=1.15*v*k
        self.assertGreater(previous,.1)

    def test_sign_reversal_handoff_is_infeasible_zero_and_freezes_integral(self):
        c=make();_,row,q=steady(c);old_ff=c.last_curvature_yaw_reference
        self.assertGreater(old_ff,.1)
        c.velocity_integral[:]=[.07,-.02,.03];before=c.velocity_integral.copy()
        c.set_path(curved(-1),'opposite',900_000_000,10_900_000_000,position=[0.,0.,.32])
        self.assertEqual(c.last_curvature_yaw_reference,old_ff)
        command,row=c.update(*sources(930_000_000,quaternion=q),mode_override='drive')
        self.assertEqual(row['mode'],'reference_constraint_hold')
        self.assertFalse(row['curvature_speed_supervisor']['reference_budget_feasible'])
        self.assertTrue(row['curvature_speed_supervisor']['temporal_yaw_slew_checked'])
        np.testing.assert_array_equal(command,[0.,0.,0.]);np.testing.assert_array_equal(c.velocity_integral,before)
        self.assertEqual(row['integral_dt_s'],0.)

    def test_external_locked_turn_excludes_curvature_and_freezes_xy_only(self):
        c=make();_,_,q=steady(c);c.velocity_integral[:]=[.07,-.02,.03]
        command,row=c.update(*sources(1_000_000_000,quaternion=q),mode_override='turn',external_turn_heading=.5)
        self.assertEqual(row['mode'],'turn');self.assertEqual(row['reference_yaw_rad'],.5)
        self.assertEqual(row['yaw_PD']['curvature_FF'],0.);self.assertFalse(row['curvature_speed_supervisor']['enabled'])
        np.testing.assert_array_equal(command[:2],[0.,0.]);np.testing.assert_array_equal(c.velocity_integral[:2],[.07,-.02])
        self.assertTrue(row['velocity_PI']['translation_integral_frozen'])

    def test_coarse_true_corner_both_flags_stops_without_smoothing_through(self):
        c=make([[0.,0.,.32],[1.,0.,.32],[1.,1.,.32]])
        for stamp in (100_000_000,200_000_000):
            command,row=c.update(*sources(stamp,position=(.9,0.,.32)),mode_override='drive')
        self.assertEqual(row['spatial_reference']['fallback_reason'],'corner_chord_deviation')
        self.assertEqual(row['mode'],'reference_constraint_hold');np.testing.assert_array_equal(command,[0.,0.,0.])

    def test_both_flags_duplicate_header_does_not_reintegrate_and_nan_stops(self):
        c=make();_,_,q=steady(c);state=copy.deepcopy(c.__dict__)
        command,row=c.update(*sources(900_000_000,quaternion=q),mode_override='drive')
        self.assertFalse(row['controller_updated']);np.testing.assert_array_equal(c.velocity_integral,state['velocity_integral'])
        self.assertEqual(c.last_curvature_yaw_reference,state['last_curvature_yaw_reference'])
        args=list(sources(1_000_000_000,quaternion=q));args[0]['position_world_xyz'][0]=float('nan')
        command,row=c.update(*args,mode_override='drive');self.assertEqual(row['mode'],'protect')
        np.testing.assert_array_equal(command,[0.,0.,0.])

    def test_production_wrapper_constraint_hold_exact_zero_and_pending_replan(self):
        c=make();_,_,q=steady(c)
        c.set_path(curved(-1),'opposite',900_000_000,10_900_000_000,position=[0.,0.,.32])
        feedback,imu,ack,clock,wall=sources(930_000_000,quaternion=q)
        node=SimpleNamespace(cascade=c,feedback=feedback,paired_imu=imu,ensure_cascade=lambda:None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock)),get_ack=lambda:ack,
            obstacle_hold=False,heading_gate=SimpleNamespace(phase='drive',heading=None),steering={'direction':[1.,0.]},
            state='running',command=np.array([.2,0.,.3]),last_command_time=.9,rotation=np.eye(3),
            pid_records=0,pose_stamp=930_000_000,path_receipt=None,pose=np.array([0.,0.,.32]),pose_quaternion=q,
            request_id='r',waypoint_index=0,active_trajectory_id=2,trajectory_archive_reference='test.npz',waypoints=np.array([[3.,3.,.32]]),
            control_clock_hold=SimpleNamespace(guard_required=False))
        method=wrapper_method('select_pid_velocity',dict(time=SimpleNamespace(monotonic_ns=lambda:wall),slew=slew))
        xy,w=method(node,np.array([.3,0.]),0.,np.array([1.,0.,.32]),True)
        np.testing.assert_array_equal(xy,[0.,0.]);self.assertEqual(w,0.)
        self.assertTrue(node.reference_replan_pending);self.assertEqual(node.pid_row['mode'],'reference_constraint_hold')
        np.testing.assert_array_equal(node.pid_prepared_output,[0.,0.,0.])

    def test_speed_budget_direct_inequalities_grid(self):
        cfg=configuration();cases=0
        for k in (-3.,-1.,0.,.2,2.):
            for dk in (0.,.4,8.):
                for old in (0.,.05,.3):
                    for previous_yaw in (-.3,0.,.3):
                        v,b=speed_budget(.3,k,dk,.6,.08,old,.033,cfg,previous_yaw_reference=previous_yaw)
                        cases+=1
                        if b['reference_budget_feasible']:
                            self.assertTrue(0<=v<=.3)
                            self.assertLessEqual(abs(k)*v,.4+1e-10)
                            self.assertLessEqual(abs(dk)*v*v+abs(k)*abs(v-old)/.033,.6+1e-10)
                            self.assertLessEqual(abs(k*v-previous_yaw),.6*.033+1e-10)
                        else:self.assertEqual(v,0.)
        self.assertEqual(cases,135)

if __name__=='__main__':unittest.main(verbosity=2)
