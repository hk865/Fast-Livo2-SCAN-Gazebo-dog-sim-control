"""Offline mathematical/regression checks, not a navigation or plant receipt."""
import math
import importlib.util
from pathlib import Path
import unittest
import numpy as np

from cascade_core import Controller
from pure_tests import ready, sources
from spatial_reference import configuration, reference, speed_budget


def path_reference(points,s,**settings):
    points=np.asarray(points,float)
    cumulative=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
    index=min(max(0,int(np.searchsorted(cumulative,s,side='right'))-1),len(points)-2)
    delta=points[index+1]-points[index]
    tangent=delta[:2]/np.linalg.norm(delta[:2])
    point=points[index]+(s-cumulative[index])/(cumulative[index+1]-cumulative[index])*delta
    return reference(points,cumulative,s,point,point,tangent,configuration(settings))


def circle(radius=1.,start=0.,stop=2.):
    angle=np.linspace(start,stop,2001)
    return np.c_[radius*np.sin(angle),radius*(1.-np.cos(angle)),np.full(len(angle),.32)]


class SpatialReferenceChecks(unittest.TestCase):
    def test_original_hold_capture_and_straight_commands_are_preserved(self):
        # Compare externally meaningful original behavior, rather than an
        # obsolete demand for identical dictionaries after adding evidence.
        spec=importlib.util.spec_from_file_location('v19_regression_core',Path(__file__).resolve().parent.parent/'pipeline_v19'/'cascade_core.py')
        old_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(old_module)
        for scenario in ('drive','capture','hold'):
            old=old_module.Controller({'external_heading_gate':True},[4.,0.,.32],0.,'same')
            new=Controller({'external_heading_gate':True},[4.,0.,.32],0.,'same')
            for c in (old,new):c.set_path([[0.,0.,.32],[4.,0.,.32]],'line',0,0)
            for i in range(1,12):
                position=(3.99,0.,.32) if scenario=='capture' else (.05*i,.03,.32)
                mode=('capture' if i==2 else None) if scenario=='capture' else ('hold' if i==5 and scenario=='hold' else 'drive')
                a,ar=old.update(*sources(i*100_000_000,position=position),mode_override=mode)
                b,br=new.update(*sources(i*100_000_000,position=position),mode_override=mode)
                np.testing.assert_array_equal(a,b)
                for name in ('velocity_integral','hold_integral','filtered','inner_filtered'):
                    np.testing.assert_array_equal(getattr(old,name),getattr(new,name))
                for name in ('mode','controller_updated','failure_latched','fixed_goal',
                    'parking_capture_pose_stamp_ns','parking_hold_declared_pose_stamp_ns'):
                    self.assertEqual(ar[name],br[name])

    def test_microsegment_changes_heading_without_replacing_path(self):
        c=Controller({'external_heading_gate':True},[2.,.001,.32],0.,'g')
        points=[[0.,0.,.32],[.0001,.001,.32],[2.,.001,.32]]
        c.set_path(points,'micro',0,0)
        old_path=c.path.copy();old_progress=c.progress
        ref=c.preview_reference([0.,0.,.32])
        self.assertGreater(math.atan2(*ref['raw_projection']['tangent_xy'][::-1]),1.)
        self.assertLess(abs(ref['heading_rad']),.01)
        self.assertFalse(ref['original_path_replaced'])
        self.assertFalse(ref['chord_is_collision_checked_path'])
        self.assertTrue(ref['actual_motion_guard_required'])
        np.testing.assert_array_equal(c.path,old_path)
        self.assertEqual(c.progress,old_progress)
        self.assertTrue(c.new_path)

    def test_cross_control_uses_its_own_normal_raw_cross_remains(self):
        c=Controller({'external_heading_gate':True},[2.,.001,.32],0.,'g')
        c.set_path([[0.,0.,.32],[.0001,.001,.32],[2.,.001,.32]],'micro',0,0)
        position=np.array([-.001,.0005,.32])
        preview=c.preview_reference(position)
        _,row=ready(c,position=position)
        np.testing.assert_allclose(row['control_horizontal_normal'],preview['normal_xy'])
        raw=preview['raw_projection']
        expected=float((position[:2]-np.asarray(raw['point_xyz'])[:2])@np.asarray(preview['normal_xy']))
        self.assertAlmostEqual(row['control_error_cross_m'],expected)
        self.assertAlmostEqual(row['error_cross_m'],raw['cross_m'])
        self.assertNotAlmostEqual(row['error_cross_m'],row['control_error_cross_m'],places=5)

    def test_fixed_goal_tail_heading_is_identical_inside_and_outside(self):
        c=Controller({'external_heading_gate':True},[1.,0.,.32],.7,'g')
        c.set_path([[0.,0.,.32],[1.,0.,.32]],'tail',0,0)
        preview=c.preview_reference([.9,0.,.32])
        _,row=ready(c,position=(.9,0.,.32))
        self.assertEqual(preview['heading_source'],'fixed_goal_tail')
        self.assertEqual(preview['heading_rad'],.7)
        self.assertEqual(row['reference_yaw_rad'],preview['heading_rad'])
        self.assertLessEqual(preview['arc_used_m'],.10000001)
        self.assertFalse(preview['curvature_valid'])

    def test_ramp_microsegment_vertical_reference_matches_declared_gradient(self):
        c=Controller({'external_heading_gate':True},[2.,.001,.82],0.,'ramp')
        c.set_path([[0.,0.,.32],[.0001,.001,.320025],[2.,.001,.82]],'ramp_micro',0,0)
        _,row=ready(c,position=(-.001,.0005,.32))
        world=np.asarray(row['reference_velocity_world'])
        gradient=row['local_grade_dz_ds']*np.asarray(row['local_horizontal_tangent'])
        self.assertAlmostEqual(world[2],float(world[:2]@gradient))
        self.assertIn('assuming_raw_normal_equal_height',row['vertical_reference_model'])
        self.assertFalse(row['vertical_reference_terrain_or_support_verified'])
        # The smoothed direction is not silently substituted into raw slope
        # evidence, nor is the old forward-only z formula used with new xy.
        self.assertGreater(abs(row['control_horizontal_tangent'][0]-row['local_horizontal_tangent'][0]),.5)
        self.assertGreater(abs(world[2]-.3*row['local_grade_dz_ds']),1e-3)

    def test_true_corner_is_not_silently_rounded_into_diagonal_path(self):
        ref=path_reference([[0.,0.,.32],[1.,0.,.32],[1.,1.,.32]],.9)
        self.assertEqual(ref['fallback_reason'],'corner_chord_deviation')
        self.assertGreater(ref['chord_deviation_m'],.06)
        np.testing.assert_allclose(ref['tangent_xy'],[1.,0.])
        self.assertFalse(ref['curvature_valid'])
        self.assertFalse(ref['original_path_replaced'])

    def test_reversal_chord_cancellation_is_explicit(self):
        ref=path_reference([[0.,0.,.32],[.1,0.,.32],[0.,0.,.32]],0.)
        self.assertEqual(ref['fallback_reason'],'forward_chord_cancellation')
        self.assertFalse(ref['curvature_valid'])
        self.assertTrue(np.isfinite(ref['tangent_xy']).all())

    def test_circle_curvature_and_s_curvature_sign_are_spatial(self):
        ref=path_reference(circle(radius=.6),.45)
        self.assertTrue(ref['curvature_valid'])
        self.assertAlmostEqual(ref['curvature_per_m'],1./.6,places=4)
        self.assertAlmostEqual(ref['curvature_rate_per_m2'],0.,places=3)
        s=np.linspace(0.,6.,3001);theta=.6*np.sin(2.*math.pi*s/6.)
        ds=s[1]-s[0]
        points=np.c_[np.r_[0.,np.cumsum(np.cos(theta[:-1])*ds)],
            np.r_[0.,np.cumsum(np.sin(theta[:-1])*ds)],np.full(len(s),.32)]
        left=path_reference(points,.3);right=path_reference(points,2.7)
        self.assertGreater(left['curvature_per_m'],.4)
        self.assertLess(right['curvature_per_m'],-.4)
        self.assertGreater(abs(path_reference(points,1.)['curvature_rate_per_m2']),.1)

    def test_handoff_maps_real_pose_and_preserves_command_pi_and_filter(self):
        c=Controller({'external_heading_gate':True},[4.,0.,.32],0.,'g')
        c.set_path([[0.,0.,.32],[4.,0.,.32]],'old',0,0)
        ready(c,position=(2.,0.,.32),velocity=(.17,.02,0.))
        c.update(*sources(300_000_000,position=(2.01,0.,.32),velocity=(.17,.02,0.)))
        old=[c.command.copy(),c.velocity_integral.copy(),c.filtered.copy(),c.inner_filtered.copy(),c.last_reference_speed]
        c.set_path([[0.,.01,.32],[4.,0.,.32]],'new',250_000_000,10_250_000_000,position=[2.01,0.,.32])
        self.assertGreater(c.progress,2.)
        self.assertEqual(c.path_handoff['source'],'provided_actual_SLAM_pose')
        for actual,expected in zip([c.command,c.velocity_integral,c.filtered,c.inner_filtered,c.last_reference_speed],old):
            np.testing.assert_array_equal(actual,expected)
        preview=c.preview_reference([2.02,0.,.32])
        _,row=c.update(*sources(400_000_000,position=(2.02,0.,.32),velocity=(.17,.02,0.)))
        self.assertAlmostEqual(row['reference_yaw_rad'],preview['heading_rad'])
        self.assertAlmostEqual(row['nearest_projection_progress_m'],preview['nearest_s_m'])

    def test_handoff_and_preview_do_not_jump_to_same_xy_lower_floor(self):
        c=Controller({'external_heading_gate':True},[2.,0.,1.32],0.,'g')
        c.set_path([[0.,0.,1.32],[2.,0.,1.32]],'upper',0,0)
        ready(c,position=(1.,0.,1.32))
        points=[[0.,0.,.32],[2.,0.,.32],[2.,2.,1.32],[0.,0.,1.32],[2.,0.,1.32]]
        c.set_path(points,'overlap',150_000_000,10_150_000_000,position=[1.,0.,1.32])
        ref=c.preview_reference([1.01,0.,1.32])
        self.assertEqual(ref['raw_projection']['segment'],3)
        self.assertEqual(ref['raw_projection']['point_xyz'][2],1.32)
        self.assertEqual(c.path_handoff['segment'],3)

    def test_replan_during_external_turn_keeps_lock_then_drive_uses_spatial_reference(self):
        c=Controller({'external_heading_gate':True},[2.,.001,.32],0.,'g')
        c.set_path([[0.,0.,.32],[.0001,.001,.32],[2.,.001,.32]],'old',0,0)
        lock=.7
        for stamp in (100_000_000,200_000_000):
            _,row=c.update(*sources(stamp),mode_override='turn',external_turn_heading=lock)
        self.assertEqual(row['reference_yaw_rad'],lock)
        c.set_path([[0.,0.,.32],[.0001,-.001,.32],[2.,.001,.32]],'new',200_000_000,10_200_000_000,position=[0.,0.,.32])
        preview=c.preview_reference([0.,0.,.32])
        cmd,row=c.update(*sources(300_000_000),mode_override='turn',external_turn_heading=lock)
        self.assertEqual(row['reference_yaw_rad'],lock)
        self.assertAlmostEqual(row['ungated_reference_yaw_rad'],preview['heading_rad'])
        np.testing.assert_array_equal(cmd[:2],[0.,0.])
        _,row=c.update(*sources(400_000_000),mode_override='drive',external_turn_heading=lock)
        self.assertAlmostEqual(row['reference_yaw_rad'],preview['heading_rad'])
        self.assertNotIn('external_turn_heading_rad',row)

    def test_invalid_external_turn_reference_remains_fatal_before_duplicate_reuse(self):
        c=Controller({'external_heading_gate':True},[2.,0.,.32],0.,'g')
        c.set_path([[0.,0.,.32],[2.,0.,.32]],'line',0,0)
        ready(c)
        cmd,row=c.update(*sources(200_000_000),mode_override='turn',external_turn_heading=float('nan'))
        self.assertEqual(row['mode'],'protect')
        self.assertEqual(row['failure_latched'],'Invalid finite external turn heading')
        np.testing.assert_array_equal(cmd,[0.,0.,0.])

    def test_disabled_reference_preserves_original_segment_direction(self):
        ref=path_reference([[0.,0.,.32],[.0001,.001,.32],[2.,.001,.32]],0.,enabled=False)
        self.assertGreater(ref['heading_rad'],1.)
        self.assertEqual(ref['fallback_reason'],'spatial_reference_disabled')
        self.assertEqual(ref['curvature_per_m'],0.)

    def test_speed_budget_rate_acceleration_and_infeasible_transition(self):
        cfg=configuration()
        limited,row=speed_budget(.6,2.,0.,.6,.08,None,0.,cfg)
        self.assertAlmostEqual(limited,.2)
        self.assertLessEqual(2.*limited+.08+cfg['yaw_rate_reserve_radps'],.6+1e-10)
        limited,row=speed_budget(.6,0.,4.,.6,0.,None,0.,cfg)
        self.assertAlmostEqual(limited,math.sqrt(.6/4.))
        limited,row=speed_budget(.3,1.,0.,.6,0.,0.,.1,cfg)
        self.assertAlmostEqual(limited,.06)
        limited,row=speed_budget(.3,2.,10.,.6,0.,.3,.01,cfg)
        self.assertFalse(row['reference_budget_feasible'])
        self.assertEqual(limited,0.)
        # A new path can reverse curvature without any spatial k-prime on
        # either path. The temporal reference check must still catch it.
        limited,row=speed_budget(.3,-1.,0.,.6,0.,.3,.1,cfg,previous_yaw_reference=.3)
        self.assertFalse(row['reference_budget_feasible'])
        self.assertTrue(row['temporal_yaw_slew_checked'])
        self.assertEqual(limited,0.)
        limited,row=speed_budget(.3,1.,0.,.6,0.,None,0.,cfg,previous_yaw_reference=0.)
        self.assertEqual(limited,0.)
        self.assertTrue(row['reference_budget_feasible'])

    def test_curve_feature_flags_are_separate_and_corner_hold_freezes_pi(self):
        cfg={'external_heading_gate':True,'spatial_reference':{'curvature_speed_limit_enabled':True}}
        c=Controller(cfg,[1.,1.,.32],math.pi/2.,'g')
        c.set_path([[0.,0.,.32],[1.,0.,.32],[1.,1.,.32]],'corner',0,0)
        _,row=ready(c,position=(.9,0.,.32))
        self.assertEqual(row['mode'],'reference_constraint_hold')
        self.assertFalse(row['curvature_speed_supervisor']['reference_budget_feasible'])
        np.testing.assert_array_equal(row['command_body'],[0.,0.,0.])
        np.testing.assert_array_equal(c.velocity_integral,[0.,0.,0.])
        c=Controller({'external_heading_gate':True,'spatial_reference':{'curvature_feedforward_enabled':True}},[2.,2.,.32],0.,'g')
        c.set_path(circle(),'curve',0,0)
        position=circle()[800]
        _,row=ready(c,position=position,quaternion=(math.cos(.45),0.,0.,math.sin(.45)))
        self.assertGreater(row['yaw_PD']['curvature_FF'],0.)
        self.assertFalse(row['curvature_speed_supervisor']['enabled'])

    def test_invalid_configuration_and_handoff_are_rejected(self):
        for settings in ({'arc_length_m':float('nan')},{'curvature_feedforward_enabled':1},
            {'unknown':1},{'enabled':False,'curvature_speed_limit_enabled':True},
            {'yaw_acceleration_reserve_radps2':1.}):
            with self.assertRaises(ValueError):configuration(settings)
        c=Controller({},[2.,0.,.32],0.,'g')
        c.set_path([[0.,0.,.32],[2.,0.,.32]],'old',0,0)
        old=c.path.copy()
        with self.assertRaises(ValueError):
            c.set_path([[0.,0.,.32],[2.,0.,.32]],'bad',1,1,position=[float('nan'),0.,.32])
        np.testing.assert_array_equal(c.path,old)


if __name__=='__main__':
    unittest.main()
