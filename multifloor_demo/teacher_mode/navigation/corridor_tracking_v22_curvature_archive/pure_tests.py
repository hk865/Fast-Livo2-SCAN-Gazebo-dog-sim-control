"""Offline invariants only. No Teacher, ROS, simulator, or source-run writes."""
import copy
import json
import math
import unittest
import numpy as np
from cascade_core import Controller, DEFAULT_GAINS, DEFAULT_COM, rotation


def controller(goal=(4.,0.,.32),yaw=0.,**config):
    c=Controller(dict(desired_speed=.3,external_heading_gate=True,**config),list(goal),yaw,'fixed-test-goal')
    c.set_path([[0.,0.,.32],list(goal)],'scan-1',0,0)
    return c


def sources(stamp,position=(0.,0.,.32),velocity=(0.,0.,0.),gyro=(0.,0.,0.),quaternion=(1.,0.,0.,0.),wall=None):
    wall=10_000_000_000+stamp if wall is None else wall
    pose=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='camera_init',
        position_world_xyz=list(position),quaternion_wxyz=list(quaternion),origin_velocity_body=list(velocity))
    imu=dict(stamp_ns=stamp-10_000_000,received_wall_ns=wall-10_000_000,
        frame_id='body',angular_velocity_body=list(gyro))
    ack=dict(stamp_ns=stamp-5_000_000,received_wall_ns=wall-5_000_000,
        sequence=stamp//1_000_000,requested_command_body=[0.,0.,0.],applied_command_body=[0.,0.,0.])
    return pose,imu,ack,stamp+5_000_000,wall


def ready(c,**kwargs):
    c.update(*sources(100_000_000,**kwargs))
    return c.update(*sources(200_000_000,**kwargs))


class PureChecks(unittest.TestCase):
    def test_verified_gains_are_preserved(self):
        c=controller(); self.assertEqual(c.g,DEFAULT_GAINS)
        with self.assertRaises(ValueError):
            controller(gains={**DEFAULT_GAINS,'cross_kp':1.})

    def test_two_fresh_recovery_and_hold_does_not_integrate(self):
        c=controller(); first=c.update(*sources(100_000_000))
        self.assertEqual(first[1]['mode'],'recovering')
        _,row=c.update(*sources(200_000_000)); self.assertTrue(row['controller_updated'])
        args=sources(200_000_000); args=list(args); args[-1]+=20_000_000
        old=c.velocity_integral.copy(); _,row=c.update(*args)
        self.assertFalse(row['controller_updated']); np.testing.assert_array_equal(c.velocity_integral,old)

    def test_math_update_is_header_event_not_heartbeat_rate(self):
        c=controller(); count=0
        for stamp in range(100_000_000,700_000_000,100_000_000):
            args=sources(stamp)
            count+=int(c.update(*args)[1]['controller_updated'])
            for delay in (10_000_000,20_000_000,30_000_000):
                held=list(args); held[-1]+=delay
                count+=int(c.update(*held)[1]['controller_updated'])
        self.assertEqual(count,5)

    def test_true_receipt_gap_checked_before_new_measurement(self):
        c=controller(); ready(c)
        _,row=c.update(*sources(300_000_000,wall=10_600_000_000))
        self.assertEqual(row['mode'],'recovering'); self.assertEqual(row['reason'],'feedback_gap_before_fresh')
        _,row=c.update(*sources(400_000_000,wall=10_700_000_000))
        self.assertTrue(row['controller_updated']); self.assertEqual(row['header_dt_s'],0.)

    def test_future_gyro_or_ack_rejected_then_recoverable(self):
        c=controller(); ready(c)
        args=list(sources(300_000_000)); args[1]['stamp_ns']=300_000_001
        _,row=c.update(*args); self.assertEqual(row['mode'],'protect'); self.assertIsNone(row['failure_latched'])
        c.update(*sources(400_000_000)); self.assertTrue(c.update(*sources(500_000_000))[1]['controller_updated'])
        args=list(sources(600_000_000)); args[2]['stamp_ns']=600_000_001
        self.assertEqual(c.update(*args)[1]['mode'],'protect')

    def test_nan_and_backwards_pose_latch_failure(self):
        for kind in ('nan','backward'):
            c=controller(); ready(c)
            args=list(sources(300_000_000 if kind=='nan' else 150_000_000))
            if kind=='nan':args[0]['position_world_xyz'][0]=float('nan')
            row=c.update(*args)[1]; self.assertIsNotNone(row['failure_latched'])
            row=c.update(*sources(400_000_000))[1]; self.assertEqual(row['mode'],'protect')

    def test_repeated_header_cannot_refresh_receipt(self):
        c=controller(); ready(c)
        args=list(sources(200_000_000)); args[0]['received_wall_ns']+=1; args[-1]+=1
        self.assertIsNotNone(c.update(*args)[1]['failure_latched'])

    def test_scan_replan_preserves_integral_and_filter(self):
        c=controller(); ready(c); c.update(*sources(300_000_000))
        integral=c.velocity_integral.copy(); filtered=c.inner_filtered.copy()
        self.assertGreater(integral[0],0.)
        c.set_path([[0.,.05,.32],[4.,0.,.32]],'scan-2',250_000_000,10_250_000_000)
        np.testing.assert_array_equal(c.velocity_integral,integral)
        np.testing.assert_array_equal(c.inner_filtered,filtered)
        self.assertFalse(c.set_path([[0.,.05,.32],[4.,0.,.32]],'scan-2',250_000_000,10_250_000_000))
        with self.assertRaises(ValueError):
            c.set_path([[0.,.06,.32],[4.,0.,.32]],'scan-2',250_000_000,10_250_000_000)

    def test_static_checked_path_does_not_expire_with_source_ttl(self):
        c=controller(); c.update(*sources(5_000_000_000))
        self.assertTrue(c.update(*sources(5_100_000_000))[1]['controller_updated'])

    def test_3d_projection_separates_overlapping_floors(self):
        c=Controller({},[2.,0.,1.32],0.,'upper-floor')
        c.set_path([[0.,0.,.32],[2.,0.,.32],[2.,2.,1.32],[0.,0.,1.32],[2.,0.,1.32]],'floors',0,0)
        row=ready(c,position=(1.,0.,1.32))[1]
        self.assertEqual(row['segment'],3); self.assertAlmostEqual(row['nearest_projection_xyz'][2],1.32)

    def test_cross_sign_and_grade_are_geometric(self):
        c=Controller({'external_heading_gate':True},[4.,0.,1.32],0.,'ramp')
        c.set_path([[0.,0.,.32],[4.,0.,1.32]],'ramp',0,0)
        row=ready(c,position=(1.,.1,.57))[1]
        self.assertAlmostEqual(row['error_cross_m'],.1)
        self.assertAlmostEqual(row['local_grade_dz_ds'],.25)
        self.assertLess(row['reference_velocity_world'][1],0.)
        self.assertAlmostEqual(row['reference_velocity_world'][2],.075)

    def test_euler_yawrate_and_com_lever(self):
        roll,pitch,yaw=.2,.3,.4
        cr,sr=math.cos(roll/2),math.sin(roll/2); cp,sp=math.cos(pitch/2),math.sin(pitch/2)
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        q=(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
        c=controller(); row=ready(c,quaternion=q,velocity=(.2,-.1,.05),gyro=(.1,.2,.3))[1]
        expected=(math.sin(roll)*.2+math.cos(roll)*.3)/math.cos(pitch)
        self.assertAlmostEqual(row['measured_Euler_yawrate_radps'],expected)
        np.testing.assert_allclose(row['measured_COM_velocity_body'],
            np.array([.2,-.1,.05])+np.cross([.1,.2,.3],DEFAULT_COM))

    def test_ack_residual_antiwindup_not_guessed_slew(self):
        c=controller(); ready(c)
        args=list(sources(300_000_000)); args[2]['requested_command_body']=[.2,0.,0.]
        args[2]['applied_command_body']=[.01,0.,0.]
        row=c.update(*args)[1]
        self.assertTrue(row['velocity_PI']['blocked_actual_ack'][0])
        self.assertEqual(c.velocity_integral[0],0.)
        self.assertFalse(row['final_slew_performed_here'])
        self.assertGreater(row['command_body'][0],.012)

    def test_limits_and_turn_do_not_integrate_unactuated_xy(self):
        c=controller(command_limits=[.6,.2,.3]); ready(c)
        old=c.velocity_integral.copy()
        _,row=c.update(*sources(300_000_000),mode_override='turn')
        self.assertEqual(row['command_body'][:2],[0.,0.])
        np.testing.assert_array_equal(c.velocity_integral[:2],old[:2])
        self.assertTrue(np.all(abs(np.array(row['command_body']))<=[.6,.2,.3]))

    def test_full_loop_cannot_complete_at_initial_endpoint(self):
        c=Controller({'external_heading_gate':True},[0.,0.,.32],0.,'lap')
        c.set_path([[0.,0.,.32],[1.,0.,.32],[1.,1.,.32],[0.,1.,.32],[0.,0.,.32]],'loop',0,0)
        row=ready(c)[1]
        self.assertIsNone(row['parking_capture_pose_stamp_ns']); self.assertEqual(row['progress_m'],0.)

    def test_capture_hold_fixed_target_first_window_and_integral(self):
        c=controller(); p=(3.99,0.,.32)
        c.update(*sources(100_000_000,position=p))
        _,row=c.update(*sources(200_000_000,position=p),mode_override='capture')
        self.assertEqual(row['mode'],'capture'); self.assertTrue(row['capture_entry_integral_reset'])
        for t in range(300_000_000,900_000_000,100_000_000):
            _,row=c.update(*sources(t,position=p))
        self.assertEqual(row['mode'],'active_hold')
        self.assertEqual(row['parking_hold_declared_pose_stamp_ns'],800_000_000)
        self.assertEqual(row['command_limits_body'],[.06,.04,.1])
        self.assertFalse(row['capture_entry_integral_reset']); self.assertGreater(c.hold_integral[0],0.)
        row['fixed_goal']['position_world_xyz'][0]=99.
        c.update(*sources(900_000_000,position=p),guard_reason='obstacle')
        c.update(*sources(1_000_000_000,position=p))
        _,row=c.update(*sources(1_100_000_000,position=p))
        self.assertEqual(row['fixed_goal']['position_world_xyz'][0],4.)
        self.assertEqual(row['parking_hold_declared_pose_stamp_ns'],800_000_000)
        self.assertTrue(row['parking_window_interrupted'])

    def test_fixed_goal_is_copied_and_wrong_floor_cannot_capture(self):
        goal=[4.,0.,.32]; c=Controller({'external_heading_gate':True},goal,0.,'f')
        c.set_path([[0.,0.,.32],[4.,0.,.32]],'p',0,0); goal[0]=99.
        ready(c,position=(4.,0.,1.32))
        _,row=c.update(*sources(300_000_000,position=(4.,0.,1.32)),mode_override='capture')
        self.assertEqual(row['mode'],'protect'); self.assertIsNone(row['parking_capture_pose_stamp_ns'])
        self.assertEqual(row['fixed_goal']['position_world_xyz'][0],4.)


if __name__=='__main__':
    unittest.main(verbosity=2)
