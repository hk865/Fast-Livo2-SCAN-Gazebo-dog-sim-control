"""Offline controller safety/regression checks; no Teacher or simulator run."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from core import (Controller,FrozenV1Controller,StaticPath,rotation,V1_DRIVE_SOURCE_SHA256,
                  V1_DRIVE_SOURCE_SIZE_BYTES,parameter_hash,parking_target_record)
from profiles import parking_contract,from_v1


OFFSET=[.05523092034548942,-.001869001919385796,.006095299184261034]


def profile(kind='s_curve',hz=25,heading=0.):
    path=dict(kind=kind,origin_world_xyz=[0.,0.,.32],heading0_rad=heading,
              entry_straight_m=2.,exit_straight_m=2.,direction_sign=1)
    path.update(dict(radius_m=.6,turn_angle_rad=2*math.pi) if kind=='circle' else
                dict(curve_length_m=6.,heading_amplitude_rad=1.))
    geometry=StaticPath(path)
    p=dict(schema='truth_teacher_static_curve_profile/v1',scenario='pure_test_only',terrain='flat',
           path=path,path_parameters_sha256=parameter_hash(path),path_length_m=geometry.length,
           maximum_abs_curvature_1pm=geometry.maximum_abs_curvature,desired_speed=.3,feedback_hz=hz,
           duration_s=180.,post_completion_s=7.,command_limits=[.8,.35,.8],gains={},
           base_com_offset=OFFSET.copy(),spawn=[0.,0.,.4,heading],frozen_source_hashes={})
    p['active_parking']=parking_contract(geometry)
    return p


def quaternion(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return np.array([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])


def state(world_t,position,yaw=0.,world_velocity=(0.,0.,0.),omega=(0.,0.,0.),roll=0.,pitch=0.):
    q=quaternion(roll,pitch,yaw);R=rotation(q);s=np.zeros(64)
    s[0]=world_t;s[1:4]=position;s[4:8]=q
    s[8:11]=R.T@np.asarray(world_velocity)+np.cross(omega,OFFSET);s[11:14]=omega
    s[50:55]=[0.,1.,1.,1.,1.]
    return s


def at_endpoint(controller,offset=(0.,0.),yaw_offset=0.,world_t=3.01):
    position=controller.path.final_xyz.copy();position[:2]+=offset
    return state(world_t,position,controller.path.heading0+yaw_offset)


class ActiveParking(unittest.TestCase):
    def test_frozen_v1_prefix_and_drive_replay_match(self):
        data=Path(__file__).with_name('core.py').read_bytes()
        self.assertEqual(hashlib.sha256(data[:V1_DRIVE_SOURCE_SIZE_BYTES]).hexdigest(),V1_DRIVE_SOURCE_SHA256)
        for kind in ('circle','s_curve'):
            for hz in (10,25,50):
                p=profile(kind,hz);old=FrozenV1Controller(p);new=Controller(p)
                old.progress=new.progress=2.
                for frame in range(121):
                    elapsed=3.+frame*.02;arc=2.+frame*.006
                    point,tangent,_,heading,kappa=new.path.geometry(arc)
                    native=state(elapsed+.01,point,heading,.3*tangent,(0.,0.,.3*kappa))
                    with patch('core.time.monotonic',return_value=100.+frame*.02):
                        a,ma=old.update(native,elapsed);b,mb=new.update(native,elapsed)
                    np.testing.assert_array_equal(a,b);self.assertEqual(ma,mb)
                    np.testing.assert_array_equal(old.velocity_integral,new.velocity_integral)
                    self.assertEqual(old.progress,new.progress)
                self.assertIsNone(new.capture_stamp)

    def test_circle_endpoint_cannot_skip_full_lap(self):
        c=Controller(profile('circle'))
        for frame in range(101):
            elapsed=3.+frame*.02
            with patch('core.time.monotonic',return_value=100.+frame*.02):
                _,mode=c.update(at_endpoint(c,world_t=elapsed+.01),elapsed)
        self.assertIsNone(c.capture_stamp);self.assertIsNone(c.completed_t)
        self.assertEqual(mode,'drive')

    def test_capture_resets_separate_PI_not_fixed_target(self):
        p=profile();c=Controller(p);c.progress=c.path.length-.08;c.velocity_integral[:]=[.2,.1,.05]
        target=c.path.final_xyz.copy()
        with patch('core.time.monotonic',return_value=100.):
            cmd,mode=c.update(at_endpoint(c,(-.08,0.)),3.)
        self.assertEqual(mode,'capture');self.assertTrue(c.row['entry_integral_reset'])
        np.testing.assert_array_equal(c.hold_integral,[0.,0.,0.])
        np.testing.assert_array_equal(c.velocity_integral,[.2,.1,.05])
        np.testing.assert_array_equal(c.row['parking_target_world_xyz'],target)
        self.assertNotEqual(c.row['arrival_capture_position_world'],target.tolist())
        self.assertIsNone(c.completed_t)
        p['active_parking']['target']['position_world_xyz'][0]=99.
        c.row['parking_target_world_xyz'][0]=99.
        with patch('core.time.monotonic',return_value=100.02):
            c.update(at_endpoint(c,(-.08,0.),world_t=3.03),3.02)
        np.testing.assert_array_equal(c.row['parking_target_world_xyz'],target)

    def test_rotated_position_COM_offset_and_Euler_yaw_conversion(self):
        c=Controller(profile(heading=.8));c.progress=c.path.length-.08
        position=c.path.final_xyz.copy();position[:2]-=.08*np.array([math.cos(.8),math.sin(.8)])
        roll,pitch=.13,.18;omega_y=.07;yaw=.78;yawdot=.02
        omega=(.03,omega_y,(yawdot*math.cos(pitch)-math.sin(roll)*omega_y)/math.cos(roll))
        native=state(3.01,position,yaw,(0.,0.,0.),omega,roll,pitch)
        with patch('core.time.monotonic',return_value=100.):c.update(native,3.)
        self.assertEqual(c.row['mode'],'capture')
        np.testing.assert_allclose(c.row['measured_origin_velocity_world'],0.,atol=1e-12)
        self.assertAlmostEqual(c.row['measured_yaw_rate'],yawdot)
        R=rotation(native[4:8]);target=np.asarray(c.row['v_reference_body'])
        expected_w=(c.row['parking_reference_yaw_rate_radps']*math.cos(pitch)-math.sin(roll)*omega_y)/math.cos(roll)
        self.assertAlmostEqual(target[2],expected_w)
        expected_com=R.T@np.asarray(c.row['reference_velocity_world'])+np.cross([0.,0.,expected_w],OFFSET)
        np.testing.assert_allclose(target[:2],expected_com[:2],atol=1e-12)

    def test_capture_fresh_dwell_hold_PI_and_fixed_declared_stamp(self):
        c=Controller(profile());c.progress=c.path.length-.02
        for frame in range(32):
            elapsed=3.+frame*.02;before=c.hold_integral.copy()
            with patch('core.time.monotonic',return_value=100.+frame*.02):
                c.update(at_endpoint(c,world_t=elapsed+.01),elapsed)
            if frame and not c.row['controller_updated']:np.testing.assert_array_equal(c.hold_integral,before)
            if frame:self.assertFalse(c.row['entry_integral_reset'])
        self.assertEqual(c.row['mode'],'active_hold');self.assertAlmostEqual(c.hold_stamp-c.capture_stamp,.6)
        stamp=c.hold_stamp
        with patch('core.time.monotonic',return_value=100.64):
            command,mode=c.update(at_endpoint(c,yaw_offset=.08,world_t=3.65),3.64)
        self.assertEqual(mode,'active_hold');self.assertNotEqual(command[2],0.)
        self.assertEqual(c.hold_stamp,stamp)
        # The abrupt synthetic heading error initially blocks I against the
        # downstream slew. It may integrate only once that command catches up.
        for frame in range(33,43):
            elapsed=3.+frame*.02
            with patch('core.time.monotonic',return_value=100.+frame*.02):
                c.update(at_endpoint(c,yaw_offset=.08,world_t=elapsed+.01),elapsed)
        self.assertEqual(c.hold_stamp,stamp);self.assertGreater(abs(c.hold_integral[2]),0.)

    def test_updated25Hz_holds_do_not_integrate(self):
        c=Controller(profile());c.progress=c.path.length-.08;times=[]
        for frame in range(20):
            elapsed=3.+frame*.02;before=c.hold_integral.copy()
            with patch('core.time.monotonic',return_value=100.+frame*.02):
                c.update(at_endpoint(c,(-.08,0.),world_t=elapsed+.01),elapsed)
            if c.row['controller_updated']:times.append(c.row['feedback_time_s'])
            else:np.testing.assert_array_equal(c.hold_integral,before)
        np.testing.assert_allclose(np.diff(times),.04,atol=1e-12)

    def test_late_fresh_wall_arrival_latches_before_fresh_reset(self):
        c=Controller(profile());c.progress=c.path.length-.02
        with patch('core.time.monotonic',side_effect=[100.,100.02,100.4]):
            c.update(at_endpoint(c),3.);c.update(at_endpoint(c,world_t=3.03),3.02)
            command,mode=c.update(at_endpoint(c,world_t=3.05),3.04)
        self.assertEqual(mode,'feedback_timeout');np.testing.assert_array_equal(command,0.)
        self.assertGreater(c.row['previous_fresh_wall_receipt_gap_s'],.3)
        self.assertEqual(c.last_wall,100.)
        with patch('core.time.monotonic',return_value=100.42):
            command,mode=c.update(at_endpoint(c,world_t=3.07),3.06)
        self.assertEqual(mode,'feedback_timeout');np.testing.assert_array_equal(command,0.)

    def test_fault_after_declared_hold_still_has_TTL(self):
        c=Controller(profile());c.progress=c.path.length-.02
        for frame in range(31):
            elapsed=3.+frame*.02
            with patch('core.time.monotonic',return_value=100.+frame*.02):
                c.update(at_endpoint(c,world_t=elapsed+.01),elapsed)
        self.assertIsNotNone(c.completed_t)
        with patch('core.time.monotonic',return_value=100.62):
            command,mode=c.update(at_endpoint(c,world_t=4.0),3.99)
        self.assertEqual(mode,'feedback_timeout');np.testing.assert_array_equal(command,0.)
        with self.assertRaises(ValueError):c.update(at_endpoint(c,world_t=4.0),4.)
        with self.assertRaises(ValueError):c.update(np.full(64,np.nan),4.)

    def test_hold_command_limit_antiwindup_and_slew_pressure(self):
        c=Controller(profile());c.progress=c.path.length-.02
        with patch('core.time.monotonic',return_value=100.):c.update(at_endpoint(c),3.)
        previous=c.applied_command_estimate.copy();blocked=False
        for frame in range(1,41):
            elapsed=3.+frame*.02
            native=state(elapsed+.01,c.path.final_xyz,world_velocity=(-1.,1.,0.))
            with patch('core.time.monotonic',return_value=100.+frame*.02):command,_=c.update(native,elapsed)
            self.assertTrue(np.all(abs(command)<=c.capture_limits+1e-12))
            self.assertLessEqual(np.linalg.norm(command[:2]),.15+1e-12)
            self.assertTrue(np.all(abs(c.applied_command_estimate-previous)<=np.array([.6,.6,.8])*.02+1e-12))
            previous=c.applied_command_estimate.copy()
            if c.row['controller_updated']:blocked=blocked or any(c.row['velocity_PI']['blocked_axis'])
        self.assertTrue(blocked);np.testing.assert_array_equal(c.hold_integral,0.)

    def test_cross_numpy_roundoff_keeps_exact_prospective_target(self):
        p=profile();p['active_parking']['target']['position_world_xyz'][0]+=2e-15
        p['active_parking']['target_sha256']=parameter_hash(p['active_parking']['target'])
        c=Controller(p)
        self.assertEqual(c._target,p['active_parking']['target'])
        self.assertEqual(c._target_sha,p['active_parking']['target_sha256'])
        p=profile();p['active_parking']['target']['position_world_xyz'][0]+=2e-12
        p['active_parking']['target_sha256']=parameter_hash(p['active_parking']['target'])
        with self.assertRaises(ValueError):Controller(p)

    def test_capture_approach_and_hold_envelopes_are_distinct(self):
        c=Controller(profile());c.progress=c.path.length-.08
        with patch('core.time.monotonic',return_value=100.):c.update(at_endpoint(c,(-.08,0.)),3.)
        self.assertEqual(c.row['mode'],'capture')
        np.testing.assert_array_equal(c.row['parking_command_limits_body'],[.15,.07,.10])
        self.assertAlmostEqual(np.linalg.norm(c.row['reference_velocity_world'][:2]),.05)
        self.assertGreater(c.row['command_body'][0],.06)
        for frame in range(1,34):
            elapsed=3.+frame*.02
            with patch('core.time.monotonic',return_value=100.+frame*.02):c.update(at_endpoint(c,world_t=elapsed+.01),elapsed)
        self.assertEqual(c.row['mode'],'active_hold')
        np.testing.assert_array_equal(c.row['parking_command_limits_body'],[.06,.04,.10])
        self.assertLessEqual(np.linalg.norm(c.row['reference_velocity_world'][:2]),.025)
        self.assertLessEqual(np.linalg.norm(c.row['command_body'][:2]),.06)
        p=profile();p['active_parking']['capture_command_limits_body'][0]=.151
        with self.assertRaises(ValueError):Controller(p)

    def test_invalid_profile_target_or_envelope_rejected(self):
        p=profile();p['active_parking']['target']['position_world_xyz'][0]=99.
        with self.assertRaises(ValueError):Controller(p)
        p=profile();p['active_parking']['command_limits_body'][0]=.061
        with self.assertRaises(ValueError):Controller(p)
        p=profile();p['path']['origin_world_xyz'][0]=1.
        with self.assertRaises(ValueError):Controller(p)

    def test_profile_hash_and_only_parking_derivation(self):
        old=profile();old.pop('active_parking')
        with tempfile.TemporaryDirectory(prefix='active_parking_offline_') as name:
            source=Path(name)/'v1.json';source.write_text(json.dumps(old))
            digest=hashlib.sha256(source.read_bytes()).hexdigest();new=from_v1(source,digest)
            for key in ('path','desired_speed','feedback_hz','gains','base_com_offset','command_limits','spawn'):
                self.assertEqual(old[key],new[key])
            self.assertEqual(new['active_parking']['target_sha256'],parameter_hash(parking_target_record(StaticPath(old['path']))))
            with self.assertRaises(ValueError):from_v1(source,'0'*64)


if __name__=='__main__':unittest.main(verbosity=2)
