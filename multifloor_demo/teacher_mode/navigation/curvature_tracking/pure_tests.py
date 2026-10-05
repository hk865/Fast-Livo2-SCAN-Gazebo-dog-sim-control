"""Meaningful CPU geometry/controller checks; no simulated plant or pass claim."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from core import Controller, StaticPath, rotation, wrap
from profiles import (circle_profile,s_curve_profile,MODEL_SHA,REQUIRED_PLANT_CHECKS,
                      REQUIRED_PLANT_INPUTS,APPROVED_PLANT_PROTOCOL_SHA,APPROVED_PLANT_ANALYZER_SHA)


def quaternion(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return np.array([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy])


def state(world_t,position,yaw=0.,origin_world_velocity=(0.,0.,0.),
          body_omega=(0.,0.,0.),roll=0.,pitch=0.):
    p=circle_profile(2.)
    q=quaternion(roll,pitch,yaw);R=rotation(q)
    result=np.zeros(64);result[0]=world_t;result[1:4]=position;result[4:8]=q
    result[8:11]=R.T@np.asarray(origin_world_velocity)+np.cross(body_omega,p['base_com_offset'])
    result[11:14]=body_omega;result[50:55]=[0.,1.,1.,1.,1.]
    return result


class Geometry(unittest.TestCase):
    def test_circle_tangent_and_signed_curvature_both_directions(self):
        for sign in [-1,1]:
            p=StaticPath(circle_profile(2.,direction_sign=sign)['path'])
            s=p.entry+math.pi*p.radius/2
            point,tangent,normal,heading,kappa=p.geometry(s)
            np.testing.assert_allclose(point,[4.,2.*sign,.32],atol=1e-12)
            self.assertAlmostEqual(kappa,sign/2)
            self.assertAlmostEqual(wrap(heading),sign*math.pi/2)
            np.testing.assert_allclose(tangent,[0.,sign,0.],atol=1e-12)
            np.testing.assert_allclose(p.position(p.entry+p.curve_length),[2.,0.,.32],atol=1e-12)
            np.testing.assert_allclose(p.final_xyz,[4.,0.,.32],atol=1e-12)

    def test_s_true_arclength_derivative_curvature_and_tangent_endpoints(self):
        for sign in [-1,1]:
            p=StaticPath(s_curve_profile(8.,.5,direction_sign=sign)['path'])
            for u in np.linspace(.1,7.9,41):
                s=p.entry+u;epsilon=1e-5
                finite=(p.position(s+epsilon)-p.position(s-epsilon))/(2*epsilon)
                _,tangent,normal,heading,kappa=p.geometry(s)
                np.testing.assert_allclose(finite,tangent,atol=2e-9)
                hleft=p.geometry(s-epsilon)[3];hright=p.geometry(s+epsilon)[3]
                self.assertAlmostEqual((hright-hleft)/(2*epsilon),kappa,places=8)
                self.assertAlmostEqual(float(tangent@normal),0.,places=12)
            self.assertAlmostEqual(p.position(p.entry+p.curve_length)[1],0.,places=12)
            self.assertAlmostEqual(p.geometry(p.entry)[4],0.)
            self.assertAlmostEqual(p.geometry(p.entry+p.curve_length)[4],0.)
            self.assertAlmostEqual(p.geometry(p.length)[3],0.)

    def test_projection_lap_cannot_skip_at_circle_overlap(self):
        p=StaticPath(circle_profile(1.)['path']);progress=0.
        for s in np.linspace(0,p.length,401):
            value,details=p.project(p.position(s),progress)
            self.assertGreaterEqual(value,progress)
            self.assertAlmostEqual(value,s,places=7)
            self.assertLessEqual(value-progress,p.forward_window+1e-9)
            progress=value
        progress,details=p.project(p.position(p.entry),p.entry)
        self.assertLess(progress,p.entry+.1)
        self.assertLess(progress,p.length-.15)

    def test_path_hash_and_immutable_input(self):
        profile=circle_profile(2.);controller=Controller(profile)
        original=controller.path.final_xyz.copy()
        profile['path']['radius_m']=99.
        np.testing.assert_array_equal(controller.path.final_xyz,original)
        bad=circle_profile(2.);bad['path']['radius_m']=3.
        with self.assertRaises(ValueError):Controller(bad)

    def test_pending_radius_and_real_evidence_margin(self):
        p=circle_profile(2.)
        self.assertEqual(p['feasibility']['status'],'pending_actual_minimum_radius')
        with tempfile.TemporaryDirectory(prefix='curve_source_pure_test_') as name:
            # Synthetic receipt fixture tests hash rejection; never saved as a
            # real experiment and never used for a tracking feasibility claim.
            run=Path(name)
            archived=run/'sources/curvature';archived.mkdir(parents=True)
            original=Path(__file__).resolve().parent.parent/'curvature_tuning'
            manifest={}
            for n in ['protocol.json','evaluate_radius.py']:
                dst=archived/n;dst.write_bytes((original/n).read_bytes())
                manifest['curvature/'+n]={'path':str(dst),'sha256':hashlib.sha256(dst.read_bytes()).hexdigest()}
            for n in REQUIRED_PLANT_INPUTS:
                if n!='protocol.json':(run/n).write_text('{}')
            (run/'source_manifest.json').write_text(json.dumps(manifest))
            plan=run/'curvature_plan.json'
            plan.write_text(json.dumps({'speed_mps':.3,'yaw_rate_radps':.2,'model_sha256':MODEL_SHA,
                                        'protocol_sha256':APPROVED_PLANT_PROTOCOL_SHA}))
            receipt=run/'summary_radius_independent.json'
            receipt.write_text(json.dumps({'schema':'teacher_continuous_turn_plant_independent/v1',
                'status':'passed','run':str(run),'checks':{n:{'status':'passed'} for n in REQUIRED_PLANT_CHECKS},
                'analyzer_sha256':APPROVED_PLANT_ANALYZER_SHA,
                'input_hashes':{n:hashlib.sha256((archived/n if n=='protocol.json' else run/n).read_bytes()).hexdigest()
                                for n in REQUIRED_PLANT_INPUTS},
                'metrics':{'actual_fitted_radius_m':1.}}))
            evidence=dict(status='passed',desired_speed_mps=.3,direction_sign=1,
                          minimum_of_passed_tested_actual_radii_m=1.,source_path=str(receipt),
                          source_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
            with self.assertRaises(ValueError):circle_profile(1.4,minimum_radius_evidence=evidence)
            p=circle_profile(1.5,minimum_radius_evidence=evidence)
            self.assertFalse(p['feasibility']['circle_radius_is_tested_feasible'])
            wrong=copy.deepcopy(evidence);wrong['source_sha256']='0'*64
            with self.assertRaises(ValueError):circle_profile(1.5,minimum_radius_evidence=wrong)
            incomplete=json.loads(receipt.read_text());incomplete['checks'].pop(REQUIRED_PLANT_CHECKS[0])
            old_receipt=receipt.read_bytes();receipt.write_text(json.dumps(incomplete))
            wrong=copy.deepcopy(evidence);wrong['source_sha256']=hashlib.sha256(receipt.read_bytes()).hexdigest()
            with self.assertRaises(ValueError):circle_profile(1.5,minimum_radius_evidence=wrong)
            receipt.write_bytes(old_receipt)
            plan.write_text('{}')
            with self.assertRaises(ValueError):circle_profile(1.5,minimum_radius_evidence=evidence)


class ControllerBehavior(unittest.TestCase):
    def test_actual_update_cadence_and_held_PI(self):
        for hz in [10,25,50]:
            c=Controller(circle_profile(2.,hz=hz));times=[];oldI=None
            for frame in range(251):
                t=frame*.02
                cmd,mode=c.update(state(t+.01,[0.,0.,.32]),t)
                self.assertLessEqual(np.linalg.norm(cmd[:2]),.8+1e-12)
                if c.row['controller_updated']:
                    times.append(c.row['feedback_time_s'])
                    oldI=c.row['velocity_PI']['I']
                elif c.row.get('velocity_PI'):
                    self.assertEqual(c.row['velocity_PI']['I'],oldI)
            np.testing.assert_allclose(np.diff(times),1/hz,atol=1e-12)

    def test_curvature_yaw_feedforward_and_exact_body_conversion(self):
        p=circle_profile(2.)
        c=Controller(p);c.progress=c.path.entry+1.
        point,tangent,normal,heading,kappa=c.path.geometry(c.progress)
        roll=.13;pitch=.18;omega_y=.07
        desired_yawdot=.3*kappa
        body_wz=(desired_yawdot*math.cos(pitch)-math.sin(roll)*omega_y)/math.cos(roll)
        native=state(3.01,point,heading,.3*tangent,(0.,omega_y,body_wz),roll,pitch)
        command,mode=c.update(native,3.)
        self.assertEqual(mode,'drive')
        self.assertAlmostEqual(c.row['yaw_feedforward_radps'],desired_yawdot,places=10)
        self.assertAlmostEqual(c.row['measured_yaw_rate'],desired_yawdot,places=10)
        self.assertAlmostEqual(c.row['v_reference_body'][2],body_wz,places=9)
        self.assertAlmostEqual(c.row['yaw_PD']['D'],0.,places=9)
        self.assertNotEqual(c.row['v_reference_body'][2],desired_yawdot)
        np.testing.assert_allclose(c.row['measured_origin_velocity_world'],.3*tangent,atol=1e-12)

    def test_endpoint_cannot_complete_without_full_path_progress(self):
        c=Controller(circle_profile(2.))
        endpoint=c.path.final_xyz
        for i in range(101):
            _,mode=c.update(state(3.01+.02*i,endpoint),3.+.02*i)
        self.assertIsNone(c.completed_t)
        self.assertNotIn(mode,('goal_dwell','parking'))

    def test_final_goal_zero_dwell_freezes_I_and_parks(self):
        c=Controller(circle_profile(2.));c.progress=c.path.length-.07
        c.velocity_integral[:]=[.11,-.02,.01]
        before=c.velocity_integral.copy()
        for i in range(101):
            cmd,mode=c.update(state(3.01+.02*i,c.path.final_xyz),3.+.02*i)
            np.testing.assert_array_equal(cmd,np.zeros(3))
            np.testing.assert_array_equal(c.velocity_integral,before)
        self.assertIsNotNone(c.completed_t);self.assertEqual(mode,'parking')
        self.assertFalse(c.row['controller_updated'])

    def test_timeout_latches_zero_and_clock_fault_rejects(self):
        c=Controller(circle_profile(2.))
        c.update(state(3.01,[0.,0.,.32]),3.)
        cmd,mode=c.update(state(3.5,[0.,0.,.32]),3.49)
        self.assertEqual(mode,'feedback_timeout');np.testing.assert_array_equal(cmd,0.)
        cmd,mode=c.update(state(3.52,[0.,0.,.32]),3.51)
        self.assertEqual(mode,'feedback_timeout');np.testing.assert_array_equal(cmd,0.)
        with self.assertRaises(ValueError):c.update(state(3.52,[0.,0.,.32]),3.51)
        with self.assertRaises(ValueError):c.update(np.full(64,np.nan),4.)

    def test_late_fresh_wall_receipt_cannot_reset_timeout(self):
        c=Controller(circle_profile(2.,hz=25))
        with patch('core.time.monotonic',side_effect=[10.,10.02,10.4]):
            c.update(state(3.01,[0.,0.,.32]),3.)
            c.update(state(3.03,[0.,0.,.32]),3.02)
            command,mode=c.update(state(3.05,[0.,0.,.32]),3.04)
        self.assertEqual(mode,'feedback_timeout')
        self.assertGreater(c.row['previous_fresh_wall_receipt_gap_s'],.3)
        np.testing.assert_array_equal(command,0.)


if __name__=='__main__':
    unittest.main(verbosity=2)
