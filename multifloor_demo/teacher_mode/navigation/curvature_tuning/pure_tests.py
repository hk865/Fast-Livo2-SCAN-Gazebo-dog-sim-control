#!/usr/bin/env python3
"""Pure schedule and geometry negatives; no ROS, Gazebo or model inference."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent


def module(name):
    spec=importlib.util.spec_from_file_location(name,HERE/(name+'.py'))
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


worker=module('plant_worker');evaluator=module('evaluate_radius')
runner=module('run')
protocol=json.loads((HERE/'protocol.json').read_text())
plan={'speed_mps':.5,'yaw_rate_radps':-.8,'timing':protocol['timing']}


class PureTests(unittest.TestCase):
    def test_fixed_schedule_boundaries(self):
        for t,expected in [(0,[0,0,0]),(3,[0,0,0]),(4.5,[.25,0,-.4]),(6,[.5,0,-.8]),(25.999,[.5,0,-.8]),(26,[0,0,0]),(33,[0,0,0])]:
            np.testing.assert_allclose(worker.requested(plan,t)[0],expected,atol=1e-12)
    def test_positive_circle_exact(self):
        a=np.linspace(.2,4.2,4001);p=np.column_stack((2+3*np.cos(a),-4+3*np.sin(a)))
        fit=evaluator.circle_fit(p);self.assertAlmostEqual(fit['radius_m'],3,places=10);self.assertLess(fit['radial_rmse_m'],1e-10);self.assertAlmostEqual(fit['signed_arc_rad'],4,places=10)
    def test_negative_circle_direction(self):
        a=np.linspace(.2,-3.8,4001);p=np.column_stack((3*np.cos(a),3*np.sin(a)))
        self.assertLess(evaluator.circle_fit(p)['signed_arc_rad'],0)
    def test_in_place_not_radius(self):
        with self.assertRaises(ValueError):evaluator.circle_fit(np.ones((100,2)))
    def test_straight_not_finite_circle(self):
        with self.assertRaises(ValueError):evaluator.circle_fit(np.column_stack((np.linspace(0,4,100),np.zeros(100))))
    def test_incomplete_fit_not_pass(self):
        with self.assertRaises(ValueError):evaluator.circle_fit(np.zeros((3,2)))
    def test_pulsed_gait_complete_window(self):
        t=np.linspace(6,26,4001);v=.5+.4*np.sin(2*np.pi*5*t)
        end,mean=evaluator.rolling_mean(t,v,1)
        self.assertEqual(len(end),3801);self.assertGreaterEqual(mean.min(),.5-1e-10);self.assertLess(v.min(),.35)
    def test_intermittent_stop_rolling_fails(self):
        t=np.linspace(6,26,4001);v=np.full_like(t,.5);v[(t>=12)&(t<=14)]=0
        _,mean=evaluator.rolling_mean(t,v,1);self.assertLess(mean.min(),.35)
    def test_rolling_does_not_cross_stop(self):
        t=np.linspace(6,26,4001);end,mean=evaluator.rolling_mean(t,np.full_like(t,.3),1)
        self.assertGreaterEqual(end[0],7);self.assertLessEqual(end[-1],26);np.testing.assert_allclose(mean,.3,atol=1e-12)
    def test_time_backwards_not_pass(self):
        with self.assertRaises(ValueError):evaluator.rolling_mean(np.array([0,.5,.3,1]),np.ones(4),1)
    def test_origin_COM_conversion(self):
        origin=np.array([.3,.04,0]);omega=np.array([0,0,.8]);offset=np.array([.055,-.002,.006])
        com=origin+np.cross(omega,offset);np.testing.assert_allclose(com-np.cross(omega,offset),origin,atol=1e-15)
    def test_parking_zero_requested_and_unchanged_slew(self):
        cmd=np.array([1.,0,.8]);steps=[]
        for _ in range(100):cmd+=np.clip(-cmd,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02);steps.append(cmd.copy())
        self.assertEqual(float(np.max(abs(steps[-1]))),0.)
    def test_curve_gate_values_pre_registered(self):
        g=protocol['gates'];self.assertEqual(g['forward_window_s'],1);self.assertEqual(g['minimum_forward_rolling_mean_fraction'],.7)
        self.assertEqual(protocol['timing']['parking_start_s'],28);self.assertEqual(protocol['timing']['parking_end_s'],33)
        self.assertEqual(g['parking_xy_drift_m'],.05);self.assertEqual(g['parking_yaw_drift_rad'],.1)
    def test_missing_original_evidence_never_passes(self):
        with tempfile.TemporaryDirectory(prefix='pure_missing_',dir=HERE) as temporary:
            result=evaluator.evaluate(Path(temporary))
            self.assertEqual(result['status'],'failed')
            self.assertEqual(result['checks']['complete_original_evidence']['status'],'unverified')
    def test_XML_layout_tails_are_not_robot_mutations(self):
        root=ET.fromstring('<world><model name="go2"><pose>0 0 .4 0 0 0</pose><link name="base"><inertial><mass>7.3</mass></inertial></link></model></world>')
        ET.indent(root);model=root.find('model')
        reparsed=ET.fromstring(ET.tostring(model,encoding='unicode'))
        self.assertNotEqual(model.tail,reparsed.tail)
        self.assertEqual(runner.structural_fingerprint(model),runner.structural_fingerprint(reparsed))
    def test_structural_model_data_changes_remain_detected(self):
        a=ET.fromstring('<model name="go2"><link name="base"><inertial><mass>7.3</mass></inertial><sensor type="camera"><update_rate>2</update_rate></sensor></link><plugin name="Teacher" filename="same.so"/></model>')
        for xpath,value in [('link/inertial/mass','7.4'),('link/sensor/update_rate','10')]:
            b=ET.fromstring(ET.tostring(a));b.find(xpath).text=value
            self.assertNotEqual(runner.structural_fingerprint(a),runner.structural_fingerprint(b))
        b=ET.fromstring(ET.tostring(a));b.find('plugin').set('filename','other.so')
        self.assertNotEqual(runner.structural_fingerprint(a),runner.structural_fingerprint(b))
    def test_prepare_exception_has_fresh_failed_receipt(self):
        with tempfile.TemporaryDirectory(prefix='pure_prepare_failure_',dir=HERE) as temporary:
            root=Path(temporary);(root/'runs').mkdir()
            args=SimpleNamespace(plan=root/'absent.json')
            with mock.patch.object(runner,'ROOT',root),mock.patch.object(runner,'_prepare_into',side_effect=ValueError('pure negative')):
                with self.assertRaisesRegex(ValueError,'pure negative'):runner.prepare(args)
            receipt=json.loads((args.preparation_directory/'prepare_failure.json').read_text())
            self.assertEqual(receipt['status'],'failed');self.assertFalse(receipt['simulation_started'])
            self.assertEqual(receipt['run'],str(args.preparation_directory))


if __name__=='__main__':unittest.main(verbosity=2)
