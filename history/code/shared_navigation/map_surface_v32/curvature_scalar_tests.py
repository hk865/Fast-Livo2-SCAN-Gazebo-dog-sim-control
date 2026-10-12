"""V21 scalar-boundary regressions, including the frozen V20 failure.

No ROS, Gazebo or raw-run replay. Old source is loaded by exact frozen file SHA.
The production Controller uses the actual frozen profile's numerical settings.
"""
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'corridor_tracking_v20'
from cascade_core import Controller
from spatial_reference import configuration,speed_budget
from pure_tests import sources
from spatial_reference_tests import circle
from controller_reference_tests import wrapper_method
from pid_core import slew

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
def old_modules():
    pins={'spatial_reference.py':'b358b03997759c0f7066f51c731370437998ee1fb953cd8c7bd1722b29739469',
          'cascade_core.py':'3c1a3cb00993fe213b622b0df2bdaf511cb3fcbbd2728f0fc998a87c155f4e82'}
    for name,pin in pins.items():
        if hashlib.sha256((OLD/name).read_bytes()).hexdigest()!=pin:raise AssertionError('Frozen V20 baseline source changed')
    spatial=module('scalar_regression_old_spatial',OLD/'spatial_reference.py')
    core=module('scalar_regression_old_core',OLD/'cascade_core.py')
    core.speed_budget=spatial.speed_budget;core.spatial_configuration=spatial.configuration;core.spatial_direction=spatial.reference
    return spatial,core
def config():
    value=json.loads((OLD/'profiles/pipeline_staged_original46.json').read_text())['cascade']
    value['spatial_reference'].update(curvature_feedforward_enabled=True,curvature_speed_limit_enabled=True)
    return value
def prepared(cls=Controller):
    c=cls(config(),[3.,3.,.32],0.,'g');p=circle(radius=.3);c.set_path(p,'left',0,0)
    h=c.preview_reference(p[0])['heading_rad'];q=[math.cos(h/2),0.,0.,math.sin(h/2)]
    serialized=[]
    for i in range(1,20):
        cmd,row=c.update(*sources(i*100_000_000,quaternion=q),mode_override='drive')
        serialized.append(row)
    p[:,1]*=-1;c.set_path(p,'right',1_900_000_000,11_900_000_000,position=[0.,0.,.32])
    return c,q,serialized
def reversal(cls=Controller):
    c,q,rows=prepared(cls);before=c.velocity_integral.copy()
    cmd,row=c.update(*sources(1_930_000_000,quaternion=q,velocity=(.1,.05,0.)),mode_override='drive')
    return c,cmd,row,c.velocity_integral-before,rows
def append_method():
    path=HERE.parent/'runtime_io.py';tree=ast.parse(path.read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef)and x.name=='EvidenceWriter')
    fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef)and x.name=='append')
    ns=dict(json=json,Path=Path)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(path),'exec'),ns)
    return ns['append']

class CurvatureScalarChecks(unittest.TestCase):
    def test_frozen_v20_reproduces_numpy_false_identity_and_json_failure(self):
        old,_=old_modules()
        v,row=old.speed_budget(*map(np.float64,(.3,-1.,0.,.6,0.,.3,.033)),configuration(),previous_yaw_reference=np.float64(.3))
        self.assertEqual(v,0.);self.assertIsInstance(row['reference_budget_feasible'],np.bool_)
        self.assertFalse(bool(row['reference_budget_feasible']));self.assertFalse(row['reference_budget_feasible']is False)
        with self.assertRaisesRegex(TypeError,'bool_'):json.dumps(row,allow_nan=False)

    def test_numpy_scalar_inputs_return_only_native_json_scalar_values(self):
        for scalar in (float,np.float64,np.float32,np.int64):
            cfg={k:(scalar(v)if type(v)is float else v)for k,v in configuration().items()}
            # Keep fractional numerical constants while separately exercising integer inputs.
            if scalar is np.int64:cfg=configuration()
            args=tuple(map(scalar,(1.,-1.,0.,1.,0.,1.,1.)))if scalar is np.int64 else tuple(map(scalar,(.3,-1.,0.,.6,0.,.3,.033)))
            v,row=speed_budget(*args,cfg,previous_yaw_reference=scalar(.3)if scalar is not np.int64 else scalar(1))
            self.assertIs(type(v),float)
            for key,value in row.items():self.assertIn(type(value),(float,bool,type(None)),key)
            json.dumps(dict(speed=v,budget=row),allow_nan=False)
        _,row=speed_budget(*map(np.float64,(.3,-1.,0.,.6,0.,.3,.033)),configuration(),previous_yaw_reference=np.float64(.3))
        self.assertIs(row['reference_budget_feasible'],False)

    def test_direct_budget_math_equivalent_to_old_float_and_numpy_truth_values(self):
        old,_=old_modules();count=0
        for k in (-3.,-1.,0.,.2,2.):
            for dk in (0.,.4,8.):
                for prev in (0.,.05,.3):
                    for previous_yaw in (-.3,0.,.3):
                        for scalar in (float,np.float64):
                            args=tuple(map(scalar,(.3,k,dk,.6,.08,prev,.033)));oldv,oldr=old.speed_budget(*args,configuration(),previous_yaw_reference=scalar(previous_yaw))
                            v,r=speed_budget(*args,configuration(),previous_yaw_reference=scalar(previous_yaw))
                            self.assertAlmostEqual(v,float(oldv),places=12);self.assertEqual(bool(oldr['reference_budget_feasible']),r['reference_budget_feasible'])
                            for key in r:
                                if type(r[key])is bool:self.assertEqual(r[key],bool(oldr[key]))
                                elif r[key]is None:self.assertIsNone(oldr[key])
                                else:self.assertAlmostEqual(r[key],float(oldr[key]),places=12)
                            count+=1
        self.assertEqual(count,270)

    def test_production_profile_old_misses_hold_new_exact_zero_freezes_pi(self):
        _,old=old_modules();_,oldcmd,oldrow,olddelta,_=reversal(old.Controller)
        self.assertEqual(oldrow['mode'],'drive');self.assertFalse(bool(oldrow['curvature_speed_supervisor']['reference_budget_feasible']))
        self.assertGreater(float(np.linalg.norm(oldcmd)),.01);self.assertGreater(float(np.linalg.norm(olddelta[:2])),0.)
        _,cmd,row,delta,prior=reversal()
        self.assertEqual(row['mode'],'reference_constraint_hold');self.assertIs(row['curvature_speed_supervisor']['reference_budget_feasible'],False)
        np.testing.assert_array_equal(cmd,[0.,0.,0.]);np.testing.assert_array_equal(delta,[0.,0.,0.]);self.assertEqual(row['integral_dt_s'],0.)
        for event in [*prior,row]:json.dumps(event,allow_nan=False)

    def test_actual_evidence_append_accepts_new_row_and_rejects_old(self):
        _,old=old_modules();_,_,bad,_,_=reversal(old.Controller);_,_,good,_,_=reversal()
        pending=[];node=SimpleNamespace(enqueue=pending.append);append=append_method()
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp)/'evidence.jsonl'
            with self.assertRaisesRegex(TypeError,'bool_'):append(node,path,bad)
            self.assertEqual(len(pending),0)
            append(node,path,good);self.assertEqual(len(pending),1);pending[0]()
            saved=json.loads(path.read_text());self.assertEqual(saved['mode'],'reference_constraint_hold')
            self.assertIs(saved['curvature_speed_supervisor']['reference_budget_feasible'],False)

    def test_wrapper_production_selection_stops_numpy_reversal_before_slew(self):
        c,q,_=prepared();feedback,imu,ack,clock,wall=sources(1_930_000_000,quaternion=q,velocity=(.1,.05,0.))
        node=SimpleNamespace(cascade=c,feedback=feedback,paired_imu=imu,ensure_cascade=lambda:None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock)),get_ack=lambda:ack,
            obstacle_hold=False,heading_gate=SimpleNamespace(phase='drive',heading=None),steering={'direction':[1.,0.]},
            state='running',command=np.array([.2,0.,.3]),last_command_time=1.9,rotation=np.eye(3),
            pid_records=0,pose_stamp=1_930_000_000,path_receipt=None,pose=np.array([0.,0.,.32]),pose_quaternion=q,
            request_id='r',waypoint_index=0,active_trajectory_id=2,trajectory_archive_reference='test.npz',waypoints=np.array([[3.,3.,.32]]),
            control_clock_hold=SimpleNamespace(guard_required=False))
        method=wrapper_method('select_pid_velocity',dict(time=SimpleNamespace(monotonic_ns=lambda:wall),slew=slew))
        xy,w=method(node,np.array([.3,0.]),0.,np.array([1.,0.,.32]),True)
        np.testing.assert_array_equal(xy,[0.,0.]);self.assertEqual(w,0.);self.assertTrue(node.reference_replan_pending)
        self.assertEqual(node.pid_row['mode'],'reference_constraint_hold');json.dumps(node.pid_row,allow_nan=False)

    def test_optional_previous_values_keep_json_contract(self):
        for speed in (np.float64(0.),np.float64(.2)):
            value,row=speed_budget(speed,np.float64(0.),np.float64(0.),np.float64(.6),np.float64(0.),None,np.float64(0.),configuration())
            self.assertIs(type(value),float);self.assertIs(row['reference_acceleration_checked'],False)
            self.assertIs(row['temporal_yaw_slew_checked'],False);self.assertIsNone(row['previous_curvature_yaw_reference_radps'])
            json.dumps(row,allow_nan=False)

if __name__=='__main__':unittest.main(verbosity=2)
