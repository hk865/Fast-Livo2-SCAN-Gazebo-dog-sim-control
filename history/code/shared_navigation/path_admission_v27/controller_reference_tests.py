"""Exercise the production wrapper method without constructing ROS processes.

The actual cascade supplies reference geometry; only ROS recording and the
legacy guard's return object are fixtures. These are finite software tests,
not evidence of safe terrain or physical tracking.
"""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from cascade_core import Controller
from pure_tests import sources


def wrapper_method(name, extra=None):
    tree=ast.parse((Path(__file__).parent/'controller.py').read_text())
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace=dict(np=np,math=math,copy=copy)
    namespace.update(extra or {})
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),
                 'production_controller_method','exec'),namespace)
    return namespace[name]


class WrapperReferenceChecks(unittest.TestCase):
    def make(self,path=None,goal=(2.,0.,.32),position=(0.,.01,.32)):
        config=dict(desired_speed=.2,external_heading_gate=True,
                    spatial_reference=dict(enabled=True,arc_length_m=.2))
        cascade=Controller(config,list(goal),0.,'goal')
        cascade.set_path(path or [[0.,0.,.32],[0.,.000067,.32],[2.,0.,.32]],'scan',0,0,
                         position=np.asarray(position))
        records=[]
        node=SimpleNamespace(cascade=cascade,pose=np.asarray(position),rotation=np.eye(3),
            request_id='test',waypoint_index=0,pose_stamp=200_000_000,feedback=None,
            run=Path('/unused'),ensure_cascade=lambda:None,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=205_000_000)),
            evidence=SimpleNamespace(append=lambda p,row:records.append(row)))
        legacy=(np.array([.2,0.]),.0,np.array([.5,0.,.32]),
                dict(heading=math.pi/2,error=math.pi/2,direction=[0.,1.],exhausted=False))
        method=wrapper_method('follow_checked_trajectory',
            dict(follow_trajectory=lambda *args,**kwargs:copy.deepcopy(legacy)))
        return cascade,node,records,method,legacy

    def test_heading_uses_same_finite_arc_as_inner_controller(self):
        c,n,records,method,legacy=self.make()
        c.update(*sources(100_000_000,position=n.pose),mode_override='drive')
        progress=c.progress
        result=method(n,return_steering=True)
        self.assertEqual(c.progress,progress)
        _,row=c.update(*sources(200_000_000,position=n.pose),mode_override='drive')
        self.assertAlmostEqual(result[3]['heading'],row['reference_yaw_rad'],places=12)
        self.assertLess(abs(result[3]['heading']),.01)
        self.assertEqual(records[-1]['steering']['cascade_projection']['path_id'],'scan')

    def test_original_guard_target_is_not_replaced_with_unchecked_chord(self):
        c,n,records,method,legacy=self.make()
        result=method(n,return_steering=True)
        np.testing.assert_array_equal(result[2],legacy[2])
        self.assertEqual(result[3]['original_lookahead_steering'],legacy[3])
        self.assertFalse(records[-1]['navigation_ground_truth_used'])

    def test_repeated_previews_do_not_mutate_progress_integrals_or_filters(self):
        c,n,records,method,legacy=self.make()
        c.update(*sources(100_000_000,position=n.pose),mode_override='drive')
        c.update(*sources(200_000_000,position=n.pose),mode_override='drive')
        before=copy.deepcopy(c.__dict__)
        for _ in range(3): method(n,return_steering=True)
        self.assertEqual(c.progress,before['progress'])
        np.testing.assert_array_equal(c.velocity_integral,before['velocity_integral'])
        np.testing.assert_array_equal(c.inner_filtered,before['inner_filtered'])
        self.assertEqual(c.last_math_stamp,before['last_math_stamp'])

    def test_tail_heading_agrees_with_fixed_goal_capture(self):
        path=[[0.,0.,.32],[1.9,.05,.32],[2.,0.,.32]]
        c,n,records,method,legacy=self.make(path=path,position=(1.96,.02,.32))
        result=method(n,return_steering=True)
        self.assertEqual(result[3]['heading'],c.goal_yaw)

    def test_new_path_receives_actual_pose_at_atomic_handoff(self):
        tree=ast.parse((Path(__file__).parent/'controller.py').read_text())
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='ensure_cascade')
        calls=[n for n in ast.walk(method) if isinstance(n,ast.Call) and
               isinstance(n.func,ast.Attribute) and n.func.attr=='set_path']
        self.assertEqual(len(calls),1)
        pose=next(k.value for k in calls[0].keywords if k.arg=='position')
        self.assertEqual(ast.unparse(pose),'self.pose')

    def test_invalid_reference_stops_same_tick_and_records_failure(self):
        records=[]
        def invalid(now):raise ValueError('invalid finite arc')
        node=SimpleNamespace(_control_candidate=invalid,state='running',command=[.2,.1,.1],
            reference_replan_pending=True,run=Path('/unused'),pose_stamp=123,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=456)),
            evidence=SimpleNamespace(append=lambda p,row:records.append(row)))
        node.publish_command=lambda:setattr(node,'command',[0.,0.,0.])
        wrapper_method('control')(node,0.)
        self.assertEqual(node.state,'failed')
        self.assertEqual(node.command,[0.,0.,0.])
        self.assertFalse(node.reference_replan_pending)
        self.assertEqual(records[0]['error_type'],'ValueError')


if __name__=='__main__': unittest.main(verbosity=2)
