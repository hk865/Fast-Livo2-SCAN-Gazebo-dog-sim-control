#!/usr/bin/env python3
"""Use actual production callback AST, no Node shim or simulator process."""
import ast,copy,hashlib,json,math,os,sys,unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
NAV=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(NAV))
import goal_regions as goal_module
from goal_regions import parse_request,definitions_sha256,contains,ArrivalWindow
from control_core import follow_trajectory,steering_obstacle_ahead,HeadingGate

source=Path(os.environ.get('DEMO_TEST_CONTROLLER_SOURCE',str(NAV/'controller.py')));tree=ast.parse(source.read_text());klass=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Navigation')
names=('on_request','on_stop','reset_region_arrival','measured_region_arrival','control')
methods=[n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name in names]
scope=dict(np=np,json=json,copy=copy,math=math,replace=replace,parse_goal_request=parse_request,
           definitions_sha256=definitions_sha256,goal_contains=contains,ArrivalWindow=ArrivalWindow,
           follow_trajectory=follow_trajectory,steering_obstacle_ahead=steering_obstacle_ahead,
           goal_contains_control=getattr(goal_module,'contains_control',contains))
compiled=ast.ClassDef(name='ActualCallbacks',bases=[],keywords=[],body=methods,decorator_list=[])
exec(compile(ast.fix_missing_locations(ast.Module(body=[compiled],type_ignores=[])),str(source),'exec'),scope)

class Callbacks(scope['ActualCallbacks']):
    def __init__(self):
        self.arrival_radius=.22;self.request_id=None;self.state='idle';self.status_count=0
        self.raw_imu=SimpleNamespace(tilt=0.,max_tilt=0.)
        self.heading_gate=SimpleNamespace(reset=lambda:None)
        self.trajectory_association=SimpleNamespace(reset=lambda:None)
        self.obstacle_hold=self.tilt_hold=False;self.bridge_safety={'state':'ready'}
        self.region_raw_pose=np.array([0.,0.,0.]);self.pose=np.array([100.,100.,100.]);self.pose_stamp=1000000000
    def publish_status(self):self.status_count+=1
    def publish_command(self):self.zero_called=True

def request(radius=.35):
    return dict(request_id='region',frame_id='camera_init',schema_version=2,
        goals=[dict(goal_id='flat',center=[0,0,0],arrival=dict(type='disc_prism',radius_m=radius,height_half_span_m=.1,dwell_sim_s=.4))])

class MethodsTests(unittest.TestCase):
    def test_actual_request_freezes_center_and_goal_hash(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())))
        self.assertEqual(n.state,'running');self.assertEqual(n.waypoints.tolist(),[[0,0,0]])
        self.assertEqual(n.goals_definition_sha256,definitions_sha256(n.goals))
    def test_same_id_changed_region_is_rejected_without_touching_running_state(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())));window=n.region_arrival
        n.samples=object();old=n.samples;h=n.goals_definition_sha256
        n.on_request(SimpleNamespace(data=json.dumps(request(.4))))
        self.assertIs(n.samples,old);self.assertIs(n.region_arrival,window)
        self.assertEqual(n.goals_definition_sha256,h);self.assertEqual(n.state,'running');self.assertIsNotNone(n.last_rejected)
    def test_idempotent_same_hash_preserves_arrival_streak(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())));window=n.region_arrival
        n.measured_region_arrival();start=window.since
        n.on_request(SimpleNamespace(data=json.dumps(request())))
        self.assertIs(n.region_arrival,window);self.assertEqual(window.since,start)
    def test_legacy_request_uses_legacy_window_and_custom_radius(self):
        n=Callbacks();n.arrival_radius=.17;n.on_request(SimpleNamespace(data=json.dumps(dict(request_id='legacy',frame_id='camera_init',waypoints=[[0,0,0]]))))
        self.assertIsNone(n.region_arrival);self.assertTrue(n.goals[0].legacy);self.assertEqual(n.goals[0].radius,.17)
    def test_arrival_uses_accepted_raw_position_not_mutable_or_filtered_pose(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())))
        for t in (1000000000,1100000000,1200000000,1300000000):n.pose_stamp=t;inside,arrived=n.measured_region_arrival();self.assertTrue(inside);self.assertFalse(arrived)
        n.pose_stamp=1400000000;self.assertEqual(n.measured_region_arrival(),(True,True))
        d=n.region_arrival_evidence;self.assertEqual(d['raw_position'],[0.,0.,0.]);self.assertEqual(d['stamp_ns']-d['start_stamp_ns'],400000000)
    def test_duplicate_stamps_never_finish_via_ros_timer(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())))
        for _ in range(100):self.assertEqual(n.measured_region_arrival(),(True,False))
        self.assertEqual(n.region_arrival_evidence['dwell_ns'],0)
    def test_actual_obstacle_and_bridge_hold_reset_window(self):
        for mode in ('obstacle','bridge'):
            n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())));n.measured_region_arrival()
            if mode=='obstacle':n.obstacle_hold=True
            else:n.bridge_safety={'state':'hold'}
            n.pose_stamp=1400000000;self.assertEqual(n.measured_region_arrival(),(False,False));self.assertIsNone(n.region_arrival.since)
    def test_actual_stop_clears_dwell_and_zeroes_command(self):
        n=Callbacks();n.on_request(SimpleNamespace(data=json.dumps(request())));n.measured_region_arrival()
        n.on_stop(SimpleNamespace(data=True));self.assertEqual(n.state,'stopped');self.assertIsNone(n.region_arrival.since)
        self.assertIsNone(n.region_arrival_evidence['start_stamp_ns']);self.assertTrue(n.zero_called)

class ControlCallbacks(Callbacks):
    def __init__(self):
        super().__init__();self.on_request(SimpleNamespace(data=json.dumps(request())))
        self.pose=np.array([.1,0.,0.]);self.region_raw_pose=self.pose.copy()
        self.rotation=np.eye(3);self.tracking_pose=self.pose.copy();self.max_speed=.12
        self.segment_start=np.array([-1.,0.,0.]);self.segment_started_ros=10.
        self.samples=np.array([[-1.,0.,0.],[0.,0.,0.]])
        self.cloud=np.array([[.5,0.,.1],[.6,.1,.1],[.6,-.1,.1]])
        self.pose_updated=self.cloud_updated=0.;self.pose_timeout=self.cloud_timeout=1000.
        self.raw_imu.fresh=lambda *args:True
        self.heading_gate=HeadingGate();self.heading_gate.phase='drive'
        self.obstacle_hold=True;self.obstacle_hold_started_ros=9.
        self.last_obstacle_check=-math.inf;self.obstacle_result=(False,None,0)
        self.last_reference=0.;self.zeros=[];self.requests=0;self.ros=10.
        self.pending_obstacle_event=None;self.obstacle_guard_context=None
        self.obstacle_resume_pending=False;self.obstacle_stopped_duration=0.
        self.active_trajectory_id=1
    def apply_tilt_guard(self,*args):return False
    def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(self.ros*1e9)))
    def publish_command(self,velocity=None,yaw_rate=0.):
        self.zeros.append([0.,0.,0.] if velocity is None else list(velocity)+[yaw_rate])
    def request_plan(self):self.requests+=1
    def tick_at(self,t):
        self.ros=t;self.pose_stamp=round(t*1e9);self.control(t)

class HeldRegionControlTests(unittest.TestCase):
    def test_real_cloud_still_blocks_inside_region_at_checked_endpoint(self):
        n=ControlCallbacks();n.tick_at(10.)
        self.assertTrue(n.obstacle_hold);self.assertEqual(n.obstacle_result[2],3)
        self.assertIsNone(n.obstacle_clear_since);self.assertEqual(n.waypoint_index,0)
        self.assertFalse(n.region_arrivals);self.assertEqual(n.zeros,[[0.,0.,0.]])
    def test_endpoint_hold_rechecks_clear_cloud_waits_and_requests_fresh_scan(self):
        n=ControlCallbacks();n.cloud=np.array([[5.,5.,-.5]])
        n.tick_at(10.);self.assertTrue(n.obstacle_hold)
        n.tick_at(10.5);self.assertTrue(n.obstacle_hold)
        n.tick_at(11.1);self.assertFalse(n.obstacle_hold)
        self.assertEqual(n.obstacle_resumes,1);self.assertIsNone(n.samples)
        self.assertEqual(n.requests,1);self.assertEqual(n.waypoint_index,0)
        self.assertTrue(all(c==[0.,0.,0.] for c in n.zeros))
        # After actual clearance only new raw odom can begin its .4 s dwell.
        for t in (11.2,11.3,11.4,11.5,11.6):n.tick_at(t)
        self.assertEqual(n.state,'succeeded');self.assertEqual(len(n.region_arrivals),1)
        self.assertEqual(n.region_arrivals[0]['start_stamp_ns'],11200000000)
        self.assertEqual(n.region_arrivals[0]['dwell_ns'],400000000)
        self.assertTrue(all(c==[0.,0.,0.] for c in n.zeros))
    def test_inside_held_timeout_remains_exact_and_cannot_arrive(self):
        n=ControlCallbacks();n.tick_at(100.1)
        self.assertEqual(n.state,'failed');self.assertEqual(n.waypoint_index,0)
        self.assertFalse(n.region_arrivals);self.assertEqual(n.requests,0)
        self.assertEqual(n.zeros,[[0.,0.,0.]])
    def test_unheld_exhausted_path_never_drives_beyond_checked_path(self):
        n=ControlCallbacks();n.obstacle_hold=False
        n.pose=n.region_raw_pose=n.tracking_pose=np.array([.5,0.,0.]);n.tick_at(10.)
        self.assertIsNone(n.samples);self.assertEqual(n.requests,1)
        self.assertFalse(n.obstacle_hold);self.assertEqual(n.waypoint_index,0)
        self.assertEqual(n.zeros,[[0.,0.,0.]])
    def test_v2_inside_fresh_slow_odom_cannot_wait_beyond_declared_timeout(self):
        n=ControlCallbacks();n.obstacle_hold=False
        # .3 s is below original .8 s pose freshness but exceeds new .2 s
        # region observation continuity. It must not fake dwell or hang.
        for i in range(301):n.tick_at(10.+.3*i)
        self.assertEqual(n.state,'running');self.assertFalse(n.region_arrivals)
        n.tick_at(100.3)
        self.assertEqual(n.state,'failed');self.assertFalse(n.region_arrivals)
        self.assertEqual(n.waypoint_index,0);self.assertEqual(n.requests,0)
        self.assertTrue(all(c==[0.,0.,0.] for c in n.zeros))
    def test_legacy_inside_timer_behavior_not_replaced_by_new_timeout(self):
        n=ControlCallbacks();n.state='stopped'
        n.on_request(SimpleNamespace(data=json.dumps(dict(request_id='old-inside',frame_id='camera_init',waypoints=[[0,0,0]]))))
        n.pose=np.array([.1,0.,0.]);n.segment_start=np.array([-1.,0.,0.]);n.segment_started_ros=10.
        n.tick_at(100.3)
        self.assertEqual(n.state,'running');self.assertEqual(n.waypoint_index,0)
        self.assertEqual(n.zeros,[[0.,0.,0.]])
    def test_legacy_endpoint_handling_is_unchanged(self):
        n=ControlCallbacks();n.state='stopped'
        n.on_request(SimpleNamespace(data=json.dumps(dict(request_id='old-held',frame_id='camera_init',waypoints=[[0,0,0]]))))
        n.pose=np.array([.5,0.,0.]);n.tracking_pose=n.pose.copy()
        n.segment_start=np.array([-1.,0.,0.]);n.segment_started_ros=10.
        n.samples=np.array([[-1.,0.,0.],[0.,0.,0.]])
        n.obstacle_hold=True;n.tick_at(10.)
        self.assertIsNone(n.samples);self.assertEqual(n.requests,1)
        self.assertEqual(n.obstacle_result,(False,None,0));self.assertTrue(n.obstacle_hold)
        self.assertEqual(n.zeros,[[0.,0.,0.]])

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(k) for k in (MethodsTests,HeldRegionControlTests)]))
    (Path(__file__).with_name(os.environ.get('DEMO_TEST_CONTROLLER_RESULT','controller_method_result.json'))).write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,
        method_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),goal_core_sha256=hashlib.sha256((NAV/'goal_regions.py').read_bytes()).hexdigest(),
        scope=__doc__,failures=[(str(t),s) for t,s in result.errors+result.failures]),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
