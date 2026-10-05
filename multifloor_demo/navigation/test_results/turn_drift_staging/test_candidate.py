"""Selected-profile + actual wrapper methods; no ROS node, truth or physics."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest
import numpy as np
from scipy.interpolate import BSpline

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'navigation'))
from control_core import HeadingGate,follow_trajectory,trajectory_has_progress
from trajectory_contract import TrajectoryAssociation
from turn_drift import TurnDriftSupervisor

def original_method(name):
    parsed=ast.parse((ROOT/'navigation/controller.py').read_text())
    cls=next(n for n in parsed.body if isinstance(n,ast.ClassDef) and n.name=='Navigation')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    space=dict(np=np,math=math,time=time,BSpline=BSpline,trajectory_has_progress=trajectory_has_progress)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(ROOT/'navigation/controller.py'),'exec'),space)
    return space[name]

class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
    def get_subscription_count(self):return 1

class Base:
    # Test boundaries replace middleware only. The production plan identity,
    # progress validation and entire deadline/control implementation are real.
    control=original_method('control')
    request_plan=original_method('request_plan')
    accept_spline=original_method('accept_spline')
    @staticmethod
    def stamp(msg):return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    def get_clock(self):return S(now=lambda:S(nanoseconds=self.clock_ns))
    def publish_command(self,velocity=None,yaw_rate=0.):
        self.command=[0.,0.,0.] if velocity is None else [float(velocity[0]),float(velocity[1]),float(yaw_rate)]
        self.published.append(tuple(self.command))
    def publish_status(self):self.status_calls+=1
    def reset_region_arrival(self,reason):self.arrival_resets.append(reason)
    def measured_region_arrival(self):return True,True
    def on_odom(self,msg):
        if msg.t>self.pose_stamp:
            self.pose_stamp=msg.t;self.region_raw_pose=np.array(msg.p);self.pose_updated=time.monotonic()
        self.pose=np.array(msg.p);self.tracking_pose=self.pose.copy()
    def on_bridge_safety(self,msg):
        self.bridge_safety=json.loads(msg.data)
        if self.bridge_safety['state']=='failed':self.state='failed';self.publish_command()
    def on_request(self,msg):self.request_id=msg.data
    def make_path(self,points,height_offset=0.):
        sec,nsec=divmod(self.clock_ns,1000000000)
        return S(header=S(stamp=S(sec=sec,nanosec=nsec)),points=points,height_offset=height_offset)
    def apply_tilt_guard(self,now,ros):return False

space=dict(Navigation=Base,json=json,np=np,math=math,time=time,
           follow_trajectory=follow_trajectory,TurnDriftSupervisor=TurnDriftSupervisor)
code=ast.parse((HERE/'nav_drift_controller.py').read_text())
klass=next(n for n in code.body if isinstance(n,ast.ClassDef) and n.name=='NavigationDrift')
exec(compile(ast.Module(body=[klass],type_ignores=[]),str(HERE/'nav_drift_controller.py'),'exec'),space)
Nav=space['NavigationDrift']

def node():
    n=Nav.__new__(Nav)
    n.clock_ns=0;n.drift=None;n.drift_pending=False;n.drift_stop_clock_ns=None
    n.drift_stop_wall=None;n.drift_stop_count=None;n.drift_bridge_wall=time.monotonic()
    n.drift_bridge_sequence=0;n.drift_consumed_bridge_sequence=0
    n.drift_log=deque(maxlen=20000);n.drift_log_dropped=0;n.drift_events=0
    n.request_id='request';n.state='running';n.waypoint_index=0
    n.goals=[S(goal_id='goal',legacy=False,timeout_sim_s=90.)];n.waypoints=np.array([[1.,0,0]])
    n.samples=np.array([[0.,0,0],[1.,0,0]]);n.active_trajectory_id=5
    n.trajectory_association=TrajectoryAssociation();n.trajectory_association.request([1,0],[1,0,0]);n.trajectory_association.accepted={'traj_id':5}
    n.region_raw_pose=np.zeros(3);n.pose=np.zeros(3);n.pose_stamp=0;n.rotation=np.eye(3);n.tracking_pose=np.zeros(3)
    n.heading_gate=HeadingGate();n.heading_gate.phase='align';n.heading_gate.heading=0.
    n.command=[0.,0.,0.];n.tilt_hold=False;n.obstacle_hold=False
    n.pose_updated=n.cloud_updated=time.monotonic();n.pose_timeout=.8;n.cloud_timeout=1.
    n.raw_imu=S(fresh=lambda *a:True);n.arrival_since=None
    n.bridge_safety=dict(state='ready',safe=[0.,0.,.12],requested=[0.,0.,.12],
        joint_adapter=dict(state='walk',sim=0.,nominal_calibrated=True,counters=dict(stops=0)),joint_adapter_wall_age=0.)
    n.published=[];n.status_calls=0;n.arrival_resets=[];n.steering=None;n.planning_start_state=None
    n.segment_start=np.zeros(3);n.segment_started_ros=0.;n.obstacle_resume_pending=False;n.alignment_hold=True
    n.last_reference=-math.inf;n.last_reference_stamp=-1;n.reference_requests=0;n.reference_pub=Publisher()
    n.max_speed=.12;n.arrival_radius=.22;n.degenerate_splines=0;n.replans=0
    n.last_spline_rejected=None;n.path_pub=Publisher();n.message=''
    return n

def trigger(n):
    n.publish_command(np.zeros(2),.12)
    for t in [100000000,200000000,300000000]:
        n.clock_ns=t;n.on_odom(S(t=t,p=[.16,0,0]))
    assert n.drift_pending

def bridge(n,t,**kw):
    n.clock_ns=t
    data=dict(state=kw.get('bridge_state','ready'),safe=kw.get('safe',[0.,0.,0.]),requested=kw.get('requested',[0.,0.,0.]),
        joint_adapter=dict(state=kw.get('adapter_state','idle'),sim=kw.get('adapter_sim',t/1e9),
            nominal_calibrated=kw.get('nominal',True),counters=dict(stops=kw.get('stops',1))),
        joint_adapter_wall_age=kw.get('age',0.))
    n.on_bridge_safety(S(data=json.dumps(data)))

def handoff(n):
    for t in range(400000000,1500000000,100000000):bridge(n,t)

def pair(n,*,span=1.,traj=6,ref=None):
    ref=ref or list(n.drift.reference)
    pts=[[.16,0,0],[.16+span/3,0,0],[.16+2*span/3,0,0],[.16+span,0,0]]
    msg=S(order=3,traj_id=traj,start_time=S(sec=2,nanosec=0),pos_pts=[S(x=p[0],y=p[1],z=p[2]) for p in pts],knots=[0.,0.,0.,0.,1.,1.,1.,1.])
    meta=dict(schema=1,reference_stamp=ref,body_goal=[1.,0.,0.],trajectory=dict(order=3,traj_id=traj,
         start_time=[2,0],pos_pts=pts,knots=msg.knots))
    return msg,meta

class Profile(unittest.TestCase):
    def fresh(self):
        g=TurnDriftSupervisor(require_path_offset=False,profile='displacement_015_02')
        g.begin(('r','g',5,(1,0)),0,[0,0,0],0,[1,0,0]);return g
    def obs(self,g,t,p=.15,**kw):
        return g.observe(t,[p,0,0],0,('r','g',5,(1,0)),phase=kw.get('phase','align'),
            actual_command=kw.get('cmd',[0.,0.,.12]),adapter_state=kw.get('adapter','walk'),
            protected=kw.get('protected',False),pose_age_ns=kw.get('age',0))
    def test_selected_fixed_limits(self):
        g=self.fresh();self.assertEqual((g.LIMITS['planar_drift_m'],g.LIMITS['persistence_ns'],g.LIMITS['min_raw_observations']),(.15,200000000,3))
    def test_three_raw_and_duration_both_required(self):
        g=self.fresh();self.obs(g,100000000);self.assertIsNone(self.obs(g,300000000)['event'])
        self.assertIsNotNone(self.obs(g,300000001)['event'])
    def test_199999999_not_enough(self):
        g=self.fresh();self.obs(g,100000000);self.obs(g,200000000)
        self.assertIsNone(self.obs(g,299999999)['event']);self.assertIsNotNone(self.obs(g,300000000)['event'])
    def test_duplicate_not_three_raw(self):
        g=self.fresh();self.obs(g,100000000)
        for _ in range(10):self.assertIsNone(self.obs(g,100000000)['event'])
        self.assertIsNone(self.obs(g,300000000)['event'])
    def test_gap_plus_one_resets(self):
        g=self.fresh();self.obs(g,100000000);self.obs(g,300000001)
        self.assertIsNone(self.obs(g,400000001)['event']);self.obs(g,500000001)
        self.assertIsNotNone(self.obs(g,600000001)['event'])
    def test_strict_pure_turn_freshness_protection(self):
        for kw in [dict(cmd=[1e-12,0,.12]),dict(phase='drive'),dict(adapter='returning'),dict(protected=True),dict(age=250000001)]:
            g=self.fresh()
            for t in range(100000000,1000000000,100000000):self.assertIsNone(self.obs(g,t,**kw)['event'])
    def test_below_threshold_and_single_impulse(self):
        g=self.fresh()
        for t in range(100000000,1000000000,100000000):self.assertIsNone(self.obs(g,t,p=.14999999)['event'])
        self.obs(g,1100000000,p=.4);self.obs(g,1200000000,p=0.)
        self.assertIsNone(self.obs(g,1300000000,p=.4)['event'])

class Integration(unittest.TestCase):
    def stopped(self):n=node();trigger(n);return n
    def test_first_stop_exact_zero_and_no_region_or_deadline_change(self):
        n=self.stopped();self.assertEqual(n.published[-1],(0.,0.,0.))
        self.assertIsNone(n.samples);self.assertIsNone(n.trajectory_association.reference_stamp)
        self.assertEqual((n.waypoint_index,n.segment_started_ros),(0,0.))
        self.assertEqual(n.drift_log[-1]['event'],'exact_zero_published')
    def test_pending_never_publishes_nonzero(self):
        n=self.stopped();n.publish_command(np.array([.12,0]),.12)
        self.assertEqual(n.published[-1],(0.,0.,0.))
    def test_pending_region_window_blocked(self):
        n=self.stopped();self.assertEqual(n.measured_region_arrival(),(False,False))
    def test_original_real_deadline_still_fails(self):
        n=self.stopped();n.clock_ns=90000000001;n.control(time.monotonic())
        self.assertEqual(n.state,'failed');self.assertEqual(n.published[-1],(0.,0.,0.))
    def test_stopped_request_plan_does_not_fake_reference(self):
        n=self.stopped();n.request_plan();self.assertEqual(n.reference_requests,0)
    def test_timer_cannot_accumulate_cached_zero(self):
        n=self.stopped();bridge(n,400000000)
        n.clock_ns=2400000000
        for _ in range(100):n.drift_handoff(time.monotonic())
        self.assertEqual(n.reference_requests,0)
    def test_old_idle_and_unchanged_stop_count_no_release(self):
        for kw in [dict(adapter_sim=.2),dict(stops=0),dict(adapter_state='returning'),dict(nominal=False),dict(age=.250000001)]:
            n=self.stopped()
            for t in range(400000000,1900000000,100000000):bridge(n,t,**kw)
            self.assertEqual(n.reference_requests,0)
    def test_duplicate_clock_bridge_never_dwell(self):
        n=self.stopped()
        for _ in range(50):bridge(n,400000000)
        self.assertEqual(n.reference_requests,0)
    def test_nonzero_requested_or_safe_never_release(self):
        for kw in [dict(requested=[0,0,.01]),dict(safe=[0,0,.01])]:
            n=self.stopped()
            for t in range(400000000,1900000000,100000000):bridge(n,t,**kw)
            self.assertEqual(n.reference_requests,0)
    def test_status_gap_breaks_actual_zero_window(self):
        n=self.stopped();bridge(n,400000000);bridge(n,1500000000)
        self.assertEqual(n.reference_requests,0)
    def test_real_idle_new_ref_still_no_motion(self):
        n=self.stopped();handoff(n)
        self.assertEqual(n.reference_requests,1);self.assertTrue(n.drift_pending)
        self.assertEqual(n.drift.state,'await_paired_source')
        n.publish_command(np.array([.12,0]),.12);self.assertEqual(n.published[-1],(0.,0.,0.))
    def test_returning_zero_does_not_start_actual_idle_window(self):
        n=self.stopped()
        for t in range(400000000,1400000000,100000000):bridge(n,t,adapter_state='returning')
        bridge(n,1400000000)
        self.assertEqual(n.reference_requests,0)
        for t in range(1500000000,2500000000,100000000):bridge(n,t)
        self.assertEqual(n.reference_requests,1)
    def test_old_reference_and_old_local_id_rejected(self):
        for kw in [dict(ref=[1,0]),dict(traj=5)]:
            n=self.stopped();handoff(n);msg,meta=pair(n,**kw);n.accept_spline(msg,meta)
            self.assertTrue(n.drift_pending);self.assertIsNone(n.samples)
    def test_real_production_degenerate_check_retains_hold(self):
        n=self.stopped();handoff(n);msg,meta=pair(n,span=0.)
        n.accept_spline(msg,meta);self.assertTrue(n.drift_pending);self.assertIsNone(n.samples)
        self.assertEqual(n.degenerate_splines,1)
    def test_full_payload_mismatch_never_release(self):
        n=self.stopped();handoff(n);msg,meta=pair(n);meta['trajectory']['pos_pts'][0][0]+=.01
        n.accept_spline(msg,meta);self.assertTrue(n.drift_pending);self.assertIsNone(n.samples)
    def test_new_real_progress_path_only_original_preturn(self):
        n=self.stopped();handoff(n);msg,meta=pair(n);n.accept_spline(msg,meta)
        self.assertFalse(n.drift_pending);self.assertIsNotNone(n.samples)
        self.assertEqual(n.heading_gate.phase,'pre_turn')
        self.assertFalse(n.heading_gate.update(n.clock_ns/1e9,0.,1.)[0])
    def test_failed_bridge_never_requests_or_moves(self):
        n=self.stopped();bridge(n,400000000,bridge_state='failed')
        self.assertEqual(n.state,'failed');self.assertEqual(n.reference_requests,0)
        self.assertEqual(n.published[-1],(0.,0.,0.))
    def test_new_goal_identity_cannot_share_drift_anchor(self):
        n=node();n.publish_command(np.zeros(2),.12)
        n.goals=[S(goal_id='other')];n.clock_ns=100000000;n.on_odom(S(t=100000000,p=[.4,0,0]))
        self.assertIsNone(n.drift);self.assertFalse(n.drift_pending)
    def test_drive_or_nonzero_planned_translation_not_arm(self):
        for velocity,phase in [(np.array([.12,0]),'align'),(np.zeros(2),'drive')]:
            n=node();n.heading_gate.phase=phase;n.publish_command(velocity,.12)
            self.assertIsNone(n.drift)
    def test_stale_bridge_does_not_trigger(self):
        n=node();n.publish_command(np.zeros(2),.12);n.drift_bridge_wall=time.monotonic()-.3
        for t in [100000000,200000000,300000000]:n.clock_ns=t;n.on_odom(S(t=t,p=[.4,0,0]))
        self.assertFalse(n.drift_pending)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([
        unittest.defaultTestLoader.loadTestsFromTestCase(Profile),unittest.defaultTestLoader.loadTestsFromTestCase(Integration)]))
    (HERE/'candidate_contract_result.json').write_text(json.dumps(dict(tests=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),
        scope='selected profile + actual excluded methods with real production plan/progress/deadline, middleware test boundary only'),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
