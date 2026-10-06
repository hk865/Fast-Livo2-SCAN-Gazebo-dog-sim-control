"""Pure/actual-method boundaries for the excluded ALIGN candidate, no ROS node."""
import ast,hashlib,importlib.util,json,math,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from align_translation import AlignTranslation
from turn_drift import TurnDriftSupervisor
ROOT=HERE.parents[2];sys.path.insert(0,str(ROOT/'navigation'))
from control_core import limit_acceleration,obstacle_ahead,follow_trajectory
spec=importlib.util.spec_from_file_location('old_method_contract',HERE.parent/'turn_drift_staging/test_candidate.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)

class Twist:
    def __init__(self):self.linear=S(x=0.,y=0.,z=0.);self.angular=S(x=0.,y=0.,z=0.)
class Pub:
    def __init__(self,n=None):self.n=n
    def publish(self,m):
        if self.n is not None:self.n.published.append((m.linear.x,m.linear.y,m.angular.z))
def base_odom(self,msg):
    if msg.t>self.pose_stamp:
        self.pose_stamp=msg.t;self.region_raw_pose=np.array(msg.p);self.pose_updated=time.monotonic()
    self.pose=np.array(msg.p);self.tracking_pose=self.pose.copy();self.rotation=np.array(msg.R)
parsed=ast.parse((ROOT/'navigation/controller.py').read_text())
base=next(x for x in parsed.body if isinstance(x,ast.ClassDef) and x.name=='Navigation')
publish=next(x for x in base.body if isinstance(x,ast.FunctionDef) and x.name=='publish_command')
space=dict(Twist=Twist,Bool=lambda **kw:S(**kw),np=np,limit_acceleration=limit_acceleration)
exec(compile(ast.Module(body=[publish],type_ignores=[]),'actual production publish_command','exec'),space)
old.Base.publish_command=space['publish_command'];old.Base.on_odom=base_odom
space=dict(NavigationDrift=old.Nav,Navigation=old.Base,AlignTranslation=AlignTranslation,
    TurnDriftSupervisor=TurnDriftSupervisor,math=math,time=time,np=np,obstacle_ahead=obstacle_ahead,
    follow_trajectory=follow_trajectory,json=json,Path=Path,hashlib=hashlib)
parsed=ast.parse((HERE/'nav_align_controller.py').read_text())
cls=next(x for x in parsed.body if isinstance(x,ast.ClassDef) and x.name=='NavigationAlign')
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(HERE/'nav_align_controller.py'),'exec'),space)
Nav=space['NavigationAlign']
C=('r','g',5,(1,0))
def helper(v=-.1):
    h=AlignTranslation();h.begin(C,0,[0,0,0],np.eye(3),0)
    for t in [100000000,200000000,300000000,400000000]:h.observe(C,t,[v*t/1e9,0,0],np.eye(3),t)
    h.output(C,400000000,400000000,phase='align')
    return h
def node(enabled=True):
    n=old.node();n.__class__=Nav;n.align_enabled=enabled;n.align_pd=AlignTranslation()
    n.align_raw=dict(stamp=0,p=np.zeros(3),R=np.eye(3),wall=time.monotonic_ns())
    n.align_motion_guard=None;n.align_emit=None;n.cmd_pub=Pub(n);n.freeze_pub=Pub()
    n.counts={'commands':0};n.last_command_time=0.
    n.cloud=np.array([[5.,5.,-.5],[5.,6.,-.5],[6.,5.,-.5]])
    n.cloud_stamp=0;n.cloud_input_context={'message_stamp_ns':0,'frame_id':'camera_init','filtering_body_stamp_ns':0,'filtered_points':3}
    return n
def sample(n,t,p,*,R=None):
    n.clock_ns=t;n.on_odom(S(t=t,p=p,R=np.eye(3) if R is None else R))
def valid_pd(n,v=-.1):
    n.publish_command(np.zeros(2),.12)
    for t in [100000000,200000000,300000000,400000000]:
        sample(n,t,[v*t/1e9,0,0]);n.publish_command(np.zeros(2),.12)
    n.command=[0.,0.,.08]
    n.clock_ns=500000000;n.align_raw['wall']=time.monotonic_ns()

class Helper(unittest.TestCase):
    def test_predeclared_profile(self):
        c=AlignTranslation.CONFIG;self.assertEqual((c['kp_per_s'],c['kd'],c['max_abs_vx_m_s'],c['slew_m_s2']),(.8,.4,.1,.2))
        self.assertFalse(c['integral_enabled']);self.assertEqual(c['mixed_yaw_cap_rad_s'],.08)
    def test_signs_and_original_raw_anchor(self):
        for v,sign in [(-.1,1),(.1,-1)]:
            h=helper(v);o=h.output(C,500000000,500000000,phase='align')
            self.assertGreater(sign*o,0.);self.assertTrue(np.array_equal(h.anchor,[0,0,0]))
    def test_rotate_full_body_frame(self):
        h=AlignTranslation();R=Rotation.from_euler('z',np.pi/2).as_matrix();h.begin(C,0,[0,0,0],R,0)
        for t in range(100000000,500000000,100000000):h.observe(C,t,[0,-.1*t/1e9,0],R,t)
        h.output(C,400000000,400000000,phase='align')
        self.assertGreater(h.output(C,500000000,500000000,phase='align'),0.)
    def test_duplicate_and_foreign_never_refresh(self):
        h=helper();last=h.last_wall;count=len(h.samples)
        self.assertFalse(h.observe(C,400000000,[-1,0,0],np.eye(3),900000000))
        self.assertEqual((h.last_wall,len(h.samples)),(last,count))
        self.assertFalse(h.observe(('new','g',5,(1,0)),500000000,[0,0,0],np.eye(3),500000000));self.assertIsNone(h.context)
    def test_min_samples_and_span(self):
        h=AlignTranslation();h.begin(C,0,[0,0,0],np.eye(3),0)
        for t in [100000000,200000000]:h.observe(C,t,[-.1,0,0],np.eye(3),t)
        self.assertEqual(h.output(C,200000000,200000000,phase='align'),0.)
        h.observe(C,300000000,[-.1,0,0],np.eye(3),300000000)
        self.assertEqual(h.output(C,300000000,300000000,phase='align'),0.)
    def test_exact_gap_age_and_plus_one(self):
        h=helper();self.assertTrue(h.observe(C,600000000,[-.06,0,0],np.eye(3),600000000))
        self.assertTrue(h.output(C,850000000,850000000,phase='align')>=0.)
        self.assertEqual(h.output(C,850000001,850000001,phase='align'),0.);self.assertIsNone(h.context)
        h=helper();self.assertFalse(h.observe(C,600000001,[-.06,0,0],np.eye(3),600000001))
    def test_clock_and_wall_rollback_exact_zero(self):
        for clock,wall in [(550000000,610000000),(610000000,550000000)]:
            h=helper();h.output(C,500000000,500000000,phase='align');h.output(C,600000000,600000000,phase='align')
            self.assertEqual(h.output(C,clock,wall,phase='align'),0.);self.assertIsNone(h.context)
    def test_outside_phase_and_protection_exact_zero(self):
        for kw in [dict(phase='pre_turn'),dict(phase='settle'),dict(phase='drive'),dict(phase='align',protected=True)]:
            h=helper();h.output(C,500000000,500000000,phase='align');self.assertEqual(h.output(C,600000000,600000000,**kw),0.)
    def test_fit_residual_and_nonfinite_reject(self):
        h=helper();h.samples[2]=(200000000,np.array([1.,0,0]),np.eye(3))
        self.assertEqual(h.output(C,500000000,500000000,phase='align'),0.)
        self.assertEqual(h.latest['reason'],'fit_residual')
        with self.assertRaises(ValueError):h.observe(C,500000000,[np.nan,0,0],np.eye(3),500000000)
    def test_deadbands_and_slew(self):
        h=helper(0.);self.assertEqual(h.output(C,500000000,500000000,phase='align'),0.)
        h=helper(-.3)
        for t in range(450000000,650000001,50000000):
            before=h.command;o=h.output(C,t,t,phase='align');self.assertLessEqual(abs(o-before),.0100000001);self.assertLessEqual(abs(o),.1)

class Method(unittest.TestCase):
    def test_baseline_class_ast_exact_and_disabled_command_equal(self):
        a=ast.parse((HERE/'nav_align_controller.py').read_text());b=ast.parse((HERE.parent/'turn_drift_staging/nav_drift_controller.py').read_text())
        ka=next(x for x in a.body if isinstance(x,ast.ClassDef) and x.name=='NavigationDrift')
        kb=next(x for x in b.body if isinstance(x,ast.ClassDef) and x.name=='NavigationDrift')
        self.assertEqual(ast.dump(ka),ast.dump(kb))
        x=node(False);y=node(False);y.__class__=old.Nav
        for phase,v,w in [('pre_turn',[0,0],0),('align',[0,0],.12),('drive',[.12,0],-.08)]:
            for t in range(100000000,400000000,50000000):
                for n in [x,y]:n.clock_ns=t;n.heading_gate.phase=phase;n.publish_command(np.array(v),w)
                self.assertEqual(x.command,y.command)
    def test_legacy_not_arm(self):
        n=node();n.goals[0].legacy=True;n.publish_command(np.zeros(2),.12)
        self.assertIsNone(n.drift);self.assertIsNone(n.align_pd.context)
    def test_pure_to_mixed_uses_final_cap(self):
        n=node();valid_pd(n);n.command=[0.,0.,.12]
        for t in range(500000000,750000000,50000000):
            n.clock_ns=t;n.align_raw['wall']=time.monotonic_ns();n.publish_command(np.zeros(2),.12)
            if n.command[0]!=0.:self.assertLessEqual(abs(n.command[2]),.08+1e-12)
        self.assertTrue(any(v[0]>0 for v in n.published))
    def test_mixed_to_zero_is_exact_translation_and_bounded_yaw(self):
        n=node();valid_pd(n);n.command=[.03,0,.08];n.align_pd.command=0.
        n.align_pd.samples.clear();n.publish_command(np.zeros(2),.12)
        self.assertEqual(n.command[0],0.);self.assertLessEqual(abs(n.command[2]),.08)
    def test_compensation_drift_not_escape_guard(self):
        n=node();valid_pd(n);n.command=[.05,0,.08];n.bridge_safety['safe']=[.05,0,.08]
        for t in [600000000,700000000,800000000]:sample(n,t,[-.16,0,0])
        self.assertTrue(n.drift_pending);self.assertEqual(n.command,[0.,0.,0.]);self.assertEqual(n.segment_started_ros,0.)
    def test_signed_body_forward_and_backward_corridors(self):
        for vx,sgn in [(.05,1),(-.05,-1)]:
            n=node();n.cloud=np.array([[sgn*.65,y,.1] for y in [-.1,0,.1]])
            blocked,diag=n._align_body_corridor(vx)
            self.assertTrue(blocked);self.assertEqual(diag['body_x_sign'],sgn)
            self.assertFalse(n._align_body_corridor(-vx)[0])
    def test_corridor_stops_before_any_new_translation(self):
        n=node();valid_pd(n);n.cloud=np.array([[.65,y,.1] for y in [-.1,0,.1]])
        before=len(n.published);n.publish_command(np.zeros(2),.12)
        self.assertEqual(n.command,[0.,0.,0.]);self.assertTrue(n.drift_pending)
        self.assertTrue(all(v==(0.,0.,0.) for v in n.published[before:]))
    def test_stale_pending_hold_and_missing_cloud_failclosed(self):
        for kind in ['stale','pending','hold','missingcloud']:
            n=node();valid_pd(n)
            if kind=='stale':n.clock_ns=800000001
            elif kind=='pending':n.drift_pending=True
            elif kind=='hold':n.tilt_hold=True
            else:n.cloud_input_context=None
            n.publish_command(np.zeros(2),.12);self.assertEqual(n.command,[0.,0.,0.])
    def test_old_future_foreign_empty_unpaired_cloud_failclosed(self):
        for kind in ['old','future','frame','count','pair','empty','nonfinite','wallstale']:
            n=node();valid_pd(n);ctx=n.cloud_input_context
            if kind=='old':n.cloud_stamp=100000000
            elif kind=='future':ctx['message_stamp_ns']=600000000;n.cloud_stamp=600000000
            elif kind=='frame':ctx['frame_id']='world'
            elif kind=='count':ctx['filtered_points']=4
            elif kind=='pair':ctx['filtering_body_stamp_ns']=150000001
            elif kind=='empty':n.cloud=np.zeros((0,3));ctx['filtered_points']=0
            elif kind=='nonfinite':n.cloud[0,0]=np.nan
            else:n.cloud_updated=time.monotonic()-1.001
            n.publish_command(np.zeros(2),.12)
            self.assertEqual(n.command,[0.,0.,0.])
            # The original global wall-stale watchdog prevents this callback
            # from reaching any movement/corridor and already emits exact zero.
            if kind!='wallstale':self.assertTrue(n.drift_pending)
    def test_raw_gap_exact_stop_and_no_arrival_or_deadline_change(self):
        n=node();valid_pd(n);sample(n,700000001,[-.06,0,0])
        self.assertTrue(n.drift_pending);self.assertEqual(n.command,[0.,0.,0.])
        self.assertEqual(n.measured_region_arrival(),(False,False));self.assertEqual(n.segment_started_ros,0.)
    def test_raw_quaternion_duplicate_not_pd_input(self):
        n=node();sample(n,100000000,[-.01,0,0],R=np.eye(3));raw=n.align_raw.copy()
        sample(n,100000000,[4.,0,0],R=Rotation.from_euler('z',2.).as_matrix())
        self.assertEqual(n.align_raw['stamp'],raw['stamp']);self.assertTrue(np.array_equal(n.align_raw['R'],raw['R']))
    def test_stop_uses_original_real_idle_and_reference_chain(self):
        n=node();valid_pd(n);n._align_checked_stop('test_quality',{})
        old.bridge(n,600000000,adapter_state='returning')
        self.assertEqual(n.reference_requests,0)
        for t in range(700000000,1900000000,100000000):old.bridge(n,t)
        self.assertEqual(n.reference_requests,1);self.assertTrue(n.drift_pending)
        msg,meta=old.pair(n);n.accept_spline(msg,meta)
        self.assertFalse(n.drift_pending);self.assertEqual(n.heading_gate.phase,'pre_turn')

if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in [Helper,Method]])
    r=unittest.TextTestRunner(verbosity=2).run(suite)
    out=HERE/'contract_result.json'
    out.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),
        scope=__doc__,source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['nav_align_controller.py','align_translation.py','turn_drift.py']}),indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
