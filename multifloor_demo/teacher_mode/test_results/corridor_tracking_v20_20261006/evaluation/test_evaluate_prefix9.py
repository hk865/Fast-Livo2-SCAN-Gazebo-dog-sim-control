"""Finite evidence-gate tests; synthetic data never counts as navigation PASS."""
import copy
import tempfile
from pathlib import Path
import unittest
import evaluate_prefix9 as audit
from navigation.goal_regions import parse_goal, definitions_sha256


def source(index): return dict(file='synthetic',line=index,offset=index,length=1,sha256='synthetic')
def pose(t, x=0.): return dict(stamp_ns=t,position=[x,0.,0.],quaternion=[0.,0.,0.,1.],
    frame_id='camera_init',callback_ros_clock_ns=t, _source=source(t))
def native(t): return dict(state_physics_world_time=t/1e9,body_lin_vel=[0.,0.,0.],body_ang_vel=[0.,0.,0.],
    rpy=[0.,0.,0.],qd=[0.]*12,applied_torque=[0.]*12,q_target=[0.]*12,command=[0.]*3,
    contacts={'body':0},body_clearance=.3,fault=None,actor_inferred_this_frame=True,command_expired=False,_source=source(t))
def parking_data():
    start=1_000_000_000
    p=[pose(t) for t in range(start,start+5_000_000_001,50_000_000)]
    n=[native(t) for t in range(start,start+5_000_000_001,20_000_000)]
    c=[];s=[]
    for t in range(start,start+5_000_000_001,50_000_000):
        cascade=dict(mode='active_hold',parking_hold_declared_clock_ns=start,
            parking_hold_declared_pose_stamp_ns=start,parking_window_interrupted=False,
            fixed_goal_sha256='one-fixed-goal',protection_active=False,failure_latched=False)
        c.append(dict(request_id='r',waypoint_index=9,control_stamp_ns=t,cascade=cascade,_source=source(t)))
        s.append(dict(request_id='r',state='succeeded',ros_sim_time=t/1e9,region_arrivals=[{}]*9,
            cascade_parking=cascade,obstacle_hold=False,tilt_hold=False,execution_bridge_safety={'state':'ready'},_source=source(t)))
    return c,s,p,n


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.goal=parse_goal(dict(goal_id='exploration:0',center=[0.,0.,0.],arrival=dict(
            type='disc_prism',radius_m=.35,height_half_span_m=.1,dwell_sim_s=.4,
            control_band=dict(radius_m=.25,height_half_span_m=.1)),timeout_sim_s=90.))
        self.poses=[pose(t) for t in range(1_000_000_000,1_400_000_001,50_000_000)]
        self.receipt=dict(goal_id=self.goal.goal_id,waypoint_index=0,request_id='r',
            goals_definition_sha256=definitions_sha256([self.goal]),arrival_definition=self.goal.definition()['arrival'],
            control_arrival_definition=self.goal.control_arrival_definition(),start_stamp_ns=1_000_000_000,
            stamp_ns=1_400_000_000,dwell_ns=400_000_000,max_observation_gap_ns=200_000_000,
            raw_position=[0.,0.,0.],protected=False,region_inside=True,control_region_inside=True,
            reason='arrived',goal_activated_ros_clock_ns=0)
    def validate(self, poses=None, receipt=None):
        return audit.validate_dwell(self.goal,self.receipt if receipt is None else receipt,
            self.poses if poses is None else poses,'r',definitions_sha256([self.goal]),0)
    def test_raw_dwell_pass_and_gap_fails(self):
        self.assertTrue(self.validate()['passed'])
        self.assertFalse(self.validate([self.poses[0],self.poses[-2],self.poses[-1]])['passed'])
    def test_count_cannot_replace_actual_slam_endpoints(self):
        self.assertFalse(self.validate(self.poses[1:])['passed'])
    def test_outer_inside_but_inner_outside_fails(self):
        points=copy.deepcopy(self.poses);points[4]['position'][0]=.3
        self.assertFalse(self.validate(points)['checks']['original_control_region'])
    def test_changed_geometry_or_timeout_fails(self):
        changed=copy.deepcopy(self.receipt);changed['arrival_definition']['radius_m']=.36
        self.assertFalse(self.validate(receipt=changed)['passed'])
        changed=copy.deepcopy(self.receipt);changed['goal_activated_ros_clock_ns']=-90_000_000_000
        self.assertFalse(self.validate(receipt=changed)['checks']['within_original_timeout'])
    def test_native_safe_rejects_body_contact_and_nonfinite(self):
        row=native(0);self.assertTrue(audit.native_safe(row))
        row['contacts']['body']=1;self.assertFalse(audit.native_safe(row))
        row=native(0);row['applied_torque'][4]=float('nan');self.assertFalse(audit.native_safe(row))
    def test_valid_fixed5s(self):
        c,s,p,n=parking_data();self.assertTrue(audit.evaluate_parking('r',c,s,p,n)['passed'])
    def test_nine_arrivals_without_hold_is_not_pass(self):
        _,s,p,n=parking_data()
        self.assertFalse(audit.evaluate_parking('r',[],s,p,n)['passed'])
    def test_truncated_first_window_is_not_pass(self):
        c,s,p,n=parking_data()
        self.assertFalse(audit.evaluate_parking('r',c[:-2],s[:-2],p[:-2],n[:-5])['passed'])
    def test_first_interruption_cannot_be_hidden_by_later_quiet_window(self):
        c,s,p,n=parking_data();c[10]['cascade']=copy.deepcopy(c[10]['cascade'])
        c[10]['cascade']['parking_window_interrupted']=True
        self.assertFalse(audit.evaluate_parking('r',c,s,p,n)['passed'])
    def test_changing_declaration_or_goal_fails(self):
        c,s,p,n=parking_data();c[10]['cascade']=copy.deepcopy(c[10]['cascade'])
        c[10]['cascade']['parking_hold_declared_clock_ns']+=50_000_000
        self.assertFalse(audit.evaluate_parking('r',c,s,p,n)['passed'])
        c,s,p,n=parking_data();c[10]['cascade']=copy.deepcopy(c[10]['cascade'])
        c[10]['cascade']['fixed_goal_sha256']='different'
        self.assertFalse(audit.evaluate_parking('r',c,s,p,n)['passed'])
    def test_first_window_velocity_violation_is_not_diluted(self):
        c,s,p,n=parking_data();n[20]['body_lin_vel'][0]=.081
        self.assertFalse(audit.evaluate_parking('r',c,s,p,n)['passed'])
    def test_unstopped_run_is_refused_before_full_reads(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(RuntimeError,'Full audit is refused'):audit.audit(Path(temp),None)
    def test_jsonl_line_binding_and_partial_row_error(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'rows.jsonl';path.write_bytes(b'{"v":1}\n{"v":2}')
            sources=audit.Sources();rows=list(sources.rows(path))
            self.assertEqual(len(rows),1);self.assertEqual(rows[0][1]['sha256'],audit.sha(b'{"v":1}\n'))
            self.assertEqual(len(sources.errors),1)


if __name__=='__main__':unittest.main()
