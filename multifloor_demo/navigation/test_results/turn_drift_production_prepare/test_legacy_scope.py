"""Actual prepared methods versus frozen candidate, no ROS or physics."""
import ast,copy,hashlib,importlib.util,json,sys,time
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import test_prepared as prepared
spec=importlib.util.spec_from_file_location('original_frozen_drift_methods',HERE.parent/'turn_drift_staging/test_candidate.py')
frozen=importlib.util.module_from_spec(spec);spec.loader.exec_module(frozen)

def snapshots(node):
    return dict(state=node.state,index=node.waypoint_index,command=node.command,
        published=node.published,pending=node.drift_pending,interruptions=node.drift_events,
        reference_requests=node.reference_requests,reference_stamp=node.trajectory_association.reference_stamp,
        accepted=node.trajectory_association.accepted,phase=node.heading_gate.phase,
        drift_state=None if node.drift is None else node.drift.state,
        region_resets=node.arrival_resets,deadline=node.segment_started_ros,
        sample_points=None if node.samples is None else node.samples.tolist())

class Scope(unittest.TestCase):
    def test_legacy_cannot_arm_even_actual_pure_turn_and_large_raw_drift(self):
        n=prepared.node();n.goals[0].legacy=True
        self.assertIsNone(n.drift_context())
        n.publish_command(np.zeros(2),.12)
        for t in range(100000000,1500000000,100000000):
            n.clock_ns=t;n.on_odom(S(t=t,p=[.5,0,0]))
        self.assertIsNone(n.drift);self.assertFalse(n.drift_pending);self.assertEqual(n.drift_events,0)
        self.assertEqual(n.reference_requests,0);self.assertEqual(n.published[-1],(0.,0.,.12))
    def test_legacy_inside_keeps_original_base_arrival_without_guard_wait(self):
        n=prepared.node();n.goals[0].legacy=True;n.region_arrival=None
        n.region_arrivals=[];n.region_arrival_evidence=None;n.pose=np.array([1.,0,0]);n.region_raw_pose=n.pose.copy()
        n.publish_command(np.zeros(2),.12)
        n.clock_ns=1000000000;n.control(time.monotonic())
        n.clock_ns=1500000000;n.control(time.monotonic())
        self.assertEqual(n.state,'succeeded');self.assertEqual(n.waypoint_index,1)
        self.assertFalse(n.drift_pending);self.assertEqual(n.drift_events,0)
    def test_legacy_original_90_deadline_is_unchanged(self):
        n=prepared.node();n.goals[0].legacy=True;n.clock_ns=90000000001;n.control(time.monotonic())
        self.assertEqual(n.state,'failed');self.assertEqual(n.published[-1],(0.,0.,0.))
    def test_request_reset_removes_pending_before_switching_to_legacy(self):
        n=prepared.node();prepared.trigger(n);n.on_request(S(data='next_request'))
        n.goals[0].legacy=True
        self.assertFalse(n.drift_pending);self.assertIsNone(n.drift_context())
        n.publish_command(np.zeros(2),.12);self.assertIsNone(n.drift)
    def test_original_boundary_demonstrates_the_reason_for_v2_scope(self):
        n=frozen.node();n.goals[0].legacy=True;frozen.trigger(n)
        self.assertTrue(n.drift_pending)
        n.region_arrival=None;n.region_arrivals=[];n.region_arrival_evidence=None
        n.pose=np.array([1.,0,0]);n.region_raw_pose=n.pose.copy()
        n.clock_ns=1000000000;n.control(time.monotonic())
        n.clock_ns=1500000000;n.control(time.monotonic())
        self.assertEqual(n.state,'succeeded') # Original retained negative proof.
    def test_v2_stop_idle_reference_checked_progress_all_match_frozen(self):
        a=frozen.node();b=prepared.node()
        for n in [a,b]:prepared.trigger(n)
        self.assertEqual(snapshots(a),snapshots(b))
        for t in range(400000000,1500000000,100000000):
            for n in [a,b]:prepared.bridge(n,t)
            self.assertEqual(snapshots(a),snapshots(b))
        for n in [a,b]:
            msg,meta=prepared.pair(n);n.accept_spline(msg,meta)
        self.assertEqual(snapshots(a),snapshots(b));self.assertEqual(b.heading_gate.phase,'pre_turn')
    def test_v2_protected_wrong_identity_and_deadline_match_frozen(self):
        for action in ['failed_bridge','old_spline','degenerate','deadline']:
            a=frozen.node();b=prepared.node()
            for n in [a,b]:
                prepared.trigger(n)
                if action=='failed_bridge':prepared.bridge(n,400000000,bridge_state='failed')
                elif action=='deadline':n.clock_ns=90000000001;n.control(time.monotonic())
                else:
                    prepared.handoff(n)
                    msg,meta=prepared.pair(n,ref=[1,0] if action=='old_spline' else None,span=0. if action=='degenerate' else 1.)
                    n.accept_spline(msg,meta)
            self.assertEqual(snapshots(a),snapshots(b))
    def test_only_drift_context_method_ast_changes_and_is_exact_legacy_guard(self):
        old=ast.parse((HERE.parent/'turn_drift_staging/nav_drift_controller.py').read_text())
        new=ast.parse((HERE/'drift_controller.py').read_text())
        classes=lambda t:{n.name:n for n in t.body if isinstance(n,ast.ClassDef)}
        a,b=classes(old),classes(new)
        changes=[]
        for name in a:
            for x,y in zip(a[name].body,b[name].body):
                if ast.dump(x)!=ast.dump(y):changes.append(x.name)
        self.assertEqual(changes,['drift_context'])
        fn=next(n for n in b['NavigationDrift'].body if isinstance(n,ast.FunctionDef) and n.name=='drift_context')
        fn=copy.deepcopy(fn);fn.body[0].test.values=[n for n in fn.body[0].test.values
            if not (isinstance(n,ast.Attribute) and n.attr=='legacy')]
        original=next(n for n in a['NavigationDrift'].body if isinstance(n,ast.FunctionDef) and n.name=='drift_context')
        self.assertEqual(ast.dump(fn),ast.dump(original))
    def test_runtime_declaration_matches_actual_profile_status_and_helper(self):
        d=json.loads((HERE/'guard_configuration.json').read_text());n=prepared.node();status=n.drift_status()
        g=prepared.TurnDriftSupervisor(profile=d['profile'],require_path_offset=d['require_path_offset'])
        self.assertEqual(d['limits'],dict(g.LIMITS,require_path_offset=False));self.assertEqual(d['limits'],status['limits'])
        self.assertEqual(d['profile'],status['candidate']);self.assertEqual(d['request_scope'],'schema2_nonlegacy_goals')

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Scope))
    (HERE/'legacy_scope_result.json').write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,
        errors=len(r.errors),failures=len(r.failures),scope=__doc__),indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
