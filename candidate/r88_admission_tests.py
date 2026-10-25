"""Finite real admission-method tests; no ROS node, graph, actuator or physics.

The production PID admission, Teacher/shared lower acceptance, and prepared
archive transaction run unchanged. Physical-source fixtures, handoff prediction
and cascade staging are controlled injection boundaries, not claimed physics.
Evidence is queued until the callback returns, as in the real asynchronous
writer. The tests inspect both in-memory outcomes and the resulting ledgers.
"""
from contextlib import ExitStack
import copy
import inspect
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

import controller
from automatic_reference_runtime import MAX_REFERENCE_POINTS
from mode3_trajectory_contract import payload
from r85_physical_path_gate import R85PhysicalPathGate


def message():
    return NS(order=3, traj_id=2, start_time=NS(sec=1,nanosec=0),
        pos_pts=[NS(x=x,y=0.,z=0.) for x in (0.,1/3,2/3,1.)],
        knots=[0.,0.,0.,0.,1.,1.,1.,1.],yaw_pts=[],yaw_dt=0.)


class Evidence:
    def __init__(self):
        self.error=None;self.rows=[];self.work=[];self.fail_batch_enqueue=False
    def append(self,path,row):self.rows.append((path.name,copy.deepcopy(row)))
    def enqueue(self,work):
        if self.fail_batch_enqueue and isinstance(getattr(work,'__self__',None),controller.PreparedAdmissionBatch):
            raise RuntimeError('injected_existing_writer_enqueue_failure')
        self.work.append(work)
    def flush(self):
        work,self.work=self.work,[]
        for call in work:call()


class PhysicalGate:
    def __init__(self):
        self.allowed=True;self.fresh_calls=0;self.reject_fresh_at=None
        self.certify_calls=0;self.complete_calls=0;self.delay=0.;self.use_actual_fresh=False
        self.binding_context=dict(fixture_binding=1,planner_epoch=2,planner_process_id=123,
            localization_generation=1)
    def certify(self,node,value,metadata,binding):
        self.certify_calls+=1
        if self.delay:time.sleep(self.delay)
        return self.result()
    def result(self):
        return dict(allowed=self.allowed,
            reason='actual_curve_physical_space_certified' if self.allowed else 'r85_physical_binding_rejected',
            source_binding=copy.deepcopy(self.binding_context))
    def complete(self,*args,**kwargs):
        self.complete_calls+=1
        if args and isinstance(args[-1],dict) and 'allowed' in args[-1]:return copy.deepcopy(args[-1])
        return self.result()
    def prepare(self,node,value,metadata,binding):
        return dict(physical_payload=copy.deepcopy(value),source_pose_stamp_ns=node.pose_stamp)
    def context(self,node,binding):return copy.deepcopy(self.binding_context)
    def fresh(self,node,binding,generation):
        self.fresh_calls+=1
        if self.use_actual_fresh:return R85PhysicalPathGate.fresh(self,node,binding,generation)
        return dict(allowed=self.fresh_calls!=self.reject_fresh_at,
            checked_monotonic_wall=time.monotonic(),source_binding=copy.deepcopy(self.binding_context),
            source_pose_stamp_ns=node.pose_stamp,source_cloud_stamp_ns=node.cloud_stamp,
            source_timestamps_refreshed=False)


class Fixture(controller.PIDNavigation):
    def __init__(self,run):
        self.run=Path(run);self.profile={};self.evidence=Evidence()
        self.request_id='r88-fixture';self.waypoint_index=0
        self.waypoints=np.array([[2.,0.,0.]]);self.arrival_radius=.22
        self.pose=np.zeros(3);self.pose_stamp=self.cloud_stamp=1_000_000_000
        self.pose_updated=self.cloud_updated=self.bridge_updated=time.monotonic()
        self.bridge_safety={'state':'ready'};self.raw_imu=NS(fresh=lambda *args:True)
        self.state='running';self.obstacle_hold=self.tilt_hold=False
        self.global_reference_settings={'enabled':False};self.continuous_route=None
        self.automatic_reference=NS(file_sha='fixture-reference',config={'max_reference_points':MAX_REFERENCE_POINTS},
            scope=(2,123,1),reference={'sent_reference':{'stamp':[1,0],'frame_id':'camera_init'}})
        self.binding=dict(schema='r85_automatic_route_fence_binding/v1',reference_file_sha256='fixture-reference')
        self.path_admission_fence=NS(load=lambda:(np.array([[0.,0.,0.],[2.,0.,0.]]),
            {'horizontal_radius_m':.45,'height_error_m':.3},copy.deepcopy(self.binding)))
        self.msg=message();self.value=payload(self.msg)
        self.metadata={'schema':2,'trajectory':copy.deepcopy(self.value),'reference_stamp':[1,0]}
        self.trajectory_association=NS(accepted=copy.deepcopy(self.value),reference_stamp=[1,0],goal=[2.,0.,0.])
        self.known_scene_inputs={'generation':1};self.known_scene_reference_generation=1
        self.known_scene_commit_token=('fixture-producer',1)
        self.known_scene_scan_epoch=dict(schema='known_scene_SCAN_epoch_ready/v1',
            run_id=self.run.name,ready=True,generation=1,localization_generation=1,
            planner_epoch=2,planner_identity={'pid':123})
        self.scan_reference_coverage_index=0;self.pending_path_commit=None
        self.samples=np.array([[0.,0.,0.],[.5,0.,0.]])
        self.active_trajectory_id=1;self.active_reference_stamp=(1,0)
        self.planning_start_state={'old':True};self.execution_path_exhaustion={'old_exhaustion':True}
        self.trajectory_archive_reference='old-array';self.replans=0;self.degenerate_splines=0
        self.cascade=None;self.cascade_key=None;self.cascade_path_id='old-path'
        self.path_receipt={'old_receipt':True}
        self.path_admission_active=dict(old_active=True,identity=self.admission_identity(),
            trajectory_id=1,active_reference_stamp=(1,0))
        self.old_active=copy.deepcopy(self.path_admission_active)
        self.path_admission_hold=False;self.path_admission_replan_pending=False
        self.reference_replan_pending=False;self.reference_consistency_hold=False
        self.reference_consistency_check=None;self.heading_gate={'old_heading':True}
        self.pid_guard_needs_evaluation=False;self.path_admission_sequence=0
        self.path_admission_last_decision=None;self.last_spline_rejected=None
        self.r85_physical_gate=PhysicalGate();self.r85_physical_gate.run=self.run
        self.command=[.1,0.,0.]
        self.zero_count=0;self.path_publications=[]
        self.path_pub=NS(publish=lambda value:self.path_publications.append(copy.deepcopy(value)))
        self.message='';self.retain_allowed=False;self._r85_protocol_liveness_enabled=False
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=1_100_000_000))
    def _known_generation_ready(self):return True
    def make_path(self,points):return np.asarray(points).tolist()
    def publish_command(self):self.zero_count+=1;self.command=[0.,0.,0.]
    def retain_admitted_path(self,*args,**kwargs):
        return dict(allowed=self.retain_allowed,reason='fixture_retention',samples_present=self.samples is not None)


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='r88-real-admission-')
        self.addCleanup(self.tmp.cleanup);self.node=Fixture(self.tmp.name)
        self.stack=ExitStack();self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(controller,'preview_path_handoff',return_value={'allowed':True,'required':False}))
        self.stack.enter_context(patch.object(controller,'staged_cascade_binding',return_value=None))
        # Future-reference causality is tested separately with the actual helper.
        self.stack.enter_context(patch.object(controller,'defer_future_path',return_value=False))
    def admit(self,**kwargs):
        self.node._admit_matched_spline(self.node.msg,self.node.metadata,**kwargs)
    def ledger(self):
        self.node.evidence.flush()
        path=self.node.run/'path_admission_decisions.jsonl'
        if path.exists():return [json.loads(line) for line in path.read_text().splitlines()]
        return [row for name,row in self.node.evidence.rows if name=='path_admission_decisions.jsonl']
    def assert_stopped(self):
        self.assertTrue(self.node.path_admission_hold)
        self.assertTrue(self.node.path_admission_replan_pending)
        self.assertGreater(self.node.zero_count,0)
        self.assertEqual(self.node.command,[0.,0.,0.])
    def assert_old_execution(self):
        self.assertEqual(self.node.active_trajectory_id,1)
        self.assertEqual(self.node.active_reference_stamp,(1,0))
        np.testing.assert_array_equal(self.node.samples,[[0.,0.,0.],[.5,0.,0.]])
        self.assertEqual(self.node.path_receipt,{'old_receipt':True})
        self.assertEqual(self.node.path_admission_active,self.node.old_active)

    def test_full_original_lower_acceptance_and_batch_settle_once(self):
        received=time.monotonic_ns()-20_000_000
        self.admit(received_wall_ns=received)
        self.assertEqual(self.node.active_trajectory_id,2)
        self.assertFalse(self.node.path_admission_hold)
        self.assertEqual(self.node.path_receipt['received_wall_ns'],received)
        self.assertEqual(self.node.r85_physical_gate.fresh_calls,2)
        rows=self.ledger();self.assertEqual(len(rows),1);self.assertTrue(rows[0]['admitted'])
        self.assertTrue((self.node.run/'cascade_path_history.jsonl').exists())
        self.assertEqual(len(self.node.path_publications),1)

    def test_association_superseded_is_original_denial_and_zero(self):
        self.node.trajectory_association.accepted=dict(self.node.value,traj_id=3)
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.node.r85_physical_gate.certify_calls,0)
        decision=self.ledger()[0]
        self.assertIn('r85_candidate_not_bound_by_schema2_association',decision['reason'])
        self.assertFalse(decision['admitted'])

    def test_physical_rejection_keeps_original_fail_closed_branch(self):
        self.node.r85_physical_gate.allowed=False
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.ledger()[0]['reason'],'r85_physical_binding_rejected')
        self.assertFalse(self.node.path_publications)

    def test_outside_fence_only_retains_explicitly_eligible_old_path(self):
        self.node.retain_allowed=True
        with patch.object(controller,'certify_curve',return_value={'allowed':False,'reason':'outside_fence'}):self.admit()
        self.assertFalse(self.node.path_admission_hold);self.assertTrue(self.node.path_admission_replan_pending)
        self.assertTrue(self.node.pid_guard_needs_evaluation);self.assert_old_execution()
        self.assertEqual(self.node.zero_count,0);self.assertFalse(self.ledger()[0]['admitted'])

    def test_original_strict_budget_counts_physical_compute(self):
        self.node.r85_physical_gate.delay=.115
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.ledger()[0]['reason'],'admission_callback_budget_exhausted')

    def test_before_lower_source_rejection_never_publishes_new_path(self):
        self.node.r85_physical_gate.reject_fresh_at=1
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.ledger()[0]['reason'],'r85_pre_execution_source_or_budget_rejected')
        self.assertFalse(self.node.path_publications)

    def test_actual_freshness_and_protection_predicates_still_deny_at_commit(self):
        mutations=(
            ('pose_header_exact_300ms',lambda n:setattr(n,'pose_stamp',800_000_000),'pose_header_fresh'),
            ('cloud_header_future',lambda n:setattr(n,'cloud_stamp',1_100_000_001),'cloud_header_fresh'),
            ('pose_wall_old',lambda n:setattr(n,'pose_updated',time.monotonic()-.31),'pose_wall_fresh'),
            ('imu_old',lambda n:setattr(n,'raw_imu',NS(fresh=lambda *args:False)),'raw_imu_fresh'),
            ('bridge_not_ready',lambda n:n.bridge_safety.update(state='hold'),'bridge_ready'),
            ('obstacle_held',lambda n:setattr(n,'obstacle_hold',True),'obstacle_not_held'),
            ('tilt_held',lambda n:setattr(n,'tilt_hold',True),'tilt_not_held'),
            ('planner_generation_changed',lambda n:n.known_scene_inputs.update(generation=2),'same_generation'))
        for name,mutate,predicate in mutations:
            with self.subTest(name=name):
                directory=self.node.run/name;directory.mkdir()
                self.node=Fixture(directory);self.node.r85_physical_gate.use_actual_fresh=True
                mutate(self.node);self.admit();self.assert_stopped();self.assert_old_execution()
                row=self.ledger()[0]
                self.assertEqual(row['reason'],'r85_pre_execution_source_or_budget_rejected')
                self.assertFalse(row['proof']['r85_before_lower_source_gate']['conditions'][predicate])
                self.assertFalse(self.node.path_publications)

    def test_final_source_rejection_restores_execution_but_keeps_lower_counter(self):
        self.node.r85_physical_gate.reject_fresh_at=2
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.node.replans,1)
        rows=self.ledger();self.assertEqual(len(rows),1);self.assertFalse(rows[0]['admitted'])
        self.assertTrue(rows[0]['lower_gate_execution_identity_restored'])
        self.assertIn('final_source_not_fresh',rows[0]['binding_error'])
        self.assertFalse((self.node.run/'cascade_path_history.jsonl').exists())

    def test_original_lower_refusal_cannot_become_execution_receipt(self):
        with patch.object(controller.TeacherNavigation,'accept_spline',return_value=None):self.admit()
        self.assert_stopped();self.assert_old_execution()
        self.assertIn('existing_execution_gate_rejected',self.ledger()[0]['binding_error'])

    def test_existing_writer_enqueue_failure_rolls_back_with_no_pass(self):
        self.node.evidence.fail_batch_enqueue=True
        self.admit();self.assert_stopped();self.assert_old_execution()
        self.assertFalse(self.node.path_admission_last_decision['admitted'])
        self.assertIn('enqueue_failure',self.node.path_admission_last_decision['binding_error'])
        self.node.evidence.flush()
        self.assertFalse((self.node.run/'cascade_path_history.jsonl').exists())
        self.assertFalse((self.node.run/'path_admission_decisions.jsonl').exists())

    def test_failure_after_state_promotion_restores_original_execution(self):
        original=controller.admission_callback_rejected
        for reason in ('admission_callback_budget_exhausted_after_state_commit',
                       'admission_callback_budget_exhausted_at_execution_release',
                       'admission_callback_budget_exhausted_at_completion_checkpoint'):
            with self.subTest(reason=reason):
                directory=Path(self.tmp.name)/reason;directory.mkdir();self.node=Fixture(directory)
                def reject(run,profile,elapsed,observed):
                    return observed==reason or original(run,profile,elapsed,observed)
                with patch.object(controller,'admission_callback_rejected',side_effect=reject):self.admit()
                self.assert_stopped();self.assert_old_execution()
                row=self.ledger()[0];self.assertFalse(row['admitted']);self.assertIn(reason,row['binding_error'])
                self.assertFalse((self.node.run/'cascade_path_history.jsonl').exists())

    def test_terminal_settle_failure_cannot_emit_earlier_success(self):
        original=controller.PreparedAdmissionBatch.settle
        def settle(batch,accepted,*args):
            if accepted:raise RuntimeError('injected_terminal_settle_failure')
            return original(batch,accepted,*args)
        with patch.object(controller.PreparedAdmissionBatch,'settle',new=settle):self.admit()
        self.assert_stopped();self.assert_old_execution()
        row=self.ledger()[0];self.assertFalse(row['admitted'])
        self.assertIn('terminal_settle_failure',row['binding_error'])
        self.assertFalse((self.node.run/'cascade_path_history.jsonl').exists())

    def test_capture_keeps_checked_old_path_without_clearing_capture(self):
        self.node.cascade=NS(capture_stamp=123,spatial_config={});self.node.cascade_key=('r88-fixture',0)
        self.node.retain_allowed=True
        self.admit();self.assert_old_execution()
        self.assertEqual(self.node.cascade.capture_stamp,123)
        self.assertEqual(self.node.zero_count,0)
        self.assertEqual(self.ledger()[0]['reason'],'terminal_capture_kept_checked_path')

    def test_resume_preserves_original_transaction_start(self):
        self.assertIn('admission_started',inspect.signature(controller.PIDNavigation._admit_matched_spline).parameters,
            'R88 integration must expose original transaction start')
        started=time.monotonic()-.115
        self.admit(admission_started=started)
        self.assert_stopped();self.assert_old_execution()
        self.assertEqual(self.ledger()[0]['reason'],'admission_callback_budget_exhausted')

    def test_resume_consumes_prepared_result_without_second_geometry_call(self):
        self.assertIn('physical_result',inspect.signature(controller.PIDNavigation._admit_matched_spline).parameters)
        self.admit(physical_result={'prepared':{'fixture_prepared':True},'proof':{'fixture_proof':True}})
        self.assertEqual(self.node.r85_physical_gate.certify_calls,0)
        self.assertEqual(self.node.r85_physical_gate.complete_calls,1)
        self.assertTrue(self.ledger()[0]['admitted'])

    def test_worker_error_uses_original_negative_admission_branch(self):
        self.admit(physical_result={'error':'injected_worker_eof'})
        self.assert_stopped();self.assert_old_execution()
        row=self.ledger()[0]
        self.assertEqual(row['reason'],'r88_physical_worker_rejected')
        self.assertEqual(self.node.r85_physical_gate.certify_calls,0)

    def test_diagnostic_budget_observation_keeps_elapsed_wait_visible(self):
        self.node.profile={'diagnostic_observe_only':{'enabled':True}}
        with patch('diagnostic_observation.record',return_value=True) as record:
            self.admit(admission_started=time.monotonic()-.115)
            row=self.ledger()[0]
        self.assertTrue(row['admitted'])
        elapsed=[call.args[4]['callback_elapsed_wall_ms'] for call in record.call_args_list
                 if len(call.args)>4 and call.args[2]=='admission_callback_elapsed']
        self.assertTrue(elapsed);self.assertTrue(all(value>=115. for value in elapsed))


class ManualWorker:
    """Deterministic transport boundary; geometry is covered separately."""
    def __init__(self):self.busy=False;self.pid=12345;self.submitted=[];self.result=None
    def submit(self,token,value):
        if self.busy:raise RuntimeError('single_flight')
        self.busy=True;self.submitted.append((token,copy.deepcopy(value)))
    def finish(self,proof,error=None):
        self.result=(self.submitted[-1][0],copy.deepcopy(proof),{'error':error} if error else {})
        self.busy=False
    def poll(self):result,self.result=self.result,None;return result
    def close(self):return {'alive':False}


class OrderedAdmissionTests(unittest.TestCase):
    """Production coordinator through the actual admission transaction."""
    def setUp(self):
        AdmissionTests.setUp(self)
        from r88_async_admission import Admission
        self.worker=ManualWorker();self.async_admission=Admission(self.node,self.worker)
        self.node._r88_admission=self.async_admission
    ledger=AdmissionTests.ledger
    assert_stopped=AdmissionTests.assert_stopped
    assert_old_execution=AdmissionTests.assert_old_execution
    def receive(self,msg):
        def associate(actual):
            value=payload(actual)
            self.node.trajectory_association.accepted=value
            self.node.metadata=dict(self.node.metadata,trajectory=value)
            self.node.accept_spline(actual,self.node.metadata)
        return self.async_admission.receive('spline',msg,associate)
    def test_fifo_negative_result_settles_before_next_positive_candidate(self):
        first=message();second=message();second.traj_id=3
        self.receive(first);first_receipt=self.async_admission.current['received_wall_ns']
        self.receive(second)
        self.assertEqual(len(self.worker.submitted),1)
        self.assertEqual(self.node.trajectory_association.accepted['traj_id'],2)
        negative=dict(self.node.r85_physical_gate.result(),allowed=False,reason='injected_negative_candidate')
        self.worker.finish(negative);self.async_admission.poll()
        self.assert_stopped();self.assert_old_execution()
        self.assertEqual(len(self.worker.submitted),2)
        self.assertEqual(self.node.trajectory_association.accepted['traj_id'],3)
        self.worker.finish(self.node.r85_physical_gate.result());self.async_admission.poll()
        self.assertEqual(self.node.active_trajectory_id,3);self.assertFalse(self.node.path_admission_hold)
        rows=self.ledger()
        # Early denial ledger uses the in-memory append fixture; success uses
        # actual PreparedAdmissionBatch output. Both must remain observable.
        rejected=[row for name,row in self.node.evidence.rows
                  if name=='path_admission_decisions.jsonl' and row['association_payload_id']==2]
        self.assertEqual(len(rejected),1);self.assertFalse(rejected[0]['admitted'])
        self.assertTrue(rows[-1]['admitted']);self.assertEqual(rows[-1]['association_payload_id'],3)
        consumed=[row for name,row in self.node.evidence.rows
                  if name=='r88_async_admission.jsonl' and row['event']=='result_consumed']
        self.assertEqual(len(consumed),2)
        self.assertEqual(consumed[0]['original_received_wall_ns'],first_receipt)
        self.assertEqual(self.node.pose_stamp,1_000_000_000)
    def test_changed_task_result_never_modifies_new_task_state(self):
        self.receive(message());self.node.request_id='new-task'
        self.node.path_admission_hold=True;self.node.path_admission_replan_pending=False
        self.node.message='new-task-sentinel'
        self.worker.finish(self.node.r85_physical_gate.result());self.async_admission.poll()
        self.assertEqual(self.async_admission.counts['invalidated'],1)
        self.assertEqual(self.node.message,'new-task-sentinel')
        self.assertTrue(self.node.path_admission_hold);self.assertFalse(self.node.path_admission_replan_pending)
        self.assertEqual(self.node.zero_count,0);self.assertEqual(self.node.replans,0)
        self.assertFalse(self.node.path_publications)
    def test_changed_source_epoch_discards_result_without_refresh(self):
        self.receive(message());self.node.known_scene_commit_token=('new-producer',1)
        before=(self.node.pose_updated,self.node.cloud_updated,self.node.pose_stamp,self.node.cloud_stamp)
        self.worker.finish(self.node.r85_physical_gate.result());self.async_admission.poll()
        self.assertEqual(self.async_admission.counts['invalidated'],1)
        self.assertEqual(before,(self.node.pose_updated,self.node.cloud_updated,self.node.pose_stamp,self.node.cloud_stamp))
        self.assertEqual(self.node.replans,0)


class ExistingFutureCausalityTests(unittest.TestCase):
    """Legacy schema1 helper only; schema2 support is not claimed or changed."""
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='r88-future-causality-')
        self.addCleanup(self.tmp.cleanup);self.node=Fixture(self.tmp.name)
        self.node.trajectory_association.reference_stamp=[2,0]
        from path_admission import payload as legacy_payload
        self.metadata=dict(schema=1,trajectory=legacy_payload(self.node.msg),
            reference_stamp=[2,0],body_goal=[2.,0.,0.])
        self.node._admit_matched_spline=Mock();self.node.retain_allowed=True
    def test_original_receipt_survives_actual_source_wait(self):
        from pending_path_commit import defer_future,commit_ready
        received=time.monotonic_ns()-30_000_000
        self.assertTrue(defer_future(self.node,self.node.msg,self.metadata,received_wall_ns=received))
        self.assertEqual(self.node.pending_path_commit['received_wall_ns'],received)
        self.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=9_000_000_000))
        self.assertFalse(commit_ready(self.node));self.node._admit_matched_spline.assert_not_called()
        self.node.pose_stamp=self.node.cloud_stamp=2_000_000_000
        self.assertTrue(commit_ready(self.node))
        self.assertEqual(self.node._admit_matched_spline.call_args.kwargs['received_wall_ns'],received)
        self.assertIsNone(self.node.pending_path_commit)
    def test_changed_generation_discards_future_without_restarting_admission(self):
        from pending_path_commit import defer_future,commit_ready
        self.assertTrue(defer_future(self.node,self.node.msg,self.metadata))
        self.node.known_scene_inputs['generation']=2
        self.node.pose_stamp=self.node.cloud_stamp=2_000_000_000
        self.assertFalse(commit_ready(self.node));self.node._admit_matched_spline.assert_not_called()
        self.assertIsNone(self.node.pending_path_commit)


if __name__=='__main__':unittest.main(verbosity=2)
