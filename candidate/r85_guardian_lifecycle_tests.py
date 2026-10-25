"""Pure CPU lifecycle bounds; no ROS, Actor, subprocess launch or physics."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import guardian_archive_shutdown as subject
import owned_guardian as guardian
import r85_session_guard as session_guard


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,seconds):self.t+=seconds


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='r85_guardian_fixture_')
        self.run=Path(self.temp.name)
        self.parent=dict(pid=1,start_ticks=1,pgid=1,session=1,boot_id='fixture')
        self.worker=dict(pid=2,start_ticks=2,pgid=2,session=2,boot_id='fixture')
        self.roles=dict(worker=self.worker,gazebo=dict(pid=3))
        self.cleanup=dict(gazebo=dict(remaining_members=[]))
        self.pipeline=dict(normal_completed=True,error=None,gate_errors=[],summary=dict(
            normal_completed=True,accepted=1,delivered=1,committed=1,pending=0,
            inflight=0,ready=0,bytes=0,failure='',uncommitted_packets=[]))
        self.plan=dict(schema='student_pct_scan_closed_loop_runtime_plan/v1',
            wall_budget_s=4590,duration_s=1500,profile=dict(duration_s=1500,
            actor_kind='student',controller_kind='student',navigation_ground_truth_used=False,
            max_speed_mps=.2,cascade=dict(desired_speed=.2),
            r85_session=dict(enabled=True,schema='r85_pct_scan_student_session/v1',
                deadline_utc=subject.R85_DEADLINE_UTC,simulation_only=True,
                maximum_retained_bytes=130_000_000_000,maximum_round_bytes=10_000_000_000),
            r85_task=dict(kind='f1_to_f3',maximum_task_sim_s=1500,
                          progress_window_sim_s=90,minimum_progress_m=.5)))
        self.owner=dict(parent=self.parent,wall_deadline=100.)
        self.freeze()
        (self.run/'actuator.jsonl').write_text(json.dumps(dict(kind='physics_step',
            t=223.98,dt=.005,iteration=1,mode='active',fault=0,terminating=False))+'\n')
        self.snapshot=dict(hard_bytes=130_000_000_000,soft_stop_bytes=129_463_129_088,
            stop_reasons=[],conservative_session_growth_bytes=49_000_000_000,
            current_retained_budget_bytes=49_000_000_000,
            budget_basis='current_retained_experiment_data',historical_counter_used_for_capacity=False,
            R85_round_budget=dict(hard_B=10_000_000_000,hard_stop=False,
                predictive_soft_stop=False,projected_full_stop_B=5_000_000_000))
    def tearDown(self):self.temp.cleanup()
    def freeze(self):
        for name,key,data in [('source_manifest.json','source_manifest_sha256',{}),
                ('r85_task_contract.json','r85_task_contract_sha256',{'run_id':self.run.name})]:
            raw=json.dumps(data).encode();(self.run/name).write_bytes(raw)
            self.plan[key]=hashlib.sha256(raw).hexdigest()
        (self.run/'input_profile.json').write_text(json.dumps(self.plan['profile']))
        raw=json.dumps(self.plan).encode();(self.run/'runtime_plan.json').write_bytes(raw)
        self.owner['runtime_plan_sha256']=hashlib.sha256(raw).hexdigest()
    def observe(self,*,exit_after=.2,snapshot=None,reason='parent_identity_lost',wall=100.):
        self.clock=Clock();owner={**self.owner,'wall_deadline':wall}
        def identities(pid):return self.worker if pid==2 else None
        def members(saved):
            return [self.worker]if saved.get('pid')==2 and self.clock.t<exit_after else []
        return subject.observe_archive_close(self.run,reason,owner,self.plan,self.roles,
            self.cleanup,self.pipeline,lambda previous:copy.deepcopy(snapshot or self.snapshot),
            None,identities,members,usage_check=lambda:dict(logical_bytes=1000,allocated_bytes=4096),
            clock=self.clock.now,sleep=self.clock.sleep)
    def test_bound_r85_parent_loss_waits_for_eof_without_pass_or_signals(self):
        r=self.observe()
        self.assertEqual(r['reason'],'worker_owned_group_exit_observed')
        self.assertTrue(r['group_exit_observed']);self.assertAlmostEqual(r['observed_wait_wall_s'],.2)
        self.assertEqual(r['signals_sent_by_observer'],[])
        self.assertEqual(r['worker_returncode'],'UNVERIFIED');self.assertFalse(r['navigation_verified'])
    def test_stuck_worker_never_exceeds_30_seconds(self):
        r=self.observe(exit_after=1000)
        self.assertEqual(r['reason'],'archive_grace_expired');self.assertEqual(self.clock.t,30.)
    def test_wall_or_hard_stop_never_earns_extra_wait(self):
        self.assertEqual(self.observe(wall=0.)['observed_wait_wall_s'],0.)
        self.assertEqual(self.observe(reason='independent_wall_budget')['observed_wait_wall_s'],0.)
    def test_changed_bounds_or_binding_rejected_before_sleep(self):
        for field,value in [('wall_budget_s',4591),('duration_s',1501)]:
            with self.subTest(field=field):
                original=self.plan[field];self.plan[field]=value
                self.assertIn('archive_observation_rejected',self.observe()['reason'])
                self.assertEqual(self.clock.t,0.);self.plan[field]=original
        (self.run/'source_manifest.json').write_text('{"changed":true}')
        self.assertIn('binding_changed',self.observe()['reason']);self.assertEqual(self.clock.t,0.)
    def test_round_predictive_stop_or_reserve_vetoes_wait(self):
        for changes in [dict(predictive_soft_stop=True),dict(projected_full_stop_B=9_850_000_000)]:
            snap=copy.deepcopy(self.snapshot);snap['R85_round_budget'].update(changes)
            self.assertEqual(self.observe(snapshot=snap)['reason'],
                'cancel_R85_current_retained_or_round_archive_reserve')
            self.assertEqual(self.clock.t,0.)
    def test_live_physics_or_undrained_pipeline_vetoes_wait(self):
        self.cleanup['gazebo']['remaining_members']=[3]
        self.assertIn('preceding_cleanup_not_confirmed',self.observe()['reason'])
        self.cleanup['gazebo']['remaining_members']=[]
        self.pipeline['normal_completed']=False
        self.assertEqual(self.observe()['reason'],'skip_pipeline_normal_drain_not_confirmed')
    def test_shared_hard_or_unknown_metric_rejected(self):
        snap=copy.deepcopy(self.snapshot);snap['hard_bytes']=80_000_000_000
        self.assertEqual(self.observe(snapshot=snap)['reason'],'cancel_original_storage_gate_or_unknown_snapshot')
        snap=copy.deepcopy(self.snapshot);snap['historical_counter_used_for_capacity']=True
        self.assertEqual(self.observe(snapshot=snap)['reason'],'cancel_R85_current_retained_or_round_archive_reserve')
    def test_terminal_normal_release_reports_only_existing_facts(self):
        def release(run,storage):
            guardian.atomic(run/'guardian_result.json',dict(status='parent_cleanup_verified',
                all_owned_groups_absent=True,original_runner_exitcode='not_recorded_by_guardian'))
            return 0
        with patch.object(guardian,'_watch',release):self.assertEqual(guardian.watch(self.run),0)
        r=json.loads((self.run/'terminal_facts.json').read_text())
        self.assertTrue(r['all_owned_groups_absent']);self.assertFalse(r['navigation_verified'])
        self.assertFalse(r['artifacts']['worker_result.json']['exists'])
        self.assertFalse(r['worker_PASS_created'])
    def test_terminal_unexpected_exception_keeps_unknown_exit_and_missing_archive(self):
        with patch.object(guardian,'_watch',side_effect=RuntimeError('fixture_failure')):
            with self.assertRaisesRegex(RuntimeError,'fixture_failure'):guardian.watch(self.run)
        r=json.loads((self.run/'terminal_facts.json').read_text())
        self.assertIn('fixture_failure',r['observer_error']);self.assertIsNone(r['all_owned_groups_absent'])
        self.assertFalse(r['artifacts']['observations_actions.npz']['exists'])
        self.assertFalse(r['original_artifacts_repaired'])
    def test_deadline_and_finish_reserve_are_consistent(self):
        self.assertEqual(session_guard.DEADLINE.isoformat(),subject.R85_DEADLINE_UTC)
        self.assertEqual(session_guard.FINISH_RESERVE_S,1200)
        for filename in ['r85_f1_f3.json','r85_prefix_chain.json']:
            p=json.loads((Path(__file__).parent/'profiles'/filename).read_text())
            self.assertEqual(p['r85_session']['deadline_utc'],subject.R85_DEADLINE_UTC)


if __name__=='__main__':unittest.main()
