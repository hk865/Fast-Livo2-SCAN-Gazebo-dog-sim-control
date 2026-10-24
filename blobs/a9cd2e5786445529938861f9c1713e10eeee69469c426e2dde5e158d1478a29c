"""Finite CPU recovery contracts, including a real R25 rejected proposal.

Synthetic clocks, transport receipts and stop booleans are not runtime evidence.
"""
import ast,copy,io,json,sys,tempfile,time,unittest
from collections import OrderedDict
from pathlib import Path
from types import SimpleNamespace
import numpy as np
HERE=Path(__file__).resolve().parent
FIXTURES=Path('/opt/experiment/history/2026-10-07/task/go2_engineering_v33/R26_STALE_TRANSACTION_RECOVERY_STAGE')
V=Path('/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation/engineering_v34_continuous_known_map')
sys.path[:0]=[str(HERE),str(V)]
import known_scene_runtime as runtime
from known_scene_stale_recovery import recovery_proposal,CONTRACT

def fixture():
    d=json.loads((FIXTURES/'R25_PENDING_PROPOSAL_CPU_FIXTURE.json').read_text())
    c=runtime.CorrectionState(initial_c=d['result']['input_C_world_odom'])
    c.target=np.asarray(d['old_target']);c.generation=333;c.last_match_ns=d['last_match_ns'];c.last_tick_ns=299_000_000_000
    wall=time.monotonic();run_id='finite_CPU_fixture'
    scan=dict(schema='known_scene_SCAN_epoch_ready/v1',run_id=run_id,ready=False,monotonic_wall=wall,ros_clock_ns=c.last_tick_ns)
    args=dict(result=d['result'],correction=c,pending_target=d['old_target'],pending_body=d['old_body'],
        pending_source_ns=d['old_source_ns'],recovery_count=0,clock_ns=c.last_tick_ns,
        complete_source_ns=c.last_tick_ns,complete_received_wall=wall,now_wall=wall,
        scan_status=scan,run_id=run_id,witness_sha256=d['result']['provenance']['witness_sha256'])
    return args

class RecoveryTests(unittest.TestCase):
    def test_actual_R25_proposal_remains_rejected_by_original_epoch_gate(self):
        a=fixture();r=a['result'];c=a['correction']
        self.assertFalse(runtime.proposal_epoch_valid(r,c.generation,np.asarray(a['pending_target']),True))
        event=recovery_proposal(**a)
        self.assertIsNotNone(event);self.assertFalse(event['successful_release'])
        self.assertGreater(event['old_residual']['frozen_body_m'],.005)
        self.assertEqual(event['new_attempt_source_ns'],298620000000)
    def test_recovery_proposal_does_not_mutate_any_state_or_target(self):
        a=fixture();c=a['correction'];before=copy.deepcopy(c.__dict__)
        recovery_proposal(**a)
        for k,v in before.items():
            if isinstance(v,np.ndarray):np.testing.assert_array_equal(v,c.__dict__[k])
            else:self.assertEqual(v,c.__dict__[k])
    def test_original_stale_boundary_is_strict_greater_than3s(self):
        a=fixture();a['correction'].last_tick_ns=a['correction'].last_match_ns+3_000_000_000
        self.assertIsNone(recovery_proposal(**a))
    def test_new_proposal_age_limit_and_no_future_stamp(self):
        for ns in (299000000001,295999999999):
            a=fixture();a['result']['source_ns']=ns;a['result']['T_odom_body_source_ns']=ns
            self.assertIsNone(recovery_proposal(**a))
    def test_latest_exact_bundle_must_be_fresh_in_source_and_wall(self):
        for field,value in [('complete_source_ns',298700000000),('complete_source_ns',299000000001),('complete_received_wall',0.)]:
            a=fixture();a[field]=value;self.assertIsNone(recovery_proposal(**a))
    def test_SCAN_ready_missing_wrong_run_or_stale_receipt_rejects(self):
        for mutate in [dict(ready=True),dict(ready=None),dict(run_id='other'),dict(monotonic_wall=0.),dict(ros_clock_ns=299000000001)]:
            a=fixture();a['scan_status'].update(mutate);self.assertIsNone(recovery_proposal(**a))
    def test_exact_current_parent_and_input_C_required(self):
        a=fixture();a['result']['parent_generation']-=1;self.assertIsNone(recovery_proposal(**a))
        a=fixture();a['result']['input_C_world_odom'][0][3]+=.000001;self.assertIsNone(recovery_proposal(**a))
        a=fixture();del a['result']['input_C_world_odom'];self.assertIsNone(recovery_proposal(**a))
    def test_same_source_body_and_C_To_Tw_identity_required(self):
        a=fixture();a['result']['T_odom_body_source_ns']+=1;self.assertIsNone(recovery_proposal(**a))
        a=fixture();a['result']['T_world_body'][0][3]+=.01;self.assertIsNone(recovery_proposal(**a))
    def test_provenance_witness_and_no_forbidden_prior_required(self):
        for key in ['point_time_verified','post_lio_association_verified','source_hashes_verified','no_robot_truth_pose','integer_stamp_join','source_membership_verified']:
            a=fixture();a['result']['provenance'][key]=False;self.assertIsNone(recovery_proposal(**a))
        a=fixture();a['witness_sha256']='0'*64;self.assertIsNone(recovery_proposal(**a))
        for key in ['robot_truth_pose_used','IMU_world_orientation_used','frontend_height_used_as_hypothesis']:
            a=fixture();a['result'][key]=True;self.assertIsNone(recovery_proposal(**a))
    def test_accepted_independently_confirmed_quality_must_pass(self):
        for key in ['accepted','evidence_verified']:
            a=fixture();a['result'][key]=False;self.assertIsNone(recovery_proposal(**a))
        a=fixture();a['result']['confirmations']=1;self.assertIsNone(recovery_proposal(**a))
        a=fixture();a['result']['best']['rejections']=['ray_p95'];self.assertIsNone(recovery_proposal(**a))
    def test_original_frozen_target_and_finite_retry_cap_required(self):
        a=fixture();a['correction'].target[0,3]+=.01;self.assertIsNone(recovery_proposal(**a))
        for n in [-1,2,True]:
            a=fixture();a['recovery_count']=n;self.assertIsNone(recovery_proposal(**a))
    def callback_fixture(self,run,stop=True,running=True,retries=0):
        a=fixture();a['scan_status']['run_id']=run.name
        (run/'scan_epoch_ready.json').write_text(json.dumps(a['scan_status']))
        (run/'navigation_status.json').write_text(json.dumps(dict(state='running' if running else 'failed')))
        tree=ast.parse((HERE/'known_scene_runtime.py').read_text())
        main=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='main')
        klass=next(x for x in main.body if isinstance(x,ast.ClassDef) and x.name=='Runtime')
        fn=copy.deepcopy(next(x for x in klass.body if isinstance(x,ast.FunctionDef) and x.name=='tick'))
        namespace=dict(runtime.__dict__);namespace.update(run=run,String=SimpleNamespace)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(HERE/'known_scene_runtime.py'),'exec'),namespace)
        c=a['correction'];body=object();wall=a['now_wall']
        o=SimpleNamespace(loaded_witness=True,worker=SimpleNamespace(poll=lambda:copy.deepcopy(a['result'])),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=a['clock_ns'])),
            last_result=None,history=io.StringIO(),initialized=True,correction=c,
            pending_target=np.asarray(a['pending_target']),pending_body=np.asarray(a['pending_body']),
            pending_source_ns=a['pending_source_ns'],pending_recovery_count=retries,transition=True,
            last_complete_body=(body,wall,a['complete_source_ns']),slots=OrderedDict(),
            idle_or_stopped=lambda b,initial=False:b is body and stop,
            transaction_count=6,join=SimpleNamespace(witness={'witness_sha256':a['witness_sha256']}),
            last_submit=a['clock_ns'],bundle_count=10,error=None,status_pub=SimpleNamespace(publish=lambda m:None),
            decision_history=io.StringIO(),stop_since_ns=298900000000)
        return o,namespace['tick'],a
    def test_actual_callback_explicit_abort_new_frozen_target_without_release_or_C_change(self):
        with tempfile.TemporaryDirectory(prefix='go2_R26_recovery_cpu_')as folder:
            o,tick,a=self.callback_fixture(Path(folder));before=o.correction.applied.copy();tick(o)
            self.assertTrue(o.transition);self.assertEqual(o.transaction_count,6)
            np.testing.assert_array_equal(before,o.correction.applied);self.assertEqual(o.correction.generation,333)
            np.testing.assert_array_equal(o.pending_target,a['result']['C_world_odom'])
            np.testing.assert_array_equal(o.pending_body,a['result']['T_odom_body'])
            self.assertEqual(o.pending_source_ns,298620000000);self.assertEqual(o.pending_recovery_count,1)
            self.assertEqual(o.correction.last_match_ns,298620000000)
            events=[json.loads(x)for x in o.history.getvalue().splitlines()]
            self.assertEqual(sum(x.get('event')=='expired_transaction_ABORTED_replacement_proposed' for x in events),1)
            self.assertIsNone(o.stop_since_ns)
    def test_callback_no_physical100ms_stop_or_failed_mission_cannot_replace_or_renew(self):
        for stop,running in [(False,True),(True,False)]:
            with tempfile.TemporaryDirectory(prefix='go2_R26_recovery_cpu_')as folder:
                o,tick,a=self.callback_fixture(Path(folder),stop=stop,running=running);target=o.pending_target.copy();tick(o)
                self.assertTrue(o.transition);self.assertEqual(o.correction.last_match_ns,a['correction'].last_match_ns)
                self.assertEqual(o.correction.last_match_ns,294560000000)
                np.testing.assert_array_equal(o.pending_target,target);self.assertNotIn('ABORTED',o.history.getvalue())
    def test_callback_retries_capped_without_deadline_reset(self):
        with tempfile.TemporaryDirectory(prefix='go2_R26_recovery_cpu_')as folder:
            o,tick,a=self.callback_fixture(Path(folder),retries=2);tick(o)
            self.assertEqual(o.correction.last_match_ns,294560000000);self.assertEqual(o.pending_recovery_count,2)
            self.assertEqual(o.transaction_count,6);self.assertNotIn('ABORTED',o.history.getvalue())
        self.assertFalse(CONTRACT['deadline_reset'])
    def test_original_consistent_fresh_proposal_renews_same_target_without_ABORT(self):
        with tempfile.TemporaryDirectory(prefix='go2_R26_recovery_cpu_')as folder:
            o,tick,a=self.callback_fixture(Path(folder));target=o.pending_target.copy()
            result=copy.deepcopy(a['result']);result['C_world_odom']=target.tolist()
            result['T_world_body']=(redacted-contact@example.invalid(result['T_odom_body'])).tolist()
            o.worker.poll=lambda:result;tick(o)
            self.assertTrue(o.transition);self.assertEqual(o.pending_recovery_count,0)
            self.assertEqual(o.pending_source_ns,292550000000)
            self.assertEqual(o.correction.last_match_ns,298620000000)
            np.testing.assert_array_equal(o.pending_target,target)
            self.assertNotIn('ABORTED',o.history.getvalue());self.assertEqual(o.transaction_count,6)
    def test_original_continuation_convergence_stop_and_C_rate_functions_byte_AST_identical(self):
        def node(path,name):
            tree=ast.parse(path.read_text())
            return next(x for x in ast.walk(tree) if isinstance(x,ast.FunctionDef) and x.name==name)
        for name in ['proposal_epoch_valid','transaction_convergence','idle_or_stopped','publish_bundle']:
            actual=node(HERE/'known_scene_runtime.py',name);old=node(FIXTURES/'known_scene_runtime_R25_before.txt',name)
            if name=='publish_bundle':
                # Sole addition retains one exact small body witness after publication.
                actual.body=[x for x in actual.body if not (isinstance(x,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='last_complete_body' for t in x.targets))]
            self.assertEqual(ast.dump(actual),ast.dump(old),name)

if __name__=='__main__':unittest.main(verbosity=2)
