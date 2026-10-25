"""Finite CPU positive/negative archive-lifecycle fixtures; no ROS/Actor."""
import json
import ast
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
import zipfile

import worker_archive_shutdown as subject


class Clock:
    def __init__(self): self.t = 0.0
    def now(self): return self.t
    def sleep(self, seconds): self.t += seconds


class Process:
    pid = 42
    def __init__(self, clock, exit_after=None, rc=0):
        self.clock, self.exit_after, self.rc = clock, exit_after, rc
    def poll(self):
        return self.rc if self.exit_after is not None and self.clock.t >= self.exit_after else None


SAVED = dict(pid=42, start_ticks=11, pgid=42, session=42, boot_id='fixture')
PHYSICS_STOPPED = dict(started=True, returncode=0, remaining_owned_group_members=[])


def npy(samples, width, *, dtype='<f4', shape=None):
    header = repr(dict(descr=dtype, fortran_order=False,
                       shape=shape if shape is not None else (samples, width)))
    header += ' '*((64-(10+len(header)+1)%64)%64)+'\n'
    return b'\x93NUMPY\x01\x00'+len(header).to_bytes(2, 'little')+header.encode()+b'\x00'*(samples*width*4)


def artifact(root, *, samples=3, missing=None, duplicate=False, corrupt=False,
             width=247, dtype='<f4', shape=None):
    (root/'worker_result.json').write_text(json.dumps(dict(completed=True, samples=samples, fault=None)))
    path = root/'observations_actions.npz'
    with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        if missing != 'observations.npy':
            archive.writestr('observations.npy', npy(samples, width, dtype=dtype, shape=shape))
        if missing != 'actions.npy': archive.writestr('actions.npy', npy(samples, 12))
        if duplicate: archive.writestr('actions.npy', npy(samples, 12))
    if corrupt: path.write_bytes(path.read_bytes()[:-10])


class ShutdownTests(unittest.TestCase):
    def wait(self, *, exit_after=None, rc=0, saved=SAVED, physics=PHYSICS_STOPPED,
             ident=lambda pid: SAVED, validator=lambda saved: [], abort=lambda: None):
        clock = Clock()
        proc = Process(clock, exit_after, rc)
        result = subject.await_archive_close(proc, saved, physics, ident, validator,
                                            abort_reason=abort, clock=clock.now, sleep=clock.sleep)
        self.assertEqual(result['sent_signals'], [])
        self.assertLessEqual(clock.t, subject.ARCHIVE_GRACE_S)
        return result

    def test_normal_close_is_observed_without_interrupt(self):
        result = self.wait(exit_after=.2)
        self.assertTrue(result['natural_worker_exit'])
        self.assertEqual(result['worker_returncode'], 0)
        self.assertAlmostEqual(result['waited_wall_s'], .2)

    def test_existing_nonzero_exit_is_preserved(self):
        result = self.wait(exit_after=0, rc=9)
        self.assertEqual(result['worker_returncode'], 9)

    def test_stuck_worker_hits_30_second_bound(self):
        result = self.wait()
        self.assertEqual(result['reason'], 'archive_grace_expired')
        self.assertEqual(result['waited_wall_s'], 30)

    def test_live_physics_does_not_earn_wait(self):
        result = self.wait(physics={**PHYSICS_STOPPED, 'returncode':None})
        self.assertEqual(result['waited_wall_s'], 0)

    def test_residual_physics_group_does_not_earn_wait(self):
        result = self.wait(physics={**PHYSICS_STOPPED, 'remaining_owned_group_members':[123]})
        self.assertEqual(result['reason'], 'native_physics_stop_not_confirmed')

    def test_no_saved_identity_does_not_earn_wait(self):
        self.assertEqual(self.wait(saved=None)['reason'], 'worker_identity_unavailable')

    def test_reused_pid_is_rejected(self):
        self.assertEqual(self.wait(ident=lambda pid:{**SAVED,'start_ticks':12})['reason'],
                         'worker_identity_mismatch')

    def test_missing_live_identity_is_rejected(self):
        self.assertEqual(self.wait(ident=lambda pid:None)['reason'], 'live_worker_identity_unavailable')

    def test_foreign_group_validator_rejection_is_preserved(self):
        def reject(saved): raise RuntimeError('foreign group fixture')
        self.assertIn('foreign group fixture', self.wait(validator=reject)['reason'])

    def test_storage_gate_cancels_before_any_wait(self):
        self.assertIn('storage hard gate', self.wait(abort=lambda:'storage hard gate')['reason'])

    def test_guardian_failure_mid_close_is_bounded(self):
        clock = Clock(); proc = Process(clock)
        r = subject.await_archive_close(proc, SAVED, PHYSICS_STOPPED, lambda pid:SAVED, lambda saved:[],
            abort_reason=lambda:'guardian gate' if clock.t >= .1 else None,
            clock=clock.now, sleep=clock.sleep)
        self.assertEqual(r['waited_wall_s'], .1)
        self.assertIn('guardian gate', r['reason'])

    def test_unstarted_worker_is_explicit(self):
        r = subject.await_archive_close(None, None, {}, lambda pid:None, lambda saved:[])
        self.assertEqual(r['reason'], 'worker_not_started')


class IntegrityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='r41_archive_fixture_')
        self.root = Path(self.temp.name)
    def tearDown(self): self.temp.cleanup()
    def inspect(self): return subject.inspect_worker_archive(self.root)

    def test_valid_arrays_crc_shapes_and_samples(self):
        artifact(self.root)
        r = self.inspect()
        self.assertTrue(r['verified'], r['error'])
        self.assertEqual(r['arrays']['observations.npy']['shape'], [3,247])
        self.assertTrue(all(a['crc_verified'] for a in r['arrays'].values()))

    def test_missing_result_is_rejected_without_repair(self):
        artifact(self.root); (self.root/'worker_result.json').unlink()
        self.assertFalse(self.inspect()['verified'])
        self.assertFalse((self.root/'worker_result.json').exists())

    def test_truncated_zip_is_rejected_without_changing_bytes(self):
        artifact(self.root, corrupt=True); before=(self.root/'observations_actions.npz').read_bytes()
        self.assertFalse(self.inspect()['verified'])
        self.assertEqual(before, (self.root/'observations_actions.npz').read_bytes())

    def test_payload_crc_mismatch_is_rejected(self):
        artifact(self.root)
        path = self.root/'observations_actions.npz'
        with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_STORED) as archive:
            archive.writestr('observations.npy', npy(3,247))
            archive.writestr('actions.npy', npy(3,12))
        raw = bytearray(path.read_bytes())
        with zipfile.ZipFile(path) as archive:
            entry = archive.getinfo('observations.npy')
            start = entry.header_offset+30+len(entry.filename.encode())
            raw[start+entry.file_size-1] ^= 1
        path.write_bytes(raw)
        r = self.inspect()
        self.assertFalse(r['verified'])
        self.assertIn('CRC', r['error'])

    def test_missing_array_is_rejected(self):
        artifact(self.root, missing='actions.npy'); self.assertFalse(self.inspect()['verified'])

    def test_duplicate_member_is_rejected(self):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore'); artifact(self.root, duplicate=True)
        self.assertFalse(self.inspect()['verified'])

    def test_wrong_observation_width_is_rejected(self):
        artifact(self.root, width=246); self.assertFalse(self.inspect()['verified'])

    def test_object_dtype_is_rejected(self):
        artifact(self.root, dtype='|O8'); self.assertFalse(self.inspect()['verified'])

    def test_declared_shape_payload_mismatch_is_rejected(self):
        artifact(self.root, shape=(4,247)); self.assertFalse(self.inspect()['verified'])

    def test_boolean_shape_dimension_is_rejected_even_with_valid_crc(self):
        artifact(self.root, samples=1, shape=(True,247)); self.assertFalse(self.inspect()['verified'])

    def test_float_shape_dimension_is_rejected_even_with_valid_crc(self):
        artifact(self.root, samples=1, shape=(1.0,247)); self.assertFalse(self.inspect()['verified'])

    def test_nonpositive_samples_are_rejected(self):
        artifact(self.root); (self.root/'worker_result.json').write_text('{"samples":0,"completed":true}')
        self.assertFalse(self.inspect()['verified'])

    def test_boolean_sample_count_is_rejected(self):
        artifact(self.root); (self.root/'worker_result.json').write_text('{"samples":true,"completed":true}')
        self.assertFalse(self.inspect()['verified'])

    def test_fault_metadata_does_not_become_navigation_pass(self):
        artifact(self.root); (self.root/'worker_result.json').write_text('{"samples":3,"completed":true,"fault":"EOFError"}')
        r=self.inspect(); self.assertTrue(r['verified']); self.assertEqual(r['worker_fault'],'EOFError')
        self.assertNotIn('navigation_PASS', r)

    def test_symlink_artifact_is_rejected(self):
        artifact(self.root); target=self.root/'preserved.npz'
        (self.root/'observations_actions.npz').rename(target)
        (self.root/'observations_actions.npz').symlink_to(target)
        self.assertFalse(self.inspect()['verified'])

    def test_real_fixture_eof_finishes_archive_before_owner_escalation(self):
        # Tiny standalone archival process, no simulator, Actor, ROS or signals.
        payload = {name: npy(3,width).hex() for name,width in [('observations.npy',247),('actions.npy',12)]}
        code = '''import sys,time,json,zipfile
from pathlib import Path
root=Path(sys.argv[1]); payload=json.loads(sys.argv[2])
print('ready',flush=True)
sys.stdin.buffer.read()
time.sleep(.05)
with zipfile.ZipFile(root/'observations_actions.npz','w',compression=zipfile.ZIP_DEFLATED)as archive:
 for name,value in payload.items():archive.writestr(name,bytes.fromhex(value))
(root/'worker_result.json').write_text(json.dumps(dict(completed=True,samples=3,fault='fixture_native_EOF')))
'''
        proc = subprocess.Popen([sys.executable, '-B', '-c', code, str(self.root), json.dumps(payload)],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(proc.stdout.readline(), b'ready\n')
        saved = {**SAVED, 'pid':proc.pid}
        proc.stdin.close()
        result = subject.await_archive_close(proc, saved, PHYSICS_STOPPED,
                                              lambda pid:saved, lambda saved:[])
        proc.wait(timeout=1)
        proc.stdout.close(); stderr=proc.stderr.read(); proc.stderr.close()
        self.assertEqual(proc.returncode, 0, stderr)
        self.assertEqual(result['sent_signals'], [])
        self.assertTrue(result['natural_worker_exit'])
        self.assertTrue(self.inspect()['verified'])


class ActualRunnerClassificationTests(unittest.TestCase):
    def check_reason(self, reason=None, *, returncode=None, sample=False,
                     storage_error=None, guardian_error=None, expired_wall=False,
                     pipeline_complete=True, cleanup=None):
        # Compile the real staged runner's closure, rather than a copied rule.
        tree = ast.parse((Path(__file__).parent/'run.py').read_text())
        matches = [node for node in ast.walk(tree)
                   if isinstance(node, ast.FunctionDef) and node.name=='archive_abort_reason']
        self.assertEqual(len(matches), 1)
        namespace = dict(startup_stop=None if reason is None else {'reason':reason},
                         sample_complete=sample,
                         children={'worker':types.SimpleNamespace(poll=lambda:returncode)},
                         watchdog=types.SimpleNamespace(error=storage_error),
                         guardian=types.SimpleNamespace(error=guardian_error),
                         pipeline_stop={'normal_completed':pipeline_complete},
                         plan={'profile':{}},cpu_archive_budget={'eligible':False},
                         cleanup={} if cleanup is None else cleanup,
                         time=types.SimpleNamespace(monotonic=lambda:1.0),
                         archive_deadlines={'runner':0.0 if expired_wall else 100.0})
        exec(compile(ast.Module(body=matches,type_ignores=[]),'actual_runner_grace_closure','exec'),namespace)
        return namespace['archive_abort_reason']()

    def test_original_mission_failure_allows_postphysics_archive_close(self):
        self.assertIsNone(self.check_reason('actual_original_mission_failed'))

    def test_original_navigation_failure_allows_postphysics_archive_close(self):
        self.assertIsNone(self.check_reason('actual_original_navigation_failed'))

    def test_physical_fault_cannot_earn_archive_wait(self):
        self.assertIsNotNone(self.check_reason('actual_original_native_fault'))

    def test_output_cap_cannot_earn_archive_wait(self):
        self.assertIsNotNone(self.check_reason('new_output_cap'))

    def test_original_native_and_wall_cutoffs_cannot_earn_archive_wait(self):
        for reason in ['native1500s_original_route_total_cutoff','wall9090s_original_route_total_cutoff']:
            with self.subTest(reason=reason): self.assertIsNotNone(self.check_reason(reason))

    def test_initialization_rejection_and_exception_cannot_earn_wait(self):
        for reason in ['actual_initialization_nonrecoverable_rejection',None,'unknown']:
            with self.subTest(reason=reason): self.assertIsNotNone(self.check_reason(reason))

    def test_storage_guardian_and_wall_gate_override_normal_failure(self):
        for changes in [{'storage_error':'budget'},{'guardian_error':'lost'},{'expired_wall':True}]:
            with self.subTest(changes=changes):
                self.assertIsNotNone(self.check_reason('actual_original_mission_failed',**changes))

    def test_already_exited_worker_and_completed_sampler_need_no_signal(self):
        self.assertIsNone(self.check_reason(returncode=0))
        self.assertIsNone(self.check_reason(sample=True))

    def test_failed_pipeline_drain_cancels_historical_mission_failure_grace(self):
        self.assertIsNotNone(self.check_reason('actual_original_mission_failed',pipeline_complete=False))

    def test_remaining_foreign_or_unconfirmed_owned_roles_cannot_earn_wait(self):
        for result in [{'remaining_owned_group_members':[123]}, {'identity_mismatch':True},
                       {'identity_unavailable':True}, {'started':True,'returncode':None}]:
            with self.subTest(result=result):
                self.assertIsNotNone(self.check_reason('actual_original_mission_failed',cleanup={'bridge':result}))

    def test_nonzero_cleanup_exit_cancels_grace(self):
        self.assertIsNotNone(self.check_reason('actual_original_mission_failed',
            cleanup={'capture':{'started':True,'returncode':2,'remaining_owned_group_members':[]}}))


if __name__ == '__main__':
    unittest.main()
