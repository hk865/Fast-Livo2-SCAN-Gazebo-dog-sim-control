"""No ROS/Gazebo: real filesystem races and owned disposable cleanup processes."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import run_once_bounded as M


class MonitorTests(unittest.TestCase):
    def test_real_unlink_between_discovery_and_stat(self):
        with tempfile.TemporaryDirectory(prefix='r30_unlink_') as temp:
            root = Path(temp)
            victim = root / 'frame.tmp'
            victim.write_bytes(b'x'*1000)
            original = Path.lstat
            deleted = []
            def unlink_then_stat(path, *args, **kwargs):
                if path == victim and not deleted:
                    victim.unlink()
                    deleted.append(True)
                return original(path, *args, **kwargs)
            with patch.object(Path, 'lstat', unlink_then_stat):
                self.assertGreaterEqual(M.usage(root), 0)
            self.assertTrue(deleted)
            self.assertFalse(victim.exists())

    def test_concurrent_real_atomic_replacement_and_delete(self):
        with tempfile.TemporaryDirectory(prefix='r30_replace_') as temp:
            root = Path(temp)
            ready = threading.Event()
            ended = threading.Event()
            errors = []
            replacements = []
            def replace():
                try:
                    ready.set()
                    for i in range(500):
                        src = root / 'frame.tmp'
                        src.write_bytes(bytes([i%251])*32768)
                        os.replace(src, root / 'frame.jpg')
                        if i % 3 == 0:
                            (root / 'frame.jpg').unlink(missing_ok=True)
                        replacements.append(i)
                except Exception as exc:
                    errors.append(str(exc))
                finally:
                    ended.set()
            thread = threading.Thread(target=replace)
            thread.start()
            self.assertTrue(ready.wait(1))
            scans = 0
            while not ended.is_set() or scans < 500:
                self.assertGreaterEqual(M.usage(root), 0)
                scans += 1
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(len(replacements), 500)

    def test_other_budget_error_is_not_hidden(self):
        with tempfile.TemporaryDirectory(prefix='r30_budget_') as temp:
            with patch.object(M, 'existing_usage', side_effect=PermissionError('synthetic denied')):
                with self.assertRaises(PermissionError):
                    M.usage(Path(temp))

    def test_entrypoint_error_seals_outcome(self):
        with tempfile.TemporaryDirectory(prefix='r30_entrypoint_') as temp:
            root = Path(temp)
            with patch.object(M, 'S', root), patch.object(M, 'main', side_effect=RuntimeError('finite prelaunch failure')):
                with self.assertRaises(RuntimeError):
                    M.entrypoint()
            self.assertEqual(json.loads((root/'ENTRYPOINT_ERROR.json').read_text())['phase'],'entrypoint')
            self.assertEqual(json.loads((root/'BASELINE_CAPTURE_RUN_OUTCOME.json').read_text())['entrypoint_error']['type'],'RuntimeError')

    def launch_disposable_original_cleanup(self, root):
        source = '''import json, signal, sys, time
from pathlib import Path
root=Path(sys.argv[1])
def stop(sig,frame):
    (root/'ORIGINAL_CLEANUP.json').write_text(json.dumps({'signal':sig,'original_cleanup_ran':True}))
    sys.exit(0)
signal.signal(signal.SIGINT,stop)
(root/'READY').write_text('ready')
while True: time.sleep(.02)
'''
        proc = subprocess.Popen([sys.executable, '-B', '-c', source, str(root)], start_new_session=True)
        deadline = time.monotonic()+3
        while not (root/'READY').exists():
            if time.monotonic()>deadline:
                proc.kill()
                proc.wait()
                self.fail('Disposable runner did not become ready')
            time.sleep(.01)
        self.addCleanup(lambda: proc.poll() is None and (proc.kill(), proc.wait()))
        return proc

    def test_monitor_exception_seals_result_and_original_cleanup(self):
        for error in [FileNotFoundError('synthetic unexpected missing root'), PermissionError('synthetic non-bypassed permission')]:
            with self.subTest(type=type(error).__name__), tempfile.TemporaryDirectory(prefix='r30_exception_') as temp:
                root = Path(temp)
                proc = self.launch_disposable_original_cleanup(root)
                def fail():
                    raise error
                outcome = M.supervise(proc, M.identity(proc.pid), fail, root, lambda: root, 2)
                self.assertEqual(proc.returncode, 0)
                self.assertTrue(json.loads((root/'ORIGINAL_CLEANUP.json').read_text())['original_cleanup_ran'])
                self.assertEqual(outcome['monitor_error']['type'], type(error).__name__)
                self.assertIsNone(outcome['cleanup_error'])
                self.assertEqual(json.loads((root/'BASELINE_CAPTURE_RUN_OUTCOME.json').read_text())['monitor_error']['type'],type(error).__name__)
                self.assertEqual(json.loads((root/'STOP_INTENT.json').read_text())['runner_pid'],proc.pid)
                self.assertEqual(outcome['remaining_saved_groups'], [])

    def test_normal_stop_seals_result_without_monitor_error(self):
        with tempfile.TemporaryDirectory(prefix='r30_stop_') as temp:
            root = Path(temp)
            proc = self.launch_disposable_original_cleanup(root)
            outcome = M.supervise(proc, M.identity(proc.pid), lambda: {'reason':'finite_test_stop'}, root, lambda:root, 2)
            self.assertEqual(outcome['returncode'], 0)
            self.assertIsNone(outcome['monitor_error'])
            self.assertTrue((root/'ORIGINAL_CLEANUP.json').exists())

    def test_start_receipt_failure_enters_owned_cleanup_and_seals_outcome(self):
        with tempfile.TemporaryDirectory(prefix='r30_startreceipt_') as temp:
            root = Path(temp)
            proc = self.launch_disposable_original_cleanup(root)
            original_write = M.write
            def fail_start(path, value):
                if Path(path).name == 'RUNNER_START.json':
                    raise OSError('finite start receipt write failure')
                return original_write(path,value)
            with patch.object(M,'write',fail_start):
                outcome = M.supervise(proc,None,lambda:M.write(root/'RUNNER_START.json',{}),root,lambda:root,2)
            self.assertEqual(proc.returncode,0)
            self.assertTrue((root/'ORIGINAL_CLEANUP.json').exists())
            self.assertEqual(outcome['monitor_error']['type'],'OSError')
            self.assertTrue((root/'BASELINE_CAPTURE_RUN_OUTCOME.json').exists())

    def test_initial_identity_failure_is_sealed_without_permission_bypass(self):
        with tempfile.TemporaryDirectory(prefix='r30_startidentity_') as temp:
            root=Path(temp)
            proc=subprocess.Popen([sys.executable,'-B','-c','import time;time.sleep(.10)'],start_new_session=True)
            with patch.object(M,'identity',side_effect=PermissionError('finite initial proc stat failure')):
                outcome=M.supervise(proc,None,lambda:None,root,lambda:root,2)
            self.assertEqual(proc.returncode,0)
            self.assertEqual(outcome['monitor_error']['type'],'PermissionError')
            self.assertEqual(outcome['cleanup_error']['type'],'RuntimeError')
            self.assertFalse((root/'STOP_INTENT.json').exists())
            self.assertTrue((root/'BASELINE_CAPTURE_RUN_OUTCOME.json').exists())

    def test_identity_mismatch_refuses_signal_and_seals_error(self):
        with tempfile.TemporaryDirectory(prefix='r30_identity_') as temp:
            root = Path(temp)
            proc = self.launch_disposable_original_cleanup(root)
            # The disposable exits itself shortly after writing a ready marker;
            # a mismatch must not signal it, but supervise still waits for exit.
            original_poll=proc.poll
            timer=threading.Timer(.1,lambda:os.kill(proc.pid,signal.SIGINT))
            timer.start()
            outcome = M.supervise(proc, 'wrong_start_ticks', lambda:{'reason':'finite_test_stop'}, root, lambda:root, 2)
            timer.join(1)
            self.assertEqual(outcome['cleanup_error']['type'], 'RuntimeError')
            self.assertFalse((root/'STOP_INTENT.json').exists())
            self.assertTrue((root/'ORIGINAL_CLEANUP.json').exists())
            self.assertEqual(proc.returncode,0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
