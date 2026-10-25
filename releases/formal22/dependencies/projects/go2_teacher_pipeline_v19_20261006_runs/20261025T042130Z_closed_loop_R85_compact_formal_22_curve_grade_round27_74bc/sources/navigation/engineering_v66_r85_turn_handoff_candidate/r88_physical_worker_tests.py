"""Real static leaf and bounded IPC tests; no ROS node or simulation startup."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import r85_geometry_admission as geometry
from r85_physical_path_gate import R85PhysicalPathGate
import r88_physical_worker as service

FIXTURE = Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/'
    '20261018T113004Z_closed_loop_R85_R87_diagnostic_round02_9522/'
    'path_admission_candidates/000124_trajectory_126.json')


def wait_result(worker, timeout=8.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = worker.poll()
        if result is not None:
            return result
        time.sleep(.002)
    raise AssertionError('worker result timeout')


def canonical_proof(proof):
    return {k: v for k, v in proof.items() if k != 'elapsed_wall_ms'}


class PhysicalWorkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = json.loads(FIXTURE.read_text())
        cls.payload = cls.document['decision']['proof']['r85_physical']['physical_payload']
        cls.proof = geometry.certify_physical_curve(cls.payload)

    def worker(self):
        worker = service.Worker()
        self.addCleanup(worker.close)
        return worker

    def fake_worker(self, script):
        real_popen = subprocess.Popen
        with patch.object(service.subprocess, 'Popen', side_effect=lambda *a, **kw:
                          real_popen([sys.executable, '-u', '-c', script], **kw)):
            return self.worker()

    def test_real_geometry_single_flight_and_original_input_frozen(self):
        worker = self.worker()
        payload = copy.deepcopy(self.payload)
        self.assertTrue(worker.submit(1, payload))
        self.assertTrue(worker.busy)
        with self.assertRaisesRegex(RuntimeError, 'single_flight'):
            worker.submit(2, payload)
        payload['pos_pts'][0][0] += 100
        token, proof, metrics = wait_result(worker)
        self.assertEqual(token, 1)
        self.assertEqual(canonical_proof(proof), canonical_proof(self.proof))
        self.assertFalse(worker.busy)
        self.assertEqual(metrics['worker_pid'], worker.pid)
        self.assertNotEqual(worker.pid, os.getpid())
        self.assertGreater(metrics['worker_tid'], 0)
        self.assertTrue(worker.ready)
        self.assertEqual(worker.startup_receipt['process_group'], os.getpgrp())
        self.assertEqual(worker.startup_receipt['thread_caps'], service.THREAD_CAPS)
        self.assertEqual(worker.startup_receipt['modules']['worker'], service._module_receipt(service.__file__))
        self.assertEqual(worker.startup_receipt['modules']['geometry'], service._module_receipt(geometry.__file__))
        for key in ('compute_wall_ns', 'compute_thread_cpu_ns', 'compute_process_cpu_ns'):
            self.assertGreater(metrics[key], 0)
        self.assertLessEqual(metrics['submitted_monotonic_ns'], metrics['compute_started_monotonic_ns'])
        self.assertLessEqual(metrics['compute_finished_monotonic_ns'], metrics['received_monotonic_ns'])
        worker.submit('second', self.payload)
        self.assertEqual(wait_result(worker)[0], 'second')

    def test_real_positive_geometry(self):
        document = json.loads(FIXTURE.with_name('000128_trajectory_130.json').read_text())
        payload = document['decision']['proof']['r85_physical']['physical_payload']
        baseline = geometry.certify_physical_curve(payload)
        self.assertTrue(baseline['allowed'])
        worker = self.worker()
        worker.submit(1, payload)
        result = wait_result(worker)
        self.assertEqual(canonical_proof(result[1]), canonical_proof(baseline))

    def test_readiness_nonblocking_without_job(self):
        worker = self.worker()
        deadline = time.monotonic() + 5
        while not worker.ready and time.monotonic() < deadline:
            self.assertIsNone(worker.poll())
            time.sleep(.002)
        self.assertTrue(worker.ready)
        self.assertFalse(worker.busy)

    def test_input_bounds_do_not_create_job(self):
        worker = self.worker()
        for token in (None, True, -1, 2**63, '', 'x'*513):
            with self.assertRaises(ValueError):
                worker.submit(token, self.payload)
        for change in ({'pos_pts': [[0, 0, 0]]*513}, {'knots': []},
                       {'pos_pts': [[float('nan'), 0, 0]]*4, 'knots': [0]*4+[1]*4},
                       {'extra': 'x'*service.MAX_REQUEST_BYTES}):
            with self.assertRaises(ValueError):
                worker.submit(1, dict(self.payload, **change))
        self.assertFalse(worker.busy)

    def test_poll_does_not_wait_for_compute(self):
        worker = self.fake_worker('import sys,time; sys.stdin.buffer.readline(); time.sleep(10)')
        worker.submit(1, self.payload)
        times = []
        for _ in range(20):
            start = time.monotonic()
            self.assertIsNone(worker.poll())
            times.append(time.monotonic()-start)
        self.assertLess(max(times), .1)
        worker._submitted_ns -= int(9e9)
        token, proof, metrics = worker.poll()
        self.assertEqual(token, 1)
        self.assertIsNone(proof)
        self.assertIn('timeout', metrics['error'])
        self.assertIsNone(worker.poll())

    def test_worker_death_is_error_without_fallback(self):
        worker = self.worker()
        worker.submit(1, self.payload)
        worker._process.kill()
        token, proof, metrics = wait_result(worker)
        self.assertEqual(token, 1)
        self.assertIsNone(proof)
        self.assertIn('physical_worker_', metrics['error'])
        with self.assertRaises(RuntimeError):
            worker.submit(2, self.payload)

    def test_malformed_response_and_wrong_token_fail_closed(self):
        for response in (b'not-json\n', b'{"token":2,"proof":{},"metrics":{}}\n',
                         b'{"token":true,"proof":{},"metrics":{}}\n',
                         b'{"token":1,"proof":{},"metrics":{}}\n{}\n',
                         b'{"token":1,"proof":null,"metrics":{}}\n'):
            with self.subTest(response=response):
                worker = self.fake_worker('import sys,time; sys.stdin.buffer.readline(); '
                    'sys.stdout.buffer.write('+repr(response)+'); sys.stdout.buffer.flush(); time.sleep(1)')
                worker.submit(1, self.payload)
                token, proof, metrics = wait_result(worker)
                self.assertEqual(token, 1)
                self.assertIsNone(proof)
                self.assertTrue(metrics['error'])
                worker.close()

    def test_oversized_response_fail_closed(self):
        worker = self.fake_worker('import sys,time; sys.stdin.buffer.readline(); '
            'sys.stdout.buffer.write(b"x"*'+str(service.MAX_RESULT_BYTES+1)+'); '
            'sys.stdout.buffer.flush(); time.sleep(1)')
        worker.submit(1, self.payload)
        self.assertIn('byte_bound', wait_result(worker)[2]['error'])

    def test_child_request_and_result_errors_are_bounded(self):
        worker = self.worker()
        # Exercise the independent child's validation, not parent preflight.
        worker._pending = 1
        worker._submitted_ns = time.monotonic_ns()
        worker._outgoing = b'{"token":1,"payload":{}}\n'
        token, proof, metrics = wait_result(worker)
        self.assertEqual(token, 1)
        self.assertIsNone(proof)
        self.assertIn('compute', metrics['error'])
        self.assertLess(len(metrics['error']), 1100)

    def test_close_only_child_bounded_and_idempotent(self):
        worker = self.fake_worker('import time; time.sleep(10)')
        start = time.monotonic()
        result = worker.close()
        self.assertLess(time.monotonic()-start, 1.0)
        self.assertTrue(result['exited'])
        self.assertTrue(worker.close()['exited'])

    def test_gate_prepared_input_binding_and_original_pose_stamp(self):
        document = self.document
        old = document['decision']['proof']['r85_physical']
        context = copy.deepcopy(old['source_binding'])
        # The production source leaf is located in this checkout, as it is in
        # a real deployed run; actual geometry output binds that exact file.
        context['static_sources']['physical_code'] = self.proof['sources']['leaf']
        node = SimpleNamespace(request_id=self.payload['request_id'], pose_stamp=self.payload['source_stamp_ns'],
            automatic_reference=SimpleNamespace(reference=dict(sent_reference={
                k: v for k, v in document['metadata']['reference_content'].items() if k != 'global_curve'})),
            trajectory_association=SimpleNamespace(accepted=document['payload'],
                reference_stamp=document['metadata']['reference_stamp']))
        gate = R85PhysicalPathGate(FIXTURE.parents[1])
        gate.context = lambda n, b: copy.deepcopy(context)
        prepared = gate.prepare(node, document['payload'], document['metadata'], {})
        node.pose_stamp += 1_000_000
        result = gate.complete(node, document['payload'], document['metadata'], {}, prepared, self.proof)
        self.assertNotEqual(result.get('reason'), 'r85_physical_binding_rejected')
        self.assertEqual(result['source_pose_stamp_ns'], self.payload['source_stamp_ns'])
        self.assertEqual(result['physical_payload'], self.payload)
        context['planner_epoch'] += 1
        self.assertEqual(gate.complete(node, document['payload'], document['metadata'], {}, prepared,
                         self.proof)['reason'], 'r85_physical_binding_rejected')
        context['planner_epoch'] -= 1
        node.trajectory_association.accepted = None
        self.assertEqual(gate.complete(node, document['payload'], document['metadata'], {}, prepared,
                         self.proof)['reason'], 'r85_physical_binding_rejected')
        node.trajectory_association.accepted = document['payload']
        corrupt = copy.deepcopy(prepared)
        corrupt['physical_payload']['pos_pts'][0][0] += 1
        self.assertEqual(gate.complete(node, document['payload'], document['metadata'], {}, corrupt,
                         self.proof)['reason'], 'r85_physical_binding_rejected')
        proof = copy.deepcopy(self.proof)
        proof['payload_sha256'] = 'corrupt'
        self.assertEqual(gate.complete(node, document['payload'], document['metadata'], {}, prepared,
                         proof)['reason'], 'r85_physical_binding_rejected')


if __name__ == '__main__':
    unittest.main(verbosity=2)
