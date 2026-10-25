"""Bounded eager IPC submission tests; no ROS node or Gazebo startup."""
import copy
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import r88_physical_worker as service
from r88_physical_worker_tests import FIXTURE, canonical_proof, wait_result


class EagerSendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.payload = json.loads(FIXTURE.read_text())['decision']['proof']['r85_physical']['physical_payload']

    def fake(self, eager=True):
        worker = service.Worker.__new__(service.Worker)
        worker.eager_submit = eager
        worker._process = SimpleNamespace(pid=12345,
            stdin=SimpleNamespace(fileno=lambda: 42), stdout=SimpleNamespace(fileno=lambda: 43),
            poll=Mock(return_value=None), terminate=Mock())
        worker._pending = None
        worker._outgoing = b''
        worker._incoming = bytearray()
        worker._submitted_ns = None
        worker._submit_transport_error = None
        worker._closed = worker._failure_reported = False
        worker.failure = None
        worker.ready = False
        worker.startup_receipt = None
        return worker

    def test_boolean_option_is_checked_before_child_creation(self):
        with patch.object(service.subprocess, 'Popen') as popen:
            for option in (None, 0, 1, '1', [], {}):
                with self.subTest(option=option), self.assertRaisesRegex(ValueError, 'boolean_required'):
                    service.Worker(eager_submit=option)
            popen.assert_not_called()

    def test_disabled_submit_has_no_write_or_read_and_same_timeout_origin(self):
        worker = self.fake(False)
        with patch.object(service.os, 'write') as write, patch.object(service.os, 'read') as read, \
                patch.object(service.time, 'monotonic_ns', return_value=1000):
            self.assertTrue(worker.submit('token', self.payload))
        write.assert_not_called()
        read.assert_not_called()
        self.assertEqual(worker._submitted_ns, 1000)
        self.assertEqual(worker._pending, 'token')
        self.assertEqual(worker._outgoing, service._encoded(dict(token='token', payload=self.payload), service.MAX_REQUEST_BYTES))

    def test_eager_submit_attempts_one_bounded_write_no_read_and_preserves_partial(self):
        worker = self.fake()
        payload = dict(self.payload, padding='x'*(service.IO_CHUNK_BYTES+200))
        raw = service._encoded(dict(token=7, payload=payload), service.MAX_REQUEST_BYTES)
        with patch.object(service.os, 'write', return_value=17) as write, \
                patch.object(service.os, 'read') as read, \
                patch.object(service.time, 'monotonic_ns', return_value=2000) as clock:
            self.assertTrue(worker.submit(7, payload))
        write.assert_called_once_with(42, raw[:service.IO_CHUNK_BYTES])
        read.assert_not_called()
        clock.assert_called_once_with()
        self.assertEqual(worker._submitted_ns, 2000)
        self.assertEqual(worker._outgoing, raw[17:])
        self.assertEqual(worker._pending, 7)

    def test_blocking_io_keeps_entire_message_for_poll_without_retry(self):
        worker = self.fake()
        with patch.object(service.os, 'write', side_effect=BlockingIOError()) as write:
            worker.submit(2, self.payload)
        self.assertEqual(write.call_count, 1)
        self.assertIsNone(worker._submit_transport_error)
        self.assertEqual(worker._outgoing, service._encoded(dict(token=2, payload=self.payload), service.MAX_REQUEST_BYTES))
        self.assertTrue(worker.busy)

    def test_zero_write_is_one_attempt_and_keeps_remainder(self):
        worker = self.fake()
        with patch.object(service.os, 'write', return_value=0) as write:
            worker.submit(3, self.payload)
        self.assertEqual(write.call_count, 1)
        self.assertTrue(worker._outgoing)
        self.assertIsNone(worker._submit_transport_error)

    def test_partial_send_is_finished_once_per_original_poll_without_duplicates(self):
        worker = self.fake()
        raw = service._encoded(dict(token=4, payload=self.payload), service.MAX_REQUEST_BYTES)
        written = []
        def write(fd, data):
            self.assertEqual(fd, 42)
            amount = 11 if not written else len(data)
            written.append(data[:amount])
            return amount
        with patch.object(service.os, 'write', side_effect=write) as sender, \
                patch.object(service.os, 'read', side_effect=BlockingIOError()):
            worker.submit(4, self.payload)
            origin = worker._submitted_ns
            self.assertIsNone(worker.poll())
            self.assertIsNone(worker.poll())
        self.assertEqual(sender.call_count, 2)
        self.assertEqual(b''.join(written), raw)
        self.assertEqual(worker._outgoing, b'')
        self.assertEqual(worker._submitted_ns, origin)
        self.assertTrue(worker.busy)

    def test_broken_pipe_reports_original_token_once_via_poll_without_rewrite(self):
        worker = self.fake()
        with patch.object(service.os, 'write', side_effect=BrokenPipeError(32, 'closed')) as write, \
                patch.object(service.os, 'read') as read:
            self.assertTrue(worker.submit(5, self.payload))
            self.assertTrue(worker.busy)
            self.assertIsNone(worker.failure)
            self.assertFalse(worker._failure_reported)
            worker._incoming.extend(b'prior bytes')
            token, proof, metrics = worker.poll()
            self.assertIsNone(worker.poll())
        self.assertEqual(write.call_count, 1)
        read.assert_not_called()
        self.assertEqual(token, 5)
        self.assertIsNone(proof)
        self.assertIn('physical_worker_transport:BrokenPipeError:', metrics['error'])
        self.assertFalse(worker.busy)
        self.assertEqual(worker._outgoing, b'')
        self.assertEqual(worker._incoming, b'')
        self.assertIsNone(worker._submit_transport_error)
        worker._process.terminate.assert_called_once_with()
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            worker.submit(6, self.payload)

    def test_eager_and_original_poll_transport_errors_have_same_result(self):
        for error_type in (BrokenPipeError, OSError, ValueError, TypeError, OverflowError):
            outcomes = []
            for eager in (False, True):
                worker = self.fake(eager)
                with patch.object(service.os, 'write', side_effect=error_type('x'*3000)), \
                        patch.object(service.os, 'read') as read:
                    worker.submit('failure-token', self.payload)
                    result = worker.poll()
                read.assert_not_called()
                outcomes.append((result[0], result[1], result[2]['error'], worker.busy, worker._outgoing))
                self.assertLessEqual(len(result[2]['error']), 1024)
            self.assertEqual(outcomes[0], outcomes[1], error_type)

    def test_deferred_error_reaches_real_coordinator_global_fail_closed_after_owner_loss(self):
        import r88_async_admission as coordinator
        from r88_async_admission_tests import MinNode, message
        for lose_owner in (False, True):
            with self.subTest(lose_owner=lose_owner):
                worker, node = self.fake(), MinNode()
                admission = coordinator.Admission(node, worker)
                with patch.object(service.os, 'write', side_effect=BrokenPipeError('closed')) as write, \
                        patch.object(service.os, 'read') as read:
                    admission.start(message(), {})
                    self.assertIsNotNone(admission.current)
                    self.assertEqual(node.state, 'running')
                    admission.receive('metadata', SimpleNamespace(data='queued'),
                        lambda msg: node.order.append('must-not-consume'))
                    if lose_owner:
                        node.request_id = 'new-owner'
                    admission.poll()
                    admission.poll()
                write.assert_called_once()
                read.assert_not_called()
                self.assertEqual(node.state, 'failed')
                self.assertEqual(node.command, [0., 0., 0.])
                self.assertNotIn('must-not-consume', node.order)
                self.assertEqual(admission.fault, 'r88_physical_worker_failed')
                self.assertEqual(admission.counts['faults'], 1)
                self.assertEqual(len(node.resumes), 0 if lose_owner else 1)
                if node.resumes:
                    self.assertIn('BrokenPipeError', node.resumes[0]['physical_result']['error'])
                self.assertFalse(worker.busy)

    def test_invalid_request_or_busy_never_sends(self):
        worker = self.fake()
        with patch.object(service.os, 'write') as write:
            for token, payload in ((True, self.payload),
                    (1, dict(self.payload, padding='x'*service.MAX_REQUEST_BYTES)),
                    (1, dict(self.payload, knots=[]))):
                with self.assertRaises(ValueError):
                    worker.submit(token, payload)
            write.assert_not_called()
        self.assertFalse(worker.busy)
        worker._pending = 12
        with patch.object(service.os, 'write') as write, self.assertRaisesRegex(RuntimeError, 'single_flight'):
            worker.submit(13, self.payload)
        write.assert_not_called()

    def test_eager_result_does_not_bypass_original_job_timeout(self):
        worker = self.fake()
        with patch.object(service.os, 'write', side_effect=BlockingIOError()), \
                patch.object(service.os, 'read', side_effect=BlockingIOError()):
            worker.submit(6, self.payload)
            worker._submitted_ns -= int(9e9)
            token, proof, metrics = worker.poll()
        self.assertEqual(token, 6)
        self.assertIsNone(proof)
        self.assertEqual(metrics['error'], 'physical_worker_job_timeout')

    def test_wrong_startup_mode_fails_original_readiness_gate(self):
        worker = self.fake()
        raw = json.dumps(dict(event='ready', receipt=dict(worker_pid=12345,
            eager_submit=False, thread_caps=service.THREAD_CAPS))).encode()+b'\n'
        with patch.object(service.os, 'write', side_effect=lambda fd, data:len(data)), \
                patch.object(service.os, 'read', return_value=raw):
            worker.submit(8, self.payload)
            token, proof, metrics = worker.poll()
        self.assertEqual(token, 8)
        self.assertIsNone(proof)
        self.assertEqual(metrics['error'], 'physical_worker_readiness_identity')

    def test_real_worker_off_on_preserve_positive_and_negative_proofs_and_receipts(self):
        fixtures = [FIXTURE, FIXTURE.with_name('000128_trajectory_130.json')]
        proofs = {}
        for eager in (False, True):
            worker = service.Worker(eager_submit=eager)
            self.addCleanup(worker.close)
            for index, fixture in enumerate(fixtures):
                payload = json.loads(fixture.read_text())['decision']['proof']['r85_physical']['physical_payload']
                worker.submit(index+1, copy.deepcopy(payload))
                token, proof, metrics = wait_result(worker)
                self.assertEqual(token, index+1)
                self.assertEqual(worker.startup_receipt['eager_submit'], eager)
                self.assertEqual(metrics['startup_receipt']['eager_submit'], eager)
                self.assertLessEqual(metrics['submitted_monotonic_ns'], metrics['compute_started_monotonic_ns'])
                if not eager:
                    proofs[index] = canonical_proof(proof)
                else:
                    self.assertEqual(canonical_proof(proof), proofs[index])
            result = worker.close()
            self.assertEqual(result['returncode'], 0)
            self.assertFalse(result['terminated'])
            self.assertFalse(result['killed'])

    def test_real_child_closed_input_defers_error_and_close_clears_it(self):
        worker = service.Worker(eager_submit=True)
        self.addCleanup(worker.close)
        worker._process.stdin.close()
        self.assertTrue(worker.submit(9, self.payload))
        self.assertTrue(worker.busy)
        token, proof, metrics = worker.poll()
        self.assertEqual(token, 9)
        self.assertIsNone(proof)
        self.assertIn('physical_worker_transport:ValueError:', metrics['error'])
        self.assertTrue(worker.close()['exited'])
        self.assertIsNone(worker.poll())
        self.assertIsNone(worker._submit_transport_error)


def benchmark():
    """Finite interleaved control-period benchmark, not a controller/Gazebo run."""
    from r88_physical_worker_tests import geometry
    fixtures = [FIXTURE, FIXTURE.with_name('000128_trajectory_130.json')]
    payloads = [json.loads(path.read_text())['decision']['proof']['r85_physical']['physical_payload']
                for path in fixtures]
    expected = [canonical_proof(geometry.certify_physical_curve(payload)) for payload in payloads]
    workers = {mode: service.Worker(eager_submit=mode) for mode in (False, True)}
    results, exits = [], {}
    period_ns = 50_000_000
    try:
        # Both modes complete the same finite warmup before timed comparisons.
        for mode, worker in workers.items():
            for warm in range(3):
                worker.submit('warm-'+str(warm), payloads[warm % 2])
                result = wait_result(worker)
                if canonical_proof(result[1]) != expected[warm % 2]:
                    raise AssertionError('warmup proof mismatch')
        for pair in range(12):
            fixture_index = pair % 2
            for mode in ((False, True) if pair % 2 == 0 else (True, False)):
                worker = workers[mode]
                token = pair*2+int(mode)+1
                start_ns = time.monotonic_ns()
                owner_cpu = time.thread_time_ns()
                steps = []
                completed = None
                submit_wall_ns = submit_cpu_ns = None
                for index in range(11):
                    scheduled_ns = start_ns+index*period_ns
                    remaining_ns = scheduled_ns-time.monotonic_ns()
                    if remaining_ns > 0:
                        time.sleep(remaining_ns/1e9)
                    step_start, cpu_start = time.monotonic_ns(), time.thread_time_ns()
                    if index == 0:
                        begin_wall, begin_cpu = time.monotonic_ns(), time.thread_time_ns()
                        worker.submit(token, payloads[fixture_index])
                        submit_wall_ns = time.monotonic_ns()-begin_wall
                        submit_cpu_ns = time.thread_time_ns()-begin_cpu
                    elif completed is None:
                        completed = worker.poll()
                    step_end, cpu_end = time.monotonic_ns(), time.thread_time_ns()
                    steps.append(dict(index=index, scheduled_ns=scheduled_ns,
                        start_ns=step_start, lateness_ns=max(0, step_start-scheduled_ns),
                        wall_ns=step_end-step_start, thread_cpu_ns=cpu_end-cpu_start,
                        result_available=completed is not None))
                if completed is None or completed[0] != token or completed[2].get('error'):
                    raise AssertionError('benchmark result missing or failed')
                if canonical_proof(completed[1]) != expected[fixture_index]:
                    raise AssertionError('benchmark proof changed')
                metrics = completed[2]
                results.append(dict(pair=pair, eager_submit=mode, fixture_index=fixture_index,
                    submit_wall_ns=submit_wall_ns, submit_thread_cpu_ns=submit_cpu_ns,
                    owner_thread_cpu_ns=time.thread_time_ns()-owner_cpu,
                    submitted_to_compute_start_ns=metrics['compute_started_monotonic_ns']-metrics['submitted_monotonic_ns'],
                    compute_wall_ns=metrics['compute_wall_ns'],
                    compute_thread_cpu_ns=metrics['compute_thread_cpu_ns'],
                    submitted_to_result_read_ns=metrics['received_monotonic_ns']-metrics['submitted_monotonic_ns'],
                    compute_finish_to_result_read_ns=metrics['received_monotonic_ns']-metrics['compute_finished_monotonic_ns'],
                    max_step_wall_ns=max(x['wall_ns'] for x in steps),
                    max_step_lateness_ns=max(x['lateness_ns'] for x in steps),
                    lateness_ge_5ms=sum(x['lateness_ns'] >= 5_000_000 for x in steps),
                    steps=steps, metrics=metrics))
    finally:
        for mode, worker in workers.items():
            exits[str(mode)] = worker.close()
    def stats(values):
        ordered = sorted(values)
        return dict(min=min(values), median=statistics.median(values),
                    p95_nearest=ordered[min(len(ordered)-1, int(.95*len(ordered)))], max=max(values))
    keys = ('submit_wall_ns', 'submit_thread_cpu_ns', 'owner_thread_cpu_ns',
            'submitted_to_compute_start_ns', 'compute_wall_ns', 'compute_thread_cpu_ns',
            'submitted_to_result_read_ns', 'compute_finish_to_result_read_ns',
            'max_step_wall_ns', 'max_step_lateness_ns')
    summaries = {str(mode): {key: stats([row[key] for row in results if row['eager_submit']==mode])
        for key in keys} for mode in (False, True)}
    pairs = []
    for pair in range(12):
        off = next(x for x in results if x['pair']==pair and not x['eager_submit'])
        on = next(x for x in results if x['pair']==pair and x['eager_submit'])
        pairs.append(dict(pair=pair,
            compute_start_saved_ns=off['submitted_to_compute_start_ns']-on['submitted_to_compute_start_ns'],
            submit_wall_added_ns=on['submit_wall_ns']-off['submit_wall_ns'],
            submit_thread_cpu_added_ns=on['submit_thread_cpu_ns']-off['submit_thread_cpu_ns']))
    return dict(schema='r89_eager_nonblocking_submit_benchmark/v1',
        scope='Two real static leaf workers; interleaved OFF/ON 12 pairs, alternating order, 50ms fixed control-period loop. No ROS node, admission transaction or Gazebo.',
        limitations='A finite observation, not a latency bound or full-loop success claim. Submit occurs just after poll phase, so OFF waits one full nominal period. Real callback phases differ. Added syscall can have nonzero cost; no strict zero-overhead claim.',
        loop_period_ns=period_ns, steps=11, pairs=12,
        runtime=dict(pid=os.getpid(), affinity=sorted(os.sched_getaffinity(0)), load_average=os.getloadavg(), python=sys.version),
        sources={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
                 (Path(service.__file__), Path(__file__), Path(geometry.__file__), *fixtures)},
        summaries_ns=summaries, paired_deltas_ns=pairs, trials=results, exits=exits,
        all_proofs_match=True, all_workers_exited_naturally=all(
            x['returncode']==0 and x['exited'] and not x['terminated'] and not x['killed'] for x in exits.values()))


if __name__ == '__main__':
    if sys.argv[1:] == ['--benchmark']:
        print(json.dumps(benchmark(), ensure_ascii=False, indent=2))
    else:
        unittest.main(verbosity=2)
