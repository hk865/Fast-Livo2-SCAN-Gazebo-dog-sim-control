"""Finite, no-node R87 observer validation and full real-CDR mailbox benchmark.

No rclpy.init(), ROS graph, simulator, external data write, or payload retention
by the observer. Run under /opt/ros/jazzy/setup.bash. Benchmark results describe
this host/run only; they are not latency bounds or navigation acceptance.
"""
import argparse
from array import array
import builtins
from collections import deque
import gc
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

import r87_diagnostic_probe as diag
import r86_prepared_feedback as current
from point_provenance import message_sha, verify_corrected
from r86_prepared_feedback_tests import packet, feed

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent
BASELINE = 'ebec30cae924e1fd4487709ffe9dd4396277cec6'


def baseline_module():
    name = '_r87_original_ebec30ca_mailbox'
    if name not in sys.modules:
        code = subprocess.check_output(
            ['git', '-C', str(SOURCE), 'show', BASELINE + ':candidate/r86_prepared_feedback.py'],
            text=True)
        module = types.ModuleType(name)
        module.__file__ = 'git:' + BASELINE + ':candidate/r86_prepared_feedback.py'
        sys.modules[name] = module
        exec(compile(code, module.__file__, 'exec'), module.__dict__)
    return sys.modules[name]


def packet_with_cloud_bytes(point_count=65536):
    """2 MiB padded XYZ cloud, within existing 65536-point/4-MiB part bounds."""
    data = packet()
    cloud = PointCloud2()
    cloud.header = data['body'].header
    cloud.height, cloud.width = 1, point_count
    cloud.fields = [PointField(name=name, offset=index * 4,
        datatype=PointField.FLOAT32, count=1) for index, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian, cloud.is_dense = False, True
    cloud.point_step, cloud.row_step = 32, point_count * 32
    # Fixture construction is outside timing; instrumentation never sees arrays.
    payload = np.zeros(cloud.row_step, dtype=np.uint8)
    xyz = np.ndarray((point_count, 3), dtype='<f4', buffer=payload, strides=(32, 4))
    xyz[:] = (1., .4, .1)
    cloud.data = array('B', payload.tobytes())
    data['cloud'] = cloud
    data['metadata']['corrected_cloud_message_sha256'] = message_sha(cloud)
    return data


def complete_cycle(box, data):
    """Intake freeze/join/decode/hash, take, control decode, commit and release."""
    feed(box, data)
    batch = box.take_batch()
    if len(batch) != 1:
        raise AssertionError('benchmark packet was not prepared')
    frame = batch[0]
    body = box.decode('body', frame.body)
    cloud = box.decode('cloud', frame.cloud)
    metadata = box.decode('metadata', frame.metadata)
    if metadata['source_ns'] != frame.source_ns or cloud.width != data['cloud'].width:
        raise AssertionError('benchmark decode changed packet identity')
    box.committed(frame, time.monotonic_ns())
    box.release_batch(batch)
    if box.bytes or box.inflight:
        raise AssertionError('benchmark retained an in-flight packet')
    return frame, body, cloud, metadata


class CoreProbeTests(unittest.TestCase):
    def test_default_off_and_explicit_on_import(self):
        source = Path(diag.__file__).read_text()
        for option, expected in ((None, False), ('0', False), ('1', True)):
            namespace = {'__name__': '_r87_option_test'}
            with patch.dict(os.environ, {}, clear=False):
                if option is None:
                    os.environ.pop(diag.KEY, None)
                else:
                    os.environ[diag.KEY] = option
                exec(compile(source, diag.__file__, 'exec'), namespace)
            self.assertEqual(namespace['enabled'](), expected)

    def test_bounded_tail_first_anomaly_and_exact_drop_accounting(self):
        recorder = diag.Recorder(capacity=16)
        for index in range(24):
            recorder.mark('before', index)
        recorder.anomaly('first', 23)
        frozen = recorder.frozen
        for index in range(32):
            recorder.mark('after', 100 + index)
        recorder.anomaly('second', 131)
        self.assertIs(recorder.frozen, frozen)
        self.assertEqual(recorder.trigger['reason'], 'first')
        self.assertEqual(len(recorder.rows), 16)
        self.assertEqual(len(recorder.frozen), 16)
        with recorder.guard:
            recorder.mark('contended')
        snapshot = recorder.snapshot()
        self.assertEqual(snapshot['attempted'], 59)
        self.assertEqual(snapshot['recorded'], 58)
        self.assertEqual(snapshot['evicted'], 42)
        self.assertEqual(snapshot['dropped_lock_contention'], 1)
        self.assertEqual(snapshot['probe_errors'], 0)
        self.assertEqual(snapshot['rows'][-1]['stage'], 'anomaly')
        self.assertTrue(any(row['stage'] == 'before' for row in snapshot['anomaly_window']))

    def test_payloads_not_retained_and_scalar_fields_bounded(self):
        recorder = diag.Recorder(capacity=4)
        data = packet_with_cloud_bytes(16)
        row = recorder.mark('s' * 200, 12, cloud=data['cloud'], data=data['cloud'].data,
            metadata=data['metadata'], sequence=[1, 2], text='x' * 500, infinite=float('inf'))
        self.assertTrue(all(key not in row for key in ('cloud', 'data', 'metadata', 'sequence')))
        self.assertEqual(len(row['stage']), 120)
        self.assertEqual(len(row['text']), 240)
        self.assertIsNone(row['infinite'])
        json.dumps(recorder.snapshot(), allow_nan=False)

    def test_first_anomaly_survives_contention_and_freezes_on_next_writer_or_exit(self):
        for freeze_at_exit in (False, True):
            recorder = diag.Recorder(capacity=16)
            recorder.mark('before', 1)
            with recorder.guard:
                recorder.anomaly('first contended fault', 2)
                recorder.anomaly('later contended fault', 3)
            self.assertIsNone(recorder.frozen)
            self.assertEqual(recorder.pending_anomaly['first']['source_ns'], 2)
            if not freeze_at_exit:
                recorder.mark('successful next writer', 4)
            snapshot = recorder.snapshot()
            self.assertEqual(snapshot['anomaly_trigger']['reason'], 'first contended fault')
            self.assertEqual(snapshot['anomaly_trigger']['source_ns'], 2)
            self.assertGreaterEqual(snapshot['anomaly_trigger']['frozen_monotonic_wall_ns'],
                                    snapshot['anomaly_trigger']['monotonic_wall_ns'])
            self.assertEqual(snapshot['anomaly_window'][0]['stage'], 'before')
            self.assertEqual(snapshot['dropped_lock_contention'], 2)

    def test_optional_exit_flush_failure_does_not_abort_cleanup(self):
        recorder = diag.Recorder(capacity=4)
        recorder.mark('tail')
        cleanup = []
        stderr = io.StringIO()
        with patch.object(diag, 'RECORDER', recorder),\
             patch.object(Path, 'write_text', side_effect=OSError('fixture disk failure')),\
             patch.object(sys, 'stderr', stderr):
            diag.flush(Path('/not-written-fixture'))
            cleanup.append('existing cleanup reached')
        self.assertEqual(cleanup, ['existing cleanup reached'])
        self.assertIn('R87 diagnostic timeline flush failed: OSError', stderr.getvalue())
        self.assertEqual(stderr.getvalue().count('\n'), 1)

    def test_observer_counter_failures_preserve_business_result_and_exception(self):
        recorder = diag.Recorder(capacity=32)
        seen = []
        @diag.observe('test.callback')
        def callback(value):
            seen.append(value)
            return value + 1
        with patch.object(diag, 'RECORDER', recorder),\
             patch.object(diag, 'sample', side_effect=OSError('injected sampler failure')):
            self.assertEqual(callback(4), 5)
            with self.assertRaisesRegex(RuntimeError, 'original business error'):
                with diag.span('test.business_error'):
                    raise RuntimeError('original business error')
            diag.anomaly('still observable failure')
        self.assertEqual(seen, [4])
        self.assertGreaterEqual(recorder.snapshot()['probe_errors'], 5)

    def test_no_hot_path_io_or_print(self):
        recorder = diag.Recorder(capacity=16)
        forbidden = AssertionError('observer hot-path I/O')
        with patch.object(diag, 'RECORDER', recorder),\
             patch.object(builtins, 'open', side_effect=forbidden),\
             patch.object(builtins, 'print', side_effect=forbidden),\
             patch.object(os, 'open', side_effect=forbidden),\
             patch.object(os, 'write', side_effect=forbidden),\
             patch.object(Path, 'write_text', side_effect=forbidden),\
             patch.object(json, 'dumps', side_effect=forbidden):
            with diag.span('no_io', 1):
                diag.mark('inside', 1, value=2)
            diag.anomaly('retained only', 1)
        self.assertEqual(recorder.snapshot()['probe_errors'], 0)

    def test_actual_native_tid_and_clocks_per_thread(self):
        recorder = diag.Recorder(capacity=16)
        barrier = threading.Barrier(3)
        tids = []
        def worker():
            tids.append(threading.get_native_id())
            barrier.wait()
            recorder.mark('thread')
        threads = [threading.Thread(target=worker) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
        self.assertEqual({row['native_tid'] for row in recorder.rows}, set(tids))
        for row in recorder.rows:
            self.assertGreaterEqual(row['wall_end_ns'], row['wall_begin_ns'])
            self.assertGreater(row['thread_cpu_ns'], 0)
            self.assertGreater(row['process_cpu_ns'], 0)

    def test_original_lock_released_when_business_raises(self):
        recorder = diag.Recorder(capacity=32)
        original = threading.Lock()
        with patch.object(diag, 'RECORDER', recorder):
            with self.assertRaisesRegex(ValueError, 'business'):
                with diag.lock(original, 'test.lock'):
                    self.assertTrue(original.locked())
                    raise ValueError('business')
        self.assertFalse(original.locked())
        self.assertEqual([row['stage'] for row in recorder.rows],
            ['test.lock.wait', 'test.lock.acquired', 'test.lock.release_before', 'test.lock.release_after'])

    def test_mailbox_original_disabled_enabled_wire_and_decision_parity(self):
        outcomes = []
        for module, recorder in ((baseline_module(), None), (current, None),
                                 (current, diag.Recorder(capacity=4096))):
            with patch.object(diag, 'RECORDER', recorder):
                box = module.ros_mailbox(verify_corrected)
                box.set_scope(b'fixed-scope')
                feed(box, packet(bad_hash=True))
                frame, body, cloud, metadata = complete_cycle(box, packet())
                feed(box, packet())  # Duplicate source still rejects without receipt refresh.
                self.assertEqual(box.take_batch(), ())
                outcomes.append((frame.body, frame.cloud, frame.metadata, frame.source_ns,
                    frame.token, frame.generation, frame.arrivals_ns, frame.scopes,
                    dict(box.counts), box.fault, box.bytes, body.pose.pose.position.x, cloud.width,
                    metadata['source_ns']))
        self.assertEqual(outcomes[0], outcomes[1])
        self.assertEqual(outcomes[0], outcomes[2])


def distribution(values):
    ordered = sorted(values)
    return dict(n=len(values), mean=statistics.mean(values), median=statistics.median(values),
        p95=ordered[int(.95 * (len(ordered) - 1))], p99=ordered[int(.99 * (len(ordered) - 1))],
        maximum=max(values), minimum=min(values))


def deep_size(value, seen=None):
    """Unique reachable Python allocation bytes, excludes interpreter/allocator RSS."""
    if seen is None:
        seen = set()
    if id(value) in seen:
        return 0
    seen.add(id(value))
    size = sys.getsizeof(value)
    if isinstance(value, dict):
        size += sum(deep_size(k, seen) + deep_size(v, seen) for k, v in value.items())
    elif isinstance(value, (list, tuple, deque)):
        size += sum(deep_size(item, seen) for item in value)
    return size


def memory_and_cpu_examples():
    recorder = diag.Recorder()
    with patch.object(diag, 'RECORDER', recorder):
        for index in range(diag.CAPACITY):
            recorder.mark('mailbox.batch_member', index, batch_id=index,
                prepared_ns=time.monotonic_ns(), scope_body='a' * 64,
                scope_cloud='b' * 64, scope_metadata='c' * 64)
        recorder.anomaly('synthetic_first_failure', diag.CAPACITY - 1)
        for index in range(diag.CAPACITY):
            recorder.mark('post_failure', index)
        with diag.span('example.wait', 1):
            time.sleep(.06)
        with diag.span('example.cpu', 2):
            deadline = time.thread_time_ns() + 60_000_000
            accumulator = 0
            while time.thread_time_ns() < deadline:
                accumulator += 1
    snapshot = recorder.snapshot()
    examples = {}
    for stage in ('example.wait', 'example.cpu'):
        start = next(row for row in snapshot['rows'] if row['stage'] == stage + '.enter')
        end = next(row for row in snapshot['rows'] if row['stage'] == stage + '.exit')
        examples[stage] = dict(native_tid=start['native_tid'],
            wall_ns=end['wall_begin_ns'] - start['wall_end_ns'],
            thread_cpu_ns=end['thread_cpu_ns'] - start['thread_cpu_ns'],
            process_cpu_ns=end['process_cpu_ns'] - start['process_cpu_ns'],
            begin_usage=start['thread_usage'], end_usage=end['thread_usage'])
    return dict(capacity=diag.CAPACITY, rows=len(snapshot['rows']),
        frozen_rows=len(snapshot['anomaly_window']), slow_spans=len(snapshot['slow_spans']),
        unique_retained_python_bytes=deep_size((recorder.rows, recorder.frozen, recorder.slow)),
        compact_snapshot_serialized_bytes=len(json.dumps(snapshot, separators=(',', ':')).encode()),
        payload_copies_by_instrumentation=0,
        size_caveat='Fixture-filled reachable Python allocations and compact JSON bytes; not RSS or a universal per-row size bound.',
        counters={key: snapshot[key] for key in ('recorded', 'attempted', 'evicted',
            'dropped_lock_contention', 'probe_errors')}, examples=examples)


def benchmark():
    baseline = baseline_module()
    report = dict(schema='r87_full_mailbox_probe_benchmark/v1', baseline_sha=BASELINE,
        observed_only_not_bound=True, ros_node_initialized=False, simulator_started=False,
        python=sys.version, host=platform.node(), affinity=sorted(os.sched_getaffinity(0)),
        load_average_start=os.getloadavg(), environment_switch=os.environ.get(diag.KEY, '0'),
        monotonic_started_ns=time.monotonic_ns(),
        git_head=subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip(),
        source_sha256={path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in
            (Path(diag.__file__), Path(current.__file__), HERE / 'point_provenance.py')},
        timer_scope='feed all 3 roles -> real CDR/JSON/decode/semantic hash -> take_batch -> body/cloud/metadata control decode -> committed -> release_batch; box/fixture setup and snapshot excluded',
        modes='original deployed mailbox; current observer OFF; current observer ON, serial calls in one thread',
        method='5 interleaved order-rotated blocks per mode, 12 warmup cycles per mode/payload, GC remains enabled; no ROS node or scheduler contention model',
        cases={})
    for label, data, iterations in (('tiny_24_byte_xyz', packet(), 100),
                                     ('representative_2_mib_padded_xyz', packet_with_cloud_bytes(), 30)):
        modes = {'baseline': (baseline, None), 'disabled': (current, None),
                 'enabled': (current, diag.Recorder())}
        raw = {name: dict(wall_ns=[], thread_cpu_ns=[], process_cpu_ns=[], block_wall_mean_ns=[])
            for name in modes}
        for module, recorder in modes.values():
            with patch.object(diag, 'RECORDER', recorder):
                for _ in range(12):
                    box = module.ros_mailbox(verify_corrected)
                    complete_cycle(box, data)
        for block in range(5):
            names = list(modes)
            names = names[block % 3:] + names[:block % 3]
            for name in names:
                module, recorder = modes[name]
                samples = raw[name]
                first = len(samples['wall_ns'])
                with patch.object(diag, 'RECORDER', recorder):
                    for _ in range(iterations):
                        box = module.ros_mailbox(verify_corrected)
                        process_before, cpu_before, wall_before = (
                            time.process_time_ns(), time.thread_time_ns(), time.monotonic_ns())
                        complete_cycle(box, data)
                        wall_after, cpu_after, process_after = (
                            time.monotonic_ns(), time.thread_time_ns(), time.process_time_ns())
                        samples['wall_ns'].append(wall_after - wall_before)
                        samples['thread_cpu_ns'].append(cpu_after - cpu_before)
                        samples['process_cpu_ns'].append(process_after - process_before)
                samples['block_wall_mean_ns'].append(statistics.mean(samples['wall_ns'][first:]))
        outputs = {}
        for name, samples in raw.items():
            outputs[name] = {key: distribution(value) for key, value in samples.items()}
            recorder = modes[name][1]
            if recorder:
                snapshot = recorder.snapshot()
                outputs[name]['probe'] = {key: snapshot[key] for key in ('recorded', 'attempted',
                    'evicted', 'dropped_lock_contention', 'probe_errors')}
                outputs[name]['probe']['events_per_cycle'] = snapshot['attempted'] / (5 * iterations + 12)
        comparisons = {}
        for name in ('disabled', 'enabled'):
            comparisons[name + '_vs_baseline'] = {
                clock: dict(median_delta_ns=outputs[name][clock]['median'] - outputs['baseline'][clock]['median'],
                    median_ratio=outputs[name][clock]['median'] / outputs['baseline'][clock]['median'],
                    mean_delta_ns=outputs[name][clock]['mean'] - outputs['baseline'][clock]['mean'],
                    mean_ratio=outputs[name][clock]['mean'] / outputs['baseline'][clock]['mean'])
                for clock in ('wall_ns', 'thread_cpu_ns', 'process_cpu_ns')}
        report['cases'][label] = dict(cloud_payload_bytes=len(data['cloud'].data),
            point_count=data['cloud'].width, point_step=data['cloud'].point_step,
            samples_per_mode=5 * iterations, warmup_cycles_per_mode=12, measurements=outputs,
            comparisons=comparisons)
    report['memory_and_cpu_examples'] = memory_and_cpu_examples()
    report['monotonic_finished_ns'] = time.monotonic_ns()
    report['load_average_end'] = os.getloadavg()
    report['gc_enabled'] = gc.isenabled()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, help='write exit-only benchmark summary JSON')
    parser.add_argument('--skip-tests', action='store_true')
    args = parser.parse_args()
    if not args.skip_tests:
        result = unittest.TextTestRunner(verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(CoreProbeTests))
        if not result.wasSuccessful():
            raise SystemExit(1)
    if args.benchmark:
        report = benchmark()
        args.benchmark.write_text(json.dumps(report, indent=2) + '\n')
        print('Benchmark written:', args.benchmark)
        for name, case in report['cases'].items():
            print(name, {mode: round(measurement['wall_ns']['median'] / 1000, 3)
                for mode, measurement in case['measurements'].items()}, 'median wall microseconds')


if __name__ == '__main__':
    main()
