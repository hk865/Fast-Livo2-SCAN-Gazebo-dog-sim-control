"""No-node coordinator ownership/FIFO tests and real static-geometry timing.

The coordinator is production code. FakeWorker and MinNode are explicit unit
boundaries, not navigation acceptance. The benchmark uses the real geometry
leaf and real independent Worker but a small result sink; coordinator resume
time is not the full production admission transaction or a control deadline.
"""
import argparse
import ast
from collections import deque
import copy
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import r88_async_admission as service
from mode3_trajectory_contract import payload

HERE = Path(__file__).resolve().parent
FIXTURE = Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/'
    '20261018T113004Z_closed_loop_R85_R87_diagnostic_round02_9522/'
    'path_admission_candidates/000124_trajectory_126.json')


def message(identifier=2):
    return NS(order=3, traj_id=identifier, start_time=NS(sec=1, nanosec=0),
        pos_pts=[NS(x=x, y=0., z=0.) for x in (0., 1/3, 2/3, 1.)],
        knots=[0., 0., 0., 0., 1., 1., 1., 1.], yaw_pts=[], yaw_dt=0.)


class FakeWorker:
    """Only producer completion is concurrent; control methods retain one owner."""
    def __init__(self):
        self.busy = False
        self.pid = 123456789
        self.pending = None
        self.results = deque()
        self.submissions = []
        self.closed = False
        self.submit_error = None

    def submit(self, token, value):
        if self.submit_error:
            raise self.submit_error
        if self.busy:
            raise AssertionError('test observed overlapping worker jobs')
        self.busy, self.pending = True, token
        self.submissions.append((token, copy.deepcopy(value)))

    def finish(self, *, token=None, allowed=True, error=None):
        token = self.pending if token is None else token
        self.results.append((token, {'allowed': allowed}, {'error': error} if error else {}))

    def poll(self):
        if not self.results:
            return None
        result = self.results.popleft()
        if result[0] == self.pending:
            self.busy, self.pending = False, None
        return result

    def close(self):
        self.closed, self.busy = True, False
        return dict(exited=True, worker_pid=self.pid)


class MinNode:
    def __init__(self):
        self.run = Path('/r88-unit-no-write')
        self.rows, self.order, self.resumes = [], [], []
        self.evidence = NS(append=lambda path, row: self.rows.append(copy.deepcopy(row)))
        self.clock = NS(nanoseconds=1_100_000_000)
        self.pose_stamp = self.cloud_stamp = 1_000_000_000
        self.request_id, self.waypoint_index = 'request-a', 0
        self.state, self.message = 'running', ''
        self.automatic_reference = NS(scope=(2, 123, 1), file_sha='f' * 64,
            reference={'sent_reference': {'stamp': [1, 0], 'frame_id': 'camera_init'}})
        self.trajectory_association = NS(epoch=2, planner_process_id=123,
            localization_generation=1, reference_stamp=[1, 0], reference_revision=1,
            accepted=payload(message()), accepted_metadata={'id': 2}, accepted_highwater_id=2,
            accepted_attempt_highwater_id=2, goal=[2., 0., 0.])
        self.known_scene_inputs = {'generation': 1}
        self.known_scene_reference_generation = 1
        self.known_scene_commit_token = ('a' * 32, 10)
        self.continuous_route = NS(next_index=2)
        self.scan_reference_coverage_index = 2
        self.active_trajectory_id, self.active_reference_stamp = 1, (1, 0)
        self.cascade_path_id = 'old-path'
        self.path_admission_fence = NS(load=lambda: ([], {}, {'fixture': True}))
        self.r85_physical_gate = NS(prepare=lambda node, value, metadata, binding:
            {'physical_payload': copy.deepcopy(value), 'fixture': True})
        self.command = [.1, 0., 0.]
        self.zero_count = 0
        self.last_status = -float('inf')
        self.statuses = []

    def admission_identity(self):
        return {'request': self.request_id, 'waypoint': self.waypoint_index}

    def get_clock(self):
        return NS(now=lambda: self.clock)

    def publish_command(self, *args, **kwargs):
        self.zero_count += 1
        self.command = [0., 0., 0.]
        self.order.append('zero')

    def publish_status(self):
        self.statuses.append(self.state)

    def _admit_matched_spline(self, msg, metadata, **kwargs):
        self.order.append(('admission', msg.traj_id))
        self.resumes.append(dict(msg=msg, metadata=metadata, **kwargs))

    def tick(self):
        self.order.append('control_tick')


class CoordinatorTests(unittest.TestCase):
    def make(self):
        node, worker = MinNode(), FakeWorker()
        admission = service.Admission(node, worker)
        node._r88_admission = admission
        return node, worker, admission

    def test_fifo_negative_then_positive_and_single_flight(self):
        node, worker, admission = self.make()
        admission.start(message(2), {'id': 2})
        admission.receive('attempt', NS(data='negative'),
            lambda msg: node.order.append(('negative', msg.data)))
        admission.receive('spline', message(3), lambda msg: admission.start(msg, {'id': 3}))
        admission.receive('metadata', NS(data='after'), lambda msg: node.order.append(msg.data))
        admission.poll()
        self.assertEqual(node.order, [])
        self.assertEqual(len(worker.submissions), 1)
        worker.finish(allowed=False)
        admission.poll()
        self.assertEqual(node.order, [('admission', 2), ('negative', 'negative')])
        self.assertEqual([token for token, _ in worker.submissions], [1, 2])
        self.assertEqual(len(admission.events), 1)
        worker.finish()
        admission.poll()
        self.assertEqual(node.order[-2:], [('admission', 3), 'after'])
        self.assertEqual([r['physical_result']['proof']['allowed'] for r in node.resumes], [False, True])
        self.assertEqual(admission.queue_bytes, 0)

    def test_all_owner_scope_fields_invalidate_without_resuming(self):
        changes = {
            'request': lambda n: setattr(n, 'request_id', 'request-b'),
            'waypoint': lambda n: setattr(n, 'waypoint_index', 1),
            'state': lambda n: setattr(n, 'state', 'failed'),
            'runtime_scope': lambda n: setattr(n.automatic_reference, 'scope', (3, 123, 1)),
            'reference_file_sha': lambda n: setattr(n.automatic_reference, 'file_sha', 'new'),
            'sent_reference': lambda n: n.automatic_reference.reference['sent_reference'].update(stamp=[2, 0]),
            'generation': lambda n: n.known_scene_inputs.update(generation=2),
            'requested_generation': lambda n: setattr(n, 'known_scene_reference_generation', 2),
            'coverage': lambda n: setattr(n.continuous_route, 'next_index', 3),
            'requested_coverage': lambda n: setattr(n, 'scan_reference_coverage_index', 3),
            'active_trajectory': lambda n: setattr(n, 'active_trajectory_id', 2),
            'active_reference': lambda n: setattr(n, 'active_reference_stamp', (2, 0)),
            'cascade_path': lambda n: setattr(n, 'cascade_path_id', 'new-path'),
        }
        for field in ('epoch', 'planner_process_id', 'localization_generation',
                'reference_stamp', 'reference_revision', 'accepted', 'accepted_metadata',
                'accepted_highwater_id', 'accepted_attempt_highwater_id', 'goal'):
            changes['association.' + field] = lambda node, key=field: setattr(node.trajectory_association, key, None)
        for label, change in changes.items():
            with self.subTest(field=label):
                node, worker, admission = self.make()
                admission.start(message(), {})
                change(node)
                worker.finish()
                admission.poll()
                self.assertEqual(node.resumes, [])
                self.assertIsNone(admission.current)
                self.assertEqual(admission.counts['invalidated'], 1)

    def test_source_epoch_commit_rollback_and_normal_progress(self):
        for token, valid in ((('a'*32, 10), True), (('a'*32, 11), True),
                             (('a'*32, 9), False), (('b'*32, 11), False),
                             (('a'*32, True), False), (['a'*32, 11], False), (None, False)):
            with self.subTest(token=token):
                node, worker, admission = self.make()
                admission.start(message(), {})
                node.known_scene_commit_token = token
                node.pose_stamp += 20_000_000  # Advancing live feedback is deliberately allowed.
                worker.finish()
                admission.poll()
                self.assertEqual(bool(node.resumes), valid)

    def test_pending_scope_or_source_loss_cannot_revive_after_aba(self):
        for changed_field in ('goal', 'source'):
            with self.subTest(changed_field=changed_field):
                node, worker, admission = self.make()
                admission.start(message(), {})
                admission.receive('metadata', NS(data='old queued event'),
                    lambda msg: node.order.append('unexpected queued event'))
                original_goal = copy.deepcopy(node.trajectory_association.goal)
                original_source = node.known_scene_commit_token
                if changed_field == 'goal':
                    node.trajectory_association.goal = [3., 0., 0.]
                else:
                    node.known_scene_commit_token = ('b'*32, 10)
                admission.poll()  # No worker result exists yet; observe owner loss.
                self.assertTrue(worker.busy)
                self.assertIsNone(admission.current)
                self.assertEqual(admission.counts['invalidated'], 1)
                self.assertEqual(len(admission.events), 0)
                invalidation = next(row for row in node.rows if row['event'] == 'owner_invalidated')
                self.assertEqual(invalidation['reason'], 'owner_changed_while_compute_pending')
                node.trajectory_association.goal = original_goal
                node.known_scene_commit_token = original_source
                worker.finish()
                admission.poll()
                self.assertEqual(node.resumes, [])
                self.assertEqual(node.order, [])
                self.assertEqual(node.state, 'running')
                self.assertEqual(node.command, [.1, 0., 0.])
                self.assertEqual(node.rows[-1]['event'], 'orphan_result')

    def test_real_request_method_invalidates_accepted_change_and_late_aba_cannot_overwrite(self):
        tree = ast.parse((HERE/'controller.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PIDNavigation')
        method = copy.deepcopy(next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_request'))
        class Base(MinNode):
            def on_request(self, msg):
                self.order.append(('request_base', self._r88_admission.current is None))
                self.request_id = msg.data
        subclass = ast.ClassDef(name='PIDNavigation', bases=[ast.Name(id='Base', ctx=ast.Load())],
            keywords=[], body=[method], decorator_list=[])
        namespace = {'Base': Base, 'time': time}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[subclass], type_ignores=[])),
                     '<actual-r88-on-request>', 'exec'), namespace)
        node, worker = namespace['PIDNavigation'](), FakeWorker()
        node._r85_protocol_liveness_enabled = False
        node.cascade = NS(old=True)
        node.continuous_settings = {'enabled': False}
        admission = node._r88_admission = service.Admission(node, worker)
        admission.start(message(2), {})
        admission.receive('metadata', NS(data='queued-old'), lambda msg: node.order.append(msg.data))
        first_owner = admission.current
        node.on_request(NS(data='request-a'))
        self.assertIs(admission.current, first_owner)
        node.on_request(NS(data='request-b'))
        node.on_request(NS(data='request-a'))  # Same textual scope again must not resurrect job 1.
        worker.finish(token=1)
        admission.poll()
        self.assertEqual(node.resumes, [])
        self.assertEqual(node.order, [('request_base', False), ('request_base', False),
                                     'zero', ('request_base', True), 'zero'])
        admission.start(message(3), {})
        new_owner = admission.current
        worker.finish(token=1)  # Defensive stale-token injection while job 2 exists.
        admission.poll()
        self.assertIs(admission.current, new_owner)
        self.assertEqual(node.resumes, [])
        worker.finish(token=2)
        admission.poll()
        self.assertEqual([row['msg'].traj_id for row in node.resumes], [3])

    def test_original_receipt_start_and_queued_message_copy_preserved(self):
        node, worker, admission = self.make()
        worker.busy = True
        msg = message(2)
        admission.receive('spline', msg,
            lambda value: admission.start(value, {'id': 2}, admission_started=12.75))
        original_receipt = admission.events[0]['received_wall_ns']
        original_ros = admission.events[0]['received_ros_ns']
        msg.traj_id = 999
        worker.busy = False
        node.clock.nanoseconds += 10_000_000
        admission.poll()
        self.assertEqual(admission.current['msg'].traj_id, 2)
        self.assertEqual(admission.current['started'], 12.75)
        self.assertEqual(admission.current['received_wall_ns'], original_receipt)
        worker.finish()
        admission.poll()
        self.assertEqual(node.resumes[0]['received_wall_ns'], original_receipt)
        self.assertEqual(node.resumes[0]['admission_started'], 12.75)
        consumed = next(row for row in node.rows if row['event'] == 'event_consumed')
        self.assertEqual(consumed['original_received_ros_ns'], original_ros)

    def test_queue_count_and_byte_bounds_fail_closed_and_drop_backlog(self):
        for kind in ('count', 'bytes'):
            node, worker, admission = self.make()
            admission.start(message(), {})
            with patch.object(service, 'MAX_EVENTS', 2 if kind == 'count' else 32),\
                 patch.object(service, 'MAX_QUEUE_BYTES', 100 if kind == 'bytes' else 2**20):
                for _ in range(3):
                    admission.receive('metadata', NS(data='x'*60), lambda msg: node.order.append('unexpected'))
            self.assertEqual(admission.fault, 'r88_scan_event_queue_bound')
            self.assertEqual(node.state, 'failed')
            self.assertEqual(node.command, [0., 0., 0.])
            self.assertEqual(admission.queue_bytes, 0)
            self.assertEqual(len(admission.events), 0)
            worker.finish()
            admission.poll()
            self.assertEqual(node.resumes, [])

    def test_message_count_and_utf8_byte_limits_fail_before_callback(self):
        too_many_points = message(); too_many_points.pos_pts *= 129
        too_many_knots = message(); too_many_knots.knots *= 65
        for kind, msg in (('spline', too_many_points), ('spline', too_many_knots),
                          ('metadata', NS(data='x'*(service.MAX_EVENT_BYTES+1))),
                          ('metadata', NS(data='中'*(service.MAX_EVENT_BYTES//2)))):
            node, worker, admission = self.make()
            admission.receive(kind, msg, lambda value: node.order.append('unexpected'))
            self.assertEqual(node.state, 'failed')
            self.assertEqual(admission.counts['consumed'], 0)
            self.assertEqual(worker.submissions, [])

    def test_drain_is_bounded_to_three_callbacks_and_preserves_all_order(self):
        node, worker, admission = self.make()
        worker.busy = True
        for index in range(8):
            admission.receive('attempt', NS(data=str(index)), lambda msg: node.order.append(msg.data))
        worker.busy = False
        admission.poll()
        self.assertEqual(node.order, ['0', '1', '2'])
        admission.poll()
        self.assertEqual(node.order, ['0', '1', '2', '3', '4', '5'])
        admission.poll()
        self.assertEqual(node.order, list(map(str, range(8))))
        self.assertEqual(admission.queue_bytes, 0)

    def test_worker_error_uses_denial_then_fail_closed_without_next_callback(self):
        node, worker, admission = self.make()
        admission.start(message(), {})
        admission.receive('attempt', NS(data='later'), lambda msg: node.order.append('unexpected'))
        worker.finish(error='injected physical failure')
        admission.poll()
        self.assertEqual(node.resumes[0]['physical_result'], {'error': 'injected physical failure'})
        self.assertEqual(node.order, [('admission', 2), 'zero'])
        self.assertEqual(node.state, 'failed')
        self.assertEqual(len(admission.events), 0)

    def test_worker_error_after_invalidation_still_fail_closed(self):
        node, worker, admission = self.make()
        admission.start(message(), {})
        admission.invalidate('request-change')
        admission.receive('attempt', NS(data='new-request'), lambda msg: node.order.append('unexpected'))
        worker.finish(error='injected physical failure after invalidation')
        admission.poll()
        self.assertEqual(node.resumes, [])
        self.assertEqual(node.state, 'failed')
        self.assertEqual(node.command, [0., 0., 0.])
        self.assertNotIn('unexpected', node.order)

    def test_post_submit_evidence_failure_does_not_duplicate_admission(self):
        node, worker, admission = self.make()
        original = node.evidence.append
        def write(path, row):
            if row['event'] == 'submitted':
                raise OSError('injected observer sink failure')
            original(path, row)
        node.evidence.append = write
        with self.assertRaisesRegex(OSError, 'observer sink'):
            admission.start(message(), {})
        self.assertTrue(worker.busy)
        self.assertIsNone(admission.current)
        self.assertEqual(node.resumes, [])
        self.assertEqual(node.state, 'failed')
        self.assertEqual(node.command, [0., 0., 0.])

    def test_record_failure_sets_failed_and_zero_before_propagating(self):
        for operation in ('record', 'fail'):
            with self.subTest(operation=operation):
                node, worker, admission = self.make()
                publication_states = []
                publish = node.publish_command
                def zero(*args, **kwargs):
                    publication_states.append(node.state)
                    return publish(*args, **kwargs)
                node.publish_command = zero
                node.evidence.append = lambda *args: (_ for _ in ()).throw(OSError('record disk failure'))
                with self.assertRaisesRegex(OSError, 'record disk failure'):
                    if operation == 'record':
                        admission.record('fixture_event')
                    else:
                        admission.fail('fixture_fatal')
                self.assertTrue(publication_states)
                self.assertTrue(all(state == 'failed' for state in publication_states))
                self.assertEqual(node.state, 'failed')
                self.assertEqual(node.command, [0., 0., 0.])
                self.assertEqual(admission.fault, 'r88_async_evidence_failed')
                self.assertEqual(node.resumes, [])

    def test_close_reaps_worker_even_when_invalidation_record_fails(self):
        node, worker, admission = self.make()
        admission.start(message(), {})
        admission.receive('metadata', NS(data='queued'), lambda msg: node.order.append('unexpected'))
        node.evidence.append = lambda *args: (_ for _ in ()).throw(OSError('close record failure'))
        receipt = admission.close()
        self.assertTrue(worker.closed)
        self.assertTrue(receipt['worker']['exited'])
        self.assertIn('OSError:close record failure', receipt['evidence_error'])
        self.assertIsNone(admission.current)
        self.assertEqual(admission.queue_bytes, 0)
        self.assertEqual(len(admission.events), 0)
        self.assertEqual(admission.counts['discarded_events'], 1)
        self.assertIsNone(admission.close())

    def test_concurrent_worker_completion_only_resumes_on_owner_poll(self):
        node, worker, admission = self.make()
        owner_tid = threading.get_native_id()
        admission.start(message(), {})
        thread = threading.Thread(target=worker.finish)
        thread.start(); thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(node.resumes, [])
        seen = []
        original = node._admit_matched_spline
        node._admit_matched_spline = lambda *args, **kwargs: (
            seen.append(threading.get_native_id()), original(*args, **kwargs))
        admission.poll()
        self.assertEqual(seen, [owner_tid])

    def test_close_is_idempotent_and_future_callbacks_are_discarded(self):
        node, worker, admission = self.make()
        admission.start(message(), {})
        result = admission.close()
        self.assertTrue(result['worker']['exited'])
        self.assertIsNone(admission.close())
        admission.receive('attempt', NS(data='closed'), lambda msg: node.order.append('unexpected'))
        worker.finish(token=1)
        admission.poll()
        self.assertEqual(node.resumes, [])
        self.assertEqual(admission.counts['discarded_events'], 1)


class FeedbackOrderTests(unittest.TestCase):
    def test_real_tick_poll_exception_publishes_zero_then_preserves_exception(self):
        from known_scene_inputs import KnownSceneInputMixin
        class Node(KnownSceneInputMixin, MinNode):
            def __init__(self):
                MinNode.__init__(self)
                self._r86_prepared_mailbox = None
        class PollFailure(RuntimeError):
            pass
        node = Node()
        def poll():
            node.order.append('poll')
            raise PollFailure('original poll failure')
        node._r88_admission = NS(poll=poll)
        with self.assertRaisesRegex(PollFailure, 'original poll failure'):
            node.tick()
        self.assertEqual(node.order, ['poll', 'zero'])
        self.assertEqual(node.state, 'failed')
        self.assertEqual(node.command, [0., 0., 0.])
        self.assertIn('PollFailure:original poll failure', node.message)

    def test_real_mailbox_tick_preserves_feedback_and_blocks_async_after_bad_frame(self):
        from known_scene_inputs import KnownSceneInputMixin
        from point_provenance import verify_corrected
        from r86_prepared_feedback import ros_mailbox
        from r86_prepared_feedback_tests import feed, packet
        class Node(KnownSceneInputMixin, MinNode):
            def __init__(self, bad_source):
                MinNode.__init__(self)
                self._r86_prepared_mailbox = ros_mailbox(verify_corrected)
                self._r86_committing_bundle = self._r86_scope = None
                self._r86_scope_since_ns = 0
                self.feedback = None
                self.bad_source = bad_source
                self.known_scene_callbacks = {'body': self.body, 'cloud': self.cloud}
            def body(self, msg):
                ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                self.order.append(('body', ns))
                if ns != self.bad_source:
                    self.pose_stamp = ns
            def cloud(self, msg):
                ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                self.order.append(('cloud', ns))
                self.cloud_stamp = ns
        stamps = [1_040_000_000, 1_050_000_000, 1_060_000_000]
        for bad in (None, stamps[1]):
            with self.subTest(bad_source=bad):
                node, worker = Node(bad), FakeWorker()
                admission = node._r88_admission = service.Admission(node, worker)
                admission.start(message(), {})
                worker.finish()
                for index, stamp in enumerate(stamps):
                    feed(node._r86_prepared_mailbox, packet(stamp, commit=11+index))
                node.tick()
                if bad is None:
                    self.assertEqual(node.order, [(kind, stamp) for stamp in stamps
                        for kind in ('body', 'cloud')] + [('admission', 2), 'control_tick'])
                    self.assertEqual(node.known_scene_commit_token, ('a'*32, 13))
                    self.assertEqual(len(node.resumes), 1)
                else:
                    self.assertEqual(node.order[:3], [('body', stamps[0]), ('cloud', stamps[0]), ('body', stamps[1])])
                    self.assertEqual(node.order[3:], ['zero'])
                    self.assertEqual(node.resumes, [])
                    self.assertEqual(node.state, 'failed')
                    self.assertEqual(node._r86_prepared_mailbox.counts['committed'], 1)
                    self.assertEqual(node._r86_prepared_mailbox.snapshot()['inflight_frames'], 0)


def canonical(proof):
    return {key: value for key, value in proof.items() if key != 'elapsed_wall_ms'}


def summary(values):
    ordered = sorted(values)
    return dict(n=len(values), median=statistics.median(values), mean=statistics.mean(values),
        minimum=min(values), maximum=max(values), p95=ordered[int(.95*(len(values)-1))])


def wait(worker, timeout=8.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = worker.poll()
        if result is not None:
            return result
        time.sleep(.002)
    raise AssertionError('bounded worker timeout')


def benchmark():
    import r85_geometry_admission as geometry
    from r88_physical_worker import Worker
    document = json.loads(FIXTURE.read_text())
    physical = document['decision']['proof']['r85_physical']['physical_payload']
    value = document['payload']
    msg = NS(order=value['order'], traj_id=value['traj_id'],
        start_time=NS(sec=value['start_time'][0], nanosec=value['start_time'][1]),
        pos_pts=[NS(x=p[0], y=p[1], z=p[2]) for p in value['pos_pts']],
        knots=value['knots'], yaw_pts=value['yaw_pts'], yaw_dt=value['yaw_dt'])
    report = dict(schema='r88_real_geometry_control_availability/v1',
        fixture=str(FIXTURE), fixture_sha256=hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
        physical_payload_sha256=service.digest(physical),
        source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (HERE/'r88_async_admission.py', HERE/'r88_physical_worker.py', HERE/'r85_geometry_admission.py', Path(__file__))},
        limits='Observed finite no-node measurements, not latency bounds, navigation PASS, or a physics/controller run.',
        result_dispatch_limit='MinNode result sink: coordinator scope/source revalidation and dispatch are measured; full production live guards/atomic admission are not benchmarked here.',
        runtime=dict(python=sys.version, native_tid=threading.get_native_id(), pid=os.getpid(),
            affinity=sorted(os.sched_getaffinity(0)), load_average=os.getloadavg()),
        loop_period_ns=50_000_000, loop_steps=11, repeats=4,
        synchronous=[], asynchronous=[], ipc_observations=[])
    baseline_proof = geometry.certify_physical_curve(physical)
    worker = Worker()
    try:
        worker.submit(0, physical)
        warm = wait(worker)
        if canonical(warm[1]) != canonical(baseline_proof):
            raise AssertionError('warmup worker proof differs from synchronous proof')
        report['worker_startup_including_imports'] = warm[2]
        for iteration in range(6):
            started, cpu = time.monotonic_ns(), time.thread_time_ns()
            worker.submit(100+iteration, physical)
            submit_elapsed = time.monotonic_ns()-started
            result = wait(worker)
            total = time.monotonic_ns()-started
            if canonical(result[1]) != canonical(baseline_proof):
                raise AssertionError('IPC proof parity failed')
            metrics = result[2]
            report['ipc_observations'].append(dict(submit_wall_ns=submit_elapsed,
                total_wall_ns=total, owner_thread_cpu_ns=time.thread_time_ns()-cpu,
                noncompute_wall_ns=total-metrics['compute_wall_ns'],
                noncompute_limit='Includes JSON/pipe transport and 2ms polling resolution; not isolated IPC CPU.',
                worker=metrics))
        for repeat in range(4):
            for mode in (('synchronous', 'asynchronous') if repeat % 2 == 0 else ('asynchronous', 'synchronous')):
                node = MinNode()
                node.r85_physical_gate.prepare = lambda *args: {'physical_payload': copy.deepcopy(physical)}
                admission = service.Admission(node, worker)
                # Avoid token reuse with the persistent real worker across trials.
                admission.sequence = 1000 + repeat*10
                steps = []
                result_proof = None
                begun, owner_cpu = time.monotonic_ns(), time.thread_time_ns()
                compute = None
                for index in range(11):
                    deadline = begun + index*50_000_000
                    delay = (deadline-time.monotonic_ns())/1e9
                    if delay > 0:
                        time.sleep(delay)
                    entry, cpu = time.monotonic_ns(), time.thread_time_ns()
                    if index == 0:
                        if mode == 'synchronous':
                            wall, cpu_compute = time.monotonic_ns(), time.thread_time_ns()
                            result_proof = geometry.certify_physical_curve(physical)
                            compute = dict(wall_ns=time.monotonic_ns()-wall,
                                thread_cpu_ns=time.thread_time_ns()-cpu_compute)
                        else:
                            admission.start(msg, document['metadata'])
                    if mode == 'asynchronous':
                        admission.poll()
                    end = time.monotonic_ns()
                    steps.append(dict(index=index, scheduled_ns=deadline, started_ns=entry,
                        lateness_ns=max(0, entry-deadline), step_wall_ns=end-entry,
                        step_thread_cpu_ns=time.thread_time_ns()-cpu,
                        result_consumed=bool(node.resumes)))
                if mode == 'asynchronous':
                    if not node.resumes:
                        raise AssertionError('worker did not finish within bounded 500ms measurement window')
                    result_proof = node.resumes[0]['physical_result']['proof']
                    metrics = next(row['metrics'] for row in node.rows if row['event']=='result_consumed')
                    compute = metrics
                if canonical(result_proof) != canonical(baseline_proof):
                    raise AssertionError('control-loop geometry proof differs')
                report[mode].append(dict(repeat=repeat, elapsed_wall_ns=time.monotonic_ns()-begun,
                    owner_thread_cpu_ns=time.thread_time_ns()-owner_cpu, steps=steps, compute=compute,
                    missed_50ms_lateness=sum(step['lateness_ns'] >= 50_000_000 for step in steps),
                    missed_5ms_lateness=sum(step['lateness_ns'] >= 5_000_000 for step in steps),
                    max_lateness_ns=max(step['lateness_ns'] for step in steps),
                    max_step_wall_ns=max(step['step_wall_ns'] for step in steps),
                    proof_equal_to_synchronous=True))
    finally:
        report['worker_exit'] = worker.close()
    for mode in ('synchronous', 'asynchronous'):
        report[mode+'_summary'] = {key: summary([row[key] for row in report[mode]]) for key in
            ('owner_thread_cpu_ns', 'max_lateness_ns', 'max_step_wall_ns', 'missed_50ms_lateness', 'missed_5ms_lateness')}
        if mode == 'asynchronous':
            commit_steps = []
            for trial in report[mode]:
                commit_steps.append(next(step['step_wall_ns'] for step in trial['steps'] if step['result_consumed']))
            report[mode+'_summary']['result_validation_dispatch_step_wall_ns'] = summary(commit_steps)
    report['ipc_summary'] = {key: summary([row[key] for row in report['ipc_observations']]) for key in
        ('submit_wall_ns', 'total_wall_ns', 'owner_thread_cpu_ns', 'noncompute_wall_ns')}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path)
    parser.add_argument('--skip-tests', action='store_true')
    args = parser.parse_args()
    if not args.skip_tests:
        suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
        if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
            raise SystemExit(1)
    if args.benchmark:
        report = benchmark()
        args.benchmark.write_text(json.dumps(report, indent=2)+'\n')
        print('Bounded real geometry benchmark:', args.benchmark)
        for mode in ('synchronous', 'asynchronous'):
            print(mode, report[mode+'_summary'])


if __name__ == '__main__':
    main()
