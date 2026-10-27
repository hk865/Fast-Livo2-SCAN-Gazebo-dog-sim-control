"""Three bounded processed matcher calls, on the existing evidence writer lane.

Observation only. Never changes a match result, clock, quality gate or control.
"""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time
import numpy as np

KEY = 'MATCH_QUERY_CAPTURE'
MAX_POINTS = 30720
QUERY_BYTES = MAX_POINTS * 3 * 8
TOTAL_BYTES = 8 * 1024 * 1024
REFERENCE_BYTES = 1024 * 1024
METADATA_BYTES = 64 * 1024
RESERVE_BYTES = 512 * 1024
START_NS, END_NS = 150_000_000_000, 190_000_000_000


def create(run, matcher, writer):
    selected = os.environ.get(KEY, '0')
    if selected not in ('0', '1'):
        raise ValueError(KEY + ' must be 0 or 1')
    return QueryCapture(run, matcher, writer) if selected == '1' else None


class QueryCapture:
    def __init__(self, run, matcher, writer):
        self.run = Path(run)
        self.root = self.run / 'matcher_query_capture'
        self.writer = writer
        self.lock = threading.Lock()
        self.written = threading.Event()
        self.closed = self.done = self.enqueued = self.complete = False
        self.current = self.good = self.bad = None
        self.selected = []
        self.error = None
        self.counts = dict(prepared=0, outside_window=0, excluded=0,
            replaced_good=0, interrupted_bad=0, missing_prepared=0,
            query_copies=0, small_matrix_copy_bytes=0, queue_drops=0)
        self.prepare_wall_ns = self.prepare_cpu_ns = self.prepare_max_wall_ns = 0
        self.finish_wall_ns = self.finish_cpu_ns = 0
        self.encode_wall_ns = self.encode_cpu_ns = 0
        self.write_wall_ns = self.write_cpu_ns = 0
        self.files = {}
        self.usage = dict(logical_bytes=0, allocated_bytes=0)
        self.stop_reason = None
        self.policy = asdict(matcher.policy)
        self.policy_sha = hashlib.sha256(self._json(self.policy)).hexdigest()
        self.reference_sha = matcher.reference.sha256
        self.world_sha = matcher.reference.source['sha256']
        here = Path(__file__).resolve().parent
        self.sources = {name: hashlib.sha256((here / name).read_bytes()).hexdigest()
            for name in ('known_scene_matcher.py', 'known_scene_adapter.py', 'match_query_capture.py')}

    @staticmethod
    def _json(value):
        return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()

    def note_error(self, error):
        with self.lock:
            if self.error is None:
                self.error = str(error)[:240]

    def prepared(self, source_ns, query, odom, current_c, mount, prediction, seeds,
                 provenance, finite_points, pending, confirmations):
        """After original fitting/scoring and duration sampling, before qualification.

        ExactSourceJoin allocates body_points as adapted_xyz[indices]+mount for
        this one worker call. Matcher only reads it; retaining it needs no cloud
        copy. Small pose/seed/state matrices are copied once to freeze values.
        """
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        try:
            with self.lock:
                if self.closed or self.done:
                    return
                if not START_NS <= source_ns <= END_NS:
                    self.counts['outside_window'] += 1
                    expire = source_ns > END_NS
                else:
                    expire = False
                    q = np.asarray(query)
                    if (q.dtype != np.dtype('float64') or q.ndim != 2 or q.shape[1] != 3
                            or not q.flags.c_contiguous or not q.flags.owndata
                            or len(q) > MAX_POINTS or q.nbytes > QUERY_BYTES):
                        self.counts['excluded'] += 1
                        raise ValueError('private_float64_query_shape_or_byte_bound')
                    arrays = [np.array(x, dtype=np.float64, copy=True)
                        for x in (odom, current_c, mount, prediction, *seeds)]
                    prior = None if pending is None else (pending[0], np.array(pending[1], copy=True))
                    self.current = dict(source_ns=source_ns, query=q,
                        arrays=arrays, seed_count=len(seeds), finite_points=finite_points,
                        provenance=provenance, pending=prior, confirmations=confirmations)
                    self.counts['prepared'] += 1
                    self.counts['small_matrix_copy_bytes'] += sum(a.nbytes for a in arrays) + (0 if prior is None else prior[1].nbytes)
            if expire:
                self.finalize('window_ended')
        except Exception as error:
            self.note_error('prepare:' + type(error).__name__ + ':' + str(error))
            self.finalize('input_not_capturable')
        finally:
            elapsed = time.monotonic_ns() - wall
            with self.lock:
                self.prepare_wall_ns += elapsed
                self.prepare_cpu_ns += time.thread_time_ns() - cpu
                self.prepare_max_wall_ns = max(self.prepare_max_wall_ns, elapsed)

    def finish(self, job, result, parent_generation):
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        try:
            with self.lock:
                if self.closed or self.done or not START_NS <= job.get('source_ns', -1) <= END_NS:
                    return
                snapshot, self.current = self.current, None
                if snapshot is None or snapshot['source_ns'] != job.get('source_ns'):
                    self.counts['missing_prepared'] += 1
                    raise ValueError('actual_match_input_not_captured')
                snapshot['parent_generation'] = parent_generation
                snapshot['received_wall_ns'] = job.get('original_received_wall_ns')
                snapshot['result'] = dict(accepted=result.get('accepted'), reason=result.get('reason'),
                    evidence_verified=result.get('evidence_verified'),
                    completed_seeds=result.get('completed_seeds'),
                    candidates=[dict(initial_position=c.get('initial_position'),
                        quality_passed=c.get('quality_passed'), rejections=c.get('rejections'),
                        static_surface_rms_capped_m=c.get('static_surface_rms_capped_m'),
                        foreground_fraction=c.get('foreground', {}).get('fraction'),
                        surface_p95_m=c.get('surface_p95_m'), rays=c.get('rays'))
                        for c in result.get('candidates', [])])
                good = result.get('accepted') is True and result.get('evidence_verified') is True
                if good:
                    self.counts['replaced_good'] += int(self.good is not None)
                    self.counts['interrupted_bad'] += int(self.bad is not None)
                    self.good, self.bad = snapshot, None
                    selected = None
                elif self.bad is None:
                    self.bad = snapshot
                    selected = None
                else:
                    selected = [(role, value) for role, value in
                        [('last_good', self.good), ('first_bad', self.bad), ('second_bad', snapshot)]
                        if value is not None]
            if selected is not None:
                self._enqueue(selected, 'first_two_consecutive_bad')
        except Exception as error:
            self.note_error('finish:' + type(error).__name__ + ':' + str(error))
            self.finalize('input_not_capturable')
        finally:
            with self.lock:
                self.finish_wall_ns += time.monotonic_ns() - wall
                self.finish_cpu_ns += time.thread_time_ns() - cpu

    def finalize(self, reason):
        with self.lock:
            if self.done:
                return
            selected = [(role, value) for role, value in
                [('last_good', self.good), ('first_bad', self.bad)] if value is not None]
        self._enqueue(selected, reason)

    def _enqueue(self, records, reason):
        with self.lock:
            if self.done:
                return
            self.done = True
            self.stop_reason = reason
            self.current = self.good = self.bad = None
            self.selected = [dict(role=role, source_ns=s['source_ns']) for role, s in records]
        if not records:
            self.written.set()
            return
        try:
            if self.writer.error:
                raise RuntimeError('existing_writer_error:' + str(self.writer.error))
            # Existing EvidenceWriter has no submitted counter/closed flag.
            # Queue.put/task_done retain its native unfinished/completed accounting.
            # Its enqueue() poisons global error on Full; this observer must not.
            self.writer.queue.put_nowait(lambda: self._save(records))
            self.enqueued = True
        except queue.Full:
            with self.lock:
                self.counts['queue_drops'] += 1
            self.note_error('existing_writer_queue_full')
            self.written.set()
        except Exception as error:
            self.note_error('enqueue:' + type(error).__name__ + ':' + str(error))
            self.written.set()

    def _save(self, records):
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        try:
            if len(records) > 3 or sum(s['query'].nbytes for _, s in records) + REFERENCE_BYTES + 4 * METADATA_BYTES + RESERVE_BYTES > TOTAL_BYTES:
                raise ValueError('capture_total_reservation')
            with (self.run / 'known_scene_reference.json').open('rb') as stream:
                reference = stream.read(REFERENCE_BYTES + 1)
            if len(reference) > REFERENCE_BYTES or hashlib.sha256(reference).hexdigest() != self.reference_sha:
                raise ValueError('capture_reference_changed_or_oversize')
            self.root.mkdir(exist_ok=False)
            (self.root / 'REFERENCE.json').write_bytes(reference)
            for role, s in records:
                stem = role + '_' + str(s['source_ns'])
                with (self.root / (stem + '.npy')).open('xb') as stream:
                    np.save(stream, s['query'], allow_pickle=False)
                arrays = s['arrays']
                metadata = dict(schema='known_scene_matcher_call_capture/v1', run_id=self.run.name,
                    role=role, source_ns=s['source_ns'], parent_generation=s['parent_generation'],
                    query_file=stem + '.npy', query_shape=list(s['query'].shape),
                    query_dtype=s['query'].dtype.str, query_bytes=s['query'].nbytes,
                    query_retained_private_reference=True, query_copied=False,
                    query_sha256=hashlib.sha256(memoryview(s['query']).cast('B')).hexdigest(),
                    finite_query_points=s['finite_points'], T_odom_body=arrays[0].tolist(),
                    input_C_world_odom=arrays[1].tolist(), mount=arrays[2].tolist(),
                    prediction=arrays[3].tolist(), requested_seeds=[a.tolist() for a in arrays[4:]],
                    executed_seed_count=s['result']['completed_seeds'],
                    matcher_pending=None if s['pending'] is None else [s['pending'][0], s['pending'][1].tolist()],
                    matcher_confirmations=s['confirmations'], provenance=s['provenance'],
                    original_received_wall_ns=s['received_wall_ns'], original_result=s['result'],
                    policy=self.policy, policy_sha256=self.policy_sha, source_sha256=self.sources,
                    reference_file='REFERENCE.json', reference_sha256=self.reference_sha,
                    reference_world_sha256=self.world_sha, original_log='known_scene_matches.jsonl',
                    complete_processed_call=True, complete_raw_sensor_group=False,
                    quality_gates_changed=False, control_authority=False)
                start, cpu_start = time.monotonic_ns(), time.thread_time_ns()
                encoded = self._json(metadata)
                self.encode_wall_ns += time.monotonic_ns() - start
                self.encode_cpu_ns += time.thread_time_ns() - cpu_start
                if len(encoded) > METADATA_BYTES:
                    raise ValueError('capture_metadata_bound_no_truncation')
                (self.root / (stem + '.json')).write_bytes(encoded)
            paths = [self.root, *self.root.iterdir()]
            self.usage = dict(logical_bytes=sum(p.stat().st_size for p in paths if p.is_file()),
                allocated_bytes=sum(p.stat().st_blocks * 512 for p in paths))
            if max(self.usage.values()) + METADATA_BYTES > TOTAL_BYTES:
                raise ValueError('capture_filesystem_bound')
            self.files = {p.name: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                for p in paths if p.is_file()}
            self.complete = True
        except Exception as error:
            self.note_error('write:' + type(error).__name__ + ':' + str(error))
        finally:
            self.write_wall_ns = time.monotonic_ns() - wall
            self.write_cpu_ns = time.thread_time_ns() - cpu
            self.written.set()

    def close(self, worker_drained):
        with self.lock:
            self.closed = True
        self.finalize('worker_closed')
        finished = self.written.wait(timeout=2.)
        receipt = dict(schema='known_scene_matcher_query_capture_receipt/v1', enabled=True,
            source_window_ns=[START_NS, END_NS], selected=self.selected,
            first_two_bad_triggered=self.stop_reason == 'first_two_consecutive_bad',
            selection_complete={x['role'] for x in self.selected} == {'last_good','first_bad','second_bad'},
            missing_roles=sorted({'last_good','first_bad','second_bad'} - {x['role'] for x in self.selected}),
            stop_reason=self.stop_reason, worker_drained=bool(worker_drained),
            writer_operation_finished=finished, archive_files_complete=bool(finished and self.complete),
            file_complete=bool(finished and self.complete and worker_drained),
            writer_existing_error=self.writer.error, error=self.error, files=self.files,
            maximum_inputs=3, maximum_query_points=MAX_POINTS, maximum_query_bytes=QUERY_BYTES,
            maximum_total_bytes=TOTAL_BYTES, receipt_and_filesystem_reserve_bytes=RESERVE_BYTES,
            maximum_retained_query_bytes=3*QUERY_BYTES,
            reference_sha256=self.reference_sha, policy_sha256=self.policy_sha, source_sha256=self.sources,
            usage=self.usage, counts=self.counts,
            prepare_wall_ns=self.prepare_wall_ns, prepare_cpu_ns=self.prepare_cpu_ns,
            prepare_max_wall_ns=self.prepare_max_wall_ns, finish_wall_ns=self.finish_wall_ns,
            finish_cpu_ns=self.finish_cpu_ns, encode_wall_ns=self.encode_wall_ns,
            encode_cpu_ns=self.encode_cpu_ns, write_wall_ns=self.write_wall_ns, write_cpu_ns=self.write_cpu_ns,
            new_runtime_threads=0, existing_writer_lane=True, truncated=False,
            quality_gates_changed=False, control_authority=False)
        try:
            raw = self._json(receipt)
            if len(raw) > METADATA_BYTES:
                raise ValueError('capture_receipt_bound')
            (self.run / 'matcher_query_capture_receipt.json').write_bytes(raw)
        except Exception as error:
            receipt.update(receipt_write_error=type(error).__name__, file_complete=False)
        return receipt
