"""Bounded scalar observations of the original canonical delivery path.

No observations grant publication, localization, commands or mission progress.
The runtime retains its original callbacks, gates, slots and publisher order.
The disk cap covers this one file; it is not a process RSS or timing guarantee.
"""
from collections import deque
from functools import wraps
from pathlib import Path
import json
import time

ROLES = ('raw', 'adapted', 'registered', 'body', 'lidar', 'mapper',
         'source_identity', 'point_identity', 'odom_identity')
STAGES = ('receive', 'publish_bundle', 'tick', 'decode', 'join_put',
          'observe_source', 'geometry_bundle', 'corrected_odom',
          'corrected_cloud', 'message_sha', 'bundle_history', 'publish_metadata',
          'publish_body', 'publish_lidar', 'publish_cloud', 'status_write')
LIMIT = 2 * 1024 * 1024
FILE_LIMIT = LIMIT - 4096  # Reserve growth of the existing exit receipt.
RESERVE = 32768
CHUNK_LIMIT = 256 * 1024
GAP_NS = 300000000

def encoded(row):
    # Identical encoding to the unchanged EvidenceWriter.append.
    return (json.dumps(row, allow_nan=False, ensure_ascii=False) + '\n').encode()

def rounded(n):
    return ((n + 4095) // 4096) * 4096

def diagnostic_error(trace):
    try:
        trace.errors += 1
    except Exception:
        pass

class DeliveryDiagnostics:
    def __init__(self, run, writer, *, wall_ns=time.monotonic_ns):
        self.path = Path(run) / 'R68_CANONICAL_DELIVERY_DIAGNOSTICS.jsonl'
        self.writer = writer
        self.wall_ns = wall_ns
        self.ring = deque(maxlen=512)
        self.roles = {r: dict(count=0, last_source_ns=None, last_received_wall_ns=None)
                      for r in ROLES}
        self.stages = {s: dict(count=0, maximum_duration_ns=0) for s in STAGES}
        self.gates = {s: 0 for s in ('original_initialized_or_header_veto',
                                    'original_wall_age_veto',
                                    'original_header_and_wall_gates_passed',
                                    'explicit_source_commit_veto')}
        self.evictions = 0
        self.events = 0
        self.chunks = 0
        self.bytes = 0
        self.errors = 0
        self.queue_drops = 0
        self.enqueued_lines = 0
        self.written_lines = 0
        self.written_bytes = 0
        self.write_errors = 0
        self.last_write_error = None
        self.original_writer_error_observed = False
        self.oversize = 0
        self.unwritten_chunks = 0
        self.in_gap = False
        self.last_snapshot_wall_ns = None
        self.maximum_captured_gap_ns = 0
        self.maximum_observed_gap_ns = 0
        self.gap_episodes = 0
        self.first_gap_clock_ns = None
        self.last_recovery_clock_ns = None
        self.recovery_saved = False
        self.terminal_saved = False
        self.terminal_attempts = 0
        self.diagnostic_wall_ns = 0
        self.closed = False

    def note(self, kind, **fields):
        if self.closed:
            return
        row = dict(kind=kind, wall_ns=self.wall_ns(), **fields)
        if len(encoded(row)) > 1024:
            self.oversize += 1
            return
        self.events += 1
        self.ring.append(row)

    def call(self, stage, source_ns, operation, *args, **kwargs):
        try:
            begin = self.wall_ns()
            begin_cpu = time.thread_time_ns()
            self.note('stage_begin', stage=stage, source_ns=int(source_ns))
        except Exception:
            diagnostic_error(self)
            begin = None
        result = None
        try:
            result = operation(*args, **kwargs)
            return result
        finally:
            # Diagnostics cannot replace the operation's result or exception.
            try:
                if begin is not None:
                    self.duration(stage, source_ns, begin, self.wall_ns(),
                                  thread_cpu_ns=time.thread_time_ns()-begin_cpu,
                                  boolean_result=result if type(result) is bool else None)
                if stage == 'decode' and isinstance(result, dict):
                    token = {k: result[k] for k in ('commit', 'packet', 'producer_epoch', 'producer_pid')
                             if k in result and (type(result[k]) is int or
                             isinstance(result[k], str) and len(result[k]) <= 64)}
                    self.note('identity_decoded',
                              source_ns=result.get('stamp_ns') if type(result.get('stamp_ns')) is int else None,
                              actual_metadata_token=token)
            except Exception:
                diagnostic_error(self)

    def duration(self, stage, source_ns, begin, end, **fields):
        if stage not in self.stages:
            raise ValueError('Unknown bounded diagnostic stage')
        row = self.stages[stage]
        row['count'] += 1
        row['maximum_duration_ns'] = max(row['maximum_duration_ns'], end - begin)
        self.note(stage, source_ns=int(source_ns), started_wall_ns=begin,
                  duration_ns=end - begin, **fields)

    def receive_started(self, role, source_ns):
        row = self.roles[role]
        row.update(count=row['count'] + 1, last_source_ns=int(source_ns),
                   last_received_wall_ns=self.wall_ns())

    def evict(self, slots):
        # Invoke the same original OrderedDict operation exactly once.
        key, slot = slots.popitem(last=False)
        try:
            self.evictions += 1
            self.note('canonical_slot_eviction', source_ns=int(key),
                      present_role_mask=sum(1 << i for i, r in enumerate(ROLES) if r in slot),
                      reason='unchanged_canonical_12_slot_cap')
        except Exception:
            diagnostic_error(self)
        return key, slot

    def gate(self, source_ns, clock_ns, decision):
        try:
            key = decision if decision in self.gates else 'explicit_source_commit_veto'
            self.gates[key] += 1
            self.note('original_publication_gate', source_ns=int(source_ns),
                      clock_ns=int(clock_ns), header_age_ns=int(clock_ns-source_ns),
                      ages_sampled_after_original_gate=True, decision=decision[:256])
        except Exception:
            self.errors += 1

    def receive_clock(self, role, source_ns, clock_ns):
        try:
            self.note('original_receive_clock', role=role, source_ns=int(source_ns),
                      clock_ns=int(clock_ns), header_age_ns=int(clock_ns-source_ns))
        except Exception:
            self.errors += 1

    def cleanup(self, source_ns, slots):
        try:
            self.note('canonical_success_cleanup', source_ns=int(source_ns),
                      source_stamps=[int(k) for k in slots if k<=source_ns])
        except Exception:
            self.errors += 1

    def snapshot(self, runtime, clock_ns, reason):
        # Scalar masks and arrival times only. No message or point payload copy.
        slots = [dict(source_ns=int(ns), present_role_mask=sum(
                 1 << i for i, r in enumerate(ROLES) if r in slot),
                 received_wall_ns={r: round(pair[1]*1e9) for r, pair in slot.items()})
                 for ns, slot in list(runtime.slots.items())]
        row = dict(schema='R68_canonical_delivery_scalar_window/v1', reason=reason,
                   clock_ns=int(clock_ns), wall_ns=self.wall_ns(),
                   last_published_source_ns=int(runtime.last_published),
                   last_published_header_age_ns=int(clock_ns-runtime.last_published),
                   initialized=bool(runtime.initialized), canonical_slot_cap=12,
                   canonical_slots=slots, roles=self.roles, stages=self.stages,
                   publication_gates=self.gates, canonical_evictions=self.evictions,
                   events_seen=self.events, retained_events=list(self.ring),
                   events_not_retained=max(0, self.events-len(self.ring)),
                   retained_first_wall_ns=None if not self.ring else self.ring[0]['wall_ns'],
                   retained_last_wall_ns=None if not self.ring else self.ring[-1]['wall_ns'],
                   gap_episodes=self.gap_episodes, first_gap_notice_clock_ns=self.first_gap_clock_ns,
                   maximum_observed_publication_age_ns=self.maximum_observed_gap_ns,
                   last_recovery_notice_clock_ns=self.last_recovery_clock_ns,
                   exact_gap_onset_unverified=True,
                   diagnostic_errors=self.errors, oversize_events=self.oversize,
                   diagnostic_only=True, decisions_changed=False,
                   payloads_retained=False, DDS_drop_counts_observed=False,
                   separate_matcher_join_slots_observed=False)
        size = len(encoded(row))
        terminal = reason == 'original_mission_terminal'
        maximum_chunks = 8 if terminal else 7
        byte_ceiling = FILE_LIMIT-RESERVE if terminal else FILE_LIMIT-RESERVE-CHUNK_LIMIT
        if (self.chunks >= maximum_chunks or size > CHUNK_LIMIT or
                rounded(self.bytes+size) > byte_ceiling):
            self.unwritten_chunks += 1
            return False
        if not self.enqueue(row):
            self.unwritten_chunks += 1
            return False
        self.bytes += size
        self.chunks += 1
        return True

    def after(self, runtime):
        if self.closed or not runtime.initialized or runtime.last_published <= 0:
            return
        clock_ns = int(runtime.get_clock().now().nanoseconds)
        gap = clock_ns-runtime.last_published >= GAP_NS
        age = clock_ns-runtime.last_published
        self.maximum_observed_gap_ns = max(self.maximum_observed_gap_ns, age)
        now = self.wall_ns()
        if gap and not self.in_gap:
            self.gap_episodes += 1
            if self.first_gap_clock_ns is None:
                self.first_gap_clock_ns = clock_ns
        if gap and (self.last_snapshot_wall_ns is None or
                    (age >= self.maximum_captured_gap_ns+GAP_NS and
                     now-self.last_snapshot_wall_ns >= 1000000000)):
            saved = self.snapshot(runtime, clock_ns, 'original_300ms_publication_age')
            self.last_snapshot_wall_ns = now
            if saved:
                self.maximum_captured_gap_ns = max(self.maximum_captured_gap_ns, age)
        elif self.in_gap and not gap:
            self.last_recovery_clock_ns = clock_ns
            if not self.recovery_saved:
                saved = self.snapshot(runtime, clock_ns, 'publication_recovered')
                self.last_snapshot_wall_ns = now
                self.recovery_saved = saved
        self.in_gap = gap

    def mission(self, runtime, data, clock_ns, operation):
        result = operation(data, clock_ns)
        try:
            # Reuse the existing subscription; no authority comes from this parse.
            stage = json.loads(data).get('stage')
            if result is True and stage in ('failed', 'completed') and not self.terminal_saved:
                self.terminal_attempts += 1
                self.terminal_saved = self.snapshot(runtime, clock_ns, 'original_mission_terminal')
        except Exception:
            diagnostic_error(self)
        return result

    def close(self):
        try:
            self._close()
        except Exception:
            diagnostic_error(self)
            self.closed = True

    def enqueue(self, row):
        """Use the existing worker queue, with isolated diagnostic write errors.

        Calling EvidenceWriter.enqueue on a full queue poisons its original
        error state. A diagnostic put_nowait can drop without changing it.
        The immutable operation catches its own disk error in the worker.
        """
        try:
            if self.writer.error:
                self.original_writer_error_observed = True
                self.queue_drops += 1
                return False
            line = encoded(row).decode('utf-8')
            def write():
                try:
                    with self.path.open('a') as stream:
                        stream.write(line)
                    self.written_lines += 1
                    self.written_bytes += len(line.encode('utf-8'))
                except Exception as error:
                    self.write_errors += 1
                    self.last_write_error = type(error).__name__+':'+str(error)[:128]
            write.diagnostic_line = line  # Immutable finite CPU queue witness.
            self.writer.queue.put_nowait(write)
            self.enqueued_lines += 1
            return True
        except Exception:
            diagnostic_error(self)
            self.queue_drops += 1
            return False

    def receipt(self):
        """Read after the unchanged original writer.close has drained."""
        try:
            return dict(schema='R68_canonical_delivery_drain_receipt/v1',
                        enqueued_lines=self.enqueued_lines, written_lines=self.written_lines,
                        written_bytes=self.written_bytes, queue_drops=self.queue_drops,
                        write_errors=self.write_errors, last_write_error=self.last_write_error,
                        diagnostic_errors=self.errors, terminal_attempts=self.terminal_attempts,
                        terminal_snapshot_enqueued=self.terminal_saved,
                        original_writer_error_observed=self.original_writer_error_observed,
                        file_cap_B=FILE_LIMIT, total_file_and_exit_growth_cap_B=LIMIT,
                        closed=self.closed, diagnostic_only=True, decisions_changed=False)
        except Exception:
            return dict(diagnostic_receipt_available=False, diagnostic_only=True)

    def _close(self):
        if self.closed:
            return
        row = dict(schema='R68_canonical_delivery_scalar_close/v1', final=True,
                   roles=self.roles, stages=self.stages, gates=self.gates,
                   evictions=self.evictions, events_seen=self.events,
                   snapshots=self.chunks, unwritten_snapshots=self.unwritten_chunks,
                   errors=self.errors, oversize_events=self.oversize,
                   measured_span_finally_diagnostic_wall_ns=self.diagnostic_wall_ns,
                   other_instrumentation_overhead_not_included=True,
                   gap_episodes=self.gap_episodes, first_gap_notice_clock_ns=self.first_gap_clock_ns,
                   maximum_observed_publication_age_ns=self.maximum_observed_gap_ns,
                   last_recovery_notice_clock_ns=self.last_recovery_clock_ns,
                   terminal_snapshot_enqueued=self.terminal_saved,
                   terminal_snapshot_attempts=self.terminal_attempts,
                   previous_bytes=self.bytes, maximum_file_allocated_and_logical_B=FILE_LIMIT,
                   scalar_ring_maximum_events=512, not_a_process_RSS_limit=True,
                   timing_equivalence_unverified=True, decisions_changed=False)
        size = len(encoded(row))
        if rounded(self.bytes+size) > FILE_LIMIT:
            raise RuntimeError('Diagnostic final reserve exceeded')
        if self.enqueue(row):
            self.bytes += size
        self.closed = True

def delivery_span(stage):
    def decorate(operation):
        @wraps(operation)
        def wrapped(runtime, *args, **kwargs):
            trace = None
            try:
                trace = runtime.delivery_diagnostics
                begin = trace.wall_ns()
                begin_cpu = time.thread_time_ns()
            except Exception:
                diagnostic_error(trace)
                return operation(runtime, *args, **kwargs)
            source_ns = -1
            try:
                trace.note('callback_begin', stage=stage, started_wall_ns=begin)
                if stage == 'receive':
                    role, msg = args
                    source_ns = (msg['stamp_ns'] if isinstance(msg, dict) else
                                 int(msg.header.stamp.sec)*10**9 + int(msg.header.stamp.nanosec))
                    trace.receive_started(role, source_ns)
                    token = {k: msg[k] for k in ('commit', 'packet', 'producer_epoch', 'producer_pid')
                             if isinstance(msg, dict) and k in msg and
                             (type(msg[k]) is int or isinstance(msg[k], str) and len(msg[k]) <= 64)}
                    trace.note('callback_input', role=role, source_ns=int(source_ns),
                               callback_entry_wall_ns=begin, actual_metadata_token=token,
                               DDS_receive_time_observed=False)
                elif stage == 'publish_bundle':
                    source_ns = args[0]
            except Exception:
                diagnostic_error(trace)
            try:
                return operation(runtime, *args, **kwargs)
            finally:
                try:
                    diagnostic_begin = trace.wall_ns()
                    trace.duration(stage, source_ns, begin, trace.wall_ns(),
                                   thread_cpu_ns=time.thread_time_ns()-begin_cpu)
                    trace.after(runtime)
                    trace.diagnostic_wall_ns += trace.wall_ns()-diagnostic_begin
                except Exception:
                    diagnostic_error(trace)
        return wrapped
    return decorate
