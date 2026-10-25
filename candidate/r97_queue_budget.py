"""Explicit diagnostic-only byte budget and bounded, control-owner observations.

No worker, lock, hot-path I/O, payload copies or control authority. Rate basis is
size-validated subscription receipt / original business callback invocation;
it is not the C++ publisher rate or a claim of successful path acceptance.
"""
from collections import deque
import os
import time

from diagnostic_observation import enabled as diagnostic_enabled

KEY = 'R97_DIAGNOSTIC_QUEUE_BYTES'
ORIGINAL_BYTES = 2 * 1024 * 1024
MAX_DIAGNOSTIC_BYTES = 4 * 1024 * 1024
ROW_CAPACITY = 64


def selection(profile):
    raw = os.environ.get(KEY)
    effective = ORIGINAL_BYTES
    diagnostic = diagnostic_enabled(profile)
    profile = profile if isinstance(profile, dict) else {}
    d = profile.get('diagnostic_observe_only', {})
    session = profile.get('r85_session', {})
    formal = profile.get('r85_formal_regression', {})
    formal_off = formal is None or (isinstance(formal, dict) and
                                   formal.get('enabled', False) is False)
    authorized = (diagnostic and formal_off and
        d.get('schema') == 'r85_existing_diagnostic_business_observation/v1' and
        d.get('simulation_only') is True and
        d.get('formal_acceptance_pass_allowed') is False and
        d.get('navigation_truth_injection') is False and
        isinstance(session, dict) and session.get('enabled') is True and
        session.get('simulation_only') is True and
        profile.get('actor_kind') == 'student' and
        profile.get('controller_kind') == 'student' and
        profile.get('navigation_ground_truth_used') is False)
    requested = None
    if raw is not None:
        # Canonical ASCII decimal only, bounded before conversion/allocation.
        if not 1 <= len(raw) <= 7 or any(c not in '0123456789' for c in raw):
            raise ValueError(KEY + ' requires canonical decimal bytes; effective cap remains 2097152')
        requested = int(raw)
        if str(requested) != raw or not ORIGINAL_BYTES <= requested <= MAX_DIAGNOSTIC_BYTES:
            raise ValueError(KEY + ' must be in [2097152,4194304]; effective cap remains 2097152')
        if requested > ORIGINAL_BYTES and not authorized:
            raise ValueError(KEY + ' increase requires diagnostic simulation scope and formal disabled; effective cap remains 2097152')
        effective = requested
    return dict(environment_key=KEY, environment_value=raw, requested_bytes=requested,
        effective_bytes=effective, original_shadow_bytes=ORIGINAL_BYTES,
        maximum_diagnostic_bytes=MAX_DIAGNOSTIC_BYTES,
        diagnostic_enabled=diagnostic, diagnostic_simulation_authorized=authorized,
        observation_enabled=raw is not None and authorized,
        max_events=32, max_event_bytes=1024*1024,
        control_or_navigation_PASS=False)


class Observation:
    def __init__(self, selected):
        self.selected = dict(selected)
        self.started_ns = time.monotonic_ns()
        self.rows = deque(maxlen=ROW_CAPACITY)
        self.first_row = None
        self.row_attempts = self.evicted_rows = 0
        self.first_shadow = self.last_shadow = None
        self.shadow_projected_exceedances = 0
        self.max_queue_count = self.max_queue_bytes = 0
        self.max_sampled_oldest_age_ns = self.max_consumed_age_ns = 0
        self.above_since_ns = None
        self.above_duration_ns = 0
        self.above_entries = self.natural_recoveries = self.natural_empty = 0
        self.natural_idle_empty = 0
        self.empty_pending_idle = False
        self.clear_terminations_above = self.clear_calls = 0
        self.clear_discarded_count = self.clear_discarded_bytes = 0
        self.streams = {name: dict(count=0, bytes=0, first_ns=None, last_ns=None,
            by_kind={k: dict(count=0, bytes=0, first_ns=None, last_ns=None)
                     for k in ('spline', 'metadata', 'other')})
            for name in ('received', 'enqueued', 'popleft', 'consumed',
                         'callback_completed', 'callback_raised', 'invalidated')}

    def _stream(self, name, event, now):
        stream = self.streams[name]
        stream['count'] += 1
        stream['bytes'] += event['size']
        if stream['first_ns'] is None:
            stream['first_ns'] = now
        stream['last_ns'] = now
        kind = event['kind'] if event['kind'] in ('spline', 'metadata') else 'other'
        stream['by_kind'][kind]['count'] += 1
        stream['by_kind'][kind]['bytes'] += event['size']
        if stream['by_kind'][kind]['first_ns'] is None:
            stream['by_kind'][kind]['first_ns'] = now
        stream['by_kind'][kind]['last_ns'] = now

    def _sample(self, events, queue_bytes, now):
        age = None if not events else max(0, now-events[0]['received_wall_ns'])
        self.max_queue_count = max(self.max_queue_count, len(events))
        self.max_queue_bytes = max(self.max_queue_bytes, queue_bytes)
        self.max_sampled_oldest_age_ns = max(self.max_sampled_oldest_age_ns, age or 0)
        return dict(queue_count=len(events), queue_bytes=queue_bytes,
            oldest_event_index=None if not events else events[0]['index'],
            oldest_received_wall_ns=None if not events else events[0]['received_wall_ns'],
            oldest_queue_age_ns=age, sampled_monotonic_wall_ns=now)

    def _row(self, boundary, sample, event=None, current=None, **fields):
        row = dict(boundary=boundary, **sample,
            event_index=None if event is None else event['index'],
            event_kind=None if event is None else event['kind'],
            event_size_bytes=None if event is None else event['size'],
            original_received_wall_ns=None if event is None else event['received_wall_ns'],
            original_received_ros_ns=None if event is None else event['received_ros_ns'],
            job_token=None if current is None else current['token'],
            owner=None if current is None else current['owner'], **fields)
        self.row_attempts += 1
        self.evicted_rows += int(len(self.rows) == ROW_CAPACITY)
        self.rows.append(row)
        if self.first_row is None:
            self.first_row = row
        return row

    def received(self, event, events, queue_bytes):
        now = event['received_wall_ns']
        self._stream('received', event, now)
        self._sample(events, queue_bytes, now)

    def projected(self, event, events, queue_bytes, current):
        now = time.monotonic_ns()
        projected = queue_bytes + event['size']
        if projected > ORIGINAL_BYTES:
            self.shadow_projected_exceedances += 1
            row = self._row('original_2mib_projected_exceeded',
                self._sample(events, queue_bytes, now), event, current,
                projected_queue_bytes=projected, projected_queue_count=len(events)+1,
                original_check_passed=False,
                diagnostic_continued=(projected <= self.selected['effective_bytes'] and len(events)<32),
                bytes_bound=projected > self.selected['effective_bytes'], count_bound=len(events)>=32)
            if self.first_shadow is None:
                self.first_shadow = row
            self.last_shadow = row
        elif len(events) >= 32:
            self._row('actual_queue_count_bound',
                self._sample(events, queue_bytes, now), event, current,
                projected_queue_bytes=projected, projected_queue_count=len(events)+1,
                count_bound=True, bytes_bound=projected > self.selected['effective_bytes'],
                diagnostic_continued=False)

    def appended(self, event, events, queue_bytes, current):
        now = time.monotonic_ns()
        self._stream('enqueued', event, now)
        self.empty_pending_idle = False
        sample = self._sample(events, queue_bytes, now)
        if queue_bytes > ORIGINAL_BYTES and self.above_since_ns is None:
            self.above_since_ns = now
            self.above_entries += 1
            self._row('actual_above_2mib_enter', sample, event, current)

    def popped(self, event, events, queue_bytes, current):
        now = time.monotonic_ns()
        self._stream('popleft', event, now)
        # This is the only recovery path; invalidate/close cannot call it.
        self.max_sampled_oldest_age_ns = max(self.max_sampled_oldest_age_ns,
            max(0, now-event['received_wall_ns']))
        sample = self._sample(events, queue_bytes, now)
        if self.above_since_ns is not None and queue_bytes <= ORIGINAL_BYTES:
            self.above_duration_ns += now-self.above_since_ns
            self.above_since_ns = None
            self.natural_recoveries += 1
            self._row('popleft_natural_below_or_equal_2mib', sample, event, current)
        if not events:
            self.natural_empty += 1
            self.empty_pending_idle = True
            self._row('popleft_natural_empty', sample, event, current)

    def consumed(self, event):
        now = time.monotonic_ns()
        self._stream('consumed', event, now)
        self.max_consumed_age_ns = max(self.max_consumed_age_ns,
            max(0, now-event['received_wall_ns']))

    def callback_finished(self, event, completed):
        self._stream('callback_completed' if completed else 'callback_raised',
                     event, time.monotonic_ns())

    def idle(self, events, queue_bytes, current, busy):
        if self.empty_pending_idle and not events and current is None and not busy:
            self.empty_pending_idle = False
            self.natural_idle_empty += 1
            self._row('natural_empty_and_no_active_job',
                      self._sample(events, queue_bytes, time.monotonic_ns()))

    def cleared(self, events, queue_bytes, current, reason):
        now = time.monotonic_ns()
        sample = self._sample(events, queue_bytes, now)
        self.empty_pending_idle = False
        self.clear_calls += 1
        for event in events:
            self._stream('invalidated', event, now)
        self.clear_discarded_count += len(events)
        self.clear_discarded_bytes += queue_bytes
        if self.above_since_ns is not None:
            self.above_duration_ns += now-self.above_since_ns
            self.above_since_ns = None
            self.clear_terminations_above += 1
        self._row('invalidate_clear_not_recovery', sample, current=current,
                  reason=str(reason)[:240])

    def snapshot(self, events, queue_bytes, node, fault):
        now = time.monotonic_ns()
        sample = self._sample(events, queue_bytes, now)
        elapsed = max(1, now-self.started_ns)
        streams = {name: dict(value,
            count_per_s_over_observation=value['count']*1e9/elapsed,
            bytes_per_s_over_observation=value['bytes']*1e9/elapsed)
            for name, value in self.streams.items()}
        route = getattr(node, 'continuous_route', None)
        return dict(schema='r97_bounded_diagnostic_queue_observation/v1',
            selection=dict(self.selected), started_monotonic_wall_ns=self.started_ns,
            finished_monotonic_wall_ns=now, rate_denominator_ns=elapsed,
            rate_basis='size-validated subscription receipt / queue append / popleft / business callback invocation and completion or raise / invalidation; not C++ publication or successful acceptance',
            streams=streams, current_queue=sample, max_queue_count=self.max_queue_count,
            max_queue_bytes=self.max_queue_bytes,
            max_sampled_oldest_queue_age_ns=self.max_sampled_oldest_age_ns,
            max_business_consumed_age_ns=self.max_consumed_age_ns,
            shadow_projected_exceedances=self.shadow_projected_exceedances,
            first_shadow_exceedance=self.first_shadow, last_shadow_exceedance=self.last_shadow,
            actual_above_entries=self.above_entries,
            actual_above_duration_ns=self.above_duration_ns +
                (0 if self.above_since_ns is None else now-self.above_since_ns),
            actual_above_still_active=self.above_since_ns is not None,
            popleft_natural_recoveries=self.natural_recoveries,
            popleft_natural_empty=self.natural_empty,
            natural_empty_and_no_active_job=self.natural_idle_empty,
            invalidate_clear_calls=self.clear_calls,
            invalidate_discarded_count=self.clear_discarded_count,
            invalidate_discarded_bytes=self.clear_discarded_bytes,
            invalidate_terminations_above_shadow=self.clear_terminations_above,
            first_transition=self.first_row, tail_rows=list(self.rows),
            row_capacity=ROW_CAPACITY, row_attempts=self.row_attempts,
            evicted_rows=self.evicted_rows, dropped_rows=0,
            maximum_retained_distinct_rows=ROW_CAPACITY+3,
            node_state=getattr(node, 'state', None),
            stop_reason=None if not (fault or getattr(node, 'message', None)) else
                str(fault or node.message)[:240],
            route_next_index=None if route is None else getattr(route, 'next_index', None),
            no_hot_path_disk_io=True, control_or_navigation_PASS=False)
