"""Default-off, bounded whole-method Teacher timing; no control authority."""
from collections import deque
from functools import wraps
from pathlib import Path
import heapq
import json
import os
import threading
import time

KEY = 'r74_callback_cost_witness'
SCHEMA = 'r74_bounded_teacher_callback_cost_witness/v3'
ATTRIBUTE = '_r74_callback_cost_witness'
KINDS = ('publish_status', 'known_receive', 'control', 'admission', 'reference_attempt')
FIRST, LAST, TOP, OUTPUT_BYTES = 8, 64, 8, 1024*1024


def enabled(profile):
    if type(profile) is not dict:
        raise ValueError('Callback cost profile must be an object')
    if KEY not in profile:
        return False
    option = profile[KEY]
    if (type(option) is not dict or set(option) != {'enabled', 'schema'}
            or type(option['enabled']) is not bool
            or type(option['schema']) is not str or option['schema'] != SCHEMA):
        raise ValueError('Exact typed callback cost witness selector required')
    if option['enabled'] is False:
        return False
    from experiments.diagnostics.r74_corrected_delivery_witness import enabled as delivery_enabled
    from localization.sensors.metadata_qos import subscription_depth
    diagnostic = profile.get('diagnostic_observe_only')
    formal = profile.get('r85_formal_regression')
    if (not delivery_enabled(profile) or subscription_depth(profile) != 1
            or type(diagnostic) is not dict
            or any(diagnostic.get(k) is not v for k, v in (
                ('enabled', True), ('simulation_only', True),
                ('formal_acceptance_pass_allowed', False),
                ('recording_requested', False), ('navigation_truth_injection', False),
                ('full_evidence_after_endpoint', False)))
            or type(diagnostic.get('maximum_physics_rounds')) is not int
            or diagnostic['maximum_physics_rounds'] != 1
            or type(formal) is not dict or set(formal) != {'enabled'}
            or type(formal['enabled']) is not bool or formal['enabled'] is not False
            or profile.get('controller_kind') != 'student'
            or profile.get('actor_kind') != 'student'
            or profile.get('navigation_ground_truth_used') is not False):
        raise ValueError('Callback cost witness requires selected minimal delivery/depth1 diagnostic Student scope')
    return True


def _range_add(values, value):
    values['count'] += 1
    values['sum_ns'] += value
    values['min_ns'] = value if values['min_ns'] is None else min(values['min_ns'], value)
    values['max_ns'] = value if values['max_ns'] is None else max(values['max_ns'], value)


def _range():
    return dict(count=0, sum_ns=0, min_ns=None, max_ns=None)


class Witness:
    def __init__(self, run):
        self.run = Path(run)
        self.lock = threading.Lock()
        self.flushed = False
        self.errors = 0
        self.last_error = None
        self.kinds = {kind: dict(count=0, returned=0, raised=0,
            wall=_range(), thread_CPU=_range(), invalid_thread_CPU_count=0,
            first=[], last=deque(maxlen=LAST), top=[]) for kind in KINDS}

    def record(self, kind, begin, end, begin_cpu, end_cpu, begin_tid, end_tid, returned):
        if (kind not in KINDS or any(type(x) is not int or x < 0
                for x in (begin, end, begin_cpu, end_cpu, begin_tid, end_tid))
                or end < begin or type(returned) is not bool):
            raise ValueError('Invalid callback timing sample')
        wall = end-begin
        cpu_valid = begin_tid == end_tid and end_cpu >= begin_cpu
        cpu = end_cpu-begin_cpu if cpu_valid else None
        # The original method has returned/raised before this record-only lock.
        with self.lock:
            if self.flushed:
                raise ValueError('Callback sample after final flush')
            state = self.kinds[kind]
            state['count'] += 1
            state['returned' if returned else 'raised'] += 1
            _range_add(state['wall'], wall)
            if cpu_valid:
                _range_add(state['thread_CPU'], cpu)
            else:
                state['invalid_thread_CPU_count'] += 1
            row = dict(sequence=state['count'], entry_monotonic_wall_ns=begin,
                exit_monotonic_wall_ns=end, entry_thread_CPU_ns=begin_cpu,
                exit_thread_CPU_ns=end_cpu, entry_native_TID=begin_tid,
                exit_native_TID=end_tid, same_thread_CPU_valid=cpu_valid,
                wall_ns=wall, thread_CPU_ns=cpu, original_returned=returned)
            if len(state['first']) < FIRST:
                state['first'].append(row)
            else:
                state['last'].append(row)
            heapq.heappush(state['top'], (wall, row['sequence'], row))
            if len(state['top']) > TOP:
                heapq.heappop(state['top'])

    def error(self, error):
        with self.lock:
            self.errors += 1
            try:
                self.last_error = (type(error).__name__+': '+str(error))[:160]
            except Exception:
                self.last_error = type(error).__name__

    def flush_once(self):
        with self.lock:
            if self.flushed:
                return
            self.flushed = True
            kinds = {}
            for kind, state in self.kinds.items():
                first, last = list(state['first']), list(state['last'])
                top = [x[2] for x in sorted(state['top'], reverse=True)]
                retained = {row['sequence'] for row in first+last+top}
                kinds[kind] = dict(count=state['count'], returned=state['returned'],
                    raised=state['raised'], wall=dict(state['wall']),
                    thread_CPU=dict(state['thread_CPU']),
                    invalid_thread_CPU_count=state['invalid_thread_CPU_count'],
                    first=first, last=last, top_by_wall=top,
                    retained_unique_count=len(retained),
                    omitted_sample_rows=state['count']-len(retained))
            document = dict(schema=SCHEMA, run_id=self.run.name,
                kinds=kinds, errors=self.errors, last_error=self.last_error,
                first_capacity=FIRST, tail_capacity=LAST, top_capacity=TOP,
                maximum_output_bytes=OUTPUT_BYTES,
                scope='whole original method with small wrapper sampling overhead',
                sum_across_nested_kinds_is_valid=False,
                wall_minus_thread_CPU_is_single_cause_attribution=False,
                samples_not_executor_queue_or_DDS_arrival_times=True,
                aggregates_cover_successfully_recorded_pairs_only=True,
                retention_order='per-kind completion order; top rows may overlap first/tail',
                no_control_authority=True, formal_acceptance_pass=False)
        raw = (json.dumps(document, separators=(',', ':'), allow_nan=False)+'\n').encode()
        if len(raw) > OUTPUT_BYTES:
            raise ValueError('Callback timing output exceeds1MiB; no truncation')
        with (self.run/'r74_teacher_callback_cost_witness.json').open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())


def _error(witness, error):
    try:
        witness.error(error)
    except Exception:
        pass


def create(run, profile):
    if not enabled(profile):
        return None
    try:
        return Witness(run)
    except Exception:
        # An absent observer/output is missing evidence, never a motion veto.
        return None


def observe(kind):
    if kind not in KINDS:
        raise ValueError('Unselected callback timing kind')
    def decorate(operation):
        @wraps(operation)
        def wrapped(self, *args, **kwargs):
            witness = getattr(self, ATTRIBUTE, None)
            if witness is None:
                return operation(self, *args, **kwargs)
            try:
                begin_tid = threading.get_native_id()
                begin, begin_cpu = time.monotonic_ns(), time.thread_time_ns()
            except Exception as error:
                _error(witness, error)
                return operation(self, *args, **kwargs)
            returned = False
            try:
                result = operation(self, *args, **kwargs)
                returned = True
                return result
            finally:
                try:
                    end_cpu, end = time.thread_time_ns(), time.monotonic_ns()
                    end_tid = threading.get_native_id()
                    witness.record(kind, begin, end, begin_cpu, end_cpu,
                        begin_tid, end_tid, returned)
                except Exception as error:
                    _error(witness, error)
        return wrapped
    return decorate


def flush_once(witness):
    if witness is not None:
        try:
            witness.flush_once()
        except Exception as error:
            _error(witness, error)
