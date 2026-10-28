"""Default-off bounded corrected-bundle delivery witness; no control authority."""
from collections import OrderedDict, deque
from pathlib import Path
import json
import math
import os
import time

KEY = 'r74_corrected_delivery_witness'
SCHEMA = 'r74_bounded_corrected_delivery_pipeline_witness/v1'
ATTRIBUTE = '_r74_corrected_delivery_witness'
FIRST, LAST, PENDING = 64, 512, 24
STAGE_CAP, ROW_BYTES, OUTPUT_BYTES = 16, 3072, 2*1024*1024
PRODUCER_STAGES = frozenset(('geometry_bundle', 'corrected_odom', 'corrected_cloud',
    'message_sha', 'publish_metadata', 'publish_body', 'publish_lidar', 'publish_cloud',
    'bundle_history', 'publish_bundle'))
ROLES = ('body', 'cloud', 'metadata')
HASHES = ('corrected_body_message_sha256', 'corrected_lidar_message_sha256',
          'corrected_cloud_message_sha256')


def enabled(profile):
    if type(profile) is not dict:
        raise ValueError('Corrected delivery profile must be an object')
    if KEY not in profile:
        return False
    value = profile[KEY]
    if (type(value) is not dict or set(value) != {'enabled', 'schema'} or
            type(value['enabled']) is not bool or value['schema'] != SCHEMA):
        raise ValueError('Exact typed corrected delivery witness option required')
    if not value['enabled']:
        return False
    from experiments.candidates.minimal_simulation_baseline import enabled as minimal_enabled
    from experiments.diagnostics.r74_teacher_substage_timing import enabled as timing_enabled
    diagnostic = profile.get('diagnostic_observe_only')
    formal = profile.get('r85_formal_regression')
    if (not minimal_enabled(profile) or not timing_enabled(profile) or
            type(diagnostic) is not dict or any(diagnostic.get(k) is not v for k, v in (
                ('enabled', True), ('simulation_only', True), ('recording_requested', False),
                ('formal_acceptance_pass_allowed', False), ('navigation_truth_injection', False))) or
            type(diagnostic.get('maximum_physics_rounds')) is not int or diagnostic['maximum_physics_rounds'] != 1 or
            type(formal) is not dict or set(formal) != {'enabled'} or type(formal['enabled']) is not bool or formal['enabled'] is not False or
            profile.get('actor_kind') != 'student' or profile.get('controller_kind') != 'student' or
            profile.get('navigation_ground_truth_used') is not False):
        raise ValueError('Corrected delivery witness requires isolated minimal timing Student scope')
    return True


def _int(value):
    return value if type(value) is int and -(2**63) <= value < 2**63 else None


def _wall_ns(value):
    return _int(round(value*1e9)) if type(value) in (int, float) and math.isfinite(value) else None


def _metadata(value):
    parent = value.get('source_commit', {})
    return dict(generation=_int(value.get('generation')),
        producer_epoch=parent.get('producer_epoch') if type(parent.get('producer_epoch')) is str and len(parent['producer_epoch']) <= 32 else None,
        commit=_int(parent.get('commit')), parent_stamp_ns=_int(parent.get('stamp_ns')),
        metadata_message_sha256={k: value.get(k) if type(value.get(k)) is str and len(value[k]) == 64 else None for k in HASHES})


def _error(witness, error):
    if witness is not None:
        try:
            witness.errors += 1
            try:
                detail = str(error)
            except Exception:
                detail = 'unprintable observation exception'
            witness.last_error = (type(error).__name__+':'+detail)[:160]
        except Exception:
            pass


def _safe(witness, method, *args):
    if witness is None:
        return
    try:
        return getattr(witness, method)(*args)
    except Exception as error:
        _error(witness, error)


class Witness:
    def __init__(self, run, role):
        if role not in ('producer', 'consumer'):
            raise ValueError('Fixed producer/consumer witness role required')
        self.run, self.role = Path(run), role
        self.first, self.last, self.pending = [], deque(maxlen=LAST), OrderedDict()
        self.active = None
        self.total = self.attempts = self.errors = self.evicted = self.partial = 0
        self.stage_overflow = self.oversize = 0
        self.arrivals = {k: 0 for k in ROLES}
        self.last_error = None
        self.flushed = False

    def record(self, row):
        self.total += 1
        row['sequence'] = self.total
        if len(self.first) < FIRST:
            self.first.append(row)
        else:
            self.last.append(row)

    def stage(self, row, name, begin, end, cpu):
        stages = row.setdefault('stages', [])
        if len(stages) >= STAGE_CAP:
            self.stage_overflow += 1
            return
        stages.append(dict(kind=name, wall_start_ns=_int(begin), wall_end_ns=_int(end), thread_cpu_ns=_int(cpu)))

    def producer_begin(self, ns, clock, begin, begin_cpu, slot):
        self.attempts += 1
        if self.active is not None:
            self.partial += 1
        self.active = dict(source_ns=_int(ns), producer_receive_ros_clock_ns=_int(clock),
            producer_receive_header_age_ns=_int(clock-ns), bundle_entry_wall_ns=_int(begin),
            bundle_entry_thread_cpu_ns=_int(begin_cpu), stages=[],
            canonical_role_receipt_wall_ns={k: _wall_ns(slot[k][1]) for k in (
                'raw', 'adapted', 'registered', 'body', 'lidar', 'mapper',
                'source_identity', 'point_identity', 'odom_identity') if k in slot})

    def producer_stage(self, name, ns, begin, end, cpu):
        if self.active is not None and self.active['source_ns'] == ns and name in PRODUCER_STAGES:
            self.stage(self.active, name, begin, end, cpu)

    def producer_metadata(self, ns, metadata):
        if self.active is not None and self.active['source_ns'] == ns:
            self.active.update(_metadata(metadata))
            self.active['hash_semantics'] = 'existing_producer_computed_hashes'

    def producer_finish(self, ns, successful):
        row, self.active = self.active, None
        if row is None or row['source_ns'] != ns:
            self.partial += 1
            return
        if successful:
            row['app_publications_returned'] = True
            self.record(row)
        else:
            self.partial += 1

    def consumer_arrival(self, role, ns, metadata, entry_wall_ns):
        if role not in ROLES or _int(ns) is None:
            raise ValueError('Unexpected corrected input role or source stamp')
        self.arrivals[role] += 1
        wall = time.monotonic_ns()
        projected = _metadata(metadata) if role == 'metadata' else None
        if ns not in self.pending and len(self.pending) >= PENDING:
            self.pending.popitem(last=False)
            self.evicted += 1
        row = self.pending.setdefault(ns, dict(source_ns=ns, arrivals={}, stages=[]))
        previous = row['arrivals'].get(role)
        entry = wall if entry_wall_ns is None else entry_wall_ns
        row['arrivals'][role] = dict(first_callback_entry_wall_ns=entry if previous is None else previous['first_callback_entry_wall_ns'],
            last_callback_entry_wall_ns=entry, known_receive_wall_ns=wall,
            callbacks=1 if previous is None else previous['callbacks']+1)
        if role == 'metadata':
            row.update(projected)
            row['hash_semantics'] = 'metadata_expected_until_original_verify_returns'

    def consumer_stage(self, ns, name, begin, end, cpu, completed):
        row = self.pending.get(ns)
        if row is not None:
            if name == 'verify_corrected':
                row['exact_triple_ready_wall_ns'] = _int(begin)
                row['original_verify_returned'] = completed
            self.stage(row, name, begin, end, cpu)

    def consumer_finish(self, ns, outcome, pose, cloud):
        row = self.pending.pop(ns, None)
        if row is None:
            self.partial += 1
            return
        row.update(outcome=outcome, accepted_pose_stamp_ns=_int(pose), accepted_cloud_stamp_ns=_int(cloud))
        if all(role in row['arrivals'] for role in ROLES):
            self.record(row)
        else:
            self.partial += 1

    def flush(self):
        if self.flushed:
            return
        self.flushed = True
        def bounded(row):
            if len(json.dumps(row, separators=(',', ':'), allow_nan=False).encode()) <= ROW_BYTES:
                return row
            self.oversize += 1
            return dict(sequence=row.get('sequence'), source_ns=row.get('source_ns'), capture_oversize=True)
        first = [bounded(row) for row in self.first]
        last = [bounded(row) for row in self.last]
        pending = [bounded(row) for row in self.pending.values()]
        retained = [*first, *last]
        sources = [r['source_ns'] for r in retained if type(r.get('source_ns')) is int]
        walls = []
        for item in retained:
            walls.append(item.get('bundle_entry_wall_ns'))
            for span in item.get('stages', []):
                walls.extend((span.get('wall_start_ns'), span.get('wall_end_ns')))
            for arrival in item.get('arrivals', {}).values():
                walls.extend(arrival.get(k) for k in ('first_callback_entry_wall_ns', 'last_callback_entry_wall_ns', 'known_receive_wall_ns'))
        walls = [v for v in walls if type(v) is int]
        row = dict(schema=SCHEMA, role=self.role, run_id=self.run.name,
            first_capacity=FIRST, tail_capacity=LAST, private_pending_capacity=PENDING,
            stage_capacity=STAGE_CAP, maximum_row_bytes=ROW_BYTES, maximum_output_bytes=OUTPUT_BYTES,
            total_completed_frames=self.total, omitted_completed_frames=self.total-len(retained),
            producer_attempts=self.attempts,
            producer_rejected_or_unfinished_attempts=self.partial if self.role=='producer' else 0,
            consumer_partial_or_unmatched_finish=self.partial if self.role=='consumer' else 0,
            pending_evictions=self.evicted, pending_unfinished=len(pending),
            active_producer_unfinished=self.active is not None, callback_arrivals=self.arrivals,
            stage_overflow=self.stage_overflow, oversize_rows=self.oversize,
            errors=self.errors, last_error=self.last_error, first=first, last=last, pending=pending,
            retained_source_min_ns=min(sources) if sources else None,
            retained_source_max_ns=max(sources) if sources else None,
            retained_wall_min_ns=min(walls) if walls else None, retained_wall_max_ns=max(walls) if walls else None,
            callback_wall_is_DDS_receive_time=False, producer_ROS_clock_is_receive_entry_sample=True,
            no_hotpath_IO=True, no_control_authority=True, formal_acceptance_pass=False,
            observed_span_not_guaranteed_seconds=True, monotonic_samples_are_readonly=True)
        raw = (json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n').encode()
        if len(raw) > OUTPUT_BYTES:
            raise ValueError('Corrected delivery output cap exceeded; no output written')
        path = self.run/f'r74_corrected_{self.role}_delivery_witness.json'
        with path.open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())


def create(run, profile, role):
    return Witness(run, role) if enabled(profile) else None


def producer_begin(witness, ns, clock, begin, cpu, slot):
    _safe(witness, 'producer_begin', ns, clock, begin, cpu, slot)


def producer_stage(witness, name, ns, begin, end, cpu):
    _safe(witness, 'producer_stage', name, ns, begin, end, cpu)


def producer_metadata(node, ns, metadata):
    _safe(getattr(node, ATTRIBUTE, None), 'producer_metadata', ns, metadata)


def producer_finish(witness, ns, successful):
    _safe(witness, 'producer_finish', ns, successful)


def consumer_entry(node):
    witness = getattr(node, ATTRIBUTE, None)
    if witness is not None:
        try:
            return time.monotonic_ns()
        except Exception as error:
            _error(witness, error)
    return None


def consumer_arrival(node, role, ns, metadata, entry_wall_ns):
    _safe(getattr(node, ATTRIBUTE, None), 'consumer_arrival', role, ns, metadata, entry_wall_ns)


def consumer_call(node, ns, name, operation, *args):
    witness = getattr(node, ATTRIBUTE, None)
    if witness is None:
        return operation(*args)
    try:
        begin, cpu = time.monotonic_ns(), time.thread_time_ns()
    except Exception as error:
        _error(witness, error)
        return operation(*args)
    completed = False
    try:
        result = operation(*args)
        completed = True
        return result
    finally:
        try:
            end, end_cpu = time.monotonic_ns(), time.thread_time_ns()
            witness.consumer_stage(ns, name, begin, end, end_cpu-cpu, completed)
        except Exception as error:
            _error(witness, error)


def consumer_finish(node, ns, outcome):
    witness = getattr(node, ATTRIBUTE, None)
    if witness is not None:
        try:
            witness.consumer_finish(ns, outcome, node.pose_stamp, node.cloud_stamp)
        except Exception as error:
            _error(witness, error)


def flush(witness):
    _safe(witness, 'flush')
