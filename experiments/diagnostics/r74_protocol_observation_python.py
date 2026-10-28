"""Default-off bounded application protocol observation; no control authority."""
import binascii
import copy
import json
import os
import time
from collections import deque

KEY = 'r74_protocol_observation_python'
SCHEMA = 'r74_bounded_protocol_observation_python/v1'
FIRST, TAIL, CHECKSUM_ATTEMPTS, STRING_BYTES, OUTPUT_BYTES = 32, 64, 512, 1024*1024, 512*1024
ATTRIBUTE = '_r74_protocol_observation_python'
TOPICS = {'bspline': '/demo/navigation/bspline',
          'trajectory_metadata': '/demo/navigation/trajectory_metadata',
          'reference_attempt': '/demo/navigation/reference_attempt'}


def enabled(profile):
    if type(profile) is not dict:
        raise ValueError('Protocol observation profile must be an object')
    if KEY not in profile:
        return False
    value = profile[KEY]
    if (type(value) is not dict or set(value) != {'enabled', 'schema'}
            or type(value['enabled']) is not bool or type(value['schema']) is not str
            or value['schema'] != SCHEMA):
        raise ValueError('Exact typed Python protocol observation selector required')
    if not value['enabled']:
        return False
    from experiments.candidates.minimal_simulation_baseline import enabled as minimal_enabled
    if (not minimal_enabled(profile)
            or profile.get('r85_protocol_intake_trace') != {'enabled': True}
            or type(profile['r85_protocol_intake_trace'].get('enabled')) is not bool):
        raise ValueError('Protocol observation requires minimal simulation and original intake observer')
    return True


def _scalar(value):
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        return value if value.bit_length() <= 128 else 'integer_outside128bit_observation_range'
    if type(value) is str:
        return value[:160]
    return 'unsupported_' + type(value).__name__[:32]


def small(value):
    # Unvalidated JSON fields never recurse or retain an input container.
    if type(value) in (list, tuple):
        if len(value) > 5:
            return 'unsupported_sequence_size'
        return [_scalar(x) for x in value]
    return _scalar(value)


def scope(node):
    a = getattr(node, 'trajectory_association', None)
    return {k: small(getattr(a, k, None)) for k in ('run_id', 'epoch', 'planner_process_id',
        'localization_generation', 'reference_stamp', 'reference_revision',
        'observed_reference_revision', 'rejected', 'last_rejected',
        'local_replan_request_highwater')}


def identity(value):
    if type(value) is not dict:
        return {}
    d = {k: small(value.get(k)) for k in ('run_id', 'reference_epoch', 'planner_process_id',
        'localization_generation', 'reference_stamp', 'committed_reference_revision')}
    a = value.get('attempt')
    if type(a) is dict:
        d['attempt'] = {k: small(a.get(k)) for k in ('id', 'origin',
            'local_replan_request_sequence', 'plan_success')}
    return d


class Observer:
    def __init__(self, node):
        self.run = str(node.run)
        self.run_id = node.run.name
        self.pid = os.getpid()
        self.first = []
        self.tail = deque(maxlen=TAIL)
        self.sticky = {}
        self.counts = {}
        self.sequence = self.errors = self.checksum_attempts = 0
        self.checksums_computed = self.oversize_strings = self.checksum_budget_omitted = 0
        self.last_error = None
        self.flushed = False
        self.endpoints = []
        self.last_failed = None
        self.cpp_first = []
        self.cpp_tail = deque(maxlen=32)
        self.cpp_received = self.cpp_oversize = self.cpp_errors = self.cpp_budget_omitted = 0
        try:
            for sub in node.subscriptions:
                if sub.topic_name not in TOPICS.values():
                    continue
                q = sub.qos_profile
                self.endpoints.append(dict(resolved_topic=sub.topic_name,
                    qos={k: small(getattr(q, k, None)) if k == 'depth'
                         else str(getattr(q, k, None))[:80]
                         for k in ('history', 'depth', 'reliability', 'durability')},
                    GID_observed=False))
        except Exception as error:
            self.error(error)

    def error(self, error):
        self.errors += 1
        self.last_error = (type(error).__name__ + ':' + str(error))[:160]

    def add(self, row):
        self.sequence += 1
        row['sequence'] = self.sequence
        self.counts[row['kind']] = self.counts.get(row['kind'], 0) + 1
        if len(self.first) < FIRST:
            self.first.append(row)
        self.tail.append(row)
        return row

    def fingerprint(self, msg):
        out = dict(data_utf8_bytes=None, crc32_ieee_hex=None, hash_is_cdr=False,
                   hash_is_authentication=False, checksum_state='unavailable')
        if self.checksum_attempts >= CHECKSUM_ATTEMPTS:
            self.checksum_budget_omitted += 1
            out['checksum_state'] = 'process_budget_omitted'
            return out
        self.checksum_attempts += 1  # Includes encoding/oversize attempts; never resets by epoch.
        raw = msg.data
        if type(raw) is not str:
            out['checksum_state'] = 'not_string'
            return out
        if len(raw) > STRING_BYTES:
            self.oversize_strings += 1
            out.update(checksum_state='oversize_characters', data_characters=len(raw))
            return out
        encoded = raw.encode('utf-8')
        out['data_utf8_bytes'] = len(encoded)
        if len(encoded) > STRING_BYTES:
            self.oversize_strings += 1
            out['checksum_state'] = 'oversize_utf8'
            return out
        out.update(crc32_ieee_hex=f'{binascii.crc32(encoded) & 0xffffffff:08x}',
                   checksum_state='computed')
        self.checksums_computed += 1
        return out


def setup(node):
    if enabled(getattr(node, 'profile', None)):
        observer = Observer(node)
        setattr(node, ATTRIBUTE, observer)
        # The one extra endpoint is diagnostic only. No real output is duplicated.
        try:
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            observer.cpp_subscription = node.create_subscription(String,
                '/demo/navigation/r74_protocol_observation_cpp',
                lambda msg: cpp_received(node, observer, msg), qos)
        except Exception as error:
            observer.error(error)


def cpp_received(node, observer, msg):
    observer.cpp_received += 1
    if observer.cpp_received > 128:
        observer.cpp_budget_omitted += 1
        return
    try:
        text = msg.data
        if type(text) is not str or len(text) > 4096:
            observer.cpp_oversize += 1
            return
        raw = text.encode('utf-8')
        if len(raw) > 4096:
            observer.cpp_oversize += 1
            return
        packet = json.loads(text)
        if type(packet) is not dict or packet.get('schema') != 'r74_protocol_observation_cpp/v1':
            raise ValueError('Unexpected C++ diagnostic envelope schema')
        # Retain a complete bounded diagnostic envelope, never a partial real String.
        compact = json.dumps(packet, allow_nan=False, ensure_ascii=False, separators=(',', ':')).encode()
        if len(compact) > 4096:
            observer.cpp_oversize += 1
            return
        row = dict(diagnostic_receive_sequence=observer.cpp_received,
            diagnostic_receive_monotonic_wall_ns=time.monotonic_ns(),
            diagnostic_receive_ros_clock_ns=int(node.get_clock().now().nanoseconds),
            diagnostic_packet_utf8_bytes=len(raw),
            diagnostic_packet=packet, diagnostic_packet_checksum_not_computed=True,
            proves_real_topic_delivery=False)
        if len(observer.cpp_first) < 16:
            observer.cpp_first.append(row)
        observer.cpp_tail.append(row)
    except Exception as error:
        observer.cpp_errors += 1
        observer.error(error)


def begin(node, topic, msg, original_context):
    observer = getattr(node, ATTRIBUTE, None)
    if observer is None:
        return None
    try:
        old = (original_context or {}).get('original_entry_receipt', {})
        row = dict(kind=topic, scope_before=scope(node), phase='entry',
            entry_state=small((original_context or {}).get('entry_state')),
            has_automatic_reference=hasattr(node, 'automatic_reference'),
            original_entry_receipt={k: small(old.get(k)) for k in (
                'entry_ros_clock_ns', 'entry_monotonic_wall_ns', 'entry_unix_wall_ns')},
            receipt_reused_from_original_intake=True, receipt_available=bool(old))
        if topic == 'bspline':
            row['partial_bspline_identity'] = dict(traj_id=small(getattr(msg, 'traj_id', None)),
                start_time=[small(getattr(getattr(msg, 'start_time', None), k, None))
                            for k in ('sec', 'nanosec')], order=small(getattr(msg, 'order', None)),
                pos_pts_count=len(getattr(msg, 'pos_pts', ())), knots_count=len(getattr(msg, 'knots', ())),
                full_payload_or_wire_identity=False)
        else:
            row['fingerprint'] = observer.fingerprint(msg)
        observer.add(row)
        return (observer, row)
    except Exception as error:
        observer.error(error)
        return None


def parsed(context, value):
    if context is not None:
        observer, row = context
        try:
            row['parsed_identity'] = identity(value)
        except Exception as error:
            observer.error(error)
    return value


def associated(node, context, matched):
    if context is None:
        return
    observer, row = context
    try:
        after = scope(node)
        row.update(phase='association_return', scope_after=after, matched=matched is not None)
        before_rejected = row['scope_before'].get('rejected')
        after_rejected = after.get('rejected')
        row['new_rejection_count'] = (after_rejected-before_rejected
            if type(before_rejected) is int and type(after_rejected) is int else None)
        if row['new_rejection_count']:
            observer.sticky.setdefault(row['kind']+':first_rejection', copy.deepcopy(row))
        ident = row.get('parsed_identity', {})
        attempt = ident.get('attempt', {})
        revision = ident.get('committed_reference_revision')
        if (row['kind'] == 'reference_attempt' and attempt.get('plan_success') is False
                and type(revision) is int and after.get('observed_reference_revision') == revision
                and row['new_rejection_count'] == 0):
            row['failed_attempt_revision_observed_without_new_rejection'] = True
            observer.last_failed = dict(identity=copy.deepcopy(ident), entry_sequence=row['sequence'])
            observer.sticky.setdefault('first_failed_attempt_revision_observed', copy.deepcopy(row))
    except Exception as error:
        observer.error(error)


def exception(context, error):
    if context is not None:
        observer, row = context
        try:
            row.update(phase='original_callback_exception', error=(type(error).__name__+':'+str(error))[:160])
            observer.sticky.setdefault(row['kind']+':first_exception', copy.deepcopy(row))
        except Exception as observer_error:
            observer.error(observer_error)


def retry_published(node, scope_tuple, revision, sequence, stamp, clock, pose_stamp, wall, ack_used):
    observer = getattr(node, ATTRIBUTE, None)
    if observer is None:
        return
    try:
        row = dict(kind='local_retry_publish_completed', scope_after=scope(node),
            planner_epoch=small(scope_tuple[0]), planner_process_id=small(scope_tuple[1]),
            localization_generation=small(scope_tuple[2]), committed_reference_revision=small(revision),
            sequence_in_protocol=small(sequence), reference_stamp=small(stamp),
            original_control_clock_ns=small(clock), source_pose_stamp_ns=small(pose_stamp),
            original_owner_last_reference_wall=wall, commit_ACK_fallback_used=ack_used,
            after_original_publish_note_and_evidence=True,
            original_note_highwater=small(getattr(node.trajectory_association, 'local_replan_request_highwater', None)))
        if observer.last_failed is not None:
            old = observer.last_failed
            ident = old['identity']
            row['same_scope_as_last_failed_observation'] = (
                ident.get('reference_epoch') == scope_tuple[0]
                and ident.get('planner_process_id') == scope_tuple[1]
                and ident.get('localization_generation') == scope_tuple[2]
                and ident.get('committed_reference_revision') == revision
                and ident.get('reference_stamp') == small(stamp))
            row['last_failed_entry_sequence'] = old['entry_sequence']
        observer.add(row)
        observer.sticky.setdefault('first_actual_local_retry', copy.deepcopy(row))
        if row.get('same_scope_as_last_failed_observation'):
            observer.sticky.setdefault('first_actual_retry_after_observed_failed_attempt', copy.deepcopy(row))
    except Exception as error:
        observer.error(error)


def flush_once(node):
    observer = getattr(node, ATTRIBUTE, None)
    if observer is None or observer.flushed:
        return
    observer.flushed = True
    try:
        rows = {r['sequence']: r for r in observer.first}
        rows.update({r['sequence']: r for r in observer.tail})
        cpp_rows = {r['diagnostic_receive_sequence']: r for r in observer.cpp_first}
        cpp_rows.update({r['diagnostic_receive_sequence']: r for r in observer.cpp_tail})
        document = dict(schema=SCHEMA, run_id=observer.run_id, own_pid=observer.pid,
            counts=observer.counts, total_events=observer.sequence,
            first_capacity=FIRST, tail_capacity=TAIL, retained=[rows[k] for k in sorted(rows)],
            sticky=observer.sticky, endpoints=observer.endpoints,
            cpp_diagnostic=dict(topic='/demo/navigation/r74_protocol_observation_cpp',
                qos='BEST_EFFORT KEEP_LAST1 VOLATILE', first_capacity=16, tail_capacity=32,
                maximum_packets_parsed=128, maximum_packet_utf8_bytes=4096,
                received=observer.cpp_received, oversize=observer.cpp_oversize,
                errors=observer.cpp_errors, budget_omitted=observer.cpp_budget_omitted,
                retained=[cpp_rows[k] for k in sorted(cpp_rows)],
                delivery_of_diagnostic_not_delivery_of_real_topics=True),
            checksum_attempt_limit=CHECKSUM_ATTEMPTS, maximum_string_bytes=STRING_BYTES,
            checksum_attempts=observer.checksum_attempts, checksums_computed=observer.checksums_computed,
            oversize_strings=observer.oversize_strings, checksum_budget_omitted=observer.checksum_budget_omitted,
            errors=observer.errors, last_error=observer.last_error,
            observation_only=True, motion_authority=False, navigation_PASS=False,
            checksum_not_authentication_or_complete_wire=True, source_timestamps_unchanged=True,
            maximum_output_bytes=OUTPUT_BYTES)
        raw = (json.dumps(document, allow_nan=False, ensure_ascii=False, separators=(',', ':'))+'\n').encode()
        if len(raw) > OUTPUT_BYTES:
            raise ValueError('Python protocol observation close output exceeds512KiB; no truncation')
        with (node.run/'r74_protocol_observation_python.json').open('xb') as stream:
            stream.write(raw)
    except Exception as error:
        observer.error(error)
        # One close-only visible failure notice; missing/partial output is not success.
        try:
            node.get_logger().warning(('R74 protocol observation capture FAILED; output missing or partial, '
                'not successful capture: ' + observer.last_error)[:512])
        except Exception:
            pass
        # Ordinary logger errors cannot prevent original destroy/cleanup.
