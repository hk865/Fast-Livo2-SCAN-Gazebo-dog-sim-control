"""Default-off scalar observation of the two existing sensor hold predicates.

No fresh/readiness checker is called here. Snapshots are sequential scalar reads,
not an atomic sensor transaction, and never authorize a command or a timer reset.
"""
from collections import deque
import json
import math
import numbers
import os
from pathlib import Path
import time

KEY = 'r74_sensor_hold_witness'
SCHEMA = 'r74_bounded_sensor_hold_predicate_witness/v1'
FIRST = 64
LAST = 512
ATTRIBUTE = '_r74_sensor_hold_witness_memory'


def enabled(profile):
    if KEY not in profile:
        return False
    value = profile[KEY]
    if (type(value) is not dict or set(value) != {'enabled', 'schema'}
            or type(value['enabled']) is not bool
            or type(value['schema']) is not str or value['schema'] != SCHEMA):
        raise ValueError('Exact typed R74 sensor hold witness option required')
    if not value['enabled']:
        return False
    from minimal_simulation_baseline import enabled as minimal_enabled
    diagnostic = profile.get('diagnostic_observe_only')
    formal = profile.get('r85_formal_regression')
    if (not minimal_enabled(profile) or type(diagnostic) is not dict
            or any(diagnostic.get(k) is not v for k, v in (
                ('enabled', True), ('simulation_only', True),
                ('formal_acceptance_pass_allowed', False),
                ('recording_requested', False), ('navigation_truth_injection', False)))
            or type(formal) is not dict or set(formal) != {'enabled'}
            or type(formal['enabled']) is not bool or formal['enabled'] is not False
            or profile.get('actor_kind') != 'student'
            or profile.get('navigation_ground_truth_used') is not False):
        raise ValueError('Sensor hold witness requires isolated minimal diagnostic Student scope')
    return True


def _scalar(value):
    if value is None or type(value) is bool:
        return value
    if isinstance(value, numbers.Integral):
        return int(value) if int(value).bit_length() <= 128 else '<oversized integer>'
    if isinstance(value, numbers.Real):
        number = float(value)
        return number if math.isfinite(number) else repr(number)
    if type(value) is str:
        return value[:512]
    return '<unsupported:' + type(value).__name__[:64] + '>'


def _finite(value):
    try:
        return bool(math.isfinite(value))
    except (ValueError, TypeError, OverflowError):
        return False


def _fields(value, keys):
    return {key: _scalar(value.get(key)) for key in keys} if type(value) is dict else None


def _phase(node, now):
    since = node.stale_since
    elapsed = None if since is None else now - since
    return dict(state=_scalar(node.state), message=_scalar(node.message),
                stale_since=_scalar(since), elapsed_wall_from_stale_since_s=_scalar(elapsed))


class _CallbackMemory:
    def __init__(self):
        self.total = self.errors = 0
        self.first = []
        self.last = deque(maxlen=LAST)
        self.last_error = None
        self.by_kind = {kind: dict(total=0, accepted=0, rejected=0,
            last_actual_accepted=None, first_reject_after_last_accept=None)
            for kind in ('odom', 'cloud')}

    def snapshot(self):
        return dict(first_capacity=FIRST, tail_capacity=LAST, total=self.total,
            errors=self.errors, last_error=self.last_error,
            omitted_from_first_tail=self.total-len(self.first)-len(self.last),
            first=self.first, last=list(self.last), by_kind=self.by_kind,
            timestamps_resampled=False, downstream_odom_reject_reason_known=False,
            odom_exceptional_exits_not_captured=True,
            actual_acceptance_definition='accepted stamp strictly advanced; not a metadata/token receipt',
            sticky_definition='first rejection since the most recent observed actual stamp advance, or start')


class _Memory:
    def __init__(self):
        self.total = self.completed = self.errors = 0
        self.first = []
        self.last = deque(maxlen=LAST)
        self.first_failed = None
        self.last_error = None
        self.flushed = False
        self.callbacks = _CallbackMemory()


def _error(node, error):
    try:
        memory = getattr(node, ATTRIBUTE, None)
        if memory is not None:
            memory.errors += 1
            memory.last_error = (type(error).__name__ + ':' + str(error))[:512]
    except Exception:
        pass


def begin(node, gate, now, ros_clock_ns):
    try:
        if not enabled(node.profile):
            return None
        memory = getattr(node, ATTRIBUTE, None)
        if memory is None:
            memory = _Memory()
            setattr(node, ATTRIBUTE, memory)
        memory.total += 1
        raw = node.raw_imu
        row = dict(sequence=memory.total, gate=gate,
            predicate_now_monotonic_s=_scalar(now), predicate_ROS_clock_ns=_scalar(ros_clock_ns),
            witness_before_monotonic_ns=time.monotonic_ns(), before=_phase(node, now),
            pose=dict(stamp_ns=_scalar(node.pose_stamp), updated_monotonic_s=_scalar(node.pose_updated),
                timeout_s=_scalar(node.pose_timeout), receipt_isfinite=_finite(node.pose_updated)),
            cloud=dict(stamp_ns=_scalar(node.cloud_stamp), updated_monotonic_s=_scalar(node.cloud_updated),
                timeout_s=_scalar(node.cloud_timeout), receipt_isfinite=_finite(node.cloud_updated)),
            latest_raw_IMU=dict(stamp_s=_scalar(raw.stamp), updated_monotonic_s=_scalar(raw.updated),
                timeout_s=_scalar(raw.timeout), tilt_rad=_scalar(raw.tilt),
                stamp_isfinite=_finite(raw.stamp), updated_isfinite=_finite(raw.updated),
                timeout_isfinite=_finite(raw.timeout), tilt_isfinite=_finite(raw.tilt),
                last_rejected=_scalar(getattr(raw, 'last_rejected', None))),
            known_readiness=_fields(getattr(node, '_r73_known_readiness_observation', None),
                ('hard_rejected', 'reason', 'source_ns', 'generation', 'original_known_scene_ready',
                 'committed_same_frame', 'original_control_allowed', 'correction_stale')),
            known_inputs=_fields(getattr(node, 'known_scene_inputs', None),
                ('generation', 'source_ns', 'control_allowed', 'correction_stale', 'correction_transition')),
            known_status=_fields(getattr(node, 'known_scene_status', None),
                ('generation', 'source_ns', 'ready', 'initialized', 'correction_transition')),
            known_scan_epoch=_fields(getattr(node, 'known_scene_scan_epoch', None),
                ('generation', 'source_ns', 'ready', 'initialized', 'correction_transition')),
            known_scene_transition=_scalar(getattr(node, 'known_scene_transition', None)))
        if gate == 'ordinary_parking':
            row['parking_inputs'] = dict(
                paired_IMU_is_none=node.paired_imu is None,
                bridge_updated_monotonic_s=_scalar(node.bridge_updated),
                bridge_state=_scalar(node.bridge_safety.get('state')),
                evidence_error=_scalar(node.evidence.error),
                obstacle_hold=_scalar(node.obstacle_hold),
                tilt_hold=_scalar(node.tilt_hold),
                tilt_source=_scalar(node.tilt_source),
                tilt_clear_since=_scalar(node.tilt_clear_since),
                SLAM_rotation_row_major_3x3=[_scalar(node.rotation[i,j])
                    for i in range(3) for j in range(3)])
        return row
    except Exception as error:
        _error(node, error)
        return None


def parking_protection(node, row, original_return):
    """Observe the one original apply_tilt_guard return before cascade messages change."""
    if row is None:
        return
    try:
        row['parking_original_protection'] = dict(
            returned=_scalar(original_return),
            state=_scalar(node.state), message=_scalar(node.message),
            tilt_hold=_scalar(node.tilt_hold), tilt_source=_scalar(node.tilt_source))
    except Exception as error:
        _error(node, error)


def end(node, row, original_stale):
    if row is None:
        return
    try:
        memory = getattr(node, ATTRIBUTE)
        row.update(original_predicate_evaluated=original_stale is not None,
                   original_stale=_scalar(original_stale), after=_phase(node, row['predicate_now_monotonic_s']),
                   witness_after_monotonic_ns=time.monotonic_ns())
        memory.completed += 1
        if len(memory.first) < FIRST:
            memory.first.append(row)
        else:
            memory.last.append(row)
        if memory.first_failed is None and (row['before']['state'] == 'failed' or row['after']['state'] == 'failed'):
            memory.first_failed = row
    except Exception as error:
        _error(node, error)


def _callback_record(node, kind, row):
    # Only private projected scalar trees are retained; no ROS/message alias.
    memory = getattr(node, ATTRIBUTE, None)
    if memory is None:
        memory = _Memory()
        setattr(node, ATTRIBUTE, memory)
    callbacks = memory.callbacks
    callbacks.total += 1
    row['sequence'] = callbacks.total
    row['kind'] = kind
    if len(callbacks.first) < FIRST:
        callbacks.first.append(row)
    else:
        callbacks.last.append(row)
    state = callbacks.by_kind[kind]
    state['total'] += 1
    if row['actual_stamp_advanced']:
        state['accepted'] += 1
        state['last_actual_accepted'] = row
        state['first_reject_after_last_accept'] = None
    else:
        state['rejected'] += 1
        if state['first_reject_after_last_accept'] is None:
            state['first_reject_after_last_accept'] = row


def _callback_error(node, error):
    try:
        memory = getattr(node, ATTRIBUTE, None)
        if memory is not None:
            memory.callbacks.errors += 1
            memory.callbacks.last_error = (type(error).__name__ + ':' + str(error))[:512]
    except Exception:
        pass


def _callback_context(node):
    return dict(known_inputs=_fields(getattr(node, 'known_scene_inputs', None),
                    ('generation', 'source_ns')),
                pose_stamp_at_observation_ns=_scalar(node.pose_stamp),
                cloud_stamp_at_observation_ns=_scalar(node.cloud_stamp))


def _odom_inputs(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    v = message.twist.twist.linear
    w = message.twist.twist.angular
    return dict(frame_id=_scalar(message.header.frame_id),
        child_frame_id=_scalar(message.child_frame_id),
        position_xyz=[_scalar(x) for x in (p.x, p.y, p.z)],
        linear_velocity_xyz=[_scalar(x) for x in (v.x, v.y, v.z)],
        angular_velocity_xyz=[_scalar(x) for x in (w.x, w.y, w.z)],
        quaternion_xyzw=[_scalar(x) for x in (q.x, q.y, q.z, q.w)])


def odom_callback(node, stamp, clock, wall, before, after, *, early_future=False, message=None):
    try:
        if not enabled(node.profile):
            return
        advanced = after > before
        row = dict(producer_stamp_ns=_scalar(stamp),
            received_ROS_clock_ns=_scalar(clock), received_monotonic_wall_ns=_scalar(wall),
            original_header_age_ns=_scalar(clock-stamp),
            before_accepted_stamp_ns=_scalar(before), after_accepted_stamp_ns=_scalar(after),
            actual_stamp_advanced=bool(advanced), early_future_branch=bool(early_future),
            called_super=not early_future,
            original_odom_input_scalars=(_odom_inputs(message)
                if not early_future and message is not None else None),
            reason=('exact_early_future' if early_future else
                    'accepted_pose_stamp_advanced' if advanced else
                    'called_super_but_not_accepted_unknown'),
            **_callback_context(node))
        _callback_record(node, 'odom', row)
    except Exception as error:
        _callback_error(node, error)


def cloud_callback(node, original):
    try:
        if not enabled(node.profile):
            return
        before = original['previous_accepted_stamp_ns']
        after = original['accepted_stamp_ns']
        row = dict(original_callback_sequence=_scalar(original.get('sequence')),
            producer_stamp_ns=_scalar(original.get('producer_stamp_ns')),
            received_ROS_clock_ns=_scalar(original.get('received_ros_clock_ns')),
            received_monotonic_wall_s=_scalar(original.get('received_monotonic_wall')),
            original_header_age_ns=_scalar(original['received_ros_clock_ns']-original['producer_stamp_ns']),
            before_accepted_stamp_ns=_scalar(before), after_accepted_stamp_ns=_scalar(after),
            actual_stamp_advanced=bool(after > before),
            original_branch_accepted=_scalar(original.get('accepted')),
            reason=_scalar(original.get('reason')),
            decision_details=_fields(original.get('decision_details'),
                ('decision_ros_clock_ns', 'previous_accepted_stamp_ns', 'actual_header_age_ns',
                 'timeout_s', 'error', 'closest_pose_stamp_ns', 'pose_cloud_gap_ns',
                 'filtering_pose_stamp_ns', 'self_filtered_points', 'filtered_points',
                 'cloud_input_context_available')),
            finished_monotonic_wall_s=_scalar(original.get('finished_monotonic_wall')),
            finished_ROS_clock_ns=_scalar(original.get('finished_ros_clock_ns')),
            callback_duration_wall_s=_scalar(original.get('callback_duration_wall_s')),
            **_callback_context(node))
        _callback_record(node, 'cloud', row)
    except Exception as error:
        _callback_error(node, error)


def flush_once(node):
    try:
        if not enabled(node.profile):
            return
        memory = getattr(node, ATTRIBUTE, None)
        if memory is None:
            memory = _Memory()
            setattr(node, ATTRIBUTE, memory)
        if memory.flushed:
            return
        memory.flushed = True
        retained = {row['sequence'] for row in [*memory.first, *memory.last]}
        if memory.first_failed is not None:
            retained.add(memory.first_failed['sequence'])
        data = dict(schema=SCHEMA, first_capacity=FIRST, tail_capacity=LAST,
            total=memory.total, completed=memory.completed, errors=memory.errors,
            omitted=memory.completed-len(retained), unfinished_or_capture_error=memory.total-memory.completed,
            last_error=memory.last_error, first=memory.first, last=list(memory.last),
            first_observed_failed=memory.first_failed, hotpath_IO=False,
            sequential_scalar_snapshot_not_atomic=True, additional_fresh_or_readiness_calls=False,
            raw_IMU_semantics='latest accepted RawImuTilt state, not latest callback or causal pair',
            predicate_parameters_are_original=True, receipts_timers_control_state_changed=False,
            flush_method='exclusive_once_after_original_R47_flush', formal_acceptance_pass=False,
            callbacks=memory.callbacks.snapshot())
        raw = json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n'
        with (Path(node.run)/'r74_sensor_hold_witness.json').open('x', encoding='utf-8') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception as error:
        _error(node, error)
