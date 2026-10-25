"""Default-off bounded mission intake; waiting never grants source/Actor authority."""
from collections import deque
import hashlib
import json
import math
from typing import NamedTuple

KEY = 'r74_dynamic_causal_intake'
SCHEMA = 'r74_bounded_dynamic_mission_clock_intake/v1'
MAX_ITEMS = 8
MAX_UTF8_BYTES = 1024 * 1024
MAX_EPISODE_WALL_NS = 300_000_000


def enabled(profile):
    if KEY not in profile:
        return False
    option = profile[KEY]
    if (type(option) is not dict or set(option) != {'enabled', 'schema'}
            or type(option['enabled']) is not bool or option['enabled'] is not True
            or type(option['schema']) is not str or option['schema'] != SCHEMA):
        raise ValueError('r74_dynamic_causal_intake_typed_opt_in_required')
    from diagnostic_observation import strict_formal_enabled
    if strict_formal_enabled(profile):
        return True
    from minimal_simulation_baseline import enabled as minimal_enabled
    if not minimal_enabled(profile):
        raise ValueError('r74_dynamic_causal_intake_requires_minimal_simulation')
    diagnostic = profile.get('diagnostic_observe_only')
    formal = profile.get('r85_formal_regression')
    if (type(diagnostic) is not dict
            or any(diagnostic.get(k) is not v for k, v in (
                ('enabled', True), ('simulation_only', True),
                ('formal_acceptance_pass_allowed', False),
                ('recording_requested', False), ('navigation_truth_injection', False)))
            or type(formal) is not dict or set(formal) != {'enabled'}
            or type(formal['enabled']) is not bool or formal['enabled'] is not False
            or profile.get('navigation_ground_truth_used') is not False
            or profile.get('actor_kind') != 'student'):
        raise ValueError('r74_dynamic_causal_intake_diagnostic_scope_required')
    for section, leaf in (
            ('r85_completed_history_async_qualification', 'enabled'),
            ('r85_dynamic_event_contract', 'completed_same_geometry_preservation')):
        parent = profile.get(section)
        if (type(parent) is not dict or type(parent.get(leaf)) is not bool
                or parent[leaf] is not False):
            raise ValueError('r74_dynamic_causal_intake_requires_explicit_noncompleted_leaves')
    return True


def clock_targets(payload, run_id):
    """Only original pre-clock eligibility/schema/types; no reference permit."""
    try:
        state = json.loads(payload)
        if type(state) is not dict:
            return None
        reference = state.get('r85_reference', {})
        if (type(reference) is not dict or state.get('run_id') != run_id
                or state.get('stage') != 'navigating'
                or state.get('navigation_ground_truth_used') is not False
                or reference.get('reference_scope') != 'full_route'
                or state.get('schema') != 'r85_actual_single_task_mission_state/v1'
                or type(state.get('ros_sim_time_ns')) is not int
                or type(state.get('source_actual_pose_stamp_ns')) is not int
                or state['ros_sim_time_ns'] < 0 or state['source_actual_pose_stamp_ns'] < 0):
            return None
        return state['ros_sim_time_ns'], state['source_actual_pose_stamp_ns']
    except (ValueError, TypeError, RecursionError):
        return None


class Envelope(NamedTuple):
    data: str
    raw_sha256: str
    utf8_bytes: int
    received_wall: float
    arrival_clock_ns: int
    mission_clock_ns: int
    pose_clock_ns: int


def envelope(payload, received_wall, arrival_clock_ns, targets):
    raw = payload.encode('utf-8')
    return Envelope(payload, hashlib.sha256(raw).hexdigest(), len(raw), received_wall,
                    arrival_clock_ns, targets[0], targets[1])


def _metadata(value):
    return dict(raw_sha256=value.raw_sha256, utf8_bytes=value.utf8_bytes,
                first_received_wall=value.received_wall,
                arrival_clock_ns=value.arrival_clock_ns,
                mission_clock_ns=value.mission_clock_ns, pose_clock_ns=value.pose_clock_ns)


class BoundedMissionIntake:
    def __init__(self, run_id):
        self.run_id = run_id
        self._queue = deque()
        self._bytes = 0
        self._episode_wall = None
        self._last_clock = self._last_wall_ns = None
        self.failure = None
        self.counts = dict(enqueued=0, released=0, episodes_started=0, episodes_ended=0,
                           failures=0, max_items=0, max_utf8_bytes=0)
        self.first = self.last = None

    @property
    def pending(self):
        return bool(self._queue)

    def fail(self, reason, rejected=None):
        if self.failure is None:
            self.failure = reason
            self.counts['failures'] += 1
            if rejected is not None:
                self.last = dict(event='rejected', **_metadata(rejected), reason=reason)
        raise ValueError(self.failure)

    def check(self, clock, wall):
        if self.failure is not None:
            raise ValueError(self.failure)
        if (type(clock) is not int or clock < 0 or type(wall) not in (int, float)
                or not math.isfinite(wall) or wall < 0):
            self.fail('r74_dynamic_causal_intake_clock_sample_invalid')
        try:
            wall_ns = round(wall * 1e9)
        except (ValueError, OverflowError):
            self.fail('r74_dynamic_causal_intake_wall_sample_invalid')
        if self._last_clock is not None and clock < self._last_clock:
            self.fail('r74_dynamic_causal_intake_ROS_clock_rollback')
        if self._last_wall_ns is not None and wall_ns < self._last_wall_ns:
            self.fail('r74_dynamic_causal_intake_wall_clock_rollback')
        self._last_clock, self._last_wall_ns = clock, wall_ns
        if (self._queue and
                wall_ns - round(self._episode_wall * 1e9) >= MAX_EPISODE_WALL_NS):
            self.fail('r74_dynamic_causal_intake_pending_wall_budget_exhausted')

    def enqueue(self, item, clock, wall):
        self.check(clock, wall)
        if len(self._queue) >= MAX_ITEMS:
            self.fail('r74_dynamic_causal_intake_FIFO_item_budget_exhausted', item)
        if self._bytes + item.utf8_bytes > MAX_UTF8_BYTES:
            self.fail('r74_dynamic_causal_intake_UTF8_payload_budget_exhausted', item)
        if not self._queue:
            self._episode_wall = item.received_wall
            self.counts['episodes_started'] += 1
        self._queue.append(item)
        self._bytes += item.utf8_bytes
        self.counts['enqueued'] += 1
        self.counts['max_items'] = max(self.counts['max_items'], len(self._queue))
        self.counts['max_utf8_bytes'] = max(self.counts['max_utf8_bytes'], self._bytes)
        self.last = dict(event='enqueued', **_metadata(item))
        if self.first is None:
            self.first = dict(self.last)
        self.check(clock, wall)

    def pop_ready(self, clock, wall):
        # Expiry/rollback is tested before readiness, even if this sample caught up.
        self.check(clock, wall)
        if not self._queue:
            return None
        item = self._queue[0]
        if clock < item.mission_clock_ns or clock < item.pose_clock_ns:
            return None
        self._queue.popleft()
        self._bytes -= item.utf8_bytes
        self.counts['released'] += 1
        self.last = dict(event='released_to_original_validator', **_metadata(item),
                         ready_clock_ns=clock, deferred_validation_clock_ns=None)
        if not self._queue:
            self._episode_wall = None
            self.counts['episodes_ended'] += 1
        return item

    def note_validation(self, item, clock):
        # This is the original body's real clock sample, never a replacement stamp.
        self.last = dict(event='original_validator_clock', **_metadata(item),
                         deferred_validation_clock_ns=clock)

    def snapshot(self, node_failure=None):
        return dict(schema=SCHEMA, run_id=self.run_id, enabled=True,
                    max_items=MAX_ITEMS, max_UTF8_bytes=MAX_UTF8_BYTES,
                    max_episode_wall_ns=MAX_EPISODE_WALL_NS,
                    pending=self.pending, pending_UTF8_bytes=self._bytes,
                    pending_episode_first_received_wall=self._episode_wall,
                    pending_items=[_metadata(x) for x in self._queue],
                    counts=dict(self.counts), first=self.first, last=self.last,
                    intake_failure=self.failure, node_failure=node_failure,
                    unresolved_pending_not_accepted=self.pending,
                    payload_and_original_receipts_preserved=True,
                    queued_is_not_reference_acceptance=True, dynamic_PASS=False)
