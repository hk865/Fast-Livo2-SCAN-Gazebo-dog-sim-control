"""Data-only, single-owner future-feedback coordination; default OFF.

Keep the original FIFO head inside the existing R86 Mailbox. This module owns
no messages, clocks, timers, Node/PID state, callback, thread, lock or I/O. A
ready result only delegates to original validators, never accepts feedback or
qualifies positive parking. Scope changes are observations, not feedback faults.
"""
from collections import deque
from dataclasses import dataclass, replace
import os

KEY = 'R99_CLOCK_FEEDBACK'
TIMEOUT_NS = 300_000_000
EVENT_CAPACITY = 128


def selected(profile, *, pose_timeout_s=None, cloud_timeout_s=None,
             feedback_timeout_s=None):
    value = os.environ.get(KEY, '0')
    if value not in ('0', '1'):
        raise ValueError(KEY + ' must be 0 or 1')
    if value == '0':
        return False
    from localization.sensors.feedback_mailbox import enabled as prepared_enabled
    from common.time.clock_groups import enabled as independent_clock_enabled
    if (type(profile) is not dict or not prepared_enabled(profile) or
            not independent_clock_enabled(profile)):
        raise ValueError('R99 requires original R86 and R74 independent simulation clock')
    cascade = profile.get('cascade', {})
    configured = (profile.get('pose_cloud_timeout_s'),
                  cascade.get('feedback_ttl_sim_and_wall_s') if type(cascade) is dict else None)
    runtime = (pose_timeout_s, cloud_timeout_s, feedback_timeout_s)
    if (any(type(v) is not float or v != .3 for v in configured) or
            any(v is not None and (type(v) is not float or v != .3) for v in runtime)):
        raise ValueError('R99 requires unchanged exact 0.3s pose/cloud/feedback timeouts')
    return True


def _ns(value):
    return type(value) is int and 0 <= value < 2**63


def _digest(value):
    if type(value) is bytes and len(value) == 32:
        return value.hex()
    if type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value):
        return value
    return None


def _token(value):
    return (type(value) is tuple and len(value) == 2 and
            type(value[0]) is str and len(value[0]) == 32 and
            all(c in '0123456789abcdef' for c in value[0]) and
            type(value[1]) is int and 1 <= value[1] < 2**64)


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str
    source_ns: int | None
    deadline_ns: int | None
    pending: bool


@dataclass(frozen=True)
class _Head:
    source_ns: int
    token: tuple
    original_body_arrival_ns: int
    deadline_ns: int
    original_scopes: tuple
    first_scope: str
    latest_scope: str
    owner_identity: str
    first_clock_ns: int
    first_wall_ns: int
    observed_future: bool = False
    phase: str = 'wait'
    attempts: int = 0
    scope_changes: int = 0


class Policy:
    """Exactly one control owner calls peek/clear/snapshot, never intake threads.

    owner_identity is a digest of the caller's actual control owner, independent
    of r86 control_scope's command/zero bit. Changing only scope never changes
    this owner. Caller must keep original Bundle.scopes and original R86 positive
    qualification formula; this policy grants no scope/parking qualification.
    """
    def __init__(self):
        self._head = None
        self._clock_highwater = self._wall_highwater = None
        self._finalized_source = -1
        self.fault = None
        self.events = deque(maxlen=EVENT_CAPACITY)
        self.first_wait = self.first_fault = self.last_final = None
        self.counts = dict(peeks=0, waits=0, retries=0, ready=0, finalizations=0,
                           scope_changes=0, faults=0, event_attempts=0, dropped_events=0)

    @property
    def pending(self):
        return self.fault is None and self._head is not None and self._head.phase == 'wait'

    @property
    def deadline_ns(self):
        return None if self._head is None else self._head.deadline_ns

    @property
    def source_ns(self):
        return None if self._head is None else self._head.source_ns

    def _event(self, event, clock_ns=None, wall_ns=None, **extra):
        h = self._head
        row = dict(event=event, clock_ns=clock_ns, wall_ns=wall_ns,
            source_ns=None if h is None else h.source_ns,
            token=None if h is None else h.token,
            original_body_arrival_ns=None if h is None else h.original_body_arrival_ns,
            deadline_ns=None if h is None else h.deadline_ns,
            original_scopes=None if h is None else h.original_scopes,
            first_scope=None if h is None else h.first_scope,
            latest_scope=None if h is None else h.latest_scope,
            owner_identity=None if h is None else h.owner_identity,
            attempts=0 if h is None else h.attempts,
            observed_future=False if h is None else h.observed_future, **extra)
        self.counts['event_attempts'] += 1
        self.counts['dropped_events'] += int(len(self.events) == EVENT_CAPACITY)
        self.events.append(row)
        return row

    def _decision(self, action, reason):
        return Decision(action, reason, self.source_ns, self.deadline_ns, self.pending)

    def _fail(self, reason, clock_ns=None, wall_ns=None):
        if self.fault is None:
            self.fault = 'r99_clock_feedback_' + reason
            self.counts['faults'] += 1
            self.first_fault = self._event('fault', clock_ns, wall_ns, reason=self.fault)
        return self._decision('fault', self.fault)

    def peek_decision(self, *, clock_ns, wall_ns, head_source_ns,
                      original_body_arrival_ns=None, scope_digest=None, token=None,
                      head_scopes=None, owner_identity=None, owner_valid=True,
                      observed_future=False):
        """No dequeue/receipt refresh. At most one immutable head is remembered.

        Call again before dispatching a previously held head if decode/work may
        have crossed its deadline. After real commit/rejection call clear with
        its final outcome. observed_future=True preserves a future suffix found
        at the original fixed take, even if clock caught up during prefix work.
        Use current clocks here; never replay an old sample below the watermark.
        Head=None is only an idle sample when no head is active.
        """
        self.counts['peeks'] += 1
        if self.fault is not None:
            return self._decision('fault', self.fault)
        if not _ns(clock_ns) or not _ns(wall_ns):
            return self._fail('invalid_clock_sample')
        if self._clock_highwater is not None and clock_ns < self._clock_highwater:
            return self._fail('ros_clock_rollback', clock_ns, wall_ns)
        if self._wall_highwater is not None and wall_ns < self._wall_highwater:
            return self._fail('monotonic_clock_rollback', clock_ns, wall_ns)
        self._clock_highwater, self._wall_highwater = clock_ns, wall_ns
        if type(observed_future) is not bool:
            return self._fail('invalid_observed_future_flag', clock_ns, wall_ns)
        if type(owner_valid) is not bool or not owner_valid:
            return self._fail('owner_invalid', clock_ns, wall_ns)
        if head_source_ns is None:
            if self._head is not None:
                return self._fail('pending_head_missing', clock_ns, wall_ns)
            return self._decision('idle', 'no_head')
        if (not _ns(head_source_ns) or not _ns(original_body_arrival_ns) or
                wall_ns < original_body_arrival_ns or not _token(token)):
            return self._fail('invalid_head_identity_or_receipt', clock_ns, wall_ns)
        scope, owner = _digest(scope_digest), _digest(owner_identity)
        if (scope is None or owner is None or type(head_scopes) is not tuple or
                len(head_scopes) != 3 or
                any(v is not None and _digest(v) is None for v in head_scopes)):
            return self._fail('invalid_scope_or_owner_digest', clock_ns, wall_ns)
        original_scopes = tuple(None if v is None else _digest(v) for v in head_scopes)
        h = self._head
        if h is not None:
            if (head_source_ns != h.source_ns or token != h.token or
                    original_body_arrival_ns != h.original_body_arrival_ns or
                    original_scopes != h.original_scopes):
                return self._fail('pending_head_identity_changed', clock_ns, wall_ns)
            if owner != h.owner_identity:
                return self._fail('owner_identity_changed', clock_ns, wall_ns)
            self.counts['retries'] += 1
            changes = int(scope != h.latest_scope)
            self.counts['scope_changes'] += changes
            self._head = replace(h, latest_scope=scope, attempts=h.attempts+1,
                                 scope_changes=h.scope_changes+changes)
            # Expiry precedes readiness, even if this sample catches up exactly.
            if wall_ns >= h.deadline_ns:
                return self._fail('original_arrival_deadline', clock_ns, wall_ns)
        else:
            if head_source_ns <= self._finalized_source:
                return self._fail('finalized_head_replayed', clock_ns, wall_ns)
            if clock_ns >= head_source_ns and not observed_future:
                # Normal nonfuture head retains original Teacher/PID behavior;
                # no new wall freshness gate is applied to that normal path.
                return Decision('ready', 'original_nonfuture_head', head_source_ns, None, False)
            self._head = _Head(head_source_ns, token, original_body_arrival_ns,
                original_body_arrival_ns+TIMEOUT_NS, original_scopes, scope, scope,
                owner, clock_ns, wall_ns, observed_future=observed_future, attempts=1)
            first = self._event('first_future_head', clock_ns, wall_ns)
            if self.first_wait is None:
                self.first_wait = first
            if wall_ns >= self._head.deadline_ns:
                return self._fail('original_arrival_deadline', clock_ns, wall_ns)
        if clock_ns >= head_source_ns:
            self._head = replace(self._head, phase='ready')
            self.counts['ready'] += 1
            self._event('ready_for_original_validators', clock_ns, wall_ns)
            return self._decision('ready', 'clock_caught_up_before_original_deadline')
        self.counts['waits'] += 1
        self._event('wait_original_fifo_head', clock_ns, wall_ns)
        return self._decision('wait', 'future_original_fifo_head')

    def clear(self, final_outcome):
        """Finalization only: never resets fault or either clock high-water.

        committed/commit_rejected mean original validators actually ran. Other
        outcomes cancel ownership without consuming or reclassifying the packet.
        The caller, not this helper, owns mailbox release and timer cancellation.
        """
        outcomes = ('committed', 'commit_rejected', 'mailbox_fault', 'owner_invalidated', 'shutdown')
        if final_outcome not in outcomes:
            return self._fail('invalid_final_outcome', self._clock_highwater, self._wall_highwater)
        if self._head is None:
            return None
        if self.pending and final_outcome in ('committed', 'commit_rejected'):
            return self._fail('future_head_finalized_without_ready', self._clock_highwater, self._wall_highwater)
        self.last_final = self._event('final_outcome', self._clock_highwater,
                                      self._wall_highwater, outcome=final_outcome)
        self._finalized_source = max(self._finalized_source, self._head.source_ns)
        self.counts['finalizations'] += 1
        self._head = None
        return None

    def snapshot(self):
        h = self._head
        return dict(schema='r99_bounded_clock_feedback_policy/v1', enabled=True,
            timeout_ns=TIMEOUT_NS, pending=self.pending, deadline_ns=self.deadline_ns,
            active_source_ns=self.source_ns, active_phase=None if h is None else h.phase,
            clock_highwater_ns=self._clock_highwater, wall_highwater_ns=self._wall_highwater,
            finalized_source_highwater_ns=self._finalized_source, fault=self.fault,
            counts=dict(self.counts), event_capacity=EVENT_CAPACITY,
            events=[dict(e) for e in self.events],
            first_wait=None if self.first_wait is None else dict(self.first_wait),
            first_fault=None if self.first_fault is None else dict(self.first_fault),
            last_final=None if self.last_final is None else dict(self.last_final),
            maximum_retained_distinct_events=EVENT_CAPACITY+3,
            original_scopes_preserved=True, source_receipts_refreshed=False,
            positive_parking_qualification_granted=False,
            only_original_validators_may_accept_feedback=True,
            no_messages_clocks_timers_threads_or_io_owned=True)
