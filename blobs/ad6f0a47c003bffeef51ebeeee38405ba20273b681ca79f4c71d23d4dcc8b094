"""Default-off observation of real protocol callbacks; no motion authority.

UTF-8 hashes below identify the received String data, not DDS/CDR bytes.  Spline
identity is deliberately partial; the unchanged Association still compares every
coefficient.  This module neither buffers messages nor changes source/receipt
timestamps.  Fixed-size status is updated on each entry; bounded event history
records semantic changes only.  Observer failures are exposed, never substituted
for a control, parsing, association or physical-admission decision.
"""
import hashlib
import math
import time


LEAF = 'r85_protocol_intake_trace'
TOPICS = ('bspline', 'trajectory_metadata', 'reference_attempt')
MAX_EVENTS = 4096


def enabled(profile):
    config = profile.get(LEAF, {}) if isinstance(profile, dict) else {}
    return (isinstance(config, dict) and type(config.get('enabled')) is bool
            and config['enabled'] is True)


def _small(value):
    if value is None or type(value) in (bool, int):
        return value
    if type(value) is float:
        return value if math.isfinite(value) else str(value)
    if isinstance(value, str):
        return value[:512]
    if isinstance(value, (list, tuple)) and len(value) <= 5:
        return [_small(x) for x in value]
    return None


def _fields(value, names):
    return {name: _small(value.get(name)) for name in names} if isinstance(value, dict) else {}


def _identity(value):
    result = _fields(value, ('schema', 'run_id', 'reference_epoch', 'planner_process_id',
        'localization_generation', 'committed_reference_revision', 'reference_stamp'))
    if isinstance(value, dict):
        result['attempt'] = _fields(value.get('attempt'),
            ('id', 'origin', 'local_replan_request_sequence', 'plan_success'))
        result['trajectory'] = _fields(value.get('trajectory'), ('traj_id', 'start_time', 'order'))
    return result


def _association(node):
    association = getattr(node, 'trajectory_association', None)
    result = {name: _small(getattr(association, name, None)) for name in (
        'run_id', 'epoch', 'planner_process_id', 'localization_generation',
        'reference_stamp', 'reference_revision', 'observed_reference_revision',
        'accepted_highwater_id', 'accepted_attempt_highwater_id', 'rejected', 'last_rejected')}
    result['pending'] = {name: len(getattr(association, name, ()))
        for name in ('splines', 'metadata', 'attempts')}
    return result


def _clock(node):
    return dict(observed_ros_clock_ns=int(node.get_clock().now().nanoseconds),
        observed_monotonic_wall_ns=time.monotonic_ns(), observed_unix_wall_ns=time.time_ns())


class _Observer:
    def __init__(self, node):
        self.node = node
        self.counts = {topic: {} for topic in TOPICS}
        self.last = {}
        self.signatures = {}
        self.current = None
        self.events_enqueued = self.events_suppressed = self.observer_errors = 0
        self.last_observer_error = None

    def count(self, topic, name):
        counts = self.counts[topic]
        counts[name] = counts.get(name, 0) + 1

    def emit(self, topic, phase, semantic, details):
        # Payload/trajectory identities and advancing clocks are evidence fields,
        # not change keys. Repeated identical states do not create hot logs.
        key = (topic, phase)
        if self.signatures.get(key) == semantic:
            self.events_suppressed += 1
            return
        self.signatures[key] = semantic
        if self.events_enqueued >= MAX_EVENTS:
            self.events_suppressed += 1
            return
        row = dict(schema='r85_protocol_intake_change/v1', topic=topic, phase=phase,
            run_id=getattr(getattr(self.node, 'run', None), 'name', None),
            request_id=_small(getattr(self.node, 'request_id', None)),
            navigation_PASS=False, motion_authority=False,
            event_index=self.events_enqueued + 1, **_clock(self.node), **details)
        self.node.evidence.append(self.node.run/'r85_protocol_intake_changes.jsonl', row)
        self.events_enqueued += 1

    def error(self, error):
        self.observer_errors += 1
        self.last_observer_error = (type(error).__name__ + ': ' + str(error))[:512]


def _observer(node):
    if not enabled(getattr(node, 'profile', None)):
        return None
    observer = getattr(node, '_r85_protocol_intake_observer', None)
    if observer is None:
        observer = _Observer(node)
        node._r85_protocol_intake_observer = observer
    return observer


def begin(node, topic, msg):
    observer = _observer(node)
    if observer is None:
        return None
    try:
        if topic not in TOPICS:
            raise ValueError('unknown_protocol_topic')
        observer.count(topic, 'entry')
        state = _small(getattr(node, 'state', None))
        receipt = _clock(node)
        # These are actual callback-entry clocks, not new source headers and not
        # newly received stamps to be reused by a deferred path transaction.
        receipt = {name.replace('observed_', 'entry_', 1): value for name, value in receipt.items()}
        if topic == 'bspline':
            identity = dict(traj_id=_small(getattr(msg, 'traj_id', None)),
                start_time=[_small(getattr(getattr(msg, 'start_time', None), name, None))
                            for name in ('sec', 'nanosec')],
                order=_small(getattr(msg, 'order', None)),
                original_pos_pts_count=len(getattr(msg, 'pos_pts', ())),
                original_knots_count=len(getattr(msg, 'knots', ())),
                identity_is_partial_not_payload_hash=True)
        else:
            raw = msg.data
            identity = dict(string_utf8_sha256=hashlib.sha256(raw.encode('utf-8')).hexdigest(),
                string_characters=len(raw), hash_is_cdr=False)
        context = dict(observer=observer, topic=topic, phase='entry', entry_state=state,
            original_entry_receipt=receipt, message_identity=identity,
            association_before=_association(node), outer_terminal_seen=False)
        observer.current = context
        observer.last[topic] = {key: value for key, value in context.items() if key != 'observer'}
        scope = context['association_before']
        observer.emit(topic, 'entry', (state, scope['epoch'], scope['planner_process_id'],
            scope['localization_generation']), dict(entry_state=state,
            original_entry_receipt=receipt, message_identity=identity))
        return context
    except Exception as error:
        observer.error(error)
        return None


def ignored(context, reason):
    if context is None:
        return
    observer = context['observer']
    try:
        observer.count(context['topic'], 'ignored')
        context.update(phase='ignored', original_filter_reason=reason)
        observer.last[context['topic']] = {key: value for key, value in context.items() if key != 'observer'}
        observer.emit(context['topic'], 'ignored', (context['entry_state'], reason),
            dict(reason=reason, original_entry_receipt=context['original_entry_receipt'],
                 message_identity=context['message_identity']))
        observer.current = None
    except Exception as error:
        observer.error(error)


def associating(context, parsing_json=False):
    if context is not None:
        context['phase'] = 'json_parse' if parsing_json else 'association'


def parsed(context, value):
    # This transparent wrapper returns the exact object from the original
    # json.loads call; no second JSON parse or message copy takes place.
    if context is not None:
        observer = context['observer']
        try:
            observer.count(context['topic'], 'parsed')
            context.update(phase='association', source_identity=_identity(value))
        except Exception as error:
            observer.error(error)
    return value


def associated(context, matched):
    if context is None:
        return
    observer = context['observer']
    try:
        after = _association(observer.node)
        before = context['association_before']
        rejected_delta = (after['rejected'] - before['rejected']
            if type(after['rejected']) is int and type(before['rejected']) is int else None)
        outcome = ('matched_candidate_only' if matched is not None else
                   'rejected' if rejected_delta is not None and rejected_delta > 0 else 'pending_or_no_match')
        observer.count(context['topic'], 'association_' + outcome)
        context.update(phase='association_return', association_after=after,
            association_outcome=outcome, association_rejected_delta=rejected_delta)
        if matched is not None:
            context['matched_identity'] = _identity(matched[1])
            context['matched_trajectory_id'] = _small(getattr(matched[0], 'traj_id', None))
        observer.last[context['topic']] = {key: value for key, value in context.items() if key != 'observer'}
        observer.emit(context['topic'], 'association', (outcome, after['last_rejected'],
            tuple(after['pending'].items()), after['epoch'], after['planner_process_id'],
            after['localization_generation']), dict(outcome=outcome,
            original_entry_receipt=context['original_entry_receipt'],
            message_identity=context['message_identity'], source_identity=context.get('source_identity'),
            association_before=before, association_after=after,
            association_rejected_delta=rejected_delta, outer_admission_assumed=False))
    except Exception as error:
        observer.error(error)


def exception(context, error):
    if context is None:
        return
    observer = context['observer']
    try:
        phase = context['phase']
        observer.count(context['topic'], 'exception_' + phase)
        reason = (type(error).__name__ + ': ' + str(error))[:512]
        context.update(exception_phase=phase, exception=reason,
            association_after=_association(observer.node), phase='exception')
        observer.last[context['topic']] = {key: value for key, value in context.items() if key != 'observer'}
        observer.emit(context['topic'], 'exception', (phase, reason),
            dict(exception_phase=phase, error=reason,
                original_entry_receipt=context['original_entry_receipt'],
                message_identity=context['message_identity'],
                association_after=context['association_after']))
    except Exception as observer_error:
        observer.error(observer_error)


def accepting(context):
    if context is not None:
        context['phase'] = 'outer_admission'


def outer(node, value, metadata, decision, origin):
    observer = _observer(node)
    if observer is None:
        return
    try:
        # Called only AFTER the unchanged terminal admission operation. These
        # facts never authorize a command and never write Association state.
        context = observer.current
        candidate_id = value.get('traj_id') if isinstance(value, dict) else None
        same_context = bool(context is not None
            and context.get('association_outcome') == 'matched_candidate_only'
            and context.get('matched_trajectory_id') == candidate_id
            and context.get('matched_identity') == _identity(metadata))
        topic = context['topic'] if same_context else 'bspline'
        if same_context:
            context['outer_terminal_seen'] = True
        admitted = decision.get('admitted')
        if type(admitted) is not bool:
            raise ValueError('outer_terminal_boolean_required')
        observer.count(topic, 'outer_admitted' if admitted else 'outer_rejected')
        proof = decision.get('proof', {})
        final_source = proof.get('r85_final_execution_source_gate', {}) if isinstance(proof, dict) else {}
        details = dict(origin=origin, admitted=admitted, reason=_small(decision.get('reason')),
            admission_sequence=_small(decision.get('sequence')), candidate_trajectory_id=_small(candidate_id),
            source_identity=_identity(metadata), final_source_allowed=_small(final_source.get('allowed')),
            original_entry_receipt=context['original_entry_receipt'] if same_context else None,
            entry_receipt_available=same_context, no_new_receive_stamp_assigned=True,
            post_terminal_observer_hook=True, original_callback_budget_not_rechecked_by_observer=True)
        observer.last['outer'] = dict(**details, **_clock(node))
        observer.emit(topic, 'outer', (admitted, details['reason'], details['origin'],
            details['final_source_allowed'], details['source_identity'].get('reference_epoch'),
            details['source_identity'].get('planner_process_id'),
            details['source_identity'].get('localization_generation')), details)
    except Exception as error:
        observer.error(error)


def returned(context):
    if context is None:
        return
    observer = context['observer']
    try:
        if not context['outer_terminal_seen']:
            observer.count(context['topic'], 'outer_return_without_terminal_decision')
            pending = getattr(observer.node, 'pending_path_commit', None) is not None
            observer.emit(context['topic'], 'outer_return_without_terminal_decision', (pending,),
                dict(pending_path_commit_present=pending, outer_admission='UNKNOWN_NOT_ASSUMED',
                    original_entry_receipt=context['original_entry_receipt']))
    except Exception as error:
        observer.error(error)


def done(context):
    if context is not None:
        context['observer'].current = None


def summary(node):
    observer = _observer(node)
    if observer is None:
        return None
    # Bounded shallow scalar trees only. No coefficients, point arrays, raw
    # String payloads, association cache values, message replay or new messages.
    return dict(schema='r85_protocol_intake_status/v1',
        counts={topic: dict(counts) for topic, counts in observer.counts.items()},
        last=dict(observer.last), events_enqueued=observer.events_enqueued,
        event_limit=MAX_EVENTS, event_limit_reached=observer.events_enqueued >= MAX_EVENTS,
        semantic_repeats_or_cap_suppressed=observer.events_suppressed,
        observer_errors=observer.observer_errors, last_observer_error=observer.last_observer_error,
        navigation_PASS=False, motion_authority=False,
        event_delivery='existing_evidence_writer_enqueue_only',
        evidence_writer_error=_small(getattr(node.evidence, 'error', None)))
