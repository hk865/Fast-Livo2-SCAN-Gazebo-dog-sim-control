"""Simulation-only owned-event XY command mode. No model/actuator/physics IO.

This changes control state and policy velocity input. It grants no quiet, ACK,
formal zero or fixed-five credit. Normal PID evaluation is retained as a shadow;
the persistent XY I/D state and actual post-slew command are recorded separately.
"""
import copy
import hashlib
import json
import math
import numbers
from pathlib import Path

KEY = 'student_xy_stop_takeover'
REQUIRED = KEY + '_required'
FLAG = 'xy_stop_takeover_enabled'
SCHEMA = 'student_xy_stop_takeover/v1'
KIND = 'owned_prezero_native_xy_stop_takeover_v1'
ENTRY_SPEED = .01
ENTRY_SPAN_NS = 300_000_000
EXIT_SPEED = .02
EXIT_SPAN_NS = 100_000_000
SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def configuration(profile):
    ack = profile.get('cascade', {}).get('r85_student_lower_loop_ack', {})
    if FLAG not in ack:
        return None
    if type(ack[FLAG]) is not bool:
        raise ValueError('XY stop mode flag must be an explicit bool')
    if ack[FLAG] is False:
        return None
    if profile.get('actor_kind') != 'student' or profile.get('controller_kind') != 'student':
        raise ValueError('XY stop mode requires the original Student scope')
    return dict(enabled=True, kind=KIND, entry_speed_strict_below_mps=ENTRY_SPEED,
                entry_native_span_ns=ENTRY_SPAN_NS, exit_speed_strict_above_mps=EXIT_SPEED,
                exit_native_span_ns=EXIT_SPAN_NS)


def _sha(v):
    return isinstance(v, str) and len(v) == 64 and all(c in '0123456789abcdef' for c in v)


def _vec(v):
    if not isinstance(v, (list, tuple)) or len(v) != 3 or any(
            isinstance(x, (bool,)) or not isinstance(x, numbers.Real) or not math.isfinite(x) for x in v):
        raise ValueError('Finite numeric body vector3 required')
    return [float(x) for x in v]


def _window(first=None, last=None, samples=0):
    return dict(first_clock_ns=first, last_clock_ns=last, samples=samples,
                span_ns=0 if first is None else last-first)


def _check_window(w, clock):
    if not isinstance(w, dict) or set(w) != {'first_clock_ns', 'last_clock_ns', 'samples', 'span_ns'}:
        raise ValueError('XY mode actual-sample window shape differs')
    first, last, n, span = (w[k] for k in ('first_clock_ns', 'last_clock_ns', 'samples', 'span_ns'))
    if first is None:
        if w != _window():
            raise ValueError('Empty XY mode window differs')
    elif (any(type(x) is not int for x in (first, last, n, span)) or n < 1
          or not 0 <= first <= last <= clock or span != last-first
          or abs(span-(n-1)*20_000_000) > max(1, n-1)*1000):
        raise ValueError('XY mode window is not distinct continuous native samples')


def _state(pid):
    return dict(integral=pid.integral.tolist(), filtered_derivative=pid.filtered_derivative.tolist(),
                previous_measured=None if pid.previous_measured is None else pid.previous_measured.tolist())


def _clear_xy(pid, measured):
    pid.integral[:2] = 0.
    pid.filtered_derivative[:2] = 0.
    if pid.previous_measured is not None:
        pid.previous_measured[:2] = measured[:2]


def validate_source_binding(s, scope, clock, requested, unload_helper=None):
    if unload_helper is None:
        import experiments.candidates.pre_zero_integral_unload as unload_helper
    if not isinstance(s, dict):
        raise ValueError('XY mode active source binding is absent')
    e = unload_helper.validate_event(s.get('event'), s.get('run_id'), scope, clock)
    unload_helper.validate_event(e, s['run_id'], scope, s.get('source_control_clock_ns'))
    r = _vec(requested)
    context = e['context']
    if (r[:2] != [0., 0.] or not any(r) or s.get('accepted_request_body') != r
            or s.get('scope_sha256') != scope or s.get('source_context') != context
            or s.get('motion_epoch') != context['request_id']+':waypoint:'+str(context['waypoint_index'])
            or s.get('source_request_id') != context['request_id']
            or type(s.get('source_waypoint_index')) is not int or s.get('source_waypoint_index') != context['waypoint_index']
            or s.get('stage') != 'controlled_braking' or s.get('cascade_mode') != 'brake'
            or type(s.get('source_zero_handoff_count')) is not int or s.get('source_zero_handoff_count') != e['entry_zero_handoff_count']
            or not _sha(s.get('path_sha256')) or s.get('goal_sha256') != e['goal_sha256']
            or not _sha(s.get('read_raw_bytes_sha256')) or s.get('read_status') != 'accepted'
            or type(s.get('producer_sequence')) is not int or s['producer_sequence'] < 1
            or s.get('read_clock_ns') != clock or s.get('physics_clock_ns') != clock
            or s.get('envelope_healthy') is not True or s.get('envelope_stop_requested') is not False
            or s.get('envelope_scope_sha256') != scope or s.get('authority_rejected') is not False):
        raise ValueError('XY mode source/read/request/event binding differs')
    pose, imu, control = (s.get(k) for k in ('source_pose_stamp_ns', 'paired_imu_stamp_ns', 'source_control_clock_ns'))
    if (any(type(v) is not int for v in (pose, imu, control))
            or not 0 <= clock-pose <= 300_000_000 or not 0 <= pose-imu <= 20_000_000
            or not pose <= control <= clock):
        raise ValueError('XY mode original source clocks are stale or noncausal')
    return s


def validate_receipt(q, scope, stamp, frame, requested, pid=None):
    """Stateless same-frame validation; window continuity is produced by one owner."""
    if (not isinstance(q, dict) or q.get('schema') != SCHEMA or q.get('kind') != KIND
            or q.get('enabled') is not True or q.get('new_control_math') is not True
            or q.get('base_PID_math_claim') is not False or q.get('stop_or_ACK_credit') is not False
            or q.get('pid_terms_are_shadow') is not True or q.get('scope_sha256') != scope
            or not _sha(scope) or q.get('input_clock_ns') != stamp
            or type(stamp) is not int or type(q.get('input_clock_ns')) is not int
            or type(q.get('input_frame_serial')) is not int or q.get('input_frame_serial') != frame
            or type(frame) is not int or frame <= 0 or not .019999 <= q.get('dt_s', 0) <= .020001
            or type(q.get('active')) is not bool or q.get('yaw_unchanged') is not True
            or type(q.get('XY_cleared_before_PID')) is not bool
            or q.get('transition') not in ('none', 'entry', 'release')
            or not isinstance(q.get('reason'), str) or not isinstance(q.get('run_id'), str)
            or not q['run_id'] or q.get('helper_identity', {}).get('sha256') != SOURCE_SHA256):
        raise ValueError('XY mode same-frame explicit receipt differs')
    requested = _vec(list(requested))
    measured = _vec(q.get('measured_native_com_body'))
    speed = q.get('measured_native_COM_XY_norm')
    if (isinstance(speed, bool) or not isinstance(speed, numbers.Real) or not math.isfinite(speed)
            or abs(speed-math.hypot(*measured[:2])) > 1e-14):
        raise ValueError('XY mode speed is not actual native COM norm')
    for name in ('entry_window', 'drift_window'):
        _check_window(q.get(name), stamp)
    before = _vec(q.get('pid_before_stop_override_body'))
    shadow = _vec(q.get('legacy_after_slew_shadow_body'))
    actual = _vec(q.get('actual_after_takeover_body'))
    active = q['active']
    if q.get('source_binding') is not None:
        validate_source_binding(q['source_binding'], scope, stamp, requested)
        if q['source_binding']['run_id'] != q['run_id'] or q['source_binding']['motion_epoch'] != q.get('motion_epoch'):
            raise ValueError('XY mode run/epoch differs from this accepted event')
    if (q.get('bypassed_xy_axes') != [active, active, False] or actual[2] != shadow[2]
            or q.get('XY_I_frozen') is not active or q.get('XY_D_cleared_and_measurement_tracked') is not active
            or q.get('state') != ('active' if active else 'qualifying' if q['entry_window']['samples'] else 'idle')):
        raise ValueError('XY mode bypass/state/yaw identity differs')
    if active:
        validate_source_binding(q.get('source_binding'), scope, stamp, requested)
        if (q['source_binding']['run_id'] != q['run_id'] or q['source_binding']['motion_epoch'] != q.get('motion_epoch')
                or actual[:2] != [0., 0.] or q['entry_window']['span_ns'] < ENTRY_SPAN_NS
                or q['entry_window']['samples'] < 16 or q.get('ordinary_zero_priority') is not False):
            raise ValueError('XY mode active command/entry authority differs')
        if q['drift_window']['samples'] and (q['measured_native_COM_XY_norm'] <= EXIT_SPEED
                                              or q['drift_window']['last_clock_ns'] != stamp
                                              or q['drift_window']['span_ns'] >= EXIT_SPAN_NS):
            raise ValueError('XY mode active drift window already requires release')
    elif actual != shadow:
        raise ValueError('Inactive XY mode changed the original command')
    if not active and q['entry_window']['samples'] and (q.get('source_binding') is None
            or q['entry_window']['last_clock_ns'] != stamp or q['measured_native_COM_XY_norm'] >= ENTRY_SPEED):
        raise ValueError('XY mode qualification lacks current valid below-speed sample')
    if q['entry_window']['samples'] and (q.get('source_binding') is None
            or q['entry_window']['first_clock_ns'] < q['source_binding']['event']['brake_entry_clock_ns']):
        raise ValueError('XY mode credited samples before current owned event')
    if q.get('transition') == 'entry' and not active:
        raise ValueError('XY mode entry is not active')
    if q.get('transition') == 'entry' and (q['measured_native_COM_XY_norm'] >= ENTRY_SPEED
                                          or q['entry_window']['last_clock_ns'] != stamp):
        raise ValueError('XY mode entry lacks this actual qualifying sample')
    if q.get('transition') == 'release' and (active or q.get('XY_cleared_before_PID') is not True):
        raise ValueError('XY mode release did not clear XY state before feedback resume')
    if q.get('reason') == 'native_XY_drift_release' and (q.get('transition') != 'release'
            or q['measured_native_COM_XY_norm'] <= EXIT_SPEED or q['drift_window']['span_ns'] < EXIT_SPAN_NS
            or q['drift_window']['samples'] < 6 or q['drift_window']['last_clock_ns'] != stamp):
        raise ValueError('XY mode drift release lacks actual continuous above-speed span')
    shadow_pid = q.get('pid_before_stop_override')
    if not isinstance(shadow_pid, dict) or before != shadow_pid.get('corrected_before_legacy_slew_body'):
        raise ValueError('XY mode complete PID shadow/body differs')
    if (requested != _vec(shadow_pid.get('requested_command_body'))
            or measured != _vec(shadow_pid.get('measured_native_com_body'))
            or shadow_pid.get('motion_epoch') != q.get('motion_epoch')
            or any(abs(e-(r-y)) > 1e-12 for e,r,y in zip(_vec(shadow_pid.get('pid_error')),requested,measured))):
        raise ValueError('XY mode native request/measurement/epoch differs from this actual PID shadow')
    if abs(shadow_pid.get('dt_s', -1)-q['dt_s']) > 1e-12:
        raise ValueError('XY mode clock interval differs from original PID native interval')
    if pid is not None:
        if (any(pid.get(k) != v for k, v in shadow_pid.items())
                or pid.get('final_after_legacy_slew_body') != actual):
            raise ValueError('XY mode PID shadow or actual Actor input differs')
    pre, post = q.get('pid_state_after_shadow'), q.get('persistent_PID_state_after')
    if not isinstance(pre, dict) or not isinstance(post, dict):
        raise ValueError('XY mode real persistent PID state missing')
    for k in ('integral', 'filtered_derivative', 'previous_measured'):
        _vec(pre.get(k)); _vec(post.get(k))
        if pre[k][2] != post[k][2]:
            raise ValueError('XY mode changed yaw PID state')
    if q['XY_cleared_before_PID']:
        seeded = q.get('pid_state_before')
        prior = q.get('pid_state_before_release_clear')
        if (not isinstance(seeded, dict) or not isinstance(prior, dict)
                or seeded.get('integral', [])[:2] != [0., 0.]
                or seeded.get('filtered_derivative', [])[:2] != [0., 0.]
                or seeded.get('previous_measured', [])[:2] != measured[:2]
                or any(seeded[k][2] != prior[k][2] for k in ('integral', 'filtered_derivative', 'previous_measured'))):
            raise ValueError('XY mode release state was not cleared/rebased or changed yaw')
    if active and (post['integral'][:2] != [0., 0.] or post['filtered_derivative'][:2] != [0., 0.]
                   or post['previous_measured'][:2] != measured[:2]):
        raise ValueError('XY mode active I/D state not frozen/tracked')
    return q


class XYStopTakeover:
    def __init__(self, config, unload_helper, run_id, scope_sha, identity):
        expected = dict(enabled=True, kind=KIND, entry_speed_strict_below_mps=ENTRY_SPEED,
                        entry_native_span_ns=ENTRY_SPAN_NS, exit_speed_strict_above_mps=EXIT_SPEED,
                        exit_native_span_ns=EXIT_SPAN_NS)
        if config != expected or not _sha(scope_sha) or identity.get('sha256') != SOURCE_SHA256:
            raise ValueError('Enabled XY mode needs exact bound configuration')
        self.config, self.unload_helper = config, unload_helper
        self.run_id, self.scope_sha, self.identity = run_id, scope_sha, identity
        self.active = False
        self.context = None
        self.entry = _window(); self.drift = _window()
        self.last_clock = None; self.last_frame = None; self.pending = None

    def _source(self, evidence, requested, epoch, clock, authorization):
        if evidence.get('rejected') is not False:
            return None, 'original_authority_rejected'
        if not any(requested):
            return None, 'original_complete_zero_priority'
        if any(requested[:2]):
            return None, 'new_nonzero_XY_request'
        envelope = evidence.get('actual_original_envelope') or {}
        source = envelope.get('controller_source') or {}
        if source.get('heading_transition_stage') != 'controlled_braking':
            return None, 'outside_owned_controlled_braking'
        if authorization is None:
            raise ValueError('Enabled XY mode controlled braking has no original owned authorization')
        e = authorization['event']
        if not e['brake_entry_clock_ns'] <= clock < e['expires_clock_ns']:
            return None, 'owned_event_expired_at_native_input'
        read = evidence.get('actual_read_attempt') or {}
        if read.get('decoded_envelope') != envelope:
            raise ValueError('XY mode accepted read/envelope copy differs')
        s = dict(run_id=self.run_id, scope_sha256=self.scope_sha, motion_epoch=epoch,
                 event=copy.deepcopy(e), source_context=copy.deepcopy(source.get('heading_transition_context')),
                 source_request_id=source.get('request_id'), source_waypoint_index=source.get('waypoint_index'),
                 stage=source.get('heading_transition_stage'), cascade_mode=source.get('cascade_mode'),
                 source_zero_handoff_count=source.get('heading_zero_handoff_count'),
                 path_sha256=source.get('path_sha256'), goal_sha256=source.get('goal_sha256'),
                 source_control_clock_ns=source.get('control_stamp_ns'), source_pose_stamp_ns=source.get('source_pose_stamp_ns'),
                 paired_imu_stamp_ns=source.get('paired_imu_stamp_ns'), producer_sequence=envelope.get('sequence'),
                 read_raw_bytes_sha256=read.get('raw_bytes_sha256'), read_status=read.get('status'),
                 read_clock_ns=read.get('read_clock_ns'), physics_clock_ns=evidence.get('physics_clock_ns'),
                 envelope_healthy=envelope.get('healthy'), envelope_stop_requested=envelope.get('stop_requested'),
                 envelope_scope_sha256=envelope.get('acceptance', {}).get('sha256'),
                 authority_rejected=evidence.get('rejected'), accepted_request_body=list(requested))
        if (source.get('pre_zero_integral_unload') != e or list(source.get('command_after_slew', [])) != list(requested)
                or envelope.get('command') != list(requested) or evidence.get('command') != list(requested)):
            raise ValueError('XY mode original event/request differs from source')
        validate_source_binding(s, self.scope_sha, clock, requested, self.unload_helper)
        return s, 'valid_owned_zero_XY_intent'

    def before_pid(self, evidence, requested, measured, epoch, clock, frame, authorization, pid):
        requested, measured = _vec(list(requested)), _vec(list(measured))
        if (self.pending is not None or type(clock) is not int or type(frame) is not int or frame <= 0
                or self.last_clock is not None and (not 19_999_000 <= clock-self.last_clock <= 20_001_000
                                                   or frame != self.last_frame+1)):
            raise ValueError('XY mode requires one fresh contiguous native frame')
        dt = .02 if self.last_clock is None else (clock-self.last_clock)/1e9
        self.last_clock, self.last_frame = clock, frame
        source, reason = self._source(evidence or {}, requested, epoch, clock, authorization)
        context = None if source is None else (
            epoch, source['event']['event_id'], source['path_sha256'], source['goal_sha256'],
            json.dumps(source['source_context'], sort_keys=True, separators=(',', ':')))
        speed = math.hypot(*measured[:2])
        prior_context = self.context
        transition = 'none'; clear_before = False
        changed = self.context is not None and context != self.context
        if source is None or changed:
            if self.active:
                transition = 'release'; clear_before = True
            self.active = False; self.entry = _window(); self.drift = _window()
            if changed and source is not None:
                reason = 'owned_event_epoch_path_or_context_changed'
            self.context = context
        else:
            self.context = context
            if self.active:
                if speed > EXIT_SPEED:
                    self.drift = (_window(clock, clock, 1) if not self.drift['samples'] else
                                  _window(self.drift['first_clock_ns'], clock, self.drift['samples']+1))
                    if self.drift['span_ns'] >= EXIT_SPAN_NS:
                        self.active = False; transition = 'release'; clear_before = True
                        reason = 'native_XY_drift_release'; self.entry = _window()
                else:
                    self.drift = _window()
            elif speed < ENTRY_SPEED:
                self.drift = _window()
                self.entry = (_window(clock, clock, 1) if not self.entry['samples'] else
                              _window(self.entry['first_clock_ns'], clock, self.entry['samples']+1))
                if self.entry['span_ns'] >= ENTRY_SPAN_NS:
                    self.active = True; transition = 'entry'; reason = 'native_XY_stop_span_reached'
            else:
                self.entry = _window(); self.drift = _window(); reason = 'native_XY_not_below_entry'
        before = _state(pid)
        if clear_before:
            _clear_xy(pid, measured)
        self.pending = dict(schema=SCHEMA, kind=KIND, enabled=True, new_control_math=True,
            base_PID_math_claim=False, stop_or_ACK_credit=False, pid_terms_are_shadow=True,
            run_id=self.run_id, scope_sha256=self.scope_sha, helper_identity=copy.deepcopy(self.identity),
            motion_epoch=epoch, input_clock_ns=clock, input_frame_serial=frame, dt_s=dt,
            previous_mode_context=copy.deepcopy(prior_context), current_mode_context=copy.deepcopy(context),
            active=self.active, state='active' if self.active else 'qualifying' if self.entry['samples'] else 'idle',
            transition=transition, reason=reason, measured_native_com_body=measured,
            measured_native_COM_XY_norm=speed, entry_window=copy.deepcopy(self.entry),
            drift_window=copy.deepcopy(self.drift), source_binding=source,
            ordinary_zero_priority=not any(requested), XY_cleared_before_PID=clear_before,
            XY_I_frozen=self.active, XY_D_cleared_and_measurement_tracked=self.active,
            pid_state_before=_state(pid) if clear_before else before,
            pid_state_before_release_clear=before)
        return self.pending

    def after_pid(self, pid):
        q = self.pending
        if q is None or 'pid_before_stop_override' in q:
            raise ValueError('XY mode PID evaluation is not one current native frame')
        q['pid_before_stop_override'] = copy.deepcopy(pid.last_diagnostic)
        q['pid_before_stop_override_body'] = list(pid.last_diagnostic['corrected_before_legacy_slew_body'])
        q['pid_state_after_shadow'] = _state(pid)
        if q['active']:
            _clear_xy(pid, q['measured_native_com_body'])
        q['persistent_PID_state_after'] = _state(pid)

    def after_legacy_slew(self, cmd, requested, ordinary_proof, pid):
        q = self.pending
        if q is None or 'pid_state_after_shadow' not in q:
            raise ValueError('XY mode has no current original PID shadow')
        q['legacy_after_slew_shadow_body'] = cmd.tolist()
        if q['active']:
            if ordinary_proof is not None or not any(requested):
                raise ValueError('XY mode cannot override original complete-zero priority')
            # The original worker's sole slew memory is this exact mutable cmd.
            # No hidden command cache or second slew exists in original prepare.
            cmd[:2] = 0.
        q['actual_after_takeover_body'] = cmd.tolist()
        q['bypassed_xy_axes'] = [q['active'], q['active'], False]
        q['yaw_unchanged'] = bool(cmd[2] == q['legacy_after_slew_shadow_body'][2])
        validate_receipt(q, self.scope_sha, q['input_clock_ns'], q['input_frame_serial'], requested)
        self.pending = None
        return q
