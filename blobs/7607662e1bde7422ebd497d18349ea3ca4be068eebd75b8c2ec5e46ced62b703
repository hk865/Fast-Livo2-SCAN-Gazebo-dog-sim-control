"""Five bounded event-selected source groups; no localization/control authority.

Runtime hooks observe original inputs and decisions. Only this writer serializes
messages. Its receipt is a diagnostic stop hint, never a mission/quality PASS.
No ROS node, process, thread or file is created by importing this module.
"""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
from collections import OrderedDict, deque

MIB = 1024 * 1024
ROLES = ('raw', 'adapted', 'registered', 'body', 'mapper')
GROUP_ROLES = ('A0', 'A1', 'B', 'C', 'D')
CAPS = dict(groups=5, group_bytes=4*MIB, raw_bytes=MIB, adapted_bytes=MIB,
            registered_bytes=1536*1024, auxiliary_bytes=512*1024,
            shared_bytes=MIB, total_bytes=24*MIB, event_window_ns=90_000_000_000)
GATES = dict(native_response_ns=200_000_000, source_and_SCAN_freshness_ns=300_000_000,
             correction_age_ns=3_000_000_000, allray_abs_p95_m=.35,
             foreground_fraction=.05, surface_p95_m=.12,
             stage_geometry_RMS_m=.006, stage_geometry_maximum_m=.030,
             stage_nearest_index_bijection=True, require_complete_imu=False)
TOPICS = dict(imu_source='/livox/imu', imu_relay='/demo/teacher/slam/imu', clock='/clock')
PHASES = dict(exploring='exploration', returning='return_origin', navigating='navigation_f1_f3')
TRIGGER = dict(original_reason='no_quality_candidate', distinct_rejections=2,
               anchor_age_lead_ns=2_500_000_000, complete_ring_groups=4)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n').encode()


def msg_stamp(msg, clock=False):
    s = msg.clock if clock else msg.header.stamp
    return int(s.sec)*1_000_000_000 + int(s.nanosec)


def as_list(value):
    return value.tolist() if hasattr(value, 'tolist') else value


def independent(a, b):
    """Original confirmation bounds, used only to label two captured anchors."""
    dt = b['source_ns'] - a['source_ns']
    if not 150_000_000 <= dt <= 10_000_000_000:
        return False
    ca, cb = a['result'].get('C_world_odom'), b['result'].get('C_world_odom')
    if ca is None or cb is None:
        return False
    translation = math.sqrt(sum((ca[i][3]-cb[i][3])**2 for i in range(3)))
    trace = sum(sum(ca[k][i]*cb[k][i] for k in range(3)) for i in range(3))
    angle = math.acos(max(-1., min(1., (trace-1)/2)))
    return translation <= .15 and angle <= .08


class BoundedSourceFrames:
    """API compatible with MatcherWorker.consider/reject; hooks are read-only."""
    def __init__(self, run, *, contract=None, serialize=None, deserialize=None,
                 start_writer=True, wall_ns=time.monotonic_ns):
        self.run = Path(run)
        self.root = self.run/'known_scene_event_frames'
        self.wall_ns = wall_ns
        self.contract_path = self.run/'known_scene_event_capture_contract.json'
        if contract is None:
            if self.contract_path.stat().st_size > 64*1024:
                raise ValueError('bounded capture contract too large')
            contract = json.loads(self.contract_path.read_text())
        if (contract.get('schema') != 'go2_event_source_capture_contract/v2'
                or contract.get('run_id') != self.run.name
                or contract.get('caps') != CAPS or contract.get('unchanged_gates') != GATES
                or contract.get('actual_mission_phases') != PHASES
                or contract.get('diagnostic_trigger') != TRIGGER
                or contract.get('topics') != TOPICS
                or contract.get('diagnostic_only') is not True):
            raise ValueError('exact event-aware diagnostic contract required')
        self.contract = contract
        self.contract_sha256 = sha(encoded(contract))
        self.bindings = []
        for item in contract.get('source_bindings', []):
            if file_sha(item['path']) != item['sha256']:
                raise ValueError('capture frozen source hash mismatch: '+item['role'])
            self.bindings.append(dict(item))
        if not self.bindings:
            raise ValueError('source bindings required')
        if self.root.exists():
            raise ValueError('new exclusive capture directory required')
        self.root.mkdir()
        self.block = max(4096, os.statvfs(self.root).f_frsize)
        self.serialize, self.deserialize = serialize, deserialize
        self.lock = threading.RLock()
        self.jobs = queue.Queue(maxsize=5)
        self.writer = None
        self.closed = False
        self.write_failed = False
        self.matcher_worker_drained = None
        self.witness = None
        self.witness_status = None
        self.source_checker = None
        self.mission = None
        self.last_valid_request = None
        self.request_epoch = 0
        self.request_start_ns = None
        self.inflight_binding = None
        self.arm = None
        self.expired = False
        self.terminal = None
        self.last_clock = 0
        self.last_metadata = None
        self.qualified = OrderedDict()  # <=4 original accepted matcher jobs
        self.anchors = deque(maxlen=2)  # actual metadata-anchored jobs
        self.complete = OrderedDict()  # <=4 complete five-role groups, no poses looked up
        self.inflight_source = None  # one original verified matcher job, before slow fitting
        self.last_complete_seen = -1
        self.watermark = -1
        self.aux = {k: deque(maxlen=1200 if k != 'clock' else 256) for k in TOPICS}
        self.aux_overflow = {k: 0 for k in TOPICS}
        self.selected = {}
        self.saved = {}
        self.missing = {}
        self.rejected = deque(maxlen=32)
        self.events = deque(maxlen=64)
        self.b_event = None
        self.pending_rejection = None  # one first rejected complete matcher job
        self.c_event = None
        self.c_first_after = None
        self.clear_event = None
        self.last_clear = False
        self.progress_dirty = threading.Event()
        self.progress_dirty.set()
        if start_writer:
            self.writer = threading.Thread(target=self._writer, name='bounded_source_writer', daemon=True)
            self.writer.start()

    def configure_witness(self, status, checker):
        """Called only after original validate_runtime_witness succeeded."""
        if status.get('valid') is not True or status.get('run_id') != self.run.name:
            raise ValueError('original runtime witness not valid for this run')
        p = self.run/'known_scene_runtime_witness.json'
        if p.stat().st_size > 64*1024:
            raise ValueError('witness too large')
        witness = json.loads(p.read_text())
        with self.lock:
            self.witness = dict(path=str(p), sha256=file_sha(p),
                model=witness.get('producer_model'),
                header_snapshot_time_verified=witness.get('header_snapshot_time_verified') is True,
                post_lio_single_owner_publication_verified=witness.get('post_lio_single_owner_publication_verified') is True)
            self.witness_status = dict(status)
            self.source_checker = checker

    def _event(self, event_kind, **values):
        row = dict(event=event_kind, observed_wall_ns=self.wall_ns())
        row.update(values)
        self.events.append(row)
        self.progress_dirty.set()

    def reject(self, source, reason):
        with self.lock:
            self.rejected.append(dict(source_ns=source, reason=str(reason)[:512]))
            self.progress_dirty.set()
        return False

    def _clock(self, ns):
        self.last_clock = max(self.last_clock, int(ns))
        if self.arm and ns >= self.arm['runtime_clock_ns'] + CAPS['event_window_ns']:
            if not self.expired:
                self.expired = True
                self._event('event_window_expired', runtime_clock_ns=int(ns))

    def _observing(self):
        return bool(not self.expired and not self.terminal and not self.closed
                    and self.mission
                    and 0 <= self.wall_ns()-self.mission['observed_wall_ns'] < 300_000_000)

    def _active(self):
        return bool(self.arm and self._observing() and self.mission['request_id'] == self.arm['request_id'])

    def _arm_pending(self):
        """Observe persistence or impending ORIGINAL expiry; no control decision."""
        pending = self.pending_rejection
        if self.arm or not pending or not self._observing():
            return False
        a1 = pending['anchors'][-1]
        metadata = self.last_metadata
        lead = bool(metadata and metadata.get('generation') == pending['generation']
                    and metadata.get('match_source_ns') == a1['source_ns']
                    and metadata['source_ns']-a1['source_ns'] >= TRIGGER['anchor_age_lead_ns'])
        if len(pending['rejection_sources']) < TRIGGER['distinct_rejections'] and not lead:
            return False
        self.arm = dict(self.mission, runtime_clock_ns=self.last_clock,
                        trigger='sustained_original_quality_rejection_or_anchor_expiry_lead',
                        trigger_source_ns=pending['snapshot']['source_ns'],
                        persistence_sources_ns=list(pending['rejection_sources']),
                        original_anchor_ns=a1['source_ns'], anchor_expiry_lead_observed=lead)
        self._event('actual_phase_rejection_event_armed', **self.arm)
        self.b_event = dict(pending['event'])
        self._select('A1', a1, dict(kind='last_actual_anchor_before_B', B_source_ns=self.b_event['source_ns']))
        if len(pending['anchors']) == 2 and independent(pending['anchors'][0], a1):
            self._select('A0', pending['anchors'][0], dict(kind='previous_independent_actual_anchor',
                                                        B_source_ns=self.b_event['source_ns']))
        else:
            self.missing['A0'] = 'independent previous actual anchor unavailable before actual rejection'
        self._select('B', pending['snapshot'], self.b_event)
        self._event('B_selected', **self.b_event)
        self.pending_rejection = None
        self._select_c()
        return True

    def observe_mission(self, text, runtime_clock_ns):
        """Actual mission topic, not a guessed XY/time cue; payload retained by SHA."""
        try:
            if not isinstance(text, str) or len(text.encode()) > 128*1024:
                return self.reject(None, 'bounded_mission_payload')
            d = json.loads(text)
            if (d.get('run_id') != self.run.name or d.get('navigation_ground_truth_used') is not False
                    or d.get('schema') != 'teacher_original_mission46_state/v1'
                    or d.get('source') != 'actual_SLAM_IMU_SCAN_Teacher46_coordinator'):
                with self.lock:
                    self.mission = None
                return False
            with self.lock:
                self._clock(runtime_clock_ns)
                if d.get('stage') in ('failed', 'completed', 'stopped'):
                    self.terminal = dict(stage=d['stage'], sim_ns=d.get('sim_ns'), observed_wall_ns=self.wall_ns())
                    self._event('original_mission_terminal', **self.terminal)
                    return False
                request = d.get('current_request')
                valid = (d.get('schema') == 'teacher_original_mission46_state/v1'
                    and d.get('stage') in PHASES and isinstance(request, str)
                    and request.startswith(self.run.name+':'+PHASES[d['stage']]+':')
                    and bool(d.get('current_goals_sha256'))
                    and d.get('source') == 'actual_SLAM_IMU_SCAN_Teacher46_coordinator')
                if not valid:
                    self.mission = None
                    return False
                row = dict(stage=d['stage'], request_id=request, sim_ns=d.get('sim_ns'),
                    sequence=d.get('sequence'), current_goals_sha256=d['current_goals_sha256'],
                    payload_sha256=sha(text.encode()), observed_wall_ns=self.wall_ns())
                if self.arm and request != self.arm['request_id']:
                    self.terminal = dict(stage='request_changed', request_id=request, observed_wall_ns=self.wall_ns())
                    self._event('request_changed_stop_selection', request_id=request)
                elif not self.arm and request != self.last_valid_request:
                    self.pending_rejection = None
                    self.anchors.clear(); self.qualified.clear(); self.complete.clear()
                    self.inflight_source = None
                    self.inflight_binding = None
                    self.last_metadata = None; self.c_first_after = None
                    self.request_epoch += 1
                    self.request_start_ns = d.get('sim_ns')
                    self._event('pretrigger_actual_request_changed', request_id=request)
                self.last_valid_request = request
                self.mission = row
                return self._observing()
        except Exception as error:
            return self.reject(None, 'mission_observation:'+type(error).__name__+':'+str(error))

    def observe_aux(self, role, msg, runtime_clock_ns, received_wall_ns=None):
        """Retain small bounded source/relay IMU and clock rings, including before arm."""
        try:
            if role not in TOPICS:
                return False
            ns = msg_stamp(msg, role == 'clock')
            wall = self.wall_ns() if received_wall_ns is None else received_wall_ns
            with self.lock:
                self._clock(runtime_clock_ns)
                if self.closed or self.expired or self.terminal:
                    return False
                ring = self.aux[role]
                if ring and ns <= ring[-1]['source_ns']:
                    return self.reject(ns, 'aux_duplicate_or_backward:'+role)
                if len(ring) == ring.maxlen:
                    self.aux_overflow[role] += 1
                ring.append(dict(source_ns=ns, received_wall_ns=wall, message=msg))
                while ring and ns-ring[0]['source_ns'] > 4_000_000_000:
                    ring.popleft()
                return True
        except Exception as error:
            return self.reject(None, 'aux_observation:'+type(error).__name__+':'+str(error))

    def observe_source(self, join, source_ns, runtime_clock_ns):
        """Tap AFTER unchanged join.put. No _check, serialization or file IO here."""
        try:
            with self.lock:
                self._clock(runtime_clock_ns)
                if not self._observing():
                    return False
            with join.lock:
                slot = join.slots.get(source_ns)
                if not slot or any(k not in slot for k in ROLES):
                    return False
                pairs = {k: slot[k] for k in ROLES}
                watermark = min(join.last_by_role.get(k, -1) for k in ROLES)
            if any(msg_stamp(v[0]) != source_ns for v in pairs.values()):
                return self.reject(source_ns, 'tap_integer_stamp_mismatch')
            if any(len(pairs[k][0].data) > CAPS[k+'_bytes'] for k in ('raw','adapted','registered')):
                return self.reject(source_ns, 'tap_cloud_payload_admission_bound')
            snap = dict(source_ns=source_ns, source_messages={k:v[0] for k,v in pairs.items()},
                original_received_wall_ns={k:v[1] for k,v in pairs.items()},
                observed_complete_wall_ns=self.wall_ns(), runtime_clock_ns=int(runtime_clock_ns),
                source_checker=join._check, source_slot=pairs, result=None, input_C_world_odom=None,
                parent_generation=None, provenance=None, T_odom_body=None)
            with self.lock:
                if not self._observing() or source_ns in self.complete:
                    return False
                self.complete[source_ns] = snap
                snap['aux_at_source_observation'] = self._aux_snapshot(source_ns)
                while len(self.complete) > TRIGGER['complete_ring_groups']:
                    self.complete.popitem(last=False)
                self.last_complete_seen = source_ns
                self.watermark = max(self.watermark, watermark)
                self._select_c()
                return True
        except Exception as error:
            return self.reject(source_ns, 'source_observation:'+type(error).__name__+':'+str(error))

    def _snapshot(self, job, result, c, generation):
        ns = job['source_ns']
        messages = job.get('source_messages', {})
        if set(messages) != set(ROLES) or any(msg_stamp(m) != ns for m in messages.values()):
            raise ValueError('five whole messages at exact integer source stamp required')
        if any(len(messages[k].data) > CAPS[k+'_bytes'] for k in ('raw','adapted','registered')):
            raise ValueError('matcher cloud payload admission bound')
        prov = job.get('provenance', {})
        if not (prov.get('integer_stamp_join') and prov.get('source_membership_verified')
                and prov.get('stage') == 'frozen_post_LIO_single_owner_publication'):
            raise ValueError('original stage/member provenance required')
        core = json.loads(encoded(result))
        return dict(source_ns=ns, source_messages=dict(messages),
            original_received_wall_ns=dict(job['original_received_wall_ns']),
            observed_complete_wall_ns=self.wall_ns(), runtime_clock_ns=self.last_clock,
            result=core, input_C_world_odom=as_list(c), parent_generation=generation,
            provenance=dict(prov), T_odom_body=as_list(job['T_odom_body']),
            aux_at_source_observation=self._aux_snapshot(ns))

    def observe_job(self, job, current_c, parent_generation):
        """Pin one ORIGINAL verified in-flight job before fit; no IO or recheck."""
        try:
            with self.lock:
                if not self._observing():
                    return False
                ns = job['source_ns']; messages = job['source_messages']; prov = job['provenance']
                if type(self.request_start_ns) is not int or ns < self.request_start_ns:
                    return self.reject(ns, 'inflight_source_precedes_actual_request')
                if (set(messages) != set(ROLES) or any(msg_stamp(msg) != ns for msg in messages.values())
                        or not prov.get('integer_stamp_join') or not prov.get('source_membership_verified')
                        or prov.get('stage') != 'frozen_post_LIO_single_owner_publication'):
                    return self.reject(ns, 'inflight_original_source_provenance')
                if any(len(messages[k].data) > CAPS[k+'_bytes'] for k in ('raw','adapted','registered')):
                    return self.reject(ns, 'inflight_whole_payload_bound')
                self.inflight_source = dict(source_ns=ns, source_messages=dict(messages),
                    original_received_wall_ns=dict(job['original_received_wall_ns']),
                    observed_complete_wall_ns=self.wall_ns(), runtime_clock_ns=self.last_clock,
                    result=None, input_C_world_odom=as_list(current_c), parent_generation=parent_generation,
                    provenance=dict(prov), T_odom_body=as_list(job['T_odom_body']),
                    aux_at_source_observation=self._aux_snapshot(ns))
                self.inflight_binding = dict(job_identity=id(job), source_ns=ns,
                    request_epoch=self.request_epoch, request_id=self.mission['request_id'])
                self._select_c()
                return True
        except Exception as error:
            return self.reject(job.get('source_ns'), 'inflight_observation:'+type(error).__name__+':'+str(error))

    def consider(self, job, result, current_c, parent_generation):
        try:
            with self.lock:
                if not self._observing():
                    return False
                ns = job['source_ns']
                binding = self.inflight_binding
                if (not binding or binding['job_identity'] != id(job) or binding['source_ns'] != ns
                        or binding['request_epoch'] != self.request_epoch
                        or binding['request_id'] != self.mission['request_id']):
                    return self.reject(ns, 'completed_job_diagnostic_request_epoch_changed')
                if self.arm and ns > self.arm['runtime_clock_ns']+CAPS['event_window_ns']:
                    return False
                top = (result.get('candidates') or [None])[0]
                qualified = (result.get('accepted') is True and result.get('evidence_verified') is True
                             and top and top.get('quality_passed') is True)
                quality_rejected = bool(result.get('reason') == TRIGGER['original_reason']
                                        and top and top.get('quality_passed') is False and top.get('rejections'))
                if qualified:
                    snap = self._snapshot(job, result, current_c, parent_generation)
                    self.qualified[ns] = snap
                    while len(self.qualified) > 4:
                        self.qualified.popitem(last=False)
                    if self.b_event and 'D' not in self.selected and ns > self.b_event['source_ns']:
                        self._select('D', snap, dict(kind='first_original_quality_recovery',
                            source_ns=ns, parent_generation=parent_generation,
                            original_reason=result.get('reason'), original_quality_passed=True))
                if self.arm is None and quality_rejected and self.anchors:
                    a1 = self.anchors[-1]
                    if ns <= a1['source_ns'] or parent_generation != a1['parent_generation']:
                        return False
                    if (not self.pending_rejection or self.pending_rejection['generation'] != parent_generation
                            or self.pending_rejection['anchors'][-1]['source_ns'] != a1['source_ns']):
                        snap = self._snapshot(job, result, current_c, parent_generation)
                        event = dict(kind='first_original_quality_rejection_of_sustained_episode', source_ns=ns,
                            parent_generation=parent_generation, anchor_ns=a1['source_ns'],
                            observed_wall_ns=self.wall_ns(), original_reason=result.get('reason'),
                            original_rejections=list(top['rejections']), request_id=self.mission['request_id'],
                            actual_stage=self.mission['stage'])
                        self.pending_rejection = dict(snapshot=snap, anchors=list(self.anchors),
                                                      generation=parent_generation, event=event, rejection_sources=[ns])
                    elif ns not in self.pending_rejection['rejection_sources']:
                        self.pending_rejection['rejection_sources'].append(ns)
                        self.pending_rejection['rejection_sources'] = self.pending_rejection['rejection_sources'][-2:]
                    self._arm_pending()
                elif (self.b_event and self.clear_event and quality_rejected and 'D' not in self.selected
                      and ns >= self.clear_event['sim_ns'] and ns > self.b_event['source_ns']):
                    self._select('D', self._snapshot(job, result, current_c, parent_generation),
                        dict(kind='first_complete_matched_group_after_actual_clear_still_rejected',
                            actual_clear_event=self.clear_event, original_quality_passed=False,
                            original_rejections=list(top['rejections'])))
                return True
        except Exception as error:
            return self.reject(job.get('source_ns'), 'match_observation:'+type(error).__name__+':'+str(error))

    def observe_metadata(self, metadata, runtime_clock_ns):
        """Called after original bundle publication/log write, observes actual stale."""
        try:
            with self.lock:
                self._clock(runtime_clock_ns)
                if not self._observing() or metadata.get('run_id') != self.run.name:
                    return False
                row = dict(metadata, runtime_clock_ns=int(runtime_clock_ns), observed_wall_ns=self.wall_ns())
                old = self.last_metadata
                self.last_metadata = row
                anchor = metadata.get('match_source_ns')
                if not metadata.get('correction_stale') and anchor in self.qualified:
                    snap = self.qualified[anchor]
                    if (snap['parent_generation'] == metadata.get('generation')
                            and (not self.anchors or self.anchors[-1]['source_ns'] != anchor)):
                        snap = dict(snap, actual_anchor_metadata=row)
                        self.anchors.append(snap)
                if self.pending_rejection and (metadata.get('generation') != self.pending_rejection['generation']
                        or (not metadata.get('correction_stale') and anchor != self.pending_rejection['anchors'][-1]['source_ns'])):
                    self._event('transient_rejection_pending_cleared_by_actual_anchor', match_source_ns=anchor)
                    self.pending_rejection = None
                self._arm_pending()
                if (self.b_event and self.c_event is None and old
                        and old.get('correction_stale') is False and metadata.get('correction_stale') is True
                        and metadata.get('generation') == self.b_event['parent_generation']
                        and anchor == self.b_event['anchor_ns']):
                    self.c_event = dict(kind='first_actual_stale_after_B',
                        event_source_ns=metadata['source_ns'], event_runtime_clock_ns=int(runtime_clock_ns),
                        generation=metadata['generation'], match_source_ns=anchor,
                        observed_wall_ns=self.wall_ns(), exact_event_metadata=row)
                    self._event('C_actual_stale', **self.c_event)
                    self._select_c()
                return True
        except Exception as error:
            return self.reject(metadata.get('source_ns'), 'metadata_observation:'+type(error).__name__+':'+str(error))

    def _select_c(self):
        if self.c_event is None or 'C' in self.selected:
            return
        ns = self.c_event['event_source_ns']
        complete = dict(self.complete)
        if self.inflight_source:
            complete.setdefault(self.inflight_source['source_ns'], self.inflight_source)
        if ns in complete:
            snap = complete[ns]
            self._select('C', snap, dict(self.c_event, association='exact_same_integer_stamp',
                captured_group_source_ns=ns, nearest_pose_used=False))
            return
        later = [v for k,v in complete.items() if k > ns
                 and v['observed_complete_wall_ns'] >= self.c_event['observed_wall_ns']]
        if later and self.c_first_after is None:
            self.c_first_after = min(later, key=lambda v:v['observed_complete_wall_ns'])
        # All original role watermarks past the event prove its complete group
        # cannot later enter the monotonic original join. No nearest-pose join.
        if self.c_first_after and self.watermark > ns:
            self.missing['C_exact_event_group'] = 'original five-role monotonic watermark passed event stamp'
            snap = self.c_first_after
            self._select('C', snap, dict(self.c_event,
                association='first_later_complete_group_exact_event_group_missing',
                captured_group_source_ns=snap['source_ns'], nearest_pose_used=False,
                original_role_watermark_ns=self.watermark))

    def observe_obstacle(self, text, runtime_clock_ns):
        try:
            if not isinstance(text, str) or len(text.encode()) > 16*1024:
                return False
            d = json.loads(text)
            with self.lock:
                self._clock(runtime_clock_ns)
                if (not self._active() or d.get('run_id') != self.run.name
                        or d.get('request_id') != self.mission['request_id']
                        or d.get('navigation_ground_truth_used') is not False
                        or d.get('robot_pose_service_called') is not False):
                    return False
                clear = (d.get('visible_phase') == 'clear' and d.get('failure') is None
                    and d.get('gazebo_updates', 0) > 0
                    and d.get('source') == 'actual Teacher world SetEntityPose execution ACK')
                if clear and not self.last_clear:
                    self.clear_event = dict(kind='actual_obstacle_clear_ACK_state', sim_ns=d['sim_ns'],
                        runtime_clock_ns=int(runtime_clock_ns), request_id=d['request_id'],
                        payload_sha256=sha(text.encode()), observed_wall_ns=self.wall_ns(),
                        actual_state=d, dynamic_ray_assignment_proven=False)
                    self._event('scene_clear_observed', sim_ns=d['sim_ns'])
                self.last_clear = clear
                return True
        except Exception as error:
            return self.reject(None, 'obstacle_observation:'+type(error).__name__+':'+str(error))

    def _aux_snapshot(self, source):
        # Current frozen no-time producer is instantaneous. Nonzero/unknown
        # point-time fields are never reinterpreted as a 30Hz scan interval.
        out = {}
        for role, ring in self.aux.items():
            before = [r for r in ring if r['source_ns'] <= source]
            after = [r for r in ring if r['source_ns'] >= source]
            rows = ([before[-1]] if before else []) + ([after[0]] if after else [])
            out[role] = list({r['source_ns']: dict(r) for r in rows}.values())
        return out

    def _select(self, role, snap, event):
        if role in self.selected or len(self.selected) >= 5:
            return False
        request = dict(role=role, snapshot=dict(snap), event=dict(event),
            aux=snap.get('aux_at_source_observation', self._aux_snapshot(snap['source_ns'])),
            mission=dict(self.mission),
            witness=None if self.witness is None else dict(self.witness),
            witness_status=self.witness_status, auxiliary_ring_overflow=dict(self.aux_overflow))
        try:
            self.jobs.put_nowait(request)
        except queue.Full:
            self.missing[role] = 'bounded writer queue full; no retry or replacement'
            return self.reject(snap['source_ns'], 'writer_queue_full:'+role)
        self.selected[role] = dict(source_ns=snap['source_ns'], event=event)
        self._event('group_selected', role=role, source_ns=snap['source_ns'])
        return True

    def _codecs(self):
        if self.serialize is None:
            from rclpy.serialization import serialize_message, deserialize_message
            self.serialize, self.deserialize = serialize_message, deserialize_message

    def _whole(self, msg):
        payload = bytes(self.serialize(msg))
        if self.deserialize(payload, type(msg)) != msg:
            raise ValueError('whole message CDR field roundtrip mismatch')
        return payload

    def _layout(self, msg):
        return dict(frame_id=msg.header.frame_id, height=int(msg.height), width=int(msg.width),
            point_step=int(msg.point_step), row_step=int(msg.row_step),
            is_bigendian=bool(msg.is_bigendian), is_dense=bool(msg.is_dense),
            fields=[dict(name=f.name, offset=int(f.offset), datatype=int(f.datatype), count=int(f.count))
                    for f in msg.fields], payload_bytes=len(msg.data), payload_sha256=sha(bytes(msg.data)))

    def _write_group(self, request):
        self._codecs()
        snap = request['snapshot']; source = snap['source_ns']; role = request['role']
        messages = snap['source_messages']
        if set(messages) != set(ROLES) or any(msg_stamp(m) != source for m in messages.values()):
            raise ValueError('not five exact integer stamp source messages')
        payload = {k+'.cdr': self._whole(m) for k,m in messages.items()}
        for k in ('raw', 'adapted', 'registered'):
            if len(payload[k+'.cdr']) > CAPS[k+'_bytes']:
                raise ValueError('complete '+k+' CDR exceeds hard role admission bound')
        stage_error = None
        if snap.get('source_checker'):
            try:
                checked = snap['source_checker'](source, snap['source_slot'])
                snap.update(provenance=checked['provenance'], T_odom_body=as_list(checked['T_odom_body']))
            except Exception as error:
                stage_error = dict(reason=type(error).__name__+':'+str(error),
                    detail=getattr(error, 'detail', None))
        layouts = {k:self._layout(messages[k]) for k in ('raw','adapted','registered')}
        point_fields = [f['name'] for f in layouts['raw']['fields']
                        if f['name'] in ('time','t','timestamp','offset_time')]
        witness = request['witness'] or {}
        instant = (not point_fields and witness.get('model') == 'gz_gpu_lidar_instantaneous_header_snapshot'
                   and witness.get('header_snapshot_time_verified') is True)
        scan_interval = [source, source] if instant else None
        aux_meta = {}
        for aux_role, rows in request['aux'].items():
            records = []
            for i, row in enumerate(rows):
                name = aux_role+'_'+str(i)+'.cdr'
                data = self._whole(row['message']); payload[name] = data
                records.append(dict(path=name, source_ns=row['source_ns'],
                    received_wall_ns=row['received_wall_ns'], bytes=len(data), sha256=sha(data),
                    arrived_at_collector_before_source_role={k:row['received_wall_ns'] <= v
                        for k,v in snap['original_received_wall_ns'].items()}))
            stamps = [r['source_ns'] for r in records]
            enclosed = bool(instant and stamps and min(stamps) <= source <= max(stamps))
            aux_meta[aux_role] = dict(topic=TOPICS[aux_role], records=records,
                scan_interval_enclosed=enclosed, actual_bracket_gap_ns=max(stamps)-min(stamps) if stamps else None,
                frontend_consumption_observed=False, scan_coverage_verified=enclosed,
                point_time_interval_verified=instant, source_and_arrival_are_actual=True)
        source_imus = {r['source_ns']:r['sha256'] for r in aux_meta['imu_source']['records']}
        relay_imus = {r['source_ns']:r['sha256'] for r in aux_meta['imu_relay']['records']}
        same_stamps = sorted(set(source_imus) & set(relay_imus))
        imu_identity = {str(ns):source_imus[ns] == relay_imus[ns] for ns in same_stamps}
        metadata = dict(schema='known_scene_event_complete_group/v1', run_id=self.run.name,
            role=role, source_ns=source, event=request['event'], actual_mission=request['mission'],
            contract_sha256=self.contract_sha256, source_bindings=self.bindings,
            original_source_received_wall_ns=snap['original_received_wall_ns'],
            observed_complete_wall_ns=snap['observed_complete_wall_ns'],
            capture_writer_wall_ns=self.wall_ns(), diagnostic_only=True,
            provenance=snap.get('provenance'), source_join_rejection=stage_error,
            T_odom_body=snap.get('T_odom_body'), input_C_world_odom=snap.get('input_C_world_odom'),
            parent_generation=snap.get('parent_generation'),
            original_match_result=snap.get('result'),
            original_match_result_sha256=None if snap.get('result') is None else sha(encoded(snap['result'])),
            original_match_log=str(self.run/'known_scene_matches.jsonl'),
            match_hash_projection='result before worker timing and parent_generation additions',
            actual_anchor_metadata=snap.get('actual_anchor_metadata'),
            producer_witness=witness, original_witness_validation=request['witness_status'],
            cloud_layouts=layouts, raw_point_time_fields=point_fields,
            point_time_model='version_bound_instantaneous_header' if instant else 'UNVERIFIED',
            real_scan_interval_ns=scan_interval,
            fabricated_point_time=False, auxiliary=aux_meta,
            source_relay_same_stamp_complete_CDR_equal=imu_identity,
            auxiliary_ring_overflow=request['auxiliary_ring_overflow'],
            require_complete_imu=False, complete_original_LIO_IMU_update_claimed=False,
            explicit_message_commit_token=False,
            five_role_integer_stamp_join=True, nearest_pose_used=False,
            full_source_messages_preserved=True, source_arrays_truncated=False,
            capture_representation='complete_lossless_received_ROS_message_reserialization_not_wire_CDR',
            serialized_message_roundtrip_fields_verified=True,
            robot_truth_pose_used=False, IMU_world_orientation_used_as_prior=False,
            frontend_height_used_as_prior=False, localization_admission=False, navigation_verified=False,
            files={name:dict(bytes=len(data), sha256=sha(data)) for name,data in payload.items()})
        body = encoded(metadata); payload['GROUP.json'] = body
        auxiliary_size = sum(len(v) for k,v in payload.items() if k not in ('raw.cdr','adapted.cdr','registered.cdr'))
        total = sum(map(len, payload.values()))
        if auxiliary_size > CAPS['auxiliary_bytes'] or total > CAPS['group_bytes']:
            raise ValueError('whole group/auxiliary hard admission bound')
        current = self.usage()
        rounded = sum(((len(v)+self.block-1)//self.block)*self.block for v in payload.values())+2*self.block
        # Reserve the shared1MiB AND3MiB padding/staging margin. Temporary group
        # is renamed, never duplicated, and every file is preflighted as a whole.
        if max(current.values())+max(total,rounded)+4*MIB > CAPS['total_bytes']:
            raise ValueError('logical/allocated/staging total capture hard cap')
        stage = self.root/('.'+role+'_'+str(source)+'.staging')
        stage.mkdir()
        for name, data in payload.items():
            # No optimistic partial oversize writes. Any external/I/O failure
            # leaves diagnostic partial evidence and shuts off further capture.
            if max(self.usage().values())+((len(data)+self.block-1)//self.block)*self.block+4*MIB > CAPS['total_bytes']:
                raise ValueError('prewrite capture hard cap')
            with (stage/name).open('xb') as f:
                f.write(data)
        final = self.root/(role+'_'+str(source)); stage.rename(final)
        with self.lock:
            self.saved[role] = dict(source_ns=source, path=str(final), bytes=total,
                GROUP_sha256=sha(body), association=request['event'].get('association'),
                stage_geometry_verified=stage_error is None and bool(snap.get('provenance')),
                real_scan_interval_verified=instant,
                source_IMU_interval_enclosed=aux_meta['imu_source']['scan_interval_enclosed'],
                relay_IMU_interval_enclosed=aux_meta['imu_relay']['scan_interval_enclosed'],
                explicit_message_commit_token=False)
            self._event('group_saved', role=role, source_ns=source)

    def usage(self):
        paths = [self.root, *self.root.rglob('*')]
        return dict(logical_bytes=sum(p.stat().st_size for p in paths if p.is_file()),
            allocated_bytes=sum(p.stat().st_blocks*512 for p in paths))

    def receipt(self, final=False, writer_drained=False):
        with self.lock:
            missing = dict(self.missing)
            if final or self.expired or self.terminal:
                for role in GROUP_ROLES:
                    if role not in self.saved:
                        missing.setdefault(role, 'event/group not saved in bounded window')
            return dict(schema='known_scene_event_capture_receipt/v1', run_id=self.run.name,
                contract_sha256=self.contract_sha256, caps=CAPS, unchanged_gates=GATES,
                arm=self.arm, expired=self.expired, original_terminal=self.terminal,
                last_runtime_clock_ns=self.last_clock, selected=dict(self.selected), saved=dict(self.saved),
                missing=missing, rejected=list(self.rejected), bounded_events=list(self.events),
                actual_stale_event=self.c_event, actual_clear_event=self.clear_event,
                usage=self.usage(), final=final, writer_drained=writer_drained,
                matcher_worker_drained=self.matcher_worker_drained,
                ready_for_diagnostic_stop=(len(self.saved)==5 or self.expired),
                stop_hint_requires_original_safety_fault_priority=True,
                original_mission_completion_claimed=False, navigation_verified=False,
                formal_R6_capture_complete=False, general_localization_admission=False,
                quality_gates_changed=False, original_algorithms_changed=False,
                fixture=self.contract.get('fixture') is True)

    def _progress(self, final=False, drained=False):
        self.progress_dirty.clear()
        data = encoded(self.receipt(final, drained))
        if len(data) > 128*1024:
            raise ValueError('bounded progress receipt too large')
        path = self.root/'RECEIPT.json'; tmp = self.root/'RECEIPT.tmp'
        if max(self.usage().values())+((len(data)+self.block-1)//self.block)*self.block > CAPS['total_bytes']:
            raise ValueError('receipt/staging hard cap')
        with tmp.open('wb') as f:
            f.write(data)
        tmp.replace(path)

    def _process_one(self, item):
        try:
            self._write_group(item)
        except Exception as error:
            with self.lock:
                self.missing[item['role']] = type(error).__name__+':'+str(error)
                self.reject(item['snapshot']['source_ns'], 'whole_group_rejected:'+item['role']+':'+str(error))
                # A disk partial/write failure stops further writes. Pure
                # admission failures have no files and leave other roles viable.
                if any(p.name.endswith('.staging') for p in self.root.iterdir()):
                    self.write_failed = True
                    self.terminal = dict(stage='capture_partial_write_failure', observed_wall_ns=self.wall_ns())
        finally:
            self.jobs.task_done()

    def _writer(self):
        while not self.closed or not self.jobs.empty():
            try:
                item = self.jobs.get(timeout=.1)
            except queue.Empty:
                item = None
            if item:
                if not self.write_failed:
                    self._process_one(item)
                else:
                    self.jobs.task_done()
            if self.progress_dirty.is_set():
                try:
                    self._progress()
                except Exception as error:
                    self.reject(None, 'progress_write:'+str(error))
        try:
            self._progress(final=True, drained=True)
        except Exception:
            pass

    def drain_fixture(self):
        """No thread/ROS fixture path; never called by production hooks."""
        if self.writer is not None or self.contract.get('fixture') is not True:
            raise ValueError('fixture-only deterministic drain')
        while not self.jobs.empty():
            self._process_one(self.jobs.get_nowait())
        self._progress()

    def close(self, worker_drained=True):
        with self.lock:
            self.matcher_worker_drained = bool(worker_drained)
            self.closed = True
        if self.writer is not None:
            self.writer.join(timeout=2.)
            clean = not self.writer.is_alive()
        else:
            clean = self.jobs.empty()
            self._progress(final=True, drained=clean)
        result = self.receipt(final=clean, writer_drained=clean)
        return result


def diagnostic_stop_hint(run, clock_ns):
    """Call ONLY after original native/mission/navigation fault and cap checks.

    The helper remains an observer. Its owner performs existing owned cleanup.
    No signal, command, timer, ROS action or success receipt is emitted here.
    """
    run = Path(run)
    path = run/'known_scene_event_frames/RECEIPT.json'
    if not path.exists():
        return None
    try:
        if path.stat().st_size > 128*1024:
            raise ValueError('event receipt hard bound')
        raw = path.read_bytes(); d = json.loads(raw)
        contract_path = run/'known_scene_event_capture_contract.json'
        if contract_path.stat().st_size > 64*1024:
            raise ValueError('contract hard bound')
        contract = json.loads(contract_path.read_text())
        if (d.get('schema') != 'known_scene_event_capture_receipt/v1'
                or d.get('run_id') != run.name or contract.get('run_id') != run.name
                or d.get('contract_sha256') != sha(encoded(contract))
                or d.get('caps') != CAPS or d.get('unchanged_gates') != GATES
                or d.get('navigation_verified') is not False
                or d.get('original_mission_completion_claimed') is not False):
            raise ValueError('event receipt identity/contract/nonPASS fields')
        arm = d.get('arm')
        if arm is None:
            return None
        if (arm.get('stage') not in PHASES
                or not arm.get('request_id', '').startswith(run.name+':'+PHASES[arm['stage']]+':')
                or arm.get('trigger') != 'sustained_original_quality_rejection_or_anchor_expiry_lead'
                or type(arm.get('runtime_clock_ns')) is not int
                or not 0 < arm['runtime_clock_ns'] <= clock_ns):
            raise ValueError('actual phase arm binding')
        if max(d['usage'].values()) > CAPS['total_bytes']:
            raise ValueError('capture usage hard cap')
        saved = d.get('saved', {})
        if set(saved) == set(GROUP_ROLES):
            for role, info in saved.items():
                group = Path(info['path'])
                if group.parent != path.parent or group.name != role+'_'+str(info['source_ns']):
                    raise ValueError('exclusive group path')
                meta_path = group/'GROUP.json'
                if meta_path.stat().st_size > CAPS['auxiliary_bytes']:
                    raise ValueError('whole group metadata bound')
                data = meta_path.read_bytes()
                meta = json.loads(data)
                if (sha(data) != info['GROUP_sha256'] or meta.get('run_id') != run.name
                        or meta.get('role') != role or meta.get('source_ns') != info['source_ns']
                        or meta.get('diagnostic_only') is not True
                        or meta.get('contract_sha256') != d['contract_sha256']
                        or meta.get('full_source_messages_preserved') is not True
                        or meta.get('source_arrays_truncated') is not False):
                    raise ValueError('saved complete group metadata binding')
            return dict(stop=True, reason='five_event_groups_saved_diagnostic_only',
                event_capture_receipt=str(path), event_capture_receipt_sha256=sha(raw),
                navigation_PASS=False, full46_or_parking_verified=False,
                original_mission_completion_claimed=False, source_contract_missing=d.get('missing'))
        if clock_ns-arm['runtime_clock_ns'] >= CAPS['event_window_ns']:
            return dict(stop=True, reason='actual_phase_rejection_event_window90s_cutoff_diagnostic_only',
                event_capture_receipt=str(path), event_capture_receipt_sha256=sha(raw),
                navigation_PASS=False, full46_or_parking_verified=False,
                original_mission_completion_claimed=False, source_contract_missing=d.get('missing'))
        return None
    except (OSError, ValueError, TypeError, KeyError) as error:
        return dict(stop=True, reason='event_capture_diagnostic_receipt_invalid',
            diagnostic_error=type(error).__name__+':'+str(error), navigation_PASS=False)
