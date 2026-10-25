"""Optional bounded observer after original matching; no localization decisions.

Only complete already-associated source messages enter the single writer lane.
The matcher callback retains thin references and uses nonblocking queue puts.
Original runtime adoption/freshness must be cross-checked in original logs.
"""
import hashlib
import json
import queue
import threading
import time
from pathlib import Path
from known_scene_adapter import stamp_ns

ROLES = ('raw', 'adapted', 'registered', 'body', 'mapper')


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class TemporalFrames:
    def __init__(self, run):
        self.run = Path(run)
        here = Path(__file__).resolve().parent
        self.contract = json.loads((here / 'KNOWN_SCENE_TEMPORAL_CAPTURE_CONTRACT.json').read_text())
        if (self.contract['schema'] != 'known_scene_baseline_temporal_capture/v1'
                or self.contract['required_run_label'] not in self.run.name
                or self.contract['original_matcher_sha256'] != sha((here / 'known_scene_matcher.py').read_bytes())
                or self.contract['maximum_groups'] != 4
                or self.contract['maximum_allocated_and_logical_bytes'] != 20 * 1024 * 1024
                or self.contract['original_source_freshness_ns'] != 3_000_000_000):
            raise ValueError('Wrong optional baseline observation contract')
        self.root = self.run / 'known_scene_temporal_frames'
        self.pending = queue.Queue(maxsize=4)
        self.lock = threading.Lock()
        self.last_qualified = None
        self.first_rejection = None
        self.anchor_ns = None
        self.last_seen_ns = None
        self.selected = set()
        self.role_coverage = set()
        self.events = []
        self.errors = []
        self.saved = []
        self.callback_max_ns = 0
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._write_loop, name='baseline-temporal-observer', daemon=True)
        self.thread.start()

    def error(self, reason, ns=None):
        with self.lock:
            self.errors.append(dict(source_ns=ns, reason=str(reason)[:512]))
            self.errors = self.errors[-8:]

    def consider(self, job, result, current_c, parent_generation):
        started = time.monotonic_ns()
        try:
            self._consider(job, result, parent_generation)
        except Exception as exc:
            self.error(type(exc).__name__ + ':' + str(exc), job.get('source_ns'))
        finally:
            with self.lock:
                self.callback_max_ns = max(self.callback_max_ns, time.monotonic_ns() - started)

    def _consider(self, job, result, parent_generation):
        ns = job.get('source_ns')
        if type(ns) is not int or type(parent_generation) is not int or self.closed.is_set():
            raise ValueError('Missing exact source or parent generation')
        if self.last_seen_ns is not None and ns <= self.last_seen_ns:
            raise ValueError('Repeated/backward actual job stamp')
        self.last_seen_ns = ns
        messages = job.get('source_messages', {})
        if set(messages) != set(ROLES) or any(stamp_ns(messages[r]) != ns for r in ROLES):
            raise ValueError('Incomplete/split-stage source group')
        if (len(messages['raw'].data) > self.contract['maximum_raw_CDR_bytes']
                or len(messages['adapted'].data) > self.contract['maximum_adapted_CDR_bytes']
                or len(messages['registered'].data) > self.contract['maximum_registered_CDR_bytes']):
            raise ValueError('Source payload too large to retain')
        provenance = job['provenance']
        if (provenance.get('integer_stamp_join') is not True
                or provenance.get('stage') != 'frozen_post_LIO_single_owner_publication'
                or provenance.get('source_membership_verified') is not True):
            raise ValueError('Original exact-source association evidence missing')
        candidates = result.get('candidates') or []
        qualified = (result.get('accepted') is True and result.get('evidence_verified') is True
                     and result.get('best', {}).get('quality_passed') is True
                     and result['best']['rays']['abs_p95_m'] <= result['policy']['ray_abs_p95_max_m'])
        # Stable top-level snapshot before worker timing/parent fields are added.
        # Source message objects are immutable original subscriber deliveries;
        # no body_points/registered numpy arrays or new sensor subscriptions.
        snapshot = dict(source_ns=ns, source_messages=dict(messages),
            original_received_wall_ns=dict(job['original_received_wall_ns']),
            provenance=dict(provenance), result_core=dict(result),
            parent_generation=parent_generation, observed_wall_ns=time.monotonic_ns())
        with self.lock:
            self.events.append(dict(source_ns=ns, accepted=result.get('accepted', False),
                evidence_verified=result.get('evidence_verified', False),
                original_full_ray_qualified=qualified, reason=result.get('reason'),
                parent_generation=parent_generation,
                top_rejections=candidates[0].get('rejections') if candidates else None))
            self.events = self.events[-128:]
        if self.first_rejection is None:
            if qualified:
                self.last_qualified = snapshot
            elif self.last_qualified is not None:
                anchor = self.last_qualified
                self.anchor_ns = anchor['source_ns']
                self.first_rejection = ns
                self._select(anchor, ['last_original_shadow_qualified_before_rejection'])
                roles = ['first_post_anchor_rejected_query']
                if ns - self.anchor_ns > self.contract['original_source_freshness_ns']:
                    roles.append('first_query_after_anchor_3s')
                self._select(snapshot, roles)
                self.last_qualified = None
            return
        if ns - self.first_rejection < self.contract['minimum_independent_source_interval_ns']:
            return
        roles = []
        if 'next_independent_query' not in self.role_coverage:
            roles.append('next_independent_query')
        if ns - self.anchor_ns > self.contract['original_source_freshness_ns'] and 'first_query_after_anchor_3s' not in self.role_coverage:
            roles.append('first_query_after_anchor_3s')
        if roles:
            self._select(snapshot, roles)

    def _select(self, snapshot, roles):
        ns = snapshot['source_ns']
        if ns in self.selected or len(self.selected) >= self.contract['maximum_groups']:
            raise ValueError('Duplicate or over-cap complete group')
        self.pending.put_nowait((snapshot, tuple(roles)))
        with self.lock:
            self.selected.add(ns)
            self.role_coverage.update(roles)

    def usage(self):
        paths = [self.root, *self.root.rglob('*')] if self.root.exists() else []
        return dict(logical_bytes=sum(p.stat().st_size for p in paths if p.is_file()),
                    allocated_bytes=sum(p.stat().st_blocks * 512 for p in paths))

    def _save(self, snapshot, roles):
        from rclpy.serialization import serialize_message, deserialize_message
        ns = snapshot['source_ns']
        messages = snapshot['source_messages']
        payload = {role: bytes(serialize_message(messages[role])) for role in ROLES}
        if any(deserialize_message(payload[r], type(messages[r])) != messages[r] for r in ROLES):
            raise ValueError('Lossless CDR roundtrip mismatch')
        c = self.contract
        if (len(payload['raw']) > c['maximum_raw_CDR_bytes']
                or len(payload['adapted']) > c['maximum_adapted_CDR_bytes']
                or len(payload['registered']) > c['maximum_registered_CDR_bytes']
                or len(payload['body']) + len(payload['mapper']) > c['maximum_pose_CDR_bytes']):
            raise ValueError('Complete group exceeds admission bound; no truncation')
        result = snapshot['result_core']
        core = {k: v for k, v in result.items() if k not in ('worker_started_wall_ns', 'worker_completed_wall_ns', 'parent_generation')}
        core_hash = sha(json.dumps(core, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())
        meta = dict(schema='baseline_temporal_complete_source_group/v1', source_ns=ns, roles=list(roles),
            original_received_wall_ns=snapshot['original_received_wall_ns'],
            observer_received_wall_ns=snapshot['observed_wall_ns'], provenance=snapshot['provenance'],
            input_C_world_odom=result['input_C_world_odom'], T_odom_body=result['T_odom_body'],
            T_odom_body_source_ns=result['T_odom_body_source_ns'], parent_generation=snapshot['parent_generation'],
            matcher_summary={k: result.get(k) for k in ('accepted', 'reason', 'evidence_verified', 'confirmations', 'T_world_body', 'C_world_odom', 'reference_sha256', 'policy')},
            matcher_core_result_sha256=core_hash, matcher_result_log_path=str(self.run / 'known_scene_matches.jsonl'),
            hash_projection='exclude_worker_started_wall_ns_worker_completed_wall_ns_parent_generation',
            original_reference_sha256=sha((self.run / 'known_scene_reference.json').read_bytes()),
            original_producer_witness_sha256=sha((self.run / 'known_scene_runtime_witness.json').read_bytes()),
            files={role: dict(path=role + '.cdr', bytes=len(p), sha256=sha(p)) for role, p in payload.items()},
            point_payload_sha256={r: sha(bytes(messages[r].data)) for r in ('raw', 'adapted', 'registered')},
            full_lossless_received_messages=True, serialized_roundtrip_fields_verified=True,
            runtime_adoption_unverified_until_original_bundle_decision_logs_checked=True,
            generation_and_applied_C_not_inferred_from_shadow_acceptance=True,
            IMU_full_messages_preserved=False, formal_R6_capture_complete=False,
            independent_observation_not_inferred_from_different_hash_only=True,
            dynamic_object_identity_proven=False, navigation_verified=False, robot_truth_pose_used=False)
        encoded = (json.dumps(meta, separators=(',', ':'), allow_nan=False) + '\n').encode()
        if len(encoded) > c['maximum_group_metadata_bytes']:
            raise ValueError('Metadata admission bound; no truncation')
        self.root.mkdir(exist_ok=True)
        allocated = sum(((len(p) + 4095) // 4096) * 4096 for p in [*payload.values(), encoded]) + 8192
        u = self.usage()
        if max(u.values()) + allocated + 256 * 1024 > c['maximum_allocated_and_logical_bytes']:
            raise ValueError('Whole group disk budget exhausted')
        group = self.root / str(ns)
        group.mkdir(exist_ok=False)
        for role, raw in payload.items():
            with (group / (role + '.cdr')).open('xb') as stream:
                stream.write(raw)
        with (group / 'GROUP.json').open('xb') as stream:
            stream.write(encoded)
        with self.lock:
            self.saved.append(dict(source_ns=ns, path=str(group), roles=list(roles), files=meta['files']))

    def progress(self, final=False):
        with self.lock:
            result = dict(schema='baseline_temporal_observer_receipt/v1', first_rejection_source_ns=self.first_rejection,
                anchor_source_ns=self.anchor_ns, saved=list(self.saved), selected_source_ns=sorted(self.selected),
                queued_role_coverage=sorted(self.role_coverage),
                saved_role_coverage=sorted({r for row in self.saved for r in row['roles']}),
                recent_original_match_events=list(self.events),
                errors=list(self.errors), matcher_callback_max_ns=self.callback_max_ns,
                final=final, writer_queue_drained=final and self.pending.empty(),
                writer_termination_requires_external_process_exit_check=True,
                maximum_groups=4, maximum_bytes=20 * 1024 * 1024, matcher_or_control_changed=False,
                actual_runtime_stale_or_adoption_requires_original_logs=True,
                no_fit_score_or_match_calls=True, IMU_full_messages_preserved=False,
                formal_R6_capture_complete=False, actual_navigation_verified=False)
        raw = (json.dumps(result, separators=(',', ':'), allow_nan=False) + '\n').encode()
        if len(raw) > self.contract['maximum_scalar_receipt_bytes']:
            raise ValueError('Scalar receipt admission bound')
        p = self.run / 'known_scene_temporal_progress.json'
        temporary = p.with_suffix('.tmp')
        temporary.write_bytes(raw)
        temporary.replace(p)

    def _write_loop(self):
        while not self.closed.is_set() or not self.pending.empty():
            try:
                snapshot, roles = self.pending.get(timeout=.25)
            except queue.Empty:
                snapshot = None
            try:
                if snapshot is not None:
                    self._save(snapshot, roles)
                self.progress()
            except Exception as exc:
                self.error(type(exc).__name__ + ':' + str(exc), None if snapshot is None else snapshot['source_ns'])
            finally:
                if snapshot is not None:
                    self.pending.task_done()
        self.last_qualified = None
        try:
            self.progress(final=True)
        except Exception as exc:
            self.error(type(exc).__name__ + ':' + str(exc))

    def close(self):
        self.closed.set()
        self.thread.join(timeout=2.)
        return not self.thread.is_alive()
