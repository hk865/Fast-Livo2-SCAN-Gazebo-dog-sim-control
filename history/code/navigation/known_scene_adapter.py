"""Exact-source joining and a bounded background worker for scene matching.

Importing this module creates no ROS node, thread, writer, or simulator action.
The integration owner supplies the runtime witness and starts/polls the worker.
No transform is automatically published or adopted by navigation.
"""
from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
import hashlib
import json
import threading
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from known_scene_matcher import checked_transform


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def pointcloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    xyz = np.empty((msg.height * msg.width, 3), dtype=float)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('truncated pointcloud')
    for index, name in enumerate(('x', 'y', 'z')):
        field = fields[name]
        if field.datatype != 7 or field.count != 1 or field.offset + 4 > msg.point_step:
            raise ValueError('XYZ must be scalar float32')
        xyz[:, index] = np.ndarray((msg.height, msg.width), dtype=endian + 'f4',
                                  buffer=msg.data, offset=field.offset,
                                  strides=(msg.row_step, msg.point_step)).reshape(-1)
    return xyz


def point_records(msg):
    return np.ndarray((msg.height, msg.width), dtype='V' + str(msg.point_step),
                      buffer=msg.data,
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def odometry_transform(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    xyzw = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(xyzw).all() or abs(np.dot(xyzw, xyzw) - 1) > 1e-3:
        raise ValueError('invalid odometry quaternion')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(xyzw).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return checked_transform(t)


def validate_runtime_witness(witness, run_id):
    """Check receipts, not just boolean names. The owner records actual loading.

    A current file hash alone is NOT a loaded-library witness. Each producer
    receipt must bind a live/observed PID and /proc maps or loaded-executable
    observation. The owner's semantic review ties the GPU render snapshot to
    header time; 30Hz is never used to invent point acquisition offsets.
    """
    reasons = []
    if witness.get('schema') != 'go2_known_scene_runtime_witness/v1' or witness.get('run_id') != run_id:
        reasons.append('runtime_witness_identity')
    if witness.get('producer_model') != 'gz_gpu_lidar_instantaneous_header_snapshot':
        reasons.append('producer_time_model')
    for key in ('actual_loaded_libraries_verified', 'header_snapshot_time_verified',
                'post_lio_single_owner_publication_verified', 'no_robot_truth_pose'):
        if witness.get(key) is not True:
            reasons.append(key)
    producer = witness.get('producer_bindings', [])
    roles = {b.get('role') for b in producer}
    required = {'gz_sensors_gpu_lidar', 'gz_rendering', 'gz_sim_sensors', 'ros_gz_bridge'}
    if not required.issubset(roles):
        reasons.append('missing_loaded_producer_roles')
    bindings = producer + witness.get('stage_source_bindings', [])
    stage_roles = {b.get('role') for b in witness.get('stage_source_bindings', [])}
    if not {'LIVMapper', 'preprocess', 'odom_adapter', 'self_echo_filter'}.issubset(stage_roles):
        reasons.append('missing_stage_sources')
    for b in bindings:
        try:
            path = Path(b['path'])
            if path.stat().st_size > 128 * 1024 * 1024:
                raise ValueError('witness file exceeds bound')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != b['sha256']:
                raise ValueError('hash differs')
            if b in producer:
                if (type(b.get('observed_pid')) is not int or b['observed_pid'] <= 0
                        or b.get('load_observation') not in ('proc_maps', 'loaded_executable')
                        or not b.get('load_observation_sha256')):
                    raise ValueError('missing actual loading observation')
                observation = Path(b['load_observation_path']).read_bytes()
                if (len(observation) > 2 * 1024 * 1024
                        or hashlib.sha256(observation).hexdigest() != b['load_observation_sha256']
                        or str(path).encode() not in observation):
                    raise ValueError('loading observation not bound to this library')
        except (OSError, KeyError, TypeError, ValueError) as error:
            reasons.append('binding:' + str(b.get('role')) + ':' + str(error))
    return dict(valid=not reasons, reasons=reasons, run_id=run_id,
                witness_sha256=hashlib.sha256(json.dumps(witness, sort_keys=True,
                                                       separators=(',', ':')).encode()).hexdigest())


class SourceGeometryMismatch(ValueError):
    """Preserve the original veto while identifying the predicate it failed."""
    def __init__(self, detail, evidence=None):
        super().__init__('registered/source stage geometry inconsistent')
        self.detail = dict(detail)
        self.evidence = evidence  # diagnostic arrays, never a replacement veto


class ExactSourceJoin:
    ROLES = ('raw', 'adapted', 'registered', 'body', 'mapper')

    def __init__(self, witness_status, maximum_stamps=12, maximum_source_age_ns=3_000_000_000,
                 interval_ns=1_000_000_000, mount=(.2, 0., .1177)):
        self.witness = dict(witness_status)
        self.maximum_stamps = maximum_stamps
        self.maximum_age = maximum_source_age_ns
        self.interval_ns = interval_ns
        self.mount = np.array(mount, dtype=float)
        self.slots = OrderedDict()
        self.last_completed_ns = None
        self.latest_source_ns = None
        self.last_by_role = {}
        self.lock = threading.Lock()
        self.rejections = []

    def put(self, role, msg, received_wall_ns=None):
        if role not in self.ROLES:
            raise ValueError('unknown join role')
        ns = stamp_ns(msg)
        wall = time.monotonic_ns() if received_wall_ns is None else received_wall_ns
        with self.lock:
            last = self.last_by_role.get(role)
            if last is not None and ns <= last:
                self.rejections.append(dict(source_ns=ns, reason='duplicate_or_backward_' + role))
                self.rejections = self.rejections[-32:]
                return False
            self.last_by_role[role] = ns
            self.latest_source_ns = max(ns, self.latest_source_ns or ns)
            slot = self.slots.setdefault(ns, {})
            slot[role] = (msg, wall)
            for key in list(self.slots):
                if len(self.slots) > self.maximum_stamps or self.latest_source_ns - key > self.maximum_age:
                    del self.slots[key]
        return True

    def pop_latest(self, clock_ns, wall_ns=None):
        wall_ns = time.monotonic_ns() if wall_ns is None else wall_ns
        with self.lock:
            complete = [ns for ns, slot in self.slots.items()
                        if all(role in slot for role in self.ROLES)]
            if not complete:
                return None
            ns = max(complete)
            if self.last_completed_ns is not None and ns - self.last_completed_ns < self.interval_ns:
                return None
            slot = self.slots[ns]
            for key in list(self.slots):
                if key <= ns:
                    del self.slots[key]
            self.last_completed_ns = ns
        if not 0 <= clock_ns - ns <= self.maximum_age:
            return dict(source_ns=ns, rejected='source_age')
        if any(not 0 <= wall_ns - pair[1] <= self.maximum_age for pair in slot.values()):
            return dict(source_ns=ns, rejected='wall_age')
        try:
            return self._check(ns, slot)
        except (ValueError, KeyError, TypeError, AssertionError) as error:
            # Keep the legacy reason stable; a stage-labelled error alone does
            # not distinguish nearest-neighbour collisions from residuals.
            name = 'ValueError' if isinstance(error, SourceGeometryMismatch) else type(error).__name__
            result = dict(source_ns=ns, rejected=name + ':' + str(error))
            if isinstance(error, SourceGeometryMismatch):
                result['join_rejection_detail'] = dict(error.detail,
                    role_source_ns={role:stamp_ns(pair[0]) for role,pair in slot.items()},
                    original_received_wall_ns={role:pair[1] for role,pair in slot.items()},
                    guard_clock_ns=clock_ns, guard_wall_ns=wall_ns,
                    explicit_message_commit_token=False)
            if isinstance(error, SourceGeometryMismatch) and error.evidence is not None:
                result['_diagnostic_failed_source'] = dict(source_ns=ns,
                    source_messages={role:pair[0] for role,pair in slot.items()},
                    original_received_wall_ns={role:pair[1] for role,pair in slot.items()},
                    T_odom_body=error.evidence['T_odom_body'],
                    _diagnostic_evidence=error.evidence)
            return result

    def _check(self, ns, slot):
        check_started_wall_ns = time.monotonic_ns()
        raw, adapted, registered, body, mapper = (slot[role][0] for role in self.ROLES)
        if raw.header.frame_id != 'velodyne' or adapted.header.frame_id != 'velodyne':
            raise ValueError('unexpected source lidar frame')
        if body.header.frame_id != mapper.header.frame_id or registered.header.frame_id != body.header.frame_id:
            raise ValueError('original cloud/pose frame differs')
        if body.child_frame_id != 'demo_slam_body':
            raise ValueError('wrong body frame')
        t = odometry_transform(body)
        m = odometry_transform(mapper)
        if np.max(np.abs(m - t)) > 1e-10:
            raise ValueError('mapper/body stage pose differs')
        raw_fields = [(f.name, f.offset, f.datatype, f.count) for f in raw.fields]
        filtered_fields = [(f.name, f.offset, f.datatype, f.count) for f in adapted.fields]
        if raw_fields != filtered_fields or raw.point_step != adapted.point_step:
            raise ValueError('adapter changed source layout')
        if any(name.lower() in ('time', 't', 'timestamp', 'offset_time') for name, *_ in raw_fields):
            raise ValueError('per_point_time_requires_separate_verified_model')
        raw_records, adapted_records = point_records(raw), point_records(adapted)
        if not np.isin(adapted_records, np.unique(raw_records)).all():
            raise ValueError('adapted point is not intact source record')
        adapted_xyz = pointcloud_xyz(adapted)
        eligible = np.isfinite(adapted_xyz).all(1) & (np.linalg.norm(adapted_xyz, axis=1) >= .3)
        local = adapted_xyz[eligible] + self.mount
        full = pointcloud_xyz(registered)
        if len(local) != len(full) or not np.isfinite(full).all():
            raise ValueError('Generic/full registered point cardinality differs')
        reconstructed = (full - t[:3, 3]) @ t[:3, :3]
        distance, index = cKDTree(local).query(reconstructed, workers=1)
        # Actual frozen Generic deskew is retained. These bounds allow its
        # measured millimetre same-time discrepancy, not a different pose epoch.
        unique_count = len(np.unique(index))
        rms_m = float(np.sqrt(np.mean(distance ** 2)))
        maximum_m = float(np.max(distance))
        evidence = None
        if getattr(self, 'diagnostic_pin_enabled', False):
            evidence = dict(index=index, distance=distance, eligible=eligible,
                local=local, full=full, reconstructed=reconstructed, T_odom_body=t,
                check_started_wall_ns=check_started_wall_ns,
                nearest_completed_wall_ns=time.monotonic_ns())
        if (unique_count != len(index) or rms_m > .006 or maximum_m > .030):
            raise SourceGeometryMismatch(dict(
                schema='exact_source_join_original_geometry_veto/v1',
                source_points=len(local), registered_points=len(full),
                unique_nearest_source_indices=unique_count,
                nearest_source_collision_count=len(index)-unique_count,
                nearest_index_bijection_failed=unique_count != len(index),
                registered_source_rms_m=rms_m if np.isfinite(rms_m) else None,
                rms_nonfinite=not bool(np.isfinite(rms_m)), rms_limit_m=.006,
                rms_failed=rms_m > .006,
                registered_source_max_m=maximum_m if np.isfinite(maximum_m) else None,
                maximum_nonfinite=not bool(np.isfinite(maximum_m)), maximum_limit_m=.030,
                maximum_failed=maximum_m > .030,
                guard_predicates_changed=False), evidence=evidence)
        verified = self.witness.get('valid') is True
        provenance = dict(point_time_verified=verified,
                          post_lio_association_verified=verified,
                          source_hashes_verified=verified, no_robot_truth_pose=True,
                          stage='frozen_post_LIO_single_owner_publication',
                          explicit_message_commit_token=False,
                          integer_stamp_join=True,
                          witness_sha256=self.witness.get('witness_sha256'),
                          registered_source_rms_m=float(np.sqrt(np.mean(distance ** 2))),
                          registered_source_max_m=float(np.max(distance)),
                          source_membership_verified=True)
        checked_job = dict(source_ns=ns, body_points=local, T_odom_body=t,
                    registered_odom_xyz=full, provenance=provenance,
                    witness_status=self.witness,
                    original_received_wall_ns={role: slot[role][1] for role in self.ROLES},
                    source_messages={role:slot[role][0] for role in self.ROLES})
        if evidence is not None:
            checked_job['_diagnostic_evidence'] = evidence
        return checked_job


class MatcherWorker:
    """At most one in-flight fit and one replaceable pending job. No cloud archive.

    Node/control callbacks only enqueue/poll. Set BLAS/OMP threads to 1 before
    importing numpy in the integration launch. Do not call submit/match while
    holding the controller or command publication lock.
    """
    def __init__(self, matcher, diagnostic_capture=None):
        self.matcher = matcher
        self.diagnostic_capture=diagnostic_capture
        self.condition = threading.Condition()
        self.job = None
        self.result = None
        self.closed = False
        self.in_flight = False
        self.thread = threading.Thread(target=self._run, name='known-scene-match', daemon=True)
        self.thread.start()

    def submit(self, job, current_c):
        if job is None or job.get('rejected'):
            return False
        with self.condition:
            if self.closed:
                return False
            self.job = (job, checked_transform(current_c))
            self.condition.notify()
        return True

    def submit_join(self, join, clock_ns, current_c, parent_generation=None):
        """Defer even byte/layout/geometry verification to this worker."""
        with self.condition:
            if self.closed:
                return False
            self.job = (dict(join_request=join, clock_ns=clock_ns,
                             parent_generation=parent_generation), checked_transform(current_c))
            self.condition.notify()
        return True

    def poll(self):
        with self.condition:
            value, self.result = self.result, None
        return value

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.job is not None or self.closed)
                if self.closed:
                    return
                (job, c), self.job = self.job, None
                self.in_flight = True
            started = time.monotonic_ns()
            parent_generation = job.get('parent_generation')
            try:
                if 'join_request' in job:
                    join = job['join_request']
                    join.diagnostic_pin_enabled = bool(self.diagnostic_capture is not None and getattr(self.diagnostic_capture, 'capture_needed', False))
                    job = join.pop_latest(job['clock_ns'])
                if job is None:
                    with self.condition:
                        self.in_flight = False
                    continue
                if job.get('rejected'):
                    result = dict(source_ns=job['source_ns'], accepted=False, control_allowed=False,
                                  reason='exact_join:' + job['rejected'])
                    if 'join_rejection_detail' in job:
                        result['join_rejection_detail'] = job['join_rejection_detail']
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.consider_rejection(job,result,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'P1_rejection_capture:'+str(error))
                            except Exception:pass
                else:
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.observe_job(job,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'capture_pin_exception:'+str(error))
                            except Exception:pass
                    result = self.matcher.match(job['source_ns'], job['body_points'], job['T_odom_body'],
                                                current_c=c, provenance=job['provenance'])
                    result['T_odom_body']=checked_transform(job['T_odom_body']).tolist()
                    result['T_odom_body_source_ns']=job['source_ns']
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.consider(job,result,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'capture_exception:'+str(error))
                            except Exception:pass
            except Exception as error:
                result = dict(source_ns=None if job is None else job.get('source_ns'),
                              accepted=False, control_allowed=False,
                              reason='matcher_exception:' + type(error).__name__ + ':' + str(error))
            result['worker_started_wall_ns'] = started
            result['worker_completed_wall_ns'] = time.monotonic_ns()
            result['parent_generation'] = parent_generation
            with self.condition:
                self.result = result
                self.in_flight = False

    def close(self, timeout_s=2.5):
        with self.condition:
            self.closed = True
            self.job = None
            self.condition.notify_all()
        self.thread.join(timeout=timeout_s)
        return not self.thread.is_alive()
