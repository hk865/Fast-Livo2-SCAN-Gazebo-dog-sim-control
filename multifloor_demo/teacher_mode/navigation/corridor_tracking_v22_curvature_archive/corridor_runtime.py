"""Bounded, read-only corridor observation off the ROS control callback.

One immutable job may be in flight. Busy diagnostic snapshots are counted and
not queued; this never drops SLAM inputs or changes actuator commands. Runtime
certificates remain explicitly shadow observations until separately enabled.
"""
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import time


MAX_SNAPSHOT_BYTES=2_000_000


def decode_snapshot(raw):
    if not isinstance(raw,str) or len(raw.encode('utf-8'))>MAX_SNAPSHOT_BYTES:
        raise ValueError('snapshot exceeds bounded diagnostic message size')
    def reject(value):raise ValueError('nonfinite snapshot JSON')
    try:d=json.loads(raw,parse_constant=reject)
    except (RecursionError,OverflowError) as error:raise ValueError('invalid nested snapshot') from error
    pending=[(d,0)]
    while pending:
        value,depth=pending.pop()
        if depth>24:raise ValueError('snapshot nesting exceeds bound')
        if isinstance(value,float) and not math.isfinite(value):raise ValueError('nonfinite snapshot number')
        if isinstance(value,dict):pending.extend((v,depth+1) for v in value.values())
        elif isinstance(value,list):pending.extend((v,depth+1) for v in value)
    if not isinstance(d,dict) or d.get('schema')!='teacher_scan_local_snapshot/v1':
        raise ValueError('wrong snapshot schema')
    if d.get('frame_id')!='camera_init':raise ValueError('wrong snapshot frame')
    if d.get('complete') is False:return d
    shape=d.get('shape_xyz')
    if (not isinstance(shape,list) or len(shape)!=3 or
        any(type(v) is not int or not 0<v<=256 for v in shape) or math.prod(shape)>65536):
        raise ValueError('invalid bounded snapshot shape')
    states=d.get('states')
    if not isinstance(states,str) or len(states)!=math.prod(shape) or set(states)-set('012'):
        raise ValueError('invalid tri-state snapshot payload')
    points=d.get('surface_points_xyz',[])
    if not isinstance(points,list) or len(points)>12000:
        raise ValueError('snapshot surface exceeds point bound')
    return d


class CorridorObserver:
    def __init__(self,limits,worker=None):
        self.limits=copy.deepcopy(limits)
        if worker is None:
            from corridor import certify_corridor
            worker=certify_corridor
        self.worker=worker
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='corridor-shadow')
        self.future=None
        self.closed=False
        self.submitted=0
        self.busy_snapshots=0
        self.completed=0
        self.last_revision=None

    def submit(self,snapshot,path,state,now_ns,received_wall_ns,snapshot_sha256):
        if self.closed:return False
        if self.future is not None:
            self.busy_snapshots+=1
            return False
        revision=snapshot.get('revision')
        if type(revision) is not int or revision<0:raise ValueError('invalid map revision')
        if self.last_revision is not None and revision<=self.last_revision:
            raise ValueError('non-increasing map revision')
        self.last_revision=revision
        self.submitted+=1
        # Inputs are owned copies, so neither callbacks nor future jobs can
        # change the exact path/map/pose used by this check.
        args=copy.deepcopy((snapshot,path,state,self.limits))
        sequence=self.submitted
        observation=copy.deepcopy(dict(sequence=sequence,snapshot_sha256=snapshot_sha256,
            map_revision=revision,snapshot_received_wall_ns=received_wall_ns,
            requested_clock_ns=now_ns,path_id=path.get('path_id'),
            path_sha256=path.get('path_sha256'),frame_id=path.get('frame_id'),
            layer_id=path.get('layer_id'),source_pose_stamp_ns=state.get('stamp_ns'),
            progress_m=state.get('progress_m'),position_world_xyz=state.get('position_world_xyz')))
        def evaluate():
            started=time.monotonic_ns()
            try:
                result=self.worker(args[1],args[0],None,args[2],args[3],now_ns)
                if not isinstance(result,dict):raise ValueError('certificate result must be a dictionary')
                record=dict(result,shadow_only=True,control_authority=False,
                observation=dict(observation,
                    compute_started_wall_ns=started,compute_finished_wall_ns=time.monotonic_ns()),
                navigation_ground_truth_used=False)
                json.dumps(record,allow_nan=False)
                return record
            except Exception as error:
                return dict(schema='teacher_corridor_certificate/v1',status='unavailable',
                    reason='certificate_compute_error:'+type(error).__name__+':'+str(error),
                    shadow_only=True,control_authority=False,navigation_ground_truth_used=False,
                    observation=dict(observation,compute_started_wall_ns=started,
                        compute_finished_wall_ns=time.monotonic_ns()))
        self.future=self.executor.submit(evaluate)
        return True

    def poll(self):
        if self.future is None or not self.future.done():return None
        future=self.future;self.future=None;self.completed+=1
        try:return future.result()
        except Exception as error:
            return dict(schema='teacher_corridor_certificate/v1',status='unavailable',
                reason='diagnostic_future_error:'+type(error).__name__+':'+str(error),
                shadow_only=True,control_authority=False,navigation_ground_truth_used=False)

    def close(self):
        self.closed=True
        self.executor.shutdown(wait=True,cancel_futures=False)
        return self.poll()


def sha_text(raw):return hashlib.sha256(raw.encode('utf-8')).hexdigest()
