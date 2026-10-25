"""Zero-only native observation and event-bound Teacher recovery bookkeeping.

The original collision, stopping, heading and movement gates retain authority.
This module never computes a PID command or changes localization authorization.
"""
from collections import deque
from pathlib import Path
import copy
import hashlib
import json
import io
import os
import numpy as np

ARRAY_LIMIT_B = 48 * 1024**2
BUFFER_LIMIT_B = 64 * 1024**2
WINDOW_NS = 2_000_000_000
BINDING_KEYS = ('sequence', 'request_id', 'compute_ros_clock_ns',
    'control_pose_stamp_ns', 'cloud_header_stamp_ns', 'cloud_frame_id',
    'filtered_xyz_float64_sha256', 'trajectory_id', 'reference_stamp',
    'route_float64_sha256')


def _binding(row):
    d={k:copy.deepcopy(row[k]) for k in BINDING_KEYS}
    if d['reference_stamp'] is not None:d['reference_stamp']=list(d['reference_stamp'])
    return d


def _frame(node):
    d=node.known_scene_inputs
    return (d['generation'],d['C_world_odom'],d['T_navigation_world'])


def retain_native_context(node, args):
    """Retain the actual, already evaluated SCAN geometry, never invent a chord."""
    if not hasattr(node,'known_scene_inputs') or node.known_scene_inputs is None:
        return
    _,_,target,direction,route=args
    node._hold_native_context=dict(request_id=node.request_id,
        waypoint_index=node.waypoint_index,trajectory_id=node.active_trajectory_id,
        reference_stamp=tuple(node.active_reference_stamp or ()),
        frame=copy.deepcopy(_frame(node)),target=np.array(target,copy=True),
        direction=np.array(direction,copy=True),route=np.array(route,copy=True))


def observation_eligible(node, now, ns):
    """A committed frame may be observed while its match TTL forbids movement."""
    try:
        d,s,e=node.known_scene_inputs,node.known_scene_status,node.known_scene_scan_epoch
        c=node._hold_native_context;active=node.path_admission_active
        fresh=lambda t: isinstance(t,(int,float)) and np.isfinite(t) and 0<=now-t<.3
        return bool(node.state=='running' and not node.tilt_hold
            and not node.known_scene_transition and not node.path_admission_hold
            and d['control_allowed'] is False and d['correction_stale'] is True
            and d['correction_transition'] is False and s['correction_transition'] is False
            and s['initialized'] is True and e['ready'] is True
            and all(fresh(v['received_monotonic_wall']) for v in (d,s,e))
            and fresh(node.pose_updated) and fresh(node.cloud_updated)
            and node.raw_imu.fresh(now,ns/1e9)
            and node.bridge_safety.get('state')=='ready' and fresh(node.bridge_updated)
            and 0<=ns-node.pose_stamp<300_000_000
            and node.pose_stamp==node.cloud_stamp==d['source_ns']
            and d['generation']==s['generation']==e['generation']==node.known_scene_reference_generation
            and _frame(node)==node._last_authorized_hold_frame==c['frame']
            and node.samples is not None and node.segment_start is not None
            and c['request_id']==node.request_id and c['waypoint_index']==node.waypoint_index
            and c['trajectory_id']==node.active_trajectory_id
            and c['reference_stamp']==tuple(node.active_reference_stamp or ())
            and active is not None and active['identity']==node.admission_identity()
            and active['trajectory_id']==node.active_trajectory_id
            and tuple(active['active_reference_stamp'])==c['reference_stamp']
            and all(np.isfinite(c[k]).all() for k in ('target','direction','route'))
            and c['target'].shape==(3,) and c['direction'].shape==(2,)
            and c['route'].ndim==2 and c['route'].shape[1]==3
            and not any(node.command))
    except (AttributeError,KeyError,TypeError,ValueError):
        return False


def _file_sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(256*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def _array_key(binding):
    return str(binding['cloud_header_stamp_ns'])+'_'+binding['filtered_xyz_float64_sha256']


def _payload_bounds(arrays,manifest,block):
    # Full npy headers, manifest, filesystem block rounding and directory reserve.
    logical=0;allocated=4*block
    for cloud in arrays.values():
        header=io.BytesIO()
        np.lib.format.write_array_header_1_0(header,np.lib.format.header_data_from_array_1_0(cloud))
        size=len(header.getvalue())+cloud.nbytes
        logical+=size;allocated+=((size+block-1)//block)*block
    meta=len((json.dumps(manifest,allow_nan=False)+'\n').encode())+4096
    logical+=meta;allocated+=((meta+block-1)//block)*block
    return logical,allocated


class GuardReplayBuffer:
    """Complete two-second bindings with deduplicated retained array payloads.

    64MiB bounds private retained/copying array payload, not whole process RSS.
    The first blocked array is included even after it leaves the time window.
    Insufficient allowance fails explicitly; no required window is truncated.
    """
    def __init__(self,node):
        self.node=node;self.rows=deque();self.first=None;self.deadline=None;self.sealed=False
    def retained(self):
        items=list(self.rows)+([] if self.first is None else [self.first])
        return {_array_key(b):cloud for b,cloud in items}
    def remember(self,row):
        if self.sealed:return
        now=row['compute_ros_clock_ns']
        while self.rows and self.rows[0][0]['compute_ros_clock_ns']<now-WINDOW_NS:
            self.rows.popleft()
        if len(self.rows)>=512:raise ValueError('Complete native replay binding window exceeds allowance')
        binding=_binding(row);key=_array_key(binding);arrays=self.retained()
        source=np.asarray(self.node.guard_cloud)
        if source.dtype not in (np.dtype('<f4'),np.dtype('<f8')) or source.ndim!=2 or source.shape[1]!=3:
            raise ValueError('Actual native replay requires original float32/float64 XYZ values')
        # Native collision evaluation has already consumed this exact source.
        # Only the evidence copy follows the receipt's existing LE float64,
        # C-order hash format; never replace the live cloud or its input receipt.
        required=source.size*8
        normalize=source.dtype!=np.dtype('<f8') or not source.flags.c_contiguous
        allocation=required if normalize or key not in arrays else 0
        if required>BUFFER_LIMIT_B or sum(x.nbytes for x in arrays.values())+allocation>BUFFER_LIMIT_B:
            raise ValueError('Complete native replay array window exceeds allowance; no truncation')
        if not np.isfinite(source).all():
            raise ValueError('Actual native replay requires finite XYZ values')
        canonical=np.array(source,dtype='<f8',order='C',copy=True) if normalize else source
        if hashlib.sha256(memoryview(canonical.reshape(-1)).cast('B')).hexdigest()!=row['filtered_xyz_float64_sha256']:
            raise ValueError('Actual native replay cloud/hash differs')
        cloud=arrays.get(key)
        if cloud is None:
            cloud=canonical if normalize else np.array(source,dtype='<f8',order='C',copy=True)
            cloud.setflags(write=False)
        item=(binding,cloud);self.rows.append(item)
        if self.first is None and row['union_result']['blocked'] and not row['obstacle_hold_before']:
            self.first=item
        if self.deadline is not None and now>=self.deadline:
            self.persist();self.sealed=True
    def persist(self):
        if self.first is None:raise ValueError('Recovery lacks actual native blocked cloud')
        items=[self.first,*[x for x in self.rows if x[0]['request_id']==self.first[0]['request_id']]]
        arrays={};bindings={}
        for b,cloud in items:
            key=_array_key(b);arrays.setdefault(key,cloud)
            bindings[str(b['sequence'])]=dict(binding=b,array_file='r57_dynamic_guard_arrays/'+key+'.npy',array_sha256='0'*64)
        node=self.node;run=node.run;block=os.statvfs(run).f_frsize
        manifest=dict(schema='r57_actual_native_guard_payloads/v1',request_id=self.first[0]['request_id'],
            arrays_limit_B=ARRAY_LIMIT_B,retained_array_payload_limit_B=BUFFER_LIMIT_B,
            complete_window_not_byte_truncated=True,logical_B=ARRAY_LIMIT_B,projected_allocated_B=ARRAY_LIMIT_B,bindings=bindings)
        logical,allocated=_payload_bounds(arrays,manifest,block)
        if max(logical,allocated)>ARRAY_LIMIT_B:
            raise ValueError('Complete native replay logical/allocated payload allowance exceeded')
        manifest.update(logical_B=logical,projected_allocated_B=allocated)
        def archive():
            directory=run/'r57_dynamic_guard_arrays';directory.mkdir(exist_ok=False)
            for key,cloud in arrays.items():
                p=directory/(key+'.npy')
                if p.exists():raise ValueError('Native replay payload would overwrite evidence')
                np.save(p,cloud,allow_pickle=False)
            for item in bindings.values():item['array_sha256']=_file_sha(run/item['array_file'])
            p=run/'r57_dynamic_guard_payloads.json'
            if p.exists():raise ValueError('Native replay manifest would overwrite evidence')
            data=json.dumps(manifest,allow_nan=False)+'\n'
            actual_logical=sum(x.stat().st_size for x in directory.iterdir())+len(data.encode())
            actual_allocated=directory.stat().st_blocks*512+sum(x.stat().st_blocks*512 for x in directory.iterdir())+((len(data.encode())+block-1)//block)*block
            if max(actual_logical,actual_allocated)>ARRAY_LIMIT_B:
                raise ValueError('Actual complete replay payload allocation exceeds allowance')
            pending=p.with_name(p.name+'.pending')
            with pending.open('x') as stream:
                stream.write(data);stream.flush();os.fsync(stream.fileno())
            if p.exists():raise ValueError('Native replay manifest would overwrite evidence')
            pending.replace(p)
        node.evidence.enqueue(archive)


def remember_native_guard(node):
    row=node.pending_guard_row
    if row is None:return
    # All phases and later events need their own token, independently of capture.
    if row['union_result']['blocked'] and not row['obstacle_hold_before']:
        node._r57_blocked_guard_token=dict(request_id=row['request_id'],waypoint_index=node.waypoint_index,
            sequence=row['sequence'],clock_ns=row['compute_ros_clock_ns'],
            cloud_source_ns=row['cloud_header_stamp_ns'],cloud_sha256=row['filtered_xyz_float64_sha256'],
            trajectory_id=row['trajectory_id'],reference_stamp=tuple(row['reference_stamp'] or ()),
            stop_count=node.obstacle_stops+1)
        node._teacher_recovery_token=None;node.obstacle_resume_pending=False
    legacy_capture = (':navigation_f1_f3:' in str(node.request_id) and node.profile.get('mission46_required') is True)
    from r85_native_guard_replay_capture import enabled as r85_capture_enabled
    if not legacy_capture and not r85_capture_enabled(node):return
    if not hasattr(node,'_r57_guard_buffer'):node._r57_guard_buffer=GuardReplayBuffer(node)
    node._r57_guard_buffer.remember(row)


def _current_event(node,token):
    current=getattr(node,'_r57_blocked_guard_token',None)
    return bool(token is not None and current is not None
        and token['request_id']==node.request_id and token['waypoint_index']==node.waypoint_index
        and token['stop_count']==node.obstacle_stops
        and node.guard_sequence>=token['sequence']
        and all(token[k]==v for k,v in current.items()))


def observe_zero_hold(node, now, ns):
    if not observation_eligible(node,now,ns):return False
    if now-node.last_obstacle_check<=.10:return False
    c=node._hold_native_context;target=c['target'];direction=c['direction'];route=c['route']
    # record_native_guard has no guard_completed/PID/heading authorization side effect.
    result=node.record_native_guard(node.cloud,node.pose,target,direction,route)
    remember_native_guard(node);node.obstacle_result=result;node.last_obstacle_check=now
    blocked,clearance,point_count=result;ros_now=ns/1e9
    if blocked:
        if not node.obstacle_hold:
            if node.cloud_input_context is None:raise ValueError('Native blocked event lacks cloud context')
            event=dict(cloud=node.cloud,pose=node.pose.copy(),rotation=node.rotation.copy(),
                tracking_pose=node.tracking_pose.copy(),checked_target=target.copy(),
                steering_direction=direction.copy(),route=route.copy(),
                segment_start=node.segment_start.copy(),goal=node.waypoints[node.waypoint_index].copy())
            meta=dict(guard_compute_stamp=ros_now,odom_stamp_ns=node.pose_stamp,
                cloud_input=node.cloud_input_context,guard_result=result,request_id=node.request_id,
                waypoint_index=node.waypoint_index,trajectory_id=node.active_trajectory_id,
                reference_stamp=node.active_reference_stamp,raw_imu_stamp=node.raw_imu.stamp,
                raw_imu_tilt=node.raw_imu.tilt,protect_result=False,tilt_hold=node.tilt_hold,
                tilt_source=node.tilt_source,execution_bridge_safety=copy.deepcopy(node.bridge_safety),
                obstacle_edge_stamp=ros_now,edge_request_id=node.request_id,
                edge_waypoint_index=node.waypoint_index,zero_command_stamp=node.last_command_time,
                zero_command=node.command.copy())
            node.obstacle_stops+=1;node.obstacle_hold_started_ros=ros_now
            node.last_obstacle_event_id=node.event_archive.enqueue(event,meta)
        node.obstacle_hold=True;node.obstacle_clear_since=None
        if node.min_obstacle_clearance is None or clearance<node.min_obstacle_clearance:
            node.min_obstacle_clearance=clearance
    elif node.obstacle_hold:
        # Observation may start the clear interval, but never release motion here.
        if node.heading_gate.stopped(ros_now,now):
            node.obstacle_clear_since=node.obstacle_clear_since or ros_now
        else:node.obstacle_clear_since=None
    row=node.pending_guard_row
    row.update(obstacle_hold_after=bool(node.obstacle_hold),obstacle_stops_after=int(node.obstacle_stops),clear_start_after_s=node.obstacle_clear_since,
        clear_elapsed_after_s=None if node.obstacle_clear_since is None else ros_now-node.obstacle_clear_since,
        after_control_ros_clock_ns=ns,command_after_control=list(node.command),
        zero_requested_after_control=True,command_ros_sim_time_s=node.last_command_time,
        state_after_control=node.state,obstacle_resumes_after=node.obstacle_resumes,
        command_source='actual sensor SLAM/SCAN controller; no mover or simulator pose navigation input',
        evidence_queue_error=node.evidence.error,localization_hold_observation_only=True,
        localization_generation=node.known_scene_inputs['generation'])
    node.evidence.append(node.run/'navigation_guard_history.jsonl',row);node.pending_guard_row=None
    return True


def arm_teacher_recovery(node):
    from transition_gate import TeacherHeadingGate
    if not isinstance(node.heading_gate,TeacherHeadingGate):return
    token=getattr(node,'_r57_blocked_guard_token',None)
    if not _current_event(node,token):
        raise ValueError('Teacher recovery lacks same-request actual blocked event')
    node._teacher_recovery_token=dict(token,phase='await_new_SCAN')


def start_teacher_recovery(node,yaw,heading):
    from transition_gate import TeacherHeadingGate
    if not isinstance(node.heading_gate,TeacherHeadingGate):return False
    token=getattr(node,'_teacher_recovery_token',None)
    if not node.obstacle_resume_pending:return False
    if not _current_event(node,token) or token.get('phase') not in ('await_new_SCAN','aligning'):
        raise ValueError('Teacher pending recovery event identity differs')
    if token['phase']=='await_new_SCAN':
        recovery_basis=None
        if getattr(node,'automatic_reference',None)is not None:
            # A persistent global definition is not republished for each local
            # replan. Use the already admitted local curve and current owner;
            # do not expand/hash/revalidate the global definition here.
            qualified=False
            try:
                active=node.path_admission_active;metadata=active['metadata'];receipt=node.path_receipt
                epoch=node.known_scene_scan_epoch;generation=node.known_scene_inputs['generation']
                stamp=tuple(node.active_reference_stamp or ());blocked_stamp=tuple(token['reference_stamp'])
                route_key=(metadata['route_id'],metadata['route_version'])
                definition=node.trajectory_association.route_definition
                physical=active['proof']['r85_physical'];certificate=physical['physical_certificate']
                physical_payload=physical['physical_payload'];payload=active['payload']
                conditions=(
                    metadata['schema']==3
                    and len(stamp)==len(blocked_stamp)==2
                    and all(type(x)is int for x in stamp+blocked_stamp)
                    and all(s[0]>=0 and 0<=s[1]<1_000_000_000 for s in (stamp,blocked_stamp)) and stamp>=blocked_stamp
                    and type(node.active_trajectory_id)is int and type(token['trajectory_id'])is int
                    and node.active_trajectory_id>token['trajectory_id']
                    and active['identity']==node.admission_identity()
                    and active['trajectory_id']==node.active_trajectory_id==payload['traj_id']==metadata['trajectory_id']
                    and metadata['trajectory_start_time']==payload['start_time']
                    and tuple(active['active_reference_stamp'])==stamp==tuple(metadata['reference_stamp'])
                    and type(route_key[0])is int and route_key[0]==stamp[0]*1_000_000_000+stamp[1]
                    and type(route_key[1])is int and route_key[1]>0 and route_key[1]==metadata['committed_reference_revision']
                    and node.trajectory_association.route_key==route_key
                    and isinstance(definition,dict) and (definition['route_id'],definition['route_version'])==route_key
                    and all(definition[k]==metadata[k] for k in ('run_id','reference_epoch','planner_process_id','localization_generation'))
                    and receipt['trajectory_id']==node.active_trajectory_id
                    and receipt['request_id']==node.request_id and receipt['waypoint_index']==node.waypoint_index
                    and receipt['frame_id']=='camera_init'
                    and not node.path_admission_hold and node.samples is not None
                    and metadata['run_id']==node.run.name and metadata['reference_epoch']==epoch['planner_epoch']
                    and metadata['planner_process_id']==epoch['planner_identity']['pid']
                    and metadata['localization_generation']==generation==active['correction_generation']
                    and tuple(node.automatic_reference.scope)==(epoch['planner_epoch'],epoch['planner_identity']['pid'],generation)
                    and active['proof']['allowed']is True
                    and physical['schema']=='r85_actual_scan_physical_execution_gate/v1' and physical['allowed']is True
                    and (physical['route_id'],physical['route_version'])==route_key
                    and certificate['schema']=='r85_continuous_scan_physical_certificate/v1'
                    and certificate['allowed']is True and certificate['status']=='CERTIFIED_STATIC_SPACE_ONLY'
                    and physical_payload['frame_id']=='camera_init' and physical_payload['run_id']==node.run.name
                    and physical_payload['request_id']==node.request_id and physical_payload['epoch']==epoch['planner_epoch']
                    and physical_payload['localization_generation']==generation
                    and all(physical_payload[k]==payload[k]for k in ('order','traj_id','start_time','pos_pts','knots','yaw_pts','yaw_dt')))
                if conditions:
                    source_gate=node.r85_physical_gate.fresh(node,active['anchor_binding'],generation)
                    context=source_gate.get('source_binding',{})
                    qualified=(source_gate.get('allowed')is True and all(physical['source_binding'][k]==context[k]
                        for k in ('run_id','request_id','reference_id','reference_file_sha256','route_id','route_version',
                                  'planner_epoch','planner_process_id','localization_generation')))
                    if qualified:recovery_basis=dict(recovery_basis='new_admitted_local_curve',
                        recovery_trajectory_id=node.active_trajectory_id,route_id=route_key[0],route_version=route_key[1])
            except (AttributeError,KeyError,TypeError,ValueError,IndexError,OverflowError,OSError):
                qualified=False
            if not qualified:raise ValueError('Teacher recovery requires a newly associated SCAN')
        elif tuple(node.active_reference_stamp or ())<=token['reference_stamp']:
            raise ValueError('Teacher recovery requires a newly associated SCAN')
        node.heading_gate.resume_after_stop(yaw,heading,node.obstacle_stopped_duration)
        token['phase']='aligning'
        if recovery_basis is not None:token.update(recovery_basis)
    return True


def complete_teacher_recovery(node):
    token=getattr(node,'_teacher_recovery_token',None)
    if not node.obstacle_resume_pending or token is None:return
    if (not _current_event(node,token) or token['phase']!='aligning'
            or node.heading_gate.phase!='drive' or node.obstacle_hold
            or node.obstacle_result[0] or node.alignment_hold or node.tilt_hold
            or node.path_admission_hold or node.samples is None
            or not node._known_generation_ready() or not any(node.command[:2])):return
    node.aligned_obstacle_resumes+=1;node.obstacle_resume_pending=False
    token['phase']='actual_drive_published'
    buffer=getattr(node,'_r57_guard_buffer',None)
    if buffer is not None and not buffer.sealed:buffer.deadline=node.get_clock().now().nanoseconds+500_000_000
    node.evidence.append(node.run/'r57_teacher_recovery_events.jsonl',dict(token,
        actual_command=list(node.command),native_guard_sequence=node.guard_sequence,
        source_ns=int(node.pose_stamp),generation=node.known_scene_inputs['generation'],
        active_reference_stamp=list(node.active_reference_stamp),navigation_ground_truth_used=False))


def replay_payload(run,row):
    """Resolve only exact actual guard bindings, retaining original geometry replay."""
    if row.get('cloud_array_file') is not None:return row['cloud_array_file']
    run=Path(run);d=json.loads((run/'r57_dynamic_guard_payloads.json').read_text())
    item=d['bindings'][str(row['sequence'])]
    if (d['schema']!='r57_actual_native_guard_payloads/v1' or d['request_id']!=row['request_id']
            or item['binding']!=_binding(row)):raise ValueError('Native replay manifest binding differs')
    p=(run/item['array_file']).resolve()
    if not p.is_relative_to(run.resolve()) or _file_sha(p)!=item['array_sha256']:
        raise ValueError('Native replay payload path/hash differs')
    return item['array_file']


def dynamic_payload_ready(run):
    """Publication readiness only; the original verifier owns all acceptance."""
    return (Path(run)/'r57_dynamic_guard_payloads.json').is_file()
