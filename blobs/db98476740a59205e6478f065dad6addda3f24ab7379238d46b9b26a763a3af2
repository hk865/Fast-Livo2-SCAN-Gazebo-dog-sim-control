"""Three whole original association-job groups; observer only, no admission.

Only the writer serializes; control callbacks pin O(1) references or small metadata.
The original evaluated NN arrays are retained, not substituted by writer replays.
Importing creates no thread, ROS node, runtime or file.
"""
from __future__ import annotations
import hashlib,io,json,math,os,queue,threading,time
from pathlib import Path
from collections import OrderedDict,deque
MIB=1024*1024
ROLES=('raw','adapted','registered','body','mapper')
GROUP_ROLES=('PRE','FAIL','POST')
CAPS=dict(groups=3,group_bytes=5*MIB,raw_bytes=MIB,adapted_bytes=MIB,
          registered_bytes=1536*1024,identity_bytes=MIB,auxiliary_bytes=512*1024,
          shared_bytes=MIB,total_bytes=16*MIB,event_window_ns=15_000_000_000,
          maximum_query_points=480*64)
GATES=dict(native_response_ns=200_000_000,source_and_SCAN_freshness_ns=300_000_000,
           correction_age_ns=3_000_000_000,allray_abs_p95_m=.35,foreground_fraction=.05,
           surface_p95_m=.12,stage_geometry_RMS_m=.006,stage_geometry_maximum_m=.030,
           stage_nearest_index_bijection=True,require_complete_imu=False)
TOPICS=dict(imu_source='/livox/imu',imu_relay='/demo/teacher/slam/imu',clock='/clock')
PHASES=dict(exploring='exploration',returning='return_origin',navigating='navigation_f1_f3')
TRIGGER=dict(original_predicate='nearest_index_bijection_failed',first_real_failure=True,
             complete_previous_jobs=1,post_original_checked_job=True)
def sha(data):return hashlib.sha256(data).hexdigest()
def file_sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def encoded(value):return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
def msg_stamp(msg,clock=False):
    s=msg.clock if clock else msg.header.stamp
    return int(s.sec)*1_000_000_000+int(s.nanosec)
def as_list(value):return value.tolist() if hasattr(value,'tolist') else value

class BoundedSourceFrames:
    def __init__(self,run,*,contract=None,serialize=None,deserialize=None,start_writer=True,wall_ns=time.monotonic_ns):
        self.run=Path(run);self.root=self.run/'known_scene_event_frames'
        self.contract_path=self.run/'known_scene_event_capture_contract.json';self.wall_ns=wall_ns
        if contract is None:
            if self.contract_path.stat().st_size>64*1024:raise ValueError('capture contract too large')
            contract=json.loads(self.contract_path.read_text())
        if (contract.get('schema')!='go2_P1_association_source_capture_contract/v1'
            or contract.get('run_id')!=self.run.name or contract.get('caps')!=CAPS
            or contract.get('unchanged_gates')!=GATES or contract.get('topics')!=TOPICS
            or contract.get('diagnostic_trigger')!=TRIGGER or contract.get('diagnostic_only') is not True):
            raise ValueError('exact frozen P1 contract required')
        self.contract=contract;self.contract_sha256=sha(encoded(contract));self.bindings=[]
        for item in contract.get('source_bindings',[]):
            if file_sha(item['path'])!=item['sha256']:raise ValueError('frozen P1 source differs')
            self.bindings.append(dict(item))
        if not self.bindings:raise ValueError('source bindings required')
        if self.root.exists():raise ValueError('exclusive new capture root required')
        self.root.mkdir();self.block=max(4096,os.statvfs(self.root).f_frsize)
        self.serialize,self.deserialize=serialize,deserialize
        self.lock=threading.RLock();self.jobs=queue.Queue(maxsize=3);self.writer=None
        self.closed=False;self.write_failed=False;self.matcher_worker_drained=None
        self.witness=None;self.witness_status=None;self.mission=None;self.previous=None
        self.inflight_source=None;self.arm=None;self.expired=False;self.terminal=None
        self.last_clock=0;self.last_metadata=None;self.metadata=OrderedDict()
        self.aux={k:deque(maxlen=400 if k!='clock' else 128) for k in TOPICS}
        self.aux_overflow={k:0 for k in TOPICS};self.selected={};self.saved={};self.missing={}
        self.rejected=deque(maxlen=32);self.events=deque(maxlen=64)
        self.progress_dirty=threading.Event();self.progress_dirty.set();self.capture_needed=True
        if start_writer:
            self.writer=threading.Thread(target=self._writer,name='bounded_source_writer',daemon=True);self.writer.start()

    def configure_witness(self,status,checker):
        if status.get('valid') is not True or status.get('run_id')!=self.run.name:raise ValueError('original runtime witness invalid')
        p=self.run/'known_scene_runtime_witness.json'
        if p.stat().st_size>64*1024:raise ValueError('witness too large')
        witness=json.loads(p.read_text())
        value=dict(path=str(p),sha256=file_sha(p),model=witness.get('producer_model'),
            header_snapshot_time_verified=witness.get('header_snapshot_time_verified') is True,
            post_lio_single_owner_publication_verified=witness.get('post_lio_single_owner_publication_verified') is True)
        with self.lock:self.witness=value;self.witness_status=dict(status)

    def _event(self,event_kind,**values):
        row=dict(event=event_kind,observed_wall_ns=self.wall_ns());row.update(values)
        self.events.append(row);self.progress_dirty.set()

    def reject(self,source,reason):
        with self.lock:
            self.rejected.append(dict(source_ns=source,reason=str(reason)[:512]));self.progress_dirty.set()
        return False

    def _clock(self,ns):
        self.last_clock=max(self.last_clock,int(ns))
        if self.arm and ns>=self.arm['runtime_clock_ns']+CAPS['event_window_ns']:
            if not self.expired:
                self.expired=True;self.capture_needed=False;self._event('P1_event_window_expired',runtime_clock_ns=int(ns))

    def observe_mission(self,text,runtime_clock_ns):
        try:
            if not isinstance(text,str) or len(text.encode())>128*1024:return False
            d=json.loads(text)
            if d.get('run_id')!=self.run.name or d.get('navigation_ground_truth_used') is not False:return False
            with self.lock:
                self._clock(runtime_clock_ns)
                self.mission=dict(stage=d.get('stage'),request_id=d.get('current_request'),sim_ns=d.get('sim_ns'),
                    payload_sha256=sha(text.encode()),observed_wall_ns=self.wall_ns())
                if d.get('stage') in ('failed','completed','stopped'):
                    self.terminal=dict(self.mission);self.capture_needed=False;self.progress_dirty.set()
            return True
        except Exception as e:return self.reject(None,'mission:'+str(e))

    def observe_aux(self,role,msg,runtime_clock_ns,received_wall_ns=None):
        if role not in TOPICS:return False
        ns=msg_stamp(msg,role=='clock');wall=self.wall_ns() if received_wall_ns is None else received_wall_ns
        with self.lock:
            self._clock(runtime_clock_ns)
            if self.closed or self.expired or self.terminal or not self.capture_needed:return False
            ring=self.aux[role]
            if ring and ns<=ring[-1]['source_ns']:return False
            if len(ring)==ring.maxlen:self.aux_overflow[role]+=1
            ring.append(dict(source_ns=ns,received_wall_ns=wall,message=msg));return True

    def observe_source(self,join,source_ns,runtime_clock_ns):
        # No join lock, payload traversal, serialization, hashes or directory IO.
        with self.lock:self._clock(runtime_clock_ns)
        return False

    def observe_metadata(self,metadata,runtime_clock_ns):
        if metadata.get('run_id')!=self.run.name or metadata.get('robot_truth_pose_used') is not False:return False
        ns=metadata.get('source_ns')
        if type(ns) is not int:return False
        with self.lock:
            self._clock(runtime_clock_ns);self.last_metadata=dict(metadata)
            self.metadata[ns]=dict(metadata)
            while len(self.metadata)>64:self.metadata.popitem(last=False)
        return True

    def observe_obstacle(self,text,runtime_clock_ns):return False

    def _snapshot(self,job,result,c,generation):
        return dict(source_ns=job['source_ns'],source_messages=dict(job['source_messages']),
            original_received_wall_ns=dict(job['original_received_wall_ns']),observed_complete_wall_ns=self.wall_ns(),
            runtime_clock_ns=self.last_clock,result=None if result is None else dict(result),
            input_C_world_odom=as_list(c),parent_generation=generation,provenance=job.get('provenance'),
            T_odom_body=as_list(job.get('T_odom_body')),association_evidence=job.get('_diagnostic_evidence'),
            original_join_rejection=job.get('original_join_rejection'),
            actual_anchor_metadata=self.metadata.get(job['source_ns']),source_checker=None,
            aux_at_source_observation=self._aux_snapshot(job['source_ns']))

    def observe_job(self,job,current_c,parent_generation):
        with self.lock:
            if self.closed or self.expired or self.terminal or not self.capture_needed:return False
            self.inflight_source=self._snapshot(job,None,current_c,parent_generation)
        return True

    def consider(self,job,result,current_c,parent_generation):
        with self.lock:
            if self.closed or self.expired or self.terminal or not self.capture_needed:return False
            snap=self.inflight_source if self.inflight_source and self.inflight_source['source_ns']==job['source_ns'] else self._snapshot(job,None,current_c,parent_generation)
            snap['result']=dict(result)
            if self.arm and job['source_ns']>self.arm['source_ns'] and 'POST' not in self.selected:
                self._select('POST',snap,dict(kind='first_successful_association_checked_job_after_collision',original_match_accepted=result.get('accepted')))
                self.capture_needed=False
            elif self.arm is None:self.previous=snap
            self.inflight_source=None
        return True

    def consider_rejection(self,job,result,current_c,parent_generation):
        detail=job.get('join_rejection_detail',{})
        if detail.get('nearest_index_bijection_failed') is not True:return False
        source=job.get('_diagnostic_failed_source')
        if source is None:return self.reject(job['source_ns'],'original_collision_missing_direct_source_pin')
        with self.lock:
            if self.arm:
                self._event('later_original_collision_not_archived',source_ns=job['source_ns'],original_detail=dict(detail))
                return False
            if self.closed or self.expired or self.terminal:return False
            ns=job['source_ns'];clock=detail.get('guard_clock_ns',self.last_clock)
            self._clock(clock)
            self.arm=dict(trigger='first_original_unique_source_collision',source_ns=ns,runtime_clock_ns=clock,
                original_detail=dict(detail),stage=(self.mission or {}).get('stage','UNVERIFIED'),request_id=(self.mission or {}).get('request_id'))
            snap=self._snapshot(source,result,current_c,parent_generation)
            snap['original_join_rejection']=dict(reason=job['rejected'],detail=dict(detail))
            if self.previous is not None and self.previous['source_ns']<ns:self._select('PRE',self.previous,dict(kind='last_original_checked_job_before_collision'))
            else:self.missing['PRE']='no previous original checked job available; no fabricated control frame'
            self._select('FAIL',snap,dict(kind='original_NN_unique_source_rejection',original_predicate=dict(detail)))
            self.previous=None;self._event('P1_original_collision_armed',source_ns=ns)
        return True

    def _aux_snapshot(self,source):
        out={}
        for role,ring in self.aux.items():
            before=[r for r in ring if r['source_ns']<=source];after=[r for r in ring if r['source_ns']>=source]
            rows=([before[-1]] if before else [])+([after[0]] if after else [])
            out[role]=list({r['source_ns']:dict(r) for r in rows}.values())
        return out

    def _select(self,role,snap,event):
        if role in self.selected or len(self.selected)>=3:return False
        request=dict(role=role,snapshot=dict(snap),event=dict(event),aux=snap['aux_at_source_observation'],
            mission=dict(self.mission or {}),witness=None if self.witness is None else dict(self.witness),
            witness_status=self.witness_status,auxiliary_ring_overflow=dict(self.aux_overflow))
        try:self.jobs.put_nowait(request)
        except queue.Full:
            self.missing[role]='bounded queue full; no replacement';return self.reject(snap['source_ns'],'queue_full:'+role)
        self.selected[role]=dict(source_ns=snap['source_ns'],event=dict(event));self._event('group_selected',role=role,source_ns=snap['source_ns'])
        return True

    def _association_payload(self,snap):
        import numpy as np
        evidence=snap.get('association_evidence')
        if not evidence:raise ValueError('direct original evaluated NN evidence missing')
        index=evidence['index'];distance=evidence['distance'];n=len(index)
        if n>CAPS['maximum_query_points'] or len(distance)!=n:raise ValueError('association cardinality bound')
        stream=io.BytesIO()
        np.savez(stream,original_source_index=index,original_distance_m=distance,
            original_target_index=np.arange(n,dtype=np.uint32),
            eligible_adapted_record_index=np.flatnonzero(evidence['eligible']).astype(np.uint32),
            original_T_odom_body=evidence['T_odom_body'])
        payload=stream.getvalue()
        if len(payload)>CAPS['identity_bytes']:raise ValueError('complete original NN relation exceeds bound')
        def array_hash(a):return sha(np.ascontiguousarray(a).tobytes())
        metadata=dict(schema='original_evaluated_source_association/v1',direct_evaluated_arrays=True,
            file='association.npz',source_index_dtype=str(index.dtype),distance_dtype=str(distance.dtype),
            source_count=n,registered_count=n,original_index_sha256=array_hash(index),
            original_distance_sha256=array_hash(distance),original_local_body_xyz_sha256=array_hash(evidence['local']),
            original_registered_odom_xyz_sha256=array_hash(evidence['full']),
            original_reconstructed_body_xyz_sha256=array_hash(evidence['reconstructed']),
            evaluated_start_wall_ns=evidence['check_started_wall_ns'],nearest_completed_wall_ns=evidence['nearest_completed_wall_ns'],
            original_rule='cKDTree(local).query(reconstructed,workers=1); unique index count=N; RMS<=.006m; maximum<=.030m',
            full_XYZ_reconstruction='full PointType CDR float32 fields->float64; (full-t[:3,3])@t[:3,:3], exact original t in relation',
            point_ID_or_explicit_producer_stage_commit_token='UNVERIFIED; raw records and actual source order preserved, no ID fabricated',
            original_gate_changed=False,writer_NN_replay_substituted=False,precision_reduced=False)
        return payload,metadata

    def receipt(self,final=False,writer_drained=False):
        # Directory traversal is outside callback-shared lock.
        usage=self.usage()
        with self.lock:
            missing=dict(self.missing)
            if final or self.expired or self.terminal:
                for role in GROUP_ROLES:
                    if role not in self.saved:missing.setdefault(role,'not saved in bounded P1 window')
            ready=('FAIL' in self.saved and 'POST' in self.saved and ('PRE' in self.saved or 'PRE' in missing))
            return dict(schema='known_scene_event_capture_receipt/v1',run_id=self.run.name,contract_sha256=self.contract_sha256,
                caps=CAPS,unchanged_gates=GATES,arm=self.arm,expired=self.expired,original_terminal=self.terminal,
                last_runtime_clock_ns=self.last_clock,selected=dict(self.selected),saved=dict(self.saved),missing=missing,
                rejected=list(self.rejected),bounded_events=list(self.events),usage=usage,final=final,writer_drained=writer_drained,
                matcher_worker_drained=self.matcher_worker_drained,ready_for_diagnostic_stop=ready or self.expired,
                original_mission_completion_claimed=False,navigation_verified=False,formal_R6_capture_complete=False,
                general_localization_admission=False,quality_gates_changed=False,original_algorithms_changed=False,
                fixture=self.contract.get('fixture') is True)

    def _progress(self,final=False,drained=False):
        self.progress_dirty.clear();data=encoded(self.receipt(final,drained))
        if len(data)>128*1024:raise ValueError('progress receipt too large')
        path=self.root/'RECEIPT.json';tmp=self.root/'RECEIPT.tmp'
        if max(self.usage().values())+((len(data)+self.block-1)//self.block)*self.block>CAPS['total_bytes']:raise ValueError('receipt cap')
        with tmp.open('wb') as f:f.write(data)
        tmp.replace(path)

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
        stage_error = snap.get('original_join_rejection')
        association_payload,association_meta = self._association_payload(snap)
        payload['association.npz'] = association_payload
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
        metadata = dict(association_evidence=association_meta, schema='known_scene_P1_complete_group/v1', run_id=self.run.name,
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
        auxiliary_size = sum(len(v) for k,v in payload.items() if k not in ('raw.cdr','adapted.cdr','registered.cdr','association.npz'))
        total = sum(map(len, payload.values()))
        if auxiliary_size > CAPS['auxiliary_bytes'] or total > CAPS['group_bytes']:
            raise ValueError('whole group/auxiliary hard admission bound')
        current = self.usage()
        rounded = sum(((len(v)+self.block-1)//self.block)*self.block for v in payload.values())+2*self.block
        # Reserve shared1MiB; enforce both logical and allocated whole group bounds. Temporary group
        # is renamed, never duplicated, and every file is preflighted as a whole.
        if max(total,rounded)>CAPS['group_bytes']:
            raise ValueError('allocated group hard cap')
        if max(current.values())+max(total,rounded)+CAPS['shared_bytes'] > CAPS['total_bytes']:
            raise ValueError('logical/allocated/staging total capture hard cap')
        stage = self.root/('.'+role+'_'+str(source)+'.staging')
        stage.mkdir()
        for name, data in payload.items():
            # No optimistic partial oversize writes. Any external/I/O failure
            # leaves diagnostic partial evidence and shuts off further capture.
            if max(self.usage().values())+((len(data)+self.block-1)//self.block)*self.block+CAPS['shared_bytes'] > CAPS['total_bytes']:
                raise ValueError('prewrite capture hard cap')
            with (stage/name).open('xb') as f:
                f.write(data)
        final = self.root/(role+'_'+str(source)); stage.rename(final)
        group_sha = sha(body)
        with self.lock:
            self.saved[role] = dict(source_ns=source, path=str(final), bytes=total,
                GROUP_sha256=group_sha, association=request['event'].get('association'),
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


    def _process_one(self, item):
        try:
            self._write_group(item)
        except Exception as error:
            try:
                partial = any(p.name.endswith('.staging') for p in self.root.iterdir())
            except OSError:
                partial = True
            with self.lock:
                self.missing[item['role']] = type(error).__name__+':'+str(error)
                self.reject(item['snapshot']['source_ns'], 'whole_group_rejected:'+item['role']+':'+str(error))
                # A disk partial/write failure stops further writes. Pure
                # admission failures have no files and leave other roles viable.
                if partial:
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


def diagnostic_stop_hint(run,clock_ns):
    run=Path(run);path=run/'known_scene_event_frames/RECEIPT.json'
    if not path.exists():return None
    if path.stat().st_size>128*1024:raise ValueError('P1 receipt size bound')
    raw=path.read_bytes();d=json.loads(raw);p=run/'known_scene_event_capture_contract.json'
    if p.stat().st_size>64*1024:raise ValueError('P1 contract size bound')
    contract=json.loads(p.read_text())
    if (d.get('run_id')!=run.name or contract.get('run_id')!=run.name
        or d.get('contract_sha256')!=sha(encoded(contract)) or d.get('caps')!=CAPS
        or d.get('unchanged_gates')!=GATES or d.get('navigation_verified') is not False
        or d.get('original_mission_completion_claimed') is not False):raise ValueError('P1 receipt identity')
    if max(d['usage'].values())>CAPS['total_bytes']:raise ValueError('P1 capture hard cap')
    arm=d.get('arm')
    if arm is None:return None
    if (arm.get('trigger')!='first_original_unique_source_collision'
        or arm.get('original_detail',{}).get('nearest_index_bijection_failed') is not True
        or type(arm.get('runtime_clock_ns')) is not int or arm['runtime_clock_ns']>clock_ns):raise ValueError('actual original P1 failure arm required')
    saved=d.get('saved',{})
    if 'FAIL' in saved and 'POST' in saved and ('PRE' in saved or 'PRE' in d.get('missing',{})):
        for role,info in saved.items():
            group=Path(info['path']);meta_path=group/'GROUP.json'
            if group.parent!=path.parent or group.name!=role+'_'+str(info['source_ns']):raise ValueError('exclusive P1 group path')
            if meta_path.stat().st_size>CAPS['auxiliary_bytes']:raise ValueError('P1 group meta cap')
            data=meta_path.read_bytes();meta=json.loads(data)
            if (sha(data)!=info['GROUP_sha256'] or meta.get('run_id')!=run.name or meta.get('role')!=role
                or meta.get('source_ns')!=info['source_ns'] or meta.get('full_source_messages_preserved') is not True
                or meta.get('source_arrays_truncated') is not False or meta.get('localization_admission') is not False
                or meta.get('association_evidence',{}).get('direct_evaluated_arrays') is not True):raise ValueError('complete original P1 binding')
        return dict(stop=True,reason='P1_original_failure_and_control_groups_saved_diagnostic_only',
            event_capture_receipt=str(path),event_capture_receipt_sha256=sha(raw),navigation_PASS=False,
            full46_or_parking_verified=False,original_mission_completion_claimed=False,source_contract_missing=d.get('missing'))
    if clock_ns-arm['runtime_clock_ns']>=CAPS['event_window_ns']:
        return dict(stop=True,reason='P1_original_collision_15s_window_cutoff_diagnostic_only',
            event_capture_receipt=str(path),event_capture_receipt_sha256=sha(raw),navigation_PASS=False,
            full46_or_parking_verified=False,original_mission_completion_claimed=False,source_contract_missing=d.get('missing'))
    return None
