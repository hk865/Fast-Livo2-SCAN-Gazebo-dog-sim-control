"""One bounded diagnostic pin of an original exact-association veto.

Reads the existing worker-owned evidence; does not alter its result, index,
predicate, command, correction, or mission. No additional sensor subscription.
"""
from pathlib import Path
import hashlib,io,json,os,queue,threading,time,sys
import numpy as np
from p95_capture import P95SourceFrames

TOTAL_CAP=16*1024*1024
ARRAY_CAP=4*1024*1024
ROLE_CAPS=dict(raw=1024*1024,adapted=1024*1024,registered=2*1024*1024,
    body=64*1024,mapper=64*1024)
MAX_POINTS=32768
ROLES=('raw','adapted','registered','body','mapper')
def digest(data):return hashlib.sha256(data).hexdigest()
def encoded(value):return (json.dumps(value,ensure_ascii=False,allow_nan=False,sort_keys=True)+'\n').encode()
def stamp(message):return int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
def build_payload(source,result,serialize,deserialize,context):
    ns=source['source_ns'];messages=source['source_messages'];e=source['_diagnostic_evidence']
    if set(messages)!=set(ROLES)or any(stamp(m)!=ns for m in messages.values()):
        raise ValueError('five exact source stamps required')
    original=result['join_rejection_detail']
    if original.get('nearest_index_bijection_failed')is not True:raise ValueError('original collision veto required')
    n=len(e['index'])
    if not 0<n<=MAX_POINTS or n!=len(e['distance'])or any(e[k].shape!=(n,3)for k in ('local','full','reconstructed')):
        raise ValueError('complete original arrays exceed cardinality bound')
    payload={}
    for role in ROLES:
        message=messages[role];data=bytes(serialize(message))
        if len(data)>ROLE_CAPS[role]:raise ValueError('whole '+role+' CDR exceeds bound')
        if deserialize(data,type(message))!=message:raise ValueError('whole CDR roundtrip differs')
        payload[role+'.cdr']=data
    stream=io.BytesIO()
    np.savez(stream,original_source_index=e['index'],original_distance_m=e['distance'],
        original_eligible_mask=e['eligible'],original_local_body_xyz=e['local'],
        original_registered_odom_xyz=e['full'],original_reconstructed_body_xyz=e['reconstructed'],
        original_T_odom_body=e['T_odom_body'])
    payload['original_arrays.npz']=stream.getvalue()
    if len(payload['original_arrays.npz'])>ARRAY_CAP:raise ValueError('whole original array archive exceeds bound')
    layouts={}
    for role in ('raw','adapted','registered'):
        m=messages[role];layouts[role]=dict(frame_id=m.header.frame_id,width=int(m.width),height=int(m.height),
            point_step=int(m.point_step),row_step=int(m.row_step),bigendian=bool(m.is_bigendian),
            fields=[dict(name=f.name,offset=int(f.offset),datatype=int(f.datatype),count=int(f.count))for f in m.fields],
            payload_bytes=len(m.data),payload_sha256=digest(bytes(m.data)))
    metadata=dict(schema='first_original_association_veto_pin/v1',source_ns=ns,
        five_role_source_ns={k:stamp(m)for k,m in messages.items()},
        original_received_wall_ns=source['original_received_wall_ns'],original_veto=result,
        evaluated_start_wall_ns=e['check_started_wall_ns'],nearest_completed_wall_ns=e['nearest_completed_wall_ns'],
        cloud_layouts=layouts,original_array_dtypes={k:str(e[k].dtype)for k in ('index','distance','eligible','local','full','reconstructed','T_odom_body')},
        original_array_sha256={k:digest(np.ascontiguousarray(e[k]).tobytes())for k in ('index','distance','eligible','local','full','reconstructed','T_odom_body')},
        processing_stage='frozen_post_LIO_single_owner_publication',explicit_message_commit_token=False,
        stage_commit_identity='UNVERIFIED; exact original messages/stamps and frozen stage code preserved',
        no_nearest_pose_substitution=True,point_ID_fabricated=False,source_payload_truncated=False,
        serialized_representation='complete received ROS message CDR reserialization; not original transport wire bytes',
        diagnostic_only=True,original_gate_changed=False,original_NN_replaced=False,navigation_PASS=False,
        context=context,files={k:dict(bytes=len(v),sha256=digest(v))for k,v in payload.items()})
    payload['PIN.json']=encoded(metadata)
    if len(payload['PIN.json'])>64*1024:raise ValueError('whole pin metadata exceeds bound')
    return payload,metadata

class FirstCollisionPin:
    def __init__(self,run):
        self.run=Path(run);self.root=self.run/'first_association_veto';self.root.mkdir(exist_ok=False)
        self.block=max(4096,os.statvfs(self.root).f_frsize);self.lock=threading.Lock()
        self.jobs=queue.Queue(maxsize=1);self.done=threading.Event();self.selected=False;self.accepting=True
        self.last_receipt_error=None;self.close_timed_out=False
        self.receipt=dict(schema='first_original_association_veto_receipt/v1',run=str(self.run),
            status='not_triggered',total_hard_B=TOTAL_CAP,diagnostic_only=True,gate_changed=False,navigation_PASS=False)
        self.thread=threading.Thread(target=self._writer,name='first_collision_pin_writer',daemon=True);self.thread.start()
    def _receipt(self):
        data=encoded(self.receipt)
        if len(data)>64*1024:raise ValueError('pin receipt exceeds bound')
        tmp=self.root/'RECEIPT.tmp'
        with tmp.open('wb')as f:f.write(data)
        tmp.replace(self.root/'RECEIPT.json')
    def _safe_receipt(self):
        try:self._receipt()
        except Exception as error:self.last_receipt_error=type(error).__name__+': '+str(error)
    def consider(self,job,result,current_c,parent_generation):
        if job.get('join_rejection_detail',{}).get('nearest_index_bijection_failed')is not True:return
        with self.lock:
            if self.selected or not self.accepting or self.done.is_set():return
            self.selected=True
            source=job.get('_diagnostic_failed_source')
            self.jobs.put_nowait((source,dict(result),parent_generation))
    def _writer(self):
        self._safe_receipt()
        while not self.done.is_set()or not self.jobs.empty():
            try:item=self.jobs.get(timeout=.05)
            except queue.Empty:continue
            try:
                source,result,generation=item
                if source is None:
                    self.receipt.update(status='original_veto_missing_existing_pin',source_ns=result.get('source_ns'))
                    self._safe_receipt();continue
                from rclpy.serialization import serialize_message,deserialize_message
                import known_scene_adapter
                runtime=sys.modules['__main__']
                if Path(runtime.__file__).resolve()!=Path(__file__).with_name('known_scene_runtime.py').resolve():
                    raise ValueError('actual known-scene runtime module differs')
                modules={m.__name__:dict(path=str(Path(m.__file__).resolve()),sha256=digest(Path(m.__file__).read_bytes()))for m in (known_scene_adapter,runtime)}
                context=dict(parent_generation=generation,actual_loaded_modules=modules,
                    source_manifest_sha256=digest((self.run/'source_manifest.json').read_bytes()),
                    runtime_plan_sha256=digest((self.run/'runtime_plan.json').read_bytes()),
                    witness_path=str(self.run/'known_scene_runtime_witness.json'),
                    witness_sha256=digest((self.run/'known_scene_runtime_witness.json').read_bytes()))
                payload,metadata=build_payload(source,result,serialize_message,deserialize_message,context)
                logical=sum(map(len,payload.values()));allocated=sum(((len(v)+self.block-1)//self.block)*self.block for v in payload.values())+4*self.block+128*1024
                if max(logical,allocated)>TOTAL_CAP:raise ValueError('whole pin total cap; no truncation')
                stage=self.root/'.first.staging';stage.mkdir()
                for name,data in payload.items():
                    with (stage/name).open('xb')as f:f.write(data)
                target=self.root/('first_'+str(source['source_ns']));stage.rename(target)
                self.receipt.update(status='saved',source_ns=source['source_ns'],path=str(target),
                    logical_B=logical,allocated_upper_bound_B=allocated,files=metadata['files'],
                    PIN_sha256=digest(payload['PIN.json']),five_role_source_ns=metadata['five_role_source_ns'],explicit_message_commit_token=False)
                self._safe_receipt()
            except Exception as error:
                self.receipt.update(status='admission_or_write_failed',error=type(error).__name__+': '+str(error));self._safe_receipt()
            finally:self.jobs.task_done()
        self.receipt.update(writer_drained=True,final=True,close_join_timed_out=self.close_timed_out)
        self._safe_receipt()
    def close(self):
        with self.lock:self.accepting=False
        self.done.set();self.thread.join(timeout=2.)
        self.close_timed_out=self.thread.is_alive()

class CollisionObservedP95SourceFrames(P95SourceFrames):
    @property
    def first_collision_evidence_needed(self):
        pin=getattr(self,'first_collision_pin',None)
        return bool(pin is not None and pin.accepting and not pin.selected)
    def _observation_error(self,source,reason):
        try:self.reject(source,reason)
        except Exception:pass
    def __init__(self,run,**kwargs):
        super().__init__(run,**kwargs);self.first_collision_pin=None
        try:self.first_collision_pin=FirstCollisionPin(run)
        except Exception as error:self._observation_error(None,'bounded_collision_constructor:'+str(error))
    def consider_rejection(self,job,result,current_c,parent_generation):
        try:
            if self.first_collision_pin is not None:self.first_collision_pin.consider(job,result,current_c,parent_generation)
        except Exception as error:self._observation_error(job.get('source_ns'),'bounded_collision_observation:'+str(error))
        return super().consider_rejection(job,result,current_c,parent_generation)
    def close(self,worker_drained=True):
        result=super().close(worker_drained)
        try:
            if self.first_collision_pin is not None:self.first_collision_pin.close()
        except Exception as error:
            try:self.reject(None,'bounded_collision_close:'+str(error))
            except Exception:pass
        return result
