"""At most two complete existing-source CDR groups for offline quality diagnosis.

No ROS initialization, new subscriber, pose lookup, array truncation or bag.
This does not implement or certify the independent R6 IMU sampling contract.
"""
import hashlib,json
from pathlib import Path
from known_scene_adapter import stamp_ns

class BoundedSourceFrames:
    def __init__(self,run,maximum_groups=2,maximum_bytes=8*1024*1024):
        if maximum_groups!=2 or maximum_bytes!=8*1024*1024:raise ValueError('Exact diagnostic8MiB contract required')
        self.run=Path(run);self.root=self.run/'known_scene_diagnostic_frames'
        self.maximum_groups=maximum_groups;self.maximum_bytes=maximum_bytes;self.saved=[];self.rejected=[]
    def consider(self,job,result,current_c,parent_generation):
        candidates=result.get('candidates') or []
        if len(self.saved)>=self.maximum_groups or not candidates or candidates[0].get('rejections')!=['ray_p95']:return False
        source=job.get('source_ns');messages=job.get('source_messages',{})
        if type(source) is not int or source<=0 or set(messages)!={'raw','adapted','registered','body','mapper'}:return self.reject(source,'incomplete_source_group')
        if any(stamp_ns(m)!=source for m in messages.values()):return self.reject(source,'source_stamp_mismatch')
        if self.saved and source-self.saved[-1]['source_ns']<150_000_000:return False
        from rclpy.serialization import serialize_message,deserialize_message
        payload={k:bytes(serialize_message(m)) for k,m in messages.items()}
        if any(deserialize_message(payload[k],type(m))!=m for k,m in messages.items()):return self.reject(source,'CDR_roundtrip_fields_mismatch')
        if any(len(payload[k])>1024*1024 for k in ('raw','adapted')):return self.reject(source,'whole_raw_adapted_CDR_1MiB_admission_bound')
        if len(payload['registered'])>1536*1024:return self.reject(source,'whole_registered_CDR_1point5MiB_admission_bound')
        if sum(len(payload[k]) for k in ('body','mapper'))>64*1024:return self.reject(source,'whole_pose_CDR_64KiB_bound')
        reference=self.run/'known_scene_reference.json';witness=self.run/'known_scene_runtime_witness.json'
        metadata=dict(schema='known_scene_complete_source_diagnostic_group/v1',source_ns=source,
            diagnostic_only=True,formal_R6_capture_complete=False,IMU_messages_preserved=False,
            original_source_received_wall_ns=job['original_received_wall_ns'],
            provenance=job['provenance'],T_odom_body=job['T_odom_body'].tolist(),input_C_world_odom=current_c.tolist(),
            parent_generation=parent_generation,
            matcher_core_result_sha256=hashlib.sha256(json.dumps(result,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),
            matcher_result_log_path=str(self.run/'known_scene_matches.jsonl'),matcher_log_source_ns=source,
            matcher_log_hash_projection='exclude_worker_started_wall_ns_worker_completed_wall_ns_parent_generation',
            matcher_top_candidate=candidates[0],matcher_policy=result.get('policy'),matcher_reason=result.get('reason'),
            reference_path=str(reference),reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
            producer_witness_path=str(witness),producer_witness_sha256=hashlib.sha256(witness.read_bytes()).hexdigest(),
            files={k:dict(path=k+'.cdr',bytes=len(v),sha256=hashlib.sha256(v).hexdigest()) for k,v in payload.items()},
            capture_representation='complete_lossless_received_ROS_message_reserialization_not_wire_CDR',
            serialized_message_roundtrip_fields_verified=True,all_match_candidates_preserved_in_original_log=True,
            point_payload_sha256={k:hashlib.sha256(bytes(messages[k].data)).hexdigest() for k in ('raw','adapted','registered')},
            raw_adapted_full_messages=True,registered_RGB_layout_preserved=True,source_arrays_truncated=False,
            robot_truth_pose_used=False)
        encoded=(json.dumps(metadata,allow_nan=False,separators=(',',':'))+'\n').encode()
        if len(encoded)>128*1024:return self.reject(source,'metadata_128KiB_bound')
        # Each accepted complete group <=3.6875MiB; both plus block rounding and
        # directory overhead remain <8MiB. Never start a partial oversize group.
        current=self.usage() if self.root.exists() else dict(logical_bytes=0,allocated_bytes=0)
        rounded=sum(((len(x)+4095)//4096)*4096 for x in [*payload.values(),encoded])+8192
        if max(current['logical_bytes'],current['allocated_bytes'])+rounded+128*1024>self.maximum_bytes:return self.reject(source,'group_disk_bound')
        group=self.root/str(source);group.mkdir(parents=True,exist_ok=False)
        for k,v in payload.items():
            with (group/(k+'.cdr')).open('xb') as f:f.write(v)
        with (group/'GROUP.json').open('xb') as f:f.write(encoded)
        self.saved.append(dict(source_ns=source,path=str(group),files=metadata['files']))
        return True
    def reject(self,source,reason):
        self.rejected.append(dict(source_ns=source,reason=reason));self.rejected=self.rejected[-8:];return False
    def usage(self):
        paths=[self.root,*self.root.rglob('*')] if self.root.exists() else []
        return dict(logical_bytes=sum(p.stat().st_size for p in paths if p.is_file()),allocated_bytes=sum(p.stat().st_blocks*512 for p in paths))
    def close(self):
        usage=self.usage();receipt=dict(schema='known_scene_diagnostic8MiB_receipt/v1',saved=self.saved,rejected=self.rejected,
            maximum_groups=self.maximum_groups,maximum_bytes=self.maximum_bytes,usage=usage,final=True,worker_drained=True,
            budget_pass=max(usage.values())<=self.maximum_bytes,formal_R6_capture_complete=False,
            navigation_verified=False,quality_gates_changed=False)
        p=self.run/'known_scene_diagnostic_capture_receipt.json';p.write_text(json.dumps(receipt,indent=2)+'\n')
        return receipt
