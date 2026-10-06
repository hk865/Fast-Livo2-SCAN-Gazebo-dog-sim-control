#!/usr/bin/env python3
"""Observe the frozen evaluator's JSONL iteration, without modifying its checks.

Only one full data pass per four large JSONL files. Compact diagnostic records
are copies; the exact parsed original rows are yielded to the original reader.
This is offline evidence collection, not controller or physics replay.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

HERE = Path(__file__).resolve().parent

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()

def select(row,keys):return {k:row.get(k) for k in keys.split()}

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();run=a.run.resolve();prefix=run.name
    report_path=HERE/(prefix+'_FULL46_ACTUAL_EVALUATION.json')
    compact_path=HERE/(prefix+'_COMPACT_DIAGNOSTIC.json.gz')
    if report_path.exists() or compact_path.exists():raise ValueError('Refusing to overwrite evidence')
    spec=importlib.util.spec_from_file_location('dc50_adapter',HERE/'evaluate_full46_v20.py')
    adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
    original_load=adapter.load_adapter
    bindings={};series={};passes={};plans={};start=time.monotonic()
    def rows(path):
        path=Path(path);name=path.name;passes[name]=passes.get(name,0)+1
        if passes[name]!=1:raise ValueError('Unexpected duplicate full read: '+name)
        out=[];series[name]=out;digest=hashlib.sha256();offset=0;count=0
        before=path.stat()
        with path.open('rb') as f:
            for line_no,raw in enumerate(f,1):
                digest.update(raw)
                if not raw.strip():offset+=len(raw);continue
                row=json.loads(raw);count+=1
                provenance=dict(line=line_no,offset=offset,length=len(raw),sha256=hashlib.sha256(raw).hexdigest())
                if name=='telemetry.jsonl':
                    record=select(row,'sim_time world_sim_time state_physics_world_time command requested measured body_lin_vel body_ang_vel position quaternion_wxyz rpy contacts fault body_clearance state command_reason command_expired actor_inferred_this_frame outer_navigation_ground_truth_used')
                    env=row.get('closed_loop_read_evidence') or {}
                    record['read_evidence']=select(env,'reason rejected command sim_time physics_clock_ns')
                    # Parse the recorded envelope only in the failure window.
                    t=row.get('state_physics_world_time',-1)
                    if 170<=t<=190:
                        text=(env.get('actual_read_attempt') or {}).get('raw_utf8')
                        if text:
                            envelope=json.loads(text)
                            record['envelope']=select(envelope,'state healthy stop_requested reason requested command registered_route_fence controller_source controller_source_error sequence sim_time slam_stamp_ns')
                    if row.get('navigation_envelope'):record['navigation_envelope']=row['navigation_envelope']
                elif name=='navigation_pid_history.jsonl':
                    record=select(row,'sequence cascade_sequence control_pose_stamp_ns source_pose_stamp_ns paired_imu_stamp_ns control_stamp_ns compute_ros_clock_ns request_id waypoint_index trajectory_id trajectory_archive_file mode desired_body_command prepared_after_slew_command command_after_slew state obstacle_hold alignment_hold stopped reset_count_after_publish queue_error feedback cascade ack goal')
                    gate=row.get('heading_gate_reference') or {};steer=gate.get('steering') or {}
                    record['heading_gate_reference']=dict(phase=gate.get('phase'),locked_heading=gate.get('locked_heading'),steering=select(steer,'heading error stamp odom_stamp trajectory_id cascade_projection heading_reference'))
                    receipt=row.get('path_receipt') or {};pid=receipt.get('path_id')
                    if pid and pid not in plans:plans[pid]=receipt
                    record['path_id']=pid
                    record['imu']=select(row.get('imu') or {},'stamp_ns angular_velocity_body orientation_xyzw')
                elif name=='navigation_status.jsonl':
                    record=select(row,'ros_sim_time request_id state waypoint_index total message goal_activated_ros_clock_ns segment_start_pose_stamp_ns current_goal region_arrival_evidence region_arrivals accepted_trajectory_id trajectory_reference_stamp replans reference_requests pose tracking_pose command alignment_phase locked_heading heading_deviation_since teacher_transition steering execution_bridge_safety last_rejected last_spline_rejected corridor_runtime cascade_parking')
                else:record=dict(row)
                record['_source']=provenance;out.append(record);offset+=len(raw)
                yield row
        after=path.stat()
        if (before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_size,after.st_mtime_ns,after.st_ctime_ns):raise ValueError('Run changed during evaluation')
        bindings[name]=dict(file=str(path),sha256=digest.hexdigest(),bytes=offset,rows=count,full_data_passes=passes[name])
    selections=[]
    def load():
        audit=original_load();audit.rows=rows
        original_snapshot=audit.snapshot
        def snapshot(run,basename,required_sha=None):
            index=audit.read(run/'navigation_source_snapshots.json')
            source=str(adapter.V20/basename)
            if source not in index:return original_snapshot(run,basename,required_sha)
            row=index[source];path=Path(row['snapshot']).resolve()
            if not path.is_relative_to(run/'sources') or sha(path)!=row['sha256'] or (required_sha is not None and row['sha256']!=required_sha):
                raise ValueError('Exact V20 archived source/hash mismatch: '+basename)
            selections.append(dict(source=source,snapshot=str(path),sha256=row['sha256'],required_sha256=required_sha))
            return path
        audit.snapshot=snapshot
        return audit
    adapter.load_adapter=load
    if hasattr(os,'sched_setaffinity'):os.sched_setaffinity(0,{max(os.sched_getaffinity(0))})
    os.nice(15)
    try:report=adapter.evaluate(run)
    except Exception as e:
        report=dict(schema='dc50_evaluator_exception/v1',run=str(run),status='failed',passed=False,error=type(e).__name__+': '+str(e),checks={})
    with report_path.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    small={}
    for name in ('navigation_anchor.json','navigation_scene_axis_registration.json','mission46_status.json','runtime_manifest.json','navigation_profile.json','source_manifest.json','navigation_source_snapshots.json','navigation_status.json','sensor_contract.json'):
        path=run/name;raw=path.read_bytes();small[name]=json.loads(raw);bindings[name]=dict(file=str(path),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
    # First contract line only. Do not claim a 200 Hz actuator trace audit.
    with (run/'actuator.jsonl').open('rb') as f:contract_raw=f.readline()
    contract=json.loads(contract_raw)
    bindings['actuator_contract_first_line']=dict(file=str(run/'actuator.jsonl'),offset=0,length=len(contract_raw),sha256=hashlib.sha256(contract_raw).hexdigest())
    for name in ('navigation_segment_activation.jsonl','navigation_cascade_paths.jsonl'):
        if (run/name).is_file():list(rows(run/name))
    compact=dict(schema='dc50_full46_single_pass_diagnostic/v1',run=str(run),run_id=prefix,
        formal_report=dict(file=str(report_path),sha256=sha(report_path),status=report['status']),
        window_world_sim_s=[170,190],series=series,path_receipts=plans,metadata=small,
        actuator_contract=contract,source_bindings=bindings,full_data_passes=passes,
        script_sha256=sha(__file__),adapter_sha256=sha(HERE/'evaluate_full46_v20.py'),
        exact_V20_snapshot_selections=selections,
        collection_history=dict(prior_interrupted_attempt=True,reason='Prepared adapter basename-only lookup raised Ambiguous/foreign source cascade_core.py; first compact was not persisted. One necessary retry recorded, not claimed as a single overall read.'),
        elapsed_s=time.monotonic()-start,simulation_only=True,navigation_ground_truth_used=False,
        controller_replay_performed=False,full_200Hz_actuator_replay_performed=False)
    with compact_path.open('xb') as f:
        with gzip.GzipFile(fileobj=f,mode='wb',mtime=0) as z:z.write(json.dumps(compact,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode())
    print(json.dumps(dict(report=str(report_path),compact=str(compact_path),status=report['status'],
        functional46_limited_pass=report.get('functional46_limited_pass'),elapsed_s=time.monotonic()-start,
        full_data_passes=passes,checks={k:v['status']for k,v in report['checks'].items()}),indent=2))

if __name__=='__main__':main()
