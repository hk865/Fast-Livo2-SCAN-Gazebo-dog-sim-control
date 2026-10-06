#!/usr/bin/env python3
"""Offline operational health; logging latency is not exclusive logger cost."""
from __future__ import annotations
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import struct
import sys
import numpy as np
from verify_diagnostics import sha,MAGIC,HEADER


def rows(path):
    with Path(path).open()as stream:
        for line in stream:
            if line.strip():yield json.loads(line)


def distribution(values):
    values=np.asarray(values,float);values=values[np.isfinite(values)]
    if not len(values):return None
    return {'count':len(values),'min':float(values.min()),'mean':float(values.mean()),
        'p50':float(np.quantile(values,.5)),'p95':float(np.quantile(values,.95)),
        'p99':float(np.quantile(values,.99)),'max':float(values.max())}


def duration_distribution(values):
    result=distribution(values)
    if result is not None:result['over_300ms']=int(np.count_nonzero(np.asarray(values,float)>.3))
    return result


def phase(time):
    return 'startup_0_10'if time<10 else'pre_detail_10_115'if time<115 else'detail_115_165'if time<=165 else'post_detail_165_end'


def analyze(run):
    run=Path(run).resolve();out={};timings=defaultdict(list);counts=Counter();raw_stamps=[];raw_receipts=[]
    started={};propagated={};stage_sequences=defaultdict(list);b=run/'fastlivo_diagnostics/records.bin'
    with b.open('rb')as stream:
        if stream.read(16)!=MAGIC:raise ValueError('Magic differs')
        while True:
            header=stream.read(HEADER.size)
            if not header:break
            if len(header)!=HEADER.size:raise ValueError('Truncated header')
            kind,seq,stamp,stage,iteration,level,n=HEADER.unpack(header)
            data=stream.read(n*8)
            if len(data)!=n*8:raise ValueError('Truncated payload')
            a=np.frombuffer(data,dtype='<f8');counts[kind]+=1;s=stamp*1e-9;window=phase(s)
            if kind==1:
                raw_stamps.append(stamp);raw_receipts.append(a[0])
            elif kind==10:started[seq]=(a[0],s,stage)
            elif kind==11:
                propagated[seq]=a[0]
                if seq in started:timings[(window,'IMU_propagation_wall_s')].append(a[0]-started[seq][0])
            elif kind in (12,13):
                if seq in started:
                    timings[(window,'total_estimator_phase_wall_s')].append(a[1]-started[seq][0])
                    stage_sequences[stage].append(s)
                if seq in propagated:timings[(window,'observation_stage_wall_s')].append(a[1]-propagated[seq])
    stamps=np.asarray(raw_stamps,np.int64);receipts=np.asarray(raw_receipts,float)
    dt=np.diff(stamps)*1e-9
    out['actual_raw_IMU_callback']={'count':len(stamps),'first_last_sim_s':[float(stamps[0]*1e-9),float(stamps[-1]*1e-9)],
        'sim_hz':float((len(stamps)-1)/((stamps[-1]-stamps[0])*1e-9)),
        'wall_hz':float((len(receipts)-1)/(receipts[-1]-receipts[0])),
        'source_gap_s':duration_distribution(dt),'callback_wall_gap_s':duration_distribution(np.diff(receipts)),
        'duplicate_headers':int(np.count_nonzero(dt==0)),'backward_headers':int(np.count_nonzero(dt<0))}
    out['estimator_phase_latency']={window:{name:duration_distribution(v)for(w,name),v in timings.items()if w==window}
        for window in sorted({w for w,name in timings})}
    out['estimator_stage_actual_header_hz']={str(stage):float((len(t)-1)/(t[-1]-t[0]))if len(t)>1 else None
        for stage,t in stage_sequences.items()}
    out['logger_exclusive_CPU_or_time_cost']='unverified: no otherwise-identical disabled-logger run; recorded stage times include original estimator and logging'
    pipe=defaultdict(list);reasons=Counter();poses=[];clouds=[]
    for row in rows(run/'navigation_cloud_callbacks.jsonl'):
        s=row['producer_stamp_ns']*1e-9;window=phase(s);reasons[row.get('reason','unknown')]+=1
        pipe[(window,'cloud_callback_wall_s')].append(row['callback_duration_wall_s'])
        pipe[(window,'cloud_source_sim_age_at_callback_s')].append((row['received_ros_clock_ns']-row['producer_stamp_ns'])*1e-9)
        if row.get('accepted'):clouds.append((s,row['received_monotonic_wall']))
    for row in rows(run/'navigation_slam_poses.jsonl'):
        s=row['stamp_ns']*1e-9;window=phase(s);poses.append((s,row['received_monotonic_wall']))
        pipe[(window,'pose_source_sim_age_at_callback_s')].append(row['sim_age_at_callback_s'])
    for row in rows(run/'navigation_pid_history.jsonl'):
        s=row['source_pose_stamp_ns']*1e-9;window=phase(s);clock=row['compute_ros_clock_ns']
        pipe[(window,'pose_sim_age_at_controller_s')].append((clock-row['source_pose_stamp_ns'])*1e-9)
        pipe[(window,'paired_imu_sim_age_at_controller_s')].append((clock-row['paired_imu_stamp_ns'])*1e-9)
        pipe[(window,'pose_wall_age_at_controller_s')].append(row['compute_monotonic_wall']-row['feedback']['received_wall_ns']*1e-9)
        if row.get('imu'):
            pipe[(window,'paired_imu_wall_age_at_controller_s')].append(row['compute_monotonic_wall']-row['imu']['received_wall_ns']*1e-9)
    for name,array in [('pose',poses),('cloud',clouds)]:
        for previous,current in zip(array,array[1:]):
            pipe[(phase(current[0]),name+'_source_sim_gap_s')].append(current[0]-previous[0])
            pipe[(phase(current[0]),name+'_callback_wall_gap_s')].append(current[1]-previous[1])
    out['navigation_pipeline']={window:{name:duration_distribution(v)for(w,name),v in pipe.items()if w==window}
        for window in sorted({w for w,name in pipe})}
    out['cloud_callback_reasons']=dict(reasons)
    resources=list(rows(run/'owned_resources.jsonl'));out['shared_resources']={
        'samples':len(resources),'system_load1':distribution([r['load_1_5_15'][0]for r in resources]),
        'logical_cpus':resources[0]['logical_cpus'],
        'GPU_used_MiB':distribution([r['gpu']['used_MiB']for r in resources if'gpu'in r]),
        'GPU_utilization_percent':distribution([r['gpu']['utilization_percent']for r in resources if'gpu'in r]),
        'RAM_available_GiB':distribution([r['memory_KiB']['MemAvailable']/1024**2 for r in resources]),
        'CPU_percent_limit':'owned_resources samples record top-level owned PIDs; they do not measure all SLAM/Gazebo grandchildren CPU'}
    faults=Counter();inference=[];clearance=[];rp=[];bodycontacts=0;steps=0;maximum_tau=0;maximum_qd=0;targets_outside=0
    for row in rows(run/'actuator.jsonl'):
        if row.get('kind')!='physics_step':continue
        steps+=1;faults[str(row.get('fault'))]+=1
        bodycontacts+=int(row['contacts'][0]>0)
        maximum_tau=max(maximum_tau,max(abs(v)for v in row['tau']))
        maximum_qd=max(maximum_qd,max(abs(v)for v in row['qd']))
        targets_outside+=int(any(row['target_outside_hard_limits']))
    telemetry_faults=Counter();requests_expired=0;telemetry_rows=0;lasttime=None
    for row in rows(run/'telemetry.jsonl'):
        telemetry_rows+=1;lasttime=row['world_sim_time'];telemetry_faults[str(row.get('fault'))]+=1
        if row.get('actor_inferred_this_frame'):inference.append(row['inference_ms'])
        clearance.append(row['body_clearance']);rp.append(max(abs(row['rpy'][0]),abs(row['rpy'][1])))
        requests_expired+=int(row.get('command_expired',False))
    out['offline_native_safety']={'physics_samples':steps,'fault_codes':dict(faults),
        'body_contact_samples':bodycontacts,'joint_torque_abs_max_Nm':maximum_tau,'joint_speed_abs_max_radps':maximum_qd,
        'target_outside_hard_limit_samples':targets_outside,'telemetry_rows':telemetry_rows,
        'telemetry_faults':dict(telemetry_faults),'last_telemetry_world_time_s':lasttime,
        'body_clearance_m':distribution(clearance),'roll_pitch_abs_rad':distribution(rp),
        'inference_ms':distribution(inference),'command_expired_frames':requests_expired,
        'native_source_use':'offline physical safety only; no native pose used for navigation'}
    statuses=Counter();failed=[];stale=[]
    for row in rows(run/'navigation_status.jsonl'):
        statuses[row.get('state','unknown')]+=1
        if row.get('state')=='failed'and len(failed)<5:failed.append({'time':row.get('ros_sim_time'),'reason':row.get('reason'),'failure':row.get('failure')})
    for row in rows(run/'navigation_sensor_gate_history.jsonl'):
        if row.get('initial_warmup_complete')and not row.get('ready')and len(stale)<20:
            stale.append({'time_s':row['ros_sim_time_ns']*1e-9,'reason':row.get('reason')})
    out['navigation_status']={'states':dict(statuses),'first_failures':failed,'first20_post_warmup_sensor_holds':stale}
    for name in ('worker_result.json','runtime_manifest.json','slam_loaded_binary.json','fastlivo_diagnostics/writer_stats.json'):
        out[name]=json.loads((run/name).read_text())
    out.update(schema='teacher_v7_runtime_health/v1',run=str(run),reader_sha256=sha(__file__),
        meaning='Finite210s diagnostic prefix operational evidence; not a600s/full32-region navigation pass',
        record_kind_counts=dict(counts))
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    report=analyze(a.run)
    with a.output.open('x')as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:report[k]for k in ['run','actual_raw_IMU_callback','navigation_status','offline_native_safety']},indent=2))


if __name__=='__main__':main()
