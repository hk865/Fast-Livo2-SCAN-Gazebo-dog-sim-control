#!/usr/bin/env python3
"""Read-only kind300 wall/CPU boundaries and owned /proc per-thread deltas.

The completion-window timer records overlap. This reader never converts their
sum, or wall minus caller CPU, into a purported communication measurement.
"""
from __future__ import annotations
import argparse,csv,hashlib,json,math,struct
from collections import defaultdict
from pathlib import Path
HEADER=struct.Struct('<7Q')
READER_SHA=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
LABELS=['Process2 whole','UndistortPcl whole','backward point deskew','LIO residual matching query','LIO StateEstimation whole','UpdateVoxelMap','VIO processFrame','publication conversion/enqueue','spin_some','lidar callback','IMU callback','image callback','lidar preprocessing','sync_packages','handleLIO whole','handleVIO whole','BuildVoxelMap initial']
NESTING={1:[2],4:[3],8:[9,10,11],9:[12],14:[3,4,5,7,16],15:[6,7]}
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l)for l in p.open()if l.strip()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def state_time(row):return (row.get('state')or{}).get('sim_time')
def cpu_window(first,last):
    duration=last['monotonic_wall']-first['monotonic_wall']
    if duration<=0:raise ValueError('Nonpositive CPU window')
    a={(p['pid'],p['starttime_ticks']):p for p in first['processes']};b={(p['pid'],p['starttime_ticks']):p for p in last['processes']};result=[]
    for key in a.keys()&b.keys():
        x,y=a[key],b[key];delta=y['cpu_seconds']-x['cpu_seconds']
        if delta<0:raise ValueError('Process CPU ticks regressed')
        item=dict(pid=key[0],starttime_ticks=key[1],comm=x['comm'],CPU_seconds=delta,wall_seconds=duration,average_CPU_cores=delta/duration,
            utime_ticks_delta=y['utime_ticks']-x['utime_ticks'],stime_ticks_delta=y['stime_ticks']-x['stime_ticks'],first_threads=x['threads'],last_threads=y['threads'],affinity_cpus=x['affinity_cpus'])
        threads_a={(t['tid'],t['starttime_ticks']):t for t in x.get('actual_FASTLIVO_owned_threads',[])};threads_b={(t['tid'],t['starttime_ticks']):t for t in y.get('actual_FASTLIVO_owned_threads',[])}
        thread_result=[]
        for identity in threads_a.keys()&threads_b.keys():
            u,v=threads_a[identity],threads_b[identity];dc=v['cpu_seconds']-u['cpu_seconds']
            thread_result.append(dict(tid=identity[0],starttime_ticks=identity[1],comm=u['comm'],is_process_main_thread=u['is_process_main_thread'],CPU_seconds=dc,
                average_CPU_cores=dc/duration,utime_ticks_delta=v['utime_ticks']-u['utime_ticks'],stime_ticks_delta=v['stime_ticks']-u['stime_ticks'],
                voluntary_context_switches=v['voluntary_ctxt_switches']-u['voluntary_ctxt_switches'],nonvoluntary_context_switches=v['nonvoluntary_ctxt_switches']-u['nonvoluntary_ctxt_switches'],
                first_processor=u['last_processor'],last_processor=v['last_processor'],affinity_cpus=u['affinity_cpus']))
        if thread_result:
            item.update(per_thread_CPU=sorted(thread_result,key=lambda t:-t['CPU_seconds']),thread_CPU_sum_seconds=sum(t['CPU_seconds']for t in thread_result),
                thread_membership_first_only=[k[0]for k in threads_a.keys()-threads_b.keys()],thread_membership_last_only=[k[0]for k in threads_b.keys()-threads_a.keys()],
                thread_role_limit='Main TID is explicit. Other TIDs are original actual threads; identical comm cannot independently distinguish OpenMP, writer and ROS/DDS roles.')
        result.append(item)
    return dict(same_physical_wall_window_s=[first['monotonic_wall'],last['monotonic_wall']],sampled_sim_time_bounds_s=[state_time(first),state_time(last)],
        wall_seconds=duration,owned_processes=sorted(result,key=lambda p:-p['CPU_seconds']),
        membership_first_only=[k[0]for k in a.keys()-b.keys()],membership_last_only=[k[0]for k in b.keys()-a.keys()])
def boundaries(run):
    result=[]
    with (run/'fastlivo_diagnostics/records.bin').open('rb')as f,(run/'fastlivo_diagnostics/index.csv').open()as idx:
        for row in csv.DictReader(idx):
            if int(row['kind'])!=300:continue
            f.seek(int(row['offset']));head=f.read(56)
            if len(head)!=56:raise ValueError('Truncated kind300 header')
            h=HEADER.unpack(head)
            if h[0]!=300 or h[6]!=90 or int(row['n_values'])!=90:raise ValueError('Unexpected kind300/index layout')
            payload=f.read(90*8)
            if len(payload)!=90*8:raise ValueError('Truncated kind300 values')
            values=struct.unpack('<90d',payload)
            if not all(math.isfinite(v)and v>=0 for v in values):raise ValueError('Nonfinite/negative timing field')
            if values[0]!=1 or values[4]!=17 or values[3]<values[2]:raise ValueError('Unsupported boundary timing schema/clock')
            stage=[]
            for i in range(17):
                calls,wall,thread,process,invalid=values[5+i*5:10+i*5]
                if calls!=int(calls)or invalid!=int(invalid)or invalid>calls:raise ValueError('Invalid boundary count')
                stage.append(dict(boundary=i,label=LABELS[i],completed_calls=int(calls),wall_seconds=wall/1e9,caller_thread_CPU_seconds=thread/1e9,concurrent_process_CPU_seconds=process/1e9,invalid_clock_calls=int(invalid)))
            result.append(dict(sequence=int(values[1]),begin_wall=values[2]/1e9,end_wall=values[3]/1e9,latest_logger_source_stamp_s=h[2]/1e9,boundaries=stage))
    return result
def summarize_boundaries(records,start,end):
    selected=[r for r in records if start<=r['begin_wall']<=r['end_wall']<=end];sums=defaultdict(lambda:dict(completed_calls=0,wall_seconds=0.,caller_thread_CPU_seconds=0.,concurrent_process_CPU_seconds=0.,invalid_clock_calls=0))
    for row in selected:
        for stage in row['boundaries']:
            for key in sums[stage['boundary']]:sums[stage['boundary']][key]+=stage[key]
    summary=[]
    for i in range(17):
        d=dict(boundary=i,label=LABELS[i],**sums[i]);count=d['completed_calls'];d.update(mean_wall_ms=d['wall_seconds']*1000/count if count else None,
            mean_caller_CPU_ms=d['caller_thread_CPU_seconds']*1000/count if count else None,mean_concurrent_process_CPU_ms=d['concurrent_process_CPU_seconds']*1000/count if count else None,
            wall_fraction_of_CPU_window=d['wall_seconds']/(end-start),caller_CPU_fraction_of_CPU_window=d['caller_thread_CPU_seconds']/(end-start),
            nested_boundary_ids=NESTING.get(i,[]),caller_CPU_to_wall=d['caller_thread_CPU_seconds']/d['wall_seconds']if d['wall_seconds']else None,
            concurrent_process_CPU_to_wall=d['concurrent_process_CPU_seconds']/d['wall_seconds']if d['wall_seconds']else None)
        summary.append(d)
    return dict(kind300_complete_windows=len(selected),requested_CPU_wall_window_s=[start,end],
        complete_kind300_window_bounds_s=[min(r['begin_wall']for r in selected),max(r['end_wall']for r in selected)]if selected else None,
        total_window_wall_seconds=sum(r['end_wall']-r['begin_wall']for r in selected),boundaries=summary)
def evaluate(run):
    samples=rows(run/'external_owned_cpu_profile.jsonl');common=read(run/'summary_closed_loop_cascade_independent.json')
    cutoff=common['checks']['execution_phase_status_and_cleanup_boundary']['last_actual_Actor_read_wall']
    physical=[r for r in samples if r['monotonic_wall']<=cutoff]
    if len(physical)<2:raise ValueError('No complete physical CPU interval')
    if any(r.get('schema')!='external_owned_cpu_time_profile/v2'for r in physical):raise ValueError('Per-thread sampler version differs')
    if any(a['monotonic_wall']>=b['monotonic_wall']for a,b in zip(physical,physical[1:])):raise ValueError('CPU sample wall chronology differs')
    records=boundaries(run);all_window=cpu_window(physical[0],physical[-1]);start,end=all_window['same_physical_wall_window_s'];bins=[]
    for lo,hi in [(0,60),(60,110),(110,115),(115,118),(118,130),(130,170),(170,210.01)]:
        s=[r for r in physical if state_time(r)is not None and lo<=state_time(r)<hi]
        if len(s)<2:
            bins.append(dict(requested_sim_band_s=[lo,hi],status='unavailable',reason='Fewer than two physical /proc samples'));continue
        item=cpu_window(s[0],s[-1]);bs,be=item['same_physical_wall_window_s'];item.update(requested_sim_band_s=[lo,hi],status='measured',boundary_timing=summarize_boundaries(records,bs,be));bins.append(item)
    schema_candidates=[]
    snapshots=read(run/'navigation_source_snapshots.json')
    for name,item in snapshots.items():
        if Path(name).name in('boundary_timing_diagnostic_schema.json','BOUNDARY_TIMING_CONTRACT.json','boundary_timing.h'):
            path=Path(item['snapshot']);actual=sha(path);schema_candidates.append(dict(source=name,snapshot=str(path),expected_sha256=item['sha256'],actual_sha256=actual,hash_verified=actual==item['sha256']))
    return dict(schema='actual_same_wall_window_proc_thread_and_boundary_cost/v2',run=str(run),reader_sha256=READER_SHA,
        common_receipt_sha256=sha(run/'summary_closed_loop_cascade_independent.json'),actual_last_Actor_read_wall=cutoff,physical_CPU_sample_count=len(physical),
        actual_proc_sampler_completion=read(run/'external_owned_cpu_profile_completion.json'),earlier_CPU_epochs='Unavailable before the first actual external sample; never reconstructed from lifecycle %CPU.',
        same_physical_CPU=all_window,same_window_boundary_timing=summarize_boundaries(records,start,end),sim_bands=bins,
        total_kind300_records=len(records),total_invalid_boundary_clock_calls=sum(s['invalid_clock_calls']for r in records for s in r['boundaries']),verified_timing_source_bindings=schema_candidates,
        scope_limits=[
            'Boundary rows have no TID; caller thread CPU cannot be individually joined to /proc TIDs. /proc main and worker CPU deltas are measured separately.',
            'Kind300 accrues completed scopes, not exclusive execution. Inner and outer scopes overlap and may finish in adjacent windows. Never sum these boundaries into total pipeline CPU or time.',
            'Only kind300 windows wholly inside the stated CPU sample interval are selected. A final partial logger window is unrecorded. Process deltas cover the exact /proc sample bounds.',
            'Boundary process CPU includes all concurrent OpenMP/DDS/logger work in the process and is not exclusive CPU cost of the named function.',
            'Wall minus caller CPU mixes worker overlap, descheduling, waiting and clock observation. Publication measures synchronous conversion/enqueue, excluding DDS communication after return.',
            'Source acquisition to callback and callback to LIO start require separate source/phase chronology; residual unspecified time is not automatically communication.'
        ])
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();d=evaluate(a.run.resolve());a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');print(json.dumps(d,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
