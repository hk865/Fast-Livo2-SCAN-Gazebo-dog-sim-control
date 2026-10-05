#!/usr/bin/env python3
"""Full-run owned CPU reader; disabled optional timers remain N/A, not zero.

Reuses the frozen V12 independent parsing equations without altering that
reader or its historical outputs. No runtime execution or control imports.
"""
from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
import performance_proc_v12 as original
def process_lifetimes(samples,cutoff):
    physical=[s for s in samples if s['monotonic_wall']<=cutoff];identities={};threads={}
    for sample in physical:
        wall=sample['monotonic_wall'];sim=original.state_time(sample)
        for process in sample['processes']:
            key=(process['pid'],process['starttime_ticks']);identities.setdefault(key,[]).append((wall,sim,process))
            for thread in process.get('actual_FASTLIVO_owned_threads',[]):
                identity=(*key,thread['tid'],thread['starttime_ticks']);threads.setdefault(identity,[]).append((wall,sim,thread))
    result=[]
    for key,observations in identities.items():
        start,finish=observations[0],observations[-1];a,b=start[2],finish[2];elapsed=finish[0]-start[0];delta=b['cpu_seconds']-a['cpu_seconds']
        if delta<0:raise ValueError('Original owned process CPU regressed')
        item=dict(pid=key[0],starttime_ticks=key[1],comm=a['comm'],CPU_seconds=delta,
            actual_observed_wall_bounds_s=[start[0],finish[0]],actual_observed_sim_bounds_s=[start[1],finish[1]],wall_seconds=elapsed,
            average_CPU_cores=delta/elapsed if elapsed else None,first_recorded_cpu_seconds=a['cpu_seconds'],affinity_cpus=a['affinity_cpus'],samples=len(observations),
            utime_ticks_delta=b['utime_ticks']-a['utime_ticks'],stime_ticks_delta=b['stime_ticks']-a['stime_ticks'])
        thread_result=[]
        for identity,observed in threads.items():
            if identity[:2]!=key:continue
            u,v=observed[0],observed[-1];x,y=u[2],v[2];dt=v[0]-u[0];dc=y['cpu_seconds']-x['cpu_seconds']
            thread_result.append(dict(tid=identity[2],starttime_ticks=identity[3],comm=x['comm'],is_process_main_thread=x['is_process_main_thread'],
                CPU_seconds=dc,average_CPU_cores=dc/dt if dt else None,actual_observed_wall_bounds_s=[u[0],v[0]],actual_observed_sim_bounds_s=[u[1],v[1]],wall_seconds=dt,
                voluntary_context_switches=y['voluntary_ctxt_switches']-x['voluntary_ctxt_switches'],nonvoluntary_context_switches=y['nonvoluntary_ctxt_switches']-x['nonvoluntary_ctxt_switches'],
                first_processor=x['last_processor'],last_processor=y['last_processor'],affinity_cpus=x['affinity_cpus'],samples=len(observed)))
        if thread_result:item['actual_per_thread_observation_windows']=sorted(thread_result,key=lambda t:-t['CPU_seconds'])
        result.append(item)
    return dict(whole_sampler_physical_wall_bounds_s=[physical[0]['monotonic_wall'],physical[-1]['monotonic_wall']],
        actual_owned_processes=sorted(result,key=lambda p:-p['CPU_seconds']),
        scope='Each original PID/starttime and TID/starttime is counted over its own first-to-last physical observations; child processes starting after the first runner sample are retained. CPU before first observation and after last observation is unavailable, not assumed zero. Different process windows are explicitly shown.')
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();run=a.run.resolve()
    profile=json.loads((run/'navigation_profile.json').read_text());data=original.evaluate(run)
    enabled=profile.get('boundary_timing')is True
    if not enabled and data['total_kind300_records']!=0:raise ValueError('Optional timing records exist despite frozen disabled profile')
    if enabled and data['total_kind300_records']==0:raise ValueError('Enabled timing profile has no recorded evidence')
    if not enabled:
        unavailable=dict(status='not_collected',reason='Frozen actual profile explicitly disables optional kind300 boundary timing',cost_values=None,missing_is_not_zero=True)
        data['same_window_boundary_timing']=unavailable
        for band in data['sim_bands']:
            if 'boundary_timing'in band:band['boundary_timing']=unavailable.copy()
    data['same_physical_CPU']=process_lifetimes(original.rows(run/'external_owned_cpu_profile.jsonl'),data['actual_last_Actor_read_wall'])
    data.update(schema='actual_fullrun_owned_proc_thread_CPU_with_explicit_optional_timing_scope/v1',
        optional_boundary_timing_enabled_in_actual_profile=enabled,base_reader_sha256=original.READER_SHA,
        reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        profile_sha256=hashlib.sha256((run/'navigation_profile.json').read_bytes()).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');print(json.dumps(data,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
