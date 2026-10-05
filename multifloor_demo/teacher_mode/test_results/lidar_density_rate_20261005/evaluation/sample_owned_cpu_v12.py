#!/usr/bin/env python3
"""Version2 external owned CPU and FASTLIVO per-thread sampling, default0.5Hz.
Never changes scheduling, affinity, physics, processes, controllers or files.
Only the audit JSONL/summary are appended to the explicitly supplied run.
"""
from __future__ import annotations
import argparse,datetime,hashlib,json,os,time
from pathlib import Path

def proc_stat(pid):
    try:
        text=Path('/proc',str(pid),'stat').read_text();closing=text.rfind(')');tail=text[closing+2:].split()
        return dict(pid=pid,comm=text[text.find('(')+1:closing],state=tail[0],ppid=int(tail[1]),
            utime_ticks=int(tail[11]),stime_ticks=int(tail[12]),threads=int(tail[17]),starttime_ticks=int(tail[19]),last_processor=int(tail[36]))
    except (OSError,ValueError,IndexError):return None
def descendants(root,all_stats):
    owned={root};changed=True
    while changed:
        added={pid for pid,d in all_stats.items()if d['ppid']in owned}-owned;changed=bool(added);owned|=added
    return owned
def small_state(run,name):
    p=run/name
    try:
        if p.stat().st_size>262144:return dict(status='too_large_for_lightweight_sampler')
        d=json.loads(p.read_text())
        return {k:v for k,v in d.items()if k in('sim_time','last_sim_time','t','ros_sim_time','monotonic_wall','state','phase','healthy','sequence','waypoint_index','command')and type(v)in(int,float,str,bool,list,type(None))}
    except (OSError,ValueError):return None
def thread_stats(pid,clockticks,wall,previous):
    result=[];task=Path('/proc',str(pid),'task')
    try:members=list(task.iterdir())
    except OSError:return []
    for p in members:
        if not p.name.isdigit():continue
        tid=int(p.name)
        try:
            text=(p/'stat').read_text();closing=text.rfind(')');tail=text[closing+2:].split()
            d=dict(tid=tid,comm=text[text.find('(')+1:closing],state=tail[0],utime_ticks=int(tail[11]),stime_ticks=int(tail[12]),
                starttime_ticks=int(tail[19]),last_processor=int(tail[36]),is_process_main_thread=tid==pid)
            status=(p/'status').read_text()
            for key in ('voluntary_ctxt_switches','nonvoluntary_ctxt_switches'):
                d[key]=int(next(line.split(':',1)[1]for line in status.splitlines()if line.startswith(key+':')))
            d['affinity_cpus']=sorted(os.sched_getaffinity(tid));d['cpu_seconds']=(d['utime_ticks']+d['stime_ticks'])/clockticks
            identity=(pid,tid,d['starttime_ticks']);old=previous.get(identity)
            if old is not None:
                d.update(delta_cpu_s=d['cpu_seconds']-old['cpu_seconds'],delta_wall_s=wall-old['wall'],
                    delta_voluntary_ctxt_switches=d['voluntary_ctxt_switches']-old['voluntary_ctxt_switches'],
                    delta_nonvoluntary_ctxt_switches=d['nonvoluntary_ctxt_switches']-old['nonvoluntary_ctxt_switches'])
                d['window_CPU_cores']=d['delta_cpu_s']/d['delta_wall_s']if d['delta_wall_s']>0 else None
            else:d.update(delta_cpu_s=None,delta_wall_s=None,window_CPU_cores=None,
                delta_voluntary_ctxt_switches=None,delta_nonvoluntary_ctxt_switches=None)
            previous[identity]=dict(wall=wall,cpu_seconds=d['cpu_seconds'],voluntary_ctxt_switches=d['voluntary_ctxt_switches'],nonvoluntary_ctxt_switches=d['nonvoluntary_ctxt_switches'])
            result.append(d)
        except (OSError,StopIteration,ValueError,IndexError):continue
    return sorted(result,key=lambda d:d['tid'])
def sample(run,root,hz,maxwall):
    clockticks=os.sysconf('SC_CLK_TCK');out=run/'external_owned_cpu_profile.jsonl'
    started=time.monotonic();own_cpu_started=time.process_time();expected_root=proc_stat(root)
    if expected_root is None:raise RuntimeError('Owned runner already exited; no fabricated CPU prefix')
    seq=0;seen=set();last={};missing=[];rows_summary={};largest_sample=0.;schedule=started;thread_previous={}
    with out.open('x')as f:
        while time.monotonic()-started<maxwall:
            begin=time.monotonic();stats={}
            for p in Path('/proc').iterdir():
                if p.name.isdigit():
                    d=proc_stat(int(p.name))
                    if d is not None:stats[d['pid']]=d
            current=stats.get(root)
            if current is None or current['starttime_ticks']!=expected_root['starttime_ticks']:break
            owned=descendants(root,stats);processes=[]
            for pid in sorted(owned):
                d=stats.get(pid)
                if d is None:continue
                key=(pid,d['starttime_ticks']);seen.add(key)
                try:d['affinity_cpus']=sorted(os.sched_getaffinity(pid))
                except ProcessLookupError:d['affinity_cpus']=None
                try:d['executable']=str(Path('/proc',str(pid),'exe').resolve()).split('/')[-1]
                except OSError:d['executable']=None
                d['cpu_seconds']=(d['utime_ticks']+d['stime_ticks'])/clockticks
                if d['comm'].startswith('fastlivo_mapp'):
                    d['actual_FASTLIVO_owned_threads']=thread_stats(pid,clockticks,begin,thread_previous)
                old=last.get(key)
                if old is not None:
                    dt=begin-old['wall'];dc=d['cpu_seconds']-old['cpu_seconds']
                    d['delta_wall_s']=dt;d['delta_cpu_s']=dc;d['window_CPU_cores']=dc/dt if dt>0 else None
                else:d['delta_wall_s']=d['delta_cpu_s']=d['window_CPU_cores']=None
                last[key]={'wall':begin,'cpu_seconds':d['cpu_seconds']}
                item=rows_summary.setdefault(str(pid)+':'+str(key[1]),dict(pid=pid,comm=d['comm'],starttime_ticks=key[1],first_wall=begin,first_cpu_seconds=d['cpu_seconds']))
                item.update(last_wall=begin,last_cpu_seconds=d['cpu_seconds'],sampled_cpu_seconds=d['cpu_seconds']-item['first_cpu_seconds'],sampled_wall_seconds=begin-item['first_wall'])
                processes.append(d)
            seq+=1;files={}
            for name in('navigation_command_publications.jsonl','navigation_pid_history.jsonl','fastlivo_diagnostics/records.bin'):
                try:files[name]=dict(size_bytes=(run/name).stat().st_size)
                except OSError:files[name]=None
            duration=time.monotonic()-begin;largest_sample=max(largest_sample,duration)
            row=dict(schema='external_owned_cpu_time_profile/v2',run=str(run),sequence=seq,monotonic_wall=begin,
                UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),CLK_TCK=clockticks,root_pid=root,root_starttime_ticks=expected_root['starttime_ticks'],
                membership='Only supplied owned runner and its actual /proc PPID descendants; PID identity includes starttime',
                state=small_state(run,'state.json'),navigation=small_state(run,'navigation_status.json'),bridge=small_state(run,'navigation_command.json'),
                file_size_metadata_only=files,processes=processes,sampler_cycle_wall_s=duration,
                sampler_cumulative_cpu_seconds=time.process_time()-own_cpu_started)
            f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');f.flush()
            schedule+=1/hz;delay=schedule-time.monotonic()
            if delay>0:time.sleep(min(delay,1/hz))
    summary=dict(schema='external_owned_cpu_time_profile_completion/v2',run=str(run),root_pid=root,samples=seq,
        start_monotonic_wall=started,end_monotonic_wall=time.monotonic(),sampling_hz=hz,CLK_TCK=clockticks,
        process_time_scope='CPU seconds are original /proc utime+stime tick deltas over sampled wall intervals, not lifecycle ps %CPU. Sampling starts when external audit is authorized; earlier epochs are unavailable. Descendant CPU totals may overlap process lifetime but per-PID/starttime deltas do not include unrelated training.',
        cumulative_sampler_cpu_s=time.process_time()-own_cpu_started,maximum_sampler_cycle_wall_s=largest_sample,
        processes=rows_summary,reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (run/'external_owned_cpu_profile_completion.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--root-pid',type=int,required=True);p.add_argument('--hz',type=float,default=.5);p.add_argument('--max-wall-s',type=float,default=1800);a=p.parse_args()
    d=sample(a.run.resolve(),a.root_pid,a.hz,a.max_wall_s);print(json.dumps({k:v for k,v in d.items()if k!='processes'},indent=2))
if __name__=='__main__':main()
