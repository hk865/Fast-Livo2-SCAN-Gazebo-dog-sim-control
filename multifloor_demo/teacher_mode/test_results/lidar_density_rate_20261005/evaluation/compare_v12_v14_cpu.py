#!/usr/bin/env python3
"""Compare completed fixed CPU bands and independently parse affinity witness.

No acceptance gate changes; shared time bands are not identical robot states.
"""
from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def cpu_list(value):
    result=set()
    for item in value.split(','):
        limits=item.strip().split('-')
        if len(limits)==1:result.add(int(limits[0]))
        elif len(limits)==2:result.update(range(int(limits[0]),int(limits[1])+1))
        else:raise ValueError('Malformed cpus_allowed_list')
    return sorted(result)
def affinity(run):
    contract=read(run/'cpu_affinity_contract.json');expected=contract['group_cpus'];seen=set();errors=[];count=0;threads=0;identities=set();walls=[]
    with(run/'cpu_affinity_witness.jsonl').open()as f:
        for line in f:
            if not line.strip():continue
            row=json.loads(line);count+=1;walls.append(row['monotonic_wall_ns'])
            if row.get('schema')!='teacher_owned_cpu_affinity_witness/v1'or row.get('only_saved_owned_process_groups')is not True:errors.append([count,'Witness schema/membership differs'])
            for group in row['groups']:
                name=group['group'];seen.add(name)
                if name not in expected or group['expected_cpus']!=expected[name]:errors.append([count,name,'Expected source contract differs'])
                for t in group['threads']:
                    threads+=1;identities.add((group['pid'],group['start_ticks'],t['tid']))
                    if name not in expected or cpu_list(t['cpus_allowed_list'])!=expected[name]:errors.append([count,group['pid'],t['tid'],'Actual mask differs'])
    if seen!=set(expected):errors.append(['Required five actual groups not all seen',sorted(seen)])
    if any(a>=b for a,b in zip(walls,walls[1:])):errors.append(['Witness wall chronology differs'])
    return dict(schema='independent_owned_actual_thread_affinity_audit/v1',run=str(run),status='failed'if errors else'passed',witness_records=count,actual_thread_samples=threads,
        unique_owned_thread_identity_count=len(identities),actual_groups_seen=sorted(seen),expected_group_cpus=expected,errors=errors,
        contract_sha256=sha(run/'cpu_affinity_contract.json'),actual_witness_sha256=sha(run/'cpu_affinity_witness.jsonl'),
        source_bindings={'navigation_scope.json':sha(run/'navigation_scope.json'),'navigation_source_snapshots.json':sha(run/'navigation_source_snapshots.json'),'runtime_manifest.json':sha(run/'runtime_manifest.json')},
        limitation='CPU masks restrict this run owned threads. They do not reserve cores or move other tasks; no training processes were changed.')
def compact(b):
    process=next(p for p in b['owned_processes']if p['comm'].startswith('fastlivo'))
    main=next(p for p in process['per_thread_CPU']if p['is_process_main_thread'])
    stages={str(s['boundary']):dict(label=s['label'],calls=s['completed_calls'],mean_wall_ms=s['mean_wall_ms'],mean_caller_CPU_ms=s['mean_caller_CPU_ms'],
        concurrent_process_CPU_to_wall=s['concurrent_process_CPU_to_wall'])for s in b['boundary_timing']['boundaries']}
    return dict(sampled_sim_bounds=b['sampled_sim_time_bounds_s'],wall_seconds=b['wall_seconds'],FASTLIVO_CPU_cores=process['average_CPU_cores'],main_CPU_cores=main['average_CPU_cores'],
        main_involuntary_context_switches=main['nonvoluntary_context_switches'],main_involuntary_per_wall_second=main['nonvoluntary_context_switches']/b['wall_seconds'],
        main_voluntary_context_switches=main['voluntary_context_switches'],main_affinity_cpus=main['affinity_cpus'],boundaries=stages)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--v12',required=True,type=Path);p.add_argument('--v14',required=True,type=Path);p.add_argument('--v14-run',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    x,y=read(a.v12),read(a.v14);paired=[]
    for requested in ([60,110],[130,170],[170,210.01]):
        left=next(b for b in x['sim_bands']if b['requested_sim_band_s']==requested);right=next(b for b in y['sim_bands']if b['requested_sim_band_s']==requested)
        if left['status']!='measured'or right['status']!='measured':paired.append(dict(requested_sim_band_s=requested,status='unavailable'));continue
        lx,ry=compact(left),compact(right);paired.append(dict(requested_sim_band_s=requested,status='measured',V12=lx,V14=ry,
            V14_over_V12_ratio={'FASTLIVO_CPU_cores':ry['FASTLIVO_CPU_cores']/lx['FASTLIVO_CPU_cores'],
                'main_involuntary_rate':ry['main_involuntary_per_wall_second']/lx['main_involuntary_per_wall_second']if lx['main_involuntary_per_wall_second']else None}))
    result=dict(schema='independent_V12_V14_same_requested_sim_bands_performance_comparison/v1',V12_run=x['run'],V14_run=y['run'],same_requested_sim_bands=paired,
        actual_affinity_witness=affinity(a.v14_run.resolve()),input_sha256={str(a.v12):sha(a.v12),str(a.v14):sha(a.v14)},reader_sha256=sha(Path(__file__)),
        limits=['The same requested simulation-time bands contain different robot poses and realized feature/iteration costs. They are not identical-scene compute replay.',
            'Actual sampled wall endpoints differ slightly; cost per completed call and context switches per wall second are separately shown.',
            'Numerical motion/source/arrival/safety/parking gates remain in independent original receipts; this performance comparison cannot upgrade partial navigation to pass.'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
