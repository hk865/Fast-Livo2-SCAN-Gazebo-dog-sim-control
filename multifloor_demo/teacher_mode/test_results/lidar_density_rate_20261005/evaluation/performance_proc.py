#!/usr/bin/env python3
"""Same-wall-window CPU deltas and measured pipeline segments; no control."""
from __future__ import annotations
import argparse,csv,hashlib,json,struct
from collections import defaultdict
from pathlib import Path
HEADER=struct.Struct('<7Q')
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l)for l in p.open()if l.strip()]
def evaluate(run):
    samples=rows(run/'external_owned_cpu_profile.jsonl');common=read(run/'summary_closed_loop_cascade_independent.json')
    boundary=common['checks']['execution_phase_status_and_cleanup_boundary']['last_actual_Actor_read_wall']
    physical=[d for d in samples if d['monotonic_wall']<=boundary]
    if len(physical)<2:raise ValueError('Insufficient actual physical CPU samples')
    first,last=physical[0],physical[-1];start=first['monotonic_wall'];end=last['monotonic_wall']
    originals={(d['pid'],d['starttime_ticks']):d for d in first['processes']};finals={(d['pid'],d['starttime_ticks']):d for d in last['processes']}
    cpu=[]
    for key in originals.keys()&finals.keys():
        a,b=originals[key],finals[key];value=b['cpu_seconds']-a['cpu_seconds']
        cpu.append(dict(pid=key[0],starttime_ticks=key[1],comm=a['comm'],CPU_seconds=value,wall_seconds=end-start,
            average_CPU_cores=value/(end-start),utime_ticks_delta=b['utime_ticks']-a['utime_ticks'],stime_ticks_delta=b['stime_ticks']-a['stime_ticks'],
            first_thread_count=a['threads'],last_thread_count=b['threads'],affinity_cpus=a['affinity_cpus']))
    begins={};propagated={};sums=defaultdict(float);counts=defaultdict(int);intervals=defaultdict(list)
    with (run/'fastlivo_diagnostics/records.bin').open('rb')as f,(run/'fastlivo_diagnostics/index.csv').open()as ix:
        for row in csv.DictReader(ix):
            kind=int(row['kind'])
            if kind not in(10,11,12,13):continue
            f.seek(int(row['offset']));head=f.read(56);h=HEADER.unpack(head);payload=f.read(int(row['n_values'])*8)
            a=struct.unpack('<'+str(h[-1])+'d',payload)
            if h[0]!=kind:raise ValueError('binary/index mismatch')
            label='LIO'if h[3]==2 else'VIO';seq=h[1]
            if kind==10:begins[seq]=a[0]
            elif kind==11:
                propagated[seq]=a[0]
                if seq in begins and start<=begins[seq]<=a[0]<=end:
                    k=label+'_Process2_plus_gravityAlignment';sums[k]+=a[0]-begins[seq];counts[k]+=1
            elif a[0]and seq in propagated and start<=propagated[seq]<=a[1]<=end:
                k=label+'_post_Process2_to_solve_end';sums[k]+=a[1]-propagated[seq];counts[k]+=1
                if seq in begins:intervals[label].append((begins[seq],a[1]))
    return dict(schema='actual_same_wall_window_proc_and_pipeline_cost/v1',run=str(run),
        same_physical_sample_wall_window_s=[start,end],actual_last_Actor_read_boundary=boundary,
        observed_state_sim_times=[first['state'].get('sim_time')if first['state']else None,last['state'].get('sim_time')if last['state']else None],
        earlier_CPU_epochs='not sampled; cannot reconstruct process CPU seconds from lifecycle ps percent',
        original_per_process_CPU_deltas=sorted(cpu,key=lambda d:-d['CPU_seconds']),
        recorded_in_window_pipeline_wall_segments={k:dict(records=counts[k],wall_seconds=sums[k],fraction_of_same_wall_window=sums[k]/(end-start))for k in sums},
        processed_phase_progress_in_same_wall_window={k:dict(processed_phases=len(v),phases_per_wall_second=len(v)/(end-start))for k,v in intervals.items()},
        scope='Original /proc CPU seconds summed across actual process threads, distinct from stage wall seconds. Pipeline segments end at solve and do not isolate map updates/cloud coloring/publication/DDS wait. Missing remaining cost is mixed and cannot be called pure communication. No full diagnostic binary scan/hash.',
        reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();d=evaluate(a.run.resolve());a.output.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');print(json.dumps(d,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
