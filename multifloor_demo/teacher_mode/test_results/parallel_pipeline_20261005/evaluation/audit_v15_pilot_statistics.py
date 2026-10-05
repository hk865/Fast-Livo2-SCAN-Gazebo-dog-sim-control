#!/usr/bin/env python3
"""Read-only statistics for the first V15 pilot; never treat inner repetitions as batches."""
from __future__ import annotations
import collections,hashlib,json,math
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
DATA=HERE.parent/'lio_v15'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(values):
    a=np.asarray(values,dtype=float)
    if len(a)==0 or not np.isfinite(a).all()or (a<=0).any():raise ValueError('Invalid timing samples')
    return dict(samples=len(a),median_ms=float(np.median(a)),p95_ms=float(np.percentile(a,95)))
def main():
    rawpath=DATA/'benchmark_results.json';raw=json.loads(rawpath.read_text());groups=collections.OrderedDict()
    for row in raw['batches']:
        groups.setdefault((row['group'],row['input_sha256']),[]).append(row)
    scenes=[];all_same=[]
    for (kind,inputhash),batches in groups.items():
        variants=collections.defaultdict(list)
        for b in batches:variants[b['variant']].append(b)
        if set(variants)!={'baseline_V12','candidate_V15_T4'}:raise ValueError('Unexpected variant set')
        scene=(Path(batches[0]['input']).stem if kind=='real_rows' else 'synthetic'+str(batches[0]['samples']['points']))
        order=[b['variant']for b in batches]
        expected=['baseline_V12','candidate_V15_T4','candidate_V15_T4','baseline_V12','candidate_V15_T4','baseline_V12','baseline_V12','candidate_V15_T4']
        if order!=expected:raise ValueError('Pilot order not ABBA+BAAB')
        summary={};outputs=set()
        for v,rows in variants.items():
            wall=[];cpu=[];state=[]
            for row in rows:
                s=row['samples'];warm=s.get('warmup',s.get('jacobian_warmup'))
                if warm<10 or s['repetitions']<30:raise ValueError('Warmup or inner measurement count insufficient')
                wall.extend(s['wall_ms']if kind=='real_rows'else s['JacobianRows_ms'])
                cpu.extend(s['process_cpu_ms']if kind=='real_rows'else s['JacobianRows_process_cpu_ms'])
                state.extend(s.get('StateEstimation_ms',[]));outputs.add(row['output_sha256'])
                if row['before'].get('P_core_affinity')!='0-7'or row['after'].get('P_core_affinity')!='0-7':raise ValueError('Pilot P mask differs')
                for edge in ('before','after'):
                    if not row[edge].get('cpu_cur_freq_khz')or not row[edge].get('procstat')or not row[edge].get('loadavg'):raise ValueError('Missing frequency/load witness')
            summary[v]=dict(row_wall=stats(wall),row_process_cpu=stats(cpu),independent_process_groups=len(rows),inner_repetitions=len(wall),warmups_per_process_minimum=min(r['samples'].get('warmup',r['samples'].get('jacobian_warmup'))for r in rows),source_frequency_and_other_load_recorded=True)
            if state:summary[v]['whole_synthetic_StateEstimation']=stats(state)
        a=summary['baseline_V12']['row_wall'];b=summary['candidate_V15_T4']['row_wall']
        same=len(outputs)==1;all_same.append(same)
        scenes.append(dict(scene=scene,input_sha256=inputhash,order=order,statistics=summary,fresh_recorded_output_hashes_identical=same,output_payloads_rehashed_in_this_reader=False,
                           median_improvement_fraction=1-b['median_ms']/a['median_ms'],p95_no_worse=b['p95_ms']<=a['p95_ms'],
                           numerical_and_performance_scope='Limited row component pilot; source and actual bytes are separate numeric proof, no full pipeline pass',
                           independent_process_batch_gate_passed=all(x['independent_process_groups']>=30 for x in summary.values())))
    receipt=dict(schema='independent_v15_pilot_statistics/v1',status='UNVERIFIED_PILOT_NOT_CONFIRMATORY',
                 meaning='Performance observations retained. Per-scene/per-variant process-batch count is insufficient; no candidate overall pass. Source and byte closure are not asserted from claimed output hashes alone.',
                 total_process_groups=len(raw['batches']),scenes=scenes,all_fresh_recorded_output_hashes_identical=all(all_same),
                 historical_recorded_HTH_replay='FAILED_BITWISE, baseline lifted loop and candidate T1/T4 share same fresh values; historical context cause remains UNVERIFIED, not a new candidate numerical exemption',
                 small_serial_fallback='Source declared below256 constraints; actual fallback/team witness pending, observed p95 regressions retained',
                 plan_sha256=sha(HERE/'PROSPECTIVE_EVALUATION_PLAN.json'),batch_definition_sha256=sha(HERE/'PROSPECTIVE_INDEPENDENT_BATCH_DEFINITION.json'),
                 verified_read_file_sha256={str(rawpath):sha(rawpath),str(DATA/'benchmark_summary.json'):sha(DATA/'benchmark_summary.json')},evaluator_sha256=sha(Path(__file__)))
    out=HERE/'V15_PILOT_STATISTICS_INDEPENDENT.json'
    with out.open('x')as f:f.write(json.dumps(receipt,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'path':str(out),'sha256':sha(out),'status':receipt['status'],'scenes':[{'scene':x['scene'],'improvement':x['median_improvement_fraction'],'p95_no_worse':x['p95_no_worse'],'groups_each':4}for x in scenes]}))
if __name__=='__main__':main()
