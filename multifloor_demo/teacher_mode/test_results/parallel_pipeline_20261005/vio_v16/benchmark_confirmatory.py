#!/usr/bin/env python3
"""32 independent process batches/scene/variant; CPU-only and synthetic."""
from pathlib import Path
import hashlib,json,math,os,statistics,subprocess,time
OUT=Path(__file__).resolve().parent
BUILD=json.loads((OUT/'fixture_build_receipt.json').read_text())
AFFINITY='0,2,4,6'
VARIANTS=['baseline_V12','candidate_V16_t1','candidate_V16_t4']
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def resource_snapshot():
 d={'monotonic_wall':time.monotonic(),'loadavg':Path('/proc/loadavg').read_text().strip(),
    'proc_stat':Path('/proc/stat').read_text(),'affinity_cpus':AFFINITY,'cpu_freq_khz':{}}
 for cpu in map(int,AFFINITY.split(',')):
  p=Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq')
  d['cpu_freq_khz'][str(cpu)]=p.read_text().strip()if p.exists()else None
 return d
def percentile(values,p):
 s=sorted(values);x=(len(s)-1)*p;i=math.floor(x);return s[i]+(s[min(i+1,len(s)-1)]-s[i])*(x-i)
def summarize(rows):
 medians=[statistics.median(r['milliseconds'])for r in rows]
 p95s=[percentile(r['milliseconds'],.95)for r in rows]
 vals=[v for r in rows for v in r['milliseconds']]
 return {'independent_process_batches':len(rows),'valid_per_batch':30,'warmups_per_batch':12,
         'independent_batch_median_ms':statistics.median(medians),
         'independent_batch_median_p95_ms':percentile(medians,.95),
         'median_of_within_batch_p95_ms':statistics.median(p95s),
         'within_process_total_samples':len(vals),'all_sample_median_ms':statistics.median(vals),
         'all_sample_p95_ms':percentile(vals,.95),'batch_medians_ms':medians,'batch_p95_ms':p95s}
bench=OUT/'benchmark_confirmatory32';bench.mkdir(exist_ok=False)
(bench/'resource_before.json').write_text(json.dumps(resource_snapshot(),indent=2)+'\n')
all_rows=[];comparisons=[];expected={};case_summaries={}
for mode in ['forward','inverse']:
 for n in [32,128,512,1024]:
  case=f'{mode}_n{n}_level1_searchmixed_exposure';by_variant={v:[]for v in VARIANTS}
  for cycle in range(16):
   a,b,c=VARIANTS;order=[a,b,c,c,b,a]if cycle%2==0 else[c,b,a,a,b,c]
   for position,variant in enumerate(order):
    index=len(by_variant[variant]);d=bench/case/f'cycle{cycle:02d}_{position}_{variant}';d.mkdir(parents=True)
    meta=BUILD['variants'][variant];env={**os.environ,**meta['env']}
    for key in ['LD_PRELOAD','V16_TEST_TEAM_RECORDS','FASTLIVO_DIAGNOSTIC_DIR']:env.pop(key,None)
    env.update(FASTLIVO_BOUNDARY_TIMING='0',FASTLIVO_DIAGNOSTIC_BEGIN='115',FASTLIVO_DIAGNOSTIC_END='118')
    if variant!='baseline_V12':env['LD_LIBRARY_PATH']=str(OUT/variant)+':'+env['LD_LIBRARY_PATH']
    cmd=['taskset','-c',AFFINITY,str(OUT/variant/'fixture'),mode,str(n),'1','0','1','1','0','0','0','0','30',str(d/'output.bin')]
    before=resource_snapshot();r=subprocess.run(cmd,env=env,text=True,capture_output=True);after=resource_snapshot()
    (d/'stdout.log').write_text(r.stdout);(d/'stderr.log').write_text(r.stderr)
    if r.returncode:raise RuntimeError((case,variant,r.returncode,r.stderr))
    parsed=json.loads(r.stdout.splitlines()[-1]);digest=sha(d/'output.bin')
    if case not in expected:expected[case]=digest
    if digest!=expected[case]:raise AssertionError(('Bit difference',case,variant,cycle,digest,expected[case]))
    row={'case':case,'independent_batch_index':index,'cycle':cycle,'position':position,'cycle_order':order,
         'variant':variant,'argv':cmd,'returncode':0,'library_sha256':meta['lib_sha256'],
         'executable_sha256':meta['executable_sha256'],'output_sha256':digest,'warmups':parsed['warmups'],
         'milliseconds':parsed['milliseconds'],'resources_before':before,'resources_after':after}
    assert row['warmups']>=10 and len(row['milliseconds'])==30
    by_variant[variant].append(row);all_rows.append(row)
  summaries={v:summarize(by_variant[v])for v in VARIANTS};case_summaries[case]=summaries
  assert all(s['independent_process_batches']==32 for s in summaries.values())
  for reference,candidate in [('baseline_V12','candidate_V16_t1'),('baseline_V12','candidate_V16_t4'),('candidate_V16_t1','candidate_V16_t4')]:
   a=summaries[reference];b=summaries[candidate]
   gain=1-b['independent_batch_median_ms']/a['independent_batch_median_ms']
   p95=b['independent_batch_median_p95_ms']/a['independent_batch_median_p95_ms']
   innerp95=b['median_of_within_batch_p95_ms']/a['median_of_within_batch_p95_ms']
   comparisons.append({'case':case,'mode':mode,'points':n,'reference':reference,'candidate':candidate,
        'baseline':a,'candidate_timings':b,'median_improvement_fraction':gain,'batch_median_p95_ratio':p95,
        'within_batch_p95_median_ratio':innerp95,'prospective_large_median_10pct_and_p95_nonregression':
        n>=512 and gain>=.1 and p95<=1 and innerp95<=1,'all_outputs_byte_identical':True})
(bench/'resource_after.json').write_text(json.dumps(resource_snapshot(),indent=2)+'\n')
report={'schema':'independent_V16_synthetic_patch_confirmatory_benchmark/v1','status':'MEASURED_NOT_RUNTIME_ACCEPTANCE',
 'fixture_source_sha256':BUILD['fixture_source_sha256'],'build_receipt_sha256':sha(OUT/'fixture_build_receipt.json'),
 'numeric_receipt_sha256':sha(OUT/'numeric_comparison.json'),'affinity':AFFINITY,
 'process_batches':len(all_rows),'warmups_per_batch':12,'valid_samples_per_batch':30,'independent_batches_per_scene_and_variant':32,
 'paired_order':'16 six-process cycles A B C C B A / C B A A B C; projection to every two variants is ABBA/BAAB',
 'statistical_unit':'independent process batch; 30 repeated measurements per process are nested samples, not independent batches',
 'timing_no_preload_or_team_override':True,'case_summaries':case_summaries,'comparisons':comparisons,'batches':all_rows,
 'limits':'Synthetic complete updateState/updateStateInverse only; no processFrame, actual image/submap replay, or ROS/30Hz chain. Same source t1 controls code-layout/compilation changes; t4 gains relative V12 cannot be wholly attributed to threads.'}
(OUT/'benchmark_confirmatory32.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'batches':len(all_rows),'comparisons':[{k:x[k]for k in ['case','reference','candidate','median_improvement_fraction','batch_median_p95_ratio','within_batch_p95_median_ratio','prospective_large_median_10pct_and_p95_nonregression']}for x in comparisons]},indent=2))
