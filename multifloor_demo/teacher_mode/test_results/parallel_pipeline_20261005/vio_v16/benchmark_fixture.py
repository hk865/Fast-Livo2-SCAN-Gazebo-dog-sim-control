#!/usr/bin/env python3
"""Alternating finite VIO update benchmarks, no ROS or physics."""
from pathlib import Path
import hashlib,json,math,os,statistics,subprocess,time
OUT=Path(__file__).resolve().parent
BUILD=json.loads((OUT/'fixture_build_receipt.json').read_text())
AFFINITY='0,2,4,6'
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
 vals=[v for r in rows for v in r['milliseconds']]
 return {'valid':len(vals),'warmups_per_batch':12,'median_ms':statistics.median(vals),'p95_ms':percentile(vals,.95),'min_ms':min(vals),'max_ms':max(vals),'mean_ms':statistics.mean(vals)}
bench=OUT/'benchmark_provisional64';bench.mkdir(exist_ok=False)
(bench/'resource_before.json').write_text(json.dumps(resource_snapshot(),indent=2)+'\n')
all_rows=[];comparisons=[];expected={}
for candidate in ['candidate_V16_t4','candidate_V16_t1']:
 for mode in ['forward','inverse']:
  for n in [32,63,128,512,1024]:
   case=f'{mode}_n{n}_level1_searchmixed_exposure'
   by_variant={'baseline_V12':[],candidate:[]}
   for batch in range(8):
    order=['baseline_V12',candidate]if batch%2==0 else[candidate,'baseline_V12']
    for position,variant in enumerate(order):
     d=bench/candidate/case/f'batch{batch:02d}_{position}_{variant}';d.mkdir(parents=True)
     meta=BUILD['variants'][variant];env={**os.environ,**meta['env']}
     for key in ['LD_PRELOAD','V16_TEST_TEAM_RECORDS','FASTLIVO_DIAGNOSTIC_DIR']:env.pop(key,None)
     env.update(FASTLIVO_BOUNDARY_TIMING='0',FASTLIVO_DIAGNOSTIC_BEGIN='115',FASTLIVO_DIAGNOSTIC_END='118')
     if variant!='baseline_V12':env['LD_LIBRARY_PATH']=str(OUT/variant)+':'+env['LD_LIBRARY_PATH']
     cmd=['taskset','-c',AFFINITY,str(OUT/variant/'fixture'),mode,str(n),'1','0','1','1','0','0','0','0','4',str(d/'output.bin')]
     before=resource_snapshot();r=subprocess.run(cmd,env=env,text=True,capture_output=True);after=resource_snapshot()
     (d/'stdout.log').write_text(r.stdout);(d/'stderr.log').write_text(r.stderr)
     if r.returncode:raise RuntimeError((case,variant,r.returncode,r.stderr))
     parsed=json.loads(r.stdout.splitlines()[-1]);digest=sha(d/'output.bin')
     if case not in expected:expected[case]=digest
     if digest!=expected[case]:raise AssertionError(('Bit difference',case,variant,batch,digest,expected[case]))
     row={'case':case,'batch':batch,'position':position,'variant':variant,'argv':cmd,'returncode':0,
          'library_sha256':meta['lib_sha256'],'executable_sha256':meta['executable_sha256'],
          'output_sha256':digest,'warmups':parsed['warmups'],'milliseconds':parsed['milliseconds'],
          'resources_before':before,'resources_after':after}
     assert row['warmups']>=10 and len(row['milliseconds'])==4
     by_variant[variant].append(row);all_rows.append(row)
   a=summarize(by_variant['baseline_V12']);b=summarize(by_variant[candidate])
   comparisons.append({'case':case,'mode':mode,'points':n,'candidate':candidate,'baseline':a,'candidate_timings':b,
       'median_improvement_fraction':1-b['median_ms']/a['median_ms'],'p95_ratio':b['p95_ms']/a['p95_ms'],
       'large_median_10pct_and_p95_nonregression':n>=512 and b['median_ms']<=.9*a['median_ms'] and b['p95_ms']<=a['p95_ms'],
       'all_outputs_byte_identical':True,'batch_order':'AB/BA alternating, 8 batches each 4 valid after 12 warmups'})
(bench/'resource_after.json').write_text(json.dumps(resource_snapshot(),indent=2)+'\n')
report={'schema':'independent_V16_synthetic_patch_benchmark/v1','status':'MEASURED_NOT_RUNTIME_ACCEPTANCE',
 'fixture_source_sha256':BUILD['fixture_source_sha256'],'build_receipt_sha256':sha(OUT/'fixture_build_receipt.json'),
 'numeric_receipt_sha256':sha(OUT/'numeric_comparison.json'),'affinity':AFFINITY,
 'threads_not_preloaded_or_overridden':True,'diagnostic_detail_window':'115..118; fixture context130.1 outside',
 'actualteam_witness_source':str(OUT/'numeric_comparison.json'),'comparisons':comparisons,'batches':all_rows,
 'limits':'Synthetic updateState/updateStateInverse kernel+solver; not processFrame, original trajectory, or complete30Hz pipeline. Serial baseline includes unchanged Eigen behavior. Competing workload recorded; no Gazebo/build during this window.'}
(OUT/'benchmark_provisional64.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'batches':len(all_rows),'comparisons':comparisons},indent=2))
