#!/usr/bin/env python3
"""Only reads frozen V16 trials; performs no estimator execution."""
from pathlib import Path
import hashlib,json,collections,math,datetime
import numpy as np
E=Path(__file__).resolve().parent;A=E.parent/'vio_v16';T=E.parents[2];inputs={};readbytes=0
def sha(p):
 global readbytes
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b);readbytes+=len(b)
 return h.hexdigest()
def read(p):
 p=Path(p);inputs[str(p)]=sha(p);return json.loads(p.read_text())
def bind(p,expected):
 got=sha(p);assert got==expected,(str(p),got,expected);inputs[str(Path(p))]=got
pplan=read(E/'PROSPECTIVE_EVALUATION_PLAN.json');read(E/'PROSPECTIVE_INDEPENDENT_BATCH_DEFINITION.json');d=read(A/'benchmark_confirmatory32.json');numeric=read(A/'numeric_comparison.json');build=read(A/'fixture_build_receipt.json');team=read(A/'team_partition_proof.json');delta=read(A/'source_delta_audit.json');baseline=read(A/'source_copy_baseline.json')
assert d['process_batches']==768 and len(d['batches'])==768 and d['independent_batches_per_scene_and_variant']==32 and d['timing_no_preload_or_team_override']
bind(A/'vio_patch_fixture.cpp',d['fixture_source_sha256']);assert inputs[str(A/'fixture_build_receipt.json')]==d['build_receipt_sha256'];assert inputs[str(A/'numeric_comparison.json')]==d['numeric_receipt_sha256']==team['numeric_sha256']
for v,b in build['variants'].items():
 bind(b['lib_path'],b['lib_sha256']);a=b['compile_argv'];bind(a[a.index('-o')+1],b['executable_sha256']);assert b['returncode']==0 and not any(x in a for x in ['-ffast-math','-Ofast','-march=native'])
# Exact changed-source bounds against recorded ordinary-copy inventory.
rootnew=T/'navigation/parallel_vio_v16';rootold=T/'navigation/lidar_sampling_v12';changed={x['path']:x for x in delta['changed']};actual_changed=[]
for rel,original in baseline['original_source_copy_hashes'].items():
 if not rel.startswith('slam_ws/src/') or not (rootnew/rel).is_file():continue
 bind(rootold/rel,original);after=sha(rootnew/rel);inside=rel[len('slam_ws/src/'):]
 if after!=original:
  actual_changed.append(inside);assert inside in changed and original==changed[inside]['before'] and after==changed[inside]['after']
 inputs[str(rootnew/rel)]=after
assert set(actual_changed)==set(changed)=={'fast_livo2_core/CMakeLists.txt','fast_livo2_core/src/vio.cpp'}
# No hidden skip/solve modifications claimed: limited exact solver/output fixtures are independently bound below.
assert numeric['process_runs']==123 and numeric['fixtures']==41 and len(numeric['comparisons'])==82
assert all(x['state_cov_G_H_errors_reference_H_byte_identical'] and x['original_nonwall_diagnostic_records_byte_identical']for x in numeric['comparisons'])
for variant,cases in numeric['results'].items():
 assert len(cases)==41
 for case,r in cases.items():assert r['returncode']==0;bind(r['argv'][-1],r['output_sha256'])
for case in numeric['results']['baseline_V12']:
 assert len({numeric['results'][v][case]['output_sha256']for v in numeric['variants']})==1
assert not team['actual_i_trace_claimed'] and len(team['regions'])>0
for region in team['regions']:
 n=region['points'];ranges=region['compiled_static_schedule_ranges'];actual=region['observed_team']['actual_team'];assert actual==len(ranges) and region['covers_all_i_once'];assert sum(r['end_i_exclusive']-r['first_i']for r in ranges)==n
 assert ranges[0]['first_i']==0 and ranges[-1]['end_i_exclusive']==n and all(a['end_i_exclusive']==b['first_i']for a,b in zip(ranges,ranges[1:]));assert all(x in [0,2,4,6]for x in region['observed_team']['cpus'])
groups=collections.defaultdict(list);destinations=set();scene_orders=collections.defaultdict(list)
for row in d['batches']:
 assert row['returncode']==0 and row['warmups']>=10 and len(row['milliseconds'])==30 and all(math.isfinite(x)and x>0 for x in row['milliseconds'])
 v=row['variant'];b=build['variants'][v];assert row['library_sha256']==b['lib_sha256'] and row['executable_sha256']==b['executable_sha256'];assert row['argv'][:3]==['taskset','-c','0,2,4,6'];out=Path(row['argv'][-1]);assert out not in destinations;destinations.add(out);bind(out,row['output_sha256'])
 for phase in ['resources_before','resources_after']:
  r=row[phase];assert r['affinity_cpus']=='0,2,4,6' and r['cpu_freq_khz'] and r['loadavg'] and r['proc_stat']
 assert row['cycle_order'][row['position']]==v
 scene_orders[row['case']].append(row);groups[row['case'],v].append(row)
assert len(groups)==24 and all(len(x)==32 for x in groups.values()) and len(scene_orders)==8
for case,rows in scene_orders.items():
 assert len(rows)==96 and len({x['output_sha256']for x in rows})==1
 for cycle in range(16):
  rs=[x for x in rows if x['cycle']==cycle];assert len(rs)==6 and [x['position']for x in rs]==list(range(6));expected=['baseline_V12','candidate_V16_t1','candidate_V16_t4','candidate_V16_t4','candidate_V16_t1','baseline_V12']if cycle%2==0 else['candidate_V16_t4','candidate_V16_t1','baseline_V12','baseline_V12','candidate_V16_t1','candidate_V16_t4'];assert [x['variant']for x in rs]==expected
stats={}
for (case,v),rows in groups.items():
 arrays=[x['milliseconds']for x in rows];meds=[float(np.median(a))for a in arrays];p95s=[float(np.percentile(a,95))for a in arrays];s={'independent_batches':len(rows),'nested_samples':sum(map(len,arrays)),'batch_median_ms':float(np.median(meds)),'batch_median_p95_ms':float(np.percentile(meds,95)),'median_within_batch_p95_ms':float(np.median(p95s)),'pooled_median_ms':float(np.median([t for a in arrays for t in a])),'pooled_p95_ms':float(np.percentile([t for a in arrays for t in a],95))};stats[case,v]=s
 declared=d['case_summaries'][case][v];assert math.isclose(s['batch_median_ms'],declared['independent_batch_median_ms'],rel_tol=1e-14)and math.isclose(s['batch_median_p95_ms'],declared['independent_batch_median_p95_ms'],rel_tol=1e-14)and math.isclose(s['median_within_batch_p95_ms'],declared['median_of_within_batch_p95_ms'],rel_tol=1e-14)
comparisons=[]
for case in sorted(scene_orders):
 n=int(case.split('_n')[1].split('_')[0]);a=stats[case,'baseline_V12']
 for v in ['candidate_V16_t1','candidate_V16_t4']:
  b=stats[case,v];im=1-b['batch_median_ms']/a['batch_median_ms'];pchecks={k:b[k]<=a[k]for k in ['batch_median_p95_ms','median_within_batch_p95_ms','pooled_p95_ms']};minimum_required=n>=512;gate=(not minimum_required or im>=.1)and all(pchecks.values());comparisons.append({'case':case,'patches':n,'candidate':v,'baseline':a,'candidate_metrics':b,'batch_median_improvement_fraction':im,'large_kernel_10percent_required':minimum_required,'p95_no_regression_views':pchecks,'performance_gate_passed':gate})
status={v:all(x['performance_gate_passed']for x in comparisons if x['candidate']==v)for v in ['candidate_V16_t1','candidate_V16_t4']}
result={'schema':'independent_V16_confirmatory_component_audit/v1','status':'PASS_STATISTICS_T1_ONLY_FP_PROOF_PENDING','statistics_gate':status,'selected_variant':'candidate_V16_t1','effective_threads_selected':1,'fresh_numerical_outputs_123_rehashed_and_bitexact':True,'canonical_benchmark_outputs_768_rehashed':True,'mandatory_batch_size_and_ABBA_BAAB_verified':True,'compiled_static_team_partition_verified':True,'source_changed_only_two_declared_files':True,'original_numeric_gates_unchanged':True,'actual_FP_mode_receipt':'PENDING_EXPLICIT_MAIN_WORKER_OBSERVATION; source/compiler do not alone prove runtime FP modes','comparisons':comparisons,'single_source_T4_vs_T1_gain':[{ 'case':case,'fraction':1-stats[case,'candidate_V16_t4']['batch_median_ms']/stats[case,'candidate_V16_t1']['batch_median_ms']}for case in sorted(scene_orders)],'not_claimed':['all benefit caused by four threads','complete processFrame or trajectory replay','actual navigation PASS'],'limits':d['limits'],'actual_physical_navigation':'UNVERIFIED_NOT_RUN_V16','verified_input_sha256':inputs,'auditor_sha256':sha(__file__),'total_bytes_hashed':readbytes}
p=E/'V16_CONFIRMATORY_INDEPENDENT.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'statistics_gate':status,'receipt':str(p),'sha256':sha(p),'failed':[{ 'case':x['case'],'candidate':x['candidate'],'median_improvement':x['batch_median_improvement_fraction'],'p95_gates':x['p95_no_regression_views']}for x in comparisons if not x['performance_gate_passed']],'bytes_hashed':readbytes}))
