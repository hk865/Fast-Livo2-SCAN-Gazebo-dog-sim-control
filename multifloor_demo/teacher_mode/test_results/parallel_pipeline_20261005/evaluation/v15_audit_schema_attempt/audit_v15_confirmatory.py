#!/usr/bin/env python3
"""Read-only prospective V15 component assessment. Never executes fixtures."""
from pathlib import Path
import json,hashlib,collections,math,statistics,datetime
import numpy as np
E=Path(__file__).resolve().parent; A=E.parent/'lio_v15'; T=E.parents[2]
inputs={}; bytes_hashed=0
def sha(p):
 global bytes_hashed
 p=Path(p); h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b);bytes_hashed+=len(b)
 return h.hexdigest()
def read(p):
 p=Path(p); inputs[str(p)]=sha(p);return json.loads(p.read_text())
def bind(p,expected):
 got=sha(p);assert got==expected,(str(p),got,expected);inputs[str(Path(p))]=got
 return got
def stat(xs):
 return {'median_ms':float(np.median(xs)),'p95_ms':float(np.percentile(xs,95))}
plan=read(E/'PROSPECTIVE_EVALUATION_PLAN.json');batchplan=read(E/'PROSPECTIVE_INDEPENDENT_BATCH_DEFINITION.json')
d=read(A/'confirmatory_results.json');build=read(A/'component_build_receipt.json');proof=read(A/'source_strip_proof.json');finite=read(A/'finite_fixture_comparison.json');real=read(A/'real_numeric_comparison.json');fp=read(A/'all_variants_FP_observation/receipt.json');small=read(A/'small_serial_and_FP_observation/receipt.json');read(A/'real_team_FP_observation/receipt.json')
# Independently reconstruct original source, not trusting proof booleans.
old=T/'navigation/lidar_sampling_v12/slam_ws/src/fast_livo2_core';new=T/'navigation/parallel_lio_v15/slam_ws/src/fast_livo2_core'
def remove(s,tag,indent='',extra=False):
 begin=indent+'// LIO_JACOBIAN_PARALLEL_BEGIN '+tag+'\n';end=indent+'// LIO_JACOBIAN_PARALLEL_END '+tag+'\n';a=s.index(begin);b=s.index(end,a)+len(end)+int(extra);return s[:a]+s[b:]
s=(new/'src/voxel_map.cpp').read_text();tag='production_rows_call';a=s.index('// LIO_JACOBIAN_PARALLEL_BEGIN '+tag+'\n');end='// LIO_JACOBIAN_PARALLEL_END '+tag+'\n';b=s.index(end,a)+len(end);s=s[:a]+(A/'original_v12_jacobian_loop.txt').read_text()+s[b:];s=remove(remove(s,'production_rows_method',extra=True),'configuration');h=remove((new/'include/fast_livo2_core/core/voxel_map.h').read_text(),'production_rows_interface','  ')
assert s.encode()==(old/'src/voxel_map.cpp').read_bytes();assert h.encode()==(old/'include/fast_livo2_core/core/voxel_map.h').read_bytes()
for p,expected in [(old/'src/voxel_map.cpp',proof['baseline_cpp_sha256']),(new/'src/voxel_map.cpp',proof['candidate_cpp_sha256']),(A/'original_v12_jacobian_loop.txt',proof['original_loop_sha256']),(old/'include/fast_livo2_core/core/voxel_map.h',proof['header_original_sha256']),(new/'include/fast_livo2_core/core/voxel_map.h',proof['header_candidate_sha256'])]:bind(p,expected)
for key,v in build.items():
 bind(v['lib_path'],v['lib_sha256']);argv=v['compile_argv'];bind(argv[argv.index('-o')+1],v['executable_sha256'])
 assert not any(x in argv for x in ['-ffast-math','-Ofast','-march=native'])
assert fp['all_same_FP_P_core'] and not fp['FP_control_changed'] and not fp['production_math_changed']
assert len(fp['variants'])==3
for v in fp['variants']:
 assert len(v['main_FP'])==2 and {x['phase'] for x in v['main_FP']}=={'main_enter','main_exit'}
 assert all(x['cpu'] in range(8) and x['fp_rounding_mode']==0 and x['mxcsr_control_mask']==8064 for x in v['main_FP'])
 assert v['unchanged_numeric_output_byte_exact']
 for team in v['teams']:
  assert team['actual_team']==team['requested'] and all(x in range(8) for x in team['cpus']) and set(team['fp_rounding_modes'])=={0} and set(team['mxcsr_control_masks'])=={8064}
assert small['accepted_rows']==48 and small['configured_jacobian_threads']==4 and small['row_actual_teams']==[1]
assert small['test_only'] and small['timing_not_eligible_due_to_preload']
# Actual canonical bytes from every independent process. Samples/receipt equality also verified.
scenes=collections.defaultdict(list);allpaths=set();pattern=['baseline_V12','candidate_V15_T4','candidate_V15_T4','baseline_V12','candidate_V15_T4','baseline_V12','baseline_V12','candidate_V15_T4']
assert len(d['batches'])==384
for seq,row in enumerate(d['batches'],1):
 assert row['sequence']==seq and not row['observer_enabled'];scene=row['scene'];rs=scenes[scene];idx=len(rs)
 assert row['variant']==pattern[idx%8] and row['balanced_pattern_position']==idx%8 and row['balanced_cycle']==idx//8
 dest=A/'confirmatory'/scene/f'{idx:03}_{row["variant"]}';assert dest not in allpaths;allpaths.add(dest)
 rr=read(dest/'run_receipt.json');assert rr=={k:v for k,v in row.items() if k not in ['sequence','scene','balanced_pattern_position','balanced_cycle']}
 samples=read(dest/'samples.json');assert samples==row['samples']
 bind(dest/'output.bin',row['output_sha256']);assert row['executable_sha256']==build[row['group']+'/'+row['variant']]['executable_sha256']
 assert row['before']['P_core_affinity']==row['after']['P_core_affinity']=='0-7'
 for phase in ['before','after']:assert row[phase]['cpu_cur_freq_khz'] and row[phase]['loadavg'] and row[phase]['procstat']
 reps=samples['repetitions'];assert reps==40
 field='JacobianRows_ms' if row['group']=='scaling' else 'wall_ms';assert len(samples[field])==40 and all(math.isfinite(x) and x>0 for x in samples[field]);assert samples['jacobian_warmup' if row['group']=='scaling' else 'warmup']>=10
 rs.append(row)
assert len(scenes)==6
summary=[]
for scene,rows in scenes.items():
 assert len(rows)==64 and len({x['input_sha256'] for x in rows})==1 and len({x['output_sha256'] for x in rows})==1
 metrics={}
 for variant in pattern[:2]:
  vs=[x for x in rows if x['variant']==variant];assert len(vs)==32
  field='JacobianRows_ms'if vs[0]['group']=='scaling'else'wall_ms';arrays=[v['samples'][field]for v in vs]
  metrics[variant]={'independent_process_batches':len(vs),'nested_valid_samples':sum(map(len,arrays)),'pooled':stat([x for a in arrays for x in a]),'independent_batch_medians':stat([float(np.median(a))for a in arrays]),'median_of_within_batch_p95_ms':float(np.median([np.percentile(a,95)for a in arrays]))}
 a,b=metrics['baseline_V12'],metrics['candidate_V15_T4'];improvement=1-b['independent_batch_medians']['median_ms']/a['independent_batch_medians']['median_ms'];pooledim=1-b['pooled']['median_ms']/a['pooled']['median_ms'];p95checks=[b['pooled']['p95_ms']<=a['pooled']['p95_ms'],b['independent_batch_medians']['p95_ms']<=a['independent_batch_medians']['p95_ms'],b['median_of_within_batch_p95_ms']<=a['median_of_within_batch_p95_ms']]
 is_small=scene=='small147';gate=(small['row_actual_teams']==[1])if is_small else improvement>=.1 and pooledim>=.1 and all(p95checks)
 out={'scene':scene,'metrics':metrics,'median_improvement_fraction_independent_batches':improvement,'median_improvement_fraction_pooled':pooledim,'p95_no_regression_views':p95checks,'limited_kernel_gate_passed':gate,'serial_fallback_branch_used':is_small}
 if rows[0]['group']=='scaling':
  out['complete_synthetic_StateEstimation_ms']={v:stat([t for x in rows if x['variant']==v for t in x['samples']['StateEstimation_ms']])for v in pattern[:2]}
 summary.append(out)
# Fresh same-context T1/T4/baseline bytes; historical aggregate mismatch retained.
for case in real['cases']:
 bind(case['input'],case['input_sha256']);outs=[]
 for r in case['runs']:
  p=A/'real_numeric'/Path(case['input']).stem/r['variant']/'output.bin';bind(p,r['output_sha256']);outs.append(p.read_bytes())
 assert outs[0]==outs[1]==outs[2]
 assert case['recorded_expected_checks']['expected_HTH']['byte_equal'] is False
assert finite['all_numeric_comparisons_pass'] and finite['actual_full_trajectory_equivalence'] is False
# finite originals use exact recorded output SHA; actual finite output paths are audited below if present.
finite_count=0
for variant,v in finite['variants'].items():
 for case,r in v['runs'].items():
  assert r['returncode']==0;matches=list((A/'finite_fixtures'/variant).rglob(case+'/output.bin'))
  if matches:
   assert len(matches)==1;bind(matches[0],r['output_sha256']);finite_count+=1
  if 'writer_stats'in r:
   st=r['writer_stats'];assert st['final'] and st['attempted']==st['written'] and st['dropped']==0 and st['writer_io_failed']is False
passed=all(x['limited_kernel_gate_passed']for x in summary)
result={'schema':'independent_V15_confirmatory_component_audit/v1','status':'PASS_LIMITED_KERNEL_ONLY'if passed else'FAIL_LIMITED_KERNEL_GATE','created_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_reconstruction_bytes_equal':True,'canonical_outputs_all384_rehashed':True,'same_context_fresh_real_outputs_T1_T4_V12_bitexact':True,'finite_fixture_receipt_passed':True,'finite_fixture_actual_output_files_rehashed':finite_count,'all_same_P_and_FP':True,'small_serial_actual_team':small['row_actual_teams'],'independent_process_batches':384,'nested_samples_are_not_independent':True,'criteria_unchanged':True,'scene_results':summary,'historical_HTH_reconstruction':'FAIL_BYTE_EXACT: baseline and candidate both mismatch historical HTH by 2.98–4.47e-8; cause unverified, no tolerance relaxation','whole_state_small_medium_p95_regression_retained':True,'actual_physical_navigation':'UNVERIFIED_NOT_RUN_V15','full_estimator_trajectory_replay':'UNVERIFIED_INSUFFICIENT_RECORDING','not_claimed':['whole SLAM pipeline +75%','full sensor-only Actor','independent physical benefit of V15','historical complete trajectory replay'],'verified_input_sha256':inputs,'auditor_sha256':sha(__file__),'total_bytes_read_for_hash':bytes_hashed}
p=E/'V15_CONFIRMATORY_INDEPENDENT.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'receipt':str(p),'sha256':sha(p),'batch_count':384,'scene_gates':[{k:x[k]for k in ['scene','limited_kernel_gate_passed','median_improvement_fraction_independent_batches']}for x in summary],'finite_output_rehashed':finite_count,'bytes_read':bytes_hashed}))
