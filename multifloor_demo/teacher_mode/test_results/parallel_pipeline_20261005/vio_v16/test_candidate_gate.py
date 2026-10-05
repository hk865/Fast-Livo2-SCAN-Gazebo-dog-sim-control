#!/usr/bin/env python3
"""Memory-only rejection tests. No candidate inputs or runtime are changed."""
from pathlib import Path
import copy,hashlib,importlib,importlib.util,json,sys
from unittest.mock import patch
OUT=Path(__file__).resolve().parent
HERE=OUT.parents[2]/'navigation/parallel_vio_v16'
sys.path.insert(0,str(HERE))
gate=importlib.import_module('vio_preflight')
spec=importlib.util.spec_from_file_location('owned_V16_run',HERE/'run.py');runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
REAL_LOADS=json.loads;REAL_SHA=gate.sha
PROFILE=REAL_LOADS((HERE/'profiles/vio_patch1_l64_r30_c30_210.json').read_text())
JSON_PATHS={name:HERE/name for name in ('VIO_PATCH_PARALLEL_PREFLIGHT.json','VIO_PATCH_PARALLEL_CONTRACT.json')}
JSON_PATHS.update({name:OUT/name for name in ('fixture_build_receipt.json','numeric_comparison.json','benchmark_confirmatory32.json')})
RAW={name:path.read_text()for name,path in JSON_PATHS.items()}
rows=[]
def exercise(name,file=None,mutation=None,profile=None,changed_hash=None,allow=False):
    def loads(s,*args,**kwargs):
        value=REAL_LOADS(s,*args,**kwargs)
        if file is not None and s==RAW[file]:mutation(value)
        return value
    def digest(path):
        if changed_hash is not None and Path(path).resolve()==changed_hash.resolve():return '0'*64
        return REAL_SHA(path)
    try:
        with patch.object(gate.json,'loads',loads),patch.object(gate,'sha',digest):
            runner.validate_profile(copy.deepcopy(profile or PROFILE))
    except (RuntimeError,ValueError,KeyError,FileNotFoundError)as e:
        rejected=True;message=str(e)
    else:rejected=False;message='accepted limited offline T1 preflight'
    passed=(not rejected)if allow else rejected
    rows.append({'case':name,'expected':'allow limited offline'if allow else'reject','rejected':rejected,'passed':passed,'result':message})
    if not passed:raise AssertionError(rows[-1])
exercise('valid_selected_T1',allow=True)
exercise('failed_T4_profile',profile=REAL_LOADS((HERE/'profiles/vio_patch4_l64_r30_c30_210.json').read_text()))
exercise('foreign_candidate_root','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(candidate_root=str(HERE)+'_foreign'))
exercise('copied_source_prepared_status','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(status='SOURCE_PREPARED_UNVERIFIED'))
exercise('claimed_actual_runtime_PASS','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(actual_runtime_verified=True))
exercise('wrong_loaded_core','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(loaded_core_sha256='0'*64))
exercise('missing_source_binding','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d['verified_inputs_sha256'].pop(str(HERE/'slam_ws/src/fast_livo2_core/src/vio.cpp')))
exercise('changed_fixture_source',changed_hash=OUT/'vio_patch_fixture.cpp')
exercise('changed_benchmark_receipt',changed_hash=OUT/'benchmark_confirmatory32.json')
exercise('numeric_flag_FALSE','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(numeric_all_outputs_byte_identical=False))
exercise('benchmark_flag_FALSE','VIO_PATCH_PARALLEL_PREFLIGHT.json',lambda d:d.update(benchmark_prospective_gates_passed=False))
exercise('threshold_not_frozen','VIO_PATCH_PARALLEL_CONTRACT.json',lambda d:d.update(threshold_finalized=False))
exercise('wrong_fixture_loaded_core','fixture_build_receipt.json',lambda d:d['variants']['candidate_V16_t1'].update(lib_sha256='0'*64))
exercise('numeric_nonwall_diag_failed','numeric_comparison.json',lambda d:next(x for x in d['comparisons']if x['candidate']=='candidate_V16_t1').update(original_nonwall_diagnostic_records_byte_identical=False))
exercise('numeric_missing_case','numeric_comparison.json',lambda d:d.update(comparisons=[x for x in d['comparisons']if not(x['candidate']=='candidate_V16_t1'and x['case']=='forward_zero')]))
def bench_change(d,fn):
    target=next(x for x in d['comparisons']if x['reference']=='baseline_V12'and x['candidate']=='candidate_V16_t1'and x['points']==1024)
    fn(target)
exercise('only31_independent_batches','benchmark_confirmatory32.json',lambda d:bench_change(d,lambda x:x['candidate_timings'].update(independent_process_batches=31)))
exercise('only29_internal_valid','benchmark_confirmatory32.json',lambda d:bench_change(d,lambda x:x['candidate_timings'].update(valid_per_batch=29)))
exercise('only9_warmups','benchmark_confirmatory32.json',lambda d:bench_change(d,lambda x:x['candidate_timings'].update(warmups_per_batch=9)))
exercise('large_performance_failed','benchmark_confirmatory32.json',lambda d:bench_change(d,lambda x:x.update(prospective_large_median_10pct_and_p95_nonregression=False)))
exercise('benchmark_output_not_equal','benchmark_confirmatory32.json',lambda d:bench_change(d,lambda x:x.update(all_outputs_byte_identical=False)))
exercise('missing_large_scene','benchmark_confirmatory32.json',lambda d:d.update(comparisons=[x for x in d['comparisons']if not(x['reference']=='baseline_V12'and x['candidate']=='candidate_V16_t1'and x['points']==1024)]))
for name,field,value in [('truth_navigation','navigation_ground_truth_used',True),('other_controller','controller_kind','champ'),('TTL_relaxed','pose_cloud_timeout_s',.31),('invalid_duration','duration_s',float('nan'))]:
    p=copy.deepcopy(PROFILE);p[field]=value;exercise(name,profile=p)
report={'schema':'independent_V16_candidate_gate_boundaries/v1','status':'PASS_FINITE_BOUNDARIES','cases':rows,
        'disk_mutations':'new report only; all negative cases use memory overlays, original candidate sources/receipts unchanged',
        'preflight_sha256':REAL_SHA(HERE/'VIO_PATCH_PARALLEL_PREFLIGHT.json'),'gate_source_sha256':REAL_SHA(HERE/'vio_preflight.py')}
with(OUT/'candidate_gate_tests.json').open('x')as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({'status':report['status'],'cases':len(rows)}))
