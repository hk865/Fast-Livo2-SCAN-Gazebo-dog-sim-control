"""Fail closed on this candidate's own offline evidence, never inherited PASS."""
import hashlib,json
from pathlib import Path
SCHEMA='deterministic_vio_patch_parallel_preflight/v1'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def evidence_files(here):
    p=Path(here)/'VIO_PATCH_PARALLEL_PREFLIGHT.json'
    if not p.is_file():raise RuntimeError('V16 independent preflight is missing')
    d=json.loads(p.read_text())
    return [Path(x)for x in d.get('verified_inputs_sha256',{})]
def verify_preflight(here,profile):
    here=Path(here).resolve()
    p=here/'VIO_PATCH_PARALLEL_PREFLIGHT.json';d=json.loads(p.read_text())
    if d.get('schema')!=SCHEMA or d.get('status')!='PASS_LIMITED_BUILD_NUMERIC_AND_BENCHMARK':
        raise RuntimeError('V16 requires its own finite numeric and scheduled benchmark evidence')
    if d.get('candidate_root')!=str(here) or d.get('actual_runtime_verified')is not False:
        raise RuntimeError('V16 preflight identity or limited scope differs')
    library=here/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so'
    if d.get('loaded_core_sha256')!=sha(library):raise RuntimeError('V16 installed core differs from independently tested binary')
    config=profile.get('vio_patch_parallel',{})
    if (config.get('threads')!=d.get('compiled_vio_patch_threads')or
        config.get('serial_below_patch_count')!=d.get('compiled_min_patch_count')or
        config.get('reduction')!='original_index_order_float_error_and_int_count'):
        raise RuntimeError('V16 profile differs from frozen patch compile contract')
    inputs=d.get('verified_inputs_sha256',{})
    out=here.parents[1]/'test_results/parallel_pipeline_20261005/vio_v16'
    required=[here/'slam_ws/src/fast_livo2_core/src/vio.cpp',here/'slam_ws/src/fast_livo2_core/CMakeLists.txt',here/'VIO_PATCH_PARALLEL_CONTRACT.json']
    required+=[out/name for name in ('fixture_build_receipt.json','numeric_comparison.json','benchmark_confirmatory32.json','team_partition_proof.json','vio_patch_fixture.cpp')]
    if any(str(x)not in inputs for x in required)or not inputs:raise RuntimeError('V16 own source bindings are incomplete')
    for name,digest in inputs.items():
        if sha(name)!=digest:raise RuntimeError('V16 offline source/evidence changed '+name)
    contract=json.loads((here/'VIO_PATCH_PARALLEL_CONTRACT.json').read_text())
    if (contract.get('threshold_finalized')is not True or
        contract.get('selected_vio_patch_threads')!=d['compiled_vio_patch_threads']or
        contract.get('serial_below_patch_count')!=d['compiled_min_patch_count']):
        raise RuntimeError('V16 contract selection was not frozen with this build')
    cache=(here/'slam_ws/build/fast_livo2_core/CMakeCache.txt').read_text()
    for key,value in [('V16_VIO_PATCH_THREADS',d['compiled_vio_patch_threads']),('V16_VIO_PATCH_MIN_POINTS',d['compiled_min_patch_count'])]:
        if key+':STRING='+str(value)+'\n'not in cache:
            raise RuntimeError('V16 actual CMake build differs from preflight '+key)
    if d.get('numeric_all_outputs_byte_identical')is not True or d.get('benchmark_prospective_gates_passed')is not True:
        raise RuntimeError('V16 offline numeric/performance gate did not pass')
    build=json.loads((out/'fixture_build_receipt.json').read_text())
    variant='candidate_V16_t'+str(d['compiled_vio_patch_threads'])
    if build.get('variants',{}).get(variant,{}).get('lib_sha256')!=d['loaded_core_sha256']:
        raise RuntimeError('V16 fixture did not load this candidate library')
    numeric=json.loads((out/'numeric_comparison.json').read_text())
    checks=[x for x in numeric.get('comparisons',[])if x.get('candidate')==variant]
    if (numeric.get('status')!='PASS_LIMITED_SYNTHETIC' or len(checks)<41 or
        any(x.get('state_cov_G_H_errors_reference_H_byte_identical')is not True or
            x.get('original_nonwall_diagnostic_records_byte_identical')is not True for x in checks)):
        raise RuntimeError('V16 own finite numeric comparison is incomplete or failed')
    bench=json.loads((out/'benchmark_confirmatory32.json').read_text())
    comparisons=[x for x in bench.get('comparisons',[])if x.get('candidate')==variant and x.get('reference')=='baseline_V12']
    expected={(mode,n)for mode in ('forward','inverse')for n in (32,128,512,1024)}
    if {(x.get('mode'),x.get('points'))for x in comparisons}!=expected:
        raise RuntimeError('V16 independent benchmark scenes differ')
    for x in comparisons:
        for key in ('baseline','candidate_timings'):
            v=x.get(key,{})
            if v.get('independent_process_batches',0)<32 or v.get('valid_per_batch',0)<30 or v.get('warmups_per_batch',0)<10:
                raise RuntimeError('V16 benchmark uses insufficient independent process batches')
        if x.get('all_outputs_byte_identical')is not True:
            raise RuntimeError('V16 benchmark output differed')
        if x['points']>=512 and x.get('prospective_large_median_10pct_and_p95_nonregression')is not True:
            raise RuntimeError('V16 large benchmark did not meet prospective performance gates')
    return d
