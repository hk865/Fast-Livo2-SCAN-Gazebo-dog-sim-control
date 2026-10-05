"""V18 authorizes only a new finite-tested combined candidate, never copied PASS."""
import hashlib,json
from pathlib import Path
SCHEMA='combined_compute_preflight/v1'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def evidence_files(here):
    d=json.loads((Path(here)/'COMBINED_COMPUTE_PREFLIGHT.json').read_text())
    return [Path(x)for x in d.get('verified_inputs_sha256',{})]
def verify_preflight(here,profile):
    here=Path(here).resolve();d=json.loads((here/'COMBINED_COMPUTE_PREFLIGHT.json').read_text())
    if d.get('schema')!=SCHEMA or d.get('status')!='PASS_LIMITED_COMBINED_KERNEL_ONLY' or d.get('allowed')is not True:
        raise RuntimeError('V18 requires its own build and combined fresh numeric evidence')
    if d.get('candidate_root')!=str(here)or d.get('actual_runtime_verified')is not False:
        raise RuntimeError('V18 identity or limited experimental scope changed')
    inputs=d.get('verified_inputs_sha256',{})
    out=here.parents[1]/'test_results/parallel_pipeline_20261005/combined_v18'
    required=[here/'COMBINED_COMPUTE_CONTRACT.json',here/'combined_preflight.py',
        here/'slam_ws/src/fast_livo2_core/src/voxel_map.cpp',here/'slam_ws/src/fast_livo2_core/include/fast_livo2_core/core/voxel_map.h',
        here/'slam_ws/src/fast_livo2_core/src/vio.cpp',here/'slam_ws/src/fast_livo2_core/CMakeLists.txt',
        here/'slam_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',here/'slam_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    required +=[out/n for n in ('build_receipt.json','source_combination_proof.json','controller_source_equivalence.json',
        'finite_fixture_comparison.json','vio_fixture_build_receipt.json','vio_numeric_comparison.json','combined_team_and_loader_witness.json')]
    if not inputs or any(str(p.resolve())not in inputs for p in required):raise RuntimeError('V18 evidence binding is incomplete')
    for path,digest in inputs.items():
        if sha(path)!=digest:raise RuntimeError('V18 offline source/binary/evidence changed '+path)
    lio=profile.get('lio_jacobian_parallelism',{});vio=profile.get('vio_patch_parallel',{})
    if(lio.get('threads')!=4 or lio.get('parallel_min_rows')!=256 or lio.get('row_operation_order')!='original_V12' or
        lio.get('floating_reduction_changed')is not False or vio.get('threads')!=1 or vio.get('serial_below_patch_count')!=64 or
        vio.get('reduction')!='original_index_order_float_error_and_int_count'):
        raise RuntimeError('V18 requires frozen LIO4/VIO1 original ordered arithmetic')
    cache=(here/'slam_ws/build/fast_livo2_core/CMakeCache.txt').read_text()
    for key,value in [('V16_VIO_PATCH_THREADS',1),('V16_VIO_PATCH_MIN_POINTS',64)]:
        if key+':STRING='+str(value)+'\n'not in cache:raise RuntimeError('V18 actual compile settings differ '+key)
    commands=json.loads((here/'slam_ws/build/fast_livo2_core/compile_commands.json').read_text())
    voxel=[x for x in commands if x['file'].endswith('/voxel_map.cpp')][0]
    if 'MP_PROC_NUM=4'not in voxel['command']:raise RuntimeError('V18 LIO4 compile scope differs')
    finite=json.loads((out/'finite_fixture_comparison.json').read_text())
    if finite.get('all_numeric_comparisons_pass')is not True or len(finite.get('comparisons',[]))!=3:
        raise RuntimeError('V18 fresh 729 StateEstimation and original nonwall gate incomplete')
    numeric=json.loads((out/'vio_numeric_comparison.json').read_text())
    checks=numeric.get('comparisons',[])
    if numeric.get('fixtures')!=41 or numeric.get('process_runs')!=82 or len(checks)!=41 or any(
        x.get('state_cov_G_H_errors_reference_H_byte_identical')is not True or
        x.get('original_nonwall_diagnostic_records_byte_identical')is not True for x in checks):
        raise RuntimeError('V18 fresh 41-case VIO combined gate incomplete')
    if d.get('combined_performance_measured')is not False:raise RuntimeError('Offline combined gate cannot invent end-to-end timing')
    return d
