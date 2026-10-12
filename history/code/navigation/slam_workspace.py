"""Fresh R69 real-source identity build; no runtime acceptance inherited."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
SLAM_WORKSPACE=Path('/__LOCAL_HOME__/Documents/Codex/2026-10-07/task/go2_engineering_v33/R69_P0_DIAGNOSTIC_STAGE/native_ws')
CONTRACT=HERE/'R33_SLAM_WORKSPACE_CONTRACT.json'
CORE_RECEIPT=SLAM_WORKSPACE.parent/'FULL_FASTLIVO_BUILD_RECEIPT.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def required_workspace_files():
    sources=sorted(p for p in (SLAM_WORKSPACE/'src').rglob('*')if p.is_file()and '__pycache__'not in p.parts)
    return sources+[SLAM_WORKSPACE/'build'/pkg/n for pkg in ('fast_livo2_core','fast_livo2_ros')for n in ('compile_commands.json','CMakeCache.txt')]+[SLAM_WORKSPACE/'install/setup.bash',SLAM_WORKSPACE/'install/local_setup.bash',SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so',SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
def verify_slam_workspace():
    d=json.loads(CONTRACT.read_text());stage=SLAM_WORKSPACE.parent
    if (d.get('schema')!='R69_explicit_point_identity_workspace/v1' or d.get('workspace')!=str(SLAM_WORKSPACE)
        or d.get('build_copied')is not False or d.get('full_source_build_complete')is not True
        or d.get('runtime_acceptance_inherited')is not False or d.get('actual_navigation_verified')is not False
        or d.get('actual_loaded_binary_witness_required')is not True
        or (d.get('LIO_threads'),d.get('VIO_threads'),d.get('Teacher_CPU_threads'))!=(4,1,1)):
        raise RuntimeError('R69 workspace scope differs')
    fix=json.loads((stage/'NATIVE_SOURCE_CHANGE.json').read_text());old=Path(fix['baseline_workspace'])
    actual={str(p.relative_to(SLAM_WORKSPACE/'src')):sha(p)for p in (SLAM_WORKSPACE/'src').rglob('*')if p.is_file()and '__pycache__'not in p.parts}
    for rel,digest in fix['baseline_sources'].items():
        if sha(old/'src'/rel)!=digest:raise RuntimeError('Protected original native source changed: '+rel)
    delta=sorted(k for k,v in actual.items()if v!=fix['baseline_sources'].get(k))
    if (actual!=fix['candidate_sources'] or delta!=fix['changed_source_relative_paths'] or delta!=d['changed_source_relative_paths']
        or 'fast_livo2_core/src/IMU_Processing.cpp'in delta or set(fix['baseline_sources'])-set(actual)):
        raise RuntimeError('R69 explicit native identity delta differs')
    if set(d['bindings_sha256'])!={str(p)for p in required_workspace_files()}:raise RuntimeError('R69 incomplete source/build/ELF bindings')
    for p,h in d['bindings_sha256'].items():
        if sha(p)!=h:raise RuntimeError('R69 source/build/ELF changed: '+p)
    if d['build_receipt']!=dict(path=str(CORE_RECEIPT),sha256=sha(CORE_RECEIPT)):raise RuntimeError('R69 build receipt changed')
    r=json.loads(CORE_RECEIPT.read_text())
    if (r.get('full_source_build_complete')is not True or r.get('build_returncode')!=0 or r.get('source_only_initial_copy')is not True
        or r.get('build_copied')is not False or r.get('old_workspace_modified')is not False or r.get('runtime_authorized_by_build_receipt')is not False):
        raise RuntimeError('R69 independent build not evidenced')
    for p,h in r['build_attempt_evidence'].items():
        if sha(p)!=h:raise RuntimeError('R69 actual build log changed')
    for pkg in ('fast_livo2_core','fast_livo2_ros'):
        cmds=json.loads((SLAM_WORKSPACE/'build'/pkg/'compile_commands.json').read_text())
        if not cmds or any(not Path(x['file']).resolve().is_relative_to(SLAM_WORKSPACE/'src')for x in cmds):raise RuntimeError('R69 inherited compilation source')
    return d
def evidence_files():
    d=verify_slam_workspace()
    return [Path(__file__).resolve(),CONTRACT,CORE_RECEIPT,SLAM_WORKSPACE.parent/'NATIVE_SOURCE_CHANGE.json',*[Path(p)for p in d['bindings_sha256']]]
