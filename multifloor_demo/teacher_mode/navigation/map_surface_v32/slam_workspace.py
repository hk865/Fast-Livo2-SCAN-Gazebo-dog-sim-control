"""Source/build-bound independent V32 workspace; no inherited runtime PASS."""
from pathlib import Path
import hashlib
import json

HERE=Path(__file__).resolve().parent
SLAM_WORKSPACE=HERE/'slam_ws'
CONTRACT=HERE/'SLAM_WORKSPACE_CONTRACT.json'
CORE_RECEIPT=HERE.parents[1]/'test_results/map_surface_v32_20261007/core/CORE_PREFLIGHT.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def required_workspace_files():
    source=sorted(p for p in (SLAM_WORKSPACE/'src').rglob('*') if p.is_file()
        and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.swp'))
    return source+[SLAM_WORKSPACE/'build'/pkg/name
        for pkg in ('fast_livo2_core','fast_livo2_ros')
        for name in ('compile_commands.json','CMakeCache.txt')]+[
        SLAM_WORKSPACE/'install'/name for name in ('setup.bash','local_setup.bash',
        'fast_livo2_core/lib/libfast_livo2_core.so',
        'fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')]


def verify_slam_workspace():
    from full46_launch_contract import SURFACE_PARAMETERS
    d=json.loads(CONTRACT.read_text())
    if (d.get('schema')!='map_surface_v32_independent_SLAM_workspace/v1'
        or d.get('workspace')!=str(SLAM_WORKSPACE.resolve())
        or d.get('source_candidate')!='map_surface_v32'
        or d.get('CPP_changed') is not True or d.get('build_copied') is not False
        or d.get('runtime_acceptance_inherited') is not False
        or d.get('actual_navigation_verified') is not False
        or d.get('actual_loaded_binary_witness_required') is not True
        or (d.get('LIO_threads'),d.get('VIO_threads'),d.get('Teacher_CPU_threads'))!=(4,1,1)
        or d.get('surface_parameters')!=SURFACE_PARAMETERS
        or d.get('surface_validity_runtime_values')!=[0,1]):
        raise RuntimeError('V32 requires its own independently built surface-quality workspace with unverified navigation')
    bindings=d.get('bindings_sha256')
    required={str(p.resolve()) for p in required_workspace_files()}
    if not required or not isinstance(bindings,dict) or set(bindings)!=required:
        raise RuntimeError('V32 complete source/build/installed bindings missing')
    for name,digest in bindings.items():
        if not Path(name).is_absolute() or sha(name)!=digest:
            raise RuntimeError('V32 SLAM source/build/ELF changed: '+name)
    receipt=d.get('core_source_receipt',{})
    if receipt.get('path')!=str(CORE_RECEIPT.resolve()) or receipt.get('sha256')!=sha(CORE_RECEIPT):
        raise RuntimeError('V32 independent core evidence identity changed')
    core=json.loads(CORE_RECEIPT.read_text())
    if (core.get('status')!='PASS_LIMITED_CORE_FOR_ROOT_REVIEW'
        or core.get('runtime_authorized_by_this_file') is not False
        or core.get('actual_navigation_verified') is not False
        or core.get('workspace')!=str(SLAM_WORKSPACE.resolve())
        or core.get('library_sha256')!=sha(SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so')
        or core.get('mapping_executable_sha256')!=sha(SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')):
        raise RuntimeError('Wrong V32 limited mechanism/build evidence')
    source=core.get('source_bindings')
    if not isinstance(source,dict) or not source:
        raise RuntimeError('V32 core mechanism source binding missing')
    for rel,digest in source.items():
        p=SLAM_WORKSPACE/'src'/rel
        if not p.resolve().is_relative_to(SLAM_WORKSPACE/'src') or sha(p)!=digest:
            raise RuntimeError('V32 mechanism source differs: '+str(rel))
    for package in ('fast_livo2_core','fast_livo2_ros'):
        commands=json.loads((SLAM_WORKSPACE/'build'/package/'compile_commands.json').read_text())
        if not commands:
            raise RuntimeError('V32 empty independent compile commands')
        for row in commands:
            if not Path(row['file']).resolve().is_relative_to(SLAM_WORKSPACE/'src'):
                raise RuntimeError('V32 compilation references an inherited/external source')
    return d


def evidence_files():
    d=verify_slam_workspace()
    return [CONTRACT,CORE_RECEIPT,*[Path(name) for name in d['bindings_sha256']]]


def candidate_contract():
    """Freeze-ready data only; caller must inspect mechanism/build evidence."""
    from full46_launch_contract import SURFACE_PARAMETERS
    return dict(schema='map_surface_v32_independent_SLAM_workspace/v1',
        workspace=str(SLAM_WORKSPACE.resolve()),source_candidate='map_surface_v32',CPP_changed=True,
        build_copied=False,runtime_acceptance_inherited=False,actual_navigation_verified=False,
        actual_loaded_binary_witness_required=True,LIO_threads=4,VIO_threads=1,Teacher_CPU_threads=1,
        surface_parameters=SURFACE_PARAMETERS,surface_validity_runtime_values=[0,1],
        core_source_receipt=dict(path=str(CORE_RECEIPT.resolve()),sha256=sha(CORE_RECEIPT)),
        bindings_sha256={str(p.resolve()):sha(p) for p in required_workspace_files()})
