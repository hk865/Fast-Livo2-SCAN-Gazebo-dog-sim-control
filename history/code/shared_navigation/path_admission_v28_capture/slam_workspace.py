"""Explicit immutable reuse of V19 SLAM; no local build or runtime PASS claim."""
from pathlib import Path
import hashlib
import json

HERE = Path(__file__).resolve().parent
SLAM_WORKSPACE = HERE.parent / 'pipeline_v19/slam_ws'
CONTRACT = HERE / 'SLAM_WORKSPACE_CONTRACT.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def required_workspace_files():
    source = sorted(p for p in (SLAM_WORKSPACE/'src').rglob('*') if p.is_file() and
        (p.suffix in ('.cpp', '.h', '.hpp') or p.name in ('CMakeLists.txt', 'package.xml')))
    return source + [SLAM_WORKSPACE/'build'/pkg/name
        for pkg in ('fast_livo2_core', 'fast_livo2_ros')
        for name in ('compile_commands.json', 'CMakeCache.txt')] + [
        SLAM_WORKSPACE/'install'/name for name in ('setup.bash', 'local_setup.bash',
            'fast_livo2_core/lib/libfast_livo2_core.so',
            'fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')]


def verify_slam_workspace():
    d = json.loads(CONTRACT.read_text())
    if (d.get('schema') != 'corridor_v20_inherited_SLAM_workspace/v1'
            or d.get('workspace') != str(SLAM_WORKSPACE.resolve())
            or d.get('source_candidate') != 'pipeline_v19' or d.get('CPP_changed') is not False
            or d.get('build_copied') is not False or d.get('runtime_acceptance_inherited') is not False
            or d.get('actual_loaded_binary_witness_required') is not True
            or (d.get('LIO_threads'), d.get('VIO_threads'), d.get('Teacher_CPU_threads')) != (4, 1, 1)):
        raise RuntimeError('V20 must reuse only the frozen V19 SLAM workspace without runtime acceptance')
    bindings = d.get('bindings_sha256')
    required = {str(p.resolve()) for p in required_workspace_files()}
    if not required or not isinstance(bindings, dict) or set(bindings) != required:
        raise RuntimeError('Inherited V19 source/build/installed bindings are incomplete')
    for name, digest in bindings.items():
        if sha(name) != digest:
            raise RuntimeError('Inherited SLAM source/build/binary changed: ' + name)
    original = d['original_source_receipt']
    expected = HERE.parents[1]/'test_results/pipeline_v19_20261006/core/CORE_PREFLIGHT.json'
    if Path(original['path']).resolve() != expected.resolve() or sha(expected) != original['sha256']:
        raise RuntimeError('Inherited V19 limited core receipt changed')
    core = json.loads(expected.read_text())
    if (core.get('status') != 'PASS_LIMITED_CORE_FOR_ROOT_REVIEW'
            or core.get('runtime_authorized_by_this_file') is not False
            or core['library_sha256'] != sha(SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so')
            or core['mapping_executable_sha256'] != sha(SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')):
        raise RuntimeError('Wrong inherited limited core/library identity')
    for rel, digest in core['source_bindings'].items():
        if sha(SLAM_WORKSPACE/'src'/rel) != digest:
            raise RuntimeError('Inherited core source no longer matches its original finite evidence')
    return d


def evidence_files():
    d = verify_slam_workspace()
    return [CONTRACT, Path(d['original_source_receipt']['path']),
            *[Path(name) for name in d['bindings_sha256']]]
