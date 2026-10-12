"""R33 independently rebuilt FASTLIVO; no inherited runtime acceptance."""
from pathlib import Path
import hashlib
import json

HERE=Path(__file__).resolve().parent
SLAM_WORKSPACE=Path('/__LOCAL_HOME__/Documents/Codex/2026-10-07/task/go2_engineering_v33/R56_DESKEW_INTERVAL_LONG_STAGE/slam_ws')
CONTRACT=HERE/'R33_SLAM_WORKSPACE_CONTRACT.json'
CORE_RECEIPT=SLAM_WORKSPACE.parent/'FULL_FASTLIVO_BUILD_RECEIPT.json'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def required_workspace_files():
    sources=sorted(p for p in (SLAM_WORKSPACE/'src').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    return sources+[SLAM_WORKSPACE/'build'/package/name
        for package in ('fast_livo2_core','fast_livo2_ros')
        for name in ('compile_commands.json','CMakeCache.txt')]+[
        SLAM_WORKSPACE/'install/setup.bash',SLAM_WORKSPACE/'install/local_setup.bash',
        SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so',
        SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
def verify_slam_workspace():
    d=json.loads(CONTRACT.read_text())
    if (d.get('schema')!='R56_local_deskew_interval_workspace/v1'
        or d.get('workspace')!=str(SLAM_WORKSPACE.resolve())
        or d.get('build_copied') is not False or d.get('full_source_build_complete') is not True
        or d.get('runtime_acceptance_inherited') is not False or d.get('actual_navigation_verified') is not False
        or d.get('actual_loaded_binary_witness_required') is not True
        or d.get('changed_source_relative_paths')!=['fast_livo2_core/src/IMU_Processing.cpp']
        or (d.get('LIO_threads'),d.get('VIO_threads'),d.get('Teacher_CPU_threads'))!=(4,1,1)):
        raise RuntimeError('R33 complete independent timestamp-codec workspace contract differs')
    stage=SLAM_WORKSPACE.parent
    fix=json.loads((stage/'NATIVE_SOURCE_CHANGE.json').read_text())
    old=Path(fix['old_workspace'])
    if fix['source_changed_relative_paths']!=['fast_livo2_core/src/IMU_Processing.cpp'] or fix['change_kind']!='native_geometry_behavior_fix_not_equivalent_glue_refactor':
        raise RuntimeError('R56 local native geometry scope differs')
    current={str(p.relative_to(SLAM_WORKSPACE/'src')):sha(p) for p in (SLAM_WORKSPACE/'src').rglob('*') if p.is_file()}
    if current!=fix['candidate_sources']:
        raise RuntimeError('R56 actual local native sources changed')
    for rel,digest in fix['baseline_sources'].items():
        if sha(old/'src'/rel)!=digest:raise RuntimeError('R56 sealed native baseline changed: '+rel)
        if rel!='fast_livo2_core/src/IMU_Processing.cpp' and current[rel]!=digest:
            raise RuntimeError('R56 unrelated native source changed: '+rel)
    original=(old/'src/fast_livo2_core/src/IMU_Processing.cpp').read_text()
    actual=(SLAM_WORKSPACE/'src/fast_livo2_core/src/IMU_Processing.cpp').read_text()
    expected=original.replace('      acc_imu << VEC_FROM_ARRAY(head->acc);',
        '      // The interval [head, tail] uses the quantities saved by its forward\n      // propagation into tail. Head stores the preceding interval\'s values.\n      acc_imu << VEC_FROM_ARRAY(tail->acc);').replace(
        '      angvel_avr << VEC_FROM_ARRAY(head->gyr);','      angvel_avr << VEC_FROM_ARRAY(tail->gyr);')
    if actual!=expected:raise RuntimeError('R56 native geometry differs beyond the two local interval assignments')
    cpu=json.loads((stage/'DESKEW_CPU_REPLAY.json').read_text())
    if cpu.get('passed')is not True or cpu.get('gate_predicates_changed')is not False or cpu.get('point_order_or_pairing_changed')is not False:
        raise RuntimeError('R56 native CPU replay not passed')
    for name,digest in cpu['source_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('R56 native CPU replay source/evidence changed: '+name)
    required={str(p.resolve()) for p in required_workspace_files()}
    if set(d['bindings_sha256'])!=required:raise RuntimeError('R33 incomplete source/build/ELF bindings')
    for name,digest in d['bindings_sha256'].items():
        if sha(name)!=digest:raise RuntimeError('R33 source/build/ELF changed: '+name)
    if d['build_receipt']!=dict(path=str(CORE_RECEIPT),sha256=sha(CORE_RECEIPT)):
        raise RuntimeError('R33 full build receipt differs')
    receipt=json.loads(CORE_RECEIPT.read_text())
    if (receipt.get('full_source_build_complete') is not True or receipt.get('build_returncode')!=0
        or receipt.get('old_workspace_modified') is not False
        or receipt.get('runtime_authorized_by_build_receipt') is not False):
        raise RuntimeError('R33 complete build not independently evidenced')
    for package in ('fast_livo2_core','fast_livo2_ros'):
        commands=json.loads((SLAM_WORKSPACE/'build'/package/'compile_commands.json').read_text())
        if not commands:raise RuntimeError('R33 empty compilation source list')
        for row in commands:
            if not Path(row['file']).resolve().is_relative_to(SLAM_WORKSPACE/'src'):
                raise RuntimeError('R33 compilation uses inherited source')
    return d
def evidence_files():
    d=verify_slam_workspace()
    return [Path(__file__).resolve(),CONTRACT,CORE_RECEIPT,*[Path(name) for name in d['bindings_sha256']]]
