"""Fresh R71 real-source identity build; no runtime acceptance inherited."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
LEGACY_WORKSPACE=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/history/2026-10-07/task/go2_engineering_v33/R73_SERIALIZATION_DYNAMIC_READY_STAGE/native_ws')
SELECTION=HERE/'SERIAL_IMU_PRIORITY_WORKSPACE.json'
def _private_selection():
    if not SELECTION.exists():return None
    value=json.loads(SELECTION.read_bytes())
    if value==dict(schema='serial_imu_priority_workspace_selection/v1',enabled=False) and type(value['enabled'])is bool:return None
    if (type(value)is not dict or set(value)!={'schema','enabled','manifest_path','manifest_sha256','component_path','component_sha256'}
            or value.get('schema')!='serial_imu_priority_workspace_selection/v1' or value.get('enabled')is not True
            or not Path(value['manifest_path']).is_absolute() or not Path(value['component_path']).is_absolute()):raise RuntimeError('Explicit serial IMU workspace selection differs')
    raw=Path(value['manifest_path']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=value['manifest_sha256']:raise RuntimeError('Private native build manifest changed')
    component_raw=Path(value['component_path']).read_bytes()
    if hashlib.sha256(component_raw).hexdigest()!=value['component_sha256']:raise RuntimeError('Private native component receipt changed')
    component=json.loads(component_raw)
    doc=json.loads(raw)
    if doc.get('schema')=='published_formal22_native571_build/v1':
        if (component.get('schema')!='published_formal22_native_component/v1'
                or component.get('passed')is not True or component.get('returncode')!=0
                or component.get('full_SLAM_causality_or_acceptance_claimed')is not False
                or component.get('no_ROS_or_node_started')is not True):
            raise RuntimeError('Published native component receipt scope differs')
    else:
        if component.get('passed')is not True or component.get('returncode')!=0 or component.get('full_SLAM_causality_or_acceptance_claimed')is not False:
            raise RuntimeError('Private native component receipt scope differs')
        if doc.get('schema')!='serial_imu_priority_native_build/v1':raise RuntimeError('Private native build schema differs')
    return value,doc
PRIVATE=_private_selection()
SLAM_WORKSPACE=Path(PRIVATE[1]['workspace']).resolve() if PRIVATE else LEGACY_WORKSPACE
SLAM_SOURCE_ROOT=Path(PRIVATE[1]['source_root']).resolve() if PRIVATE else LEGACY_WORKSPACE/'src'
CONTRACT=HERE/'R33_SLAM_WORKSPACE_CONTRACT.json'
CORE_RECEIPT=Path(PRIVATE[0]['manifest_path'])if PRIVATE else SLAM_WORKSPACE.parent/'FULL_FASTLIVO_BUILD_RECEIPT.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def required_workspace_files():
    if PRIVATE:return [Path(p)for p in PRIVATE[1]['bindings_sha256']]
    sources=sorted(p for p in (SLAM_WORKSPACE/'src').rglob('*')if p.is_file()and '__pycache__'not in p.parts)
    return sources+[SLAM_WORKSPACE/'build'/pkg/n for pkg in ('fast_livo2_core','fast_livo2_ros')for n in ('compile_commands.json','CMakeCache.txt')]+[SLAM_WORKSPACE/'install/setup.bash',SLAM_WORKSPACE/'install/local_setup.bash',SLAM_WORKSPACE/'install/fast_livo2_core/lib/libfast_livo2_core.so',SLAM_WORKSPACE/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
def _verify_published_workspace(value,doc):
    """New build provenance for the same native571 source; no runtime credit."""
    if (doc.get('public_base_commit')!='63e6b82225c7b3718c4e170e5005bfd6a387b69f'
            or doc.get('original_native_commit')!='571161b251fbd024ce755c2dc3301a2120e8637b'
            or type(doc.get('source_file_count'))is not int or doc['source_file_count']!=48
            or doc.get('build',{}).get('result')!='PASS' or doc['build'].get('returncode')!=0
            or doc.get('runtime_loading_PASS')is not False or doc.get('no_ROS_or_node_started')is not True
            or doc.get('selection',{}).get('env')!='FASTLIVO_SERIAL_IMU_PRIORITY'
            or doc['selection'].get('on_mode')!='serial' or doc['selection'].get('value')!='1'
            or doc['selection'].get('covariance_candidate_included')is not False):
        raise RuntimeError('Published native571 source/build selection differs')
    package=HERE.parent/'releases/formal22/native'
    if SLAM_WORKSPACE!=package.resolve() or SLAM_SOURCE_ROOT!=(package/'src').resolve():
        raise RuntimeError('Published native workspace/source is not this package')
    rows=doc['sources'];expected={str(Path(r['path']).resolve()):r['sha256']for r in rows}
    actual={str(p.resolve()):sha(p)for name in ('fast_livo2_core','fast_livo2_ros')
        for p in (SLAM_SOURCE_ROOT/name).rglob('*')if p.is_file()and '__pycache__'not in p.parts}
    if (len(rows)!=48 or len(expected)!=48 or actual!=expected
            or any(Path(r['path']).resolve()!=SLAM_SOURCE_ROOT/r['relative_path']for r in rows)):
        raise RuntimeError('Published native complete48 source coverage differs')
    bindings=doc['bindings_sha256']
    if len(bindings)!=54 or any(bindings.get(path)!=digest for path,digest in expected.items()):
        raise RuntimeError('Published native54 complete bindings differ')
    for path,digest in bindings.items():
        if str(Path(path).resolve())!=path or sha(path)!=digest:
            raise RuntimeError('Published native source/build/artifact changed: '+path)
    for role,relative in [('executable','fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'),
                          ('core_library','fast_livo2_core/lib/libfast_livo2_core.so')]:
        row=doc['artifacts'][role];path=(SLAM_WORKSPACE/'install'/relative).resolve()
        if (not path.is_relative_to(package.resolve()) or Path(row['path']).resolve()!=path
                or bindings.get(str(path))!=row['sha256']):
            raise RuntimeError('Published native actual artifact path/SHA differs')
    for row in doc['compile_commands']:
        if bindings.get(str(Path(row['path']).resolve()))!=row['sha256']:
            raise RuntimeError('Published native compile-command binding differs')
        commands=json.loads(Path(row['path']).read_bytes())
        if not commands or any(str(Path(c['file']).resolve())not in expected for c in commands):
            raise RuntimeError('Published native compile used foreign source')
    evidence={}
    for name in ('layout','source_semantics_audit'):
        row=doc[name];path=Path(row['path']).resolve()
        if not path.is_relative_to(package.resolve()) or sha(path)!=row['sha256']:
            raise RuntimeError('Published native extra provenance changed: '+name)
        evidence[name]=json.loads(path.read_bytes())
    audit=evidence['source_semantics_audit'];audit_rows={r['relative']:r for r in audit['native48']}
    if (len(audit_rows)!=48 or audit.get('native_non_comment_code_difference_count')!=0
            or audit.get('native_non_cpp_input_difference_count')!=0
            or any(audit_rows[r['relative_path']]['original_sha256']!=r['original571_sha256']
                or audit_rows[r['relative_path']]['published_sha256']!=r['sha256']for r in rows)):
        raise RuntimeError('Published native original571/code-semantic mapping differs')
    flags=doc['compile_flags_comparison']
    differences=[r for r in flags if r.get('equal_after_only_source_build_install_path_relocation')is not True]
    if len(flags)!=11 or len(differences)!=1 or differences[0].get('file')!='main.cpp' or not differences[0].get('diff'):
        raise RuntimeError('Published native compile flags/include-order review differs')
    link=doc['link_audit'];core=Path(doc['artifacts']['core_library']['path']).resolve()
    if (link.get('no_missing_libraries')is not True or link.get('old_native_ELF_or_core_used')is not False
            or Path(link.get('actual_new_core','')).resolve()!=core
            or evidence['layout'].get('no_legacy_target')is not True):
        raise RuntimeError('Published native link/layout source differs')
    return doc

def verify_slam_workspace():
    if PRIVATE:
        value,doc=PRIVATE
        if sha(value['component_path'])!=value['component_sha256']:raise RuntimeError('Private native component receipt changed')
        if hashlib.sha256(Path(value['manifest_path']).read_bytes()).hexdigest()!=value['manifest_sha256']:
            raise RuntimeError('Private native build manifest changed')
        if doc.get('schema')=='published_formal22_native571_build/v1':
            return _verify_published_workspace(value,doc)
        if (doc.get('baseline_commit')!='e4808b8ab197081812bd53bb8184c6aee050ca36'
                or doc.get('candidate_commit')!='571161b251fbd024ce755c2dc3301a2120e8637b'
                or doc.get('source_file_count')!=48 or doc.get('source_unchanged_count')!=46
                or doc.get('build',{}).get('result')!='PASS'
                or doc.get('selection',{}).get('env')!='FASTLIVO_SERIAL_IMU_PRIORITY'
                or doc['selection'].get('on_mode')!='serial'):
            raise RuntimeError('Private serial IMU build provenance differs')
        rows=doc['sources'];expected={str(Path(r['path']).resolve()):r['sha256']for r in rows}
        actual={str(p.resolve()):sha(p)for package in ('fast_livo2_core','fast_livo2_ros')
            for p in (SLAM_SOURCE_ROOT/package).rglob('*')if p.is_file()and '__pycache__'not in p.parts}
        changed=sorted(r['relative']for r in rows if r['sha256']!=r['baseline_sha256'])
        if (len(rows)!=48 or len(expected)!=48 or actual!=expected
                or any(Path(r['path']).resolve()!=SLAM_SOURCE_ROOT/r['relative']for r in rows)
                or changed!=['fast_livo2_core/include/fast_livo2_core/core/LIVMapper.h','fast_livo2_core/src/LIVMapper.cpp']):
            raise RuntimeError('Private native complete source/delta differs')
        bindings=doc['bindings_sha256']
        if any(bindings.get(path)!=digest for path,digest in expected.items()):raise RuntimeError('Private native source coverage missing')
        for path,digest in bindings.items():
            if sha(path)!=digest:raise RuntimeError('Private native source/build/artifact changed: '+path)
        for role,relative in [('executable','install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'),('core_library','install/fast_livo2_core/lib/libfast_livo2_core.so')]:
            row=doc['artifacts'][role];path=(SLAM_WORKSPACE/relative).resolve()
            if Path(row['path']).resolve()!=path or bindings.get(str(path))!=row['sha256']:
                raise RuntimeError('Private native artifact binding differs')
        for row in doc['compile_commands']:
            commands=json.loads(Path(row['path']).read_bytes())
            if not commands or any(str(Path(c['file']).resolve())not in expected for c in commands):
                raise RuntimeError('Private native compilation used foreign source')
        return doc
    d=json.loads(CONTRACT.read_text());stage=SLAM_WORKSPACE.parent
    if (d.get('schema')!='R77_explicit_point_identity_workspace/v1' or d.get('workspace')!=str(SLAM_WORKSPACE)
        or d.get('build_copied')is not False or d.get('full_source_build_complete')is not True
        or d.get('runtime_acceptance_inherited')is not False or d.get('actual_navigation_verified')is not False
        or d.get('actual_loaded_binary_witness_required')is not True
        or (d.get('LIO_threads'),d.get('VIO_threads'),d.get('Teacher_CPU_threads'))!=(4,1,1)):
        raise RuntimeError('R71 workspace scope differs')
    fix=json.loads((stage/'NATIVE_SOURCE_CHANGE.json').read_text());old=Path(fix['baseline_workspace'])
    actual={str(p.relative_to(SLAM_WORKSPACE/'src')):sha(p)for p in (SLAM_WORKSPACE/'src').rglob('*')if p.is_file()and '__pycache__'not in p.parts}
    for rel,digest in fix['baseline_sources'].items():
        if sha(old/'src'/rel)!=digest:raise RuntimeError('Protected original native source changed: '+rel)
    delta=sorted(k for k,v in actual.items()if v!=fix['baseline_sources'].get(k))
    if (actual!=fix['candidate_sources'] or delta!=fix['changed_source_relative_paths'] or delta!=d['changed_source_relative_paths']
        or 'fast_livo2_core/src/IMU_Processing.cpp'in delta or set(fix['baseline_sources'])-set(actual)):
        raise RuntimeError('R71 explicit native identity delta differs')
    if set(d['bindings_sha256'])!={str(p)for p in required_workspace_files()}:raise RuntimeError('R71 incomplete source/build/ELF bindings')
    for p,h in d['bindings_sha256'].items():
        if sha(p)!=h:raise RuntimeError('R71 source/build/ELF changed: '+p)
    if d['build_receipt']!=dict(path=str(CORE_RECEIPT),sha256=sha(CORE_RECEIPT)):raise RuntimeError('R71 build receipt changed')
    r=json.loads(CORE_RECEIPT.read_text())
    if (r.get('full_source_build_complete')is not True or r.get('build_returncode')!=0 or r.get('source_only_initial_copy')is not True
        or r.get('build_copied')is not False or r.get('old_workspace_modified')is not False or r.get('runtime_authorized_by_build_receipt')is not False):
        raise RuntimeError('R71 independent build not evidenced')
    for p,h in r['build_attempt_evidence'].items():
        if sha(p)!=h:raise RuntimeError('R71 actual build log changed')
    for pkg in ('fast_livo2_core','fast_livo2_ros'):
        cmds=json.loads((SLAM_WORKSPACE/'build'/pkg/'compile_commands.json').read_text())
        if not cmds or any(not Path(x['file']).resolve().is_relative_to(SLAM_WORKSPACE/'src')for x in cmds):raise RuntimeError('R71 inherited compilation source')
    return d
def evidence_files():
    d=verify_slam_workspace()
    if PRIVATE:
        extra=([Path(d[name]['path'])for name in ('layout','source_semantics_audit')]
            if d.get('schema')=='published_formal22_native571_build/v1'else [])
        return [Path(__file__).resolve(),SELECTION,Path(PRIVATE[0]['manifest_path']),Path(PRIVATE[0]['component_path']),
            *extra,*[Path(p)for p in d['bindings_sha256']]]
    return [Path(__file__).resolve(),CONTRACT,CORE_RECEIPT,SLAM_WORKSPACE.parent/'NATIVE_SOURCE_CHANGE.json',*[Path(p)for p in d['bindings_sha256']]]
