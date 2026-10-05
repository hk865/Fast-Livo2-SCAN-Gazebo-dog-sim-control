"""Root review gate for a new finite experiment; never certifies navigation."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

if not __debug__:
    raise RuntimeError('Do not disable review assertions')
ROOT=Path(__file__).resolve().parents[3]
HERE=ROOT/'navigation/pipeline_v19'
RESULT=Path(__file__).resolve().parent.parent


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    core_path=RESULT/'core/CORE_PREFLIGHT.json';core=json.loads(core_path.read_text())
    if core['status']!='PASS_LIMITED_CORE_FOR_ROOT_REVIEW' or core['runtime_authorized_by_this_file'] is not False:
        raise RuntimeError('Wrong finite core receipt')
    bindings={str(core_path):sha(core_path)}
    for rel,digest in core['source_bindings'].items():
        path=HERE/'slam_ws/src'/rel
        if sha(path)!=digest:raise RuntimeError('Core changed '+rel)
        bindings[str(path)]=digest
    required=('independent_build','bounded_queue_lifecycle','production_packet_semantics','finite_math','FP_team_loader')
    if set(core['checks'])!=set(required):raise RuntimeError('Missing independent core checks')
    for key in required:
        row=core['checks'][key]
        if row['passed'] is not True or not row['receipts']:raise RuntimeError('Incomplete '+key)
        for receipt in row['receipts']:
            if sha(receipt['path'])!=receipt['sha256']:raise RuntimeError('Core receipt changed '+receipt['path'])
            bindings[receipt['path']]=receipt['sha256']
    for name,digest in [('fast_livo2_core/lib/libfast_livo2_core.so',core['library_sha256']),
                        ('fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',core['mapping_executable_sha256'])]:
        path=HERE/'slam_ws/install'/name
        if sha(path)!=digest:raise RuntimeError('Installed artifact changed')
        bindings[str(path)]=digest
    math_names=('pid_core.py','clock_hold.py','publication_ledger.py',
                'transition_gate.py','shared_controller.py','teacher_wrapper.py')
    for name in math_names:
        if sha(HERE/name)!=sha(HERE.parent/'combined_compute_v18'/name):
            raise RuntimeError('Unreviewed control arithmetic change: '+name)
    control_path=RESULT/'integration/CONTROL_REFERENCE_REVISION03.json'
    control=json.loads(control_path.read_text())
    if control.get('reviewed') is not True or control.get('original46_geometry_or_safety_thresholds_changed') is not False:
        raise RuntimeError('Explicit control reference correction review is required')
    for name,digest in control['reviewed_source_sha256'].items():
        if sha(HERE/name)!=digest:raise RuntimeError('Reviewed heading correction changed: '+name)
    bindings[str(control_path)]=sha(control_path)
    tests=['pipeline_lifecycle_tests','mission46_integration_tests','mission46_tests','mission46_runtime_tests','heading_reference_tests']
    result=subprocess.run([sys.executable,'-B','-m','unittest',*tests,'-v'],cwd=HERE,text=True,capture_output=True)
    log=RESULT/'integration/FINAL_FINITE_REVIEW.log';log.write_text(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError('Finite mission or lifecycle checks failed')
    bindings[str(log)]=sha(log)
    for p in HERE.glob('*.py'):
        compile(p.read_bytes(),str(p),'exec');bindings[str(p)]=sha(p)
    paths=[HERE/'PIPELINE_V19_CONTRACT.json',RESULT/'PROSPECTIVE_EVALUATION.md',
           RESULT/'evaluation/MISSION46_RUNTIME_CONTRACT.json',
           RESULT/'performance/PROSPECTIVE_PERFORMANCE_PROTOCOL.json',Path(__file__).resolve()]
    paths += [HERE/'slam_ws/build'/pkg/name for pkg in ('fast_livo2_core','fast_livo2_ros')
              for name in ('compile_commands.json','CMakeCache.txt')]
    for p in paths:bindings[str(p)]=sha(p)
    checks=dict(independent_build=True,queue_lifecycle=True,production_packet_semantics=True,
        finite_math=True,fp_team_loader=True,control_reference_revision_reviewed=True,
        lifecycle_reader_negative_tests=True,prospective_protocol=True)
    gate=dict(schema='pipeline_v19_preflight/v1',status='PASS_LIMITED_NEW_EXPERIMENT',
        candidate_root=str(HERE),allowed=True,actual_navigation_verified=False,historical_pass_inherited=False,
        checks=checks,verified_inputs_sha256=bindings,
        verified_mode_combinations=[dict(mode=m,image_copy_opt=c)for m in core['modes']for c in core['copy_opts']],
        limits=core['limits']+['Mission finite tests do not certify real behavior; actual original46 requires independent evidence.'])
    path=HERE/'PIPELINE_V19_PREFLIGHT.json'
    if path.exists():raise RuntimeError('Use an explicit reviewed revision rather than overwrite a frozen gate')
    path.write_text(json.dumps(gate,indent=2)+'\n')
    print(json.dumps({'path':str(path),'sha256':sha(path),'navigation_verified':False}))


if __name__=='__main__':main()
