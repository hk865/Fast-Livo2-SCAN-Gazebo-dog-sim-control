# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""Fresh V34 source review, explicitly reusing frozen V32 build provenance."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

GATE_NAME='R33_STARTUP_PREFLIGHT.json'
REVIEW_NAME='R33_STARTUP_SOURCE_REVIEW.json'
CHANGED_ADAPTERS=frozenset(('controller.py', 'corridor_preflight.py', 'full46_launch_contract.py', 'mission46.py', 'mission46_contract.py', 'mission46_obstacle_runtime.py', 'mission46_runtime.py', 'mission46_runtime_evidence.py', 'pid_scope.py', 'run.py', 'sensor_gate.py', 'shared_controller.py', 'slam.launch.py', 'stack.launch.py', 'teacher_wrapper.py'))

CHANGED_ADAPTERS=CHANGED_ADAPTERS|frozenset(('slam_workspace.py','known_scene_adapter.py','known_scene_runtime.py','startup_trace.py','owned_guardian.py','pipeline_lifecycle.py'))

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_original_v32(here):
    original=Path(here).resolve().parent/'map_surface_v32'
    # The V32 owner appended its actual-outcome documentation after launch.
    # Audit this exact non-executable delta against the retained prelaunch copy;
    # never rewrite its historical gate or inherit its navigation result.
    gate=json.loads((original/'MAP_SURFACE_V32_PREFLIGHT.json').read_text())
    archived=original.parents[1]/'runs/20261007_153935_closed_loop_cascade_clock_hold_V32_surface_candidate_r2_a6c9/sources/navigation/map_surface_v32/README.md'
    doc=original/'README.md'
    expected_doc=''
    for name,digest in gate['verified_inputs_sha256'].items():
        if Path(name)==doc:
            if sha(doc)!=expected_doc or sha(archived)!=digest:
                raise RuntimeError('Reviewed V32 outcome-document delta changed')
        elif sha(name)!=digest:raise RuntimeError('Frozen V32 executable/test/profile provenance changed: '+name)
    code=('import json,sys;from pathlib import Path;'
        'p=Path(sys.argv[1]);sys.path.insert(0,str(p));'
        'from corridor_preflight import verify_original_v28,unchanged_navigation_bindings;'
        'from full46_launch_contract import validate_full46;'
        'from slam_workspace import verify_slam_workspace;'
        'verify_original_v28(p);unchanged_navigation_bindings(p);verify_slam_workspace();'
        'validate_full46(json.loads((p/"profiles/repair_original46.json").read_text()),p);'
        'd=json.loads((p/"MAP_SURFACE_V32_PREFLIGHT.json").read_text());'
        'print(json.dumps({k:d[k] for k in ("schema","status","candidate_root","actual_navigation_verified")}))')
    r=subprocess.run([sys.executable,'-B','-c',code,str(original)],capture_output=True,text=True,timeout=60)
    if r.returncode:raise RuntimeError('Frozen V32 provenance changed: '+r.stderr[-2000:])
    d=json.loads(r.stdout)
    if (d['schema']!='map_surface_v32_preflight/v1' or d['status']!='PASS_LIMITED_MAP_SURFACE_EXPERIMENT'
        or d['candidate_root']!=str(original) or d['actual_navigation_verified'] is not False):
        raise RuntimeError('Wrong V32 limited provenance')
    d['historical_gate_currently_invalid_for_original_launch_due_to_postrun_README']=True
    d['reviewed_nonexecuted_document_delta']=dict(path=str(doc),current_sha256=expected_doc,
        archived_prelaunch_path=str(archived),archived_sha256=sha(archived),
        reason='Added candidate_r2 command and actual outcomes; no runtime source/config/build change')
    return d

def declared_R37_loading_budget_bindings(here):
    # Three exact, reviewed implementation deltas. All safety behavior is frozen.
    here=Path(here).resolve();base=here.with_name('engineering_v34_r36_full_route_acceptance')
    frozen=json.loads((base/'R33_STARTUP_PREFLIGHT.json').read_text())['verified_inputs_sha256']
    rows={}
    for name in ('worker.py','storage_guard.py','mission46_profile.py'):
        original=base/name;candidate=here/name
        if sha(original)!=frozen[str(original)]:raise RuntimeError('Protected R36 baseline changed: '+name)
        if name=='worker.py':
            expected=original.read_text().replace('    provider=None\n',
                '    from policy_selection import record_actual_loaded\n    record_actual_loaded(run,legacy)\n    provider=None\n')
            valid=candidate.read_text()==expected
            reason='actual selected private policy path/hash witness; original worker behavior retained'
        elif name=='storage_guard.py':
            recovery=here/'STORAGE_REBOOT_RECOVERY_CONTRACT.json'
            if recovery.exists():
                rc=json.loads(recovery.read_text())
                protected=Path(rc['archived_original_storage_guard'])
                valid=(sha(protected)==''
                    and sha(candidate)==rc['recovery_storage_guard_sha256']
                    and sha(here/'storage_recovery.py')==rc['recovery_helper_sha256'])
                reason='explicit reviewed reboot recovery; original80GB limits and accounting baseline retained'
            else:
                valid=sha(candidate)==''
                reason='independently reviewed80GB cumulative guard; historical baseline retained'
        else:
            expected=original.read_text().replace('RAW_BUDGET_BYTES = 50_000_000_000','RAW_BUDGET_BYTES = 80_000_000_000')
            valid=candidate.read_text()==expected
            reason='authorized80GB budget constant only'
        if not valid:raise RuntimeError('Undeclared R37 loading/budget edit: '+name)
        rows[name]=dict(baseline=str(original),candidate=str(candidate),sha256=sha(candidate),
            baseline_sha256=sha(original),declared_change=reason)
    return rows

def unchanged_navigation_bindings(here):
    here=Path(here).resolve();base=here.parent/'engineering_v33';rows=declared_R37_loading_budget_bindings(here)
    for p in sorted(base.glob('*.py')):
        if p.name in CHANGED_ADAPTERS or p.name in rows:continue
        q=here/p.name
        if sha(q)!=sha(p):raise RuntimeError('Undeclared V34 edit: '+p.name)
        rows[p.name]=dict(baseline=str(p),candidate=str(q),sha256=sha(p))
    if not {'cascade_core.py','path_admission.py','clock_hold.py','spatial_reference.py','worker.py','bridge.py'}.issubset(rows):
        raise RuntimeError('Original safety/control sources missing')
    return rows

def evidence_files(here):
    p=Path(here)/GATE_NAME
    if not p.is_file():raise RuntimeError('V34 source review not frozen yet')
    d=json.loads(p.read_text())
    return [p,*[Path(x) for x in d['verified_inputs_sha256']]]

def required_review_files(here,profile=None):
    here=Path(here).resolve()
    from full46_launch_contract import PROFILE_NAMES,baseline_profile_path
    from mission46_profile import required_source_files
    from geometry_archive import geometry_helper_files
    from slam_workspace import evidence_files as slam_evidence
    from scan_workspace import evidence_files as scan_evidence
    base=here.parent/'map_surface_v32'
    from policy_selection import source_files as policy_source_files
    required=list(here.glob('*.py'))+policy_source_files()+list((here/'profiles').glob('engineering_*.json'))
    required += [p for p in here.glob('*.json') if p.name not in (GATE_NAME,REVIEW_NAME)]
    required += list(base.glob('*.py'))+list(base.glob('*.json'))
    required += [baseline_profile_path(here,n) for n in PROFILE_NAMES]
    required += [base.parents[1]/'runs/20261007_153935_closed_loop_cascade_clock_hold_V32_surface_candidate_r2_a6c9/sources/navigation/map_surface_v32/README.md']
    required += [*required_source_files(),*geometry_helper_files(here),*slam_evidence(),*scan_evidence(profile)]
    helper=here.parent/'evidence_runtime_v32'
    required += list(helper.glob('*.py'))+list(helper.glob('*.json'))
    root=here.parents[1]
    required += [here/'R33_SHORT_STARTUP_CONTRACT.json', Path('multifloor_demo/teacher_mode/simulation/native/current/teacher_actuator.cpp'), Path('multifloor_demo/teacher_mode/simulation/native/current/diagnostic_collision_guard.hpp'), Path('external/omitted-history/libteacher_actuator.so'), Path('external/omitted-history/NATIVE_GUARD_TEST_RECEIPT.json')]
    required += [here/'README_V34.md',here/'SUBMAP_SHADOW.md',base/'README.md',root/'simulation/teacher_actuator.cpp',
        root/'simulation/build/libteacher_actuator.so',root/'policy/worker.py',
        root/'policy/observation.py',root/'policy/contract.json',root/'runs/acceptance.json']
    return sorted(set(p.resolve() for p in required))

def validate_review_evidence(here,tests,reviews):
    """Reject known failed/stale reports; root also reads findings and code."""
    here=Path(here).resolve()
    for name in tests:
        d=json.loads(Path(name).read_text())
        if 'tests_pass' in d:passed=d['tests_pass'] is True
        elif d.get('schema')=='teacher_ipc_cpu_validation/v1':
            passed=d.get('tests_passed')==d.get('tests_run') and d.get('tests_run',0)>0
        elif d.get('schema')=='submap_shadow_synthetic_cpu_test_receipt/v1':
            passed=d.get('passed') is True and d.get('failures')==0 and d.get('errors')==0
        elif d.get('schema')=='go2_v33_root_finite_tests/v1':
            passed=d.get('status')=='PASS_FINITE_ONLY' and all(x['returncode']==0 for x in d['tests'])
        elif d.get('schema')=='R33_startup_finite_CPU_receipt/v1':
            passed=d.get('tests_pass') is True and d.get('failures')==0
        else:raise RuntimeError('Unknown finite test receipt: '+str(name))
        if not passed:raise RuntimeError('Failed finite tests: '+str(name))
        for key in ('source_sha256','source_hashes'):
            for source,value in d.get(key,{}).items():
                digest=value.get('sha256') if isinstance(value,dict) else value
                p=Path(source) if Path(source).is_absolute() else here/source
                if sha(p)!=digest:raise RuntimeError('Test source changed: '+str(p))
        for row in d.get('tests',[]):
            if sha(Path(name).parent/row['test'])!=row['sha256']:
                raise RuntimeError('Root test source changed')
        for source,row in d.get('code',{}).items():
            digest=row.get('sha256') if isinstance(row,dict) else row
            p=Path(source) if Path(source).is_absolute() else here/source
            if sha(p)!=digest:raise RuntimeError('IPC tested source changed')
    for name in reviews:
        d=json.loads(Path(name).read_text())
        if d.get('review_passed') is not True or d.get('blocking_findings')!=[]:
            raise RuntimeError('Independent review failed or not completed: '+str(name))
        for key in ('source_sha256','source_hashes'):
            for source,value in d.get(key,{}).items():
                digest=value.get('sha256') if isinstance(value,dict) else value
                p=Path(source) if Path(source).is_absolute() else here/source
                if sha(p)!=digest:raise RuntimeError('Independently reviewed source changed: '+str(p))
        if any(x.get('passed') is not True for x in d.get('checks',[])):
            raise RuntimeError('Independent finite contract check failed')

def verify_preflight(here,profile):
    here=Path(here).resolve()
    from full46_launch_contract import validate_full46,PROFILE_NAMES,profile_name
    validate_full46(profile,here);verify_original_v32(here)
    nav=unchanged_navigation_bindings(here)
    d=json.loads((here/GATE_NAME).read_text());review=json.loads((here/REVIEW_NAME).read_text())
    if (d.get('schema')!='R33_original_startup_preflight/v1' or d.get('candidate_root')!=str(here)
        or d.get('status')!='PASS_LIMITED_ENGINEERING_SOURCE_REVIEW' or d.get('allowed') is not True
        or d.get('actual_navigation_verified') is not False or d.get('historical_pass_inherited') is not False
        or d.get('allowed_profiles')!=list(PROFILE_NAMES)
        or review.get('unchanged_navigation_bindings')!=nav
        or review.get('root_review_complete') is not True or review.get('separate_contract_review_complete') is not True
        or not review.get('finite_test_evidence') or not review.get('independent_review_evidence')):
        raise RuntimeError('V34 finite tests/source review incomplete')
    bindings=d['verified_inputs_sha256']
    validate_review_evidence(here,review['finite_test_evidence'],review['independent_review_evidence'])
    required=[*required_review_files(here,profile),here/REVIEW_NAME]
    if any(str(p) not in bindings for p in required):raise RuntimeError('V34 inputs incompletely bound')
    for name,digest in bindings.items():
        if not Path(name).is_absolute() or sha(name)!=digest:raise RuntimeError('V34 frozen source/evidence changed: '+name)
    if profile!=json.loads((here/'profiles'/profile_name(profile)).read_text()):
        raise RuntimeError('Only the exact frozen V34 profile may run')
    return d

def freeze_source_review(here,finite_test_evidence,independent_review_evidence,*,root_review_complete=False):
    """Only the integration owner calls after inspecting concrete reports and code."""
    if root_review_complete is not True or not finite_test_evidence or not independent_review_evidence:
        raise RuntimeError('Concrete tests and independent review required')
    here=Path(here).resolve();verify_original_v32(here);nav=unchanged_navigation_bindings(here)
    validate_review_evidence(here,finite_test_evidence,independent_review_evidence)
    from full46_launch_contract import PROFILE_NAMES,validate_full46
    for n in PROFILE_NAMES:validate_full46(json.loads((here/'profiles'/n).read_text()),here)
    review=dict(schema='R33_original_startup_source_review/v1',candidate_root=str(here),
        actual_navigation_verified=False,historical_pass_inherited=False,root_review_complete=True,
        separate_contract_review_complete=True,unchanged_navigation_bindings=nav,
        finite_test_evidence=[str(Path(x).resolve()) for x in finite_test_evidence],
        independent_review_evidence=[str(Path(x).resolve()) for x in independent_review_evidence],
        limitations=['Finite tests permit bounded simulation only; no native stop or full46 success inherited',
            'No cloud/raw source replay acceptance when archive deliberately disabled',
            'Known-scene static-map assistance is not independent sensor-map generalization; actual46 coverage and final parking require new runtime evidence'])
    (here/REVIEW_NAME).write_text(json.dumps(review,indent=2)+'\n')
    inputs=set(required_review_files(here))|{here/REVIEW_NAME}|{
        Path(x).resolve() for x in [*finite_test_evidence,*independent_review_evidence]}
    gate=dict(schema='R33_original_startup_preflight/v1',candidate_root=str(here),
        status='PASS_LIMITED_ENGINEERING_SOURCE_REVIEW',allowed=True,
        actual_navigation_verified=False,historical_pass_inherited=False,
        allowed_profiles=list(PROFILE_NAMES),verified_inputs_sha256={str(x):sha(x) for x in sorted(inputs)})
    (here/GATE_NAME).write_text(json.dumps(gate,indent=2)+'\n')
    for n in PROFILE_NAMES:verify_preflight(here,json.loads((here/'profiles'/n).read_text()))
    return gate
