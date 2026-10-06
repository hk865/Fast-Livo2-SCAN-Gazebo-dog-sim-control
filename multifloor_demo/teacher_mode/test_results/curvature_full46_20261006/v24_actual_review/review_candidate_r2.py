#!/usr/bin/env python3
"""Independent source/finite review; reads no actual raw run streams."""
from pathlib import Path
import hashlib, json, sys, unittest, io

OUT=Path(__file__).resolve().parent
TEACHER=OUT.parents[2]
HERE=TEACHER/'navigation/corridor_tracking_v24_recovery_replan'
OLD=HERE.parent/'corridor_tracking_v23_curvature_full46'
BASE=HERE.parent/'corridor_tracking_v20/profiles/pipeline_staged_original46.json'
sys.path.insert(0,str(HERE))

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def save(name,d):
    p=OUT/name
    if p.exists(): raise FileExistsError(p)
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
    return p

import recovery_replan_tests as repair
import full46_launch_tests as launch
from replan_policy import low_command_is_exhausted_path
from geometry_archive import geometry_helper_files
from full46_launch_contract import validate_full46

# Re-run finite production AST and geometry/closure negatives; original archive
# case is reviewed via its bound completed receipt, not read for a second time.
loader=unittest.TestLoader()
selected=[name for name in loader.getTestCaseNames(repair.RecoveryReplanChecks)
          if name!='test_old_actual_recovery_zero_trigger_reproduced_new_path_preserved']
suite=unittest.TestSuite([repair.RecoveryReplanChecks(name) for name in selected])
suite.addTests(loader.loadTestsFromTestCase(launch.Full46LaunchChecks))
log=io.StringIO(); result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
assert result.wasSuccessful(),log.getvalue()
logfile=OUT/'independent_finite_r2.log'
assert not logfile.exists()
logfile.write_text(log.getvalue())
truth_cases=0
for mode in ('drive','path_end_hold','protect','recovering','pre_turn','settle','reference_constraint_hold','unknown',None):
    for exhausted in (False,True):
        for progress in (False,True):
            expected=mode=='path_end_hold' or (mode=='drive' and (exhausted or not progress))
            assert low_command_is_exhausted_path(cascade_mode=mode,exhausted=exhausted,has_geometric_progress=progress) is expected
            truth_cases+=1

# Independently exercise both real cascade and existing geometric follower at
# opposite sides of the two different terminal thresholds.
from control_core import follow_trajectory
import numpy as np
boundary_cases=[]
for remaining in (.020,.026,.040,.0499,.0501,.060):
    pose=(3.-remaining,0.,.32); c=repair.controller(goal=(4.,0.,.32))
    path=np.array([[0.,0.,.32],[3.,0.,.32]])
    c.set_path(path,'independent-boundary',0,0)
    c.update(*repair.sources(100_000_000,position=pose),mode_override='drive')
    cmd,row=c.update(*repair.sources(200_000_000,position=pose),mode_override='drive')
    _,_,_,steering=follow_trajectory(np.asarray(pose),np.eye(3),path,np.array([4.,0.,.32]),gate_translation=False,return_steering=True)
    terminal=remaining<.05
    assert (row['mode']=='path_end_hold') is terminal
    assert steering['exhausted'] is (remaining<=.025)
    actual=low_command_is_exhausted_path(cascade_mode=row['mode'],exhausted=steering['exhausted'],has_geometric_progress=True)
    assert actual is terminal
    boundary_cases.append(dict(remaining_m=remaining,core_mode=row['mode'],legacy_exhausted=steering['exhausted'],replan_predicate=actual))

changes=[p.name for p in sorted(HERE.glob('*.py')) if (OLD/p.name).exists() and p.read_bytes()!=(OLD/p.name).read_bytes()]
assert changes==['corridor_preflight.py','full46_launch_contract.py','full46_launch_tests.py','pid_scope.py','run.py','shared_controller.py'],changes
unchanged={p.name:sha(p) for p in HERE.glob('*.py') if (OLD/p.name).exists() and p.read_bytes()==(OLD/p.name).read_bytes()}
for key in ('controller.py','cascade_core.py','spatial_reference.py','transition_gate.py','route_fence.py','mission46_guard.py','teacher_wrapper.py','bridge.py','worker.py','clock_hold.py','geometry_archive.py'):
    assert key in unchanged
profile_path=HERE/'profiles/curvature_original46_on.json'
profile=read(profile_path); prior=read(OLD/'profiles/curvature_original46_on.json')
allowed=('controller_selector','profile_version','prospective_change','mission46_required_source_files')
assert {k:v for k,v in profile.items() if k not in allowed}=={k:v for k,v in prior.items() if k not in allowed}
assert [v.replace('/corridor_tracking_v24_recovery_replan/','/corridor_tracking_v23_curvature_full46/') for v in profile['mission46_required_source_files']]==prior['mission46_required_source_files']
validate_full46(profile,HERE)
finite_path=OUT.parent/'v24_recovery_replan/FINITE_RECEIPT.json'
assert sha(finite_path)=='07c5fb66ca97d312ef9dda8ad95620a38d2a60db598da98c3b9448ab4d49874f'
finite=read(finite_path)
assert finite['passed'] is True and finite['tests']==61 and finite['execution']['exit_code']==0
assert sha(finite['execution']['log'])==finite['execution']['log_sha256']
assert all(sha(p)==digest for p,digest in finite['source_bindings'].items())
diag_path=OUT.parent/'57fa_diagnosis/FINAL_INTERLOCK_DIAGNOSIS.json'
diag=read(diag_path)
examples=diag['recovery_zero_replan_examples']
assert len(examples)==4
for ex in examples:
    assert ex['before']['mode']=='recovering' and ex['before']['phase']=='drive'
    assert ex['before']['command_after_slew']==[0.,0.,0.]
    assert ex['new_reference_stamp_exactly_equals_recovery_tick'] is True and ex['same_goal'] is True
    assert 0<ex['elapsed_ms']<=40

mandatory=['geometry_archive.py','replan_policy.py','shared_controller.py','recovery_replan_tests.py','full46_launch_contract.py','run.py','pid_scope.py','corridor_preflight.py',
'controller.py','cascade_core.py','spatial_reference.py','transition_gate.py','clock_hold.py','teacher_wrapper.py','bridge.py','worker.py','route_fence.py','mission46_guard.py','mission46.py','mission46_runtime.py','mission46_runtime_evidence.py','mission46_obstacle_runtime.py','mission46_terrain.py','publication_ledger.py','pipeline_lifecycle.py','full46_launch_tests.py']
paths=[HERE/name for name in mandatory]+[profile_path,BASE,*geometry_helper_files(HERE)]
report=dict(schema='recovery_replan_v24_full46_launch_review/v1', reviewed=True,
    status='PASS_FINITE_SOURCE_REVIEW_ONLY',actual_navigation_verified=False,
    control_math_unchanged=True,profile_original46_geometry_unchanged=True,
    five_geometry_sources_direct=True,full46_only_authorization=True,original_safety_unchanged=True,
    protected_zero_not_path_exhaustion=True,genuine_exhausted_or_no_geometry_progress_replan_preserved=True,
    candidate_root=str(HERE),source_bindings={str(p.resolve()):sha(p) for p in paths},
    changed_common_python_files=changes,unchanged_parent_python_sha256=unchanged,
    finite=dict(independently_reexecuted_tests=result.testsRun,passed=True,
        truth_table_cases=truth_cases,excluded_from_second_read='Original actual-archive reproduction test; covered by bound 61-test receipt',
        log=str(logfile),log_sha256=sha(logfile),prior_receipt=str(finite_path),prior_receipt_sha256=sha(finite_path),prior_test_count=61),
    diagnostic_compact=dict(path=str(diag_path),sha256=sha(diag_path),recovery_chain_count=4,
        intervals_ms=[x['elapsed_ms'] for x in examples]),
    conclusions=[
        'Only the old low-command replan condition gained a path-state predicate; production AST is identical after removing that import and final conjunct.',
        'Recovering/protect/settle/pre_turn/constraint zeros cannot clear a valid active path through this heuristic. This does not authorize any nonzero command.',
        'Explicit steering exhausted branch, obstacle recovery, reference_constraint_hold replan after 3s, measured goal/dwell, region 90s and source 300ms guards remain unchanged.',
        'The progress predicate measures existing spline arc/extent, not time-based physical robot progress.',
        'V24 profile control/geometry tree equals V23 except candidate metadata/source paths; full original V20 baseline geometry stays exact with curvature flags ON.',
        'New exact full46 launch gate requires fresh 61-test result and tested source pins; prefix9 or historical navigation acceptance cannot authorize this run.'],
    scope=dict(raw_JSONL_read=False,actual_NPZ_reread=False,ROS_started=False,Gazebo_started=False,frozen_sources_modified=False,
        full46_acceptance=False,physical_200Hz_replay=False,remaining_SCAN_path_shape_causes_unverified=True,
        gate_and_prepare_verified=False),
    review_script=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
report['supersedes_withdrawn_review']={'path':str(OUT/'FULL46_REVIEW.json'),'sha256':sha(OUT/'FULL46_REVIEW.json'),'withdrawal':str(OUT/'REVIEW_WITHDRAWAL.json')}
report['path_end_boundary_cases']=boundary_cases
p=save('FULL46_REVIEW_R2.json',report)
print(json.dumps(dict(path=str(p),sha256=sha(p),independent_tests=result.testsRun,truth_table_cases=truth_cases,source_pins=len(report['source_bindings'])),indent=2))
