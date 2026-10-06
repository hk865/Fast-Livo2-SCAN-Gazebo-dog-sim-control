#!/usr/bin/env python3
"""Read-only source closure of a prepared V24 run; no physics or raw streams."""
from pathlib import Path
import json,hashlib,sys,copy
OUT=Path(__file__).resolve().parent
TEACHER=OUT.parents[2]; HERE=TEACHER/'navigation/corridor_tracking_v24_recovery_replan'
sys.path.insert(0,str(HERE))
from geometry_archive import verify_geometry_archive
from mission46_profile import required_source_files
from full46_launch_contract import validate_full46

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
run=Path(sys.argv[1]).resolve()
assert run.name.endswith('v24_recovery_replan_prepare_09f7')
metadata={name:read(run/name) for name in ('navigation_scope.json','navigation_source_snapshots.json','source_manifest.json','navigation_profile.json','navigation_scenario.json','mission46_runtime_interfaces.json')}
scope=metadata['navigation_scope.json']; snapshots=metadata['navigation_source_snapshots.json']; manifest=metadata['source_manifest.json']; profile=metadata['navigation_profile.json']; refs=scope['references']
assert Path(scope['run_dir']).resolve()==run
assert scope['allowed'] is True and scope['navigation_is_verified'] is False
assert scope['navigation_ground_truth_used'] is False
assert scope['runtime']['expected_launch_children']==13
assert profile==scope['profile']==read(HERE/'profiles/curvature_original46_on.json')
assert metadata['navigation_scenario.json']==profile['original_scenario']
validate_full46(profile,HERE)
five=verify_geometry_archive(run,refs,snapshots,manifest,HERE)
review=OUT/'FULL46_REVIEW_R2.json'; gate=HERE/'RECOVERY_REPLAN_V24_PREFLIGHT.json';g=read(gate)
assert sha(gate)=='2dd153f395f9342a338084470f90e8f2ca0e2ac886ac780eeeb5e026802e4fd5'
assert g['full46_review']=={'path':str(review),'sha256':sha(review)}
assert g['actual_navigation_verified'] is False and g['historical_pass_inherited'] is False
assert g['allowed_profiles']==['curvature_original46_on.json']
assert all(g['verified_inputs_sha256'][p]==digest and sha(p)==digest for p,digest in read(review)['source_bindings'].items())

def check_source(p):
    p=Path(p).resolve(); key=str(p); expected=sha(p)
    assert refs[key]==expected
    row=snapshots[key];assert row['sha256']==expected
    rel=p.relative_to(TEACHER) if p.is_relative_to(TEACHER) else Path('external')/(expected[:16]+'_'+p.name)
    target=run/'sources'/rel
    assert not target.is_symlink() and target.is_file()
    assert Path(row['snapshot']).resolve()==target.resolve() and target.resolve().is_relative_to(run/'sources')
    assert sha(target)==expected and refs[str(target)]==expected
    assert manifest[str(rel)]==expected and manifest[key]==expected
    return dict(source=key,snapshot=str(target),sha256=expected)
full=[check_source(p) for p in required_source_files()]
repair=[check_source(HERE/n) for n in ('shared_controller.py','replan_policy.py','controller.py','cascade_core.py','spatial_reference.py')]
closure=[check_source(p) for p in (gate,review)]
for row in metadata['mission46_runtime_interfaces.json'].values():
    assert row['supported'] is True and sha(row['module'])==row['implementation_sha256']
    check_source(row['module'])
# Negative source-closure checks use copied metadata only, never mutate a file.
negatives=[]
for key in five:
    bad=copy.deepcopy(snapshots);bad.pop(key)
    try: verify_geometry_archive(run,refs,bad,manifest,HERE)
    except RuntimeError: negatives.append('missing_direct_snapshot:'+key)
    else: raise AssertionError('Missing helper snapshot accepted')
receipt=dict(schema='recovery_replan_v24_independent_prepare_archive_review/v1',status='PASS_PREPARE_SOURCE_CLOSURE_ONLY',passed=True,actual_navigation_verified=False,
    run=str(run),gate=dict(path=str(gate),sha256=sha(gate)),review=dict(path=str(review),sha256=sha(review)),
    metadata_sources={str(run/name):dict(sha256=sha(run/name),bytes=(run/name).stat().st_size) for name in metadata},
    five_helpers=five,full46_required_sources=full,recovery_and_control_sources=repair,gate_review_snapshots=closure,
    five_missing_snapshot_negatives=negatives,exact_original_scenario=True,expected_launch_children=13,
    scope=dict(raw_streams_read=False,ROS_started=False,Gazebo_started=False,whole_200Hz_replay=False,full46_outcome_verified=False),
    script=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
p=OUT/'PREPARE_SOURCE_CLOSURE.json';assert not p.exists();p.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=sha(p),helpers=len(five),full46_sources=len(full),negative_cases=len(negatives)),indent=2))
