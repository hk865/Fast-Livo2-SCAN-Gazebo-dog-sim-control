"""Install reviewed observer sources into a new exclusive R39 source directory.

Preserves R38, all profiles, the protected policy/native and budget baseline.
No simulator, ROS node, Actor, build, deletion or historical pointer change.
"""
from pathlib import Path
import datetime
import hashlib
import importlib.util
import json
import shutil

N=Path(__file__).resolve().parent
B=Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_r38_event_any_phase')
C=B.with_name('engineering_v34_r39_p1_association')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
assert not C.exists(), 'exclusive candidate already exists'
gate=json.loads((B/'R33_STARTUP_PREFLIGHT.json').read_text())
for p,h in gate['verified_inputs_sha256'].items():assert sha(p)==h,p
finite=json.loads((N/'FINITE_CPU_RECEIPT.json').read_text())
assert finite['tests_pass'] and finite['failures']==0 and finite['tests_run']==10
for p,h in finite['source_sha256'].items():assert sha(p)==h,p
shutil.copytree(B,C,ignore=shutil.ignore_patterns('__pycache__'))
relocated=[]
for p in C.rglob('*'):
    if not p.is_file() or p.suffix not in ('.py','.json','.md','.yaml','.txt','.sh'):continue
    if p.parent.name=='profiles':continue
    old=p.read_text();new=old.replace(B.name,C.name)
    if new!=old:p.write_text(new);relocated.append(str(p.relative_to(C)))
for name in ['event_aware_source_capture.py','known_scene_adapter.py','first_goal_stop.py','run.py']:
    (C/name).write_bytes((N/name).read_bytes())
for p in (B/'profiles').glob('engineering_*.json'):assert sha(p)==sha(C/'profiles'/p.name)
contract=json.loads((C/'R33_SHORT_STARTUP_CONTRACT.json').read_text())
contract.update(schema='R39_original_association_collision_evidence_contract/v1',
    source_stage=str(N),candidate_root=str(C),phase='first_original_NN_collision_bounded_capture',
    max_native_seconds=120,max_wall_seconds=900,new_output_cap_bytes=1_000_000_000,
    event_sampling_window_max_native_seconds=15,preserved_shared_high_water_floor_bytes=47_772_774_400,
    base_R38_source_root=str(B),base_R38_source_gate_sha256=sha(B/'R33_STARTUP_PREFLIGHT.json'),
    shadow_diagnostic_combined=False,maximum_simulator_attempts=1)
write(C/'R33_SHORT_STARTUP_CONTRACT.json',contract)
spec=importlib.util.spec_from_file_location('P1_install_constants',N/'event_aware_source_capture.py')
capture_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture_module)
capture=json.loads((C/'EVENT_CAPTURE_CONTRACT_TEMPLATE.json').read_text())
capture.update(schema='go2_P1_association_source_capture_contract/v1',caps=capture_module.CAPS,
    unchanged_gates=capture_module.GATES,topics=capture_module.TOPICS,
    diagnostic_trigger=capture_module.TRIGGER,
    runtime_budget=dict(native_seconds=120,wall_seconds=900,new_output_cap_bytes=1_000_000_000,
        shared_hard_cap_bytes=80_000_000_000,original_profile_duration_seconds=1500),
    limitations=['Original exact source rejection and all map/control gates unchanged.',
        'Direct original evaluated NN arrays; writer does not rerun or replace pairing.',
        'PRE/POST are association-checked jobs, not automatically accepted control.',
        'Absent point-ID/stage commit remains UNVERIFIED.',
        'Complete received ROS message reserialization, not original wire CDR.',
        'No GTpose/IMU world quaternion/frontend z prior; no fabricated point timing.',
        'IMU arrival/coverage does not establish frontend consumption; require_complete_imu=false.',
        'R6 formal eight-group contract remains separate and unchanged.',
        'No online shadow scoring; fixed16MiB logical and allocated capture cap.'])
capture['authorization']['delegation']='Root authorized bounded P1 original association collision evidence; one attempt after fresh review/freeze'
for item in capture['source_bindings']:
    item['path']=item['path'].replace(str(B),str(C));item['sha256']=sha(item['path'])
write(C/'EVENT_CAPTURE_CONTRACT_TEMPLATE.json',capture)
for name in ['R33_STARTUP_PREFLIGHT.json','R33_STARTUP_SOURCE_REVIEW.json']:
    (N/('COPIED_STALE_'+name)).write_bytes((C/name).read_bytes())
old_gate=json.loads((C/'R33_STARTUP_PREFLIGHT.json').read_text())
old_gate.update(allowed=False,status='INVALID_COPIED_GATE_REQUIRES_FRESH_R39_REVIEW')
write(C/'R33_STARTUP_PREFLIGHT.json',old_gate)
changed={}
for p in C.rglob('*'):
    if p.is_file() and p.suffix in ('.py','.json','.md','.yaml','.txt','.sh'):
        original=B/p.relative_to(C)
        if original.is_file() and sha(original)!=sha(p):
            changed[str(p.relative_to(C))]=dict(before_sha256=sha(original),after_sha256=sha(p))
receipt=dict(schema='R39_exclusive_candidate_source_install/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    baseline=str(B),candidate=str(C),path_metadata_relocations=relocated,changed_files=changed,
    executable_diagnostic_sources=['event_aware_source_capture.py','known_scene_adapter.py','first_goal_stop.py','run.py'],
    all_five_profile_bytes_unchanged=True,shared_source_modified=False,original_admission_changed=False,
    source_gate_valid=False,finite_cases_passed=10,Actor_ROS_Gazebo_started=False,
    source_sha256={str(C/x):sha(C/x) for x in ['event_aware_source_capture.py','known_scene_adapter.py','first_goal_stop.py','run.py','R33_SHORT_STARTUP_CONTRACT.json','EVENT_CAPTURE_CONTRACT_TEMPLATE.json']})
write(N/'ISOLATED_CANDIDATE_COPY.json',receipt)
print(json.dumps({k:v for k,v in receipt.items() if k not in ('changed_files','source_sha256')},ensure_ascii=False))
