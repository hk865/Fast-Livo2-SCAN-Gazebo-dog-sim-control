"""Fresh R39 gate from finite evidence and the sealed independent review.

The owner's schema adapter preserves the independent report verbatim; it only
provides the list-shaped checks required by the existing preflight validator.
GPU preflight remains blocked and no runtime authorization is fabricated.
"""
from pathlib import Path
import datetime
import hashlib
import json
import sys

N=Path(__file__).resolve().parent
C=Path('/__LOCAL_HOME__/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/engineering_v34_r39_p1_association')
sys.path.insert(0,str(C))
import corridor_preflight as cp
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
review=N/'capture_review/FINAL_INDEPENDENT_REVIEW.json';d=json.loads(review.read_text())
assert d['review_passed'] is True and d['blocking_findings']==[]
for p,h in d['source_sha256'].items():assert sha(p)==h,p
assert d['checks']['bad_bindings']==[]
recon=N/'SAVED_CDR_RECONSTRUCTION_CPU_RECEIPT.json';r=json.loads(recon.read_text())
assert r['passed'] and r['fixture'] and not r['control_admission_changed']
for key in ['source_sha256','input_bindings']:
    for p,h in r[key].items():assert sha(p)==h,p
adm=dict(schema='R39_owner_schema_adapter_of_sealed_independent_review/v1',
    utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),review_passed=True,blocking_findings=[],
    role='root format adaptation only; underlying findings authored independently',
    sealed_independent_report=dict(path=str(review),sha256=sha(review)),
    original_report_unchanged=True,source_sha256={**d['source_sha256'],str(review):sha(review)},
    checks=[dict(name='sealed_independent_PASS_and_no_blockers',passed=True),
        dict(name='all_original_report_source_bindings_current',passed=True),
        dict(name='all_original_report_bad_bindings_empty',passed=True)],
    online_timing_or_real_fault_proven=False,navigation_PASS=False)
(N/'INDEPENDENT_REVIEW_ADMISSION.json').write_text(json.dumps(adm,ensure_ascii=False,indent=2)+'\n')
assert not (N/'ONE_SIMULATION_LATCH.json').exists()
gate=cp.freeze_source_review(C,[str(N/'FINITE_CPU_RECEIPT.json')],
    [str(N/'INDEPENDENT_REVIEW_ADMISSION.json'),str(N/'ROOT_RELEASE_RECONCILIATION.json')],root_review_complete=True)
extra=[N/x for x in ['AUTHORITY.json','P1_SAMPLING_CONTRACT.json','CANDIDATE_DIFF.patch',
    'create_isolated_candidate.py','ISOLATED_CANDIDATE_COPY.json','FINITE_CPU_OUTPUT.txt',
    'reconstruct_saved_association.py','SAVED_CDR_RECONSTRUCTION_CPU_RECEIPT.json',
    'STORAGE_SESSION_BEFORE_R39_ROOT.json','RESOURCE_GUARD_ATTEMPT1_FAILURE.json',
    'capture_review/P1_LOCAL_CANDIDATE_INDEPENDENT_STATIC_REVIEW.json',
    'capture_review/FINAL_INDEPENDENT_REVIEW.json','shadow_review/SHADOW_BUDGET_READONLY_REVIEW.json']]
extra += [N.parent/'R37_ROWS_EVENT_STAGE/GC_FINAL_CONTROLLED_RESULT.json',
          N.parent/'R37_ROWS_EVENT_STAGE/gc_cpu/CPU_EXECUTION_LEDGER.json',Path(__file__)]
extra += [Path(p).resolve() for p in r['input_bindings']]
gate['verified_inputs_sha256'].update({str(p):sha(p) for p in extra})
gate['diagnostic_scope']='One original association-collision P1 capture, 3 groups16MiB/120native s/900wall s/1GB; original admission unchanged; GPU resource guard currently blocked, no runtime release'
(C/cp.GATE_NAME).write_text(json.dumps(gate,indent=2)+'\n')
for p in (C/'profiles').glob('engineering_*.json'):cp.verify_preflight(C,json.loads(p.read_text()))
out=dict(schema='R39_fresh_source_freeze/v1',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    candidate=str(C),gate_sha256=sha(C/cp.GATE_NAME),bindings=len(gate['verified_inputs_sha256']),
    finite_cases=10,complete_CDR_NN_reconstruction_groups=3,reconstruction_is_synthetic_fault_fixture=True,
    sealed_independent_report_sha256=sha(review),old_R38_gate_preserved=True,
    source_changes_stopped=True,maximum_attempts=1,new_runtime_started=False,
    GPU_guard_passed=False,runtime_released=False,original_admission_unchanged=True,navigation_PASS=False)
with (N/'SOURCE_FREEZE.json').open('x') as f:f.write(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
