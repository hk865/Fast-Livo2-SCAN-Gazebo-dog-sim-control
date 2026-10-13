"""Root-only freeze of concrete R80 reviews; no simulation or inherited PASS."""
from pathlib import Path
import hashlib,json,sys
S=Path(__file__).resolve().parent;copy=json.loads((S/'COPY_RECEIPT.json').read_text())
C=Path(copy['candidate']);sys.dont_write_bytecode=True;sys.path[:0]=[str(S),str(C),str(C.parents[1])]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,data):
 p=S/name
 if p.exists():raise RuntimeError('Refuse to overwrite frozen '+name)
 p.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');return p
review_names=['INDEPENDENT_PARKING_REVIEW.json','INDEPENDENT_INPUT_EVIDENCE_REVIEW.json','INDEPENDENT_OWNER_BUDGET_REVIEW.json']
for name in review_names:
 d=json.loads((S/name).read_text())
 if not d.get('review_passed')or d.get('blocking_findings')!=[]:raise RuntimeError('Review incomplete '+name)
 for p,h in d['source_sha256'].items():
  if sha(p)!=h:raise RuntimeError('Review binding changed '+p)
business=json.loads((S/'BUSINESS_FLOW_CHECKS.json').read_text());compat=json.loads((S/'MISSION_COMPATIBILITY.json').read_text())
if not business['passed']or business['tests_run']!=3 or compat['returncode']!=0:raise RuntimeError('Fresh workflows/compatibility failed')
from full46_launch_contract import PROFILE_NAMES,validate_full46
for n in PROFILE_NAMES:validate_full46(json.loads((C/'profiles'/n).read_text()),C)
cpu_sources=dict(business['source_SHA256'])
for old_path,h in compat['source_sha256'].items():
 new_p=C/Path(old_path).name
 if sha(new_p)!=h:raise RuntimeError('Unchanged mission/test bytes differ '+str(new_p))
 cpu_sources[str(new_p)]=h
dependency=json.loads((S/'DEPENDENCY_CPU_RECEIPT.json').read_text())
if not dependency['tests_pass']:raise RuntimeError('Dependency finite check incomplete')
cpu_sources.update(dependency['source_sha256'])
stats=json.loads((S/'STATISTICS_CPU_RECEIPT.json').read_text())
if not stats['tests_pass']:raise RuntimeError('Current statistics replay incomplete')
cpu_sources.update(stats['source_sha256'])
for name in review_names:cpu_sources.update(json.loads((S/name).read_text())['source_sha256'])
for p in (S/'business_flow_checks.py',S/'storage_guard.py',S/'retained_usage.py',S/'run.py',
          C/'storage_guard.py',C/'full46_launch_contract.py',C/'policy_selection.py',C/'POLICY_SELECTION_CONTRACT.json'):
 cpu_sources[str(p)]=sha(p)
for p,h in cpu_sources.items():
 if sha(p)!=h:raise RuntimeError('Fresh checked source changed '+p)
save('FRESH_CPU_RECEIPT.json',dict(schema='R33_startup_finite_CPU_receipt/v1',tests_pass=True,
 failures=0,actual_candidate=str(C),fresh_business_workflows=3,fresh_legacy_mission_unittest_methods=0,business_workflows_verified=3,
 reused_unchanged_business_workflows=0,reused_unchanged_mission_tests=76,
 actual_new_unique_unittest_methods=0,current_statistics_replay_and_purpose_business_checks=True,source_sha256=cpu_sources,
 evidence_bindings={str(S/n):sha(S/n)for n in ('BUSINESS_FLOW_CHECKS.json','MISSION_COMPATIBILITY.json',
  'MISSION_COMPATIBILITY.log','OWNER_BUDGET_LOAD_CHECK.json','STATISTICS_CPU_RECEIPT.json',
  'RETAINED_OBSERVER_PARITY.json','PURE_REGRESSION_COMPARISON.json')},
 legacy_core_tests=dict(total=17,passed=15,failed=2,identical_original_C79_failures=True,
  evidence=str(S/'PURE_REGRESSION_COMPARISON.json'),test_files_unchanged=True,
  included_in_new_workflow_PASS=False),
 finite_profiles_validated=list(PROFILE_NAMES),simulation=False,Actor_loaded=False,
 historical_navigation_PASS_inherited=False,actual_runtime_verified=False,
 limitations=['Current production-chain three parking workflows executed; current1s statistical old-source replay and15 purpose cases independently checked; no physical efficacy',
  'Original76 mission tests and dependency helper results reused by exact checked source hashes; no broad rerun; two unchanged core baseline failures preserved',
  'Native/SDK build reused exact prior inputs; no fresh compilation or inference',
  'Shared current-retained capacity and conservative positive-growth round charge use distinct approved metrics']))
baseline=copy['baseline']
save('BASELINE.json',dict(baseline=baseline,candidate=str(C),baseline_files=copy['baseline_files'],historical_navigation_PASS_inherited=False))
excluded={'C51_REFACTOR_CONTRACT.json','R33_STARTUP_PREFLIGHT.json','R33_STARTUP_SOURCE_REVIEW.json'}
sources={str(p.relative_to(C)):sha(p)for p in C.rglob('*')if p.is_file()and'__pycache__'not in p.parts
    and'.git'not in p.parts and str(p.relative_to(C))not in excluded}
changed=[n for n,h in sources.items()if h!=copy['baseline_files'].get(n)]
save('CANDIDATE_MANIFEST.json',dict(schema='R80_declared_staged_parking_source_manifest/v1',baseline=baseline,
 candidate=str(C),sources=sources,excluded_gate_files=sorted(excluded),declared_changed_files=changed,
 original_route_geometry_and_numerical_quality_source_safety_thresholds_retained=True,
 approved_T0_revision='ordinary_zero_ack_fixed5',historical_runtime_PASS_inherited=False))
runtime_names=('ALGORITHM_CHANGE_RECEIPT.json','STATISTICS_CPU_RECEIPT.json','DEPENDENCY_CPU_RECEIPT.json','run.py','freeze_gate.py','storage_guard.py','retained_usage.py','diagnostic_execute.py',
 'stop_owned_traced.py','probe_stop.py','diagnostic_stop_policy.py','owned_guardian.py',
 'guardian_archive_shutdown.py','worker_archive_shutdown.py','archive_close_policy.py','pipeline_lifecycle.py',
 'storage_timing.py','capture_owner_stop.py','capture_candidate.py','terminal_original_decision.py',
 'ray_observer.cpp','libgo2_ray_observer.so','SDK_BINDINGS.json','AUTHORITY.json','BUDGET_BINDING.json',
 'PRE_COPY_RETAINED_BASELINE.json','RETAINED_OBSERVER_PARITY.json','OWNER_BUDGET_LOAD_CHECK.json',
 'FRESH_CPU_RECEIPT.json','BASELINE.json','CANDIDATE_MANIFEST.json')
stage_inputs={str(S/n):sha(S/n)for n in runtime_names}
bindings={**{str(C/n):h for n,h in sources.items()},**stage_inputs}
save('RUNTIME_BINDINGS.json',dict(schema='R80_static_runtime_bindings/v1',bindings=bindings,
 mutable_ledgers_and_progress_excluded=True,candidate=str(C)))
contract=json.loads((C/'C51_REFACTOR_CONTRACT.json').read_text())
contract.update(schema='R80_explicit_staged_parking_identity_handoff_admission/v1',candidate_root=str(C),
 bounded_stage=str(S),allowed_owner_entries={str(S/n):sha(S/n)for n in ('run.py','freeze_gate.py')},
 candidate_manifest=str(S/'CANDIDATE_MANIFEST.json'),candidate_manifest_sha256=sha(S/'CANDIDATE_MANIFEST.json'),
 baseline_manifest=str(S/'BASELINE.json'),baseline_manifest_sha256=sha(S/'BASELINE.json'),
 stage_input_sha256=stage_inputs,fresh_CPU_receipt=str(S/'FRESH_CPU_RECEIPT.json'),
 fresh_CPU_receipt_sha256=sha(S/'FRESH_CPU_RECEIPT.json'),approved_new_parking_contract='ordinary_zero_ack_fixed5')
(C/'C51_REFACTOR_CONTRACT.json').write_text(json.dumps(contract,indent=2)+'\n')
from refactor_preflight import verify_source_equivalence
verify_source_equivalence(C)
from corridor_preflight import freeze_source_review
gate=freeze_source_review(C,[S/'FRESH_CPU_RECEIPT.json'],[S/n for n in review_names],root_review_complete=True)
save('FREEZE_GATE_RECEIPT.json',dict(passed=True,source_inputs=len(gate['verified_inputs_sha256']),
 candidate=str(C),declared_changed_files=changed,simulation=False,actual_runtime_verified=False,
 gate_sha256=sha(C/'R33_STARTUP_PREFLIGHT.json'),root_is_only_writer=True))
print(json.dumps(dict(frozen_gate_inputs=len(gate['verified_inputs_sha256']),declared_changes=changed,simulation=False)))
