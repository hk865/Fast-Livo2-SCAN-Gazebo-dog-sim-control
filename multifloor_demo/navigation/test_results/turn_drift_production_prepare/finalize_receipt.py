"""Finalize excluded preparation provenance, never alter collected sources."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
DEMO=HERE.parents[2]
OLD=HERE.parent/'turn_drift_staging'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def functions(p):
    t=ast.parse(p.read_text())
    return {n.name:ast.dump(n) for n in t.body if isinstance(n,(ast.ClassDef,ast.FunctionDef,ast.AsyncFunctionDef))}

prior=HERE/'preparation.json'
if not (HERE/'preparation_before_v2_scope.json').exists():
    (HERE/'preparation_before_v2_scope.json').write_bytes(prior.read_bytes())
original={'nav_drift_controller.py':sha(OLD/'nav_drift_controller.py'),'turn_drift.py':sha(OLD/'turn_drift.py')}
expected={'nav_drift_controller.py':'f592194d63a3fe2689da8cb0508f43ef45f310ff07c585aa422d03740a546b57',
          'turn_drift.py':'101912f6db6e5bc3133a10a9b8bd4f862b171d972bd65af645f0aa2212a00d1e'}
frozen=json.loads((DEMO/'test_results/full18_freeze/source_manifest.json').read_text())['sha256']
changed=[name for name,h in frozen.items() if sha(DEMO/name)!=h]
names=('drift_controller.py','turn_drift.py','stack.launch.py','guard_configuration.json',
       'test_prepared.py','test_legacy_scope.py','test_archive_import.py',
       'equivalence_contract_result.json','legacy_scope_result.json','archive_import_result.json')
results={name:json.loads((HERE/name).read_text()) for name in
         ('equivalence_contract_result.json','legacy_scope_result.json','archive_import_result.json')}
cfg=json.loads((HERE/'guard_configuration.json').read_text())
cfg_matches=all(cfg['source_sha256'][n]==sha(HERE/n) for n in ('drift_controller.py','turn_drift.py'))
helper_same=functions(OLD/'turn_drift.py')==functions(HERE/'turn_drift.py')
diff=''.join(difflib.unified_diff((OLD/'nav_drift_controller.py').read_text().splitlines(True),
    (HERE/'drift_controller.py').read_text().splitlines(True),
    fromfile='frozen_f592/nav_drift_controller.py',tofile='prepared/navigation/drift_controller.py'))
(HERE/'controller_preparation.patch').write_text(diff)
receipt={
 'schema_version':2,
 'scope':'Excluded future standard entry only. Not production-adopted or physically validated in this relocated form.',
 'original_candidate_sha256':original,
 'prepared_sha256':{n:sha(HERE/n) for n in names},
 'original_candidate_bytes_unchanged':original==expected,
 'helper_all_class_and_function_AST_identical':helper_same,
 'controller_functional_method_diff':['NavigationDrift.drift_context'],
 'controller_method_diff_contract':'Exactly one additional nonlegacy predicate; removing it reproduces frozen method AST. All other method/class ASTs match (actual test).',
 'request_scope':'schema2_nonlegacy_goals',
 'legacy_behavior':'No drift supervisor arm; original base legacy control/arrival/deadline behavior. Frozen original legacy failure negative preserved.',
 'v2_behavior':'Actual-method replay snapshots match f592 for stop, fresh native idle ACK, new reference, checked path, original heading gate, wrong identity, protected and deadline branches.',
 'runtime_declaration_source_hash_matches':cfg_matches,
 'runtime_integration_boundary':'guard_configuration.json is a reviewed declaration, not a new live configuration input. Root must select/hash final entry/helper/declaration when adopting.',
 'tests':{'selected_profile_and_integration':results['equivalence_contract_result.json'],
          'legacy_scope_and_v2_equivalence':results['legacy_scope_result.json'],
          'deep_standard_archive_import':results['archive_import_result.json']},
 'collected_source_count':len(frozen),'collected_sources_changed':changed,
 'production_modified':False,'runtime_rebuilt':False,'ROS_nodes_started':False,'physics_started':False,
 'pending':'Parent adoption decision and physical uncertainty (first8 safety/arrival and dynamic interaction) remain; no PASS of complete demo.'}
receipt['ready_for_parent_review']=all(r['passed'] for r in results.values()) and not changed and helper_same and cfg_matches and original==expected
(HERE/'preparation.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({'ready_for_parent_review':receipt['ready_for_parent_review'],
      'source_sha256':{n:sha(HERE/n) for n in ('drift_controller.py','turn_drift.py','guard_configuration.json')},
      'receipt_sha256':sha(HERE/'preparation.json'),'collected_source_changes':len(changed)}))
raise SystemExit(not receipt['ready_for_parent_review'])
