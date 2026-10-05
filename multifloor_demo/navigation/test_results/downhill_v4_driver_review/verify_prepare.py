"""Small read-only archive and actual-import review. Never call main or ROS init."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT/'simulation'
OUT = Path(__file__).resolve().parent
PAIR = [SIM/'test_results'/('20261002_downhill_'+s+'_v4') for s in ('a_disabled','b_enabled')]
EXPECTED = {
 'probe_downhill_regions.py':'7c3ce31842bceff257c576592af8aa5aa8e3745d08dbe2b47c1b68bfed9db543',
 'downhill_region_contract.py':'3841d4e50817f518e92156d223ac978923f41fd252c5f9863560e64189d227e2',
 'control_bridge.py':'dbbd33efdf48a15f3d0c7e0ba5ff2ae56d9dd3dad2a236b7209d7c363dacf94c',
 'execution_safety.py':'d04b8b29f0c5f3d220ca2bd5dbab076222cafb26613abf45efde8415b481de73',
 'bridge_with_feedback.py':'570c1d11c16f890624a5677116def0951d57b3602399c895944f5b8e52721817',
 'feedback_core.py':'49a61b38b2d74d041d92affbe88dc947c3fb6a9be949605b2f3d3b0f593b412f',
}
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def tree(path):return ast.parse(path.read_text())
def dump(nodes):return [ast.dump(n,include_attributes=False) for n in nodes]

# Fresh child isolates module caches for each archive, with actual installed imports.
IMPORT = r'''
import ast,hashlib,json,os,sys
from pathlib import Path
import rclpy
called=[]
def forbidden(*a,**kw):
 called.append('rclpy.init');raise RuntimeError('ROS initialization forbidden in source review')
rclpy.init=forbidden
stage=Path(sys.argv[1]);wrapper=stage/'arming_bridge_runner.py'
nodes=ast.parse(wrapper.read_text()).body
prefix=[]
for n in nodes:
 if isinstance(n,ast.If):break
 prefix.append(n)
ns={'__file__':str(wrapper),'__name__':'archive_import_review'}
exec(compile(ast.Module(body=prefix,type_ignores=[]),str(wrapper),'exec'),ns)
import bridge_with_feedback,control_bridge,execution_safety,control_safety,control_timing_trace
modules={m.__name__: {'path':str(Path(m.__file__).resolve()),'sha256':hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()} for m in (bridge_with_feedback,control_bridge,execution_safety,control_safety,control_timing_trace)}
print(json.dumps({'modules':modules,'ros_init_calls':called,'feedback_base_is_actual_staged_bridge':bridge_with_feedback.FeedbackBridge.__bases__==(control_bridge.Bridge,), 'bridge_health_is_actual_staged_class':control_bridge.JointAdapterHealth is execution_safety.JointAdapterHealth,'no_main_called':True}))
'''
imports=[]
for run in PAIR:
 env=os.environ.copy();env.update(DEMO_TEST_ROOT=str(ROOT),DEMO_RUN_DIR=str(run),PYTHONDONTWRITEBYTECODE='1')
 proc=subprocess.run([sys.executable,'-B','-c',IMPORT,str(run/'staging')],env=env,check=True,capture_output=True,text=True)
 imports.append(json.loads(proc.stdout))

man=[json.loads((run/'fixture_manifest.json').read_text()) for run in PAIR]
source=[json.loads((run/'source_manifest.json').read_text()) for run in PAIR]
checks={}
checks['twelve_archived_sources_match_each_declared_manifest']=all(
 set(m['executed_staging_sha256'])=={f.name for f in (r/'staging').iterdir() if f.is_file()} and
 all(sha(r/'staging'/name)==value for name,value in m['executed_staging_sha256'].items()) for r,m in zip(PAIR,man))
checks['driver_contract_and_bridge_three_match_reviewed_bytes']=all(
 all(sha(r/'staging'/name)==value for name,value in EXPECTED.items()) for r in PAIR)
checks['actual_stage_import_bindings_both']=all(
 all(Path(d['modules'][name]['path'])==r/'staging'/f'{name}.py' for name in ('bridge_with_feedback','control_bridge','execution_safety'))
 for r,d in zip(PAIR,imports))
checks['original_imu_gate_and_timing_observer_imported_both']=all(
 all(Path(d['modules'][name]['path'])==SIM/f'{name}.py' for name in ('control_safety','control_timing_trace')) for d in imports)
checks['actual_class_identity_and_no_ros_init']=all(d['feedback_base_is_actual_staged_bridge'] and d['bridge_health_is_actual_staged_class'] and not d['ros_init_calls'] and d['no_main_called'] for d in imports)
checks['same329_declared_production_map']=source[0]['sha256']==source[1]['sha256']==man[0]['source_sha256']==man[1]['source_sha256'] and len(source[0]['sha256'])==329
checks['no_prepared_execution_started']=all(m['execute_pending'] and not (r/'owned_process.json').exists() for r,m in zip(PAIR,man))
checks['both_same_three_candidates_declared']=all(all(m[k] is True for k in ('motion_arming_candidate','deferred_fk_candidate','bridge_motion_arming_candidate')) for m in man)
checks['only_enable_changes_pair_execution_bytes']=all(
 (PAIR[0]/'staging'/name).read_bytes()==(PAIR[1]/'staging'/name).read_bytes()
 for name in man[0]['executed_staging_sha256'] if name!='simulation.launch.py') and (
 (PAIR[1]/'staging/simulation.launch.py').read_text().replace("        '--enable','--output'","        '--output'")==(PAIR[0]/'staging/simulation.launch.py').read_text())
checks['pair_actual_feedback_mode_0_1']=man[0]['feedback_enabled'] is False and man[1]['feedback_enabled'] is True
checks['original_spawn_and_world_centers']=all(m['spawn']==dict(x=0,y=7,z=2.7,yaw=3.141592653589793) and m['original_world_goal_centers']==[[2,7,2.4],[5,7,2.1]] for m in man)
checks['same_profile_physical_model_world']=all(man[0][key]==man[1][key] for key in ('world','world_sha256','physical_model_sha256','profile_sha256'))

# Exact observer attach/save statements copied from the production feedback wrapper.
reference=next(n for n in tree(SIM/'feedback_timing_runner.py').body if isinstance(n,ast.FunctionDef) and n.name=='main')
expected=dump(reference.body[1:])
checks['original_trace_attach_and_save_ast_exact']=all(
 dump(tree(r/'staging/arming_bridge_runner.py').body[-5:])==expected for r in PAIR)
checks['both_bridge_entry_uses_archived_wrapper']=all("str(out/'staging/arming_bridge_runner.py')" in (r/'staging/simulation.launch.py').read_text() for r in PAIR)
checks['both_driver_entry_uses_archived_sibling']=all("str(Path(__file__).parent/'probe_downhill_regions.py')" in (r/'staging/stack.launch.py').read_text() for r in PAIR)
checks['explicit_scenario_and_align_enable_unchanged']=all("scenario_path:='+str(root/'simulation/scenario.json')" in (r/'staging/stack.launch.py').read_text() and "'DEMO_TEST_ALIGN_TRANSLATION_ENABLED':'1'" in (r/'staging/stack.launch.py').read_text() for r in PAIR)
checks['bridge_22_method_contract_source_matches']=json.loads((SIM/'test_results/bridge_motion_arming_staging/contract_result.json').read_text())['sources']=={n:EXPECTED[n] for n in ('control_bridge.py','execution_safety.py','bridge_with_feedback.py')}
checks['no_shared_source_or_runtime_modified_by_review']=all(sha(Path(v['baseline_path']))==v['baseline_sha256'] for v in json.loads((SIM/'test_results/bridge_motion_arming_staging/source_receipt.json').read_text()).values())
receipt=dict(passed=all(checks.values()),checks=checks,count=len(checks),actual_import_bindings=imports,
 scope='Read-only source/archive/import review only, no main, ROS initialization, physical execution or acceptance inferred.',
 sources=[{f.name:sha(f) for f in (r/'staging').iterdir() if f.is_file()} for r in PAIR],
 original_contract={'goal_count':2,'dwell_sim_s':.4,'max_raw_gap_s':.2,'timeout_sim_s':90,'truth_max_bracket_s':.15,
 'outer_half_extents_m':[.35,.30,.10],'inner_half_extents_m':[.25,.20,.07]},
 candidate_scope='Common lifecycle/FK/bridge candidate in both A and B; A/B differs only body feedback enable. Not equivalent to v3 baseline.',
 limitations=['Imported actual installed ROS packages without initializing ROS or instantiating nodes.','Future physical runtime/receive/arming order still needs actual evidence.','No checks of applied joint force or claim startup scheduling causes fixed.'])
(OUT/'prepared_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({'passed':receipt['passed'],'count':len(checks),'failed':[k for k,v in checks.items() if not v],'receipt_sha256':sha(OUT/'prepared_receipt.json')},indent=2))
