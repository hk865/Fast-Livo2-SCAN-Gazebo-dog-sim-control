"""One no-main actual import/spec-load check of the prepared slow-return path."""
import ast,hashlib,json,os
from pathlib import Path
import rclpy
ROOT=Path(__file__).resolve().parents[3];SIM=ROOT/'simulation'
RUN=SIM/'test_results/20261002_downhill_b_slow_return_v6';STAGE=RUN/'staging'
OUT=Path(__file__).resolve().parent
os.environ['DEMO_TEST_ROOT']=str(ROOT);os.environ['DEMO_RUN_DIR']=str(RUN)
called=[]
def forbidden(*a,**kw):
 called.append(True);raise RuntimeError('No ROS initialization allowed in import review')
rclpy.init=forbidden
# Exact archived entry prefix; exclude only the final call of original main.
def import_prefix(file):
 nodes=ast.parse(file.read_text()).body
 assert isinstance(nodes[-1],ast.Expr) and isinstance(nodes[-1].value,ast.Call) and isinstance(nodes[-1].value.func,ast.Attribute) and nodes[-1].value.func.attr=='main'
 ns={'__name__':'archive_source_review','__file__':str(file)}
 exec(compile(ast.Module(body=nodes[:-1],type_ignores=[]),str(file),'exec'),ns)
 return ns
ns=import_prefix(STAGE/'slow_adapter_runner.py')
runner=ns['runner'];core=ns['joint_stop_core']
# Same original role='adapter' root/filename/spec sequence without calling main.
root=Path(runner.__file__).resolve().parent
spec=runner.importlib.util.spec_from_file_location('timed_production_adapter',root/'joint_reference_adapter.py')
module=runner.importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
body=import_prefix(STAGE/'arming_node_runner.py')['module']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def cls(p,name):return ast.dump(next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.ClassDef) and n.name==name),include_attributes=False)
fixture=json.loads((RUN/'fixture_manifest.json').read_text())
previous=SIM/'test_results/20261002_downhill_b_enabled_v4'
base=json.loads((previous/'fixture_manifest.json').read_text())
old=(previous/'staging/simulation.launch.py').read_text()
new=(STAGE/'simulation.launch.py').read_text()
entry="str(out/'staging/slow_adapter_runner.py')"
checks={
 'fourteen_archive_sources_exact':len(fixture['executed_staging_sha256'])==14 and all(sha(STAGE/n)==h for n,h in fixture['executed_staging_sha256'].items()),
 'actual_physics_adapter_entry_single_change':old.count("str(root/'control_timing_runner.py')")==1 and new==old.replace("str(root/'control_timing_runner.py')",entry,1),
 'archived_wrapper_preloads_stage_joint_core':Path(core.__file__).resolve()==STAGE/'joint_stop_core.py' and sha(STAGE/'joint_stop_core.py')=='b8a84ddf74d1067918e3d168bc89cec40d649e80bc38ca05fc2416c809283dba',
 'original_timing_runner_actually_imported':Path(runner.__file__).resolve()==SIM/'control_timing_runner.py',
 'actual_spec_loaded_adapter_still_production':Path(module.__file__).resolve()==SIM/'joint_reference_adapter.py' and module.ROOT==SIM,
 'actual_StopReturn_binds_candidate_class':module.StopReturn is core.StopReturn,
 'actual_adapter_and_body_FK_bind_candidate_class':module.JointGeometry is core.JointGeometry and body.JointGeometry is core.JointGeometry,
 'actual_body_assets_SIM_still_production':body.SIM==SIM,
 'JointGeometry_AST_unchanged':cls(STAGE/'joint_stop_core.py','JointGeometry')==cls(SIM/'joint_stop_core.py','JointGeometry'),
 'original_driver_contract_exact_and_stage_unchanged':all(fixture['executed_staging_sha256'][n]==base['executed_staging_sha256'][n] for n in ('probe_downhill_regions.py','downhill_region_contract.py','stack.launch.py','feedback_core.py','body_stabilizer_node.py','arming_node_runner.py','arming_bridge_runner.py','control_bridge.py','execution_safety.py','bridge_with_feedback.py')),
 'production329_and_assets_match_v4':fixture['source_sha256']==base['source_sha256'] and len(fixture['source_sha256'])==329 and all(fixture[k]==base[k] for k in ('spawn','world_sha256','physical_model_sha256','profile_sha256','original_world_goal_centers')),
 'declared_only_common_candidates_plus_slow_return_enabled':all(fixture[k] is True for k in ('motion_arming_candidate','deferred_fk_candidate','bridge_motion_arming_candidate','slow_return_candidate','feedback_enabled')),
 'prepared_no_owned_execution_no_ROS_init':fixture['execute_pending'] and not (RUN/'owned_process.json').exists() and not called,
}
result=dict(passed=all(checks.values()),checks=checks,count=len(checks),scope='One no-main import/spec-load only; no ROS node, Trace construction, playback or physical run.',
 bindings={k:dict(path=str(Path(v.__file__).resolve()),sha256=sha(Path(v.__file__))) for k,v in [('joint_stop_core',core),('control_timing_runner',runner),('actual_adapter_spec',module),('body_node',body)]},
 same_input_contract=dict(count=2,dwell_sim_s=.4,gap_sim_s=.2,goal_timeout_sim_s=90,truth_max_bracket_s=.15),
 limitations=['Checks selected module/class and asset roots, not actual live publishers or runtime motion.','Slow-return duration choice is reviewed separately, not inferred physically stable here.','v5 entry was invalid and never executed; its archived bytes are unchanged.'])
(OUT/'prepared_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'passed':result['passed'],'checks':checks,'receipt_sha256':sha(OUT/'prepared_receipt.json')},indent=2))
