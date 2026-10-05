"""Minimal no-main v9 binding/only-enable preparation comparison against v8."""
import ast,hashlib,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];SIM=ROOT/'simulation'
A=SIM/'test_results/20261002_downhill_disabled_envelope_v8'
B=SIM/'test_results/20261002_downhill_enabled_envelope_v9';STAGE=B/'staging'
OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
fa,fb=read(A/'fixture_manifest.json'),read(B/'fixture_manifest.json')
ra,rb=read(A/'runtime_manifest.json'),read(B/'runtime_manifest.json')
os.environ.update(DEMO_TEST_ROOT=str(ROOT),DEMO_RUN_DIR=str(B))
import rclpy
called=[]
def forbidden(*args,**kw):called.append(True);raise RuntimeError('No ROS initialization')
rclpy.init=forbidden
def prefix(path,stop):
    kept=[]
    for n in ast.parse(path.read_text()).body:
        if stop(n):break
        kept.append(n)
    ns={'__name__':'actual_archive_review','__file__':str(path)}
    exec(compile(ast.Module(body=kept,type_ignores=[]),str(path),'exec'),ns)
    return ns['module']
body=prefix(STAGE/'arming_node_runner.py',lambda n:isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=='main')
bridge=prefix(STAGE/'arming_bridge_runner.py',lambda n:isinstance(n,ast.If))
import control_bridge,execution_safety,control_safety,joint_stop_core,gait_runtime
physics=(STAGE/'simulation.launch.py').read_text();old=(A/'staging/simulation.launch.py').read_text()
selected=gait_runtime.selection(rb)
common=['spawn','world_sha256','physical_model_sha256','profile_sha256','original_world_goal_centers','motion_arming_candidate','deferred_fk_candidate','bridge_motion_arming_candidate','slow_return_candidate','damping_d15_candidate','start_envelope_candidate']
checks={
 'only_feedback_enable_flag_changes':fa['feedback_enabled'] is False and fb['feedback_enabled'] is True and all(fa[k]==fb[k] for k in common),
 'only_enable_execution_source_difference':physics==old.replace("'--output',str(out/'body_feedback.jsonl')","'--enable','--output',str(out/'body_feedback.jsonl')") and old.count("'--output',str(out/'body_feedback.jsonl')")==1 and all(fa['executed_staging_sha256'][n]==h for n,h in fb['executed_staging_sha256'].items() if n!='simulation.launch.py'),
 'all_twelve_archives_exact':len(fb['executed_staging_sha256'])==12 and all(sha(STAGE/n)==h for n,h in fb['executed_staging_sha256'].items()),
 'same_frozen_329_production_actual':fa['source_sha256']==fb['source_sha256'] and len(fb['source_sha256'])==329 and all(sha(ROOT/n)==h for n,h in fb['source_sha256'].items()),
 'same_envelope_36_source_profile_and_provenance':ra['selected_gait']['source_sha256']==selected['source_sha256'] and len(selected['source_sha256'])==36 and all(sha(Path(selected['source_root'])/n)==h for n,h in selected['source_sha256'].items()) and all(selected[n]==ra['selected_gait'][n] and sha(selected[n]['path'])==selected[n]['sha256'] for n in ('frozen_build_receipt','frozen_source_archive_receipt','frozen_patch','deployment_manifest')),
 'actual_perrun_selected_node_DSO_same_bytes':all(selected[k]['sha256']==ra['selected_gait'][k]['sha256'] and Path(selected[k]['path']).is_relative_to(B/'gait_envelope') and selected[k]==rb['artifacts'][a] for k,a in [('executable','champ_base'),('library','champ_base_controller')]),
 'actual_body_stage_and_root_assets':Path(body.__file__).resolve()==STAGE/'body_stabilizer_node.py' and body.SIM==SIM,
 'actual_bridge_stage_three_and_base_identity':Path(bridge.__file__).resolve()==STAGE/'bridge_with_feedback.py' and Path(control_bridge.__file__).resolve()==STAGE/'control_bridge.py' and Path(execution_safety.__file__).resolve()==STAGE/'execution_safety.py' and bridge.FeedbackBridge.__bases__==(control_bridge.Bridge,),
 'original_safety_StopReturn_and_adapter_entry':Path(control_safety.__file__).resolve()==SIM/'control_safety.py' and Path(joint_stop_core.__file__).resolve()==SIM/'joint_stop_core.py' and "str(root/'control_timing_runner.py'),'adapter'" in physics,
 'same_original_canonical_contract':all(fa['executed_staging_sha256'][n]==fb['executed_staging_sha256'][n] for n in ('probe_downhill_regions.py','downhill_region_contract.py','stack.launch.py')),
 'prepared_no_ROS_init_or_execution':not called and fb['execute_pending'] is True and not (B/'owned_process.json').exists() and not (B/'preparation_invalid.json').exists(),
}
result=dict(passed=all(checks.values()),checks=checks,count=len(checks),scope=__doc__,
 bindings={m.__name__:str(Path(m.__file__).resolve()) for m in (body,bridge,control_bridge,execution_safety,control_safety,joint_stop_core,gait_runtime)},
 selected_artifacts={k:selected[k] for k in ('executable','library')},
 manifest_sha256={n:sha(B/n) for n in ('fixture_manifest.json','runtime_manifest.json')},
 limitations=['No main, node, physical execution, live enable acknowledgment or motion inferred.','v8 original FAIL unchanged; v9 is prepared enabled comparison only.'])
(OUT/'prepared_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],count=len(checks),failed=[k for k,v in checks.items() if not v],sha256=sha(OUT/'prepared_receipt.json')),indent=2))
