"""Read-only v8 preparation check. No main, ROS initialization or execution."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
SIM=ROOT/'simulation'
RUN=SIM/'test_results/20261002_downhill_disabled_envelope_v8'
STAGE=RUN/'staging'
OUT=Path(__file__).resolve().parent
os.environ.update(DEMO_TEST_ROOT=str(ROOT),DEMO_RUN_DIR=str(RUN))
import rclpy
calls=[]
def forbidden(*args,**kwargs):
    calls.append('rclpy.init')
    raise RuntimeError('No ROS initialization permitted')
rclpy.init=forbidden
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def prefix(path,stop):
    body=ast.parse(path.read_text()).body
    kept=[]
    for node in body:
        if stop(node): break
        kept.append(node)
    ns={'__name__':'archive_import_review','__file__':str(path)}
    exec(compile(ast.Module(body=kept,type_ignores=[]),str(path),'exec'),ns)
    return ns
bridge=prefix(STAGE/'arming_bridge_runner.py',lambda n:isinstance(n,ast.If))['module']
body=prefix(STAGE/'arming_node_runner.py',lambda n:isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=='main')['module']
import control_bridge,execution_safety,control_safety,control_timing_trace,joint_stop_core
sys.path.insert(0,str(SIM))
import gait_runtime
fixture=read(RUN/'fixture_manifest.json'); runtime=read(RUN/'runtime_manifest.json')
selected=gait_runtime.selection(runtime)
sel=fixture['start_envelope_selection']
source=read(Path(sel['candidate_selection_path']).parent/'source_manifest.json')
build=read(selected['frozen_build_receipt']['path'])
physics=(STAGE/'simulation.launch.py').read_text()
stack=(STAGE/'stack.launch.py').read_text()
prior=SIM/'test_results/20261002_downhill_a_disabled_v4'
previous=read(prior/'fixture_manifest.json')
shared_names=set(fixture['executed_staging_sha256'])-{'run.py'}
expected={'probe_downhill_regions.py':'7c3ce31842bceff257c576592af8aa5aa8e3745d08dbe2b47c1b68bfed9db543',
          'downhill_region_contract.py':'3841d4e50817f518e92156d223ac978923f41fd252c5f9863560e64189d227e2'}
checks={
 'prepared_not_executed':fixture['execute_pending'] is True and not (RUN/'owned_process.json').exists() and not (RUN/'preparation_invalid.json').exists(),
 'only_requested_flags':fixture['feedback_enabled'] is False and all(fixture[k] is True for k in ('motion_arming_candidate','deferred_fk_candidate','bridge_motion_arming_candidate','start_envelope_candidate')) and fixture['slow_return_candidate'] is False and fixture['damping_d15_candidate'] is False,
 'twelve_archives_exact':len(fixture['executed_staging_sha256'])==12 and all(sha(STAGE/n)==h for n,h in fixture['executed_staging_sha256'].items()),
 'canonical_driver_and_contract_exact':all(sha(STAGE/n)==h for n,h in expected.items()),
 'common_v4_disabled_execution_sources_exact':all(fixture['executed_staging_sha256'][n]==previous['executed_staging_sha256'][n] for n in shared_names),
 'production329_frozen_declared_and_actual':len(fixture['source_sha256'])==329 and fixture['source_sha256']==read(ROOT/'test_results/full19_freeze/source_manifest.json')['sha256'] and all(sha(ROOT/n)==h for n,h in fixture['source_sha256'].items()),
 'unchanged_world_model_profile_actual':all(sha(fixture[p])==fixture[h]==previous[h] for p,h in [('world','world_sha256'),('physical_model_path','physical_model_sha256'),('profile_path','profile_sha256')]),
 'birth_and_canonical_centers':fixture['spawn']==dict(x=0,y=7,z=2.7,yaw=3.141592653589793) and fixture['original_world_goal_centers']==[[2,7,2.4],[5,7,2.1]],
 'selected_node_and_DSO_actual_perrun_copies':all(selected[k]==sel[k]==runtime['artifacts'][a] and sha(sel[k]['path'])==sel[k]['sha256'] and Path(sel[k]['path']).is_relative_to(RUN/'gait_envelope') for k,a in [('executable','champ_base'),('library','champ_base_controller')]),
 'candidate_selection_file_exact':sha(sel['candidate_selection_path'])==sel['candidate_selection_sha256'],
 'thirty_six_sources_and_only_leg_change':len(selected['source_sha256'])==36 and selected['source_sha256']==source['candidate_sha256']==sel['source_sha256'] and source['changed_files']==['champ/include/champ/leg_controller/leg_controller.h'] and all(sha(Path(selected['source_root'])/n)==h for n,h in selected['source_sha256'].items()),
 'build_and_four_provenance_receipts_exact':build['passed'] is True and all(sha(selected[n]['path'])==selected[n]['sha256'] for n in ('frozen_build_receipt','frozen_source_archive_receipt','frozen_patch','deployment_manifest')),
 'actual_factory_selects_perrun_gait_and_child_only_LD':"selected=selection(json.loads((out/'runtime_manifest.json').read_text()))" in physics and "gait=Node(executable=selected['executable']['path'],additional_env=child_environment(selected,os.environ)" in physics and gait_runtime.child_environment(selected,{'LD_LIBRARY_PATH':'original'})=={'LD_LIBRARY_PATH':str(Path(selected['library']['path']).parent)+':original'},
 'Gazebo_CM_selection_remains_independent':"additional_env=gazebo_controller_environment()" in physics and "gait_envelope" not in physics,
 'actual_bridge_imports_stage_three_and_original_safety':Path(bridge.__file__).resolve()==STAGE/'bridge_with_feedback.py' and Path(control_bridge.__file__).resolve()==STAGE/'control_bridge.py' and Path(execution_safety.__file__).resolve()==STAGE/'execution_safety.py' and bridge.FeedbackBridge.__bases__==(control_bridge.Bridge,) and control_bridge.JointAdapterHealth is execution_safety.JointAdapterHealth and Path(control_safety.__file__).resolve()==SIM/'control_safety.py' and Path(control_timing_trace.__file__).resolve()==SIM/'control_timing_trace.py',
 'actual_body_deferred_FK_import_and_root_assets':Path(body.__file__).resolve()==STAGE/'body_stabilizer_node.py' and body.SIM==SIM,
 'actual_adapter_original_entry_and_stop_core':"str(root/'control_timing_runner.py'),'adapter'" in physics and 'slow_adapter_runner.py' not in physics and Path(joint_stop_core.__file__).resolve()==SIM/'joint_stop_core.py',
 'startup_actual_RPC_gait_and_sensor_static_gate_preserved':all(x in stack for x in ('control_parameter_gate.py','gait_runtime.py','scripts/wait_sensors.py')) and "str(root/'simulation/config/ros_control.yaml')" in stack,
 'no_main_no_ROS_init_no_execution':not calls,
}
result=dict(passed=all(checks.values()),checks=checks,count=len(checks),run=str(RUN),scope=__doc__,
    actual_imports={m.__name__:str(Path(m.__file__).resolve()) for m in (bridge,body,control_bridge,execution_safety,control_safety,joint_stop_core,gait_runtime)},
    selected_artifacts={k:selected[k] for k in ('executable','library')},
    source_sha256={'fixture_manifest.json':sha(RUN/'fixture_manifest.json'),'runtime_manifest.json':sha(RUN/'runtime_manifest.json'),'run.py':sha(STAGE/'run.py')},
    original_contract=dict(goal_count=2,dwell_sim_s=.4,max_raw_gap_s=.2,timeout_sim_s=90,truth_max_bracket_s=.15,outer_half_extents_m=[.35,.30,.10],inner_half_extents_m=[.25,.20,.07]),
    limitations=['Source/import readiness only: actual process executable/maps and command-response require the future run.','D1 and ordinary StopReturn minimum .30 retained; no slow-return or D1.5 profile selected.','Envelope reduces startup offsets but retained phase-boundary jumps prevent a global C0 claim.'])
(OUT/'prepared_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],count=len(checks),failed=[k for k,v in checks.items() if not v],receipt_sha256=sha(OUT/'prepared_receipt.json'),selected_artifacts=result['selected_artifacts']),indent=2))
