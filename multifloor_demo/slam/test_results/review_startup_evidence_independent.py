"""Readonly preparation-only early-RPC/maps startup sequencing review. No ROS."""
from pathlib import Path
import ast,hashlib,json,importlib.util,tempfile,os
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2];S=ROOT/'simulation/test_results/startup_evidence_staging';OLD=ROOT/'simulation/test_results/classic_pd_first4_staging';RUN=ROOT/'simulation/test_results/20261002_classic_pd_first4_candidate';OUT=Path(__file__).with_name('oct2_startup_evidence_template_independent_READY.json')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
ready=json.loads((S/'ready_receipt.json').read_text());pure=json.loads((S/'pure_startup_order_result.json').read_text());ros=json.loads((S/'ros78_gate_v1/result.json').read_text());freeze=json.loads((RUN/'source_manifest.json').read_text())['sha256'];checks={}
checks['current_all18_execute_sources_match_ready']=len(ready['files'])==18 and all(sha(S/n)==v for n,v in ready['files'].items())
checks['pure36_contract_checks_true']=len(pure['checks'])==36 and all(pure['checks'].values())
checks['actual_ROS78_three_cases25_checks_true']=len(ros['rows'])==3 and sum(len(r['checks']) for r in ros['rows'])==25 and all(all(r['checks'].values()) for r in ros['rows'])
checks['actual_ROS_source_matches_current']=all(sha(S/n)==v for n,v in ros['source_sha256'].items())
checks['all_six_owned_ROS_processes_no_longer_live']=all(not Path('/proc',str(r[k])).exists() for r in ros['rows'] for k in ('server_pid','gate_pid'))
checks['all269_production_sources_unchanged']=len(freeze)==269 and all(sha(ROOT/n)==v for n,v in freeze.items())
checks['original_static_gate_byteidentical']=sha(ROOT/'scripts/wait_sensors.py')==freeze['scripts/wait_sensors.py']
unchanged=['nav_drift_controller.py','turn_drift.py','body_stabilizer_node.py','feedback_core.py','bridge_with_feedback.py','timed_feedback_bridge.py','record_actuator_suffix.py','probe_first_four_regions.py','first_four_region_contract.py','prepare_joint_sensor.py','prepare_control_profile.py','control_parameter_gate.py']
checks['all12_control_measurement_region_gate_assets_byteidentical']=all((S/n).read_bytes()==(OLD/n).read_bytes() for n in unchanged)
# Execute the exact callbacks only in a mocked event environment, without launch/ROS.
source=ast.parse((S/'stack.launch.py').read_text());env={'root':ROOT,'include':lambda p:('include',str(p)),'gate':'static','parameter_gate':'parameter','runtime_gate':'runtime','navigator':'NAV','Shutdown':lambda **kw:('shutdown',kw['reason']),'EmitEvent':lambda **kw:kw['event']}
callbacks={n.name:n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name.startswith('after_')}
for n in callbacks.values():exec(compile(ast.fix_missing_locations(ast.Module(body=[n],type_ignores=[])),'<actual staged callback>','exec'),env)
checks['exit0_exact_sequence_rpc_maps_static_slam_nav']=all(env[n](SimpleNamespace(returncode=0),None)==v for n,v in [('after_ready_gate',['parameter']),('after_parameter_gate',['runtime']),('after_runtime_gate',['static']),('after_gate',[('include',str(ROOT/'slam/launch.py')),('include',str(ROOT/'navigation/scan.launch.py')),'NAV'])])
checks['failure_every_boundary_shutdown_no_next_stage']=all(env[n](SimpleNamespace(returncode=rc),None)[0][0]=='shutdown' for n in callbacks for rc in (1,-2))
main=next(n for n in ast.parse((S/'run.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main');block=next(i for i,n in enumerate(main.body) if isinstance(n,ast.If) and ast.unparse(n.test)=='a.execute');firstenv=next(i for i,n in enumerate(main.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='env' for t in n.targets))
checks['execute_rejected_unconditionally_before_environment_spawn']=isinstance(main.body[block].body[0],ast.Raise) and block<firstenv
checks['shell_entrypoints_guarded_template']=('startup_evidence_staging/run.py' in (S/'run.sh').read_text()) and 'classic_pd_first4_staging/run.py' not in (S/'run.sh').read_text()
run=(S/'run.py').read_text();checks['cleanup_same55s_5s_5s']= 'finish_owned_process(proc,grace=55,terminate_timeout=5,kill_timeout=5)' in run
cap=(S/'capture_control_runtime.py').read_text();checks['runtime_captures_owned_group_and_exact_five_library_identity']=all(t in cap for t in ("os.getpgid(pid)!=group","len(candidates)!=1","actual[key]==wanted","stage='after actual RPC; before static gate'","not_a_terminal_maps_receipt=True"))
checks['no_new_control_publisher_in_early_gates']=all('create_publisher' not in (S/n).read_text() for n in ('await_controller_ready.py','capture_control_runtime.py','control_parameter_gate.py'))
checks['startup_maps_separate_from_original_terminal_receipt']= "startup_control_runtime.json" in (S/'stack.launch.py').read_text() and "actual_gz_process_prestop_paths.json" in run
assert all(checks.values()),{k:v for k,v in checks.items() if not v}
result={'scope':__doc__,'verdict':'READY for preparation-only evidence sequence; not a physical execution authorization','checks':checks,'all_checks_true':all(checks.values()),'source_sha256':ready['files'],'upstream_receipt_SHA256':sha(S/'ready_receipt.json'),'sequence':ready['sequence'],'original_static_gate_SHA256':sha(ROOT/'scripts/wait_sensors.py'),'actual_ROS78_parameter_gate_cases':ros['rows'],'limitations':ready['boundaries']+['The actual 125s spawner wait, bounded RPC and original150s static timeout are unchanged declared sequencing; no actual Gazebo capture has run for this preparation-only template.','Receipt tests establish transparent gate behavior; a later actual profile still requires its own live parameter reply and owned-process map/path/SHA capture.','This review makes no new gain adoption, no successful stationary initialization claim, and does not repair original classic startup FAIL.']}
OUT.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'checks':len(checks),'all_true':all(checks.values()),'path':str(OUT),'sha256':sha(OUT)}))
