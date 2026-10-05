"""Necessary pure-source/geometry review of a fresh two-region F3 component."""
from pathlib import Path
import ast, hashlib, importlib.util, json, math, sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def node(fn,name):return next(n for n in ast.walk(ast.parse(fn.read_text())) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name==name)
def astsame(a,b,name):return ast.dump(node(a,name),include_attributes=False)==ast.dump(node(b,name),include_attributes=False)

def review(a,b):
    stage=ROOT/'simulation/test_results/downhill_two_regions_staging'
    ma,mb=[read(x/'fixture_manifest.json') for x in (a,b)]
    ra,rb=[read(x/'runtime_manifest.json') for x in (a,b)]
    frozen=read(ROOT/'test_results/full19_freeze/source_manifest.json')['sha256']
    full=read(ROOT/'runs/20261002_084329_14d9cc/runtime_manifest.json')
    ca=(a/'staging/simulation.launch.py').read_text();cb=(b/'staging/simulation.launch.py').read_text()
    expected=(ROOT/'simulation/simulation.launch.py').read_text().replace('root=Path(__file__).resolve().parent',"root=Path(os.environ['DEMO_TEST_ROOT'])/'simulation'")
    expected=expected.replace("'-x','0','-y','0','-z','.30'","'-x','0','-y','7','-z','2.70','-Y','3.141592653589793'")
    child_keys=['fast_livo2_ros','fast_livo2_core','scan_planner','controller_manager','controller_interface','control_toolbox','joint_trajectory_controller','joint_state_broadcaster','gz_ros2_control','gazebo_measured_joint_plugin']
    stack=a/'staging/stack.launch.py';orig=ROOT/'scripts/stack.launch.py'
    contract_path=a/'staging/downhill_region_contract.py'
    spec=importlib.util.spec_from_file_location('downhill_pure_review',contract_path);C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
    scenario=read(ROOT/'simulation/scenario.json');raw=scenario['route_goals']['return_origin'][:2]
    geometry=[]
    for yaw in (0.,.57,math.pi):
        origin=np.array([.023,-.014,.008]);rot=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
        goals=C.downhill_goals(scenario,origin,{'yaw_camera_init_from_world':yaw})
        center_ok=all(np.allclose(g.center,origin+rot@(np.asarray(x['center'])-[0.,7.,2.4]),rtol=0,atol=2e-15) for g,x in zip(goals,raw))
        axes_ok=all(np.allclose(g.definition()['arrival']['axes'],[rot@np.asarray(v) for v in x['arrival']['axes']],rtol=0,atol=2e-15) for g,x in zip(goals,raw))
        bounds_ok=all(g.definition()['arrival']['half_extents_m']==[.35,.30,.10] and g.definition()['arrival']['control_band']['half_extents_m']==[.25,.20,.07] and g.dwell_sim_s==.4 and g.timeout_sim_s==90 for g in goals)
        geometry.append(dict(sensor_heading_yaw=yaw,raw_origin=origin.tolist(),centers=[list(g.center) for g in goals],center_offset_exact=center_ok,axes_same_rotation=axes_ok,original_bounds_dwell_timeout=bounds_ok))
    tests=read(stage/'contract_result.json')
    files=['run.py','probe_downhill_regions.py','downhill_region_contract.py','stack.launch.py','simulation.launch.py']
    checks={
        'original329_source_manifest_exact_AB_Full19':ma['source_sha256']==mb['source_sha256']==frozen and len(frozen)==329,
        'current329_byte_SHA_frozen':all(sha(ROOT/n)==h for n,h in frozen.items()),
        'both5_executed_archived_sources_SHA_match':all(set(m['executed_staging_sha256'])==set(files) and all(sha(x/'staging'/n)==h for n,h in m['executed_staging_sha256'].items()) for x,m in [(a,ma),(b,mb)]),
        'current_reviewed3_excluded_sources_equal_archived':all(sha(stage/n)==ma['executed_staging_sha256'][n]==mb['executed_staging_sha256'][n] for n in ['run.py','probe_downhill_regions.py','downhill_region_contract.py']),
        'AB_shared4_excluded_files_same_bytes':all((a/'staging'/n).read_bytes()==(b/'staging'/n).read_bytes() for n in files if n!='simulation.launch.py'),
        'AB_physics_only_original_body_CLI_enable':cb==ca.replace("'--output',str(out/'body_feedback.jsonl')","'--enable','--output',str(out/'body_feedback.jsonl')"),
        'A_original_physics_only_declared_fresh_birth_and_ROOT_resolution':ca==expected,
        'AB_fresh_declared_spawn_before_sensors_same':ma['spawn']==mb['spawn']==dict(x=0,y=7,z=2.70,yaw=math.pi),
        'same_original_model_world_profile_SHA':all(ma[k]==mb[k]==sha(ROOT/n) for k,n in [('world_sha256','simulation/generated/three_floors.sdf'),('physical_model_sha256','simulation/generated/go2_measured.urdf'),('profile_sha256','simulation/config/ros_control.yaml')]),
        'original_standard_two_world_goal_centers':ma['original_world_goal_centers']==mb['original_world_goal_centers']==[x['center'] for x in raw]==[[2.,7.,2.4],[5.,7.,2.1]],
        'actual_geometry_offset_then_one_heading_exact_original_bounds':all(g['center_offset_exact'] and g['axes_same_rotation'] and g['original_bounds_dwell_timeout'] for g in geometry),
        'same_Full19_runtime_other_installed_artefact_SHAs':all(ra['artifacts'][k]['sha256']==rb['artifacts'][k]['sha256']==full['artifacts'][k]['sha256'] for k in child_keys),
        'same_qualified_two_selected_CHAMP_binary_SHAs':all(ra['selected_gait'][k]['sha256']==rb['selected_gait'][k]['sha256']==full['selected_gait'][k]['sha256'] for k in ['executable','library']),
        'both_actual_perrun_selected_binary_bytes':all(sha(Path(r['selected_gait'][k]['path']))==r['selected_gait'][k]['sha256'] for r in [ra,rb] for k in ['executable','library']),
        'same_36_selected_source_and_provenance_records':ra['selected_gait']['source_sha256']==rb['selected_gait']['source_sha256']==full['selected_gait']['source_sha256'],
        'same_baseline_CHAMP_separate_original':ra['baseline_gait_artifacts']==rb['baseline_gait_artifacts']==full['baseline_gait_artifacts'],
        'same_controller_adapter_trace_settings_and_runtime_required':all(ra[k]==rb[k]==full[k] for k in ['joint_stop_adapter','controller_timing_configuration','control_timing_diagnostics','full_control_runtime_required']),
        'actual_early4_original_callback_functions_unchanged':all(astsame(orig,stack,n) for n in ['after_ready_gate','after_parameter_gate','after_runtime_gate','after_gait_runtime_gate']),
        'whole_pair_same_stack_sensor_gate_NAV_driver':(a/'staging/stack.launch.py').read_bytes()==(b/'staging/stack.launch.py').read_bytes(),
        'body_feedback_mode_declared_A0_B1':ma['feedback_enabled'] is False and mb['feedback_enabled'] is True,
        'root_current_tests_source_matched':tests['sources_sha256']=={n:sha(stage/n) for n in ['probe_downhill_regions.py','downhill_region_contract.py']},
        'only10_necessary_method_geometry_contracts_passed':len(tests['checks'])==10 and tests['passed'] is True and all(tests['checks'].values()),
        'no_existing_execution_in_prepared_AB':not any((x/'owned_process.json').exists() for x in (a,b)),
    }
    runsrc=(stage/'run.py').read_text();driver=(a/'staging/probe_downhill_regions.py').read_text()
    checks['driver_accepts_both_original_mode0_and_mode1']="if mode not in ('0','1'):" in driver
    checks['driver_actual_required_health_mode_and_caps_failclosed']="self.bridge.get('body_feedback_required') is True" in driver and "feedback['enabled']==self.expected_feedback_enabled" in driver and 'forward_speed_m_s=.12' in driver
    checks['driver_original_hold_fail_component_thresholds_and_canonical2']="max_tilt<.30 and holds==0" in driver and 'imu_hold_rad=.30,imu_fail_rad=.50' in driver and 'index!=2' in driver
    checks['recording_and_cleanup_original_budgets']="'--capacity','3600000','--payload-limit','2147483648'" in ca and 'grace=55.,terminate_timeout=5.,kill_timeout=5.' in runsrc
    checks['runner_capture_owned_Gz_before_signal_and_verify_onlyaftercleanup']=runsrc.index('capture_before_stop(out,proc.pid);proc.send_signal(signal.SIGINT)')<runsrc.index('finish_owned_process(proc,grace=55.')<runsrc.index('runtime=verify_after_stop')
    report=dict(scope='Prepared fresh F3 canonical return0/1 A0/B1 only. No ROS/physics/actual status or outcome claim.',
        checks=checks,ready=all(checks.values()),check_count=len(checks),geometry_proof=geometry,
        known_birth_surface_world=[0,7,2.4],fresh_model_body_spawn_world=[0,7,2.70],
        original_contract={'inner_half_extents_m':[.25,.20,.07],'outer_half_extents_m':[.35,.30,.10],
            'dwell_sim_s':.4,'raw_gap_sim_s':.2,'per_goal_timeout_sim_s':90,'GT_bracket_max_sim_s':.15,
            'imu_hold_rad':.30,'imu_fail_rad':.50,'body_feedback_gains_unchanged':True},
        fixture_manifests={str(x):sha(x/'fixture_manifest.json') for x in (a,b)},
        executed_source_SHA256={'A':ma['executed_staging_sha256'],'B':mb['executed_staging_sha256']},
        pure_receipt_sha256=sha(stage/'contract_result.json'),
        limits=['Fresh F3 birth is before sensors; this is neither Full19 continuation nor a runtime robot teleport.',
            'Known birth surface subtracts only the declared scene frame. Heading comes from10 actual SLAM/IMU pairs; GT is confined to final evaluation.',
            'Actual per-run64RPC, Gz5DSO and CHAMP mappings, static gate, input pair/terrain freshness, canonical2 original windows, stability and owned cleanup remain future live obligations.',
            'Original active feedback gains/30ms5ms100ms/contact/300ms gates are unchanged; preparation does not claim feedback stabilizes downhill.'])
    dest=ROOT/'slam/test_results/downhill_prepared_independent.json';dest.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(report=str(dest),sha256=sha(dest),ready=report['ready'],count=len(checks),false=[k for k,v in checks.items() if not v]),ensure_ascii=False))
    return report

if __name__=='__main__':review(*(Path(a).resolve() for a in sys.argv[1:3]))
