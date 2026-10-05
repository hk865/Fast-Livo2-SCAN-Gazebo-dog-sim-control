#!/usr/bin/env python3
"""Freeze prospective dwell/actual-clear-guard variant; no LaunchService."""
from pathlib import Path
import datetime,hashlib,importlib.util,json,os,sys
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))
from scoped_profile import verify_scope,prepare
from dynamic.dynamic_obstacle import verify_scope as verify_dynamic
from dynamic.prepare import runtime_files,validate_asset
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
previous=ROOT/'test_results/navigation_v43_dynamic_turn20_exact_guard_freeze.json';old=json.loads(previous.read_text())
assert sha(previous)=='c75a38a4c7ce39362628be67d5aebc3772e52452286ca3bfa8aaf5dd7610077b'
assert sha(ROOT/'scripts/run_test.py')=='50f9695afac236a7b05ca58cf98911a9333bc0fbfdf15261f4c218a10ef1de79'
unchanged=('policy/worker.py','policy/observation.py','policy/contract.json','simulation/prepare.py',
 'simulation/build/libteacher_actuator.so','navigation/runtime_io.py','navigation/bridge.py','navigation/teacher_transition.py',
 'navigation/guard_audit.py','navigation/stack.launch.py','navigation/dynamic/dynamic_obstacle.py',
 'navigation/dynamic/protocol.json','navigation/dynamic/flat_dynamic_profile.json','navigation/dynamic/flat_dynamic_profile_turn20.json',
 'navigation/dynamic/guard_trace_schema.json','navigation/flat_relative_roundtrip.json','runs/acceptance.json')
for name in unchanged:assert sha(ROOT/name)==old['source_hashes'][name],name
assert NAV/'dynamic/flat_dynamic_profile_dwell.json'in runtime_files()
assert NAV/'dynamic/guard_continuity_schema.json'in runtime_files()
spec=importlib.util.spec_from_file_location('v44_launch',NAV/'stack.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
selectors=[]
for label,rid,experiment,cap in [
 ('ordinary_flat','20261004_navigation_v44_offline_flat','flat_relative_roundtrip_v1',.12),
 ('original_dynamic12','20261004_navigation_v44_offline_dynamic12','finite_flat_dynamic_stop_resume_v1',.12),
 ('original_dynamic20','20261004_navigation_v44_offline_dynamic20','finite_flat_dynamic_stop_resume_turn20_v1',.2),
 ('prospective_dwell','20261004_navigation_v44_offline_dwell','finite_flat_dynamic_stop_resume_dwell_v1',.2)]:
 run=ROOT/'runs'/rid;os.environ.update(ROS_DOMAIN_ID='79',GZ_PARTITION='teacher_'+rid,DEMO_RUN_DIR=str(run))
 result=verify_scope(run/'navigation_scope.json');assert result['experiment']==experiment and result['profile']['max_yaw_rate_radps']==cap
 if label!='ordinary_flat':validate_asset(run);verify_dynamic(run,run/'dynamic_protocol.json')
 if label=='prospective_dwell':assert result['profile']['arrival_stop_policy']=='after_measured_dwell'and result['profile']['clear_guard_gap_max_sim_s']==.3
 else:assert 'arrival_stop_policy'not in result['profile']and 'clear_guard_gap_max_sim_s'not in result['profile']
 context=LaunchContext();context.launch_configurations['run_dir']=str(run)
 launch=module.generate_launch_description();action=next(a for a in launch.entities if isinstance(a,OpaqueFunction))
 constructed=action.execute(context);assert len(constructed)==(11 if label=='ordinary_flat'else 19)
 assert len([a for a in constructed if isinstance(a,Node)])==(1 if label=='ordinary_flat'else 2)
 selectors.append({'selector':label,'experiment':experiment,'max_yaw_rate_radps':cap,'run_dir':str(run),
  'scope_sha256':result['sha256'],'constructed_actions':len(constructed),
  'dynamic_scope_sha256':None if label=='ordinary_flat'else sha(run/'dynamic_scope.json'),
  'prepared_only':True,'ROS_started':False,'Gazebo_started':False})
for kwargs in ({'dynamic_dwell':True},{'dynamic':True,'dynamic_dwell':True},{'slam_only':True,'dynamic':True,'dynamic_turn20':True,'dynamic_dwell':True}):
 rejected=False
 try:prepare(ROOT/'runs/offline_reject_dwell',**kwargs)
 except RuntimeError as e:rejected='requires explicit dynamic Turn20 navigation'in str(e)
 assert rejected,kwargs
checks={}
for label,fn in [('arrival_controller','offline_checks.json'),('scheduled_clear_guard','continuity_checks.json'),
 ('old_gate_native_guard_regression','v43_regression_checks.json'),('nav_boundary','nav_boundary_checks.json'),
 ('original_controller_protection','controller_method_checks.json'),('physical_fixture','dynamic_checks.json')]:
 p=Path(__file__).with_name(fn);d=json.loads(p.read_text());assert d['status']=='passed'and all(c['passed']for c in d['checks'].values())
 checks[label]={'path':str(p),'sha256':sha(p),'checks':len(d['checks']),'status':'passed'}
files=sorted(NAV.glob('*.py'))+runtime_files()+[NAV/'flat_relative_roundtrip.json',ROOT/'policy/worker.py',
 ROOT/'policy/observation.py',ROOT/'policy/contract.json',ROOT/'simulation/build/libteacher_actuator.so',
 ROOT/'simulation/prepare.py',ROOT/'scripts/run_test.py',ROOT/'runs/acceptance.json']
hashes={str(p.relative_to(ROOT)):sha(p)for p in files}
diff={n:{'before':v,'after':hashes[n]}for n,v in old['source_hashes'].items()if n in hashes and v!=hashes[n]}
assert set(diff)=={'navigation/controller.py','navigation/scoped_profile.py','navigation/dynamic/prepare.py','scripts/run_test.py'}
shared=[ROOT.parent/'navigation'/n for n in ('controller.py','control_core.py','goal_regions.py','trajectory_contract.py')]
receipt={'schema':1,'version':'navigation_v4.4_measured_dwell_and_continuous_actual_clear_guard',
 'status':'offline_ready_actual_dynamic_unverified','frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'previous_v43_freeze':{'path':str(previous),'sha256':sha(previous),'original_bytes_preserved':True},
 'source_hashes':hashes,'runtime_source_differences':diff,
 'new_runtime_sources':{n:v for n,v in hashes.items()if n not in old['source_hashes']},
 'unchanged_shared_actual_geometry_arrival_sources':{str(p):sha(p)for p in shared},
 'offline_receipts':checks,'offline_checks_rerun':sum(x['checks']for x in checks.values()),
 'inherited_atomic_transport_checks':old['inherited_atomic_transport_checks'],
 'inherited_direct_service_bridge_checks':old['inherited_direct_service_bridge_checks'],
 'selector_launch_checks':selectors,'invalid_unbounded_dwell_selector_rejected':True,
 'prospective_changes':{'arrival_stop_policy':'Only newprofile returns inside and arrived, arrived from original measured_region_arrival; continue original checkedSCAN until original measured dwell passes',
  'actual_guard_continuity':'Reset original clear timer on actual ROS guard gap>0.3s or stale actual source headers; no cached tick release, wait for original scheduled actual geometry; duplicate/backward actual guard clocks fail closed'},
 'trace':{'actual_guard_filename':'navigation_guard_history.jsonl','original_schema_sha256':sha(NAV/'dynamic/guard_trace_schema.json'),
  'continuity_history_filename':'navigation_guard_continuity_history.jsonl','continuity_schema_sha256':sha(NAV/'dynamic/guard_continuity_schema.json'),
  'release_fields':['compute_ros_clock_ns','previous_actual_guard_clock_ns','clear_start_before_s','clear_elapsed_before_s','obstacle_hold_before','obstacle_hold_after','obstacle_resumes_after','clear_release_requires_this_callback_actual_guard'],
  'continuity_rows_are_not_geometry_checks':True},
 'unchanged_requirements':{'raw_SLAM_control_radius_m':.17,'arrival_dwell_sim_s':.6,'arrival_max_pose_gap_s':.2,
  'goal_timeout_sim_s':90.,'pose_cloud_header_and_command_TTL_s':.3,'actual_clear_guard_span_sim_s':1.,'actual_clear_guard_gap_max_sim_s':.3,
  'physical_box_block_sim_s':10.,'physical_parking_sim_s':5.,'parking_thresholds_changed':False,'regions_required':2,
  'original_actor_247_mapping_PD_physics':True,'old_flat_and_dynamic_profiles':True},
 'historical_v43_failure_preserved':{'run_id':'20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70',
  'outbound90s_failed':True,'region_count':0,'historical_receipts_modified':False,
  'other_failed_checks':'Original parking0.033278>0.03 and cached release/SCAN26.69 preceding strict actual clear27.69 remain independent failures'},
 'global_acceptance_preserved':json.loads((ROOT/'runs/acceptance.json').read_text())['levels'],
 'ROS_started':False,'Gazebo_started':False,'LaunchService_started':False,
 'next_required':'Freeze independent candidate4 before actual new240sim run; full dynamic span/parking/clear/newSCAN/resume +2rawSLAM regions/deadlines/finalstop/allowned0/allstartedchildren clean required'}
out=ROOT/'test_results/navigation_v44_measured_dwell_actual_guard_freeze.json'
if out.exists():raise RuntimeError('Do not replace any existing freeze')
out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({'status':receipt['status'],'freeze':str(out),'sha256':sha(out),'checks':receipt['offline_checks_rerun'],
 'source_differences':list(diff),'trace':receipt['trace'],'selectors':selectors},indent=2))
