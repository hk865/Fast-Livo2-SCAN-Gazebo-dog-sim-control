#!/usr/bin/env python3
"""Offline source/three-selector launch construction freeze; no LaunchService."""
from pathlib import Path
import ast,datetime,hashlib,importlib.util,json,os,sys
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))
from scoped_profile import verify_scope,prepare
from dynamic.dynamic_obstacle import verify_scope as verify_dynamic
from dynamic.prepare import runtime_files,validate_asset
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
previous=ROOT/'test_results/navigation_v42_cleanup_freeze.json';old=json.loads(previous.read_text())
assert sha(previous)=='c3acec04da84ec1ec35d317db8235cef8e0de3692e0c84c10a9e154a4b75c819'
assert sha(ROOT/'scripts/run_test.py')=='b497b278f1631106e70301d0796522319c982781b39f3ca85d481a4abfd4932b'
for name in ('policy/worker.py','policy/observation.py','policy/contract.json','simulation/prepare.py',
             'simulation/build/libteacher_actuator.so','navigation/runtime_io.py','navigation/bridge.py',
             'navigation/dynamic/protocol.json','navigation/dynamic/flat_dynamic_profile.json',
             'navigation/flat_relative_roundtrip.json','runs/acceptance.json'):
    assert sha(ROOT/name)==old['source_hashes'][name],name
assert NAV/'guard_audit.py'in runtime_files()
assert NAV/'dynamic/flat_dynamic_profile_turn20.json'in runtime_files()
assert NAV/'dynamic/guard_trace_schema.json'in runtime_files()

spec=importlib.util.spec_from_file_location('v43_launch',NAV/'stack.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
selectors=[]
for label,rid,experiment,cap in [
    ('ordinary_flat','20261004_navigation_v43_offline_flat','flat_relative_roundtrip_v1',.12),
    ('preserved_dynamic12','20261004_navigation_v43_offline_dynamic12','finite_flat_dynamic_stop_resume_v1',.12),
    ('prospective_dynamic20','20261004_navigation_v43_offline_dynamic20','finite_flat_dynamic_stop_resume_turn20_v1',.2)]:
    run=ROOT/'runs'/rid;os.environ.update(ROS_DOMAIN_ID='79',GZ_PARTITION='teacher_'+rid,DEMO_RUN_DIR=str(run))
    result=verify_scope(run/'navigation_scope.json');assert result['experiment']==experiment
    assert result['profile']['max_yaw_rate_radps']==cap
    if label!='ordinary_flat':validate_asset(run);verify_dynamic(run,run/'dynamic_protocol.json')
    context=LaunchContext();context.launch_configurations['run_dir']=str(run)
    launch=module.generate_launch_description();action=next(a for a in launch.entities if isinstance(a,OpaqueFunction))
    constructed=action.execute(context)
    assert len(constructed)==(11 if label=='ordinary_flat'else 19)
    # Node actions are descriptions; execute no Node/LaunchService here.
    nodes=[a for a in constructed if isinstance(a,Node)]
    assert len(nodes)==(1 if label=='ordinary_flat'else 2)
    selectors.append({'selector':label,'experiment':experiment,'max_yaw_rate_radps':cap,
        'run_dir':str(run),'scope_sha256':result['sha256'],'constructed_actions':len(constructed),
        'dynamic_scope_sha256':None if label=='ordinary_flat'else sha(run/'dynamic_scope.json'),
        'prepared_only':True,'starts_ROS':False,'starts_Gazebo':False})
reject=False
try:prepare(ROOT/'runs/does_not_exist_turn20_reject',dynamic_turn20=True)
except RuntimeError as e:reject='explicit dynamic experiment only'in str(e)
assert reject

checks={}
for name,filename in [('targeted_turn_guard','offline_checks.json'),('nav_boundary','nav_boundary_checks.json'),
                      ('controller_methods','controller_method_checks.json'),('dynamic_fixture','dynamic_checks.json')]:
    path=Path(__file__).with_name(filename);data=json.loads(path.read_text());assert data['status']=='passed'
    assert all(c['passed']for c in data['checks'].values())
    checks[name]={'path':str(path),'sha256':sha(path),'status':'passed','count':len(data['checks'])}
calibration=ROOT/'runs/20261004_094018_turn_positive_yaw02_navigation_calibration_r1_789a'
actual=json.loads((calibration/'summary.json').read_text());runtime=json.loads((calibration/'runtime_manifest.json').read_text())
assert actual['levels']['interface']=='passed'and actual['levels']['motion']=='passed'
assert all(p['returncode']==0 for p in runtime['owned_processes'])and runtime['error']is None
files=sorted(NAV.glob('*.py'))+runtime_files()+[NAV/'flat_relative_roundtrip.json',ROOT/'policy/worker.py',
    ROOT/'policy/observation.py',ROOT/'policy/contract.json',ROOT/'simulation/build/libteacher_actuator.so',
    ROOT/'simulation/prepare.py',ROOT/'scripts/run_test.py',ROOT/'runs/acceptance.json']
hashes={str(p.relative_to(ROOT)):sha(p)for p in files}
diff={name:{'before':value,'after':hashes.get(name)}for name,value in old['source_hashes'].items()
      if name in hashes and value!=hashes[name]}
new={name:value for name,value in hashes.items()if name not in old['source_hashes']}
receipt={'schema':1,'version':'navigation_v4.3_dynamic_turn20_exact_native_guard_trace',
    'status':'offline_ready_actual_dynamic_unverified','frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'previous_v42_freeze':{'path':str(previous),'sha256':sha(previous),'original_bytes_preserved':True},
    'offline_receipts':checks,'inherited_atomic_transport_checks':old['inherited_91_offline_checks']['atomic_transport'],
    'inherited_direct_service_bridge_checks':old['targeted_cleanup_offline'],
    'selector_and_launch_checks':selectors,'turn20_without_dynamic_rejected':True,
    'runtime_source_differences':diff,'new_runtime_sources':new,'source_hashes':hashes,
    'actual_positive_02_calibration':{'run_id':calibration.name,'summary_sha256':sha(calibration/'summary.json'),
        'runtime_manifest_sha256':sha(calibration/'runtime_manifest.json'),'interface':'passed','motion':'passed',
        'all_owned_exit_zero':True,'command_radps':.2,
        'actual_wz_mean_radps':actual['tests'][0]['metrics']['tracking'][0]['measured_mean'][2],
        'parking':actual['tests'][0]['metrics']['stop_teacher'],
        'scope':'One actual18s CPU-Teacher calibration; no dynamic-navigation pass inferred'},
    'unchanged':{'ordinary_flat_heading_cap_radps':.12,'old_dynamic_heading_cap_radps':.12,
        'goal_timeout_sim_s':90.,'control_arrival_radius_m':.17,'wall_and_sim_command_TTL_s':.3,
        'native_steering_geometry':True,'actual_registered_cloud_clear_sim_s':1.,'clear_gap_max_s':.3,
        'checkpoint_actor_247_mapping_PD_physics_step':True,'original_protocol_and_failure_receipts':True},
    'actual_ROS_started':False,'Gazebo_started':False,'LaunchService_started':False,
    'guard_schema_sha256':hashes['navigation/dynamic/guard_trace_schema.json'],
    'global_acceptance_preserved':json.loads((ROOT/'runs/acceptance.json').read_text())['levels'],
    'next_required':'Prospective candidate3 exact guard replay and actual240sim dynamic obstacle/parking/resume/2-region validation; preserve old6614 cleanup failure and10c7 route/clear evidence failure'}
out=ROOT/'test_results/navigation_v43_dynamic_turn20_exact_guard_freeze.json'
if out.exists():raise RuntimeError('Do not replace an existing freeze')
out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({'status':receipt['status'],'freeze':str(out),'sha256':sha(out),'checks':checks,
    'source_differences':list(diff),'new_sources':list(new),'selectors':selectors},indent=2))
