#!/usr/bin/env python3
"""Observer-only source freeze + direct launch construction; no LaunchService."""
from pathlib import Path
import datetime,hashlib,importlib.util,json,os,sys
from launch import LaunchContext
from launch.actions import OpaqueFunction
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation';sys.path.insert(0,str(NAV))
from scoped_profile import verify_scope
from dynamic.dynamic_obstacle import verify_scope as verify_dynamic
from dynamic.prepare import runtime_files,validate_asset
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
previous=ROOT/'test_results/navigation_v44_measured_dwell_actual_guard_freeze.json';old=json.loads(previous.read_text())
assert sha(previous)=='714e75459d5a6087cd147498c955013f2d7d7a16899669b1f1046bd04681314f'
changed={'navigation/dynamic/dynamic_obstacle.py','navigation/dynamic/prepare.py'}
for name,value in old['source_hashes'].items():
 if name not in changed:assert sha(ROOT/name)==value,name
assert NAV/'dynamic/observer_preroll.py'in runtime_files()and NAV/'dynamic/observer_preroll_schema.json'in runtime_files()
run=ROOT/'runs/20261004_navigation_v45_offline_dwell'
os.environ.update(ROS_DOMAIN_ID='79',GZ_PARTITION='teacher_'+run.name,DEMO_RUN_DIR=str(run))
scope=verify_scope(run/'navigation_scope.json');validate_asset(run);verify_dynamic(run,run/'dynamic_protocol.json')
assert scope['experiment']=='finite_flat_dynamic_stop_resume_dwell_v1'and scope['profile']['arrival_stop_policy']=='after_measured_dwell'
spec=importlib.util.spec_from_file_location('v45_launch',NAV/'stack.launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
context=LaunchContext();context.launch_configurations['run_dir']=str(run)
launch=module.generate_launch_description();opaque=next(x for x in launch.entities if isinstance(x,OpaqueFunction))
actions=opaque.execute(context);assert len(actions)==19
checkfile=Path(__file__).with_name('offline_checks.json');checks=json.loads(checkfile.read_text())
assert checks['status']=='passed'and all(c['passed']for c in checks['checks'].values())
files=sorted(NAV.glob('*.py'))+runtime_files()+[NAV/'flat_relative_roundtrip.json',ROOT/'policy/worker.py',
 ROOT/'policy/observation.py',ROOT/'policy/contract.json',ROOT/'simulation/build/libteacher_actuator.so',
 ROOT/'simulation/prepare.py',ROOT/'scripts/run_test.py',ROOT/'runs/acceptance.json']
hashes={str(p.relative_to(ROOT)):sha(p)for p in files}
diff={n:{'before':v,'after':hashes[n]}for n,v in old['source_hashes'].items()if n in hashes and v!=hashes[n]}
assert set(diff)==changed
receipt={'schema':1,'version':'navigation_v4.5_passive_registered_cloud_preroll_only',
 'status':'offline_ready_actual_evidence_unverified','frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'previous_v44_freeze':{'path':str(previous),'sha256':sha(previous),'original_bytes_preserved':True},
 'source_hashes':hashes,'runtime_source_differences':diff,
 'new_runtime_sources':{n:v for n,v in hashes.items()if n not in old['source_hashes']},
 'inherited_V44_controller_and_144_checks':{'scope':'Control and all original profiles/requirements byte-identical; no control-check re-run inferred',
  'receipts':old['offline_receipts']},
 'observer_targeted_checks':{'path':str(checkfile),'sha256':sha(checkfile),'status':'passed','checks':len(checks['checks'])},
 'prepared_dwell_scope':{'run_dir':str(run),'scope_sha256':scope['sha256'],'dynamic_scope_sha256':sha(run/'dynamic_scope.json'),
  'constructed_actions':len(actions),'LaunchService_started':False,'prepared_only':True},
 'observer_preroll_schema_sha256':sha(NAV/'dynamic/observer_preroll_schema.json'),
 'observer_patch':'Only actual registered-cloud waiting1s/32frame/16MiB decodedXYZ cache+entry passive flush+source-preserving metadata and explicit manifesterrors; original activephase windows unchanged',
 'controls_unchanged':{'V44_controller_profile_heading_guard_TTL_arrival_parking':True,'mover_Trigger_Program_service_physics':True,
  'checkpoint_actor_247_PD_jointmap':True,'runner_CLI':True,'navigation_no_truth':True},
 'historical_1536_preserved':{'run_id':'20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536',
  'strict_MAIN':'failed','missing_actual_prefix':'guardseq29/30 use originalcloudheader13.199999999 before entering13.24; oldobserver did not save thoseXYZ',
  'other_physical_NAV_items':'passed according to independent candidate4; not overallMAIN pass',
  'original_data_or_receipts_modified':False,'retroactive_fill_permitted':False},
 'global_acceptance_preserved':json.loads((ROOT/'runs/acceptance.json').read_text())['levels'],
 'ROS_started':False,'Gazebo_started':False,'LaunchService_started':False,
 'next_required':'Freeze independent candidate5 then actual240sim sameV44 --navigation-dynamic-dwell; complete originalXYZ coverage plus all unchanged strict physical/navigation/cleanup requirements'}
out=ROOT/'test_results/navigation_v45_observer_preroll_freeze.json'
if out.exists():raise RuntimeError('Refusing to replace any existing freeze')
out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({'status':receipt['status'],'freeze':str(out),'sha256':sha(out),'observer_checks':len(checks['checks']),
 'source_differences':diff,'new_sources':receipt['new_runtime_sources'],'scope_sha256':scope['sha256'],'launch_actions':len(actions)},indent=2))
