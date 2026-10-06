"""Only offline file preparation and static launch/runner inspection; no ROS init."""
import ast, hashlib, importlib.util, json, shutil, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
HERE=ROOT/'navigation/ramp_registration'
OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('registration_producer',HERE/'producer_profile.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
results=[]
def check(name,condition):
    if not condition:raise AssertionError(name)
    results.append({'name':name,'passed':True})
def rejects(name,fn):
    try:fn()
    except (RuntimeError,ValueError,KeyError):check(name,True)
    else:check(name,False)
base=ROOT/'runs/20261004_145454_stand_ramp12_registration_stand_r1_9b70'
run=ROOT/'runs/offline_ramp_registration_producer_20261004'
run.mkdir(exist_ok=False)
for name in ('world.sdf','asset_manifest.json','sensor_contract.json','independent_protocol.json','terrain_target_manifest.json'):
    shutil.copy2(base/name,run/name)
tspec=importlib.util.spec_from_file_location('transport_prepare',ROOT/'tests/cloud_transport_probe_20261004/prepare_transport.py')
t=importlib.util.module_from_spec(tspec);tspec.loader.exec_module(t)
t.prepare(run,'shm_64m')
protected=[HERE/'freeze.json',*(HERE/name for name in p.load(HERE/'freeze.json')['source_hashes']),
           ROOT.parent/'camera_mode/slam/camera.yaml',ROOT.parent/'camera_mode/slam/fastlivo.yaml']
before=p.references(protected)
result=p.prepare(run)
check('actual_file_prepare_and_verify',result['starts_ros'] is False and result['navigation_started'] is False)
contract=p.load(run/'navigation_slam_contract.json')
check('no_navigation_eligibility_or_scope',contract['navigation_eligibility'] is False and not (run/'navigation_scope.json').exists())
check('no_navigation_command_or_route',not any((run/n).exists() for n in ('navigation_command.json','navigation_request.json','navigation_profile.json')))
check('fixed_zero90_lower12_hashes',contract['command_protocol']['sha256']==p.ZERO_SHA and contract['privileged_actor_only']['provider_sha256']==p.PROVIDER_SHA)
check('initialization_not_geometry_input',contract['initialization_only']['used_for_registration_geometry_or_heading'] is False)
check('privileged_provider_not_SLAM_input',contract['privileged_actor_only']['used_for_SLAM_or_registration_geometry'] is False)
check('contract_mandatory_sources_all_current',all(p.sha(f)==h for f,h in contract['references'].items()))
check('all_three_configs_written',all((run/n).is_file() for n in p.NAMES))
fc=p.load(run/p.NAMES[0])['/**']['ros__parameters']
cc=p.load(run/p.NAMES[1])['/**']['ros__parameters']
check('actual_three_sensor_topics',fc['common']['img_topic']=='/demo/teacher/slam/image' and fc['common']['lid_topic']=='/demo/slam/lidar_filtered' and fc['common']['imu_topic']=='/demo/teacher/slam/imu')
check('actual_600_IMU_frames_and_span',fc['imu']['imu_int_frame']==600 and fc['imu']['init_min_span']==2.9)
check('actual_extrinsics',fc['extrin_calib']['extrinsic_T']==[.2,0,.1177] and fc['extrin_calib']['extrinsic_R']==[1.,0,0,0,1.,0,0,0,1.])
check('actual_camera_optical_extrinsics',fc['extrin_calib']['Rcl']==[0.,-1.,0.,0.,0.,-1.,1.,0.,0.] and all(abs(x-y)<1e-12 for x,y in zip(fc['extrin_calib']['Pcl'],[0,-.0177,-.08])))
check('vehicle_K_and_half_scale',cc['cam_fx']==381.3611496301472 and cc['scale']==.5 and all(cc['cam_d'+str(i)]==0 for i in range(4)))
check('overview_not_in_SLAM_scenario',set(p.load(run/'navigation_scenario.json')['sensors'])=={'imu','lidar','camera'})
original=p.references([run/n for n in (*p.NAMES,'navigation_slam_contract.json')]);p.prepare(run)
check('idempotent_same_bytes',original==p.references(map(Path,original)))
asset=p.load(run/'asset_manifest.json');asset_raw=(run/'asset_manifest.json').read_bytes()
def asset_case(name,key,value):
    a=dict(asset);a[key]=value;(run/'asset_manifest.json').write_text(json.dumps(a));rejects(name,lambda:p.validate_inputs(run));(run/'asset_manifest.json').write_bytes(asset_raw)
asset_case('wrong_spawn_rejected','spawn',[6,-.7,.4,0])
asset_case('camera_only_rejected','camera_only',True)
asset_case('different_writer_rejected','exclusive_writer','CHAMP')
asset_case('timing_change_rejected','policy_hz',100)
asset_case('dynamic_fixture_rejected','dynamic_fixture',True)
for name in ('independent_protocol.json','terrain_target_manifest.json'):
    raw=(run/name).read_bytes();(run/name).write_bytes(raw+b' ');rejects(name+'_changed_rejected',lambda:p.validate_inputs(run));(run/name).write_bytes(raw)
for name in ('navigation_scope.json','navigation_command.json','navigation_request.json','map_metadata.json'):
    (run/name).write_text('{}');rejects(name+'_existing_rejected',lambda:p.validate_inputs(run));(run/name).unlink()
sensors=p.load(run/'sensor_contract.json')
for name,change in [('wrong_vehicle_camera',{'frame':'teacher_overview_frame'}),('camera2Hz',{'rate_hz':2.})]:
    changed=json.loads(json.dumps(sensors));changed['camera'].update(change)
    raw=(run/'sensor_contract.json').read_bytes();(run/'sensor_contract.json').write_text(json.dumps(changed))
    a=dict(asset);a['sensor_contract_sha256']=p.sha(run/'sensor_contract.json');(run/'asset_manifest.json').write_text(json.dumps(a))
    rejects(name+'_rejected',lambda:p.validate_inputs(run));(run/'sensor_contract.json').write_bytes(raw);(run/'asset_manifest.json').write_bytes(asset_raw)
raw=(run/p.NAMES[0]).read_bytes();(run/p.NAMES[0]).write_bytes(raw+b' ')
rejects('modified_config_hash_rejected',lambda:p.verify(run));(run/p.NAMES[0]).write_bytes(raw)
check('final_actual_file_verify',p.verify(run)['navigation_eligibility'] is False)
check('frozen_sixcore_and_camera_files_unchanged',before==p.references(protected))
runner=ROOT/'scripts/run_test.py';tree=ast.parse(runner.read_text());rt=runner.read_text()
check('runner_explicit_newflag',"'--slam-registration-only'"in rt and "ROOT/'navigation/ramp_registration/producer_profile.py'"in rt)
check('runner_no_old_slam_only_arg_for_newhelper','args.slam_only_stack and not args.slam_registration_only' in rt)
check('runner_source_SHA_expected',p.sha(runner)=='915f743d789a3ce959df4bb192fc95f2821235a4c9b696587b4f52365e3241e6')
launch=ast.parse((ROOT/'navigation/slam.launch.py').read_text())
check('original_SLAM_launch_accepts_config_contract','navigation_slam_contract.json' in (ROOT/'navigation/slam.launch.py').read_text())
check('no_ROS_import_in_helper',not any(isinstance(n,(ast.Import,ast.ImportFrom)) and ('rclpy' in ast.unparse(n) or 'launch' in ast.unparse(n)) for n in ast.walk(ast.parse((HERE/'producer_profile.py').read_text()))))
receipt={'status':'passed','checks':results,'count':len(results),'ROS_initialized':False,'simulation_started':False,
         'offline_asset_copy_origin':str(base),'offline_prepared_run':str(run),'actual_registration':'unverified',
         'source_hashes':p.references([HERE/'producer_profile.py',runner,Path(__file__)]),'producer_contract_sha256':p.sha(run/'navigation_slam_contract.json'),
         'protected_before':before,'protected_after':p.references(protected)}
(OUT/'checks.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({'status':'passed','checks':len(results),'receipt':str(OUT/'checks.json')}))
