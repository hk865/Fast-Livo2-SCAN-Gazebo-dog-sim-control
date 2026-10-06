"""Explicit new PID experiment scopes, independent of prior flat eligibility."""
import argparse,hashlib,json,math,time
from pathlib import Path

HERE=Path(__file__).resolve().parent;NAV=HERE.parent;ROOT=NAV.parent;DEMO=ROOT.parent
SCHEMA='teacher_pid_navigation_scope/v1'
FROZEN_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
PROFILES={p.stem:p for p in(HERE/'profiles').glob('*.json')};CACHE={}
HEADING_V5_PROFILES={p.stem:p for p in(HERE/'profiles_heading_v5').glob('*.json')}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,data):
    content=json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n';path=Path(path)
    if path.exists()and path.read_text()!=content:raise RuntimeError('Refusing to replace prepared PID evidence: '+str(path))
    if not path.exists():path.write_text(content)
def runtime_files():
    return sorted(HERE.glob('*.py'))+sorted((HERE/'profiles').glob('*.json'))+sorted((HERE/'profiles_heading_v5').glob('*.json'))+sorted(NAV.glob('*.py'))+[
        DEMO/'navigation'/n for n in('controller.py','control_core.py','trajectory_contract.py','goal_regions.py','event_archive.py')]+[
        DEMO/'slam'/n for n in('odom_adapter.py','self_echo_filter.py','geometry.py','map_archive.py')]+[
        DEMO/'camera_mode/slam/fastlivo.yaml',DEMO/'camera_mode/slam/camera.yaml',
        DEMO/'slam/ros2_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',
        DEMO/'slam/ros2_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',
        DEMO/'navigation/ros2_ws/install/scan_planner/lib/scan_planner/scan_planner_node',
        ROOT/'policy/worker.py',ROOT/'policy/observation.py',ROOT/'policy/contract.json',
        ROOT/'simulation/teacher_actuator.cpp',ROOT/'simulation/build/libteacher_actuator.so',
        ROOT/'simulation/prepare.py',ROOT/'scripts/run_test.py',ROOT/'runs/acceptance.json',
        ROOT/'tests/pid_navigation/protocol.json',ROOT/'tests/pid_navigation/cases.json']
def profile_for(selector,kind='teacher',heading_reference=None):
    selector='flat_long'if selector=='flat_straight'else selector
    if heading_reference is not None:
        if heading_reference!='route_leg'or kind!='teacher'or selector not in ('flat_short','flat_long','flat_roundtrip'):
            raise RuntimeError('Route-leg yaw is an explicit Teacher-only three-flat-case V5 experiment')
        profile=json.loads(HEADING_V5_PROFILES[selector].read_text())
        if profile.get('heading_reference')!='route_leg'or profile.get('controller_kind')!='teacher':
            raise RuntimeError('Invalid explicit V5 route-leg heading profile')
        return profile
    if selector not in PROFILES:raise RuntimeError('Unknown explicit PID profile')
    profile=json.loads(PROFILES[selector].read_text())
    if kind not in ('teacher','champ'):raise RuntimeError('Unknown explicit PID executor')
    if kind=='champ':
        profile['controller_kind']='champ';profile['experiment']=profile['experiment'].replace('teacher_pid_','champ_pid_',1)
        profile['policy_observations']='No Teacher actor or privileged actor inputs; CHAMP gait/IK/effort PID executes actual sensor SLAM/SCAN outer PID commands'
    return profile
def cloud_transport_refs(run):
    """Optional prospective run-only DDS configuration; never an old-scope override."""
    run=Path(run).resolve();manifest=run/'cloud_transport_manifest.json'
    if not manifest.exists():return [],None
    data=json.loads(manifest.read_text())
    if(data.get('schema')!='cloud_transport_experiment/v1'or data.get('run')!=str(run)
        or data.get('profile')not in ('shm_512k','shm_64m')):
        raise RuntimeError('Unknown run-scoped cloud transport experiment')
    xml=Path(data['xml_path']).resolve()
    if not xml.is_relative_to(run)or xml.suffix!='.xml'or sha(xml)!=data['xml_sha256']:
        raise RuntimeError('Run-scoped copied cloud transport XML differs')
    sources=data.get('source_hashes',{})
    if not sources or any(not Path(p).is_absolute()or not isinstance(h,str)or len(h)!=64 for p,h in sources.items()):
        raise RuntimeError('Cloud transport sources are not explicitly frozen')
    for name,expected in sources.items():
        if sha(name)!=expected:raise RuntimeError('Cloud transport source changed: '+name)
    environment=data.get('environment',{})
    if(environment.get('ROS_LOCALHOST_ONLY')!='0'or environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='SYSTEM_DEFAULT'
       or environment.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp'):
        raise RuntimeError('Prospective cloud transport must explicitly select its reviewed middleware/discovery mode')
    xml_environment=[environment.get(key)for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')if key in environment]
    if not xml_environment or any(Path(value).resolve()!=xml for value in xml_environment):
        raise RuntimeError('Cloud transport environment does not reference its exact copied XML')
    return [manifest,xml]+[Path(p)for p in sources],{
        'manifest_path':str(manifest),'manifest_sha256':sha(manifest),'profile':data['profile'],
        'xml_path':str(xml),'xml_sha256':data['xml_sha256'],'environment':environment,
        'meaning':'Prospective transport experiment only; no cloud delivery, motion or navigation pass claim'}
def verify_scope(path,raw=None):
    path=Path(path).resolve();raw=path.read_bytes()if raw is None else raw;d=json.loads(raw);digest=hashlib.sha256(raw).hexdigest()
    if(d.get('schema')!=SCHEMA or d.get('allowed')is not True or d.get('status')!='experimental_unverified'
       or d.get('navigation_is_verified')is not False or d.get('navigation_ground_truth_used')is not False
       or d.get('checkpoint_sha256')!=FROZEN_SHA or d.get('review_basis')!='explicit_user_pid_speed_command_teacher_route_and_multifloor_simulation'):
        raise RuntimeError('Unknown/unreviewed PID experiment')
    run=Path(d['run_dir']).resolve();profile=profile_for(d['profile']['selector'],d['controller_kind'],d['profile'].get('heading_reference'))
    if run.parent!=(ROOT/'runs').resolve()or path.parent!=run or d['experiment']!=profile['experiment']or d['profile']!=profile:
        raise RuntimeError('PID scope/profile/run mismatch')
    refs=d.get('references',{});fingerprints={str(p):(Path(p).stat().st_size,Path(p).stat().st_mtime_ns)for p in refs}
    if(run/'cloud_transport_manifest.json').exists():
        fingerprints['run_cloud_transport_manifest_present']=True
    if not refs or any(not Path(p).is_absolute()or len(v)!=64 for p,v in refs.items()):raise RuntimeError('Malformed PID references')
    if CACHE.get(str(path))!=(digest,fingerprints):
        mandatory=runtime_files()+[run/n for n in('world.sdf','asset_manifest.json','sensor_contract.json','navigation_profile.json',
            'navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json','navigation_slam_contract.json',
            'pid_navigation_protocol.json','pid_navigation_case.json','navigation_pid_source_manifest.json',
            'terrain_target_manifest.json','pid_navigation_freeze.json')]+[ROOT/profile['terrain_target_manifest']]
        transport_files,transport=cloud_transport_refs(run);mandatory+=transport_files
        if transport is not None:
            if d.get('cloud_transport_experiment')!=transport:raise RuntimeError('PID scope omits/differs from its transport experiment')
            data=json.loads((run/'cloud_transport_manifest.json').read_text())
            if any(refs.get(p)!=h for p,h in data['source_hashes'].items()):raise RuntimeError('PID scope omits cloud transport source hashes')
        elif 'cloud_transport_experiment'in d:raise RuntimeError('PID scoped cloud transport manifest is missing')
        if d['controller_kind']=='champ':
            mandatory += [run/'champ_contract.json']
            contract=json.loads((run/'champ_contract.json').read_text())
            if(contract.get('schema')!='champ_execution_contract/v1'or contract.get('controller_kind')!='champ'
                or contract.get('run')!=str(run)or contract.get('world_sha256')!=sha(run/'world.sdf')
                or contract.get('exclusive_effort_writer')!='gz_ros2_control::GazeboSimROS2ControlPlugin'
                or contract.get('observer_writes_joint_force')is not False or contract.get('teacher_actor_started')is not False
                or contract.get('physical_scene_identical_except_actuation_plugins')is not True
                or contract.get('body_servo')is not False or contract.get('body_stabilizer')is not False
                or contract.get('state_time_offset_s')!=0 or contract.get('duration_s')!=profile['duration_s']):
                raise RuntimeError('CHAMP comparison requires its own sole-writer exact physical executor contract')
            mandatory += [Path(p)for group in ('artifacts','executor_source_sha256','protected_before')for p in contract[group]]
            for group in ('artifacts','executor_source_sha256','protected_before'):
                if any(refs.get(p)!=h for p,h in contract[group].items()):raise RuntimeError('CHAMP contract sources omitted/differ')
            mandatory += [Path(contract[g]['path'])for g in ('qualified_gait_deployment','controller_manager_library','gz_ros2_control_plugin')]
        if any(str(p.resolve())not in refs for p in mandatory):raise RuntimeError('PID scope omits mandatory frozen source/configuration')
        for p,h in refs.items():
            if sha(p)!=h:raise RuntimeError('PID scoped source changed: '+p)
        archive=json.loads((run/'navigation_pid_source_manifest.json').read_text())['sources']
        for source,row in archive.items():
            target=Path(row['snapshot']).resolve()
            if not target.is_relative_to(run/'sources/pid_runtime')or refs.get(source)!=row['sha256']or refs.get(str(target))!=row['sha256']:
                raise RuntimeError('PID exact executed-source archive is incomplete')
        a=json.loads((run/'asset_manifest.json').read_text());sensor=json.loads((run/'sensor_contract.json').read_text())
        if(a.get('terrain')!=profile['expected_asset']['terrain']or a.get('spawn')!=profile['expected_asset']['spawn']
           or a.get('sensors')is not True or a.get('exclusive_writer')!=('teacher_sim::TeacherActuator'if d['controller_kind']=='teacher'else'gz_ros2_control::GazeboSimROS2ControlPlugin')
           or a.get('physics_step_s')!=.005 or a.get('decimation')!=4 or sha(run/'world.sdf')!=a.get('world_sha256')):
            raise RuntimeError('This explicit PID scene/sole Teacher asset is not eligible')
        if sensor['camera']['rate_hz']<10:raise RuntimeError('Actual vehicle RGB must be >=10Hz')
        if json.loads((run/'navigation_profile.json').read_text())!=profile:raise RuntimeError('Prepared PID configuration mismatch')
        if (run/'terrain_target_manifest.json').read_bytes()!=(ROOT/profile['terrain_target_manifest']).read_bytes():
            raise RuntimeError('Privileged policy height-provider manifest differs from frozen profile')
        if d['global_levels_preserved']!=json.loads((ROOT/'runs/acceptance.json').read_text())['levels']:raise RuntimeError('Global historical acceptance changed')
        case=json.loads((run/'pid_navigation_case.json').read_text())
        if(case['case_id']!=profile['protocol_case_id']or case['spawn']!=a['spawn']or case['goal_deadline_s']!=profile['goal_timeout_sim_s']
            or case['duration_s']!=profile['duration_s']or case['nominal_body_forward_mps']!=profile['max_speed_mps']
            or case['maximum_body_lateral_mps']!=profile['max_lateral_speed_mps']or case['maximum_yaw_rate_radps']!=profile['max_yaw_rate_radps']):
            raise RuntimeError('Prospective PID case/profile differs')
        for row in d['finite_flat_basis']:
            s=json.loads(Path(row['path']).read_text())
            if s.get('status')!='passed'or s.get('levels',{}).get('finite_flat_navigation')!='passed':raise RuntimeError('Prior finite basis failed')
        CACHE[str(path)]=(digest,fingerprints)
    return dict(path=str(path),sha256=digest,checkpoint_sha256=FROZEN_SHA,levels=d['global_levels_preserved'],
        navigation_is_verified=False,experimental=True,experiment=d['experiment'],run_dir=str(run),profile=profile,
        controller_kind=d['controller_kind'],scope=profile['scope'])
def prepare(run,selector='flat_short',controller_kind='teacher',heading_reference=None):
    import numpy as np,yaml
    from scipy.spatial.transform import Rotation
    run=Path(run).resolve();profile=profile_for(selector,controller_kind,heading_reference)
    if run.parent!=(ROOT/'runs').resolve():raise RuntimeError('Use a unique teacher_mode/runs directory')
    asset=json.loads((run/'asset_manifest.json').read_text());sensors=json.loads((run/'sensor_contract.json').read_text())
    if asset.get('terrain')!=profile['expected_asset']['terrain']or asset.get('spawn')!=profile['expected_asset']['spawn']or not asset.get('sensors'):
        raise RuntimeError('Generate exact full-sensor PID profile assets first')
    camera=sensors['camera']
    if camera['frame']!='demo_camera_optical_frame'or camera['width']!=640 or camera['height']!=480 or not math.isclose(camera['horizontal_fov'],math.radians(80),abs_tol=1e-8):raise RuntimeError('VIO vehicle RGB/K mismatch')
    if any(sensors[k]['link']!='base_link'for k in('imu','camera','lidar')):raise RuntimeError('Resolve non-base extrinsics explicitly')
    cfg=yaml.safe_load((DEMO/'camera_mode/slam/fastlivo.yaml').read_text());params=cfg['/**']['ros__parameters']
    params['common'].update(img_topic='/demo/teacher/slam/image',lid_topic='/demo/slam/lidar_filtered',imu_topic='/demo/teacher/slam/imu')
    ri,rl,rc=[Rotation.from_euler('xyz',sensors[k]['body_rpy']).as_matrix()for k in('imu','lidar','camera')]
    pi,pl,pc=[np.asarray(sensors[k]['body_xyz'])for k in('imu','lidar','camera')];optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    params['extrin_calib'].update(extrinsic_T=(ri.T@(pl-pi)).tolist(),extrinsic_R=(ri.T@rl).reshape(-1).tolist(),
        Rcl=(optical@rc.T@rl).reshape(-1).tolist(),Pcl=(optical@rc.T@(pl-pc)).tolist())
    params['imu'].update(imu_int_frame=int(round(sensors['imu']['rate_hz']*3)),init_min_span=2.9)
    write(run/'navigation_fastlivo.yaml',cfg);write(run/'navigation_camera.yaml',yaml.safe_load((DEMO/'camera_mode/slam/camera.yaml').read_text()))
    write(run/'navigation_scenario.json',{'mode':controller_kind,'sensors':sensors});write(run/'navigation_profile.json',profile)
    terrain_source=ROOT/profile['terrain_target_manifest'];terrain_target=run/'terrain_target_manifest.json'
    if terrain_target.exists()and terrain_target.read_bytes()!=terrain_source.read_bytes():raise RuntimeError('Prepared policy terrain provider changed')
    if not terrain_target.exists():terrain_target.write_bytes(terrain_source.read_bytes())
    slam_files=[run/n for n in('world.sdf','asset_manifest.json','sensor_contract.json','navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json')]
    slam_files+=[NAV/'slam.launch.py',NAV/'sensor_relay.py']+[p for p in runtime_files()if str(DEMO/'slam')in str(p)or str(DEMO/'camera_mode/slam')in str(p)]
    write(run/'navigation_slam_contract.json',{'schema':1,'status':'prepared_unverified','ground_truth_used':False,
        'actual_sensor_sources':{'imu':'/livox/imu','image':'/demo/camera','camera_info':'/demo/camera_info','raw_lidar':'/demo/teacher/raw_lidar'},
        'image_relay':'byte-preserving original stamp/frame/640x480 image; overview excluded from VIO','references':{str(p.resolve()):sha(p)for p in slam_files}})
    case=json.loads((ROOT/'tests/pid_navigation/cases.json').read_text())[profile['protocol_case_id']]
    write(run/'pid_navigation_case.json',case);write(run/'pid_navigation_protocol.json',json.loads((ROOT/'tests/pid_navigation/protocol.json').read_text()))
    campaign=ROOT/'test_results/navigation_v4_independent_campaign/navigation_v4_independent_campaign.json';basis=json.loads(campaign.read_text())
    if basis.get('passed_runs')!=3 or basis.get('status')!='passed':raise RuntimeError('Prior exact finite SLAM/SCAN basis unavailable')
    records=[{'path':str(Path(row['run_dir'])/'summary_navigation_independent.json'),'run_id':row['run_id']}for row in basis['runs']]
    files=runtime_files()+slam_files+[campaign]+[Path(row['path'])for row in records]+[run/n for n in('navigation_profile.json','navigation_slam_contract.json','pid_navigation_case.json','pid_navigation_protocol.json','terrain_target_manifest.json')]+[terrain_source]
    refs={str(p.resolve()):sha(p)for p in files};snapshots={}
    transport_files,transport=cloud_transport_refs(run)
    refs.update({str(p.resolve()):sha(p)for p in transport_files})
    if controller_kind=='champ':
        contract=json.loads((run/'champ_contract.json').read_text());refs[str(run/'champ_contract.json')]=sha(run/'champ_contract.json')
        for group in ('artifacts','executor_source_sha256','protected_before'):refs.update(contract[group])
        for group in ('qualified_gait_deployment','controller_manager_library','gz_ros2_control_plugin'):
            refs[contract[group]['path']]=contract[group]['sha256']
    for name,h in list(refs.items()):
        p=Path(name)
        if p.is_relative_to(run):continue
        relative=p.relative_to(ROOT)if p.is_relative_to(ROOT)else Path('external')/(h+'_'+p.name)
        target=run/'sources/pid_runtime'/relative;target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists()and sha(target)!=h:raise RuntimeError('PID source snapshot differs')
        if not target.exists():target.write_bytes(p.read_bytes())
        snapshots[name]={'snapshot':str(target),'sha256':h};refs[str(target.resolve())]=h
    manifest=run/'navigation_pid_source_manifest.json';write(manifest,{'schema':1,'sources':snapshots,'navigation_ground_truth_used':False});refs[str(manifest)]=sha(manifest)
    freeze=run/'pid_navigation_freeze.json';write(freeze,{'schema':'teacher_pid_navigation_freeze/v1','status':'prepared_not_executed',
        'experiment':profile['experiment'],'controller_kind':controller_kind,'protocol_case_id':profile['protocol_case_id'],
        'source_hashes':{str(Path(name).relative_to(ROOT)):sha(Path(name))for name in snapshots if Path(name).is_relative_to(ROOT)},'source_snapshots':snapshots,
        'required_owned_roles':['worker','gazebo','bridge','capture','shadow','navigation_stack']+(['champ_stack']if controller_kind=='champ'else[]),
        'expected_launch_children':15 if controller_kind=='champ'else 11,'navigation_ground_truth_used':False,'global_acceptance_is_preserved':True})
    refs[str(freeze)]=sha(freeze)
    receipt={'schema':SCHEMA,'experiment':profile['experiment'],'controller_kind':controller_kind,'controller':controller_kind+'_pid',
        'protocol_case_id':profile['protocol_case_id'],'allowed':True,'status':'experimental_unverified','navigation_is_verified':False,
        'navigation_ground_truth_used':False,'review_basis':profile['review_basis'],'checkpoint_sha256':FROZEN_SHA,
        'run_dir':str(run),'profile':profile,'global_levels_preserved':json.loads((ROOT/'runs/acceptance.json').read_text())['levels'],
        'finite_flat_basis':records,'references':refs,'meaning':'Explicit permission to test this PID scene; never an actual pass receipt'}
    if transport is not None:receipt['cloud_transport_experiment']=transport
    write(run/'navigation_scope.json',receipt);return verify_scope(run/'navigation_scope.json')
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--profile',default='flat_short');p.add_argument('--controller-kind',default='teacher');p.add_argument('--heading-reference',choices=['route_leg']);p.add_argument('--check-only',action='store_true')
    a=p.parse_args();print(json.dumps(verify_scope(a.run/'navigation_scope.json')if a.check_only else prepare(a.run,a.profile,a.controller_kind,a.heading_reference),indent=2,ensure_ascii=False))
if __name__=='__main__':main()
