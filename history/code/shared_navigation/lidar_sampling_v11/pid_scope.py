"""Independent actual-SLAM/SCAN cascade scope authorized for closed-loop tests.

Historical open-loop/global acceptance is immutable evidence, never the gate
for this new bounded simulation-only contract. No truth-navigation source is
permitted. Every executed source and scene is frozen and copied per run.
"""
import hashlib,json,math
from pathlib import Path

HERE=Path(__file__).resolve().parent;NAV=HERE.parent;ROOT=NAV.parent;DEMO=ROOT.parent
SLAM_WS=HERE/'slam_ws'
SCHEMA='teacher_closed_loop_navigation_scope/v1'
FROZEN_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
CACHE={}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    p=Path(p);text=json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if p.exists() and p.read_text()!=text:raise RuntimeError('Refuse changing frozen run input '+str(p))
    if not p.exists():p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text)

def runtime_files():
    # Hash the actual independent core source, build settings and installed
    # artifacts, rather than claiming the protected camera core was executed.
    core_sources=sorted(p for p in (SLAM_WS/'src').rglob('*') if p.is_file() and
        (p.suffix in ('.cpp','.h','.hpp') or p.name in ('CMakeLists.txt','package.xml')))
    build_files=[SLAM_WS/'build'/package/name for package in ('fast_livo2_core','fast_livo2_ros')
                 for name in ('compile_commands.json','CMakeCache.txt')]
    schemas=sorted(HERE.glob('*diagnostic_schema.json'))
    preflight=[HERE/'diagnostic_preflight.json']if(HERE/'diagnostic_preflight.json').is_file()else[]
    preflight+=[HERE/'clock_hold_preflight.json']if(HERE/'clock_hold_preflight.json').is_file()else[]
    preflight+=[HERE/'PROSPECTIVE_CLOCK_HOLD_CRITERIA.json',HERE/'LIO_MULTICORE_CONTRACT.json',HERE/'multicore_preflight.json']
    preflight+=[HERE/'LOGGING_ACCELERATION_CONTRACT.json',HERE/'logging_acceleration_preflight.json']
    preflight+=[HERE/'PUBLICATION_LEDGER_CONTRACT.json',HERE/'PUBLICATION_LEDGER_PREFLIGHT.json',HERE/'PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json']
    return sorted(HERE.glob('*.py'))+sorted((HERE/'profiles').glob('*.json'))+schemas+preflight+core_sources+build_files+[
        HERE/'diagnostic_baseline.json',
        SLAM_WS/'install/setup.bash',SLAM_WS/'install/local_setup.bash']+[
        NAV/n for n in ('runtime_io.py','teacher_transition.py','guard_audit.py','sensor_relay.py')]+[
        DEMO/'navigation'/n for n in ('control_core.py','trajectory_contract.py','goal_regions.py','event_archive.py')]+[
        DEMO/'slam'/n for n in ('odom_adapter.py','self_echo_filter.py','geometry.py','map_archive.py','heading_alignment.py')]+[
        DEMO/'camera_mode/slam/fastlivo.yaml',DEMO/'camera_mode/slam/camera.yaml',
        SLAM_WS/'install/fast_livo2_core/lib/libfast_livo2_core.so',
        SLAM_WS/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',
        DEMO/'navigation/ros2_ws/install/scan_planner/lib/scan_planner/scan_planner_node',
        ROOT/'policy/worker.py',ROOT/'policy/observation.py',ROOT/'policy/contract.json',
        ROOT/'simulation/prepare.py',ROOT/'simulation/teacher_actuator.cpp',ROOT/'simulation/build/libteacher_actuator.so',
        ROOT/'scripts/capture.py',ROOT/'runs/acceptance.json',
        ROOT/'test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_ramp.py',
        ROOT/'test_results/closed_loop_navigation_20261005/acceptance_audit/evaluate_closed_loop.py']

def verify_scope(path,raw=None):
    path=Path(path).resolve();raw=path.read_bytes() if raw is None else raw;d=json.loads(raw);digest=hashlib.sha256(raw).hexdigest()
    if (d.get('schema')!=SCHEMA or d.get('allowed') is not True or d.get('status')!='experimental_unverified'
        or d.get('navigation_is_verified') is not False or d.get('navigation_ground_truth_used') is not False
        or d.get('checkpoint_sha256')!=FROZEN_SHA or d.get('controller_kind')!='teacher'
        or d.get('review_basis')!='explicit_user_closed_loop_controller_actual_SLAM_SCAN_multifloor_simulation'):
        raise RuntimeError('Invalid simulation-only closed-loop scope')
    run=Path(d['run_dir']).resolve();p=d['profile'];refs=d['references']
    if run.parent!=(ROOT/'runs').resolve() or path.parent!=run or p.get('controller_kind')!='teacher':raise RuntimeError('Invalid run identity')
    if p.get('navigation_ground_truth_used') is not False or p.get('pose_cloud_timeout_s')!=.3:raise RuntimeError('Source contract changed')
    from clock_hold import CONTRACT
    if p.get('control_clock_contract')!=CONTRACT:raise RuntimeError('Exact-zero control clock contract changed')
    if not (0<float(p['max_speed_mps'])<=.5 and len(p['cascade']['command_limits'])==3
            and all(0<float(x)<=y for x,y in zip(p['cascade']['command_limits'],[1,.4,1]))):raise RuntimeError('Command bounds invalid')
    fp={str(x):(Path(x).stat().st_size,Path(x).stat().st_mtime_ns) for x in refs}
    if CACHE.get(str(path))!=(digest,fp):
        mandatory=runtime_files()+[run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json',
            'navigation_profile.json','navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json',
            'navigation_slam_contract.json','sensor_sampling_contract.json','terrain_target_manifest.json','cloud_transport_manifest.json','cloud_transport.xml',
            'source_manifest.json','navigation_source_snapshots.json')]
        if any(str(x.resolve()) not in refs for x in mandatory):raise RuntimeError('Missing frozen source or configuration')
        for source,expected in refs.items():
            if sha(source)!=expected:raise RuntimeError('Frozen closed-loop input changed '+source)
        snapshots=json.loads((run/'navigation_source_snapshots.json').read_text())
        for source,row in snapshots.items():
            target=Path(row['snapshot']).resolve()
            if not target.is_relative_to(run/'sources') or sha(target)!=row['sha256'] or refs.get(source)!=row['sha256']:
                raise RuntimeError('Incomplete executed-source archive')
        a=json.loads((run/'asset_manifest.json').read_text())
        if (a['spawn']!=p['spawn'] or a['terrain']!=p['expected_asset']['terrain'] or a['sensors'] is not True
            or a['exclusive_writer']!='teacher_sim::TeacherActuator' or a['physics_step_s']!=.005 or a['decimation']!=4
            or a['world_sha256']!=sha(run/'world.sdf')):raise RuntimeError('Wrong bounded scene or executor')
        if json.loads((run/'navigation_profile.json').read_text())!=p:raise RuntimeError('Profile differs')
        if sha(ROOT/'runs/acceptance.json')!=d['historical_acceptance_sha256']:raise RuntimeError('Historical acceptance changed')
        CACHE[str(path)]=(digest,fp)
    return {'path':str(path),'sha256':digest,'run_dir':str(run),'profile':p,'controller_kind':'teacher',
        'experimental':True,'experiment':p['experiment'],'navigation_is_verified':False,
        'checkpoint_sha256':FROZEN_SHA,'scope':'Actual SLAM/IMU controller with actual SCAN path; privileged Actor disclosed'}

def prepare(run,profile):
    import numpy as np,yaml
    from scipy.spatial.transform import Rotation
    run=Path(run).resolve();s=json.loads((run/'sensor_contract.json').read_text())
    cfg=yaml.safe_load((DEMO/'camera_mode/slam/fastlivo.yaml').read_text());params=cfg['/**']['ros__parameters']
    params['preprocess']['scan_line']=s['lidar']['samples']['vertical']
    params['common'].update(img_topic='/demo/teacher/slam/image',lid_topic='/demo/slam/lidar_filtered',imu_topic='/demo/teacher/slam/imu')
    ri,rl,rc=[Rotation.from_euler('xyz',s[k]['body_rpy']).as_matrix() for k in ('imu','lidar','camera')]
    pi,pl,pc=[np.asarray(s[k]['body_xyz']) for k in ('imu','lidar','camera')];optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    params['extrin_calib'].update(extrinsic_T=(ri.T@(pl-pi)).tolist(),extrinsic_R=(ri.T@rl).reshape(-1).tolist(),
        Rcl=(optical@rc.T@rl).reshape(-1).tolist(),Pcl=(optical@rc.T@(pl-pc)).tolist())
    params['imu'].update(imu_int_frame=int(round(s['imu']['rate_hz']*3)),init_min_span=2.9)
    if profile.get('slam_imu_online_estimation'):
        estimation=profile['slam_imu_online_estimation']
        if estimation!={'gravity_est_en':True,'ba_bg_est_en':True}:
            raise ValueError('Explicit prospective actual-SLAM estimation configuration required')
        params['imu'].update(estimation)
    if profile.get('slam_visual_measurement_covariance') is not None:
        image_cov=profile['slam_visual_measurement_covariance']
        if isinstance(image_cov,bool) or image_cov!=1000:
            raise ValueError('Prospective visual covariance must be the frozen 1000 trial')
        params['vio']['img_point_cov']=image_cov

    write(run/'navigation_fastlivo.yaml',cfg);write(run/'navigation_camera.yaml',yaml.safe_load((DEMO/'camera_mode/slam/camera.yaml').read_text()))
    write(run/'navigation_profile.json',profile);write(run/'navigation_scenario.json',{'mode':'teacher','sensors':s})
    terrain=ROOT/profile['terrain_target_manifest'];write(run/'terrain_target_manifest.json',json.loads(terrain.read_text()))
    layer_files=[]
    if profile.get('terrain_layer_switch'):
        layers=profile['terrain_layer_switch']
        first=ROOT/layers['initial_manifest'];alternate=ROOT/layers['alternate_manifest']
        if first.resolve()!=terrain.resolve():raise RuntimeError('Initial privileged layer differs from frozen policy source')
        write(run/'alternate_terrain_target_manifest.json',json.loads(alternate.read_text()))
        layer_files=[first,alternate,run/'alternate_terrain_target_manifest.json']
    files=runtime_files()+[terrain]+layer_files+[run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json','sensor_bridge.yaml',
        'navigation_profile.json','navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json','terrain_target_manifest.json','sensor_sampling_contract.json')]
    transport=json.loads((run/'cloud_transport_manifest.json').read_text());files += [run/'cloud_transport_manifest.json',run/'cloud_transport.xml']+[Path(x) for x in transport['source_hashes']]
    slamrefs={str(x.resolve()):sha(x) for x in files if 'slam' in str(x) or 'sensor' in str(x) or x.name in ('world.sdf','asset_manifest.json')}
    write(run/'navigation_slam_contract.json',{'schema':1,'status':'prepared_unverified','ground_truth_used':False,
        'independent_diagnostic_workspace':str(SLAM_WS.resolve()),
        'expected_package_prefixes':{package:str((SLAM_WS/'install'/package).resolve())
            for package in ('fast_livo2_core','fast_livo2_ros')},
        'actual_sensor_sources':{'imu':'/livox/imu','image':'/demo/camera','camera_info':'/demo/camera_info','raw_lidar':'/demo/teacher/raw_lidar'},
        'image_relay':'Original real Gazebo body camera stamp/frame; overview excluded','references':slamrefs})
    files += [run/'navigation_slam_contract.json']
    refs={str(x.resolve()):sha(x) for x in files};snapshots={};source_manifest={}
    for source,h in list(refs.items()):
        original=Path(source)
        if original.is_relative_to(run):continue
        rel=original.relative_to(ROOT) if original.is_relative_to(ROOT) else Path('external')/(h[:16]+'_'+original.name)
        dest=run/'sources'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(original.read_bytes());snapshots[source]={'snapshot':str(dest),'sha256':h}
        refs[str(dest)]=h;source_manifest[str(dest.relative_to(run/'sources'))]=h
    source_manifest.update({k:v for k,v in refs.items() if not Path(k).is_relative_to(run)})
    write(run/'source_manifest.json',source_manifest);write(run/'navigation_source_snapshots.json',snapshots)
    for name in ('source_manifest.json','navigation_source_snapshots.json'):refs[str(run/name)]=sha(run/name)
    d={'schema':SCHEMA,'allowed':True,'status':'experimental_unverified','navigation_is_verified':False,
        'navigation_ground_truth_used':False,'controller_kind':'teacher','checkpoint_sha256':FROZEN_SHA,
        'review_basis':'explicit_user_closed_loop_controller_actual_SLAM_SCAN_multifloor_simulation',
        'run_dir':str(run),'profile':profile,'references':refs,'historical_acceptance_sha256':sha(ROOT/'runs/acceptance.json'),
        'runtime':{'required_owned_roles':['worker','bridge','capture','navigation_stack','gazebo'],'expected_launch_children':11},
        'prospective_gates':{'route_xy_max_m':.20,'route_xy_rms_m':.08,'heading_max_rad':.2,
            'COM_speed_MAE_floor_mps':.05,'COM_speed_MAE_reference_fraction':.25,
            'native_roll_pitch_max_rad':.65,'native_clearance_min_m':.18,'body_contact_max':0,
            'joint_torque_max_Nm':23.50001,'joint_speed_max_radps':30.001,
            'active_hold_first_window_s':5.,'active_hold_xy_drift_max_m':.05,
            'active_hold_yaw_drift_max_rad':.1,'active_hold_origin_speed_peak_mps':.08,
            'active_hold_body_wz_and_Euler_yawrate_max_radps':.1,
            'source_pose_and_IMU_timeout_s':.3,'causal_gyro_pair_gap_s':.02},
        'open_loop_error_is_not_closed_loop_launch_gate':True,'meaning':'Permission to run the bounded test; never an outcome pass'}
    write(run/'navigation_scope.json',d);return verify_scope(run/'navigation_scope.json')
