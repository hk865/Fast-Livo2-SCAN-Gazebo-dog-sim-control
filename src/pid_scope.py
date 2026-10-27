"""Independent actual-SLAM/SCAN cascade scope authorized for closed-loop tests.

Historical open-loop/global acceptance is immutable evidence, never the gate
for this new bounded simulation-only contract. No truth-navigation source is
permitted. Every executed source and scene is frozen and copied per run.
"""
import package_paths as _package_paths
import hashlib,json,math,os
from pathlib import Path

HERE=Path(__file__).resolve().parent;ROOT=_package_paths.TEACHER_ROOT;NAV=_package_paths.NAVIGATION_ROOT;DEMO=_package_paths.DEMO_ROOT
from slam_workspace import SLAM_WORKSPACE as SLAM_WS, SLAM_SOURCE_ROOT
SCHEMA='teacher_closed_loop_navigation_scope/v1'
FROZEN_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
CACHE={}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    p=Path(p);text=json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if p.exists() and p.read_text()!=text:raise RuntimeError('Refuse changing frozen run input '+str(p))
    if not p.exists():p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text)


STUDENT_SCHEMA='student_closed_loop_navigation_scope/v1'
STUDENT_CHECKPOINT_SHA='7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
STUDENT_EXPORT_SHA='d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
R85_REVIEW_BASIS='explicit_user_R85_PCT_SCAN_Student_bounded_simulation_session'
R85_RUN_INPUTS=('r85_task_contract.json','r85_typed_route_source.json','r85_physical_route_admission.json','r85_scene_geometry_equivalence.json')

def r85_profile_identity(profile):
    if profile.get('navigation_ground_truth_used')is not False or profile.get('actor_kind')!='student' or profile.get('controller_kind')!='student':raise ValueError('Actual SLAM/SCAN Student task required')
    task=profile.get('r85_task',{})
    if task.get('kind')!='f1_to_f3' or task.get('progress_window_sim_s')!=90 or task.get('minimum_progress_m')!=.5:raise ValueError('Task progress bounds invalid')
    return dict(actor_kind='student',controller_kind='student',checkpoint_sha256=STUDENT_CHECKPOINT_SHA,export_sha256=STUDENT_EXPORT_SHA,legacy_teacher_checkpoint_sha256=FROZEN_SHA)

def r85_source_files(profile):
    return [Path(profile['r85_task']['typed_route_source_file'])]

def r85_prepared_files(run,profile):
    return [Path(run)/name for name in R85_RUN_INPUTS]

def verify_external_archives(run,refs,snapshots,manifest,required):
    """Every external required input has an exact archived source, not only refs."""
    run=Path(run).resolve()
    for original in {Path(p).resolve()for p in required}:
        if original.is_relative_to(run):continue
        source=str(original);digest=refs.get(source);record=snapshots.get(source)
        if not isinstance(record,dict) or record.get('sha256')!=digest:
            raise ValueError('R85 required external source lacks archived binding: '+source)
        target=Path(record.get('snapshot','')).resolve()
        if (not target.is_relative_to(run/'sources') or sha(original)!=digest or sha(target)!=digest
            or refs.get(str(target))!=digest or manifest.get(str(target.relative_to(run/'sources')))!=digest):
            raise ValueError('R85 actual source/archive/manifest tuple differs: '+source)

def terrain_source_files(profile=None):
    """All original source manifests used by actual privileged terrain owner."""
    if profile is None:return []
    paths=[ROOT/profile['terrain_target_manifest']]
    if profile.get('mission46_required'):
        paths += [ROOT/'tests/terrain_targets'/n for n in ('lower12.json','upper23.json')]
    elif profile.get('terrain_layer_switch'):
        config=profile['terrain_layer_switch']
        paths += [ROOT/config[k]for k in ('initial_manifest','alternate_manifest')]
    return sorted({p.resolve()for p in paths})

def prepared_dependency_files(run,profile):
    """Preserve original prepare's terrain, mission and transport extras."""
    paths=terrain_source_files(profile)
    if profile.get('mission46_required'):
        from mission46_profile import required_source_files
        paths+=required_source_files()
    transport=json.loads((Path(run)/'cloud_transport_manifest.json').read_text())
    paths+=[Path(p)for p in transport['source_hashes']]
    paths+=r85_prepared_files(run,profile)
    return sorted({p.resolve()for p in paths})

def runtime_files(profile=None):
    # Inspection only. Runtime directly imports this source tree; no approved
    # source, build receipt, historical report or version list is required.
    return [q for q in HERE.rglob('*.py')if '__pycache__'not in q.parts]

def verify_scope(path,raw=None):
    return verify_r85_scope(path,raw)

def verify_r85_scope(path,raw=None):
    path=Path(path).resolve();d=json.loads(path.read_bytes()if raw is None else raw);p=d['profile'];run=Path(d['run_dir']).resolve()
    if path.parent!=run or d.get('navigation_ground_truth_used')is not False:raise RuntimeError('Owned actual-SLAM task context invalid')
    from run_storage import verify_run
    verify_run(run,ROOT)
    from r85_preflight import validate_profile
    validate_profile(p,HERE)
    asset=json.loads((run/'asset_manifest.json').read_text())
    if asset['spawn']!=p['spawn']or asset['exclusive_writer']!='teacher_sim::TeacherActuator'or asset['physics_step_s']!=.005 or asset['decimation']!=4:raise RuntimeError('Physical scene/actuator interface mismatch')
    return dict(path=str(path),sha256=sha(path),run_dir=str(run),profile=p,**r85_profile_identity(p),experimental=True,experiment=p['experiment'],navigation_is_verified=False)

def prepare(run,profile):
    import numpy as np,yaml
    from scipy.spatial.transform import Rotation
    identity=r85_profile_identity(profile)
    r85_inputs=r85_prepared_files(run,profile)
    run=Path(run).resolve();s=json.loads((run/'sensor_contract.json').read_text())
    cfg=yaml.safe_load((ROOT/'config/slam/fastlivo.yaml').read_text());params=cfg['/**']['ros__parameters']
    params['preprocess']['scan_line']=s['lidar']['samples']['vertical']
    params['common'].update(img_topic='/demo/teacher/slam/image',lid_topic='/demo/slam/lidar_filtered',imu_topic='/demo/teacher/slam/imu')
    ri,rl,rc=[Rotation.from_euler('xyz',s[k]['body_rpy']).as_matrix() for k in ('imu','lidar','camera')]
    pi,pl,pc=[np.asarray(s[k]['body_xyz']) for k in ('imu','lidar','camera')];optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    params['extrin_calib'].update(extrinsic_T=(ri.T@(pl-pi)).tolist(),extrinsic_R=(ri.T@rl).reshape(-1).tolist(),
        Rcl=(optical@rc.T@rl).reshape(-1).tolist(),Pcl=(optical@rc.T@(pl-pc)).tolist())
    params['imu'].update(imu_int_frame=int(round(s['imu']['rate_hz']*3)),init_min_span=2.9)
    if profile.get('mission46_required') or identity is not None:
        # Already enabled by the protected base YAML; persist it explicitly in
        # this generated mission config and independently audit real samples.
        params['imu']['stationary_initialization_en']=True
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

    write(run/'navigation_fastlivo.yaml',cfg);write(run/'navigation_camera.yaml',yaml.safe_load((ROOT/'config/slam/camera.yaml').read_text()))
    write(run/'navigation_profile.json',profile)
    scenario=profile['original_scenario'] if (identity is not None or profile.get('mission46_required') or
        profile.get('original46_prefix_regions') is not None) else {'mode':'teacher','sensors':s}
    write(run/'navigation_scenario.json',scenario)
    terrain=ROOT/profile['terrain_target_manifest'];write(run/'terrain_target_manifest.json',json.loads(terrain.read_text()))
    layer_files=[]
    if profile.get('mission46_required'):
        from mission46_profile import required_source_files,validate_profile
        validate_profile(profile)
        alternate=ROOT/'tests/terrain_targets/upper23.json'
        write(run/'alternate_terrain_target_manifest.json',json.loads(alternate.read_text()))
        implementations={
            'multi_request_controller':HERE/'controller.py',
            'whole_mission_worker':HERE/'worker.py',
            'current_rgb_map_save':HERE/'mission46_runtime.py',
            'original_dynamic_obstacle':HERE/'mission46_obstacle_runtime.py',
            'bidirectional_terrain_provider':HERE/'mission46_terrain.py'}
        interfaces={k:{'module':str(v.resolve()),'implementation_sha256':sha(v),'supported':True}
                    for k,v in implementations.items()}
        write(run/'mission46_runtime_interfaces.json',interfaces)
        layer_files += [alternate,run/'alternate_terrain_target_manifest.json',
                        run/'mission46_runtime_interfaces.json',*required_source_files()]
    if profile.get('terrain_layer_switch'):
        layers=profile['terrain_layer_switch']
        first=ROOT/layers['initial_manifest'];alternate=ROOT/layers['alternate_manifest']
        if first.resolve()!=terrain.resolve():raise RuntimeError('Initial privileged layer differs from frozen policy source')
        write(run/'alternate_terrain_target_manifest.json',json.loads(alternate.read_text()))
        layer_files=[first,alternate,run/'alternate_terrain_target_manifest.json']
    capture_path=run/'known_scene_event_capture_contract.json'
    capture=json.loads(capture_path.read_text())
    capture_path.write_text(json.dumps(capture,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    from policy_selection import binding
    write(run/'selected_policy_binding.json',binding(run))
    data_files=r85_inputs+[run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json','navigation_profile.json','navigation_scenario.json','terrain_target_manifest.json','known_scene_reference.json','navigation_fastlivo.yaml','navigation_camera.yaml','sensor_sampling_contract.json','cloud_transport_manifest.json','cloud_transport.xml','known_scene_event_capture_contract.json')]
    refs={str(q.resolve()):sha(q)for q in data_files}
    write(run/'navigation_slam_contract.json',dict(actual_sensor_sources=dict(imu='/livox/imu',image='/demo/camera',raw_lidar='/demo/teacher/raw_lidar'),references={}))
    refs[str(run/'navigation_slam_contract.json')]=sha(run/'navigation_slam_contract.json')
    # Compatibility metadata contains current data only, never source bindings.
    write(run/'source_manifest.json',{})
    write(run/'navigation_source_snapshots.json',{})
    d={'schema':SCHEMA,'allowed':True,'status':'experimental_unverified','navigation_is_verified':False,
        'navigation_ground_truth_used':False,'controller_kind':'teacher','checkpoint_sha256':FROZEN_SHA,
        'review_basis':'explicit_user_closed_loop_controller_actual_SLAM_SCAN_multifloor_simulation',
        'run_dir':str(run),'profile':profile,'references':refs,
        'runtime':{'required_owned_roles':['worker','bridge','capture','navigation_stack','gazebo'],
                   'expected_launch_children':(14 if profile.get('mission46_required') else 12)-int(not profile.get('engineering_recording',{}).get('raw_source_capture',True))+1+(2 if identity is not None and profile['r85_task']['kind']=='f1_to_f3'else 0)},
        'prospective_gates':{'route_xy_max_m':.20,'route_xy_rms_m':.08,'heading_max_rad':.2,
            'COM_speed_MAE_floor_mps':.05,'COM_speed_MAE_reference_fraction':.25,
            'native_roll_pitch_max_rad':.65,'native_clearance_min_m':.18,'body_contact_max':0,
            'joint_torque_max_Nm':23.50001,'joint_speed_max_radps':30.001,
            'active_hold_first_window_s':5.,'active_hold_xy_drift_max_m':.05,
            'active_hold_yaw_drift_max_rad':.1,'active_hold_origin_speed_peak_mps':.08,
            'active_hold_body_wz_and_Euler_yawrate_max_radps':.1,
            'source_pose_and_IMU_timeout_s':.3,'causal_gyro_pair_gap_s':.02},
        'open_loop_error_is_not_closed_loop_launch_gate':True,'meaning':'Permission to run the bounded test; never an outcome pass'}
    if identity is not None:
        d.update(schema=STUDENT_SCHEMA,review_basis=R85_REVIEW_BASIS,**identity)
        d.update(r85_session=dict(profile['r85_session']),r85_task=dict(profile['r85_task']),
            pct_execution='offline_generated_route_only; not executed by scope preparation',
            original46_score_drives_task=False,actual_active_from_static_proof=False)
    write(run/'navigation_scope.json',d);return verify_scope(run/'navigation_scope.json')
