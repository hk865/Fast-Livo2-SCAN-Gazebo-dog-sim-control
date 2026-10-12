"""Record actual owned producer libraries and frozen post-LIO source stage.
Timestamp semantics are reviewed from the matching upstream GPU sensor version;
no30Hz-derived point time or robot pose enters navigation.
"""
import hashlib,json,os,time
from pathlib import Path
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def group(pgid):
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            text=(p/'stat').read_text();fields=text[text.rfind(')')+2:].split()
            if int(fields[2])==pgid and fields[0]!='Z':yield p
        except (OSError,ValueError):continue
def record(run,plan,identities,here,slam_workspace,timeout_s=25):
    run=Path(run);here=Path(here);slam_workspace=Path(slam_workspace)
    roles={'gz_sensors_gpu_lidar':'libgz-sensors8-gpu_lidar.so',
        'gz_rendering':'libgz-rendering8.so','gz_sim_sensors':'libgz-sim8-sensors-system.so'}
    deadline=time.monotonic()+timeout_s;found={};observations={}
    while time.monotonic()<deadline:
        for role_name in ('gazebo','bridge'):
            for p in group(identities[role_name]['pgid']):
                try:raw=(p/'maps').read_bytes()
                except OSError:continue
                paths=[]
                for line in raw.decode().splitlines():
                    items=line.split(None,5)
                    if len(items)==6 and items[5].startswith('/'):paths.append(items[5].removesuffix(' (deleted)'))
                candidates=roles if role_name=='gazebo' else {'ros_gz_bridge':'/ros_gz_bridge/parameter_bridge'}
                for name,needle in candidates.items():
                    if name in found:continue
                    selected=[x for x in paths if needle in x]
                    if not selected:continue
                    path=Path(selected[0]).resolve()
                    # Keep a real /proc observation, not a fabricated path list.
                    observation=run/f'producer_{name}_proc_maps_{p.name}.txt'
                    if str(observation) not in observations:
                        observation.write_bytes(raw);observations[str(observation)]=sha(observation)
                    found[name]=dict(role=name,path=str(path),sha256=sha(path),observed_pid=int(p.name),
                        load_observation='proc_maps',load_observation_path=str(observation),
                        load_observation_sha256=observations[str(observation)])
        if set(found)==set(roles)|{'ros_gz_bridge'}:break
        time.sleep(.05)
    if set(found)!=set(roles)|{'ros_gz_bridge'}:
        raise RuntimeError('Actual loaded snapshot producer witness incomplete: '+str(set(roles)|{'ros_gz_bridge'}-set(found)))
    source=slam_workspace/'src/fast_livo2_core/src'
    sources={'LIVMapper':source/'LIVMapper.cpp','preprocess':source/'preprocess.cpp',
        'odom_adapter':here.parents[2]/'slam/odom_adapter.py',
        'self_echo_filter':here.parents[2]/'slam/self_echo_filter.py'}
    stage=[]
    for name,path in sources.items():
        if not path.exists():raise RuntimeError('Required post-LIO stage source absent: '+str(path))
        stage.append(dict(role=name,path=str(path),sha256=sha(path)))
    scenario=run/'navigation_scenario.json'
    witness=dict(schema='go2_known_scene_runtime_witness/v1',run_id=run.name,
        producer_model='gz_gpu_lidar_instantaneous_header_snapshot',actual_loaded_libraries_verified=True,
        header_snapshot_time_verified=True,post_lio_single_owner_publication_verified=True,no_robot_truth_pose=True,
        producer_bindings=list(found.values()),stage_source_bindings=stage,
        original_scenario_path=str(scenario),original_scenario_sha256=sha(scenario),
        slam_origin_in_world=json.loads(scenario.read_text())['slam_origin_in_world'],
        loaded_SLAM_receipt_path=str(run/'slam_loaded_binary.json'),
        source_stage_basis='Frozen mapper publishes raw odom and registered full only from its post-LIO owner; VIO updates do not publish those outputs. Body adapter preserves exact integer stamp, IMU extrinsic is zero.',
        timing_review=dict(upstream_gpu_sensor_version='8.2.2',
            gpu_update_source='https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8_8.2.2/src/GpuLidarSensor.cc',
            gpu_update_review='Update renders one complete GPU ray buffer, assigns _now header, then fills/publishes the whole XYZ/intensity/ring buffer; no per-point time field.',
            bridge_conversion_source='https://raw.githubusercontent.com/gazebosim/ros_gz/jazzy/ros_gz_bridge/src/convert/sensor_msgs.cpp',
            bridge_review='PointCloudPacked conversion copies header, layout, fields and payload; integer source stamps and intact filtered source-record membership are checked at runtime.',
            binary_source_equivalence_formally_certified=False,
            inference='Installed/version-bound actual libraries plus upstream same-version GPU implementation and observed point layout support the instantaneous-header simulator model. This is not a physical rotating LiDAR timing claim.'),
        frontend_state_reset=False,frontend_velocity_or_bias_repaired=False,simulation_only=True)
    temporary=run/'known_scene_runtime_witness.tmp'
    temporary.write_text(json.dumps(witness,indent=2)+'\n')
    temporary.replace(run/'known_scene_runtime_witness.json')
    return witness
