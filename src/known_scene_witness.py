"""Record actual loaded producer paths and process observations.
Real message stamps, commit tokens and point membership are validated downstream;
source files, code hashes and historical versions are not launch prerequisites.
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
def record(run,plan,identities,here,timeout_s=25):
    run=Path(run);here=Path(here)
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
                        observation.write_bytes(raw);observations[str(observation)]=True
                    found[name]=dict(role=name,path=str(path),observed_pid=int(p.name),
                        load_observation='proc_maps',load_observation_path=str(observation),)
        if set(found)==set(roles)|{'ros_gz_bridge'}:break
        time.sleep(.05)
    if set(found)!=set(roles)|{'ros_gz_bridge'}:
        raise RuntimeError('Actual loaded snapshot producer witness incomplete: '+str(set(roles)|{'ros_gz_bridge'}-set(found)))
    process_bindings={}
    loaded=json.loads((run/'slam_loaded_binary.json').read_text())
    process_bindings['mapper']=loaded['mapping_identity']
    if loaded.get('verified')is not True:raise RuntimeError('New mapper actual binary witness missing')
    for role,file in [('source',here/'self_echo_filter.py'),('adapter',here/'odom_adapter.py')]:
        matches=[]
        for p in group(identities['navigation_stack']['pgid']):
            try:
                raw=(p/'cmdline').read_bytes();arguments=raw.decode().strip('\0').split('\0')
                if str(file) not in arguments:continue
                text=(p/'stat').read_text();fields=text[text.rfind(')')+2:].split()
                target=run/('identity_producer_'+role+'_cmdline.txt');target.write_bytes(raw)
                matches.append(dict(pid=int(p.name),start_ticks=int(fields[19]),pgid=int(fields[2]),
                    session=int(fields[3]),source_path=str(file),observation=str(target)))
            except OSError:continue
        if len(matches)!=1:raise RuntimeError('R60 actual identity producer not unique: '+role)
        process_bindings[role]=matches[0]
    scenario=run/'navigation_scenario.json' 
    witness=dict(schema='go2_known_scene_runtime_witness/v1',run_id=run.name,
        explicit_point_commit_required=True,identity_fingerprint_model='go2_ros_semantic/v1',
        identity_producer_process_bindings=process_bindings,
        producer_model='gz_gpu_lidar_instantaneous_header_snapshot',actual_loaded_libraries_verified=True,
        header_snapshot_time_verified=True,post_lio_single_owner_publication_verified=True,no_robot_truth_pose=True,
        producer_bindings=list(found.values()),
        original_scenario_path=str(scenario),original_scenario_sha256=sha(scenario),
        slam_origin_in_world=json.loads(scenario.read_text())['slam_origin_in_world'],
        loaded_SLAM_receipt_path=str(run/'slam_loaded_binary.json'),
        source_stage_basis='Actual owned mapper/adapter process IDs and loaded producer paths; real message commit tokens, integer stamps and point membership are verified on incoming data. No C++/Python source presence or version is a runtime prerequisite.',
        timing_review=dict(model='simulation_GPU_header_snapshot',basis='actual producer roles and exact real message/header/point associations; no library/source version whitelist'),
        frontend_state_reset=False,frontend_velocity_or_bias_repaired=False,simulation_only=True)
    temporary=run/'known_scene_runtime_witness.tmp'
    temporary.write_text(json.dumps(witness,indent=2)+'\n')
    temporary.replace(run/'known_scene_runtime_witness.json')
    return witness
