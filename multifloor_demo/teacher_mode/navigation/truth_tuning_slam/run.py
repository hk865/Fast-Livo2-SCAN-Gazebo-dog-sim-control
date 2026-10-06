#!/usr/bin/env python3
"""Own a new sensor-SLAM/fixed-route CPU Teacher run. No SCAN/CHAMP process.

Preparation creates new files only. --prepare-only starts no ROS or Gazebo.
Actual launch is exclusively a root/user action, never an audit operation.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEMO = ROOT.parent
CPU_PYTHON = '/home/hyh001/IsaacLab/_isaac_sim/kit/python/bin/python3'
MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
ROS_BASE = Path('/opt/ros/jazzy/setup.bash')
SLAM_OVERLAY = DEMO/'slam/ros2_ws/install/setup.bash'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def module(path, name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec); spec.loader.exec_module(obj)
    return obj


def prepare(args):
    profile_path, core_path = args.profile.resolve(), args.controller.resolve()
    if sha(profile_path) != args.profile_sha256 or sha(core_path) != args.controller_sha256:
        raise RuntimeError('Explicit frozen controller/profile hashes do not match')
    profile = json.loads(profile_path.read_text())
    if profile.get('terrain') != 'flat':
        raise ValueError('This first isolated SLAM transfer is flat only; ramp map registration is separate')
    if profile['design'] == 'open_loop':
        raise ValueError('Open-loop benchmark is not a SLAM feedback deployment candidate')
    if profile['feedback_hz'] > 10 and not args.allow_source_rate_reduction:
        raise ValueError('Mapped SLAM supplies 10Hz; explicitly authorize the selected >10Hz profile transfer')
    if not .1 <= args.distance <= 12 or not 20 <= args.duration <= 600 or not 7 <= args.post_completion <= 30:
        raise ValueError('Bounded flat route/duration/post-completion required')
    if not 1 <= args.ros_domain <= 232 or not .1 <= args.real_time_factor <= 1:
        raise ValueError('Explicit private ROS domain and valid real-time factor required')
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    label = ''.join(x for x in args.label if x.isalnum() or x in '_-')
    run = ROOT/'runs'/f'{stamp}_SLAM_fixed_route_{label}_{uuid.uuid4().hex[:4]}'
    run.mkdir()
    (run/'sources/slam_binding').mkdir(parents=True)
    (run/'sources/policy').mkdir(parents=True)
    for name in ('adapter.py', 'worker.py', 'run.py', 'README.json'):
        if (HERE/name).exists():
            shutil.copy2(HERE/name, run/'sources/slam_binding'/name)
    shutil.copy2(core_path, run/'sources/slam_binding/core.py')
    shutil.copy2(profile_path, run/'frozen_controller_profile.json')
    for name in ('worker.py', 'observation.py', 'contract.json'):
        shutil.copy2(ROOT/'policy'/name, run/'sources/policy'/name)
    prepare_cmd = [sys.executable, str(ROOT/'simulation/prepare.py'), '--output', str(run), '--terrain', 'flat',
        '--sensors', '--camera-rate', '10', '--render-engine', 'ogre2', '--real-time-factor', str(args.real_time_factor),
        '--overview-fov', str(math.radians(80)), '--overview-pose', *map(str, args.overview_pose)]
    with (run/'prepare.log').open('w') as stream:
        subprocess.run(prepare_cmd, check=True, stdout=stream, stderr=subprocess.STDOUT)
    # Existing SLAM-only preparation is explicitly restricted to this tested
    # flat physical spawn. The route origin comes from later actual SLAM.
    tree = ET.parse(run/'world.sdf')
    tree.getroot().find("world/model[@name='go2']/pose").text = '6 -.7 .4 0 0 0'
    ET.indent(tree); tree.write(run/'world.sdf', encoding='unicode')
    asset = json.loads((run/'asset_manifest.json').read_text())
    asset.update(spawn=[6, -.7, .4, 0], world_sha256=sha(run/'world.sdf'))
    write(run/'asset_manifest.json', asset)
    model = tree.getroot().find("world/model[@name='go2']")
    plugins = [p.get('name', '') for p in model.findall('plugin')]
    if sum('TeacherActuator' in n for n in plugins) != 1 or any('ros2_control' in n.lower() or 'champ' in n.lower() for n in plugins):
        raise RuntimeError('Expected exactly one native Teacher joint writer, no old controller')
    actual_com = [float(v) for v in asset['base_inertial_pose'].split()][:3]
    if any(abs(float(a)-float(b)) > 1e-9 for a, b in zip(profile['base_com_offset'], actual_com)):
        raise RuntimeError('Frozen profile COM offset differs from unchanged simulated body')
    for command, logfile in [
        ([sys.executable, str(ROOT/'tests/cloud_transport_probe_20261004/prepare_transport.py'), '--run', str(run), '--profile', 'shm_64m'], 'transport_prepare.log'),
        ([sys.executable, str(ROOT/'navigation/scoped_profile.py'), '--run', str(run), '--slam-only'], 'SLAM_prepare.log')]:
        with (run/logfile).open('w') as stream:
            subprocess.run(command, check=True, stdout=stream, stderr=subprocess.STDOUT)
    relative_route = [[0, 0, 0], [args.distance, 0, 0]]
    if args.roundtrip:
        relative_route.append([0, 0, 0])
    binding = {'schema': 'teacher_slam_fixed_route_binding/v1', 'simulation_only': True,
        'controller_source': {'path': 'sources/slam_binding/core.py', 'sha256': sha(run/'sources/slam_binding/core.py')},
        'controller_profile': {'path': 'frozen_controller_profile.json', 'sha256': sha(run/'frozen_controller_profile.json')},
        'sensor_scenario': {'path': 'navigation_scenario.json', 'sha256': sha(run/'navigation_scenario.json')},
        'fixed_route': {'frame_id': 'initial_SLAM_body_horizontal', 'points_xyz': relative_route,
            'registration': {'source': 'actual_SLAM_initial_body_relative_route',
                'evidence': 'Root-authorized relative flat route: first two fresh actual SLAM body headers, first eligible body pose/yaw anchor frozen exactly once; no Gazebo registration or SCAN replan'}},
        'allow_reduced_source_frequency': bool(args.allow_source_rate_reduction),
        'feedback_mode': 'mapped_slam', 'pose_topic': '/demo/slam/body_odom',
        'imu_topic': '/livox/imu', 'imu_frame': 'imu_link', 'maximum_gyro_pair_gap_s': .02,
        'maximum_tilt_rad': .65, 'publish_twist': False,
        'expected_publishers': {'/demo/slam/body_odom': 'demo_slam_odom_adapter', '/livox/imu': 'ros_gz_bridge'}}
    write(run/'slam_binding.json', binding)
    adapter = module(run/'sources/slam_binding/adapter.py', 'preflight_slam_binding')
    _, _, _, _, _, inspected = adapter.load_binding(run/'slam_binding.json')
    write(run/'prepared_binding_receipt.json', inspected)
    slam_contract = json.loads((run/'navigation_slam_contract.json').read_text())
    transport = json.loads((run/'cloud_transport_manifest.json').read_text())
    source_manifest = {}
    for p in sorted((run/'sources').rglob('*')):
        if p.is_file() and '__pycache__' not in p.parts:
            source_manifest[str(p.relative_to(run/'sources'))] = sha(p)
    external = {**slam_contract['references'], **transport['source_hashes']}
    external.update({str(core_path): args.controller_sha256, str(profile_path): args.profile_sha256,
        str(ROOT/'simulation/prepare.py'): sha(ROOT/'simulation/prepare.py'),
        str(ROOT/'simulation/build/libteacher_actuator.so'): sha(ROOT/'simulation/build/libteacher_actuator.so'),
        str(ROOT/'simulation/teacher_actuator.cpp'): sha(ROOT/'simulation/teacher_actuator.cpp'),
        str(ROOT/'scripts/capture.py'): sha(ROOT/'scripts/capture.py')})
    # Snapshot every external source/binary for later source audit. Live checks
    # protect this run only and do not give historical acceptance a new status.
    snapshots = {}
    for name, digest in external.items():
        p = Path(name)
        dest = run/'sources/external'/(digest[:16]+'_'+p.name)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            shutil.copy2(p, dest)
        snapshots[name] = {'sha256': digest, 'snapshot': str(dest)}
        source_manifest[str(dest.relative_to(run/'sources'))] = sha(dest)
    source_manifest.update(external)
    write(run/'source_manifest.json', source_manifest)
    generated = [run/n for n in ('world.sdf', 'asset_manifest.json', 'sensor_contract.json', 'sensor_bridge.yaml',
        'navigation_slam_contract.json', 'navigation_scenario.json', 'navigation_fastlivo.yaml', 'navigation_camera.yaml',
        'cloud_transport.xml', 'cloud_transport_manifest.json', 'frozen_controller_profile.json', 'slam_binding.json')]
    execution = {'schema': 'teacher_slam_fixed_route_execution/v1', 'allowed': True, 'status': 'experimental_unverified',
        'review_basis': 'explicit_user_truth_tuning_then_sensor_SLAM_transfer_and_root_scoped_authorization',
        'run': str(run), 'duration_s': args.duration, 'post_completion_s': args.post_completion,
        'command_limits': profile['command_limits'], 'binding_sha256': sha(run/'slam_binding.json'),
        'immutable_generated_files': {str(p): sha(p) for p in generated}, 'source_snapshots': snapshots,
        'model_sha256': MODEL_SHA, 'exclusive_writer': 'teacher_sim::TeacherActuator',
        'real_robot': False, 'navigation_ground_truth_used': False, 'actor_observations_remain_privileged': True,
        'SCAN_started': False, 'CHAMP_started': False, 'old_PID_controller_started': False,
        'global_acceptance_modified': False, 'required_roles': ['worker', 'bridge', 'capture', 'SLAM', 'adapter', 'gazebo'],
        'expected_SLAM_children': 6, 'selected_calibration_hz': profile['feedback_hz'], 'fresh_SLAM_expected_hz': 10,
        'feedback_rate_migration': 'Explicit operating-rate reduction; gains unchanged; must be independently validated' if profile['feedback_hz'] > 10 else 'Fresh original SLAM-header feedback',
        'real_time_factor': args.real_time_factor, 'ros_domain': args.ros_domain}
    write(run/'slam_execution.json', execution)
    return run, execution


def stop(proc, group=True):
    if proc is None or proc.poll() is not None:
        return {'already_exited': True}
    signals = []
    for signum, timeout in ((signal.SIGINT, 12 if not group else 5), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            if group:
                os.killpg(proc.pid, signum)
            else:
                os.kill(proc.pid, signum)
            signals.append(int(signum)); proc.wait(timeout=timeout)
            break
        except ProcessLookupError:
            break
        except subprocess.TimeoutExpired:
            # Escalation only targets the owned group. The launch process gets
            # the first interrupt alone and forwards cleanup to its six children.
            group = True
    return {'sent_signals': signals, 'returncode': proc.poll()}


def ros_environment():
    if not ROS_BASE.is_file() or not SLAM_OVERLAY.is_file():
        raise RuntimeError('Required existing ROS/isolated SLAM overlay unavailable')
    result = subprocess.run(['bash', '-c', 'source "$1" >/dev/null; source "$2" >/dev/null; env -0',
        'binding-ros-env', str(ROS_BASE), str(SLAM_OVERLAY)], check=True, stdout=subprocess.PIPE)
    return dict(item.decode().split('=', 1) for item in result.stdout.split(b'\0') if b'=' in item)


def resource(pids=None):
    result = {'monotonic_wall': time.monotonic()}
    commands = {'gpu': ['nvidia-smi', '--query-gpu=memory.used,memory.total,utilization.gpu', '--format=csv,noheader']}
    if pids:
        commands['owned_processes'] = ['ps', '-p', ','.join(map(str, pids)), '-o', 'pid,ppid,%cpu,%mem,rss,args']
    for name, command in commands.items():
        try:
            result[name] = subprocess.check_output(command, text=True)
        except (OSError, subprocess.SubprocessError) as exc:
            result[name+'_error'] = str(exc)
    return result


def execute(run, execution):
    env = ros_environment()
    transport = json.loads((run/'cloud_transport_manifest.json').read_text())
    for key in transport['remove_environment_keys']:
        env.pop(key, None)
    env.update(transport['environment'])
    env.update(ROS_DOMAIN_ID=str(execution['ros_domain']), GZ_IP='127.0.0.1',
        GZ_PARTITION='slam_fixed_'+run.name, GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'simulation/build'),
        TEACHER_SOCKET='/tmp/teacher_slam_fixed_'+uuid.uuid4().hex[:12]+'.sock',
        TEACHER_ACTUATOR_LOG=str(run/'actuator.jsonl'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    env.pop('LIBGL_ALWAYS_SOFTWARE', None); env.pop('GALLIUM_DRIVER', None)
    write(run/'effective_environment.json', {key: env.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
        'ROS_AUTOMATIC_DISCOVERY_RANGE', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE', 'GZ_PARTITION', 'GZ_IP', 'DISPLAY', 'LIBGL_ALWAYS_SOFTWARE', 'GALLIUM_DRIVER')})
    write(run/'resources_before.json', resource())
    children, handles, cleanup = {}, [], {}
    error = None
    def start(role, command, child_env=None):
        log = (run/(role+'.log')).open('w'); handles.append(log)
        child = subprocess.Popen(command, env=env if child_env is None else child_env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children[role] = child
        return child
    def interrupted(signum, frame):
        raise InterruptedError('Runner interrupted; only owned children will be stopped')
    old_handlers = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
    resource_stream = (run/'owned_resources.jsonl').open('w', buffering=1)
    try:
        worker = start('worker', [CPU_PYTHON, str(run/'sources/slam_binding/worker.py'), '--run', str(run), '--socket', env['TEACHER_SOCKET']],
            {**env, 'CUDA_VISIBLE_DEVICES': ''})
        deadline = time.monotonic()+30
        while not (run/'worker_ready').exists():
            if worker.poll() is not None:
                raise RuntimeError('CPU Teacher failed before IPC initialization')
            if time.monotonic() > deadline:
                raise TimeoutError('CPU Teacher readiness timeout')
            time.sleep(.05)
        # Direct executable avoids the previously observed ros2-run wrapper
        # child cleanup defect; this does not change bridge topic/configuration.
        bridge_exe = Path('/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')
        if not bridge_exe.is_file():
            raise RuntimeError('Audited direct ros_gz_bridge executable unavailable')
        start('bridge', [str(bridge_exe), '--ros-args', '-r', '__node:=ros_gz_bridge', '-p', 'config_file:='+str(run/'sensor_bridge.yaml')])
        start('capture', ['/usr/bin/python3', str(ROOT/'scripts/capture.py'), '--run', str(run), '--topic', '/demo/teacher/overview', '--archive-stride', '1'])
        start('SLAM', ['ros2', 'launch', str(ROOT/'navigation/slam.launch.py'), 'run_dir:='+str(run)], {**env, 'DEMO_RUN_DIR': str(run)})
        start('adapter', ['/usr/bin/python3', str(run/'sources/slam_binding/adapter.py'), '--config', str(run/'slam_binding.json'),
            '--output-dir', str(run/'SLAM_fixed_route'), '--command-file', str(run/'slam_velocity_command.json')])
        time.sleep(.5)
        for role, child in children.items():
            if child.poll() is not None:
                raise RuntimeError(role+' exited before simulation started')
        gazebo = start('gazebo', ['gz', 'sim', '-s', '-r', str(run/'world.sdf')])
        sensor_deadline = time.monotonic()+30
        wall_deadline = time.monotonic()+execution['duration_s']/execution['real_time_factor']*3+90
        last_resource = -math.inf
        while worker.poll() is None:
            now = time.monotonic()
            for role, child in children.items():
                if role not in ('gazebo', 'worker') and child.poll() is not None:
                    raise RuntimeError(role+' exited while CPU Teacher remained active')
            if now > sensor_deadline and not (run/'frame_source.json').exists():
                raise TimeoutError('Actual hardware camera failed to become ready in 30 wall seconds')
            if now-last_resource >= 2:
                sample = resource([p.pid for p in children.values() if p.poll() is None])
                resource_stream.write(json.dumps(sample)+'\n'); last_resource = now
                if sample.get('gpu'):
                    used, total, _ = [float(x.split()[0]) for x in sample['gpu'].split(',')]
                    if used > min(14000, total-2048):
                        raise RuntimeError('Total GPU headroom guard exceeded; stop own renderer only')
            if gazebo.poll() is not None:
                worker.wait(timeout=10)
                break
            if now > wall_deadline:
                raise TimeoutError('Owned SLAM route wall deadline')
            time.sleep(.2)
        if worker.returncode != 0:
            raise RuntimeError('CPU Teacher executor exited abnormally')
        result = json.loads((run/'worker_result.json').read_text())
        if result.get('fault') is not None:
            error = 'Actor/native physical fault: '+str(result['fault'])
        try:
            gazebo.wait(timeout=8)
        except subprocess.TimeoutExpired:
            error = error or 'Gazebo did not terminate naturally after done response'
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        for s in old_handlers:
            signal.signal(s, signal.SIG_IGN)
        # SLAM launch is interrupted once, and forwards to its own children.
        for role in ('adapter', 'SLAM', 'capture', 'bridge', 'gazebo', 'worker'):
            cleanup[role] = stop(children.get(role), group=role != 'SLAM')
        Path(env['TEACHER_SOCKET']).unlink(missing_ok=True)
        resource_stream.close()
        for f in handles:
            f.close()
        actual = [{'role': role, 'pid': p.pid, 'returncode': p.returncode} for role, p in children.items()]
        if any(p['returncode'] != 0 for p in actual):
            error = error or 'One or more owned processes did not exit cleanly'
        manifest = {**execution, 'error': error, 'owned_processes': actual, 'cleanup': cleanup,
            'gazebo_partition': env['GZ_PARTITION'], 'ros_domain': execution['ros_domain'],
            'selected_feedback_and_sources': 'Mapped SLAM actual headers <=10Hz; actual 200Hz IMU causal pairing',
            'renderer': 'hardware/Ogre2', 'actor_device': 'cpu', 'training_processes_signaled': False,
            'runtime_status': 'failed' if error else 'completed_requires_independent_route_evaluation',
            'source_manifest_sha256': sha(run/'source_manifest.json'), 'slam_execution_sha256': sha(run/'slam_execution.json')}
        write(run/'runtime_manifest.json', manifest)
        write(run/'resources_after.json', resource())
        # Compatibility receipts cannot turn stand-window diagnostics into a
        # route pass. A future independent evaluator consumes the raw evidence.
        write(run/'summary_slam_fixed_route.json', {'schema': 'teacher_slam_fixed_route_summary/v1',
            'status': 'failed' if error else 'unverified', 'runtime_error': error,
            'levels': {'runtime': 'failed' if error else 'completed', 'route': 'unverified',
                'Sim2Sim': 'unverified', 'real_robot': 'unverified'}, 'navigation_ground_truth_used': False,
            'actor_privileged_observations': True, 'reason': 'Independent original-source/arrival/physical-route/parking evaluation required'})
        for s, handler in old_handlers.items():
            signal.signal(s, handler)
    return error


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--profile', type=Path, required=True)
    ap.add_argument('--profile-sha256', required=True)
    ap.add_argument('--controller', type=Path, required=True)
    ap.add_argument('--controller-sha256', required=True)
    ap.add_argument('--distance', type=float, default=6.)
    ap.add_argument('--roundtrip', action='store_true')
    ap.add_argument('--duration', type=float, default=180.)
    ap.add_argument('--post-completion', type=float, default=7.)
    ap.add_argument('--ros-domain', type=int, default=83)
    ap.add_argument('--real-time-factor', type=float, default=1.)
    ap.add_argument('--overview-pose', type=float, nargs=6, default=[8, 3, 3, 0, .65, -math.pi/2])
    ap.add_argument('--label', default='candidate')
    ap.add_argument('--allow-source-rate-reduction', action='store_true')
    ap.add_argument('--prepare-only', action='store_true')
    args = ap.parse_args()
    run, execution = prepare(args)
    if args.prepare_only:
        print(json.dumps({'run': str(run), 'status': 'prepared_unverified', 'ROS_started': False, 'Gazebo_started': False})); return
    error = execute(run, execution)
    print(json.dumps({'run': str(run), 'runtime_error': error, 'route_validation': 'unverified'}), flush=True)
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
