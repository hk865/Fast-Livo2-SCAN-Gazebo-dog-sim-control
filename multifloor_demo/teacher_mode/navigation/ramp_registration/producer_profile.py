#!/usr/bin/env python3
"""Prepare the separately authorized ramp12 zero-command SLAM producer.

Only writes configuration and an immutable provenance contract. No ROS imports,
process launch, navigation authorization, route, or velocity command is emitted.
World/provider hashes describe the initialized simulation/privileged Actor;
they are never used to populate SLAM or registration geometry.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT.parent
HERE = Path(__file__).resolve().parent
SCHEMA = 'actual_ramp_registration_slam_producer/v1'
SPAWN = [3.2, 2.0, .52, 0.0]
CORE_FREEZE_SHA = '94eb94fb2865df01aa2d7d88bc8dd0cee24f46d056ab55d9adbf28c51290029c'
ZERO_SHA = 'e081cfb20bef9e61b41ecb62b367872ed96d37a6eff62e9324092ea002e49cca'
PROVIDER_SHA = 'c9d88197a5b0bd4459bf623855d6de7f6f0efa77ae14650bfcdca1581c89da68'
FROZEN_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
NAMES = ('navigation_fastlivo.yaml', 'navigation_camera.yaml', 'navigation_scenario.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def write(path, data):
    path = Path(path)
    raw = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    if path.exists():
        if path.read_text() != raw:
            raise RuntimeError('Refusing to replace a different preparation: ' + str(path))
        return
    path.write_text(raw)


def references(paths):
    return {str(Path(p).resolve()): sha(p) for p in paths}


def sensor_vector(sensor, field):
    values = sensor.get(field)
    if not isinstance(values, list) or len(values) != 3 or not all(
            isinstance(v, (float, int)) and math.isfinite(v) for v in values):
        raise RuntimeError('Malformed physical sensor transform ' + field)
    return values


def validate_inputs(run):
    run = Path(run).resolve()
    if run.parent != (ROOT / 'runs').resolve():
        raise RuntimeError('Preparation belongs to teacher_mode/runs/<unique-run>')
    required = ['world.sdf', 'asset_manifest.json', 'sensor_contract.json',
                'independent_protocol.json', 'terrain_target_manifest.json',
                'cloud_transport_manifest.json', 'cloud_transport.xml']
    if any(not (run / name).is_file() for name in required):
        raise RuntimeError('Prepare full sensors, zero90, lower12, and shm_64m transport first')
    if any((run / name).exists() for name in ('navigation_scope.json', 'navigation_command.json',
                                             'navigation_request.json', 'map_metadata.json')):
        raise RuntimeError('This fresh stationary producer cannot contain navigation or an old map')
    asset, sensors = load(run / 'asset_manifest.json'), load(run / 'sensor_contract.json')
    if (asset.get('terrain') != 'flat' or asset.get('spawn') != SPAWN or
            asset.get('sensors') is not True or asset.get('camera_only') is not False or
            asset.get('dynamic_fixture') or asset.get('exclusive_writer') != 'teacher_sim::TeacherActuator'):
        raise RuntimeError('Only the explicitly initialized ramp12 stand with sole Teacher writer is eligible')
    if asset.get('physics_step_s') != .005 or asset.get('decimation') != 4 or asset.get('policy_hz') != 50:
        raise RuntimeError('Original Teacher physics/policy timing must be preserved')
    if asset.get('world_sha256') != sha(run / 'world.sdf') or asset.get('sensor_contract_sha256') != sha(run / 'sensor_contract.json'):
        raise RuntimeError('Actual generated asset/sensor hashes do not match')
    for name, expected in [('independent_protocol.json', ZERO_SHA), ('terrain_target_manifest.json', PROVIDER_SHA)]:
        if sha(run / name) != expected:
            raise RuntimeError('Frozen stationary protocol/provider changed: ' + name)
    if sha(ROOT / 'tests/ramp_registration_zero90.json') != ZERO_SHA or sha(ROOT / 'tests/terrain_targets/lower12.json') != PROVIDER_SHA:
        raise RuntimeError('Reviewed zero90/lower12 source changed')
    if sha(HERE / 'freeze.json') != CORE_FREEZE_SHA:
        raise RuntimeError('Read-only registration core freeze changed')
    core = load(HERE / 'freeze.json')
    for name, expected in core['source_hashes'].items():
        if sha(HERE / name) != expected:
            raise RuntimeError('Read-only registration core changed: ' + name)
    for name in ('imu', 'camera', 'lidar'):
        sensor = sensors.get(name, {})
        if sensor.get('link') != 'base_link':
            raise RuntimeError('Only the measured/generated base-link sensor extrinsics are prepared')
        sensor_vector(sensor, 'body_xyz'); sensor_vector(sensor, 'body_rpy')
    camera = sensors['camera']
    if (camera.get('frame') != 'demo_camera_optical_frame' or camera.get('width') != 640 or
            camera.get('height') != 480 or camera.get('rate_hz') != 10 or
            not math.isclose(camera.get('horizontal_fov', -1), math.radians(80), abs_tol=1e-8) or
            camera.get('ros_topic') != '/demo/camera' or camera.get('ros_camera_info_topic') != '/demo/camera_info'):
        raise RuntimeError('Actual vehicle 80-degree 640x480 camera at 10Hz is mandatory')
    if sensors['imu'].get('frame') != 'imu_link' or sensors['imu'].get('rate_hz') != 200 or sensors['imu'].get('ros_topic') != '/livox/imu':
        raise RuntimeError('Actual 200Hz imu_link measurements are mandatory')
    if sensors['lidar'].get('frame') != 'velodyne' or sensors['lidar'].get('rate_hz') != 10 or sensors['lidar'].get('ros_topic') != '/demo/teacher/raw_lidar':
        raise RuntimeError('Actual 10Hz vehicle LiDAR is mandatory')
    transport = load(run / 'cloud_transport_manifest.json')
    if (transport.get('schema') != 'cloud_transport_experiment/v1' or transport.get('profile') != 'shm_64m' or
            Path(transport.get('run', '')).resolve() != run or
            Path(transport.get('xml_path', '')).resolve() != run / 'cloud_transport.xml' or
            transport.get('xml_sha256') != sha(run / 'cloud_transport.xml')):
        raise RuntimeError('Unique reviewed shm_64m transport manifest is required')
    refs = transport.get('source_hashes', {})
    if not refs or any(not Path(p).is_absolute() or sha(p) != expected for p, expected in refs.items()):
        raise RuntimeError('Transport source hashes are unavailable or changed')
    return run, sensors, transport


def configurations(sensors):
    # This is the already validated Teacher SLAM sensor-to-sensor transform.
    # The overview image and simulation initialization pose never enter it.
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation
    cfg = yaml.safe_load((DEMO / 'camera_mode/slam/fastlivo.yaml').read_text())
    camera_cfg = yaml.safe_load((DEMO / 'camera_mode/slam/camera.yaml').read_text())
    params = cfg['/**']['ros__parameters']
    params['common'].update(img_topic='/demo/teacher/slam/image',
                            lid_topic='/demo/slam/lidar_filtered', imu_topic='/demo/teacher/slam/imu')
    imu, lidar, camera = (sensors[k] for k in ('imu', 'lidar', 'camera'))
    ri, rl, rc = [Rotation.from_euler('xyz', s['body_rpy']).as_matrix() for s in (imu, lidar, camera)]
    pi, pl, pc = [np.asarray(s['body_xyz'], dtype=float) for s in (imu, lidar, camera)]
    optical = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)
    params['extrin_calib'].update(extrinsic_T=(ri.T @ (pl-pi)).tolist(),
                                extrinsic_R=(ri.T @ rl).reshape(-1).tolist(),
                                Rcl=(optical @ rc.T @ rl).reshape(-1).tolist(),
                                Pcl=(optical @ rc.T @ (pl-pc)).tolist())
    params['imu']['imu_int_frame'] = 600
    params['imu']['init_min_span'] = 2.9
    cp = camera_cfg['/**']['ros__parameters']
    fx = 640 / (2 * math.tan(camera['horizontal_fov'] / 2))
    if (cp.get('cam_model') != 'Pinhole' or cp.get('cam_width') != 640 or cp.get('cam_height') != 480 or
            cp.get('scale') != .5 or not math.isclose(cp.get('cam_fx', -1), fx, abs_tol=1e-9) or
            not math.isclose(cp.get('cam_fy', -1), fx, abs_tol=1e-9) or cp.get('cam_cx') != 320 or
            cp.get('cam_cy') != 240 or any(cp.get('cam_d' + str(i)) != 0 for i in range(4))):
        raise RuntimeError('Vehicle pinhole calibration does not match the actual 80-degree camera')
    # No overview/world initialization or terrain geometry is consumed by SLAM.
    scenario = {'mode': 'teacher', 'sensors': {k: sensors[k] for k in ('imu', 'lidar', 'camera')}}
    return dict(zip(NAMES, (cfg, camera_cfg, scenario)))


def dependency_paths(run, transport):
    files = [run / n for n in ('world.sdf', 'asset_manifest.json', 'sensor_contract.json',
             'independent_protocol.json', 'terrain_target_manifest.json', 'cloud_transport_manifest.json', 'cloud_transport.xml', *NAMES)]
    files += [Path(__file__), HERE / 'freeze.json', ROOT / 'scripts/run_test.py',
              ROOT / 'navigation/slam.launch.py', ROOT / 'navigation/sensor_relay.py',
              ROOT / 'tests/ramp_registration_zero90.json', ROOT / 'tests/terrain_targets/lower12.json']
    files += [HERE / n for n in load(HERE / 'freeze.json')['source_hashes']]
    files += [DEMO / 'slam' / n for n in ('odom_adapter.py', 'self_echo_filter.py', 'geometry.py', 'map_archive.py')]
    files += [DEMO / 'camera_mode/slam/fastlivo.yaml', DEMO / 'camera_mode/slam/camera.yaml',
              DEMO / 'slam/ros2_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',
              DEMO / 'slam/ros2_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    files += list(map(Path, transport['source_hashes']))
    return files


def verify(run):
    run, sensors, transport = validate_inputs(run)
    contract = load(run / 'navigation_slam_contract.json')
    if (contract.get('schema') != 1 or contract.get('producer_schema') != SCHEMA or
            contract.get('status') != 'prepared_unverified' or contract.get('navigation_started') is not False or
            contract.get('ground_truth_used') is not False or contract.get('navigation_eligibility') is not False or
            contract.get('run_dir') != str(run)):
        raise RuntimeError('Not the separate unverified stationary SLAM producer contract')
    expected = configurations(sensors)
    if any(load(run / name) != value for name, value in expected.items()):
        raise RuntimeError('Prepared actual sensor SLAM configuration differs from the reviewed transform')
    refs = contract.get('references', {})
    mandatory = dependency_paths(run, transport)
    if any(str(p.resolve()) not in refs for p in mandatory) or any(sha(p) != value for p, value in refs.items()):
        raise RuntimeError('Actual SLAM producer source/configuration reference changed')
    return {'status': 'prepared_unverified', 'starts_ros': False, 'run_dir': str(run),
            'launch': str(ROOT / 'navigation/slam.launch.py'), 'configuration': str(run / NAMES[0]),
            'contract_sha256': sha(run / 'navigation_slam_contract.json'),
            'outputs': ['/demo/slam/body_odom', '/aft_mapped_to_init', '/cloud_registered_full'],
            'navigation_started': False, 'navigation_eligibility': False}


def prepare(run):
    run, sensors, transport = validate_inputs(run)
    for name, content in configurations(sensors).items():
        write(run / name, content)
    write(run / 'navigation_slam_contract.json', {
        'schema': 1, 'producer_schema': SCHEMA, 'status': 'prepared_unverified', 'run_dir': str(run),
        'experiment': 'actual_ramp12_slam_cloud_registration_zero90_20261004',
        'ground_truth_used': False, 'navigation_started': False, 'navigation_eligibility': False,
        'checkpoint_sha256': FROZEN_SHA,
        'initialization_only': {'simulation_spawn': SPAWN, 'source': 'explicit initialized simulation fixture',
                                'used_for_registration_geometry_or_heading': False},
        'privileged_actor_only': {'provider_sha256': PROVIDER_SHA, 'provider_id': 'lower12',
                                 'used_for_SLAM_or_registration_geometry': False},
        'command_protocol': {'sha256': ZERO_SHA, 'duration_s': 90, 'command': [0, 0, 0]},
        'actual_sensor_sources': {'imu': '/livox/imu', 'image': '/demo/camera',
                                 'camera_info': '/demo/camera_info', 'raw_lidar': '/demo/teacher/raw_lidar'},
        'image_relay': 'byte-preserving QoS relay; no resize, rectification or stamp change',
        'vehicle_camera': {'horizontal_fov_degrees': 80, 'width': 640, 'height': 480,
                           'actual_rate_hz': 10, 'VIO_scale': .5, 'overview_used': False},
        'geometry_sources': 'actual full cloud + actual SLAM body/raw IMU pose + actual measured IMU acceleration only',
        'references': references(dependency_paths(run, transport))})
    return verify(run)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    print(json.dumps(verify(args.run) if args.check_only else prepare(args.run), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
