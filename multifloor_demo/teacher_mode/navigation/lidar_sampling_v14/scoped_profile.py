#!/usr/bin/env python3
"""Prepare and verify a bounded navigation experiment, never upgrade acceptance.

Generation reads existing real motion evidence and generated simulation assets.
It starts no ROS or simulation process. Runtime hash checks are cached only while
all referenced file size/mtime fingerprints remain unchanged.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
NAV = HERE.parent
ROOT = HERE.parents[1]
DEMO = ROOT.parent
SLAM_WS = Path(__file__).resolve().parent.parent/'lidar_sampling_v12/slam_ws'
FROZEN_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
SCHEMA = 'teacher_navigation_scope/v1'
PROFILE = NAV/'flat_relative_roundtrip.json'
DYNAMIC_PROFILE = NAV/'dynamic/flat_dynamic_profile.json'
DYNAMIC_TURN20_PROFILE = NAV/'dynamic/flat_dynamic_profile_turn20.json'
DYNAMIC_DWELL_PROFILE = NAV/'dynamic/flat_dynamic_profile_dwell.json'
DYNAMIC_EXPERIMENTS = {'finite_flat_dynamic_stop_resume_v1', 'finite_flat_dynamic_stop_resume_turn20_v1',
                       'finite_flat_dynamic_stop_resume_dwell_v1'}
EXPERIMENTS = {'flat_relative_roundtrip_v1': PROFILE, 'finite_flat_dynamic_stop_resume_v1': DYNAMIC_PROFILE,
              'finite_flat_dynamic_stop_resume_turn20_v1': DYNAMIC_TURN20_PROFILE,
              'finite_flat_dynamic_stop_resume_dwell_v1': DYNAMIC_DWELL_PROFILE}
FLAT = ('stand', 'forward', 'backward', 'left', 'right', 'turn_positive',
        'turn_negative', 'walk_stop', 'command_timeout', 'switch')
_cache = {}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, data):
    path = Path(path)
    content = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    if path.exists():
        if path.read_text() == content:
            return
        raise RuntimeError('Refusing to replace different navigation preparation: ' + str(path))
    path.write_text(content)


def references(paths):
    return {str(Path(p).resolve()): sha(p) for p in paths}


def verify_scope(path, raw=None):
    """Authorize this finite experiment, without a formal motion/Sim2Sim pass."""
    path = Path(path).resolve()
    raw = path.read_bytes() if raw is None else raw
    digest = hashlib.sha256(raw).hexdigest()
    data = json.loads(raw)
    if data.get('schema') == 'teacher_closed_loop_navigation_scope/v1':
        from pid_scope import verify_scope as verify_diagnostic_scope
        return verify_diagnostic_scope(path, raw)
    if data.get('schema') != SCHEMA or data.get('experiment') not in EXPERIMENTS:
        raise RuntimeError('Unknown experimental navigation scope')
    if data.get('status') != 'experimental_unverified' or data.get('navigation_is_verified') is not False:
        raise RuntimeError('Experimental receipt must explicitly remain unverified')
    if data.get('allowed') is not True or data.get('checkpoint_sha256') != FROZEN_SHA:
        raise RuntimeError('Experimental scope/checkpoint is not eligible')
    if data.get('navigation_ground_truth_used') is not False:
        raise RuntimeError('Ground truth is forbidden for navigation')
    if data.get('review_basis') != 'explicit_user_continuation_and_root_scope_authorization':
        raise RuntimeError('Missing bounded experiment authorization basis')
    run = Path(data['run_dir']).resolve()
    from run_storage import verify_run
    verify_run(run,ROOT)
    if path.parent != run:
        raise RuntimeError('Scope must belong to its unique Teacher run directory')
    refs = data.get('references', {})
    if not isinstance(refs, dict) or not refs:
        raise RuntimeError('Missing source and evidence hashes')
    fingerprints = {}
    for name, expected in refs.items():
        p = Path(name)
        if not p.is_absolute() or not isinstance(expected, str) or len(expected) != 64:
            raise RuntimeError('Malformed scoped reference')
        try:
            st = p.stat()
        except OSError as exc:
            raise RuntimeError('Scoped reference unavailable: '+str(p)) from exc
        fingerprints[name] = (st.st_size, st.st_mtime_ns)
    old = _cache.get(str(path))
    if old is None or old[0] != digest or old[1] != fingerprints:
        reviewed_profile=EXPERIMENTS[data['experiment']]
        mandatory=[reviewed_profile,ROOT/'runs/acceptance.json',ROOT/'policy/worker.py',ROOT/'policy/observation.py',
            ROOT/'policy/contract.json',ROOT/'simulation/build/libteacher_actuator.so']
        mandatory+=list(Path(__file__).parent.glob('*.py'))
        if data['experiment']in DYNAMIC_EXPERIMENTS:
            from dynamic.prepare import runtime_files
            mandatory+=runtime_files()+[run/'dynamic_protocol.json',run/'dynamic_original_world.sdf',run/'dynamic_original_asset_manifest.json']
        mandatory+=[run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json',
            'navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json','navigation_profile.json')]
        if any(str(p.resolve())not in refs for p in mandatory):
            raise RuntimeError('Experimental receipt omits required live source/configuration hashes')
        for name, expected in refs.items():
            if sha(name) != expected:
                raise RuntimeError('Scoped source/evidence hash changed: '+name)
        profile = json.loads((run/'navigation_profile.json').read_text())
        if profile != json.loads(reviewed_profile.read_text()):
            raise RuntimeError('Scoped profile differs from reviewed finite experiment')
        actual_global=json.loads((ROOT/'runs/acceptance.json').read_text())
        if data.get('global_levels_preserved')!=actual_global['levels']:
            raise RuntimeError('Experimental receipt must preserve the actual overall acceptance levels')
        asset = json.loads((run/'asset_manifest.json').read_text())
        if asset.get('terrain') != 'flat' or asset.get('spawn') != [6, -.7, .4, 0]:
            raise RuntimeError('Only the previously tested open flat spawn is eligible')
        if asset.get('sensors') is not True or asset.get('exclusive_writer') != 'teacher_sim::TeacherActuator':
            raise RuntimeError('Full actual sensors and sole native Teacher actuator are required')
        if data['experiment']=='flat_relative_roundtrip_v1'and asset.get('dynamic_fixture'):
            raise RuntimeError('Old finite flat scope cannot authorize a changed dynamic fixture')
        if data['experiment']in DYNAMIC_EXPERIMENTS:
            from dynamic.prepare import validate_asset
            validate_asset(run)
        if sha(run/'world.sdf') != asset.get('world_sha256'):
            raise RuntimeError('World differs from generated asset receipt')
        sensor_contract=json.loads((run/'sensor_contract.json').read_text())
        if sensor_contract['camera']['rate_hz']<10:
            raise RuntimeError('Finite navigation requires actual vehicle RGB rate >=10Hz; 2Hz was stationary diagnostics only')
        counts = data.get('flat_motion_evidence', {})
        if set(counts) != set(FLAT) or any(len(counts[k]) != 3 for k in FLAT):
            raise RuntimeError('Three actual baseline runs per required flat capability are required')
        for items in counts.values():
            for item in items:
                if str(Path(item['summary']).resolve())not in refs:
                    raise RuntimeError('Experimental receipt omits original flat evidence hash')
                s = json.loads(Path(item['summary']).read_text())
                if any(s.get('levels', {}).get(k) != 'passed' for k in ('interface', 'motion')):
                    raise RuntimeError('A referenced flat capability did not pass')
        _cache[str(path)] = (digest, fingerprints)
    return dict(path=str(path), sha256=digest, checkpoint_sha256=FROZEN_SHA,
                levels=data['global_levels_preserved'], navigation_is_verified=False,
                experimental=True, experiment=data['experiment'], run_dir=str(run),
                profile=data['profile'], scope='Finite flat SLAM/SCAN integration experiment only')


def prepare(run, slam_only=False, dynamic=False, dynamic_turn20=False, dynamic_dwell=False):
    if dynamic_turn20 and not dynamic:raise RuntimeError('Turn20 is an explicit dynamic experiment only')
    if dynamic_dwell and (not dynamic or not dynamic_turn20 or slam_only):raise RuntimeError('Dwell policy requires explicit dynamic Turn20 navigation')
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation
    run = run.resolve()
    from run_storage import verify_run
    verify_run(run,ROOT)
    for name in ('world.sdf', 'asset_manifest.json', 'sensor_contract.json'):
        if not (run/name).is_file():
            raise RuntimeError('Generate the full-sensor Teacher assets first: '+name)
    asset = json.loads((run/'asset_manifest.json').read_text())
    sensors = json.loads((run/'sensor_contract.json').read_text())
    if asset.get('dynamic_fixture')and not dynamic:
        raise RuntimeError('Changed dynamic fixture requires explicit --dynamic scope')
    if asset.get('terrain') != 'flat' or asset.get('spawn') != [6, -.7, .4, 0] or not asset.get('sensors'):
        raise RuntimeError('Initial experiment requires full sensors and the tested flat spawn [6,-.7,.4,0]')
    if not all(k in sensors for k in ('imu', 'camera', 'lidar')):
        raise RuntimeError('Missing real sensor extrinsics')
    if any(sensors[k].get('link') != 'base_link' for k in ('imu','camera','lidar')):
        raise RuntimeError('Resolve non-base sensor link transforms explicitly before using this profile')
    camera = sensors['camera']
    if (camera.get('frame') != 'demo_camera_optical_frame' or camera.get('width') != 640
            or camera.get('height') != 480 or not math.isclose(camera['horizontal_fov'], math.radians(80), abs_tol=1e-8)):
        raise RuntimeError('VIO requires the actual vehicle 80-degree 640x480 camera')
    cfg = yaml.safe_load((DEMO/'camera_mode/slam/fastlivo.yaml').read_text())
    params = cfg['/**']['ros__parameters']
    params['common']['img_topic'] = '/demo/teacher/slam/image'
    # Actual option in this checkout is img_topic, not a guessed ROS remap.
    params['common']['lid_topic'] = '/demo/slam/lidar_filtered'
    params['common']['imu_topic'] = '/demo/teacher/slam/imu'
    ri = Rotation.from_euler('xyz', sensors['imu']['body_rpy']).as_matrix()
    rl = Rotation.from_euler('xyz', sensors['lidar']['body_rpy']).as_matrix()
    rc = Rotation.from_euler('xyz', camera['body_rpy']).as_matrix()
    pi = np.asarray(sensors['imu']['body_xyz']); pl = np.asarray(sensors['lidar']['body_xyz'])
    pc = np.asarray(camera['body_xyz'])
    optical = np.array([[0,-1,0],[0,0,-1],[1,0,0]], dtype=float)
    rcl = optical @ rc.T @ rl
    params['extrin_calib']['extrinsic_T'] = (ri.T @ (pl-pi)).tolist()
    params['extrin_calib']['extrinsic_R'] = (ri.T @ rl).reshape(-1).tolist()
    params['extrin_calib']['Rcl'] = rcl.reshape(-1).tolist()
    params['extrin_calib']['Pcl'] = (optical @ rc.T @ (pl-pc)).tolist()
    params['imu']['imu_int_frame'] = int(round(sensors['imu']['rate_hz']*3))
    params['imu']['init_min_span'] = 2.9
    # JSON is valid YAML. Preserve every unmodified original VIO/LIO setting.
    write(run/'navigation_fastlivo.yaml', cfg)
    camera_cfg = yaml.safe_load((DEMO/'camera_mode/slam/camera.yaml').read_text())
    write(run/'navigation_camera.yaml', camera_cfg)
    write(run/'navigation_scenario.json', {'mode':'teacher', 'sensors':sensors})
    profile_path=DYNAMIC_DWELL_PROFILE if dynamic_dwell else DYNAMIC_TURN20_PROFILE if dynamic_turn20 else DYNAMIC_PROFILE if dynamic else PROFILE
    profile = json.loads(profile_path.read_text())
    write(run/'navigation_profile.json', profile)
    slam_files = [run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json',
                                 'navigation_fastlivo.yaml','navigation_camera.yaml','navigation_scenario.json')]
    slam_files += [Path(__file__), HERE/'slam.launch.py', NAV/'sensor_relay.py']
    from pid_scope import runtime_files as diagnostic_runtime_files
    slam_files += [p for p in diagnostic_runtime_files() if p.is_relative_to(SLAM_WS)]
    slam_files += [DEMO/'slam'/n for n in ('odom_adapter.py','self_echo_filter.py','geometry.py','map_archive.py')]
    slam_files += [DEMO/'camera_mode/slam/fastlivo.yaml',DEMO/'camera_mode/slam/camera.yaml',
                  SLAM_WS/'install/fast_livo2_core/lib/libfast_livo2_core.so',
                  SLAM_WS/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping']
    write(run/'navigation_slam_contract.json', {'schema':1,'status':'prepared_unverified',
        'ground_truth_used':False,'actual_sensor_sources':{'imu':'/livox/imu','image':'/demo/camera',
        'camera_info':'/demo/camera_info','raw_lidar':'/demo/teacher/raw_lidar'},
        'image_relay':'byte-preserving QoS relay; no resize, rectification or stamp change',
        'references':references(slam_files)})
    if slam_only:
        return {'status':'prepared_unverified','starts_ros':False,'run_dir':str(run),
                'launch':str(Path(__file__).with_name('slam.launch.py')),
                'configuration':str(run/'navigation_fastlivo.yaml'),
                'outputs':['/demo/slam/body_odom','/cloud_registered_full'],
                'navigation_started':False}
    acceptance = json.loads((ROOT/'runs/acceptance.json').read_text())
    if camera['rate_hz']<10:
        raise RuntimeError('Generate this navigation run with --camera-rate 10; 2Hz cannot meet 300ms SLAM gate')
    candidates = acceptance.get('history', {}).get('all_runs', [])
    # Existing aggregate has run rows inside historical_runs; avoid selecting a
    # lucky subset by falling back to the exact preregistered formal campaign.
    if not candidates:
        candidates = []
        for d in sorted((ROOT/'runs').iterdir()):
            if not d.is_dir() or d.name < '20261003_193541' or d.is_symlink():
                continue
            if not (d/'summary.json').is_file() or not (d/'asset_manifest.json').is_file():
                continue
            a = json.loads((d/'asset_manifest.json').read_text())
            s = json.loads((d/'summary.json').read_text())
            if s.get('test') not in FLAT or a.get('terrain') != 'flat' or a.get('spawn') != [6,-.7,.4,0]:
                continue
            if a.get('spawn_override') or a.get('sensors') or a.get('camera_only'):
                continue
            # Formal ids have no scenario suffix between test and repetition.
            if not any('_'+k+'_r' in d.name and d.name.split('_'+k+'_r')[1].split('_')[0] in ('1','2','3') for k in FLAT):
                continue
            candidates.append({'run_id':d.name,'test':s['test'],'formal':True})
    evidence = {k:[] for k in FLAT}
    for item in candidates:
        if item.get('formal') is not True or item.get('test') not in FLAT:
            continue
        summary = ROOT/'runs'/item['run_id']/'summary.json'
        s = json.loads(summary.read_text())
        evidence[item['test']].append({'run_id':item['run_id'], 'summary':str(summary.resolve()),
                                      'interface':s['levels']['interface'], 'motion':s['levels']['motion']})
    if any(len(v) != 3 or any(x['interface'] != 'passed' or x['motion'] != 'passed' for x in v) for v in evidence.values()):
        raise RuntimeError('Required exact three formal flat repetitions are missing or failed')
    files = list(Path(__file__).parent.glob('*.py')) + [profile_path]
    if dynamic:
        from dynamic.prepare import runtime_files,basis
        basis_records,basis_campaign=basis()
        files+=runtime_files()+[run/'dynamic_protocol.json',run/'dynamic_original_world.sdf',run/'dynamic_original_asset_manifest.json',basis_campaign]+[Path(row['path'])for row in basis_records]
        if asset.get('dynamic_fixture',{}).get('protocol_sha256')!=sha(run/'dynamic_protocol.json'):
            raise RuntimeError('Prepare and freeze the physical dynamic asset before navigation scope')
    files += [run/n for n in ('world.sdf','asset_manifest.json','sensor_contract.json','navigation_fastlivo.yaml',
                              'navigation_camera.yaml','navigation_scenario.json','navigation_profile.json')]
    files += [ROOT/'policy/worker.py', ROOT/'policy/observation.py', ROOT/'policy/contract.json',
              ROOT/'simulation/build/libteacher_actuator.so', ROOT/'runs/acceptance.json']
    files += [DEMO/'navigation'/n for n in ('controller.py','control_core.py','trajectory_contract.py','goal_regions.py','event_archive.py')]
    files += [DEMO/'slam'/n for n in ('odom_adapter.py','self_echo_filter.py','geometry.py','map_archive.py')]
    files += [DEMO/'camera_mode/slam/fastlivo.yaml', DEMO/'camera_mode/slam/camera.yaml']
    files += [SLAM_WS/'install/fast_livo2_core/lib/libfast_livo2_core.so',
              SLAM_WS/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',
              DEMO/'navigation/ros2_ws/install/scan_planner/lib/scan_planner/scan_planner_node']
    files += [p for p in diagnostic_runtime_files() if p.is_relative_to(SLAM_WS)]
    files += [Path(x['summary']) for values in evidence.values() for x in values]
    receipt = dict(schema=SCHEMA, experiment=profile['experiment'], status='experimental_unverified',
        allowed=True, navigation_is_verified=False, navigation_ground_truth_used=False,
        review_basis='explicit_user_continuation_and_root_scope_authorization', run_dir=str(run),
        checkpoint_sha256=FROZEN_SHA, global_levels_preserved=acceptance['levels'],
        excluded_scenarios=profile['excluded'], profile=profile, flat_motion_evidence=evidence,
        references=references(files), prepared_wall_unix=time.time(),
        meaning='Permission to test a finite scope, not a pass receipt; existing failed and unverified global levels remain unchanged')
    write(run/'navigation_scope.json', receipt)
    result=verify_scope(run/'navigation_scope.json')
    if dynamic:
        from dynamic.prepare import prepare_scope
        prepare_scope(run)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--slam-only', action='store_true', help='Prepare only actual sensor SLAM configuration; no navigation command eligibility')
    parser.add_argument('--dynamic', action='store_true', help='Explicit bounded physical moving-obstacle experiment; requires asset-stage preparation and three actual finite-navigation passes')
    parser.add_argument('--dynamic-turn20', action='store_true', help='Prospective dynamic-only0.2rad/s heading alignment profile; keeps0.12flat and90s goal deadline')
    parser.add_argument('--dynamic-dwell', action='store_true', help='Prospective dynamic Turn20 arrival parking only after original measured0.6s dwell; all native safety stops remain')
    args = parser.parse_args()
    if args.dynamic_turn20 and not args.dynamic:parser.error('--dynamic-turn20 requires --dynamic')
    if args.dynamic_dwell and (not args.dynamic or not args.dynamic_turn20 or args.slam_only):parser.error('--dynamic-dwell requires --dynamic --dynamic-turn20 navigation')
    if args.dynamic and args.slam_only:parser.error('--dynamic requires actual navigation')
    result = verify_scope(args.run/'navigation_scope.json') if args.check_only else prepare(args.run,args.slam_only,args.dynamic,args.dynamic_turn20,args.dynamic_dwell)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
