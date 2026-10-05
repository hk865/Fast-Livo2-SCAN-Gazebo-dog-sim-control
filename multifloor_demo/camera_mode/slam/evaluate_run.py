#!/usr/bin/env python3
"""Camera-carrier acceptance using the original strict region/RGB evaluator.

Original 46 receipt windows, measured dwell, bounded same-time truth and the
single initialization SE(3) are unchanged. This is camera sensor-SLAM evidence,
not quadruped contact, actuator, tilt stability or real-hardware certification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

from _shared import CONTRACT, ROOT, load_shared, sha256

SHARED = load_shared('region_and_rgb_evaluator', 'scripts/evaluate_run.py')


def preregistered_camera_regions(directory, scenario):
    try:
        manifest = json.loads((directory/'source_manifest.json').read_text())
        digest = (directory/'source_manifest.sha256').read_text().strip()
        if sha256(directory/'source_manifest.json') != digest:
            raise ValueError('camera source manifest SHA sidecar differs')
        files = manifest.get('files')
        if manifest.get('mode') != 'camera' or not isinstance(files, list):
            raise ValueError('camera source manifest requires mode and files list')
        paths = {row['path']: row['sha256'] for row in files}
        if len(paths) != len(files):
            raise ValueError('camera source manifest has duplicate relative paths')
        relative = 'camera_mode/simulation/scenario.json'
        encoded = (directory/'sources'/relative).read_bytes()
        actual = hashlib.sha256(encoded).hexdigest()
        if paths.get(relative) != actual:
            raise ValueError('camera scenario SHA differs from prelaunch source manifest')
        declared = json.loads(encoded)
        SHARED.validate_route_regions(declared)
        if declared != scenario or declared.get('camera_mode') is not True \
                or declared.get('simulation', {}).get('kind') != 'camera_rig':
            raise ValueError('run camera declaration differs from its prelaunch source')
        counts = {name: len(declared.get(name, [])) for name in SHARED.ROUTE_KEYS.values()}
        if list(counts.values()) != [18, 14, 14]:
            raise ValueError('full camera evaluation requires the original 18+14+14 route')
        return dict(passed=True, source='prelaunch camera_mode/simulation/scenario.json',
                    sha256=actual, original_route_counts=counts,
                    center_reference='initial rig center; floor levels plus initial 0.75 m sensor-carrier height')
    except (OSError, ValueError, TypeError, KeyError) as error:
        return dict(passed=False, error=str(error))


# This is a private, independently loaded evaluator module. No production
# module/function/file is changed. Only the archive location and camera runtime
# evidence gate differ; region maths and RGB/official-PCD checks are reused.
SHARED.preregistered_region_evidence = preregistered_camera_regions
SHARED.full_control_runtime_evidence_check = lambda directory: None


def read_json(directory, name):
    try:
        document = json.loads((directory/name).read_text())
        return document if isinstance(document, dict) else {}
    except (OSError, ValueError):
        return {}


def sensor_header_evidence(path):
    """Audit integer acquisition stamps without repairing observation gaps."""
    streams = {}
    errors = []
    try:
        with path.open() as f:
            for number, line in enumerate(f, 1):
                if not line.strip(): continue
                row = json.loads(line)
                name = row.get('source')
                if name not in ('imu', 'lidar', 'camera'): continue
                stamp = row.get('stamp_ns')
                if type(stamp) is not int or stamp < 0:
                    errors.append(f'line {number}: no original integer acquisition stamp')
                    continue
                data = streams.setdefault(name, dict(samples=0, first_stamp_ns=stamp,
                    last_stamp_ns=None, nonincreasing=0, gaps=[], nominal_period_ns=1_000_000 if name=='imu' else 100_000_000))
                previous = data['last_stamp_ns']
                if previous is not None:
                    delta = stamp-previous
                    if delta <= 0: data['nonincreasing'] += 1
                    if delta != data['nominal_period_ns']:
                        data['gaps'].append(dict(before_ns=previous, after_ns=stamp, delta_ns=delta))
                data['last_stamp_ns'] = stamp
                data['samples'] += 1
    except (OSError, ValueError, TypeError) as error:
        errors.append(str(error))
    valid = not errors and set(streams) == {'imu', 'lidar', 'camera'} \
        and all(s['samples'] >= 2 and s['nonincreasing'] == 0 for s in streams.values())
    return dict(passed=valid, streams=streams, errors=errors,
                complete_nominal_sampling=valid and not any(s['gaps'] for s in streams.values()),
                scope='Actual observer acquisition headers; gaps remain diagnostics and are never filled from another source.')


def camera_runtime_artifact_evidence(runtime, declared):
    artifacts = runtime.get('artifacts')
    artifacts = artifacts if isinstance(artifacts, list) else []
    keys = ('slam/ros2_ws/install/fast_livo2_core/lib/libfast_livo2_core.so',
            'slam/ros2_ws/install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping')
    rows = {str(Path(a['path']).resolve()): a for a in artifacts
            if isinstance(a, dict) and isinstance(a.get('path'), str)}
    selected = {key: rows.get(str((ROOT/key).resolve()), {}) for key in keys}
    passed = runtime.get('mode') == 'camera' and all(
        selected[key].get('sha256') == declared['isolated_binaries'][key] for key in keys)
    return dict(passed=passed, selected_artifacts=selected,
                scope='Selected runtime manifest path and SHA; actual loaded-process evidence belongs to the parent runtime capture.')


def evaluate(directory):
    directory = Path(directory).resolve()
    report = SHARED.evaluate(directory)
    # Remove requirements that prove quadruped mechanics. These requirements
    # are inapplicable to this separately declared mode, not waived for old runs.
    excluded = {'physical_simulation_scope', 'body_contact_monitor_available',
                'no_body_contact_events', 'body_tilt_within_limit', 'full_control_runtime_evidence'}
    report['checks'] = [c for c in report['checks'] if c['name'] not in excluded]

    def check(name, passed, detail):
        report['checks'].append(dict(name=name, passed=bool(passed), detail=detail))

    scenario = read_json(directory, 'scenario.json')
    check('camera_rig_mode_declaration', scenario.get('camera_mode') is True
          and scenario.get('simulation', {}).get('kind') == 'camera_rig', scenario.get('simulation'))
    for phase, state in (('sensors', 'sensors_ready'), ('slam', 'ready')):
        gate = read_json(directory, 'startup_'+phase+'.json')
        check('camera_'+phase+'_startup_gate', gate.get('passed') is True and gate.get('mode') == 'camera'
              and gate.get('phase') == phase and gate.get('state') == state
              and gate.get('ground_truth_used') is False and gate.get('quadruped_prerequisites') == [], gate)
    declared = json.loads(CONTRACT.read_text())
    selected = read_json(directory, 'camera_slam_contract.json')
    check('original_isolated_sensor_SLAM_contract', selected.get('mode') == 'camera'
          and selected.get('sources') == declared['sources']
          and selected.get('isolated_binaries') == declared['isolated_binaries']
          and selected.get('ground_truth_used') is False
          and selected.get('self_echo_filter') == 'disabled: no Go2 body/leg mask', selected)
    runtime = read_json(directory, 'runtime_manifest.json')
    runtime_evidence = camera_runtime_artifact_evidence(runtime, declared)
    check('camera_selected_SLAM_runtime', runtime_evidence['passed'], runtime_evidence)
    cleanup = read_json(directory, 'shutdown_verification.json')
    check('camera_owned_process_cleanup', cleanup.get('mode') == 'camera' and cleanup.get('owned_clean') is True, cleanup)
    header = sensor_header_evidence(directory/'sensor_audit.jsonl')
    check('native_sensor_integer_timestamps_valid', header['passed'], header)
    camera_nav = []
    try:
        camera_nav = [json.loads(x)['status'] for x in (directory/'navigation_audit.jsonl').read_text().splitlines() if x.strip()]
    except (OSError, ValueError, TypeError, KeyError):
        pass
    check('camera_navigation_receipts_mode', bool(camera_nav)
          and all(isinstance(s, dict) and s.get('mode') == 'camera' for s in camera_nav),
          dict(status_count=len(camera_nav),
               noncamera_statuses=sum(not isinstance(s, dict) or s.get('mode') != 'camera' for s in camera_nav)))
    report.update(schema_version=2, mode='camera', evaluated_at=time.time(),
        passed=all(c['passed'] for c in report['checks']),
        failed_checks=[c['name'] for c in report['checks'] if not c['passed']],
        sensor_headers=header, excluded_quadruped_checks=sorted(excluded),
        reused_evaluator_source_sha256=declared['sources']['scripts/evaluate_run.py'])
    report['scope'] = dict(
        execution='Gazebo camera/IMU/LiDAR carrier with force-based velocity servo; no quadruped or RL locomotion evidence',
        avoidance='Observed LiDAR stop/wait/fresh checked path and actual moving-obstacle poses; no fabricated bypass',
        trajectory_errors='One frozen initialization SE(3), same-time raw-inner/GT-outer original receipt windows; GT only evaluation',
        color_evidence='This run actual RGB images and measured SLAM cloud, immutable map binary and official saved RGB PCD',
        observation_gaps='All recorded sensor acquisition gaps remain reported; valid timestamps do not imply complete 1 kHz acquisition',
        not_certified=['Go2 contact or gait', 'joint effort or applied torque', 'tilt stability of a quadruped', 'real hardware', 'RL policy'])
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = evaluate(args.run_dir)
    output = args.output or args.run_dir/'acceptance.json'
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(passed=result['passed'], failed_checks=result['failed_checks'], output=str(output)), ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
