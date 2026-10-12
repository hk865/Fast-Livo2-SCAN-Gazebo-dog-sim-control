#!/usr/bin/env python3
"""Post-acceptance long zero-command diagnostic; never adds a retrospective gate."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    summary_file = run / 'summary_pid_navigation_independent.json'
    before = sha(summary_file)
    summary = json.loads(summary_file.read_text())
    parking = summary['checks']['fixed_final_zero_command_parking']
    if parking.get('status') != 'passed':
        raise ValueError('This diagnostic requires an actual passed fixed parking window')
    begin = parking['world_window_s'][1]
    helper_path = Path(__file__).with_name('analyze_pid_navigation_v3.py')
    spec = importlib.util.spec_from_file_location('post_parking_helper', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    execution = [json.loads(x) for x in (run / 'telemetry.jsonl').open()]
    em = [r for r in execution if r['world_sim_time'] >= begin - 1e-10]
    with np.load(run / 'pid_navigation_independent_arrays.npz') as z:
        time = z['native_world_time_s']
        mask = time >= begin - 1e-10
        t, pos, vel = time[mask], z['native_position'][mask], z['native_world_COM_velocity'][mask]
        quat = z['native_quaternion_wxyz'][mask]
        yaw = np.unwrap(helper.rpy(quat)[:, 2])
        drift = np.linalg.norm(pos[:, :2] - pos[0, :2], axis=1)
        yaw_drift = np.abs(yaw - yaw[0])
        speed = np.linalg.norm(vel[:, :2], axis=1)
        rate = np.asarray([r['body_ang_vel'][2] for r in em])
        ix, iy, iv, iw = int(np.argmax(drift)), int(np.argmax(yaw_drift)), int(np.argmax(speed)), int(np.argmax(abs(rate)))
        result = {'schema': 'post_fixed_parking_long_diagnostic/v1', 'run': str(run),
            'scope': 'Additional long-standing observation after the frozen5s window; NOT an additional acceptance gate',
            'original_fixed_parking_status': parking['status'], 'original_fixed_parking': parking,
            'original_summary_sha256': before, 'original_summary_bytes_unchanged': sha(summary_file) == before,
            'actual_native_interval_world_s': [float(t[0]), float(t[-1])], 'native200hz_samples': len(t),
            'native_maximum_gap_s': float(np.diff(t).max()),
            'zero_command_receipts50hz': {'samples': len(em),
                'maximum_requested_abs': float(max(abs(v) for r in em for v in r['requested'])),
                'maximum_executed_command_abs': float(max(abs(v) for r in em for v in r['command'])),
                'continuous_actor_inferred': all(r.get('actor_inferred_this_frame') is True for r in em)},
            'physical_native200hz': {'maximum_XY_drift_from_postwindow_begin_m': float(drift[ix]),
                'maximum_XY_drift_time_world_s': float(t[ix]),
                'final_XY_delta_m': (pos[-1, :2] - pos[0, :2]).tolist(),
                'maximum_absolute_unwrapped_yaw_drift_rad': float(yaw_drift[iy]),
                'maximum_yaw_drift_time_world_s': float(t[iy]),
                'final_unwrapped_yaw_delta_rad': float(yaw[-1] - yaw[0]),
                'maximum_planar_COM_speed_mps': float(speed[iv]),
                'maximum_planar_speed_time_world_s': float(t[iv]),
                'world_planar_component_velocity_RMS_mps': np.sqrt(np.mean(vel[:, :2] ** 2, axis=0)).tolist()},
            'actual_body_gyro50hz': {'maximum_abs_yaw_rate_radps': float(abs(rate[iw])),
                'peak_world_time_s': em[iw]['world_sim_time'],
                'yaw_rate_RMS_radps': float(np.sqrt(np.mean(rate ** 2))),
                'scope': 'Actual native angular velocity received by50Hz worker; not a200Hz maximum or differentiated orientation'},
            'acceptance_reclassified': False, 'long_hold_capability_global_claim': 'unverified',
            'input_hashes': {n: sha(run / n) for n in ['summary_pid_navigation_independent.json',
                'pid_navigation_independent_arrays.npz', 'telemetry.jsonl']},
            'analyzer_sha256': sha(__file__), 'frozen_helper_sha256': sha(helper_path)}
    output = run / 'pid_post_parking_diagnostic.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(output), 'native': result['physical_native200hz'], 'gyro50hz': result['actual_body_gyro50hz']}))


if __name__ == '__main__':
    main()
