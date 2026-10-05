#!/usr/bin/env python3
"""Diagnose the existing fixed parking window without changing its verdict."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args(); run = args.run.resolve()
    summary = json.loads((run / 'summary_pid_navigation_independent.json').read_text())
    original = summary['checks']['fixed_final_zero_command_parking']
    start, end = original['world_window_s']
    protocol = json.loads((run / 'pid_navigation_protocol.json').read_text())
    execution = [json.loads(line) for line in (run / 'telemetry.jsonl').open()]
    selected = [row for row in execution if start - 1e-10 <= row['world_sim_time'] <= end + 1e-10]
    times = [row['world_sim_time'] for row in selected]
    native = []
    with (run / 'actuator.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('kind') == 'physics_step' and start - 1e-10 <= row['t'] <= end + 1e-10:
                native.append(row)
    world_velocities, yaw = [], []
    for row in native:
        w, x, y, z = row['quaternion_wxyz']; bx, by, bz = row['body_lin_vel_com']
        world_velocities.append([(1-2*(y*y+z*z))*bx + 2*(x*y-w*z)*by + 2*(x*z+w*y)*bz,
            2*(x*y+w*z)*bx + (1-2*(x*x+z*z))*by + 2*(y*z-w*x)*bz])
        angle = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        if yaw:
            while angle-yaw[-1] > math.pi: angle -= 2*math.pi
            while angle-yaw[-1] < -math.pi: angle += 2*math.pi
        yaw.append(angle)
    first = native[0]['position']; n = len(native)
    drift = max(math.hypot(row['position'][0]-first[0], row['position'][1]-first[1]) for row in native)
    yaw_drift = max(abs(angle-yaw[0]) for angle in yaw)
    velocity_rms = [math.sqrt(sum(v[k]**2 for v in world_velocities)/n) for k in range(2)]
    yaw_rate_rms = math.sqrt(sum(row['body_ang_vel'][2]**2 for row in native)/n)
    needed = protocol['parking']['evaluation_s'] / .02 - 2
    gaps = [round(times[i]-times[i-1], 9) for i in range(1, len(times))]
    limiter = []
    pattern = re.compile(r"command: \[Joint: '([^']+)', effort: ([-\d.]+)\], limited: \[Joint: '[^']+', effort: ([-\d.]+)\]")
    with (run / 'gazebo.log').open() as stream:
        for line_number, line in enumerate(stream, 1):
            match = pattern.search(line)
            if match:
                limiter.append({'line': line_number, 'joint': match[1], 'requested_effort': float(match[2]),
                    'limited_effort': float(match[3]), 'raw_line': line.rstrip()})
    parking = protocol['parking']
    diagnostic = {
        'schema': 'CHAMP_fixed_parking_sampling_diagnostic/v1', 'run': str(run),
        'source': {'path': str(Path(__file__).resolve()), 'sha256': sha(__file__)},
        'original_immutable_parking_check': original,
        'original_summary_sha256': sha(run / 'summary_pid_navigation_independent.json'),
        'fixed_window_s': [start, end], 'never_shifted_or_selected_a_favorable_window': True,
        'reader': {'all_telemetry_rows': len(execution),
            'all_time_range_s': [execution[0]['world_sim_time'], execution[-1]['world_sim_time']],
            'window_rows': len(selected), 'original_required_minimum_rows': needed,
            'original_count_requirement_met': len(selected) >= needed,
            'window_actual_time_range_s': [times[0], times[-1]],
            'actual_gap_counts_s': dict(Counter(gaps)), 'actual_max_gap_s': max(gaps),
            'requested_and_command_zero_in_all_recorded_rows': all(row['requested'] == [0., 0., 0.] and row['command'] == [0., 0., 0.] for row in selected),
            'execution_health_ready_in_all_recorded_rows': all(row['actual_execution_health']['ready'] is True for row in selected),
            'faults': dict(Counter(row['fault'] for row in selected if row['fault'])),
            'ROS_clock_minus_native_snapshot_world_time_s_range': [min(row['world_sim_time'] - row['state_physics_world_time'] for row in selected), max(row['world_sim_time'] - row['state_physics_world_time'] for row in selected)],
            'missing_records_are_not_filled_or_interpolated': True},
        'native_independent_physical_diagnostic': {'samples': n,
            'actual_time_range_s': [native[0]['t'], native[-1]['t']],
            'actual_max_gap_s': max(native[i]['t']-native[i-1]['t'] for i in range(1, n)),
            'translation_drift_m': drift, 'yaw_drift_rad': yaw_drift,
            'world_planar_component_RMS_mps': velocity_rms, 'body_yaw_rate_RMS_radps': yaw_rate_rms,
            'physical_metric_thresholds_met_only': drift <= parking['maximum_translation_drift_m'] and yaw_drift <= parking['maximum_yaw_drift_rad'] and max(velocity_rms) <= parking['maximum_world_planar_component_rms_mps'] and yaw_rate_rms <= parking['maximum_body_yaw_rate_rms_radps'],
            'does_not_override_reader_coverage_gate': True},
        'CM_limiter_direct_log_receipts': {'source': str(run / 'gazebo.log'), 'source_sha256': sha(run / 'gazebo.log'),
            'throttled_log_records_not_total_saturation_count': len(limiter),
            'max_abs_logged_requested_effort_Nm': max(abs(row['requested_effort']) for row in limiter) if limiter else None,
            'max_abs_logged_limited_effort_Nm': max(abs(row['limited_effort']) for row in limiter) if limiter else None,
            'records': limiter, 'log_wall_timestamp_not_relabelled_as_simTime': True},
        'remaining_scope': 'The 234-row versus 248-row coverage deficit is actual; no end-of-run truncation. Reader source polls a 5ms wall timer and starts new sampled frames after at least20ms advancing receivedclock. Source scheduling/clock coalescing can produce observed25ms gaps, but absent callback-level reader clock history does not isolate its precise cause. Native200Hz physical data is sufficient for a separate diagnostic, not a substitute for the frozen50Hz controller evidence gate.',
        'next_experiment_only': 'Prospective frozen reader cadence/actual-clock callback diagnostics may fix sampling. Never backfill existing rows, relabel timestamps or loosen the current verdict.',
        'original_source_and_summary_modified': False,
    }
    out = HERE / run.name / 'parking_timeline_diagnostic.json'
    with out.open('x') as stream: json.dump(diagnostic, stream, indent=2); stream.write('\n')
    print(json.dumps({'receipt': str(out), 'sha256': sha(out), 'actual_reader_rows': len(selected),
        'required_rows': needed, 'gaps': dict(Counter(gaps)), 'native_rows': n,
        'native_drift_m': drift, 'original_check_preserved': original['status'],
        'CM_limiter_records': len(limiter), 'CM_limiter_max_requested_Nm': diagnostic['CM_limiter_direct_log_receipts']['max_abs_logged_requested_effort_Nm']}))


if __name__ == '__main__': main()
