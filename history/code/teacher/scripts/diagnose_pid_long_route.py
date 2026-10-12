#!/usr/bin/env python3
"""Read-only long-route diagnosis. Preserves every frozen acceptance receipt."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()

def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]

def intervals(time, mask):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    cuts = np.flatnonzero(np.diff(indices) != 1) + 1
    return [{'begin_world_s': float(time[g[0]]), 'end_world_s': float(time[g[-1]]),
             'native_samples': int(len(g))} for g in np.split(indices, cuts)]

def diagnose(run):
    sf = run / 'summary_pid_navigation_independent.json'
    before = sf.read_bytes()
    s = json.loads(before)
    helper = Path(__file__).with_name('analyze_pid_navigation_v3.py')
    spec = importlib.util.spec_from_file_location('long_route_helpers', helper)
    h = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(h)
    with np.load(run / 'pid_navigation_independent_arrays.npz') as z:
        a = {k: z[k] for k in z.files}
    pid = rows(run / 'navigation_pid_history.jsonl')
    ex = rows(run / 'telemetry.jsonl')
    nt = a['native_world_time_s']
    pt = np.asarray([p['compute_ros_clock_ns'] / 1e9 for p in pid])
    et = a['execution_world_time_s']
    pi, pv = h.causal_index(pt, nt, .3)
    ei, ev = h.causal_index(et, nt, .020001)
    rotation = h.rotation(a['native_quaternion_wxyz'])
    rpy = h.rpy(a['native_quaternion_wxyz'])
    drive = a['drive_mask']
    errors = a['drive_heading_error_rad']
    k = int(np.argmax(np.where(drive, errors, -np.inf)))
    kr = int(np.argmax(a['lateral_error_m']))
    anchor = json.loads((run / 'navigation_anchor.json').read_text())
    astamp = anchor['pose_stamp_ns'] / 1e9
    ai = int(np.searchsorted(nt, astamp, side='right') - 1)
    neighbourhood = (nt >= nt[k] - .5) & (nt <= nt[k] + .5)
    projected_world_vx = a['native_world_COM_velocity'][:, 0]
    worldcmd = np.einsum('nij,nj->ni', rotation, a['execution_command'][ei])
    def sample(i):
        p = pid[int(pi[i])]
        e = ex[int(ei[i])]
        return {'world_time_s': float(nt[i]), 'native_position': a['native_position'][i].tolist(),
                'native_rpy': rpy[i].tolist(), 'physical_heading_error_rad': float(errors[i]),
                'finite_route_distance_m': float(a['lateral_error_m'][i]),
                'native_world_COM_velocity': a['native_world_COM_velocity'][i].tolist(),
                'PID_age_s': float(nt[i] - pt[pi[i]]), 'PID': p,
                'execution_state_time_s': e.get('state_physics_world_time'),
                'execution_requested': e['requested'], 'execution_command': e['command'],
                'execution_command_expired': e.get('command_expired'),
                'execution_envelope': e.get('navigation_envelope')}
    callbacks = rows(run / 'navigation_cloud_callbacks.jsonl')
    clock = np.asarray([p['received_ros_clock_ns'] / 1e9 for p in callbacks])
    gate = rows(run / 'navigation_sensor_gate_history.jsonl')
    j = {'schema': 'pid_long_route_actual_diagnosis/v1', 'run': str(run),
         'scope': 'Offline physical and original SLAM/PID diagnosis only; no gate, reference, runtime or receipt changed.',
         'acceptance_reclassified': False, 'original_status': s['status'],
         'original_failed_checks': [key for key, value in s['checks'].items() if value['status'] == 'failed'],
         'physical_heading_bound_rad': .35,
         'heading_exceedance_segments': intervals(nt, drive & (errors > .35)),
         'heading_exceedance_native_samples': int(np.sum(drive & (errors > .35))),
         'heading_peak': sample(k), 'maximum_finite_route_distance': sample(kr),
         'peak_neighbourhood_mean_world_COM_vx': float(np.mean(projected_world_vx[neighbourhood])),
         'peak_neighbourhood_mean_executed_world_vx': float(np.mean(worldcmd[neighbourhood, 0])),
         'immutable_anchor': anchor, 'causal_anchor_native_world_time_s': float(nt[ai]),
         'causal_anchor_native_position': a['native_position'][ai].tolist(),
         'causal_anchor_native_yaw_rad': float(rpy[ai, 2]),
         'causal_anchor_state_gap_s': float(astamp - nt[ai]),
         'actual_cloud_callbacks': len(callbacks),
         'actual_cloud_receive_clock_gap_max_s': float(np.max(np.diff(clock))) if len(clock) > 1 else None,
         'actual_cloud_callback_first': callbacks[0] if callbacks else None,
         'actual_cloud_callback_last': callbacks[-1] if callbacks else None,
         'callback_clock_fields_detected': sorted(callbacks[0]) if callbacks else [],
         'sensor_gate_rows': len(gate),
         'physical_truth_navigation': False,
         'causal_samples_valid_at_heading_peak': bool(pv[k] and ev[k]),
         'input_sha256': {p.name: sha(p) for p in [sf, run / 'pid_navigation_independent_arrays.npz',
             run / 'navigation_pid_history.jsonl', run / 'telemetry.jsonl', run / 'navigation_anchor.json',
             run / 'navigation_cloud_callbacks.jsonl', run / 'navigation_sensor_gate_history.jsonl']},
         'analysis_script_sha256': sha(__file__), 'frozen_analyzer_sha256': sha(helper)}
    if sf.read_bytes() != before:
        raise RuntimeError('Original receipt changed during read-only diagnosis')
    out = run / 'pid_long_route_diagnostic.json'
    out.write_text(json.dumps(j, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(out), 'heading_peak_world_s': float(nt[k]),
                      'heading_peak_rad': float(errors[k]), 'exceedance_samples': j['heading_exceedance_native_samples']}))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    diagnose(parser.parse_args().run.resolve())
