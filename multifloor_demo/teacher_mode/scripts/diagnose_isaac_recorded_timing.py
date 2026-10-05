#!/usr/bin/env python3
"""Preserve strict parity failures and isolate recorded sensor timing effects."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('run', type=Path)
args = p.parse_args()
out = args.run.resolve()
arrays = np.load(out / 'recorded_observation_parity.npz')
observed = arrays['recorded_observation247']
rebuilt = arrays['reconstructed_observation247']
reset = arrays['reset_frame_mask']
teacher = arrays['teacher_frame_mask']
labels = arrays['case_labels']
errors = abs(observed - rebuilt)
actor_error = abs(arrays['actor_action_from_recorded_obs'] - arrays['recorded_action'])
sections = {'linear_velocity': [0, 3], 'angular_velocity': [3, 6],
            'projected_gravity': [6, 9], 'command': [9, 12],
            'joint_position': [12, 24], 'joint_velocity': [24, 36],
            'applied_torque': [36, 48], 'last_raw_action': [48, 60],
            'height_scan': [60, 247]}
rows = []
for name in dict.fromkeys(labels):
    indices = np.flatnonzero(labels == name)
    trace = json.loads((out / f'{name}_trace.json').read_text())
    raw_height = np.asarray([r['height_scan_raw'] for r in trace])
    z = np.asarray([r['position'][2] for r in trace])
    # Flat fixture hits are z=0 to float ray precision; this recovers the
    # scanner's sampled reference z independently from root snapshot z.
    scanner_z = raw_height[:, 0] + .5
    bad_height = np.flatnonzero(errors[indices, 60:].max(1) > 1e-5)
    previous_z_errors = [float(abs(scanner_z[i] - z[i-1])) for i in bad_height if i]
    rows.append({'name': str(name), 'frames': len(indices),
                 'reset_gravity_abs_error': float(errors[indices[0], 6:9].max()),
                 'nonreset_gravity_abs_error': float(errors[indices[1:], 6:9].max()),
                 'height_snapshot_mismatch_frames': bad_height.tolist(),
                 'height_snapshot_mismatch_count': len(bad_height),
                 'height_raw_clip_recorded_observation_error': float(abs(np.clip(raw_height,-1,1) - observed[indices,60:]).max()),
                 'sampled_height_reference_z_vs_snapshot_max_m': float(abs(scanner_z-z).max()),
                 'mismatched_scan_vs_previous_snapshot_z_max_m': max(previous_z_errors, default=0),
                 'recorded_observation_actor_abs_error': float(actor_error[indices][teacher[indices]].max())})
proprio_indices = np.r_[0:6, 9:60]
result = {
    'status': 'completed', 'frames': len(observed),
    'teacher_frames': int(teacher.sum()), 'pd_init_frames_excluded': int((~teacher).sum()),
    'recorded_observation_actor_abs_error': float(actor_error[teacher].max()),
    'recorded_observation_actor_passed_2e_minus5': bool(actor_error[teacher].max() < 2e-5),
    'fresh_snapshot_all247_parity_passed': bool(errors.max() < 1e-5),
    'strict_failure_preserved_at': str(out / 'matched_analysis.json'),
    'section_abs_errors': {name: float(errors[:, a:b].max()) for name,(a,b) in sections.items()},
    'proprioception_except_gravity_max_error': float(errors[:, proprio_indices].max()),
    'nonreset_gravity_max_error': float(errors[~reset, 6:9].max()),
    'recorded_raw_height_clip_equals_scan': all(r['height_raw_clip_recorded_observation_error'] == 0 for r in rows),
    'timing_interpretation': {
        'reset': 'All9 first snapshots have identity reset quaternion but cached projected_gravity from preceding simulation state. Source root pose write invalidates pose/body/Jacobian caches but not projected_gravity; derived gravity recomputes only when sim timestamp advances. All reset differences occur during excluded initialization PD.',
        'height': '79 switch frames have sensor z matching previous snapshot z within ray float precision. Sensor timestamp accumulation uses float32 dt=.005, period=.02 and tolerance1e-6; around t16s its elapsed comparison begins skipping alternating policy updates. Exact timestep cause is inferred from source plus the measured one-frame z alignment; timestamps themselves were not recorded.',
        'scope': 'Analytical Gazebo geometry is queried freshly each policy call. Recorded Isaac scans are periodic sampled data. Replacing fields with recorded samples for diagnosis would not prove fresh snapshot parity or change physical run results.'
    },
    'source_paths': {
        'projected_gravity_cache': '/home/hyh001/IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/articulation/articulation_data.py:995',
        'root_pose_write_invalidation': '/home/hyh001/IsaacLab/source/isaaclab_physx/isaaclab_physx/assets/articulation/articulation.py:445',
        'sensor_float_timestamp_update': '/home/hyh001/IsaacLab/source/isaaclab/isaaclab/sensors/kernels.py:10',
        'sensor_update': '/home/hyh001/IsaacLab/source/isaaclab/isaaclab/sensors/sensor_base.py:194'
    },
    'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'rows': rows,
}
(out / 'recorded_timing_diagnostic.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','timing_interpretation','source_paths')}, indent=2))
