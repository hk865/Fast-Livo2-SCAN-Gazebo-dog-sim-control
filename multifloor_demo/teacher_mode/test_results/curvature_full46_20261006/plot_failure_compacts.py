#!/usr/bin/env python3
"""Plot existing compact, source-hashed records; no original-log reread."""
from pathlib import Path
import gzip
import hashlib
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
DIAG = HERE / '57fa_diagnosis'
RUN = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_165823_closed_loop_cascade_clock_hold_curvature_on_V23_full46_r1_57fa')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    files = [DIAG/'NINTH_TENTH_PID_COMPACT.json.gz', DIAG/'57fa_NATIVE_REGION9_10_COMPACT.json.gz']
    pid, native = [json.loads(gzip.decompress(p.read_bytes())) for p in files]
    rows = [r for r in pid['rows'] if r['waypoint_index'] == 9 and r['math_updated']]
    phys = [r for r in native['rows'] if 185.795 <= r['sim_time'] <= 275.88]
    status = json.loads((RUN/'navigation_status.json').read_text())
    goal = status['current_goal']; center = np.asarray(goal['center'])
    path = RUN/'navigation_trajectories/000750_trajectory_754.npz'
    with np.load(path, allow_pickle=False) as data:
        samples = data['samples'].copy()
    axes = np.asarray(goal['arrival']['axes']); bounds = np.asarray(goal['arrival']['half_extents_m'])
    local = (samples-center) @ axes.T
    inside = np.all(abs(local) <= bounds, axis=1)
    actual_local = axes @ (np.asarray(status['pose'])-center)
    source_geometry = dict(
        goal_id=goal['goal_id'], displayed_region_number=10, goal_center_camera_init=center.tolist(),
        arrival_axes=axes.tolist(), half_extents_m=bounds.tolist(),
        actual_final_camera_init=status['pose'], actual_final_local_axes=actual_local.tolist(),
        actual_final_inside=bool(np.all(abs(actual_local)<=bounds)),
        final_SCAN_array=str(path), final_SCAN_sha256=digest(path),
        final_SCAN_endpoint=samples[-1].tolist(), final_SCAN_endpoint_offset=(samples[-1]-center).tolist(),
        final_SCAN_endpoint_distance_m=float(np.linalg.norm(samples[-1]-center)),
        final_SCAN_endpoint_inside=bool(inside[-1]), sampled_SCAN_points_inside=int(inside.sum()),
        sampled_SCAN_points_total=len(samples),
        claim='The actual selected SCAN curve reaches region10. Planned samples do not prove actual arrival.',
        navigation_ground_truth_used=False, acceptance_thresholds_changed=False)
    (DIAG/'REGION10_SCAN_GEOMETRY.json').write_text(json.dumps(source_geometry, indent=2)+'\n')
    t = np.asarray([r['clock_ns']/1e9 for r in rows])
    native_t = np.asarray([r['sim_time'] for r in phys])
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    ax[0].plot(t, [r['SCAN_heading'] for r in rows], lw=.7, label='Latest selected SCAN finite-arc heading')
    ax[0].plot(t, [r['reference_yaw'] for r in rows], lw=.9, label='Effective heading (gate lock during turn)')
    ax[0].plot(t, [r['yaw'] for r in rows], lw=1, label='Actual SLAM body yaw')
    ax[0].set_ylabel('Yaw [rad]'); ax[0].legend(fontsize=8, loc='upper left')
    ax[1].plot(t, [r['command_after_slew'][2] for r in rows], lw=.8, label='Published body wz request')
    ax[1].plot(native_t, [r['command'][2] for r in phys], lw=.8, label='Actual Teacher body wz input')
    ax[1].plot(native_t, [r['body_ang_vel'][2] for r in phys], lw=.8, label='Native body wz, offline diagnosis')
    ax[1].set_ylabel('Body wz [rad/s]'); ax[1].legend(fontsize=8, loc='upper left')
    ax[2].plot(t, [r['command_after_slew'][0] for r in rows], lw=.8, label='Published vx request')
    ax[2].plot(native_t, [r['body_lin_vel'][0] for r in phys], lw=.8, label='Native body vx, offline diagnosis')
    ax[2].set_ylabel('Body vx [m/s]'); ax[2].legend(fontsize=8, loc='upper left')
    mode = {'drive':0, 'turn':1, 'reference_constraint_hold':2}
    ax[3].scatter(t, [mode.get(r['mode'],math.nan) for r in rows], s=3, label='Fresh math updates only')
    ax[3].set_yticks([0,1,2], ['drive', 'turn', 'constraint hold']); ax[3].set_xlabel('Original ROS simulation time [s]')
    for a in ax:
        a.grid(alpha=.25); a.axvline(275.88,color='red',ls='--',lw=.8)
    fig.suptitle('V23 full46 attempt: region10 reference changes and actual response\nOriginal 90 s region deadline; no acceptance changes')
    out = HERE/'plots_57fa';out.mkdir(exist_ok=True)
    fig.savefig(out/'region10_reference_and_velocity.png', dpi=150)
    fig.savefig(out/'region10_reference_and_velocity.svg')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9,4.8), constrained_layout=True)
    p = np.asarray([r['pose'] for r in rows])
    ax.plot(samples[:,0], samples[:,1], '--',lw=1.2,label='Last actual SCAN curve (planned)')
    ax.plot(p[:,0],p[:,1],lw=1,label='Actual SLAM motion at region10')
    corners = np.asarray([[-bounds[0],-bounds[1],0],[bounds[0],-bounds[1],0],
                          [bounds[0],bounds[1],0],[-bounds[0],bounds[1],0],[-bounds[0],-bounds[1],0]]) @ axes + center
    ax.plot(corners[:,0],corners[:,1],color='green',label='Region10 oriented box horizontal slice')
    ax.scatter(center[0],center[1],s=25,color='green',label='Requested region center')
    ax.scatter(samples[-1,0],samples[-1,1],s=20,color='red',label='SCAN endpoint, 6.01 mm from center')
    ax.set_aspect('equal');ax.set_xlabel('camera_init x [m]');ax.set_ylabel('camera_init y [m]');ax.grid(alpha=.25)
    ax.legend(fontsize=8);ax.set_title('SCAN reaches the requested region; the robot does not\nHorizontal view only; 3D inclusion recomputed separately')
    fig.savefig(out/'region10_SCAN_vs_actual_SLAM.png',dpi=160)
    fig.savefig(out/'region10_SCAN_vs_actual_SLAM.svg');plt.close(fig)
    receipt=dict(scope='Compact plots only; no original log reread, no threshold change',
        inputs={str(f):digest(f) for f in files},geometry_sha256=digest(DIAG/'REGION10_SCAN_GEOMETRY.json'),
        outputs={f.name:digest(f) for f in sorted(out.iterdir()) if f.is_file()},
        plotted_fresh_PID_samples=len(rows),plotted_native_samples=len(phys),
        native_scope='Offline body velocity only; never navigation input',
        missing_values_filled=False,command_signal_interpolated=False)
    (out/'PLOT_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({'plots':str(out),'region10_geometry':source_geometry,'samples':[len(rows),len(phys)]}))


if __name__ == '__main__':
    main()
