#!/usr/bin/env python3
"""Plot actual dynamic-obstacle commands/physics and original SLAM route, offline only."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def compact_native(path):
    # Plot only the four actual fields needed; avoid holding large contact/joint
    # records in memory alongside the ROS and actor logs.
    result = []
    with Path(path).open() as stream:
        for line in stream:
            record = json.loads(line)
            if record.get('kind') == 'physics_step':
                result.append([record['t']-.005, record['body_lin_vel_com'][0], record['body_ang_vel'][2],
                    float(np.linalg.norm(record['body_lin_vel_com'][:2]))])
    return np.asarray(result)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def smooth(values, width=20):
    # Trailing 100ms average is only an extra visualization; raw points remain.
    result = np.convolve(values, np.ones(width), mode='full')[:len(values)]
    return result / np.minimum(np.arange(1, len(values)+1), width)


def plot(run):
    run = Path(run).resolve()
    receipt = run / 'summary_dynamic_obstacle_independent.json'
    original = receipt.read_bytes()
    summary = json.loads(original)
    telemetry = rows(run / 'telemetry.jsonl')
    native = compact_native(run / 'actuator.jsonl')
    slam = rows(run / 'navigation_slam_poses.jsonl')
    box = [r for r in rows(run / 'obstacle_actual_pose.jsonl') if r.get('status') == 'actual_observed']
    requested = np.asarray([r['requested'] for r in telemetry])
    commands = np.asarray([r['command'] for r in telemetry])
    pt = np.asarray([r['world_sim_time'] for r in telemetry])
    nt, bodyvx, wz, planar = native.T
    bt = np.asarray([r['stamp_ns'] * 1e-9 for r in box])
    by = np.asarray([r['position'][1] for r in box])
    phases = summary['checks']['actual_box_entry_block_and_withdrawal']['phase_times_s']
    park = summary['checks']['actual_continuous_teacher_parking_while_box_blocked'].get('world_window_s')
    park_status = summary['checks']['actual_continuous_teacher_parking_while_box_blocked']['status']
    park_color = 'green' if park_status == 'passed' else '#dc2626' if park_status == 'failed' else 'gray'
    clear = summary['checks']['actual_registered_cloud_corridor_clear_after_withdrawal'].get('first_continuous_clear_confirmation_world_s')
    recovery = summary['checks']['teacher_actual_motion_resumed_on_new_scan'].get('first_accepted_motion_world_s')
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex='col', constrained_layout=True)
    for column, window in enumerate([(0, float(pt[-1])), (10, 42)]):
        av, aw, ab = axes[:, column]
        av.plot(pt, requested[:, 0], color='gray', linewidth=.8, label='Original requested body vx (50Hz)')
        av.plot(pt, commands[:, 0], color='#2563eb', linewidth=1.1, label='Actual CPU actor vx input (50Hz)')
        av.plot(nt, bodyvx, color='#dc2626', alpha=.22, linewidth=.5, label='Actual body COM vx raw (200Hz)')
        av.plot(nt, smooth(bodyvx), color='#b91c1c', linewidth=1., label='100ms trailing visual mean')
        av.plot(nt, planar, color='#7c3aed', alpha=.45, linewidth=.55, label='Actual raw planar speed (200Hz)')
        av.axhline(.03, color='#7c3aed', linestyle=':', linewidth=.7, label='Fixed parking planar limit0.03m/s')
        av.set_ylabel('Body forward velocity (m/s)')
        aw.plot(pt, commands[:, 2], color='#2563eb', linewidth=1., label='Actor yaw-rate input (50Hz)')
        aw.plot(nt, wz, color='#dc2626', alpha=.25, linewidth=.5, label='Actual body yaw rate raw (200Hz)')
        aw.plot(nt, smooth(wz), color='#b91c1c', linewidth=1.)
        aw.set_ylabel('Body yaw rate (rad/s)')
        ab.plot(bt, by, color='#9333ea', linewidth=1.2, label='Actual existing box y, observed Pose_V')
        ab.axhline(-.7, color='black', linestyle=':', linewidth=.8, label='Blocking position y')
        ab.axhline(-1.6, color='gray', linestyle=':', linewidth=.8, label='Initial withdrawn position y')
        ab.set_ylabel('Actual box world y (m)')
        ab.set_xlabel('Actual world simulation time (s)')
        for axis in axes[:, column]:
            axis.set_xlim(*window)
            axis.grid(alpha=.18)
            if 'blocking' in phases and 'leaving' in phases:
                axis.axvspan(phases['blocking'], phases['leaving'], color='orange', alpha=.12)
            if park:
                axis.axvspan(*park, color=park_color, alpha=.15)
            if clear is not None:
                axis.axvline(clear, color='#059669', linestyle='--', linewidth=.8)
            if recovery is not None:
                axis.axvline(recovery, color='#2563eb', linestyle='--', linewidth=.8)
    for axis in axes[:, 0]:
        axis.legend(fontsize=7, loc='upper right')
    axes[0, 0].set_title('Complete actual run')
    axes[0, 1].set_title('Actual entry / measured parking / clear / recovery')
    fig.suptitle(run.name + '\nOrange: actual box blocking; fixed 5s parking band: ' + park_status + '; '
                 'green dashed: observed cloud-clear confirmation; blue dashed: accepted motion recovery', fontsize=10)
    figure = run / 'dynamic_obstacle_actual_stop_restore.png'
    fig.savefig(figure, dpi=160)
    plt.close(fig)
    request = json.loads((run / 'navigation_request.json').read_text())
    xy = np.asarray([r['position'][:2] for r in slam])
    stamp = np.asarray([r['stamp_ns'] for r in slam], dtype=np.int64)
    fig, axis = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    axis.plot(xy[:, 0], xy[:, 1], color='#2563eb', linewidth=.8, label='Original actual SLAM body pose')
    for goal in request['goals']:
        center = goal['center'][:2]
        axis.add_patch(Circle(center, goal['arrival']['radius_m'], facecolor='none', edgecolor='gray', linestyle='--'))
        axis.add_patch(Circle(center, goal['arrival']['control_band']['radius_m'], facecolor='#16a34a', edgecolor='#16a34a', alpha=.1))
        axis.scatter(*center, marker='+', color='black')
        axis.annotate(goal['goal_id'], center, xytext=(5, 8), textcoords='offset points', fontsize=8)
    for arrival in summary['checks']['original_raw_slam_two_regions_and_final_stop'].get('arrival_evidence', []):
        claim = arrival.get('claimed_receipt')
        if not claim:
            continue
        mask = (stamp >= claim['start_stamp_ns']) & (stamp <= claim['stamp_ns'])
        axis.scatter(xy[mask, 0], xy[mask, 1], color='#15803d', s=13, label='Original raw SLAM dwell samples')
    axis.set_xlabel('Actual SLAM camera_init x (m)')
    axis.set_ylabel('Actual SLAM camera_init y (m)')
    axis.set_aspect('equal')
    axis.grid(alpha=.18)
    axis.set_title('Actual registered goal discs / raw SLAM route\nGoal-center separation 1m; acceptance uses 0.17m control regions')
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), fontsize=8, loc='lower center')
    xyfigure = run / 'dynamic_obstacle_actual_slam_xy.png'
    fig.savefig(xyfigure, dpi=160)
    plt.close(fig)
    archive = run / 'dynamic_obstacle_plot_arrays.npz'
    np.savez_compressed(archive, policy_world_time=pt, native_physics_world_time=nt,
        requested=requested, actor_command=commands, body_com_vx=bodyvx, body_wz=wz, body_com_planar_speed=planar,
        actual_box_world_time=bt, actual_box_y=by, original_slam_stamp_ns=stamp, original_slam_xy=xy)
    (run / 'dynamic_obstacle_plot_manifest.json').write_text(json.dumps({
        'schema': 1, 'input_sha256': {name: sha(run / name) for name in [
            'summary_dynamic_obstacle_independent.json', 'telemetry.jsonl', 'actuator.jsonl',
            'navigation_slam_poses.jsonl', 'obstacle_actual_pose.jsonl', 'navigation_request.json']},
        'output_sha256': {p.name: sha(p) for p in [figure, xyfigure, archive]},
        'analyzer_sha256': sha(__file__), 'original_receipt_bytes_preserved': receipt.read_bytes() == original,
        'physical_state_time': 'Actual native PreUpdate t minus0.005s',
        'smoothing': 'Extra trailing100ms visual mean only; raw200Hz data retained, never used for acceptance',
        'truth_scope': 'Actual box pose displayed as physical fixture diagnostic only; route plot uses original SLAM/goals without truth',
    }, indent=2) + '\n')
    print(json.dumps({'figure': str(figure), 'route': str(xyfigure), 'original_preserved': receipt.read_bytes() == original}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    plot(parser.parse_args().run)
