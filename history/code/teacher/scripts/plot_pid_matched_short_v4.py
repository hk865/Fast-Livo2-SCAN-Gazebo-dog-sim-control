#!/usr/bin/env python3
"""Scientific comparison of all six actual V4 short trials, no synthetic route."""
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'test_results/pid_matched_short_v4_20261004'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    data = json.loads((OUT / 'comparison.json').read_text())
    colors = ['#205bc2', '#d16b00', '#31915e']
    fig, ax = plt.subplots(3, 2, figsize=(15, 13), constrained_layout=True)
    provenance = {}
    samples = []
    for col, kind in enumerate(['teacher', 'champ']):
        cases = [c for c in data['cases'] if c['controller'] == kind]
        for i, case in enumerate(cases):
            run = ROOT / 'runs' / case['run']
            array_file = run / 'pid_navigation_independent_arrays.npz'
            request_file = run / 'navigation_request.json'
            with np.load(array_file) as z:
                pos = z['native_position']
                slam = z['SLAM_position']
                vertices = z['planned_world_polyline']
            label = f"Rep{i+1}: {case['status'].upper()}"
            ax[0, col].plot(pos[:, 0], pos[:, 1], color=colors[i], linewidth=.9, label=label)
            ax[1, col].plot(slam[:, 0], slam[:, 1], color=colors[i], linewidth=.9, label=label)
            request = json.loads(request_file.read_text())
            for goal in request['goals']:
                x, y = goal['center'][:2]
                ax[1, col].add_patch(Circle((x, y), .17, fill=False, linestyle='--', color=colors[i], linewidth=.7, alpha=.8))
            provenance[str(array_file)] = sha(array_file)
            provenance[str(request_file)] = sha(request_file)
            samples.append((kind, i, case))
        ax[0, col].plot(vertices[:, 0], vertices[:, 1], 'k--', linewidth=1, label='Original fixture reference')
        ax[0, col].set_title(f"{kind.upper()}: actual body route, world frame (offline only)")
        ax[0, col].set_xlabel('Gazebo world x (m)')
        ax[0, col].set_ylabel('Gazebo world y (m)')
        ax[1, col].set_title(f"{kind.upper()}: actual raw SLAM route and0.17m goal discs")
        ax[1, col].set_xlabel('camera_init x (m)')
        ax[1, col].set_ylabel('camera_init y (m)')
        for row in [0, 1]:
            ax[row, col].set_aspect('equal', adjustable='datalim')
            ax[row, col].grid(alpha=.2)
            ax[row, col].legend(fontsize=8)
    x = np.arange(len(samples))
    labels = [f'{kind[:1].upper()}{i+1}' for kind, i, _ in samples]
    colors_bars = [colors[i] for _, i, _ in samples]
    maximum = [c['physical_route']['maximum_lateral_error_m'] for _, _, c in samples]
    rms = [c['physical_route']['active_lateral_RMS_m'] for _, _, c in samples]
    ax[2, 0].bar(x - .17, maximum, .32, color=colors_bars, label='Maximum finite polyline distance')
    ax[2, 0].bar(x + .17, rms, .32, color=colors_bars, alpha=.45, label='Active route RMS')
    ax[2, 0].axhline(.45, color='#a11', linestyle='--', label='Max limit0.45m')
    ax[2, 0].axhline(.2, color='#333', linestyle=':', label='RMS limit0.20m')
    ax[2, 0].set_title('Original physical route gates, including endpoint overrun')
    ax[2, 0].set_ylabel('m')
    heading = [c['physical_route']['maximum_drive_heading_error_rad'] for _, _, c in samples]
    posture = [max(c['physical_safety']['max_roll_rad'], c['physical_safety']['max_pitch_rad']) for _, _, c in samples]
    ax[2, 1].bar(x - .17, heading, .32, color=colors_bars, label='Drive heading peak')
    ax[2, 1].bar(x + .17, posture, .32, color=colors_bars, alpha=.45, label='Roll/pitch peak from0.1s')
    ax[2, 1].axhline(.35, color='#a11', linestyle='--', label='Heading limit0.35rad')
    ax[2, 1].axhline(.65, color='#333', linestyle=':', label='Posture limit0.65rad')
    ax[2, 1].set_title('Frozen heading and200Hz posture gates')
    ax[2, 1].set_ylabel('rad')
    for col in [0, 1]:
        ax[2, col].set_xticks(x, labels)
        ax[2, col].set_xlabel('T: Teacher; C: CHAMP; all actual repetitions retained')
        ax[2, col].grid(axis='y', alpha=.2)
        ax[2, col].legend(fontsize=8)
    fig.suptitle('Same-case actual SLAM/SCAN outer PID: Teacher vs CHAMP\nSeparate navigation/physical frames; goal-center distance1m is not exact robot travel', fontsize=14)
    image = OUT / 'pid_matched_short_v4_comparison.png'
    fig.savefig(image, dpi=160)
    plt.close(fig)
    provenance[str(OUT / 'comparison.json')] = sha(OUT / 'comparison.json')
    (OUT / 'figure_manifest.json').write_text(json.dumps({'schema': 'pid_matched_actual_figure/v1',
        'figure': str(image), 'figure_sha256': sha(image), 'script_sha256': sha(__file__),
        'raw_inputs': provenance, 'all_completed_actual_runs_shown': True,
        'no_synthetic_route_or_truth_control': True, 'source_frames_overlaid': False,
        'visual_QA': 'pending'}, indent=2) + '\n')
    print(image)


if __name__ == '__main__':
    main()
