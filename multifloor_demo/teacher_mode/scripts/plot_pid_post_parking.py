#!/usr/bin/env python3
"""Plot real native standing drift after the frozen accepted parking window."""
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    helper_file = Path(__file__).with_name('analyze_pid_navigation_v3.py')
    spec = importlib.util.spec_from_file_location('plot_parking_helpers', helper_file)
    h = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(h)
    figure, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    input_hashes = {}
    for suffix, color, label in zip(['8c2a', 'f2b7', 'ecd9'], ['#205bc2', '#d16b00', '#31915e'], ['Teacher rep1', 'Teacher rep2', 'Teacher rep3']):
        runs = list((ROOT / 'runs').glob('*' + suffix))
        if len(runs) != 1:
            raise ValueError('Unique completed actual run required: ' + suffix)
        run = runs[0]
        diagnostic = run / 'pid_post_parking_diagnostic.json'
        d = json.loads(diagnostic.read_text())
        path = run / 'pid_navigation_independent_arrays.npz'
        with np.load(path) as z:
            time = z['native_world_time_s']
            mask = time >= d['actual_native_interval_world_s'][0] - 1e-10
            t = time[mask]
            p = z['native_position'][mask]
            v = z['native_world_COM_velocity'][mask]
            yaw = np.unwrap(h.rpy(z['native_quaternion_wxyz'][mask])[:, 2])
            axes[0].plot(t, np.linalg.norm(p[:, :2] - p[0, :2], axis=1), color=color, label=label, linewidth=1.2)
            axes[1].plot(t, yaw - yaw[0], color=color, linewidth=1.2)
            axes[2].plot(t, np.linalg.norm(v[:, :2], axis=1), color=color, linewidth=.8)
        input_hashes[str(path)] = sha(path)
        input_hashes[str(diagnostic)] = sha(diagnostic)
    axes[0].set_ylabel('XY drift from window end (m)')
    axes[1].set_ylabel('Native unwrapped yaw delta (rad)')
    axes[2].set_ylabel('Actual planar COM speed (m/s)')
    axes[2].set_xlabel('Native physical world time (s)')
    axes[0].legend()
    for ax in axes:
        ax.set_xlim(43, 180.005)
        ax.grid(alpha=.25)
    figure.suptitle('Actual prolonged zero-command Teacher standing after the formal5s parking window\nExtra diagnostic only; original3/3 short-case and fixed-window PASS remain unchanged', fontsize=12)
    out = ROOT / 'test_results/pid_matched_short_v4_20261004'
    image = out / 'teacher_post_parking_long_diagnostic.png'
    figure.savefig(image, dpi=160)
    plt.close(figure)
    (out / 'post_parking_figure_manifest.json').write_text(json.dumps({'schema': 'actual_post_parking_figure/v1',
        'figure': str(image), 'figure_sha256': sha(image), 'input_hashes': input_hashes,
        'script_sha256': sha(__file__), 'helper_sha256': sha(helper_file),
        'physical_data_frequency_hz': 200, 'synthetic_measurements': False,
        'diagnostic_is_new_acceptance_gate': False, 'visual_QA': 'pending'}, indent=2) + '\n')
    print(image)


if __name__ == '__main__':
    main()
