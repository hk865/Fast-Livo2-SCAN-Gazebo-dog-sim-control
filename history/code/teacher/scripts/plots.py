#!/usr/bin/env python3
"""Export scientific plots of recorded Teacher commands, motion and truth.

Default is a read-only plot plan; --render creates actual PNGs and provenance.
No synthetic route, policy replay, SLAM substitute or simulation is generated.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from aggregate import START, TESTS, REF_NAMES, collect, trace, digest, terrain_references


def render_group(test, entries, output, reference):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.facecolor': 'white', 'figure.facecolor': 'white'})
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    axes = axes.ravel()
    units, names = ('m/s', 'm/s', 'rad/s'), ('Body vx', 'Body vy', 'Body wz')
    colors = plt.get_cmap('tab10')
    manifest = {'test': test, 'scope': 'Actual Gazebo simulation truth for diagnostics, not a SLAM navigation route', 'runs': []}
    any_rows = False
    for index, entry in enumerate(entries):
        directory = Path(entry['path'])
        rows = trace(directory / 'telemetry.jsonl')
        manifest['runs'].append({'run_id': entry['run_id'], 'interface': entry['interface'], 'motion': entry['motion'],
                                 'reason': entry['reason'], 'telemetry_sha256': digest(directory / 'telemetry.jsonl'),
                                 'summary_sha256': digest(directory / 'summary.json'), 'frames': len(rows)})
        if not rows:
            continue
        any_rows = True
        t = np.asarray([row['t'] for row in rows]); command = np.asarray([row['command'] for row in rows]); measured = np.asarray([row['measured'] for row in rows])
        color = colors(index % 10)
        label = f"Gazebo {index+1} [{entry['motion']}]"
        for axis in range(3):
            axes[axis].plot(t, measured[:, axis], color=color, lw=1.1, alpha=.85, label=label)
            if index == 0:
                axes[axis].plot(t, command[:, axis], color='black', ls='--', lw=1.2, label='Actual command')
        points = np.asarray([row['position'] for row in rows if isinstance(row.get('position'), list) and len(row['position']) == 3], dtype=float)
        if len(points) and np.isfinite(points).all():
            axes[3].plot(points[:, 0], points[:, 1], color=color, lw=1.1, label=label)
            axes[3].scatter(points[0, 0], points[0, 1], color=color, marker='o', s=22)
            axes[3].scatter(points[-1, 0], points[-1, 1], color=color, marker='x', s=30)
    if not any_rows:
        plt.close(figure)
        manifest['status'] = 'unverified_no_recorded_telemetry'
        return manifest
    reference_file = reference / (REF_NAMES.get(test, test) + '_trace.json')
    ref = trace(reference_file, True)
    if ref:
        rt = np.asarray([row['t'] for row in ref]); rv = np.asarray([row['measured'] for row in ref])
        for axis in range(3):
            axes[axis].plot(rt, rv[:, axis], color='#666666', ls=':', lw=1.5, label='Recorded PhysX reference')
        positions = np.asarray([row['position'] for row in ref if isinstance(row.get('position'), list) and len(row['position']) == 3], dtype=float)
        if len(positions) and np.isfinite(positions).all():
            axes[3].plot(positions[:, 0], positions[:, 1], color='#666666', ls=':', lw=1.5, label='Recorded PhysX truth')
        manifest.update(reference_sha256=digest(reference_file), reference_file=str(reference_file.resolve()))
    for axis in range(3):
        axes[axis].set(title=names[axis], xlabel='Simulation time [s]', ylabel=f'{names[axis]} [{units[axis]}]')
        axes[axis].grid(alpha=.2)
        axes[axis].legend(fontsize=8, loc='best')
    axes[3].set(title='Recorded x/y trajectory (truth diagnostic)', xlabel='World x [m]', ylabel='World y [m]')
    axes[3].set_aspect('equal', adjustable='datalim')
    axes[3].grid(alpha=.2); axes[3].legend(fontsize=8, loc='best')
    figure.suptitle(f'Go2 frozen Teacher: {test} | all selected recorded attempts, including failures', fontsize=13)
    target = output / (test + '.png')
    figure.savefig(target, dpi=180)
    plt.close(figure)
    manifest.update(status='rendered_from_recorded_data', file=str(target.resolve()), file_sha256=digest(target))
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    parser.add_argument('--campaign-start', default=START)
    parser.add_argument('--run', type=Path, action='append', help='Explicit recorded runs; otherwise plot every baseline campaign attempt')
    parser.add_argument('--reference', type=Path, default=ROOT / 'runs/isaac_cpu_matched_reference_20261003_c')
    parser.add_argument('--terrain-reference', type=Path, help='Root containing the recorded e/f/g terrain reference directories')
    parser.add_argument('--original-origin-reference', type=Path, default=ROOT / 'runs/isaac_cpu_original_origin_20261003_h')
    parser.add_argument('--output', type=Path, default=ROOT / 'plots')
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    entries = collect(args.runs.resolve(), args.campaign_start)
    if args.run:
        selected = {str(path.resolve()) for path in args.run}
        entries = [entry for entry in entries if entry['path'] in selected]
    else:
        entries = [entry for entry in entries if entry['formal'] or entry['original_origin']]
    groups = {}
    for entry in entries:
        name = 'stand_original_origin' if entry['original_origin'] else entry['test']
        groups.setdefault(name, []).append(entry)
    if not args.render:
        print(json.dumps({'status': 'prepared_unverified', 'writes_images': False, 'plots': {name: [entry['run_id'] for entry in group] for name, group in groups.items()},
                          'output': str(args.output.resolve()), 'scope': 'Scientific PNGs from recorded commands and motion only; no SLAM route is fabricated'}, ensure_ascii=False, indent=2))
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    terrains = terrain_references(args.runs.resolve(), args.terrain_reference)
    results = [render_group(name, group, args.output,
                args.original_origin_reference if name == 'stand_original_origin' else terrains.get(name, args.reference))
               for name, group in groups.items()]
    manifest = {'status': 'recorded_plots_exported', 'plots': results, 'script_sha256': digest(Path(__file__)),
                'scope': 'Simulation truth is a diagnostic. Missing telemetry is skipped and remains unverified. PNG output is not a motion or navigation pass.'}
    (args.output / 'plot_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'output': str(args.output.resolve()), 'rendered': sum(row['status'] == 'rendered_from_recorded_data' for row in results),
                      'missing': sum(row['status'] != 'rendered_from_recorded_data' for row in results)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
