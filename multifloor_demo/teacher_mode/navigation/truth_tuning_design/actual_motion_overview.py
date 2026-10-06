#!/usr/bin/env python3
"""Summarize original D/D2 actual truth evidence; never changes any run."""
import argparse
import collections
import datetime
import hashlib
import json
from pathlib import Path
import numpy as np
from dashboard import ROOT, combined_evidence, digest, read_json

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', type=Path, default=ROOT / 'test_results/truth_pid_campaign_20261004')
    parser.add_argument('--output', type=Path, default=HERE / 'actual_motion_overview')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source_files = sorted(args.campaign.glob('phase_D*results.jsonl'))
    evidence, source_hashes, seen = [], {}, set()
    for source in source_files:
        source_hashes[str(source.resolve())] = digest(source)
        for line in source.read_text().splitlines():
            receipt = json.loads(line)
            if not receipt.get('run'):
                continue
            run = Path(receipt['run']).resolve()
            if run in seen:
                raise ValueError('Repeated run across D receipt files: ' + str(run))
            seen.add(run)
            profile = read_json(run / 'truth_profile.json')
            common = read_json(run / 'summary_truth_pid.json')
            terrain = read_json(run / 'summary_truth_terrain.json')
            combined, chain = combined_evidence(run, common, terrain)
            flat = profile['terrain'] in ('flat', 'flat_short', 'flat_long', 'flat_roundtrip')
            # Separate authority columns. Never upgrade a nonflat common result.
            motion_status = (combined['status'] if combined and chain['ancestor_chain_matches']
                             else common['status'] if flat and common else 'unverified')
            common_checks = common.get('checks', {}) if common else {}
            speed = common_checks.get('stable_real_COM_speed', {})
            route = common_checks.get('flat_or_requested_route_distance', {})
            parking = common_checks.get('fixed_final_goal_parking', {}).get('parking') or {}
            evidence.append({'run': str(run), 'scenario': profile['scenario'], 'terrain': profile['terrain'],
                             'feedback_hz': profile['feedback_hz'], 'desired_speed_mps': profile['desired_speed'],
                             'profile': profile, 'common_original_status': common.get('status') if common else None,
                             'terrain_original_status': terrain.get('status') if terrain else 'unverified',
                             'combined_original_status': combined.get('status') if combined else None,
                             'combined_ancestor_chain': chain, 'display_truth_motion_status': motion_status,
                             'actual_COM_route_projection_mean_mps': speed.get('mean_actual_forward_projection_mps'),
                             'actual_speed_MAE_mps': speed.get('mean_absolute_error_mps'),
                             'actual_route_RMS_m': route.get('drive_xy_distance_rms_m'),
                             'actual_route_max_m': route.get('maximum_xy_distance_m'),
                             'actual_parking_xy_m': parking.get('xy_drift_m'),
                             'actual_parking_yaw_rad': parking.get('yaw_drift_rad'),
                             'all_nonpassed_common_checks': {k: v for k, v in common_checks.items() if v['status'] != 'passed'},
                             'input_source_sha256': {name: digest(run / name) for name in
                                                    ('truth_profile.json', 'summary_truth_pid.json',
                                                     'summary_truth_terrain.json', 'summary_truth_motion_combined.json',
                                                     'telemetry.jsonl', 'control.jsonl', 'actuator.jsonl')}})
    if not evidence:
        raise ValueError('No actual completed D/D2 receipts')
    groups = collections.OrderedDict()
    for row in evidence:
        key = (row['scenario'], row['feedback_hz'], row['desired_speed_mps'])
        groups.setdefault(key, []).append(row)
    def stats(rows, key):
        values = [float(r[key]) for r in rows if r.get(key) is not None]
        return {'n': len(values), 'median': float(np.median(values)), 'min': min(values), 'max': max(values)} if values else None
    aggregate = []
    for (scenario, hz, speed), rows in groups.items():
        aggregate.append({'scenario': scenario, 'feedback_hz': hz, 'desired_speed_mps': speed,
                          'all_repeats_n': len(rows), 'truth_motion_counts': dict(collections.Counter(r['display_truth_motion_status'] for r in rows)),
                          'common_original_counts': dict(collections.Counter(r['common_original_status'] for r in rows)),
                          'terrain_original_counts': dict(collections.Counter(r['terrain_original_status'] for r in rows)),
                          'all_repeats_passed': all(r['display_truth_motion_status'] == 'passed' for r in rows),
                          'actual_speed': stats(rows, 'actual_COM_route_projection_mean_mps'),
                          'actual_route_RMS': stats(rows, 'actual_route_RMS_m'),
                          'actual_parking_yaw': stats(rows, 'actual_parking_yaw_rad'),
                          'runs': [r['run'] for r in rows]})
    report = {'schema': 'actual_truth_motion_overview/v1', 'updated_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Gazebo native truth feedback calibration; never SLAM fusion, navigation integration or hardware.',
              'status': 'Actual evidence snapshot; missing append-only geometry/composition remains unverified.',
              'snapshot_may_be_incomplete_while_campaign_running': True,
              'source_campaign_sha256': source_hashes, 'run_count': len(evidence),
              'truth_motion_counts': dict(collections.Counter(r['display_truth_motion_status'] for r in evidence)),
              'original_receipts_never_modified': True, 'groups': aggregate, 'runs': evidence,
              'statistics': 'Median and full min/max over every completed repetition; no best repetition selection.'}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(18, 11))
    labels = [r['scenario'] + '/' + str(r['feedback_hz']) + 'Hz' for r in aggregate]
    x = np.arange(len(aggregate))
    bottom = np.zeros(len(aggregate))
    for status, color in [('passed', '#3d8b68'), ('failed', '#b94a45'), ('unverified', '#b2b7be')]:
        values = np.array([r['truth_motion_counts'].get(status, 0) for r in aggregate])
        axes[0, 0].bar(x, values, bottom=bottom, color=color, label=status)
        bottom += values
    axes[0, 0].set_ylabel('All completed repetitions')
    axes[0, 0].set_title('Truth motion authority (geometry append required on terrain)')
    axes[0, 0].legend(fontsize=8)
    for ax, key, title, limit in [(axes[0, 1], 'actual_speed', 'Actual COM speed projected on route (m/s)', None),
                                  (axes[1, 0], 'actual_route_RMS', 'Actual path RMS (m)', .08),
                                  (axes[1, 1], 'actual_parking_yaw', 'Actual fixed5s parking yaw drift (rad)', .1)]:
        for i, group in enumerate(aggregate):
            val = group[key]
            if val:
                ax.errorbar(i, val['median'], yerr=[[val['median']-val['min']], [val['max']-val['median']]],
                            fmt='o', color='#236b95', capsize=3)
        if key == 'actual_speed':
            ax.scatter(x, [r['desired_speed_mps'] for r in aggregate], marker='x', color='#d9852c', label='Requested cruise')
            ax.legend(fontsize=8)
        if limit is not None:
            ax.axhline(limit, color='#ad645c', ls='--', lw=1)
        ax.set_title(title)
    for ax in axes.flat:
        ax.set_xticks(x, labels, rotation=60, ha='right', fontsize=8)
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Actual D/D2 frozen Teacher truth-feedback validation — all repetitions, original failures retained', fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, .965])
    picture = args.output / 'actual_motion_overview.png'
    fig.savefig(picture, dpi=140)
    plt.close(fig)
    report['figure'] = {'path': str(picture.resolve()), 'sha256': digest(picture)}
    out = args.output / 'actual_motion_overview.json'
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'report': str(out.resolve()), 'figure': str(picture.resolve()),
                      'run_count': len(evidence), 'truth_motion_counts': report['truth_motion_counts']}), flush=True)


if __name__ == '__main__':
    main()
