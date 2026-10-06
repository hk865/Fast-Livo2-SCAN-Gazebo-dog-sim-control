#!/usr/bin/env python3
"""Offline synthetic LIO only. Never install the team override in production."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import statistics
import subprocess
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def snapshot():
    files = ['/proc/loadavg', '/proc/stat', '/proc/meminfo',
             '/sys/bus/event_source/devices/cpu_core/cpus', '/sys/bus/event_source/devices/cpu_atom/cpus']
    result = {'wall_epoch_s': time.time(), 'monotonic_wall_s': time.monotonic(),
              'logical_cpus': os.cpu_count(), 'allowed_cpus': sorted(os.sched_getaffinity(0))}
    result['files'] = {p: Path(p).read_text() for p in files if Path(p).is_file()}
    result['lscpu'] = subprocess.check_output(['lscpu', '-J'], text=True)
    result['processes'] = subprocess.check_output(['ps', '-eo', 'pid,ppid,pcpu,pmem,etimes,comm,args', '--sort=-pcpu'], text=True)
    return result


def describe(values):
    a = sorted(values)
    return {'count': len(a), 'minimum_ms': min(a), 'median_ms': statistics.median(a),
            'mean_ms': statistics.mean(a), 'p95_ms': a[min(len(a)-1, math.ceil(.95*len(a))-1)], 'maximum_ms': max(a)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-only', action='store_true')
    parser.add_argument('--cpu-class', choices=('P', 'E', 'all'), default='all')
    parser.add_argument('--repetitions', type=int, default=30)
    args = parser.parse_args()
    if args.repetitions < 30:
        raise ValueError('At least30 repeats required')
    build = json.loads((HERE / 'build_receipt.json').read_text())
    assert sha(build['library']) == build['library_sha256']
    env = os.environ.copy()
    env.pop('LD_PRELOAD', None)
    env.pop('FASTLIVO_DIAGNOSTIC_DIR', None)
    for key in ('GO2_TEST_LIO_TEAM', 'GO2_TEST_TEAM_RECORDS'):
        env.pop(key, None)
    env.update(OMP_DYNAMIC='FALSE', OMP_NUM_THREADS='2', OMP_THREAD_LIMIT='8',
               OMP_MAX_ACTIVE_LEVELS='1', OPENBLAS_NUM_THREADS='1', OMP_PROC_BIND='close')
    dirs = [str(Path(build['library']).parent), '/opt/ros/jazzy/lib', '/opt/ros/jazzy/lib/x86_64-linux-gnu',
            '/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',
            str(ROOT.parents[1] / 'slam5_navigation/ros2_ws/install/vikit_common/lib'),
            str(ROOT.parents[1] / 'slam5_navigation/ros2_ws/install/vikit_ros/lib')]
    env['LD_LIBRARY_PATH'] = ':'.join(dirs + [env.get('LD_LIBRARY_PATH', '')])
    baseline = HERE / 'baseline_result.json'
    results = json.loads(baseline.read_text())['cases'] if baseline.is_file() and not args.baseline_only else {}
    matrix = [('P_nopreload2', list(range(8)), 2, False)] if args.baseline_only or not results else []
    if not args.baseline_only:
        if args.cpu_class in ('P', 'all'):
            matrix += [('P_team' + str(n), list(range(8)), n, True) for n in (1, 2, 4, 8)]
        if args.cpu_class in ('E', 'all'):
            matrix += [('E_team' + str(n), list(range(8, 16)), n, True) for n in (1, 2, 4, 8)]
    before = snapshot()
    prefix = 'baseline' if args.baseline_only else args.cpu_class
    (HERE / ('resources_' + prefix + '_before.json')).write_text(json.dumps(before, indent=2))
    start = time.monotonic()
    for name, cpus, team, override in matrix:
        out = HERE / name
        out.mkdir(exist_ok=False)
        runenv = env.copy()
        runenv['OMP_PLACES'] = ','.join('{' + str(x) + '}' for x in cpus)
        if override:
            runenv.update(LD_PRELOAD=str(HERE / 'gomp_team_override.so'), GO2_TEST_LIO_TEAM=str(team),
                          GO2_TEST_TEAM_RECORDS=str(out / 'actual_teams.jsonl'))
        t = time.monotonic()
        cmd = [str(HERE / 'fixture'), str(out / 'canonical.bin'), str(out / 'timings.json'), str(args.repetitions)]
        run = subprocess.run(cmd, env=runenv, preexec_fn=lambda: os.sched_setaffinity(0, set(cpus)),
                             capture_output=True, text=True, timeout=20)
        (out / 'stdout.log').write_text(run.stdout)
        (out / 'stderr.log').write_text(run.stderr)
        if run.returncode:
            raise RuntimeError(name + ' failed: ' + str(run.returncode) + ' ' + run.stderr)
        data = json.loads((out / 'timings.json').read_text())
        teams = [json.loads(x) for x in (out / 'actual_teams.jsonl').read_text().splitlines()] if override else []
        if override:
            assert teams and all(x['actual_team'] == team for x in teams), (name, teams[:3])
            assert all(set(x['cpus']).issubset(cpus) for x in teams), name
        results[name] = {'affinity_cpus': cpus, 'requested_test_team': team, 'test_override': override,
                         'subprocess_wall_ms': (time.monotonic()-t)*1000,
                         'points': data['points'], 'map_root_voxels': data['map_root_voxels'],
                         'accepted_residuals': data['accepted_residuals'],
                         'actual_observed_teams': sorted(set(x['actual_team'] for x in teams)),
                         'observed_regions': len(teams), 'canonical_sha256': sha(out / 'canonical.bin'),
                         'canonical_bytes': (out / 'canonical.bin').stat().st_size,
                         'canonical_state_cov_values': data['canonical_state_cov_values'],
                         'StateEstimation': describe(data['StateEstimation_ms']),
                         'BuildResidualListOMP': describe(data['BuildResidualListOMP_ms']),
                         'fixture_without_output_io_ms': data['fixture_without_output_io_ms'],
                         'query_rows': data['query_rows']}
        print(json.dumps({'case': name, 'StateEstimation_median_ms': results[name]['StateEstimation']['median_ms'],
                          'residual_median_ms': results[name]['BuildResidualListOMP']['median_ms'],
                          'subprocess_wall_ms': results[name]['subprocess_wall_ms']}), flush=True)
    total = time.monotonic()-start
    hashes = {x['canonical_sha256'] for x in results.values()}
    report = {'schema': 'teacher_synthetic_LIO_thread_scaling/v1', 'library': build['library'],
              'library_sha256': build['library_sha256'], 'test_override_not_for_production': True,
              'source_geometry': 'Synthetic three separate planes; 3x91x91 points; not actual Gazebo/SLAM map',
              'scope': 'StateEstimation and original independent-index LIO residual region only; no VIO/physics/end-to-end Hz pass',
              'repetitions_per_component': args.repetitions, 'state_warmup': 5, 'residual_warmup': 8,
              'cases': results, 'state_cov_and_ordered_residual_binary_byte_equal_all_cases': len(hashes) == 1,
              'complete_benchmark_wall_s': total}
    assert len(hashes) == 1
    assert sha(build['library']) == build['library_sha256']
    filename = 'baseline_result.json' if args.baseline_only else 'results_' + args.cpu_class + '.json'
    (HERE / filename).write_text(json.dumps(report, indent=2) + '\n')
    (HERE / ('resources_' + prefix + '_after.json')).write_text(json.dumps(snapshot(), indent=2))
    print(json.dumps({'output': str(HERE / filename), 'all_byte_equal': True, 'wall_s': total}), flush=True)


if __name__ == '__main__':
    main()
