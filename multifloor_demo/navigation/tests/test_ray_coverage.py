#!/usr/bin/env python3
"""Independent geometry review of the production C++ RayCaster (no ROS/Gazebo).

The oracle splits the actual continuous segment at every grid-plane crossing
and tests interval midpoints. It does not copy the production DDA recurrence.
The sensor and measured endpoint voxels are excluded from free-space checks:
hit/miss voting, epoch wrap, and sliding inflation need separate GridMap tests.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = r'''
#include <iostream>
#include <plan_env/raycast.h>
int main() {
  Eigen::Vector3d start, end, voxel;
  while (std::cin >> start.x() >> start.y() >> start.z()
                  >> end.x() >> end.y() >> end.z()) {
    RayCaster ray;
    ray.setInput(start, end);
    int count = 0;
    while (ray.step(voxel)) {
      std::cout << int(voxel.x()) << ',' << int(voxel.y()) << ','
                << int(voxel.z()) << ' ';
      if (++count > 20000) { std::cerr << "unbounded traversal"; return 2; }
    }
    std::cout << '\n';
  }
}
'''


def index(point):
    return tuple(int(value) for value in np.floor(point))


def interval_oracle(start, end):
    """All voxels with positive segment length; grid-corner touches do not clear."""
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    direction = end - start
    if np.linalg.norm(direction) == 0:
        return set()
    times = [0., 1.]
    for axis, delta in enumerate(direction):
        if delta == 0:
            continue
        lo, hi = sorted((start[axis], end[axis]))
        for plane in range(int(np.floor(lo)) + 1, int(np.ceil(hi))):
            t = (plane-start[axis])/delta
            if 0 < t < 1:
                times.append(float(t))
    times = sorted(set(times))
    return {index(start+direction*((a+b)/2))
            for a, b in zip(times, times[1:]) if b-a > 1e-12}


def walks(binary, rays):
    data = ''.join(' '.join(format(float(v), '.17g') for v in (*a, *b))+'\n'
                   for a, b in rays)
    result = subprocess.run([str(binary)], input=data, text=True,
                            capture_output=True, timeout=30, check=True)
    rows = result.stdout.splitlines()
    if len(rows) != len(rays):
        raise AssertionError('production probe returned the wrong number of rays')
    return [[tuple(int(v) for v in cell.split(',')) for cell in row.split()]
            for row in rows]


def check_geometry(binary, name, rays):
    paths = walks(binary, rays)
    failures = []
    for n, ((start, end), path) in enumerate(zip(rays, paths)):
        excluded = {index(start), index(end)}
        measured = set(path)-excluded
        expected = interval_oracle(start, end)-excluded
        false_free, missing_free = measured-expected, expected-measured
        if false_free or missing_free or len(path) != len(set(path)):
            failures.append({'ray': n, 'start': [float(v) for v in start], 'end': [float(v) for v in end],
                'false_free': [list(v) for v in sorted(false_free)[:6]],
                'missing_free': [list(v) for v in sorted(missing_free)[:6]],
                'duplicate_steps': len(path)-len(set(path))})
    return {'name': name, 'rays': len(rays), 'passed': not failures,
            'failed_rays': len(failures), 'examples': failures[:4]}


def fixtures():
    yield 'continuous_direction_regression', [([3.9, 1.9, .1], [.1, .1, .1]),
        ([-26.0297966, -1.9446845, -3.2309403], [11.3035870, .5450591, 1.3484183])]
    yield 'axes_zero_length_same_voxel', [([3.9, .1, .1], [.1, .1, .1]),
        ([.1, .1, .1], [3.9, .1, .1]), ([.1, .1, .1], [.1, .1, .1]),
        ([.95, .1, .9], [.1, .9, .1]), ([.1, -3.9, .1], [.1, .1, .1])]
    yield 'negative_coordinates_grid_faces_and_corner_ties', [
        ([0., 0., 0.], [-3., -2., -1.]),
        ([-3., -2., -1.], [0., 0., 0.]),
        ([.5, .5, .5], [3.5, 3.5, 3.5]),
        ([3.5, 3.5, 3.5], [.5, .5, .5]),
        ([-.5, -.5, -.5], [-3.5, -3.5, -3.5])]
    boundary_rays = [([.5, .5, .5], [2.*sx, 2.*sy, 1.*sz])
                     for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    # Mixed signs at an exact final grid-plane crossing can never reach the
    # three floor(end) indices after advancing all tied axes. Stop at the real
    # finite segment parameter, not only when a particular index is reached.
    boundary_rays += [([.5, .5, .5], [2., -2., 1.]),
                      ([.5, .5, .5], [2., -2., .5]),
                      ([0., 0., 0.], [2., -2., 1.]),
                      ([2., -2., 1.], [-2., 2., -1.])]
    yield 'mixed_sign_exact_endpoint_planes', boundary_rays
    yield 'mixed_sign_boundary_reverse', [(b, a) for a, b in boundary_rays]
    yield 'translated_mixed_sign_endpoint_planes', [
        (np.asarray(a)+offset, np.asarray(b)+offset)
        for a, b in boundary_rays for offset in (-10., 10.)]
    origin = [.1, .1, .1]
    rays = [([3.05, 1.95, .05], origin), ([3.95, 1.05, .95], origin)]
    yield 'same_endpoint_voxel_distinct_measured_segments', rays
    yield 'duplicate_ray_determinism', [rays[0]] * 20
    rng = np.random.default_rng(20260930)
    random_rays = [(rng.uniform(-5, 5, 3).tolist(), rng.uniform(-5, 5, 3).tolist())
                   for _ in range(200)]
    yield 'seeded_continuous_geometry', random_rays
    yield 'reverse_segment_geometry', [(b, a) for a, b in random_rays]


def snapshot_check(binary, path):
    metadata = json.loads(path.read_text())
    arrays = np.load(path.with_suffix('.npz'))
    topic = metadata['topics']['filtered']
    origin = np.asarray(topic['nearest_slam_lidar']['pose'])
    resolution = .08
    points = arrays['filtered']
    rays = [(p/resolution, origin/resolution) for p in points]
    result = check_geometry(binary, 'run8_measured_sparse_cloud_geometry', rays)
    paths = walks(binary, rays)
    endpoints = {index(a) for a, _ in rays}
    old_free, complete_free, seen_ends = set(), set(), set()
    for (point, _), path_cells in zip(rays, paths):
        complete_free.update(path_cells)
        cell = index(point)
        if cell in seen_ends:
            continue
        seen_ends.add(cell)
        for c in path_cells:
            if c in old_free:
                break
            old_free.add(c)
    missed = complete_free-old_free-endpoints
    observed = {tuple(v) for v in np.floor(arrays['occupied']/resolution).astype(int)}
    result.update({'snapshot': str(path), 'snapshot_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'cloud_stamp': topic['stamp'], 'lidar_stamp_difference_s': topic['lidar_stamp_difference_s'],
        'old_early_exit_unique': len(old_free), 'all_rays_unique': len(complete_free),
        'early_exit_missed_nonendpoint': len(missed),
        'missed_also_in_recorded_occupancy': len(missed & observed),
        'scope': 'sparse archived scan order and measured matched sensor origin; not original full scan order, occupancy-history replay, or navigation acceptance'})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path,
        default=ROOT/'navigation/ros2_ws/src/plan_env')
    parser.add_argument('--snapshot', type=Path, default=ROOT/
        'runs/20260930_180318_efeddd/feedback_navigation_clouds/00000212.364.json')
    parser.add_argument('--report', type=Path, default=ROOT/'navigation/test_results/review_ray_coverage.json')
    args = parser.parse_args()
    source = args.source_dir/'src/raycast.cpp'
    with tempfile.TemporaryDirectory(prefix='demo_ray_review_') as temporary:
        tmp = Path(temporary)
        (tmp/'probe.cpp').write_text(WRAPPER)
        subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
            '-I'+str(args.source_dir/'include'), str(tmp/'probe.cpp'), str(source),
            '-o', str(tmp/'probe')], check=True, capture_output=True, text=True, timeout=60)
        checks = [check_geometry(tmp/'probe', name, rays) for name, rays in fixtures()]
        checks.append(snapshot_check(tmp/'probe', args.snapshot))
    report = {'scope': 'independent continuous-segment geometry review of production RayCaster only',
        'not_covered': ['GridMap hit/miss aggregation', 'epoch wrap', 'sliding-buffer reuse',
                        'full original-scan replay', 'physical mission completion'],
        'source': str(source), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'passed': all(c['passed'] for c in checks), 'checks': checks}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'passed': report['passed'], 'report': str(args.report),
        'checks': [{k: c[k] for k in ('name', 'passed', 'rays', 'failed_rays')} for c in checks]}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
