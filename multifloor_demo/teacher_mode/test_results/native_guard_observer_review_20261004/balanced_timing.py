#!/usr/bin/env python3
"""Balanced-order synthetic timing on already archived source, no runtime imports."""
import ast
import hashlib
import json
import math
from pathlib import Path
import time
from types import FunctionType
import numpy as np

OUT = Path(__file__).resolve().parent


def load(path, names, ns):
    tree = ast.parse(path.read_text())
    body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), ns)


def describe(a):
    a = np.asarray(a) * 1e6
    return {'p50_us': float(np.quantile(a, .5)), 'p95_us': float(np.quantile(a, .95)),
            'p99_us': float(np.quantile(a, .99)), 'max_us': float(a.max()), 'mean_us': float(a.mean())}


def main():
    destination = OUT / 'balanced_timing.json'
    if destination.exists():
        raise RuntimeError('Never overwrite timing evidence')
    ns = dict(np=np, math=math, FunctionType=FunctionType)
    load(OUT / 'sources/shared_control_core.py', ['steering_obstacle_ahead', 'obstacle_ahead'], ns)
    load(OUT / 'sources/guard_audit.py', ['NativeGuardAudit'], ns)
    native = ns['steering_obstacle_ahead']
    observed = ns['NativeGuardAudit'](native)
    rng = np.random.default_rng(42)
    def case(i):
        cloud = rng.uniform([-.5, -.8, -.3], [1.5, .8, .6], size=(160, 3))
        pose = np.zeros(3)
        target = np.array([1., .1 * math.sin(i), 0.])
        return cloud, pose, target, np.array([1., .1 * math.sin(i)]), np.array([pose, target])
    warm = case(0)
    for i in range(20):
        native(*warm)
        observed(*warm)
    times = [[], []]
    mismatch = 0
    for i in range(500):
        args = case(i)
        result = [None, None]
        order = (0, 1) if i % 2 == 0 else (1, 0)
        for index in order:
            start = time.perf_counter_ns()
            result[index] = (native if index == 0 else observed)(*args)
            times[index].append((time.perf_counter_ns() - start) * 1e-9)
        mismatch += result[0] != result[1]
    delta = np.asarray(times[1]) - np.asarray(times[0])
    result = {'scope': 'Balanced-order offline synthetic160point cases,20warmup pairs500measurement pairs; archived source only',
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'source_hashes': {n: hashlib.sha256((OUT / 'sources' / n).read_bytes()).hexdigest() for n in ['guard_audit.py', 'shared_control_core.py']},
        'case_pairs': 500, 'result_mismatches': int(mismatch),
        'native': describe(times[0]), 'observer': describe(times[1]),
        'paired_observer_minus_native': describe(delta),
        'limits': 'No actual callback/disk/ROS/physics timing; scheduler/cache variability can produce negative deltas, no strict worst-case timing bound.'}
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
