#!/usr/bin/env python3
"""Bounded read-only batch-one CPU/CUDA Teacher latency/parity benchmark.

Uses archived actual Gazebo observations; never launches a robot, changes a
controller, or modifies another process. Existing Isaac Python/PyTorch only.
"""
from __future__ import annotations
import argparse
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import numpy as np
import torch

MODEL = Path('/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt')
EXPECTED_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
BASE = Path(__file__).resolve().parents[3]
DEFAULT_RUN = BASE / 'runs/20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def command(argv):
    try:
        p = subprocess.run(argv, text=True, capture_output=True, timeout=5)
        return {'command': argv, 'exit_code': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'command': argv, 'error': str(error)}


def resource_snapshot():
    ps = command(['ps', '-eo', 'pid,ppid,pcpu,pmem,etimes,args', '--sort=-pcpu'])
    rows = ps.get('stdout', '').splitlines()
    interested = re.compile(r'train\.py|rsl_rl|RL_for_unitree|IsaacLab|fastlivo|gz sim|gz-sim|worker\.py|benchmark_teacher\.py')
    return {
        'utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'monotonic_ns': time.monotonic_ns(),
        'load_average': list(os.getloadavg()), 'cpu_count': os.cpu_count(),
        'meminfo': Path('/proc/meminfo').read_text(),
        'gpu': command(['nvidia-smi', '--query-gpu=timestamp,index,name,memory.total,memory.used,utilization.gpu,utilization.memory,clocks.current.sm,temperature.gpu,power.draw', '--format=csv']),
        'gpu_compute_processes': command(['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory', '--format=csv']),
        'top_cpu_processes': '\n'.join(rows[:31]),
        'relevant_processes': '\n'.join(row for row in rows if interested.search(row)),
        'read_only_commands': True,
    }


def stats(values):
    a = np.asarray(values, dtype=float)
    return {name: float(value) for name, value in {
        'count': len(a), 'mean_ms': np.mean(a), 'p50_ms': np.median(a),
        'p95_ms': np.percentile(a, 95), 'p99_ms': np.percentile(a, 99),
        'min_ms': np.min(a), 'max_ms': np.max(a),
        'over_20ms': np.count_nonzero(a > 20),
    }.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, default=DEFAULT_RUN)
    ap.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'run_r1')
    ap.add_argument('--iterations', type=int, default=1000)
    ap.add_argument('--warmup', type=int, default=200)
    ap.add_argument('--paced-iterations', type=int, default=150)
    args = ap.parse_args()
    if not 50 <= args.iterations <= 1500 or not 20 <= args.warmup <= 300 or not 50 <= args.paced_iterations <= 200:
        raise ValueError('Benchmark must remain short and bounded')
    args.output.mkdir(parents=True, exist_ok=False)
    before = resource_snapshot()
    (args.output / 'resources_before.json').write_text(json.dumps(before, indent=2))
    model_sha = digest(MODEL)
    if model_sha != EXPECTED_SHA:
        raise RuntimeError('Frozen model hash mismatch')
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.set_float32_matmul_precision('highest')
    data_path = args.run / 'observations_actions.npz'
    with np.load(data_path, allow_pickle=False) as archive:
        observations = np.ascontiguousarray(archive['observations'], dtype=np.float32)
        actions_saved = np.ascontiguousarray(archive['actions'], dtype=np.float32)
    if observations.ndim != 2 or observations.shape[1] != 247 or not np.isfinite(observations).all():
        raise ValueError('Expected finite actual 247D observations')
    index = np.rint(np.linspace(0, len(observations) - 1, args.iterations)).astype(np.int64)
    selected = observations[index]
    np.save(args.output / 'selected_observation_indices.npy', index)
    actor = torch.nn.Sequential(torch.nn.Linear(247, 512), torch.nn.ELU(),
        torch.nn.Linear(512, 256), torch.nn.ELU(), torch.nn.Linear(256, 128),
        torch.nn.ELU(), torch.nn.Linear(128, 12)).eval()
    payload = torch.load(MODEL, map_location='cpu', weights_only=True)
    actor.load_state_dict({k.removeprefix('mlp.'): v for k, v in payload['actor_state_dict'].items() if k.startswith('mlp.')}, strict=True)
    if not all(torch.isfinite(v).all() for v in actor.state_dict().values()):
        raise ValueError('Nonfinite Actor weights')
    metadata = {
        'schema': 'go2_teacher_cpu_cuda_batch_one_benchmark/v1',
        'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
        'simulation_only': True, 'real_robot': False, 'runtime_actor_changed': False,
        'checkpoint': str(MODEL), 'checkpoint_sha256': model_sha,
        'actual_observations': str(data_path.resolve()), 'actual_observations_sha256': digest(data_path),
        'policy_manifest_sha256': digest(args.run / 'policy_manifest.json'),
        'source_worker_sha256': digest(args.run / 'sources/policy/worker.py'),
        'script_sha256': digest(__file__), 'python': os.path.realpath(os.sys.executable),
        'python_version': os.sys.version, 'torch_version': torch.__version__,
        'torch_cuda_version': torch.version.cuda, 'cuda_available': torch.cuda.is_available(),
        'cpu_threads': torch.get_num_threads(), 'interop_threads': torch.get_num_interop_threads(),
        'dtype': 'float32', 'tf32': False, 'eval': True, 'inference_mode': True,
        'batch_size': 1, 'network': [247, 512, 256, 128, 12], 'activation': 'ELU',
        'observations_count': len(observations), 'observations_dtype': str(observations.dtype),
        'selected_count': len(index), 'warmup': args.warmup, 'paced_iterations': args.paced_iterations,
        'paced_period_s': .02,
        'end_to_end_scope': 'finite NumPy CPU observation -> FP32 tensor -> forward -> NumPy 12D action on CPU; includes transfer for CUDA and explicit synchronization',
        'excludes': ['observation construction', 'ROS/Gazebo IPC', 'logging', 'joint-target PD', 'SLAM'],
        'source_observation_provenance': json.loads((args.run / 'policy_manifest.json').read_text()),
        'arch_list': torch.cuda.get_arch_list() if torch.cuda.is_available() else [],
    }
    (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
    samples, sample_stop = [], threading.Event()
    def sample_resources():
        while not sample_stop.wait(1):
            samples.append(resource_snapshot())
    sampler = threading.Thread(target=sample_resources, daemon=True)
    sampler.start()
    metrics = []
    def cpu(observation):
        if not np.isfinite(observation).all():
            raise ValueError('Nonfinite observation')
        return actor(torch.from_numpy(np.asarray(observation, dtype=np.float32)).reshape(1, 247)).numpy()[0]
    gpu_actor = None
    if torch.cuda.is_available():
        gpu_actor = copy.deepcopy(actor).to('cuda:0').eval()
        torch.cuda.synchronize()
        metadata.update(cuda_device=torch.cuda.get_device_name(0), cuda_capability=list(torch.cuda.get_device_capability(0)))
    def cuda(observation):
        if not np.isfinite(observation).all():
            raise ValueError('Nonfinite observation')
        tensor = torch.from_numpy(np.asarray(observation, dtype=np.float32)).reshape(1, 247).to('cuda:0', non_blocking=False)
        result = gpu_actor(tensor).to('cpu', non_blocking=False).numpy()[0]
        torch.cuda.synchronize()
        return result
    results = {'schema': metadata['schema'], 'cuda_tested': gpu_actor is not None, 'latency_ms': {}, 'parity': {}}
    try:
        with torch.inference_mode():
            for n in range(args.warmup):
                cpu(selected[n % len(selected)])
                if gpu_actor is not None:
                    cuda(selected[n % len(selected)])
            cpu_outputs, gpu_outputs = [], []
            cpu_ms, gpu_ms = [], []
            # Alternate ordering to reduce time-of-day/CPU-clock bias.
            for n, observation in enumerate(selected):
                order = [('cpu_burst_e2e', cpu)] if gpu_actor is None else (
                    [('cpu_burst_e2e', cpu), ('cuda_burst_e2e', cuda)] if n % 2 == 0 else
                    [('cuda_burst_e2e', cuda), ('cpu_burst_e2e', cpu)])
                for name, fn in order:
                    started = time.perf_counter_ns()
                    action = fn(observation)
                    elapsed = (time.perf_counter_ns() - started) / 1e6
                    metrics.append((name, n, elapsed))
                    if name.startswith('cpu'):
                        cpu_ms.append(elapsed); cpu_outputs.append(action.copy())
                    else:
                        gpu_ms.append(elapsed); gpu_outputs.append(action.copy())
            results['latency_ms']['cpu_burst_e2e'] = stats(cpu_ms)
            cpu_array = np.asarray(cpu_outputs)
            error_saved = np.abs(cpu_array - actions_saved[index])
            results['parity']['cpu_vs_archived_actor_actions'] = {
                'max_abs': float(error_saved.max()), 'mean_abs': float(error_saved.mean()),
                'allclose_atol_1e-5_rtol_1e-5': bool(np.allclose(cpu_array, actions_saved[index], atol=1e-5, rtol=1e-5)),
            }
            if gpu_actor is not None:
                results['latency_ms']['cuda_burst_e2e'] = stats(gpu_ms)
                gpu_array = np.asarray(gpu_outputs)
                error = np.abs(cpu_array - gpu_array)
                results['parity']['cuda_vs_cpu'] = {'max_abs': float(error.max()), 'mean_abs': float(error.mean()),
                    'p99_abs': float(np.percentile(error, 99)), 'finite': bool(np.isfinite(gpu_array).all()),
                    'allclose_atol_1e-5_rtol_1e-5': bool(np.allclose(cpu_array, gpu_array, atol=1e-5, rtol=1e-5))}
                np.savez_compressed(args.output / 'replayed_actions.npz', indices=index, cpu=cpu_array, cuda=gpu_array)
                tensor = torch.from_numpy(selected[0]).reshape(1, 247).to('cuda:0')
                wall_ms, event_ms = [], []
                first, last = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
                for n in range(args.iterations):
                    torch.cuda.synchronize()
                    started = time.perf_counter_ns()
                    first.record(); gpu_actor(tensor); last.record()
                    last.synchronize()
                    wall = (time.perf_counter_ns() - started) / 1e6
                    event = first.elapsed_time(last)
                    wall_ms.append(wall); event_ms.append(event)
                    metrics.append(('cuda_device_only_wall_sync', n, wall))
                    metrics.append(('cuda_device_only_event', n, event))
                results['latency_ms']['cuda_device_only_wall_sync'] = stats(wall_ms)
                results['latency_ms']['cuda_device_only_event'] = stats(event_ms)
            for name, fn in [('cpu_paced_50hz_e2e', cpu)] + ([] if gpu_actor is None else [('cuda_paced_50hz_e2e', cuda)]):
                latency, starts, overruns = [], [], []
                deadline = time.perf_counter()
                for n in range(args.paced_iterations):
                    delay = deadline - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                    started = time.perf_counter_ns(); starts.append(started)
                    fn(selected[n % len(selected)])
                    elapsed = (time.perf_counter_ns() - started) / 1e6
                    latency.append(elapsed); metrics.append((name, n, elapsed))
                    deadline += .02
                    overruns.append(time.perf_counter() > deadline)
                results['latency_ms'][name] = {**stats(latency),
                    'start_interval_ms': stats(np.diff(starts) / 1e6),
                    'loop_overruns_count': int(sum(overruns))}
    finally:
        sample_stop.set(); sampler.join(timeout=7)
        after = resource_snapshot()
        (args.output / 'resources_after.json').write_text(json.dumps(after, indent=2))
        (args.output / 'resource_samples.json').write_text(json.dumps(samples, indent=2))
    metadata['cuda_allocated_peak_bytes'] = torch.cuda.max_memory_allocated() if gpu_actor is not None else None
    metadata['cuda_reserved_peak_bytes'] = torch.cuda.max_memory_reserved() if gpu_actor is not None else None
    (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
    with (args.output / 'latencies.csv').open('x') as stream:
        stream.write('mode,iteration,latency_ms\n')
        for name, n, elapsed in metrics:
            stream.write(f'{name},{n},{elapsed:.9f}\n')
    (args.output / 'results.json').write_text(json.dumps(results, indent=2, allow_nan=False))
    print(json.dumps(results, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
