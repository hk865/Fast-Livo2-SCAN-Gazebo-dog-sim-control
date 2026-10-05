#!/usr/bin/env python3
"""Body-velocity sweep around an archived CPU Teacher, with no outer PID."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
sys.dont_write_bytecode = True


def requested(plan, t):
    timing = plan['timing']; target = np.array([plan['speed_mps'], 0., plan['yaw_rate_radps']], float)
    if t < timing['stand_end_s']:
        return np.zeros(3), 'initializing' if t < .1 else 'teacher_stand'
    if t < timing['ramp_end_s']:
        fraction = (t-timing['stand_end_s'])/(timing['ramp_end_s']-timing['stand_end_s'])
        return target*fraction, 'plant_ramp'
    if t < timing['cruise_end_s']:
        return target, 'plant_cruise'
    return np.zeros(3), 'stop_transition'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--run', type=Path, required=True); ap.add_argument('--socket', required=True)
    args = ap.parse_args(); run = args.run.resolve(); plan = json.loads((run/'curvature_plan.json').read_text())
    manifest = json.loads((run/'source_manifest.json').read_text())
    for item in manifest.values():
        path = Path(item['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise RuntimeError('Frozen source mismatch: '+str(path))
    sys.path.insert(0, str(run/'sources/policy'))
    spec = importlib.util.spec_from_file_location('unchanged_curvature_teacher', run/'sources/policy/worker.py')
    legacy = importlib.util.module_from_spec(spec); spec.loader.exec_module(legacy)
    old_atomic = legacy.atomic_json; old_recv = legacy.recv_exact
    capture = {'state': None}; stream = (run/'plant_commands.jsonl').open('w', buffering=1)
    def receive(conn, n):
        data = old_recv(conn, n)
        if n == 512:
            capture['state'] = np.frombuffer(data, dtype='<f8').copy()
        return data
    def command(test, t):
        cmd, phase = requested(plan, t)
        # State is used only for an original clock receipt; commands are a fixed
        # body-velocity schedule, never pose/yaw feedback or a joint override.
        state = capture['state']
        stream.write(json.dumps({'schema': 'teacher_continuous_turn_body_command/v1', 'sim_time': t,
            'world_sim_time': float(state[0]) if state is not None else None,
            'state_physics_world_time': float(state[0]-.005) if state is not None else None,
            'requested_body_command': cmd.tolist(), 'phase': phase,
            'closed_loop_route_controller': False, 'uses_navigation_truth': False}, allow_nan=False)+'\n')
        return cmd, phase
    def atomic(path, obj):
        if path.name == 'policy_manifest.json':
            obj.update(test='continuous_turn_plant', command_source='Frozen piecewise body velocity plant command',
                test_duration_s=plan['timing']['duration_s'], command_start_seconds=3., command_end_s=26.,
                command_slew_acceleration=[.6,.6,.8], curvature_plan_sha256=hashlib.sha256((run/'curvature_plan.json').read_bytes()).hexdigest(),
                navigation_truth_used=False, counts_as_SLAM_navigation=False, PID_tracking_claim=False,
                legacy_motion_evaluation_applicable=False, plant_fixture='new 60x60m flat plane; not original SLAM5 multifloor',
                stop_strategy='Original Teacher zero body-velocity command and original slew, no action/joint replacement')
        if path.name == 'worker_result.json':
            obj.update(termination_reason='fixed_33s_plant_schedule', counts_as_SLAM_navigation=False,
                       PID_tracking_claim=False, plant_test_only=True)
        old_atomic(path, obj)
    legacy.recv_exact = receive; legacy.requested = command; legacy.atomic_json = atomic
    legacy.duration = lambda _: float(plan['timing']['duration_s'])
    sys.argv = [str(run/'sources/policy/worker.py'), '--run', str(run), '--test', 'forward', '--socket', args.socket]
    try:
        legacy.main()
    finally:
        stream.close()


if __name__ == '__main__': main()
