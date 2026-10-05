#!/usr/bin/env python3
"""Read-only causal command/clock audit; never imports ROS or changes run evidence."""
import argparse
import collections
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and np.isfinite(value)


def stamp(row):
    ns = row.get('ros_sim_time_ns')
    return ns / 1e9 if finite(ns) else row.get('sim_time')


def spans(history, predicate, maximum_sample_gap_wall_s=.35):
    """Observed snapshot spans, not proof of unobserved intervals between samples."""
    result = []
    first = last = None
    for row in history:
        sim, wall = stamp(row), row.get('monotonic_wall')
        if not finite(sim) or not finite(wall):
            continue
        good = predicate(row)
        if first is not None and (not good or wall-last[1] > maximum_sample_gap_wall_s):
            result.append({'start_sim_s': first[0], 'end_sim_s': last[0],
                           'duration_sim_s': last[0]-first[0], 'duration_wall_s': last[1]-first[1]})
            first = None
        if good:
            if first is None:
                first = (sim, wall)
            last = (sim, wall)
    if first is not None:
        result.append({'start_sim_s': first[0], 'end_sim_s': last[0],
                       'duration_sim_s': last[0]-first[0], 'duration_wall_s': last[1]-first[1]})
    return sorted(result, key=lambda item: item['duration_sim_s'], reverse=True)


def evaluate(run):
    run = run.resolve()
    policy = rows(run/'telemetry.jsonl')
    health = rows(run/'navigation_sensor_gate_history.jsonl')
    commands = rows(run/'navigation_command_history.jsonl')
    status = rows(run/'navigation_status.jsonl')
    command_index = {row['sequence']: row for row in commands if isinstance(row.get('sequence'), int)}
    command_hash = {seq: hashlib.sha256(json.dumps(row, sort_keys=True, allow_nan=False).encode()).hexdigest()
                    for seq, row in command_index.items()}
    lineage_errors = []
    ttl_errors = []
    read_counts = collections.Counter()
    accepted_age = []
    matched_envelope_frames = 0
    accepted_nonzero = []
    observations = []
    moving_stale = []
    previous_seq = 0
    for index, row in enumerate(policy):
        envelope = row.get('navigation_envelope') or {}
        outcome = envelope.get('read_status')
        read_counts[outcome] += 1
        read_wall = envelope.get('read_monotonic_wall')
        world = row.get('world_sim_time')
        if finite(read_wall) and finite(world):
            observations.append((world, read_wall))
        request = np.asarray(row.get('requested', [0, 0, 0]), float)
        command = np.asarray(row.get('command', [0, 0, 0]), float)
        if row.get('command_expired') is True and np.max(abs(request)) > 1e-9:
            ttl_errors.append({'frame': index, 'reason': 'Expired input emitted nonzero requested velocity'})
        if outcome in ('accepted', 'valid_but_unhealthy_or_stale'):
            seq = envelope.get('sequence')
            original = command_index.get(seq)
            if original is None or command_hash.get(seq) != envelope.get('hash'):
                lineage_errors.append({'frame': index, 'sequence': seq, 'reason': 'Missing or mismatched original envelope'})
                continue
            matched_envelope_frames += 1
            if seq < previous_seq:
                lineage_errors.append({'frame': index, 'sequence': seq, 'reason': 'Accepted envelope sequence moved backwards'})
            previous_seq = seq
            age_sim = world-original['sim_time']
            minimum_age_wall = read_wall-original['monotonic_wall']
            age_wall = envelope.get('wall_age_s')
            if (abs(age_sim-envelope['sim_age_s']) > 1e-8 or not finite(age_wall)
                    or age_wall < minimum_age_wall-1e-8):
                lineage_errors.append({'frame': index, 'sequence': seq, 'reason': 'Worker age does not match causal envelope clock'})
            if outcome == 'accepted':
                accepted_age.append((age_wall, age_sim))
                allowed = original['healthy'] is True and -.05 <= age_wall < .3 and -.05 <= age_sim < .3
                expected = np.zeros(3) if original['stop_requested'] else np.asarray(original['command'], float)
                if not allowed or row.get('command_expired') is not False or np.max(abs(request-expected)) > 1e-9:
                    ttl_errors.append({'frame': index, 'sequence': seq, 'reason': 'Accepted command violates frozen TTL/stop semantics'})
                if np.linalg.norm(request[:2]) > .015:
                    accepted_nonzero.append(row['sim_time'])
            elif row.get('command_expired') is not True:
                ttl_errors.append({'frame': index, 'sequence': seq, 'reason': 'Invalid age/health envelope was not marked expired'})
        if row.get('command_expired') is True and index and policy[index-1].get('command_expired') is not True:
            preceding = np.asarray([v['command'] for v in policy[max(0,index-25):index]], float)
            if len(preceding) and np.linalg.norm(preceding[:,:2], axis=1).max() > .015:
                moving_stale.append({'relative_sim_time_s': row['sim_time'], 'world_sim_time_s': world,
                                     'worker_read_status': outcome, 'source_sequence': envelope.get('sequence')})
    rtf = {}
    if len(observations) >= 2:
        observed = np.asarray(observations)
        ds, dw = observed[-1]-observed[0]
        bins = []
        for begin, end in zip(np.linspace(0, len(observed)-1, 5, dtype=int)[:-1],
                              np.linspace(0, len(observed)-1, 5, dtype=int)[1:]):
            sim_delta, wall_delta = observed[end]-observed[begin]
            bins.append({'simulation_span_s': float(sim_delta), 'wall_span_s': float(wall_delta),
                         'simulation_seconds_per_wall_second': float(sim_delta/wall_delta)})
        rtf = {'simulation_span_s': float(ds), 'wall_span_s': float(dw),
               'simulation_seconds_per_wall_second': float(ds/dw),
               'four_segment_observed_ratios': bins, 'sample_source': 'Worker actual read_monotonic_wall and world_sim_time',
               'physics_timestep_s': .005, 'policy_timestep_sim_s': .02,
               'meaning': 'Measured execution speed; slow simulation is not a real-time navigation pass'}
    ages = np.asarray(accepted_age) if accepted_age else np.empty((0, 2))
    result = {'schema_version': 1, 'scope': 'Independent post-run causal timing audit; no ROS or control writes',
              'run': str(run), 'checks': {
                  'worker_read_envelope_lineage': {'status': 'passed' if not lineage_errors and matched_envelope_frames else 'failed' if lineage_errors else 'unverified',
                                                    'matched_envelope_frames': matched_envelope_frames, 'errors': lineage_errors},
                  'observed_command_TTL_boundary': {'status': 'passed' if not ttl_errors and policy else 'failed', 'errors': ttl_errors,
                                                   'limit_s': .3, 'moving_to_stale_physical_stop': 'unverified' if not moving_stale else 'requires native stop interval audit',
                                                   'limit': 'Zero standing/rejected input holding alone does not verify stopping from motion'}},
              'metrics': {'real_time_factor_observed': rtf,
                          'sensor_ready_snapshot_spans': spans(health, lambda row: row.get('ready') is True),
                          'bridge_healthy_snapshot_spans': spans(commands, lambda row: row.get('healthy') is True),
                          'sensor_health_reason_counts': dict(collections.Counter(row.get('reason') for row in health)),
                          'bridge_reason_counts': dict(collections.Counter(row.get('reason') for row in commands)),
                          'worker_read_status_counts': dict(read_counts), 'accepted_worker_frames': len(accepted_age),
                          'accepted_nonzero_planar_velocity_frames': len(accepted_nonzero),
                          'accepted_maximum_wall_age_s': float(ages[:,0].max()) if len(ages) else None,
                          'accepted_maximum_sim_age_s': float(ages[:,1].max()) if len(ages) else None,
                          'moving_to_stale_events': moving_stale,
                          'alignment_phase_snapshot_counts': dict(collections.Counter(row.get('alignment_phase') for row in status)),
                          'maximum_controller_tilt_stop_counter': max((row.get('tilt_stops',0) for row in status), default=0),
                          'maximum_raw_imu_tilt_rad': max((row.get('raw_imu_max_tilt_rad') or 0 for row in status), default=None),
                          'heading_gate_required_pre_and_post_stop_sim_s': [1., 1.],
                          'snapshot_span_limit': 'Observed adjacent ready snapshots only; the command envelope chain and actual policy commands remain primary evidence'},
              'analyzer_sha256': sha(Path(__file__)), 'source_hashes': {name: sha(run/name) for name in
                  ('telemetry.jsonl','navigation_status.jsonl','navigation_sensor_gate_history.jsonl','navigation_command_history.jsonl')},
              'global_navigation_pass_claimed': False, 'original_evidence_overwritten': False}
    (run/'navigation_timing_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    for run in parser.parse_args().runs:
        result = evaluate(run)
        print(json.dumps({'run': str(run), 'checks': result['checks'],
                          'rtf': result['metrics']['real_time_factor_observed'].get('simulation_seconds_per_wall_second'),
                          'ready_longest_sim_s': next(iter(result['metrics']['bridge_healthy_snapshot_spans']),{}).get('duration_sim_s')}, allow_nan=False))


if __name__ == '__main__':
    main()
