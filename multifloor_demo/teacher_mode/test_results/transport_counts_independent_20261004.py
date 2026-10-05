#!/usr/bin/env python3
"""Read original completed logs; do not modify runtime or other analysis output."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'test_results/transport_counts_independent_20261004.json'
SUFFIXES = ['9c33', '1c57', 'bd19', 'dba1']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'n': 0}
    return {'n': len(values), 'min_s': float(values.min()), 'p50_s': float(np.quantile(values, .5)),
        'p95_s': float(np.quantile(values, .95)), 'p99_s': float(np.quantile(values, .99)),
        'max_s': float(values.max()), 'mean_s': float(values.mean())}


def policy_counts(policy):
    top = [v.get('command_expired') for v in policy]
    # Only these statuses have current JSON parsed ages; missing/rejected can retain cached dict fields.
    parsed = [v for v in policy if (v.get('navigation_envelope') or {}).get('read_status')
              in ('accepted', 'valid_but_unhealthy_or_stale')]
    sim = [(v.get('navigation_envelope') or {}).get('sim_age_s') for v in parsed]
    wall = [(v.get('navigation_envelope') or {}).get('wall_age_s') for v in parsed]
    raw_top_age = [v.get('command_age_sim_s') for v in policy]
    pairs = [(s, w) for s, w in zip(sim, wall) if numeric(s) and numeric(w)]
    expired = [v for v in policy if v.get('command_expired') is True]
    expired_parsed = [v for v in expired if v in parsed]
    with_healthy_true = [v for v in expired_parsed if v['navigation_envelope'].get('healthy') is True]
    return {
        'policy_rows': len(policy),
        'top_level_command_expired_true': sum(v is True for v in top),
        'top_level_command_expired_false': sum(v is False for v in top),
        'top_level_command_expired_missing_or_nonbool': sum(not isinstance(v, bool) for v in top),
        'command_reason_distribution': dict(sorted(Counter(v.get('command_reason', '<missing>') for v in policy).items())),
        'navigation_envelope_read_status_distribution': dict(sorted(Counter((v.get('navigation_envelope') or {}).get('read_status', '<missing>') for v in policy).items())),
        'top_level_command_age_sim_s_since_last_nonexpired_read': {
            'available': sum(numeric(v) for v in raw_top_age),
            'at_least_300ms_count': sum(numeric(v) and v >= .3 for v in raw_top_age),
            'strictly_above_300ms_count': sum(numeric(v) and v > .3 for v in raw_top_age),
            'stats': stats([v for v in raw_top_age if numeric(v)])},
        'actual_parsed_envelope_age': {
            'freshly_parsed_rows': len(parsed),
            'not_currently_parsed_rows': len(policy) - len(parsed),
            'sim_age_at_least_300ms_count': sum(numeric(v) and v >= .3 for v in sim),
            'wall_age_at_least_300ms_count': sum(numeric(v) and v >= .3 for v in wall),
            'sim_age_strictly_above_300ms_count': sum(numeric(v) and v > .3 for v in sim),
            'wall_age_strictly_above_300ms_count': sum(numeric(v) and v > .3 for v in wall),
            'either_age_at_least_300ms_count': sum(s >= .3 or w >= .3 for s, w in pairs),
            'both_ages_at_least_300ms_count': sum(s >= .3 and w >= .3 for s, w in pairs),
            'sim_age_stats': stats([v for v in sim if numeric(v)]),
            'wall_age_stats': stats([v for v in wall if numeric(v)]),
            'sim_age_below_minus50ms_count': sum(numeric(v) and v < -.05 for v in sim),
            'wall_age_below_minus50ms_count': sum(numeric(v) and v < -.05 for v in wall)},
        'expired_or_unhealthy_breakdown': {
            'expired_current_envelope_healthy_false': sum(v['navigation_envelope'].get('healthy') is False for v in expired_parsed),
            'expired_current_envelope_healthy_true': len(with_healthy_true),
            'expired_without_currently_parsed_envelope': len(expired) - len(expired_parsed),
            'healthy_true_age_strictly_above_300ms': sum(
                v['navigation_envelope']['sim_age_s'] > .3 or v['navigation_envelope']['wall_age_s'] > .3
                for v in with_healthy_true),
            'healthy_false_with_both_current_ages_in_allowed_range': sum(
                v['navigation_envelope'].get('healthy') is False
                and -.05 <= v['navigation_envelope']['sim_age_s'] <= .3
                and -.05 <= v['navigation_envelope']['wall_age_s'] <= .3 for v in expired_parsed)},
    }


def analyze(run):
    policy = rows(run / 'telemetry.jsonl')
    history = rows(run / 'navigation_command_history.jsonl')
    durations = [v['transport_write_duration_wall_s'] for v in history]
    queues = [v['transport_queue_delay_wall_s'] for v in history]
    delivery = [v['transport_written_monotonic_wall'] - v['monotonic_wall'] for v in history]
    duration_errors = [abs(v['transport_write_duration_wall_s'] - (
        v['transport_written_monotonic_wall'] - v['transport_write_started_monotonic_wall'])) for v in history]
    queue_errors = [abs(v['transport_queue_delay_wall_s'] - (
        v['transport_write_started_monotonic_wall'] - v['monotonic_wall'])) for v in history]
    sequences = [v['sequence'] for v in history]
    sources = ['telemetry.jsonl', 'navigation_command_history.jsonl', 'policy_manifest.json',
               'sources/policy/worker.py', 'sources/navigation/bridge.py', 'sources/navigation/runtime_io.py']
    return {'run': str(run), 'duration_s': policy[-1]['sim_time'],
        'all_policy': policy_counts(policy),
        'policy_from_t3': policy_counts([v for v in policy if v['sim_time'] >= 3]),
        'per_actually_written_envelope': {
            'written_rows': len(history), 'healthy_written_rows': sum(v.get('healthy') is True for v in history),
            'unhealthy_written_rows': sum(v.get('healthy') is False for v in history),
            'write_duration_wall_s': stats(durations), 'queue_delay_wall_s': stats(queues),
            'total_original_envelope_to_write_completed_wall_s': stats(delivery),
            'write_duration_above300ms': sum(v > .3 for v in durations),
            'queue_delay_above300ms': sum(v > .3 for v in queues),
            'original_envelope_to_write_completed_above300ms': sum(v > .3 for v in delivery),
            'max_write_duration_receipt_error_s': max(duration_errors),
            'max_queue_delay_receipt_error_s': max(queue_errors),
            'original_wall_stamp_matches_embedded_stamp_every_row': all(v['monotonic_wall'] == v['stamp']['monotonic_wall'] for v in history),
            'strictly_increasing_written_sequence': all(b > a for a, b in zip(sequences, sequences[1:])),
            'first_sequence': sequences[0], 'last_sequence': sequences[-1],
            'skipped_sequence_numbers_between_written_rows': sum(b - a - 1 for a, b in zip(sequences, sequences[1:])),
            'max_recorded_cumulative_superseded': max(v['transport_superseded_envelopes'] for v in history)},
        'source_sha256': {name: sha(run / name) for name in sources}}


def main():
    if OUTPUT.exists():
        raise RuntimeError('Do not overwrite an existing independent receipt')
    runs = []
    for suffix in SUFFIXES:
        found = list((ROOT / 'runs').glob('*_' + suffix))
        if len(found) != 1:
            raise RuntimeError(f'Expected unique completed run {suffix}, got {len(found)}')
        runs.append(analyze(found[0]))
    result = {'schema': 'teacher_transport_counts_independent/v1',
        'generated_utc': datetime.now(timezone.utc).isoformat(), 'script_sha256': sha(__file__),
        'scope': 'Read-only actual completed logs; independent output; no ROS/simulator/signal and no older analysis modified',
        'definitions': {
            'command_expired': 'TOP LEVEL telemetry.command_expired bool. Returned flag means expired OR unhealthy OR missing/malformed, not pure TTL.',
            'command_age_sim_s': 'TOP LEVEL time since last nonexpired worker read, t-last_command_t; not actual original-envelope sim age.',
            'command_age_wall_s': 'No top-level key exists. Actual original-envelope wall age is navigation_envelope.wall_age_s, from actual consumer parse.',
            'actual_envelope_sim_age': 'navigation_envelope.sim_age_s, world time minus parsed original envelope sim_time.',
            'ages_not_currently_parsed': 'Only accepted/valid_but_unhealthy_or_stale statuses have fresh parsed ages; rejected/missing may retain cached dict fields and are separately counted.',
            'requested_age_counts': 'At least300ms counts use >=.3 exactly without epsilon; strictly above300ms separately uses >.3. Consumer accepts ages in[-.05,.3] inclusively.',
            'write_duration': 'Actually written receipt end-start. Includes JSON/temp/flush/close/replace (+oldfsync); not callback/queue or following append-history time.',
            'queue_delay': 'Actual write start minus ORIGINAL envelope wallstamp, never receipt completion time.',
            'quantiles': 'numpy.quantile linear interpolation; statistics in seconds, per actually written row.',
            'comparison_limit': 'One new no-fsync ordinary run versus3 old runs; timing association, not proof of unique causality or all TTL elimination.'},
        'runs': runs}
    OUTPUT.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    compact = []
    for r in runs:
        a, h = r['all_policy'], r['per_actually_written_envelope']
        compact.append({'run': Path(r['run']).name, 'policy': a['policy_rows'], 'expired_or_unhealthy': a['top_level_command_expired_true'],
            'reasons': a['command_reason_distribution'], 'top_lastgood_age_ge300ms': a['top_level_command_age_sim_s_since_last_nonexpired_read']['at_least_300ms_count'],
            'parsed_sim_ge300ms': a['actual_parsed_envelope_age']['sim_age_at_least_300ms_count'],
            'parsed_wall_ge300ms': a['actual_parsed_envelope_age']['wall_age_at_least_300ms_count'],
            'write_stats_s': h['write_duration_wall_s'], 'queue_stats_s': h['queue_delay_wall_s']})
    print(json.dumps({'output': str(OUTPUT), 'sha256': sha(OUTPUT), 'counts': compact}, ensure_ascii=False))


if __name__ == '__main__':
    main()
