#!/usr/bin/env python3
"""Audit original integer ROS clock receipts; do not backfill reader samples."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    run = parser.parse_args().run.resolve()
    if not (run / 'runtime_manifest.json').is_file(): raise RuntimeError('Completed manifest required')
    rows = [json.loads(line) for line in (run / 'telemetry.jsonl').open()]
    stamps = [row['actual_execution_health']['ros_sim_time_ns'] for row in rows]
    clock = {}
    digest = hashlib.sha256()
    with (run / 'sensor_shadow/sensor_samples.jsonl').open('rb') as stream:
        for line in stream:
            digest.update(line)
            if b'"source": "clock"' in line:
                row = json.loads(line); clock[row['stamp_ns']] = row['received_monotonic_wall']
    gap_counts = Counter(stamps[i] - stamps[i-1] for i in range(1, len(stamps)))
    candidates = Counter()
    for i in range(1, len(stamps)):
        if 24_999_998 <= stamps[i]-stamps[i-1] <= 25_000_002:
            expected = stamps[i-1] + 20_000_000
            if expected-1 in clock:
                candidates['shadow_candidate_short_by_1ns'] += 1
            elif expected in clock:
                candidates['shadow_candidate_exact_20ms'] += 1
                if clock[expected] <= rows[i]['actual_execution_health']['monotonic_wall']:
                    candidates['shadow_exact_candidate_received_before_next_reader_tick_wall'] += 1
            else: candidates['no_shadow_candidate_at_20ms'] += 1
    result = {'schema': 'CHAMP_actual_integer_clock_cadence_diagnostic/v1', 'run': str(run),
        'source': {'path': str(Path(__file__).resolve()), 'sha256': sha(__file__)},
        'original_telemetry_sha256': sha(run / 'telemetry.jsonl'),
        'original_shadow_source_sha256': digest.hexdigest(),
        'reader_rows': len(rows), 'reader_actual_clock_mod_5ms_ns_counts': dict(Counter(stamp % 5_000_000 for stamp in stamps)),
        'reader_actual_integer_clock_gap_ns_counts': dict(gap_counts),
        'shadow_original_clock_unique_stamps': len(clock),
        'shadow_actual_clock_mod_5ms_ns_counts': dict(Counter(stamp % 5_000_000 for stamp in clock)),
        '25ms_reader_gap_independent_shadow_20ms_candidates': dict(candidates),
        'one_nanosecond_boundary_error_observed_in_reader_or_shadow_clock': any(stamp % 5_000_000 for stamp in stamps) or any(stamp % 5_000_000 for stamp in clock),
        'direct_scope': 'Reader actual integer clocks and independent shadow actual original clock stamps only. Different receivers are not interchangeable; no assertion that the reader received a shadow-only candidate.',
        'cause_not_isolated': 'No current evidence of1ns clock grid errors. Callback/timer scheduling, coalescing or graph/file I/O latency remain candidates. Reader callback-level arrival/dispatch receipts would be a prospective diagnostic, not a repaired historical log.',
        'runtime_or_timestamp_modified': False, 'missing_reader_rows_filled': False,
        'no_ROS_Gazebo_or_control_signals': True}
    out = HERE / run.name / 'cadence_integer_diagnostic.json'
    with out.open('x') as stream: json.dump(result, stream, indent=2); stream.write('\n')
    print(json.dumps({'receipt': str(out), 'sha256': sha(out), 'reader_rows': len(rows),
        'reader_mod_5ms': result['reader_actual_clock_mod_5ms_ns_counts'],
        'shadow_mod_5ms': result['shadow_actual_clock_mod_5ms_ns_counts'], 'candidates': dict(candidates)}))


if __name__ == '__main__': main()
