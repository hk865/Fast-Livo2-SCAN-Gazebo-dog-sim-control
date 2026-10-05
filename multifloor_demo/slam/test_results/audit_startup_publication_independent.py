"""Offline audit of immutable startup receiver/native-publish evidence only."""
import bisect
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    a = np.array(values, dtype=np.float64)
    return dict(count=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), p99=float(np.quantile(a, .99)),
                max=float(a.max())) if len(a) else None


def audit(tag):
    run = ROOT / 'simulation/test_results' / ('20261001_balance_startup_hook_' + tag)
    inputs = [run / name for name in ['startup_timing.jsonl', 'body_feedback.jsonl',
              'startup_diagnostic_result.json', 'process_cleanup.json', 'startup_manifest.json']]
    inputs += list(run.glob('actual_gz_process.json')) + list(run.glob('joint_publications.*.jsonl'))
    rows = [json.loads(x) for x in inputs[0].open()]
    body = [json.loads(x) for x in inputs[1].open()]
    native = [json.loads(x) for x in next(run.glob('joint_publications.*.jsonl')).open()]
    summary = next(d for d in native if d['kind'] == 'capture_summary')
    pubs = [d for d in native if d['kind'] == 'publish']
    by_seq = {d['sequence'] + 1: d for d in pubs}
    joints = [d for d in rows if d['kind'] == 'joint']
    commands = [d for d in rows if d['kind'] == 'actual_command']
    ticks = [d for d in body if d.get('event') == 'input_freshness_tick']
    startup = json.load(inputs[2].open())
    clean = json.load(inputs[3].open())
    manifest = json.load(inputs[4].open())
    checks = {
        'native_count_equals_summary': len(pubs) == summary['calls'],
        'bounded_no_overflow_incomplete': summary['overflow'] == summary['incomplete'] == 0,
        'cpp_type_verified': all(d['cpp_sensor_msgs_JointState_verified'] for d in native if d['kind'] == 'publisher'),
        'original_publish_success': all(d['ret'] == 0 for d in pubs),
        'header_unchanged': all(d['header_ns'] == d['header_after_ns'] for d in pubs),
        'native_sequences_exact': list(range(len(pubs))) == [d['sequence'] for d in pubs],
        'callback_sequence_and_header_match_native': all(d['publication_sequence_number'] in by_seq and by_seq[d['publication_sequence_number']]['header_ns'] == d['stamp_ns'] for d in joints),
        'callback_sequence_exact_prefix': list(range(1, len(joints) + 1)) == [d['publication_sequence_number'] for d in joints],
        'callback_payload_lengths_12': all(d['joints'] == d['positions'] == d['velocities'] == 12 for d in joints),
        'actual_commands_present_exact_zero': bool(commands) and all(len(d['value']) == 6 and all(v == 0 for v in d['value']) for d in commands),
        'owned_clean_source_unchanged': clean['owned_group_clean'] and clean['return_code'] == 0 and clean['source_unchanged'],
        'archived_sources_match_receipt': all(sha(run / 'staging' / name) == value['sha256'] for name, value in manifest['files'].items()),
    }
    before, after = max(zip(joints, joints[1:]), key=lambda ds: ds[1]['wall_monotonic_ns'] - ds[0]['wall_monotonic_ns'])
    start = by_seq[before['publication_sequence_number']]['enter_wall_ns']
    end = by_seq[after['publication_sequence_number']]['return_wall_ns']
    streams = []
    for kind in ['clock', 'imu', 'joint', 'jtc']:
        near = [d for d in rows if d['kind'] == kind and start - 10_000_000 <= d['wall_monotonic_ns'] <= end + 10_000_000]
        gaps = [(y['wall_monotonic_ns'] - x['wall_monotonic_ns'], x['stamp_ns'], y['stamp_ns']) for x, y in zip(near, near[1:])]
        largest = max(gaps)
        streams.append(dict(kind=kind, rows=len(near), first_header_ns=near[0]['stamp_ns'],
                            last_header_ns=near[-1]['stamp_ns'], largest_callback_gap_ns=largest[0],
                            largest_gap_stamp_ns=list(largest[1:])))
    durations = [d['return_wall_ns'] - d['enter_wall_ns'] for d in pubs]
    largest_call = pubs[int(np.argmax(durations))]
    clocks = [d for d in rows if d['kind'] == 'clock']
    clock_wall = [d['wall_monotonic_ns'] for d in clocks]

    def observed_clock_before(ns):
        i = bisect.bisect_right(clock_wall, ns) - 1
        return clocks[i]['stamp_ns'] if i >= 0 else None

    receipt = {
        'scope': 'Independent offline native-call/receiver audit; no ROS, physics, control or original-result changes.',
        'run': str(run), 'original_startup_passed': startup['passed'],
        'input_sha256': {p.name: sha(p) for p in inputs}, 'checks': checks,
        'review_evidence_consistent': all(checks.values()), 'capture_summary': summary,
        'callback_count': len(joints), 'native_count': len(pubs),
        'native_tail_after_observer_stop': len(pubs) - len(joints),
        'original_reported_max_callback_gap_ns': startup['max_joint_wall_gap_ns'],
        'independent_row_timestamp_max_callback_gap_ns': after['wall_monotonic_ns'] - before['wall_monotonic_ns'],
        'callback_gap_capture_difference_explanation': 'Callback gap accumulator and serialized row sample monotonic time separately; microsecond differences are expected, neither measures native call duration.',
        'max_callback_gap': {'before': before, 'after': after,
                             'native_before': by_seq[before['publication_sequence_number']],
                             'native_after': by_seq[after['publication_sequence_number']],
                             'nearby_streams': streams},
        'original_rcl_publish_duration_ns': stats(durations),
        'dds_source_to_received_ns': stats([d['dds_received_timestamp_ns'] - d['dds_source_timestamp_ns'] for d in joints]),
        'dds_received_to_callback_ns': stats([d['wall_system_ns'] - d['dds_received_timestamp_ns'] for d in joints]),
        'largest_original_call': {'native': largest_call, 'duration_ns': max(durations),
                                 'matched_callback': next((d for d in joints if d['publication_sequence_number'] == largest_call['sequence'] + 1), None),
                                 'received_clock_before_enter_ns': observed_clock_before(largest_call['enter_wall_ns']),
                                 'received_clock_before_return_ns': observed_clock_before(largest_call['return_wall_ns'])},
        'first_calibrated_predicate': next(d for d in ticks if d['predicates']['checks']['nominal_calibrated']),
        'first_failed_predicate': next((d for d in ticks if d['failed']), None),
        'middleware_measured': next(d for d in rows if d['kind'] == 'runtime'),
        'callback_header_minus_received_clock_ns': stats([d['stamp_ns'] - d['latest_clock_ns'] for d in joints]),
        'native_system_minus_monotonic_offset_change_ns': (summary['final_realtime_ns'] - summary['final_wall_ns']) - (summary['initial_realtime_ns'] - summary['initial_wall_ns']),
        'limits': [
            'Source/received use middleware system time; clock-before-call is a separate subscriber observation, not CM internal clock.',
            'The original-call duration can include thread descheduling. It does not identify a particular DDS mutex/ACK/discovery condition.',
            'Reliable endpoint QoS plus default synchronous FastDDS behavior supports a DDS-blocking candidate, not a unique-cause claim.',
            'Hook captures Header/call boundaries; callback payload length checks do not compare joint position values.',
            'No-motion startup does not establish exploration or active-balance physical success.',
        ],
    }
    receipt['observations'] = ([
        '104ms callback gap occurs at sim .200→.204 before nominal calibration (first calibrated .523).',
        'Original calls around that gap last29.369us/40.795us, native entry gap104.349655ms. Clock/IMU/JTC also pause at .200.',
        'All callback sequences match consecutive native prefix. Remaining native publications occur after observer stop.',
        'One successful startup does not prove the previous334ms gap resolved.',
    ] if tag == 'v1' else [
        'Native seq3960/header15.846 unchanged and successful original call lasts330.472478ms; matched DDS source→received330.451316ms.',
        'Previous15.844→delayed15.846 callback gap333.607089ms; Clock/IMU continue and JTC concurrently has333.926404ms receive gap.',
        'First failure integer15.882−15.844=38ms correctly violates30ms. No complete pair means joint_wall_fresh=false is not an independently measured100ms wall-age violation.',
        'Late15.846 is332ms behind received clock16.178 at callback. This delay is inside original publishing/receive path; it is not proved CM clock lag.',
        'No actual discovery event captured at the long-call instant; periodic unchanged publisher graph cannot prove subscriber-discovery causality.',
    ])
    dest = ROOT / 'slam/test_results' / ('oct1_startup_hook_' + tag + '_independent_publication_audit.json')
    dest.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    print(dest, checks)
    return receipt


if __name__ == '__main__':
    audit('v1')
    audit('v2')
