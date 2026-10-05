#!/usr/bin/env python3
"""Read V19-only committed events/lifecycle; no ROS, launch, signals or repairs.

This reader proves trace consistency, not original message bodies, estimator
math, loaded sources or navigation. Missing stages remain N/A, never 0 ms.
"""
import argparse
import collections
import csv
import hashlib
import json
import statistics
from pathlib import Path

SUMMARY_SCHEMA = 'staged_input_pipeline_v19/v1'
TRACE_SCHEMA = 'pipeline_v19_events/v1'
MODES = ('serial', 'rx_decode', 'staged')
COLUMNS = ['sequence', 'kind', 'source_ns', 'receipt_wall_ns', 'raw_enqueue_wall_ns',
           'decoder_pop_wall_ns', 'decode_begin_wall_ns', 'decode_end_wall_ns',
           'ready_enqueue_wall_ns', 'owner_pop_wall_ns', 'commit_end_wall_ns',
           'receive_tid', 'decode_tid', 'owner_tid', 'receive_cpu_begin_ns',
           'receive_cpu_end_ns', 'decode_cpu_begin_ns', 'decode_cpu_end_ns',
           'owner_cpu_begin_ns', 'owner_cpu_end_ns', 'reserved_bytes',
           'pending_after_commit', 'bytes_after_commit']
LIFE_COLUMNS = ['event', 'wall_ns', 'thread_cpu_ns', 'tid', 'accepted', 'delivered',
                'committed', 'pending', 'inflight', 'ready', 'bytes', 'context_valid']
LIFE_NORMAL = ['start', 'receiver_cancel_begin', 'receiver_joined', 'admission_close', 'decoders_joined',
               'owner_drain_begin', 'owner_drain_complete']
SUMMARY_INTS = {'image_copy_opt', 'count_limit', 'byte_limit', 'trace_columns',
                'accepted', 'delivered', 'committed', 'pending', 'inflight', 'ready',
                'bytes', 'peak_pending', 'peak_bytes', 'canceled', 'rejected_capacity',
                'closed_rejections'}
SUMMARY_KEYS = SUMMARY_INTS | {'schema', 'mode', 'trace_schema', 'normal_completed',
                'context_valid_at_drain', 'failure', 'uncommitted_packets', 'rejected_packet'}
KIND_NAMES = {0: 'timer', 1: 'imu', 2: 'lidar', 3: 'image'}
DECODE_COLUMNS = ['decoder_pop_wall_ns', 'decode_begin_wall_ns', 'decode_end_wall_ns',
                  'decode_tid', 'decode_cpu_begin_ns', 'decode_cpu_end_ns']


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''): h.update(chunk)
    return h.hexdigest()


def no_duplicate_keys(pairs):
    d = {}
    for key, value in pairs:
        if key in d: raise ValueError('Duplicate JSON key ' + key)
        d[key] = value
    return d


def read_csv(path, columns, string_columns=()):
    rows = []
    with Path(path).open(newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != columns: raise ValueError('Foreign/incomplete CSV schema: ' + str(path))
        for raw in reader:
            if None in raw or any(raw[key] is None for key in columns):
                raise ValueError('Truncated/extra CSV fields')
            row = {}
            for key in columns:
                text = raw[key]
                if key in string_columns: row[key] = text
                elif not text.isascii() or not text.isdigit(): raise ValueError('Non-unsigned integer ' + key)
                else: row[key] = int(text)
            rows.append(row)
    return rows


def read_summary(path):
    with Path(path).open() as f: s = json.load(f, object_pairs_hook=no_duplicate_keys)
    if set(s) != SUMMARY_KEYS or s['schema'] != SUMMARY_SCHEMA or s['trace_schema'] != TRACE_SCHEMA:
        raise ValueError('Foreign/incomplete V19 summary schema')
    if s['mode'] not in MODES or s['trace_columns'] != len(COLUMNS): raise ValueError('Foreign mode/trace columns')
    if any(type(s[key]) is not int or s[key] < 0 for key in SUMMARY_INTS): raise ValueError('Invalid summary integer')
    if s['image_copy_opt'] not in (0, 1) or s['count_limit'] != 512 or s['byte_limit'] != 67108864:
        raise ValueError('Changed prospective optimization/bounds contract')
    for key in ('normal_completed', 'context_valid_at_drain'):
        if type(s[key]) is not bool: raise ValueError('Invalid summary boolean')
    if not isinstance(s['failure'], str) or not isinstance(s['uncommitted_packets'], list):
        raise ValueError('Invalid failure/uncommitted identities')
    if s['rejected_packet'] is not None and not isinstance(s['rejected_packet'], dict):
        raise ValueError('Invalid rejected packet identity')
    return s


def check(value, **details):
    return {'status': 'passed' if value else 'failed', 'passed': bool(value), **details}


def quantile(values, fraction):
    a = sorted(values); pos = (len(a) - 1) * fraction; low = int(pos)
    return a[low] + (a[min(low + 1, len(a) - 1)] - a[low]) * (pos - low)


def distribution(values, unit='ms'):
    if not values: return {'status': 'unavailable', 'count': 0, 'reason': 'Not traversed/observed; missing is not zero', 'unit': unit}
    return {'status': 'available', 'count': len(values), 'unit': unit,
            'minimum': min(values), 'median': statistics.median(values), 'mean': statistics.mean(values),
            'p95': quantile(values, .95), 'p99': quantile(values, .99), 'maximum': max(values)}


def monotonically_ordered(rows, begin, end, tid):
    previous = {}
    for r in rows:
        if r[tid] == 0: continue
        if r[begin] > r[end] or previous.get(r[tid], 0) > r[begin]: return False
        previous[r[tid]] = r[end]
    return True


def analyze(rows, lifecycle, s):
    n = len(rows); checks = {}
    checks['whole_lifecycle_no_cancel_reject_missing'] = check(
        n > 0 and s['accepted'] == s['delivered'] == s['committed'] == n
        and all(s[k] == 0 for k in ('canceled', 'rejected_capacity', 'closed_rejections', 'pending', 'inflight', 'ready', 'bytes'))
        and s['failure'] == '' and s['uncommitted_packets'] == [] and s['rejected_packet'] is None
        and s['normal_completed'] and s['context_valid_at_drain'],
        final_counters=s, cleanup_exemption_applied=False)
    checks['contiguous_committed_global_admission_order'] = check(
        [r['sequence'] for r in rows] == list(range(1, n + 1)) and all(r['kind'] in KIND_NAMES for r in rows))
    checks['all_sensor_kinds_observed'] = check({1, 2, 3}.issubset(r['kind'] for r in rows),
                                             counts=dict(collections.Counter(r['kind'] for r in rows)))
    temporal = bool(rows); bytes_ok = bool(rows)
    for r in rows:
        common = [r[k] for k in ('receipt_wall_ns', 'raw_enqueue_wall_ns', 'ready_enqueue_wall_ns', 'owner_pop_wall_ns', 'commit_end_wall_ns')]
        temporal = temporal and all(t > 0 for t in common) and common == sorted(common)
        temporal = temporal and r['receive_cpu_begin_ns'] <= r['receive_cpu_end_ns'] and r['owner_cpu_begin_ns'] <= r['owner_cpu_end_ns']
        if r['kind'] >= 2:
            chain = [r[k] for k in ('raw_enqueue_wall_ns', 'decoder_pop_wall_ns', 'decode_begin_wall_ns', 'decode_end_wall_ns', 'ready_enqueue_wall_ns')]
            temporal = temporal and all(t > 0 for t in chain) and chain == sorted(chain) and r['decode_cpu_begin_ns'] <= r['decode_cpu_end_ns'] and r['decode_tid'] > 0
        else:
            temporal = temporal and all(r[k] == 0 for k in DECODE_COLUMNS) and r['ready_enqueue_wall_ns'] == r['raw_enqueue_wall_ns']
        if r['kind'] == 0: temporal = temporal and r['source_ns'] == 0
        bytes_ok = bytes_ok and 0 < r['reserved_bytes'] <= s['byte_limit'] and r['pending_after_commit'] <= s['count_limit'] and r['bytes_after_commit'] <= s['byte_limit']
        bytes_ok = bytes_ok and ((r['pending_after_commit'] == 0) == (r['bytes_after_commit'] == 0))
    temporal = temporal and all(b['receipt_wall_ns'] >= a['receipt_wall_ns'] and b['owner_pop_wall_ns'] >= a['commit_end_wall_ns'] for a, b in zip(rows, rows[1:]))
    temporal = temporal and monotonically_ordered(rows, 'receive_cpu_begin_ns', 'receive_cpu_end_ns', 'receive_tid') and monotonically_ordered(rows, 'owner_cpu_begin_ns', 'owner_cpu_end_ns', 'owner_tid')
    decode_rows = sorted((r for r in rows if r['kind'] >= 2), key=lambda r: r['decode_begin_wall_ns'])
    temporal = temporal and monotonically_ordered(decode_rows, 'decode_cpu_begin_ns', 'decode_cpu_end_ns', 'decode_tid')
    checks['integer_clock_causality_and_decoder_NA'] = check(temporal)
    rxt = {r['receive_tid'] for r in rows}; own = {r['owner_tid'] for r in rows}
    dec = {kind: {r['decode_tid'] for r in rows if r['kind'] == kind} for kind in (2, 3)}
    team = len(rxt) == len(own) == 1 and all(x > 0 for x in rxt | own)
    if s['mode'] == 'serial': team = team and rxt == own and dec[2] == dec[3] == own
    elif s['mode'] == 'rx_decode': team = team and rxt.isdisjoint(own) and dec[2] == dec[3] == rxt
    else:
        team = team and rxt.isdisjoint(own) and all(len(dec[k]) == 1 and 0 not in dec[k] and dec[k].isdisjoint(rxt | own) for k in dec) and dec[2].isdisjoint(dec[3])
    checks['actual_mode_thread_partition'] = check(team, receive_tids=sorted(rxt), owner_tids=sorted(own), decoder_tids={str(k): sorted(v) for k, v in dec.items()})
    bytes_ok = bytes_ok and s['peak_pending'] <= s['count_limit'] and s['peak_bytes'] <= s['byte_limit'] and s['peak_pending'] >= max((r['pending_after_commit'] for r in rows), default=0) and s['peak_bytes'] >= max((r['bytes_after_commit'] for r in rows), default=0)
    checks['charged_raw_decode_ready_owner_reservation_bounds'] = check(bytes_ok,
        definition='Reservation slots charged through acknowledgement; not whole process/DDS/map RSS')
    life_ok = [r['event'] for r in lifecycle] == LIFE_NORMAL
    life_ok = life_ok and all(r['context_valid'] == 1 and r['tid'] in own and r['wall_ns'] > 0 and r['delivered'] <= r['accepted'] and r['committed'] <= r['delivered'] and r['pending'] <= s['count_limit'] and r['bytes'] <= s['byte_limit'] for r in lifecycle)
    life_ok = life_ok and all(b['wall_ns'] >= a['wall_ns'] and b['thread_cpu_ns'] >= a['thread_cpu_ns'] and b['accepted'] >= a['accepted'] and b['delivered'] >= a['delivered'] and b['committed'] >= a['committed'] for a, b in zip(lifecycle, lifecycle[1:]))
    if lifecycle:
        final = lifecycle[-1]
        life_ok = life_ok and final['accepted'] == final['delivered'] == final['committed'] == n and all(final[k] == 0 for k in ('pending', 'inflight', 'ready', 'bytes'))
        life_ok = life_ok and (not rows or final['wall_ns'] >= rows[-1]['commit_end_wall_ns'])
    checks['original_normal_context_valid_drain_lifecycle'] = check(life_ok,
        expected_events=LIFE_NORMAL, observed_events=[r['event'] for r in lifecycle],
        inflight_definition='Producer counts Decoding plus Owner, not only decoder work')
    metrics = {}
    pairs = {
        'receipt_to_reservation_marker_wall': ('receipt_wall_ns', 'raw_enqueue_wall_ns'),
        'reservation_to_decoder_claim_wall': ('raw_enqueue_wall_ns', 'decoder_pop_wall_ns'),
        'decode_wall': ('decode_begin_wall_ns', 'decode_end_wall_ns'),
        'decode_thread_CPU': ('decode_cpu_begin_ns', 'decode_cpu_end_ns'),
        'decode_end_to_ready_marker_wall': ('decode_end_wall_ns', 'ready_enqueue_wall_ns'),
        'ready_marker_to_owner_pop_wall': ('ready_enqueue_wall_ns', 'owner_pop_wall_ns'),
        'owner_callback_wall': ('owner_pop_wall_ns', 'commit_end_wall_ns'),
        'owner_callback_thread_CPU': ('owner_cpu_begin_ns', 'owner_cpu_end_ns'),
        'pre_admission_metadata_budget_thread_CPU': ('receive_cpu_begin_ns', 'receive_cpu_end_ns'),
        'receipt_to_commit_wall': ('receipt_wall_ns', 'commit_end_wall_ns'),
    }
    for kind, name in KIND_NAMES.items():
        selected = [r for r in rows if r['kind'] == kind]
        metrics[name] = {metric: distribution([(r[end] - r[begin]) / 1e6 for r in selected if r[begin] > 0 and r[end] > 0]) for metric, (begin, end) in pairs.items()}
        metrics[name]['pending_after_commit'] = distribution([r['pending_after_commit'] for r in selected], 'packets')
        metrics[name]['reserved_bytes'] = distribution([r['reserved_bytes'] for r in selected], 'bytes')
        stamps = [r['source_ns'] for r in selected] if kind else []
        ordered = all(b >= a for a, b in zip(stamps, stamps[1:]))
        distinct = list(dict.fromkeys(stamps))
        metrics[name]['source_header_rate'] = {'unique_stamps': len(distinct), 'duplicates': len(stamps) - len(distinct),
            'backward_transitions': sum(b < a for a, b in zip(stamps, stamps[1:])),
            'Hz': (len(distinct) - 1) * 1e9 / (distinct[-1] - distinct[0]) if ordered and len(distinct) > 1 and distinct[-1] > distinct[0] else None,
            'definition': 'Distinct original source headers, not nominal frequency; timer has no source header'}
    return {'schema': 'go2_pipeline_v19_stage_performance_reader/v1',
            'status': 'passed_limited_trace_and_lifecycle' if all(c['passed'] for c in checks.values()) else 'failed',
            'mode': s['mode'], 'image_copy_opt': s['image_copy_opt'], 'checks': checks, 'metrics': metrics,
            'whole_message_body_identity_proven': False, 'estimator_math_equivalence_proven': False,
            'runtime_source_binding_checked_by_this_reader': False, 'actual_navigation_or_Sim2Sim_PASS': False,
            'interpretation': {
                'queue_stage': 'All modes use charged reservation slots; serial/rx_decode claim inline. Their measured claim intervals are not asynchronous queue-wait evidence.',
                'receive_CPU': 'Metadata and conservative budget interval only, excludes queue lock/decode/owner; allocation before receipt interval is not measured.',
                'owner_interval': 'Original callback only; excludes acknowledgement, trace formatting and full estimator/map processing.',
                'wall_minus_CPU': 'Never labeled communication; no subtraction is computed.',
                'parent_child_intervals': 'receipt_to_commit contains stage intervals; do not add it to them. This is not old kind300 spin_some/preprocess timing.',
                'source_age': 'No current ROS clock/source-head trace here, so source age is unavailable, not zero.',
                'missing_stage': 'IMU/timer decoder fields zero are N/A. Missing actual input or failed causality does not become zero latency.'}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--events', type=Path, required=True)
    p.add_argument('--lifecycle', type=Path, required=True)
    p.add_argument('--summary', type=Path, required=True)
    p.add_argument('--mode', choices=MODES, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    s = read_summary(args.summary)
    if args.mode != s['mode']: raise ValueError('CLI expected mode does not match actual producer summary')
    result = analyze(read_csv(args.events, COLUMNS), read_csv(args.lifecycle, LIFE_COLUMNS, ('event',)), s)
    result['input_sha256'] = {str(path.resolve()): sha(path) for path in (args.events, args.lifecycle, args.summary)}
    result['reader_sha256'] = sha(Path(__file__))
    with args.out.open('x') as f: json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({'status': result['status'], 'out': str(args.out), 'actual_navigation_PASS': False}))
    return 0 if result['status'] == 'passed_limited_trace_and_lifecycle' else 1


if __name__ == '__main__': raise SystemExit(main())
