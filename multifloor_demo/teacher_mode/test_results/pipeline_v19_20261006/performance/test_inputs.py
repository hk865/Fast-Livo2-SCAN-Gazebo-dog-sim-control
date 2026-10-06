#!/usr/bin/env python3
"""Small synthetic reader tests only; never fabricate actual capture PASS."""
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
from input_identity import SCHEMA, TOPICS, compare, identity
from schedule_abba import schedule


def main():
    rows = [{'schema': SCHEMA, 'bag_record_index': i, 'topic': topic, 'type': typ,
             'source_ns': i + 1, 'serialized_bytes': i + 5,
             'cdr_sha256': hashlib.sha256(topic.encode()).hexdigest(),
             'bag_received_timestamp_ns': 1000 + i} for i, (topic, typ) in enumerate(TOPICS.items())]
    tests = []
    def check(name, value):
        if not value: raise RuntimeError('FAILED ' + name)
        tests.append({'name': name, 'status': 'passed', 'synthetic_only': True})
    check('complete_same_capture_identity', compare(rows, copy.deepcopy(rows))['status'] == 'passed_captured_identity_only')
    reordered = rows[::-1]
    c = compare(rows, reordered)
    check('same_content_different_cross_topic_order_is_not_pass', c['same_complete_CDR_multiset'] and not c['same_bag_record_order'] and c['status'] == 'failed')
    changed = copy.deepcopy(rows); changed[0]['cdr_sha256'] = '1' * 64
    check('changed_complete_CDR_digest_rejected', compare(rows, changed)['status'] == 'failed')
    changed = copy.deepcopy(rows); changed[0]['source_ns'] += 1
    check('changed_integer_source_stamp_rejected', compare(rows, changed)['status'] == 'failed')
    check('missing_imu_rejected', compare(rows, rows[1:])['status'] == 'failed')
    check('extra_duplicate_rejected', compare(rows, rows + rows[:1])['status'] == 'failed')
    check('incomplete_equal_indices_not_pass', compare(rows[1:], rows[1:])['status'] == 'failed')
    try:
        foreign = dict(rows[0], type='sensor_msgs/msg/Image'); identity(foreign)
        check('foreign_type_rejected', False)
    except ValueError: check('foreign_type_rejected', True)
    for candidate in ('rx_decode', 'staged'):
        planned = schedule(['small', 'medium', 'large'], 'serial', candidate)
        counts = Counter((row['scene'], row['mode']) for row in planned)
        check(candidate + '_32_independent_processes_every_scene_mode', set(counts.values()) == {32} and len(planned) == 192)
        check(candidate + '_ABBA_BAAB_alternation', [r['mode'] for r in planned[:8]] == ['serial', candidate, candidate, 'serial', candidate, 'serial', 'serial', candidate])
    result = {'schema': 'go2_pipeline_v19_finite_input_reader_tests/v1', 'status': 'passed_finite_synthetic_only',
              'actual_capture_or_pipeline_or_performance_PASS': False, 'tests': tests}
    out = Path(__file__).resolve().parent / 'FINITE_INPUT_READER_TESTS.json'
    with out.open('x') as f: json.dump(result, f, indent=2); f.write('\n')
    print(json.dumps({'finite_cases': len(tests), 'out': str(out), 'actual_PASS': False}))


if __name__ == '__main__': main()
