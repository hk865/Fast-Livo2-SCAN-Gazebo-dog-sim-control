#!/usr/bin/env python3
"""Small V19 reader boundary fixtures, never actual runtime evidence."""
import copy
import csv
import json
import tempfile
from pathlib import Path
import read_stages as reader


def fixture(mode):
    rows = []
    owner = 1 if mode == 'serial' else 4
    for sequence, kind in enumerate((1, 2, 3, 0, 1, 2, 3, 0), 1):
        t, cpu = 1_000_000_000 + sequence * 1000, sequence * 100
        r = {key: 0 for key in reader.COLUMNS}
        r.update(sequence=sequence, kind=kind, source_ns=0 if kind == 0 else sequence * 1_000_000,
                 receipt_wall_ns=t, raw_enqueue_wall_ns=t+10,
                 ready_enqueue_wall_ns=t+50 if kind >= 2 else t+10,
                 owner_pop_wall_ns=t+60, commit_end_wall_ns=t+90,
                 receive_tid=1, owner_tid=owner, receive_cpu_begin_ns=cpu+1,
                 receive_cpu_end_ns=cpu+10, owner_cpu_begin_ns=cpu+40,
                 owner_cpu_end_ns=cpu+50, reserved_bytes=64,
                 pending_after_commit=0, bytes_after_commit=0)
        if kind >= 2:
            r.update(decoder_pop_wall_ns=t+20, decode_begin_wall_ns=t+30,
                     decode_end_wall_ns=t+40, decode_tid=kind if mode == 'staged' else 1,
                     decode_cpu_begin_ns=cpu+20, decode_cpu_end_ns=cpu+30)
        rows.append(r)
    n = len(rows)
    s = {key: 0 for key in reader.SUMMARY_INTS}
    s.update(schema=reader.SUMMARY_SCHEMA, mode=mode, trace_schema=reader.TRACE_SCHEMA,
             trace_columns=len(reader.COLUMNS), count_limit=512, byte_limit=67108864,
             accepted=n, delivered=n, committed=n, peak_pending=1, peak_bytes=64,
             normal_completed=True, context_valid_at_drain=True, failure='',
             uncommitted_packets=[], rejected_packet=None)
    lifecycle = []
    for i, event in enumerate(reader.LIFE_NORMAL):
        r = {key: 0 for key in reader.LIFE_COLUMNS}; r.update(event=event, tid=owner, context_valid=1)
        if i == 0: r.update(wall_ns=1_000_000_000, thread_cpu_ns=1)
        else: r.update(wall_ns=rows[-1]['commit_end_wall_ns']+i*100,
                       thread_cpu_ns=1000+i*100, accepted=n, delivered=n, committed=n)
        lifecycle.append(r)
    return rows, lifecycle, s


def main():
    tests = []
    def ck(name, value):
        if not value: raise RuntimeError('FAILED finite reader test ' + name)
        tests.append({'name': name, 'status': 'passed', 'synthetic_only': True})
    for mode in reader.MODES:
        rows, life, summary = fixture(mode)
        result = reader.analyze(rows, life, summary)
        ck(mode + '_valid_limited_trace', result['status'] == 'passed_limited_trace_and_lifecycle')
        ck(mode + '_IMU_decode_NA_not_zero_latency', result['metrics']['imu']['decode_wall']['status'] == 'unavailable')
        ck(mode + '_no_actual_navigation_claim', result['actual_navigation_or_Sim2Sim_PASS'] is False and result['runtime_source_binding_checked_by_this_reader'] is False)
    rows, life, s = fixture('staged')
    changes = []
    x = copy.deepcopy(s); x['canceled'] = 1; changes.append(('tail_canceled_fail', rows, life, x))
    x = copy.deepcopy(s); x['closed_rejections'] = 1; changes.append(('post_close_rejection_not_hidden_by_producer_normal_true', rows, life, x))
    x = copy.deepcopy(s); x['rejected_capacity'] = 1; changes.append(('capacity_reject_fail', rows, life, x))
    x = copy.deepcopy(s); x['accepted'] += 1; changes.append(('missing_accepted_event_fail', rows, life, x))
    x = copy.deepcopy(rows); x[1]['sequence'] += 1; changes.append(('sequence_gap_fail', x, life, s))
    x = copy.deepcopy(rows); x[1]['decode_tid'] = x[1]['owner_tid']; changes.append(('foreign_thread_partition_fail', x, life, s))
    x = copy.deepcopy(rows); x[0]['decode_tid'] = 2; changes.append(('invented_IMU_decoder_fail', x, life, s))
    x = copy.deepcopy(rows); x[1]['decode_end_wall_ns'] = x[1]['decode_begin_wall_ns']-1; changes.append(('backward_stage_wall_fail', x, life, s))
    x = copy.deepcopy(rows); x[1]['decode_cpu_end_ns'] = x[1]['decode_cpu_begin_ns']-1; changes.append(('backward_threadCPU_fail', x, life, s))
    x = copy.deepcopy(rows); x[1]['reserved_bytes'] = 67108865; changes.append(('changed_byte_bound_fail', x, life, s))
    x = copy.deepcopy(life); x[-1]['context_valid'] = 0; changes.append(('invalid_context_drain_fail', rows, x, s))
    changes.append(('missing_drain_event_fail', rows, life[:-1], s))
    x = copy.deepcopy(s); x['normal_completed'] = False; changes.append(('emergency_abort_fail', rows, life, x))
    x = copy.deepcopy(s); x['uncommitted_packets'] = [{'seq': 9, 'kind': 0}]; changes.append(('uncommitted_identity_fail', rows, life, x))
    changes.append(('missing_image_sensor_fail', [r for r in rows if r['kind'] != 3], life, s))
    for name, a, b, c in changes:
        ck(name, reader.analyze(a, b, c)['status'] == 'failed')
    here = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix='finite_', dir=here) as tmp:
        tmp = Path(tmp)
        csv_path = tmp / 'events.csv'
        with csv_path.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=reader.COLUMNS); w.writeheader(); w.writerows(rows)
        ck('strict_CSV_roundtrip', reader.read_csv(csv_path, reader.COLUMNS) == rows)
        csv_path.write_text('sequence,kind,source_ns,receipt_wall\n1,2,1,1\n')
        try: reader.read_csv(csv_path, reader.COLUMNS); ck('old_V17_CSV_rejected', False)
        except ValueError: ck('old_V17_CSV_rejected', True)
        json_path = tmp / 'summary.json'; json_path.write_text(json.dumps(s))
        ck('strict_summary_roundtrip', reader.read_summary(json_path) == s)
        foreign = dict(s, schema='ordered_ingress_v17'); json_path.write_text(json.dumps(foreign))
        try: reader.read_summary(json_path); ck('foreign_summary_rejected', False)
        except ValueError: ck('foreign_summary_rejected', True)
        foreign = dict(s); del foreign['closed_rejections']; json_path.write_text(json.dumps(foreign))
        try: reader.read_summary(json_path); ck('missing_summary_counter_rejected', False)
        except ValueError: ck('missing_summary_counter_rejected', True)
    result = {'schema': 'go2_pipeline_v19_finite_stage_reader_tests/v1',
              'status': 'passed_finite_synthetic_only', 'actual_runtime_or_performance_PASS': False, 'tests': tests}
    out = here / 'FINITE_STAGE_READER_TESTS.json'
    with out.open('x') as f: json.dump(result, f, indent=2); f.write('\n')
    print(json.dumps({'finite_cases': len(tests), 'out': str(out), 'actual_PASS': False}))


if __name__ == '__main__': main()
