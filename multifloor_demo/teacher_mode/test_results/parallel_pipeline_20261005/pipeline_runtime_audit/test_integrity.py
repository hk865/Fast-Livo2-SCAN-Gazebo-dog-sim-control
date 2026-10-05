#!/usr/bin/env python3
"""Finite fabricated *negative tests*, never an actual pipeline PASS."""
from pathlib import Path
import collections,copy,importlib.util,json
HERE=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('pipeline_reader',HERE/'audit_pipeline.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
rows=[];raw=collections.defaultdict(list)
for i,kind in enumerate([1,2,3,0],1):
    r=dict(sequence=i,kind=kind,source_ns=1000000*i if kind else 0,receipt_wall=100+i*.1,decoded_wall=100+i*.1+.001 if kind else 100+i*.1,pop_wall=100+i*.1+.002,commit_end_wall=100+i*.1+.003,receive_tid=31,owner_tid=17,decode_cpu_begin_ns=i*100000,decode_cpu_end_ns=i*100000+500 if kind else i*100000,pending_after_pop=0,bytes_after_pop=0)
    rows.append(r)
    if kind:raw[kind].append(dict(header=(kind,0,r['source_ns'],0,0,0,7),values=(r['receipt_wall'],),record_index=i))
summary=dict(schema='ordered_ingress_v17',accepted=4,delivered=4,committed=4,canceled=0,rejected=0,peak_pending=1,peak_bytes=1000,failure='')
cases=[]
def test(name,mutate=None,expected=True):
    a,b,c=copy.deepcopy(rows),copy.deepcopy(summary),copy.deepcopy(raw)
    if mutate:mutate(a,b,c)
    checks=m.integrity(a,b,c);status=m.overall(checks);passed=(status=='passed')==expected
    cases.append(dict(case=name,expected='finite reference pass'if expected else'finite rejection',status=status,passed=passed,failed_checks=[k for k,v in checks.items()if v['status']=='failed']))
    assert passed,cases[-1]
test('finite_reference_only_not_actual')
test('canceled_at_cleanup_is_not_exempt',lambda a,b,c:b.update(accepted=5,canceled=1),False)
test('rejected_is_not_missing_zero',lambda a,b,c:b.update(rejected=1),False)
test('delivered_uncommitted_owner_exception',lambda a,b,c:b.update(delivered=5,accepted=5),False)
test('missing_original_raw_sensor',lambda a,b,c:c[3].clear(),False)
test('wrong_original_integer_header',lambda a,b,c:c[2][0].update(header=(2,0,999999,0,0,0,7)),False)
test('refreshed_receipt_owner_pop',lambda a,b,c:c[1][0].update(values=(a[0]['pop_wall'],)),False)
test('foreign_receiver_thread',lambda a,b,c:a[1].update(receive_tid=32),False)
test('owner_is_receiver',lambda a,b,c:[x.update(owner_tid=31)for x in a],False)
test('sequence_gap',lambda a,b,c:a[1].update(sequence=8),False)
test('reordered_committed_sensor_raw',lambda a,b,c:c[1][0].update(record_index=99),False)
test('clock_backwards',lambda a,b,c:a[1].update(pop_wall=a[1]['decoded_wall']-.1),False)
test('CPU_clock_backwards',lambda a,b,c:a[1].update(decode_cpu_end_ns=0),False)
test('RX_callbacks_overlap',lambda a,b,c:a[1].update(receipt_wall=a[0]['decoded_wall']-.0001),False)
test('owner_callbacks_overlap',lambda a,b,c:a[1].update(pop_wall=a[0]['commit_end_wall']-.0001),False)
test('queue_pending_over_bound',lambda a,b,c:a[1].update(pending_after_pop=512,bytes_after_pop=100),False)
test('queue_bytes_over_bound',lambda a,b,c:a[1].update(pending_after_pop=1,bytes_after_pop=67108865),False)
test('queue_zero_pending_nonzero_bytes',lambda a,b,c:a[1].update(bytes_after_pop=100),False)
test('peak_count_inconsistent',lambda a,b,c:b.update(peak_pending=0),False)
test('peak_bytes_inconsistent',lambda a,b,c:b.update(peak_bytes=0),False)
test('summary_overflow_failure',lambda a,b,c:b.update(failure='ingress queue capacity exceeded'),False)
test('timer_nonzero_source',lambda a,b,c:a[-1].update(source_ns=999),False)
report=dict(schema='independent_V17_finite_runtime_reader_negative_tests/v1',status='PASS_FINITE_TESTS_ONLY',actual_runtime_claim=False,case_count=len(cases),cases=cases,reader_sha256=m.sha(HERE/'audit_pipeline.py'))
with(HERE/'finite_reader_tests.json').open('x')as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({'status':report['status'],'cases':len(cases)}))
