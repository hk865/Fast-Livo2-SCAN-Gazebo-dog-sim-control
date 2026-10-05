#!/usr/bin/env python3
"""Original CDR metadata-only index. No node, sample generation, or run mutation."""
from pathlib import Path
import collections, hashlib, json, struct

ROOT=Path(__file__).resolve().parents[2]
cases={}
for name in ['20261002_cm_limits_first8_a_false','20261002_cm_limits_first8_b_true']:
    run=ROOT/'simulation/test_results'/name
    result=json.loads((run/'first_eight_result.json').read_text())
    start,end=result['origin_stamp_ns'],result['terminal_stamp_ns']
    fixed=('clock','imu','measured_joint','legacy_joint','jtc','truth','slam')
    prev={}; gaps={k:[] for k in fixed}; allprev={}; allbad={k:[] for k in fixed}
    lost=[]; counts=collections.Counter()
    expected={'clock':1_000_000,'imu':1_000_000,'measured_joint':1_000_000,'legacy_joint':4_000_000,'jtc':4_000_000}
    with (run/'actuator/actuator_suffix.cdrlog').open('rb') as f:
        magic=b'ACTUATORSUFFIX1\n'
        assert f.read(len(magic))==magic
        while b:=f.read(8):
            assert len(b)==8
            hn,cn=struct.unpack('<II',b)
            h=json.loads(f.read(hn)); f.seek(cn,1)
            k=h['kind']; ns=h.get('header_stamp_ns')
            ns=h['clock_ns'] if ns is None or ns==0 else ns
            if k=='middleware_lost':
                lost.append(h)
            if k not in fixed:
                continue
            if k in allprev and k in expected and ns-allprev[k]!=expected[k]:
                allbad[k].append([allprev[k],ns,ns-allprev[k]])
            allprev[k]=ns
            if not start<=ns<=end:
                continue
            counts[k]+=1
            if k in prev and k in expected and ns-prev[k]!=expected[k]:
                gaps[k].append([prev[k],ns,ns-prev[k]])
            prev[k]=ns
    cases[name]={
        'active_interval_ns':[start,end], 'active_metadata_counts':dict(counts),
        'active_fixed_header_gaps_ns':gaps, 'all_observed_fixed_header_gaps_ns':allbad,
        'all_middleware_loss_callbacks_original_metadata':lost,
        'bounds':'Sparse foot contact publisher gaps are not assigned missing-message timestamps. Loss callback clock_ns associates receiver latestclock, not a known lost publisher header.'
    }
dest=ROOT/'slam/test_results/oct2_CM_limit_AB_all_active_header_loss_locations.json'
report={'scope':'Read-only original CDR metadata index; payload schemas validated independently in final failed windows. No ROS/node or sample fabrication.','cases':cases}
dest.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'path':str(dest),'SHA':hashlib.sha256(dest.read_bytes()).hexdigest(),'cases':{n:{'active_fixed_gaps':d['active_fixed_header_gaps_ns'],'lost':[{k:r.get(k) for k in ['source','clock_ns','sequence','total_count','total_count_change']} for r in d['all_middleware_loss_callbacks_original_metadata']]} for n,d in cases.items()}}))
