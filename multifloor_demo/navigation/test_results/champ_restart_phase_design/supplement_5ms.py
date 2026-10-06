#!/usr/bin/env python3
"""One actual-input 5ms supplement; preserves the original 42-check receipt."""
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    prior=HERE/'result.json'; before=sha(prior)
    assert before=='d3823a3d471ea2a39904cc7d18b5c686986c0282cf99681f888241b06d21d0a3'
    results={}; inputs={}; allrows={}
    for profile in ('original','candidate'):
        binary=HERE/f'replay_{profile}'; output=HERE/f'actual_restart_5000us_{profile}.jsonl'
        argv=[str(binary),str(HERE/'input_actual_restart.txt'),'323191000','5000','1200000','1']
        with output.open('w') as f: subprocess.run(argv,stdout=f,check=True,timeout=10)
        rr=[json.loads(l) for l in output.read_text().splitlines()];allrows[profile]=rr
        qq=np.array([r['q'] for r in rr]);pp=np.array([r['foot_hip'] for r in rr]).reshape(-1,4,3)
        times=np.array([r['elapsed_us'] for r in rr]); in300=(times[1:]>=0)&(times[1:]<300000)
        deltaq=np.max(np.abs(np.diff(qq,axis=0)),axis=1)
        deltap=np.max(np.linalg.norm(np.diff(pp,axis=0),axis=2),axis=1)
        qidx=np.flatnonzero(in300)[np.argmax(deltaq[in300])]+1
        pidx=np.flatnonzero(in300)[np.argmax(deltap[in300])]+1
        swingfirst=next(r['elapsed_us'] for r in rr if r['elapsed_us']>0 and r['swing'][1]>0)
        results[profile]={'first_RF_LH_swing_elapsed_us':swingfirst,
            'max_first300ms_joint_step_rad':float(deltaq[in300].max()),
            'max_joint_step_ends_elapsed_us':int(times[qidx]),
            'max_first300ms_foot_step_m':float(deltap[in300].max()),
            'max_foot_step_ends_elapsed_us':int(times[pidx]),
            'boundary_rows':[r for r in rr if r['elapsed_us'] in (45000,50000,55000,170000,175000,180000)],
            'native_replay_binary_sha256':sha(binary),'output_path':str(output),'output_sha256':sha(output),
            'argv':argv}
    a=allrows['original'];b=allrows['candidate']
    equal=[(x['stance']==y['stance'] and x['swing']==y['swing'] and
            x['foot_hip']==y['foot_hip'] and x['q']==y['q'])
           for x,y in zip(a,b) if x['elapsed_us']>=175000]
    by={r['elapsed_us']:r for r in b}
    checks={'original_42check_receipt_unchanged':sha(prior)==before,
        'candidate_RF_LH_starts_at55ms_not175ms':results['candidate']['first_RF_LH_swing_elapsed_us']==55000,
        'original_RF_LH_first175ms':results['original']['first_RF_LH_swing_elapsed_us']==175000,
        'candidate_50ms_boundary_prev_foot':by[50000]['foot_hip']==by[45000]['foot_hip'],
        'all_phase_foot_IK_exact_from175ms':all(equal),
        'all_emitted_foot_q_finite':all(np.isfinite(np.array([r[k] for r in rr])).all()
              for rr in (a,b) for k in ('foot_hip','q')),
        'source_midpoint_lift_removed_not_full_C0_claim':results['candidate']['max_first300ms_joint_step_rad']<.05}
    receipt={'scope':'One 5ms direct-header actual Adapter-input ZOH replay, no ROS/physics/production build.',
        'original_42check_result_sha256':before,'supplement_script_sha256':sha(Path(__file__)),
        'actual_input_extract_sha256':sha(HERE/'actual_input_extract.json'),
        'candidate_patch_sha256':sha(HERE/'candidate.patch'),
        'candidate_private_header_sha256':sha(HERE/'private_include/leg_controller/phase_generator.h'),
        'results':results,'checked_rows_from175ms':len(equal),'checks':checks,'passed':all(checks.values()),
        'limit':'5ms is the production wall-timer interval, while phase uses ROS simulation time. This fixed5ms simulation-grid replay is not the unrecorded actual callback schedule.'}
    (HERE/'supplement_5ms_result.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'passed':receipt['passed'],'checks':len(checks),'result_sha256':sha(HERE/'supplement_5ms_result.json'),
        'original':{k:v for k,v in results['original'].items() if k.startswith('max_')},
        'candidate':{k:v for k,v in results['candidate'].items() if k.startswith('max_')}}))
if __name__=='__main__':main()
