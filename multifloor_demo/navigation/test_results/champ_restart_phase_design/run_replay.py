#!/usr/bin/env python3
"""Offline native-header replay; private candidate only, no ROS or physics."""
import difflib
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
INSTALL=ROOT/'go2_sim_control/install/champ/include/champ'
RUN=ROOT/'multifloor_demo/simulation/test_results/20261002_classic_lowD_align_first8_candidate'
URDF=RUN/'staging/go2_measured.urdf'

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write_json(path, obj): path.write_text(json.dumps(obj,indent=2,allow_nan=False,
    default=lambda v:v.item() if isinstance(v,np.generic) else str(v))+'\n')

def main():
    original=(INSTALL/'leg_controller/phase_generator.h').read_text()
    remove='                    swing_phase_signal[1] = 0.0;\n                    swing_phase_signal[2] = 0.0;\n'
    assert original.count(remove)==1
    candidate=original.replace(remove,'')
    header=HERE/'private_include/leg_controller/phase_generator.h'
    header.write_text(candidate)
    patch=''.join(difflib.unified_diff(original.splitlines(True),candidate.splitlines(True),
                  fromfile='original/leg_controller/phase_generator.h',
                  tofile='candidate/leg_controller/phase_generator.h'))
    (HERE/'candidate.patch').write_text(patch)

    # Match local URDF::getPose's cumulative joint-origin translations. The
    # relevant links/joints have no intervening rotated or offset fixed link.
    tree=ET.parse(URDF).getroot(); joints={j.attrib['name']:j for j in tree.findall('joint')}
    origins=[]; origin_receipt=[]
    for leg in ('lf','rf','lh','rh'):
        row=[]
        for suffix in ('hip_joint','upper_leg_joint','lower_leg_joint','foot_joint'):
            j=joints[f'{leg}_{suffix}']; o=j.find('origin')
            xyz=[float(x) for x in o.attrib.get('xyz','0 0 0').split()]
            rpy=[float(x) for x in o.attrib.get('rpy','0 0 0').split()]
            assert rpy==[0,0,0]
            row.append(xyz)
            origin_receipt.append({'joint':j.attrib['name'],'xyz':xyz,'rpy':rpy})
        origins.append(row)
    (HERE/'geometry_input.h').write_text('static const float origins[4][4][3] = '+
        json.dumps(origins).replace('[','{').replace(']','}')+';\n')

    cmds=[]
    command_log=RUN/'joint_stop_adapter.jsonl'
    with command_log.open() as f:
        for line_no,line in enumerate(f,1):
            r=json.loads(line)
            if r.get('kind')=='actual_champ_command' and 323.191-1e-8<=r.get('sim',0)<=324.391+1e-8:
                v=r['value']; cmds.append({'time_us':round(r['sim']*1e6),'vx':v[0],'vy':v[1],
                                         'wz':v[5],'line':line_no,'original':r})
    assert cmds[0]['time_us']==323191000
    write_json(HERE/'actual_input_extract.json',{'source_path':str(command_log),
        'source_sha256':sha(command_log),'rows':cmds,
        'limit':'Observed Adapter emission timestamps, not recorded CHAMP callback/phase. ZOH inference only.'})
    build=[]
    for name,extra in (('original',[]),('candidate',['-I'+str(HERE/'private_include')])):
        cmd=['/usr/bin/c++','-O3','-DNDEBUG','-std=c++17','-I'+str(HERE),*extra,
             '-I'+str(INSTALL),str(HERE/'replay.cpp'),'-o',str(HERE/f'replay_{name}')]
        proc=subprocess.run(cmd,capture_output=True,text=True)
        (HERE/f'build_{name}.log').write_text(proc.stdout+proc.stderr)
        if proc.returncode: raise RuntimeError(proc.stderr)
        build.append({'argv':cmd,'returncode':proc.returncode,'binary_sha256':sha(HERE/f'replay_{name}')})
    cases=[('actual_restart',323191000,1,cmds),
           ('forward_cap',323191000,1,[{'time_us':323191000,'vx':.12,'vy':0,'wz':0}]),
           ('pure_yaw_positive',323191000,1,[{'time_us':323191000,'vx':0,'vy':0,'wz':.12}]),
           ('pure_yaw_negative',323191000,1,[{'time_us':323191000,'vx':0,'vy':0,'wz':-.12}]),
           ('tiny_nonzero_forward',323191000,1,[{'time_us':323191000,'vx':1e-6,'vy':0,'wz':0}]),
           ('early_first_boot',100000,0,[{'time_us':100000,'vx':.12,'vy':0,'wz':0}]),
           ('early_zero_restart',200000,1,[{'time_us':200000,'vx':.12,'vy':0,'wz':0}])]
    summary={}; checks={}; all_rows={}
    for case,onset,warm,inputs in cases:
        ip=HERE/f'input_{case}.txt'
        ip.write_text(''.join(f"{r['time_us']} {r['vx']:.17g} {r['vy']:.17g} {r['wz']:.17g}\n" for r in inputs))
        for dt in (1000,4000):
            outputs={}
            for profile in ('original','candidate'):
                out=HERE/f'{case}_{dt}us_{profile}.jsonl'
                with out.open('w') as f:
                    subprocess.run([str(HERE/f'replay_{profile}'),str(ip),str(onset),str(dt),'1200000',str(warm)],
                                   stdout=f,check=True,timeout=10)
                outputs[profile]=[json.loads(l) for l in out.read_text().splitlines()]
            key=f'{case}_{dt}us'; all_rows[key]=outputs
            a=outputs['original']; b=outputs['candidate']; sub={}
            for profile,rows in outputs.items():
                ar=np.array([r['foot_hip'] for r in rows]); qs=np.array([r['q'] for r in rows])
                assert np.isfinite(ar).all() and np.isfinite(qs).all()
                initial=np.array(rows[0]['foot_hip']); tt=np.array([r['elapsed_us'] for r in rows])
                idx=(tt>=0)&(tt<300000)
                dq=np.abs(np.diff(qs,axis=0)); dp=np.diff(ar,axis=0).reshape(-1,4,3)
                onsetidx=np.flatnonzero((tt>0)&(tt<300000)&np.array([r['swing'][1]>0 for r in rows]))
                sub[profile]={'RF_first_nonzero_swing_elapsed_us':int(tt[onsetidx[0]]) if len(onsetidx) else None,
                    'max_first300ms_joint_step_rad':float(dq[idx[1:]].max()),
                    'max_first300ms_foot_step_m':float(np.linalg.norm(dp[idx[1:]],axis=2).max()),
                    'initial_q':rows[0]['q'],
                    'boundary_rows':[r for r in rows if 48000<=r['elapsed_us']<=56000 or
                                      172000<=r['elapsed_us']<=180000 or 296000<=r['elapsed_us']<=304000]}
            phase_equal=[]; foot_equal=[]
            for x,y in zip(a,b):
                if x['elapsed_us']>=175000:
                    phase_equal.append(x['stance']==y['stance'] and x['swing']==y['swing'])
                    foot_equal.append(x['foot_hip']==y['foot_hip'] and x['q']==y['q'])
            sub['same_after_175ms']={'phase_exact':all(phase_equal),'foot_q_exact':all(foot_equal),
                                     'checked_rows':len(phase_equal)}
            summary[key]=sub
            if not case.startswith('early_zero'):
                checks[f'{key}_finite']=True
                checks[f'{key}_subsequent_phase_foot_ik_exact']=all(phase_equal) and all(foot_equal)
            if case in ('actual_restart','forward_cap','pure_yaw_positive','pure_yaw_negative','tiny_nonzero_forward'):
                checks[f'{key}_RF_swing_starts_before_midpoint']=sub['candidate']['RF_first_nonzero_swing_elapsed_us']<60000
    # Exact phase0 retains prev_foot; next sample retains a small x endpoint jump.
    rows=all_rows['forward_cap_1000us']['candidate']; by={r['elapsed_us']:r for r in rows}
    p50=np.array(by[50000]['foot_hip']).reshape(4,3); p51=np.array(by[51000]['foot_hip']).reshape(4,3)
    p174=np.array(by[174000]['foot_hip']).reshape(4,3); p175=np.array(by[175000]['foot_hip']).reshape(4,3)
    checks['exact_50ms_phase0_retains_prev']=by[50000]['swing'][1]==0 and np.array_equal(p50[1],np.array(rows[0]['foot_hip']).reshape(4,3)[1])
    checks['horizontal_endpoint_jump_is_not_hidden']=abs(p51[1,0]-p50[1,0])>.02
    checks['LF_RH_stance_depth_boundary_is_unchanged']=abs(p175[0,2]-p174[0,2]+.005)<1e-7
    checks['RF_LH_lift_midpoint_jump_removed']=np.max(np.abs(p175[[1,2],2]-p174[[1,2],2]))<.001
    checks['nominal_source_IK_matches_archived_reference']=float(np.max(np.abs(np.array(rows[0]['q'])-np.array(
        json.loads((ROOT/'multifloor_demo/simulation/test_results/classic_lowD_align_first8_audit/restart_target_jump.json').read_text())['original_raw_targets'][0]['message']['points'][0]['positions']))))<1e-6
    early=all_rows['early_zero_restart_1000us']['original']
    checks['early_restart_existing_carry_is_explicit']=early[1]['elapsed_us']==0 and early[1]['swing'][1]>.5
    # Zero occurs after arbitrary moving samples, then a large-time restart.
    zero_input=HERE/'input_zero_restart.txt'
    zero_input.write_text('323000000 .12 0 0\n323123000 0 0 0\n324000000 0 0 .12\n')
    zero_summary={}
    for profile in ('original','candidate'):
        out=HERE/f'zero_restart_{profile}.jsonl'
        with out.open('w') as f:
            subprocess.run([str(HERE/f'replay_{profile}'),str(zero_input),'323000000','1000','1400000','1'],stdout=f,check=True,timeout=10)
        rr=[json.loads(l) for l in out.read_text().splitlines()]
        idle=[r for r in rr if 323123000<=r['time_us']<324000000]
        nominal=rr[0]['q']; safe=all(r['swing']==[0,0,0,0] and r['stance']==[0,0,0,0] and r['q']==nominal for r in idle)
        active=[r for r in rr if r['time_us']>=324000000 and r['swing'][1]>0]
        zero_summary[profile]={'idle_samples':len(idle),'all_idle_nominal_zero_signals':safe,
            'RF_first_swing_after_restart_us':active[0]['time_us']-324000000}
        checks[f'{profile}_arbitrary_zero_restores_nominal']=safe
    receipt={'scope':'Offline compiled harness calls installed CHAMP PhaseGenerator, TrajectoryPlanner, BodyController and IK. Only candidate private phase header changes; not physics or production build.',
        'candidate_diff':patch,'source_receipt_path':str(HERE.parent/'champ_restart_phase_source_audit/source_receipt.json'),
        'source_receipt_sha256':sha(HERE.parent/'champ_restart_phase_source_audit/source_receipt.json'),
        'actual_input_path':str(HERE/'actual_input_extract.json'),'actual_input_sha256':sha(HERE/'actual_input_extract.json'),
        'URDF_path':str(URDF),'URDF_sha256':sha(URDF),'origins':origin_receipt,
        'builds':build,'compiler_version':subprocess.check_output(['/usr/bin/c++','--version'],text=True).splitlines()[0],
        'harness_sha256':sha(HERE/'replay.cpp'),'driver_sha256':sha(Path(__file__)),
        'private_candidate_header_sha256':sha(header),'checks':checks,'checks_passed':all(checks.values()),
        'summary':summary,'zero_restart':zero_summary,
        'limits':['Input ZOH uses Adapter emission times, not captured CHAMP callback timing.',
                  '1ms and4ms samples are deterministic source checks; production CHAMP wall timer is5ms.',
                  'Candidate removes midpoint lift discontinuity only. First horizontal endpoint and LF/RH stance_depth edges remain.',
                  'Early absolute clock <stride restart carry is existing and unchanged.',
                  'No applied torque, contact, stability, new physics or production fix is demonstrated.']}
    write_json(HERE/'result.json',receipt)
    print(json.dumps({'checks_passed':receipt['checks_passed'],'checks':len(checks),
                      'result_sha256':sha(HERE/'result.json')},indent=2))

if __name__=='__main__':main()
