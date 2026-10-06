#!/usr/bin/env python3
"""Evaluate preregistered conditions without hiding incomplete/failed commands."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def evaluate(run):
    protocol=json.loads((ROOT/'tests/protocol.json').read_text())
    meta=json.loads((run/'policy_manifest.json').read_text())if (run/'policy_manifest.json').exists()else {}
    result=json.loads((run/'worker_result.json').read_text())if (run/'worker_result.json').exists()else {}
    rows=[json.loads(s)for s in (run/'telemetry.jsonl').read_text().splitlines()]if (run/'telemetry.jsonl').exists()else []
    name=meta.get('test',run.name);reasons=[];metrics={};motion_pass=False
    expected=26 if name=='switch' else 20 if name.startswith('step')else 15 if name=='stand'else 18
    if not rows:reasons.append('No completed physics observation')
    else:
        t=np.array([r['sim_time']for r in rows]);pos=np.array([r['position']for r in rows]);cmd=np.array([r['command']for r in rows]);v=np.array([r['measured']for r in rows]);rpy=np.array([r['rpy']for r in rows]);contact=np.array([list(r['contacts'].values())for r in rows]);clear=np.array([r['body_clearance']for r in rows])
        active=t>=3
        metrics.update(duration_s=float(t[-1]),samples=len(rows),max_abs_roll_pitch_rad=float(abs(rpy[active,:2]).max())if active.any()else None,
                       min_body_clearance_m=float(clear[active].min())if active.any()else None,
                       body_contact_samples=int((contact[active,0]>0).sum()),unknown_contact_samples=int((contact[active]<0).sum()),
                       max_applied_torque_Nm=float(np.abs([r['applied_torque']for r in rows]).max()),
                       position_delta_m=(pos[-1]-pos[0]).tolist(),total_yaw_delta_rad=float(np.unwrap(rpy[:,2])[-1]-rpy[0,2]))
        if t[-1]<expected-.08:reasons.append('Test ended before required duration')
        if result.get('fault'):reasons.append(result['fault'])
        if name=='stand' and any(r['state']in ['support_hold','support_capture']for r in rows):reasons.append('Invalid stand coverage: commissioning support hold replaced the Teacher')
        if active.any():
            lim=protocol['motion_limits']
            if abs(rpy[active,:2]).max()>lim['max_abs_roll_pitch_rad']:reasons.append('Roll/pitch exceeds criterion')
            if clear[active].min()<lim['min_body_clearance_m']:reasons.append('Body clearance below criterion')
            if (contact[active,0]>0).any():reasons.append('Body ground/object contact')
            if (contact[active]<0).any():reasons.append('Contact evidence missing')
        else:reasons.append('No active motion sample')
        windows=[(5,10)]if name!='switch'else [(5,7.8),(10,12.8),(15,17.8)]
        if name=='stand':windows=[]
        track=[]
        for start,end in windows:
            mask=(t>=start)&(t<end)
            if mask.sum()<50:reasons.append(f'Incomplete tracking window {start}-{end}s');continue
            err=np.sqrt(np.mean((v[mask]-cmd[mask])**2,axis=0));mean=np.mean(v[mask],axis=0);ref=np.mean(cmd[mask],axis=0)
            track.append({'window_s':[start,end],'command_mean':ref.tolist(),'measured_mean':mean.tolist(),'rmse':err.tolist()})
            axis=int(np.argmax(abs(ref)))
            for k in range(3):
                thresh=protocol['tracking']['yaw_rate_rmse_radps']if k==2 else protocol['tracking']['linear_rmse_mps']
                if abs(ref[k])<.01:thresh=protocol['tracking']['cross_yaw_rms_radps']if k==2 else protocol['tracking']['cross_linear_rms_mps']
                if err[k]>thresh:reasons.append(f'Axis {k} tracking/drift exceeds {thresh}')
            if ref[axis]!=0 and mean[axis]*np.sign(ref[axis])<abs(ref[axis])*protocol['tracking']['minimum_requested_axis_ratio']:reasons.append('Requested direction or speed ratio failed')
        metrics['tracking']=track
        if name=='stand':stop=(t>=5)&(t<=14)
        elif name=='switch':stop=(t>=22)&(t<=26)
        elif name=='command_timeout':stop=(t>=11)&(t<=17.8)
        else:stop=(t>=15)&(t<=expected)
        if stop.sum()>=50:
            p=pos[stop];angles=np.unwrap(rpy[stop,2]);drift=float(np.linalg.norm(p[-1,:2]-p[0,:2]));yaw=float(abs(angles[-1]-angles[0]));vrms=np.sqrt(np.mean(v[stop]**2,axis=0))
            metrics['stop']={'translation_drift_m':drift,'yaw_drift_rad':yaw,'velocity_rms':vrms.tolist(),'samples':int(stop.sum())}
            lim=protocol['stand_stop']
            if drift>lim['translation_drift_m']:reasons.append('Stand/stop translation drift')
            if yaw>lim['yaw_drift_rad']:reasons.append('Stand/stop yaw drift')
            if max(vrms[:2])>lim['linear_rms_mps']or vrms[2]>lim['yaw_rate_rms_radps']:reasons.append('Stand/stop residual velocity')
        else:reasons.append('Incomplete stand/stop window')
        if name=='ramp_up'and pos[-1,0]-pos[0,0]<1.5:reasons.append('Insufficient uphill progress')
        if name=='ramp_down'and pos[0,0]-pos[-1,0]<1.5:reasons.append('Insufficient downhill progress')
        if name.startswith('step'):
            h=.05 if name=='step05'else .1
            if pos[-1,0]<protocol['terrain']['step_end_x_m']or pos[-1,2]-pos[(t>=2).argmax(),2]<h*.6:reasons.append('Low step was not crossed with measured height gain')
        motion_pass=not reasons
    interface=bool(rows)and len(rows[0]['q'])==12 and meta.get('checkpoint_sha256')=='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
    summary={'test':name,'levels':{'interface':'passed'if interface else 'failed','motion':'passed'if motion_pass else 'failed','sim2sim':'unverified','navigation':'unverified','real_robot':'unverified'},
        'tests':[{'name':name,'status':'passed'if motion_pass else 'failed','reason':'; '.join(dict.fromkeys(reasons)),'metrics':metrics}],
        'protocol_sha256':hashlib.sha256((ROOT/'tests/protocol.json').read_bytes()).hexdigest(),
        'scope':'Actual Gazebo physics with privileged observations; no navigation localization claim; three independent repetitions still required.'}
    (run/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n');return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();print(json.dumps(evaluate(a.run),ensure_ascii=False))
