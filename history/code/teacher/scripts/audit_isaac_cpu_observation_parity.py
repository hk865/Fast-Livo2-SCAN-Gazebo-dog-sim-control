#!/usr/bin/env python3
"""Reconstruct247D inputs from recorded reference states, preserving reset mismatches."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'policy'))
from observation import build_observation,TerrainHeightMap
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();out=args.run.resolve()
protocol=json.loads((out/'protocol.json').read_text());terrain=TerrainHeightMap.from_sdf(out/'flat_reference.sdf')
actor=torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12))
checkpoint=torch.load(protocol['checkpoint'],map_location='cpu',weights_only=False)
actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in checkpoint['actor_state_dict'].items() if k.startswith('mlp.')});actor.eval();torch.set_num_threads(1)
allobs=[];allactions=[];resets=[];scandiffs=[];cases=[];rawheight=[];rows=[]
for path in sorted(out.glob('*_trace.json')):
    trace=json.loads(path.read_text());case_start=len(allobs);actualobs=[]
    for i,r in enumerate(trace):
        state=np.zeros(64);state[0]=r['t'];state[1:4]=r['position'];x,y,z,w=r['quaternion_xyzw'];state[4:8]=[w,x,y,z];state[8:11]=r['linear_velocity_body_com'];state[11:14]=r['angular_velocity_body'];state[14:26]=r['q'];state[26:38]=r['qd'];state[38:50]=r['torque']
        last=trace[i-1]['action'] if i else np.zeros(12)
        obs,extra=build_observation(state,r['command'],last,terrain)
        allobs.append(obs);allactions.append(r['action']);resets.append(i==0);cases.append(path.stem.removesuffix('_trace'));rawheight.append(extra['height_scan_raw'])
        scandiffs.append(float(np.max(abs(obs[60:]-np.asarray(r['height_scan'])))))
        if 'observation247' in r:actualobs.append(r['observation247'])
    selected=[r for r in trace if 5<=r['t']<10]
    v=np.asarray([np.r_[r['linear_velocity_body_com'][:2],r['angular_velocity_body'][2]] for r in selected]);cmd=np.asarray([r['command'] for r in selected])
    rows.append({'name':cases[-1],'frame_slice':[case_start,len(allobs)],'steady_window_seconds':[5,10], 'mean_actual_vx_vy_wz':v.mean(0).tolist() if len(v) else None,'rmse_vx_vy_wz':np.sqrt(((v-cmd)**2).mean(0)).tolist() if len(v) else None,'actual_full_obs_recorded':len(actualobs)})
allobs=np.asarray(allobs);allactions=np.asarray(allactions);resets=np.asarray(resets)
with torch.inference_mode(): predicted=actor(torch.from_numpy(allobs)).numpy()
actiondiff=np.max(abs(predicted-allactions),axis=1)
for row in rows:
    start,end=row['frame_slice'];row['reset_frame_action_max_error']=float(actiondiff[start]);row['nonreset_action_max_error']=float(actiondiff[start+1:end].max())
np.savez_compressed(out/'reconstructed_observation_parity.npz', reconstructed_observation247=allobs, recorded_action=allactions,reconstructed_actor_action=predicted,reconstructed_raw_height=rawheight,reset_frame_mask=resets,cases=np.asarray(cases))
result={'scope':'All recorded frames, reconstruct247 observation from physical snapshots, last raw actions, synthetic equivalent flat SDF. Raw full247 inputs were not saved in initial baseline; do not present reconstructed arrays as directly recorded Isaac obs.', 'frames':len(allobs),'reset_frames':int(resets.sum()),'nonreset_frames':int((~resets).sum()),'max_scan_error':max(scandiffs),'max_action_error_all':float(actiondiff.max()),'max_action_error_nonreset':float(actiondiff[~resets].max()),'max_action_error_reset':float(actiondiff[resets].max()),'tolerance_height':1e-5,'tolerance_action':2e-5,'passed_all_frames':bool(max(scandiffs)<1e-5 and (actiondiff<2e-5).all()),'passed_nonreset_frames':bool(max(scandiffs)<1e-5 and (actiondiff[~resets]<2e-5).all()),'reset_discrepancy':'Initial reference PhysX reset snapshot and actor input disagree for frame0; raw obs was not recorded, so exact source of reset snapshot timing mismatch remains unverified. Preserve as failed initialization parity until full247 replay confirms.', 'audit_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'observation_module_sha256':hashlib.sha256((ROOT/'policy/observation.py').read_bytes()).hexdigest(),'contract_sha256':hashlib.sha256((ROOT/'policy/contract.json').read_bytes()).hexdigest(),'rows':rows}
(out/'observation_parity_full.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
