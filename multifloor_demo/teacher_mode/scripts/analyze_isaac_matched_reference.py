#!/usr/bin/env python3
"""Analyze directly recorded247 inputs and frozen CPU matched command replay."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'policy'))
from observation import TerrainHeightMap,build_observation
parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();out=args.run.resolve()
protocol=json.loads((out/'protocol.json').read_text());results=json.loads((out/'results.json').read_text())
if results['status']!='completed':raise RuntimeError('Reference rollout has not completed')
criteria=json.loads((out/'analysis_criteria.json').read_text())
(out/'flat_reference.sdf').write_text('<sdf version="1.9"><world name="flat_reference"><model name="flat"><static>true</static><pose>0 0 -0.1 0 0 0</pose><link name="ground"><collision name="ground"><geometry><box><size>1000 1000 0.2</size></box></geometry></collision></link></model></world></sdf>\n')
terrain=TerrainHeightMap.from_sdf(out/'flat_reference.sdf')
actor=torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12));checkpoint=torch.load(protocol['checkpoint'],map_location='cpu',weights_only=False);actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in checkpoint['actor_state_dict'].items() if k.startswith('mlp.')});actor.eval();torch.set_num_threads(1)
all_observed=[];all_rebuilt=[];all_actions=[];teacher_mask=[];reset_mask=[];case_labels=[];rows=[]
for case in protocol['schedule']['cases']:
 name=case['name'];trace=json.loads((out/f'{name}_trace.json').read_text());begin=len(all_observed)
 for i,row in enumerate(trace):
  state=np.zeros(64);state[0]=row['t'];state[1:4]=row['position'];x,y,z,w=row['quaternion_xyzw'];state[4:8]=[w,x,y,z];state[8:11]=row['linear_velocity_body_com'];state[11:14]=row['angular_velocity_body'];state[14:26]=row['q'];state[26:38]=row['qd'];state[38:50]=row['torque']
  last=trace[i-1]['action'] if i else np.zeros(12)
  reconstructed,_=build_observation(state,row['command'],last,terrain)
  all_observed.append(row['observation247']);all_rebuilt.append(reconstructed);all_actions.append(row['action']);teacher_mask.append(row['controller_mode']=='teacher');reset_mask.append(i==0);case_labels.append(name)
 def metrics_window(start,end):
  window=[r for r in trace if start<=r['t']<end]
  if not window:return None
  actual=np.asarray([np.r_[r['linear_velocity_body_com'][:2],r['angular_velocity_body'][2]] for r in window]);command=np.asarray([r['command'] for r in window]);positions=np.asarray([r['position'][:2] for r in window]);quats=np.asarray([r['quaternion_xyzw'] for r in window]);x,y,z,w=quats.T;yaws=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
  return {'window_seconds':[start,end],'frames':len(window),'mean_command_vx_vy_wz':command.mean(0).tolist(),'mean_actual_vx_vy_wz':actual.mean(0).tolist(),'rmse_vx_vy_wz':np.sqrt(((actual-command)**2).mean(0)).tolist(),'mean_xy_speed':float(np.linalg.norm(actual[:,:2],axis=1).mean()),'mean_abs_yaw_rate':float(abs(actual[:,2]).mean()),'xy_drift':float(np.linalg.norm(positions[-1]-positions[0])),'yaw_drift_rad':float(yaws[-1]-yaws[0]),'max_xy_displacement':float(np.linalg.norm(positions-positions[0],axis=1).max())}
 parking=criteria['parking_windows'].get(name,criteria['parking_windows']['single']);park=metrics_window(*parking)
 steady=metrics_window(5,10) if name not in ('stand','switch','command_timeout') else None
 steady_pass=steady is None or (np.linalg.norm(steady['rmse_vx_vy_wz'][:2])<=criteria['steady_single_xy_rmse_max_mps'] and steady['rmse_vx_vy_wz'][2]<=criteria['steady_single_yaw_rmse_max_radps'])
 park_pass=park is not None and park['mean_xy_speed']<=criteria['parking_mean_xy_speed_max_mps'] and park['mean_abs_yaw_rate']<=criteria['parking_mean_abs_yaw_rate_max_radps'] and park['xy_drift']<=criteria['parking_window_xy_drift_max_m']
 segments=[metrics_window(5,7.5),metrics_window(10,12.5),metrics_window(15,17.5)] if name=='switch' else []
 result=next(r for r in results['results'] if r['name']==name)
 rows.append({'name':name,'frame_slice':[begin,len(all_observed)],'frames':len(trace),'scheduled_duration':case['duration'],'duration_recorded':result['duration_recorded'],'fell':result['failed'],'steady':steady,'switch_steady_segments':segments,'parking':park,'steady_passed':bool(steady_pass),'parking_passed':bool(park_pass),'control_passed':not result['failed'] and steady_pass and park_pass,'max_abs_roll':max(abs(r['roll']) for r in trace),'max_abs_pitch':max(abs(r['pitch']) for r in trace),'min_base_clearance':min(r['clearance'] for r in trace if r['clearance'] is not None),'max_abs_joint_target':float(np.max(abs(np.asarray([r['q_target'] for r in trace])))),'max_abs_torque':float(np.max(abs(np.asarray([r['torque'] for r in trace])))),'parking_contact_count_mean':float(np.mean([np.sum(np.linalg.norm(r['contact_forces'],axis=1)>1.) for r in trace if parking[0]<=r['t']<parking[1]])),'parking_controller_modes':sorted(set(r['controller_mode'] for r in trace if parking[0]<=r['t']<parking[1]))})
observed=np.asarray(all_observed,dtype=np.float32);rebuilt=np.asarray(all_rebuilt,dtype=np.float32);actions=np.asarray(all_actions,dtype=np.float32);teacher_mask=np.asarray(teacher_mask);reset_mask=np.asarray(reset_mask)
with torch.inference_mode(): predicted=actor(torch.from_numpy(observed)).numpy();predicted_rebuilt=actor(torch.from_numpy(rebuilt)).numpy()
obs_err=np.max(abs(observed-rebuilt),axis=1);action_err=np.max(abs(predicted-actions),axis=1);action_rebuilt_err=np.max(abs(predicted_rebuilt-actions),axis=1)
for row in rows:
 start,end=row['frame_slice'];row['reset_observation_max_error']=float(obs_err[start]);row['all_observation_max_error']=float(obs_err[start:end].max());sel=teacher_mask[start:end];row['recorded_obs_actor_max_error']=float(action_err[start:end][sel].max());row['reconstructed_obs_actor_max_error']=float(action_rebuilt_err[start:end][sel].max())
np.savez_compressed(out/'recorded_observation_parity.npz',recorded_observation247=observed,reconstructed_observation247=rebuilt,recorded_action=actions,actor_action_from_recorded_obs=predicted,actor_action_from_reconstructed_obs=predicted_rebuilt,teacher_frame_mask=teacher_mask,reset_frame_mask=reset_mask,case_labels=np.asarray(case_labels))
summary={'status':'completed','reference_scope':'Matched Gazebo command timing/bootstrap/slew. PhysX nominal flat1envCPU; material1/1 vsGazebo.7/1; training model16.087kg vsDemo16.512kg; no noise/randomization/push. Not identical dynamics and not Sim2Sim approval.','cases':len(rows),'frames':len(observed),'all_cases_fall_free':not any(row['fell'] for row in rows),'all_control_passed':all(row['control_passed'] for row in rows),'observations_directly_recorded':True,'all247_observation_max_error':float(obs_err.max()),'reset247_observation_max_error':float(obs_err[reset_mask].max()),'recorded_obs_actor_action_max_error':float(action_err[teacher_mask].max()),'reconstructed_obs_actor_action_max_error':float(action_rebuilt_err[teacher_mask].max()),'observation_parity_passed':bool(obs_err.max()<1e-5),'teacher_actor_parity_passed':bool(action_err[teacher_mask].max()<2e-5 and action_rebuilt_err[teacher_mask].max()<2e-5),'pd_init_excluded_from_teacher_actor_parity':int((~teacher_mask).sum()),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'observation_sha256':hashlib.sha256((ROOT/'policy/observation.py').read_bytes()).hexdigest(),'contract_sha256':hashlib.sha256((ROOT/'policy/contract.json').read_bytes()).hexdigest(),'criteria':criteria,'rows':rows}
(out/'matched_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('rows','criteria')},indent=2))
for row in rows:print(row['name'],json.dumps({'fell':row['fell'],'steady':row['steady'],'parking':row['parking'],'passed':row['control_passed'],'reset_obs_error':row['reset_observation_max_error'],'actor_error':row['reconstructed_obs_actor_max_error']}))
