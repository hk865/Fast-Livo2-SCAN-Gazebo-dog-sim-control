#!/usr/bin/env python3
"""Audit real SDF mesh CPU reference against unchanged Gazebo terrain criteria."""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'policy'))
from observation import TerrainHeightMap, build_observation, quaternion_rotation
p = argparse.ArgumentParser(); p.add_argument('run',type=Path); args=p.parse_args(); out=args.run.resolve()
protocol=json.loads((out/'protocol.json').read_text()); result=json.loads((out/'results.json').read_text())
if result['status']!='completed': raise RuntimeError('Physical reference must complete before analysis')
criteria=json.loads((out/'analysis_criteria.json').read_text())
terrain=TerrainHeightMap.from_sdf(out/'fixture_world.sdf')
actor=torch.nn.Sequential(torch.nn.Linear(247,512),torch.nn.ELU(),torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),torch.nn.ELU(),torch.nn.Linear(128,12))
checkpoint=torch.load(protocol['checkpoint'],map_location='cpu',weights_only=False)
actor.load_state_dict({k.removeprefix('mlp.'):v for k,v in checkpoint['actor_state_dict'].items() if k.startswith('mlp.')}); actor.eval(); torch.set_num_threads(1)

def surface(box, xy):
    starts=np.c_[xy,np.full(len(xy),25.)]
    return starts[:,2]-box.intersect(starts)

rows=[]
for case in protocol['schedule']['cases']:
    name=case['name']; trace=json.loads((out/f'{name}_trace.json').read_text()); t=np.asarray([r['t'] for r in trace]);pos=np.asarray([r['position'] for r in trace])
    box=next(s for s in terrain.shapes if s.name.startswith('ramp_12/' if name.startswith('ramp') else 'teacher_low_step/'))
    start=(t>=2)&(t<3);end=t>=t[-1]-1; p0=np.median(pos[start],axis=0);p1=np.median(pos[end],axis=0)
    command_end=max(s['end'] for s in case['segments'] if np.linalg.norm(s['command'])>0)
    active=(t>=3)&(t<=command_end); settled_stop=(t>=command_end+3)&(t<case['duration'])
    observed=np.asarray([r['observation247'] for r in trace],dtype=np.float32); actions=np.asarray([r['action'] for r in trace]); teacher=np.asarray([r['controller_mode']=='teacher' for r in trace])
    with torch.inference_mode(): predicted=actor(torch.from_numpy(observed)).numpy()
    actor_error=float(abs(predicted[teacher]-actions[teacher]).max())
    raw_scan=np.asarray([r['height_scan_raw'] for r in trace]); scan_error=float(abs(np.clip(raw_scan,-1,1)-observed[:,60:]).max())
    sensor_pos=np.asarray([r['scanner_position_world'] for r in trace]); rayhits=np.asarray([r['height_ray_hits_world'] for r in trace]); ray_expected,_=terrain.raycast(rayhits[:,:,0:3].reshape(-1,3)+[0,0,20])
    # These measured x/y are actual Isaac ray hits; fresh analytic z must match
    # the same true world geometry irrespective of periodic sensor pose lag.
    mesh_ray_error=float(abs(ray_expected.reshape(len(trace),187)-rayhits[:,:,2]).max())
    actual=np.asarray([np.r_[r['linear_velocity_body_com'][:2],r['angular_velocity_body'][2]] for r in trace]); commands=np.asarray([r['command'] for r in trace]);steady=(t>=5)&(t<min(10,command_end));rmse=np.sqrt(((actual[steady]-commands[steady])**2).mean(0))
    stop_actual=actual[settled_stop];stop_pos=pos[settled_stop];stop_drift=float(np.linalg.norm(stop_pos[-1,:2]-stop_pos[0,:2]));stop_xy_rms=float(np.sqrt((stop_actual[:,:2]**2).sum(1).mean()));stop_yaw_rms=float(np.sqrt((stop_actual[:,2]**2).mean()))
    # Actual calf net forces do not expose the contacted triangle ID. Combine
    # measured forces with FK of each URDF foot collision center for a clearly
    # labeled support inference; never call it per-shape native contact data.
    leg_order=('FR','FL','RR','RL');support=[];fk_validation=[]
    for r in trace:
        q=np.asarray(r['q']);x,y,z,w=r['quaternion_xyzw'];baseR=quaternion_rotation([w,x,y,z]);group=[]
        for leg in leg_order:
            k=leg_order.index(leg)*3;hip,thigh,calf=q[k:k+3];cx,sx=np.cos(hip),np.sin(hip);cy,sy=np.cos(thigh+calf),np.sin(thigh+calf)
            calfR=baseR@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
            bi=r['body_names'].index(leg+'_calf');center=np.asarray(r['body_positions_world'][bi])+calfR@np.array([0,0,-.213])
            if 'body_quaternions_xyzw' in r:
                bx,by,bz,bw=r['body_quaternions_xyzw'][bi];trueR=quaternion_rotation([bw,bx,by,bz]);fk_validation.append(float(abs(calfR-trueR).max()))
            ci=r['contact_body_names'].index(leg+'_calf');force=np.linalg.norm(r['contact_forces'][ci]);hit=surface(box,center[None,:2])[0]
            if force>1 and np.isfinite(hit) and abs(center[2]-.022-hit)<.02:group.append(leg)
        support.append(group)
    stop_support=[s for s,m in zip(support,end) if m];all_active=set().union(*(s for s,m in zip(support,active) if m));all_end=set().union(*stop_support)
    base_contact_available=all('base_contact_force_world' in r for r in trace)
    base_max=float(max(np.linalg.norm(r['base_contact_force_world'],axis=1).max() for r in trace)) if base_contact_available else None
    reasons=[]
    rr=next(r for r in result['results'] if r['name']==name)
    if rr['failed']:reasons.append('Physical fall/termination')
    if max(max(abs(r['roll']),abs(r['pitch'])) for r in trace)>criteria['motion_limits']['max_abs_roll_pitch_rad']:reasons.append('Orientation exceeds frozen limit')
    if min(r['clearance'] for r in trace)<criteria['motion_limits']['min_body_clearance_m']:reasons.append('Clearance below frozen limit')
    if base_max is not None and base_max>1:reasons.append('Actual base contact')
    if np.linalg.norm(rmse[:2])>criteria['tracking']['linear_rmse_mps'] or rmse[2]>criteria['tracking']['yaw_rate_rmse_radps']:reasons.append('Steady tracking RMSE exceeds frozen limit')
    if stop_drift>criteria['stand_stop']['translation_drift_m'] or stop_xy_rms>criteria['stand_stop']['linear_rms_mps'] or stop_yaw_rms>criteria['stand_stop']['yaw_rate_rms_radps']:reasons.append('Parking exceeds frozen limit')
    end_inside=bool(np.isfinite(surface(box,pos[end,:2])).all());active_inside=bool(np.isfinite(surface(box,pos[active,:2])).all())
    if not end_inside:reasons.append('Settled endpoint outside tread')
    if all_end!=set(leg_order):reasons.append('Missing contact-plus-FK evidence for final foot support')
    terrain_metrics={'start_median_position':p0.tolist(),'end_median_position':p1.tolist(),'settled_height_gain_m':float(p1[2]-p0[2]),'end_inside_tread':end_inside,'active_inside_tread':active_inside,'inferred_end_support_legs':sorted(all_end),'inferred_final_all_four_support_samples':sum(len(g)==4 for g in stop_support),'inferred_active_support_legs':sorted(all_active),'support_evidence':'Actual measured calf net contact forces >1N plus URDF foot collision center FK inside tread and bottom z within .02m of exact surface. Per-shape PhysX contact IDs were not recorded.','fk_rotation_vs_recorded_body_quat_max_error':max(fk_validation,default=None)}
    if name.startswith('ramp'):
        direction=1 if name=='ramp_up' else -1;progress=float(direction*(p1[0]-p0[0]));required=criteria['terrain']['up_ramp_min_x_progress_m' if direction==1 else 'down_ramp_min_progress_m'];rise=surface(box,p1[None,:2])[0]-surface(box,p0[None,:2])[0]
        terrain_metrics.update(progress_m=progress,required_progress_m=required,expected_surface_height_change_m=float(rise),height_change_error_m=float(abs(p1[2]-p0[2]-rise)))
        if progress<required:reasons.append('Ramp settled progress below unchanged1.5m requirement')
        if not active_inside:reasons.append('Active trace outside ramp')
        if all_active!=set(leg_order):reasons.append('Missing active support force/FK for some foot')
        if direction*(p1[2]-p0[2])<=0 or abs(p1[2]-p0[2]-rise)>criteria['stand_stop']['translation_drift_m']:reasons.append('Ramp height change inconsistent with true surface')
    else:
        height=.05 if name=='step05' else .1;minimum=height*criteria['terrain']['step_body_height_gain_fraction'];terrain_metrics.update(step_height_m=height,minimum_height_gain_m=minimum,required_end_x_m=criteria['terrain']['step_end_x_m'])
        if p1[0]<criteria['terrain']['step_end_x_m'] or p1[2]-p0[2]<minimum:reasons.append('Step settled endpoint or body height gain below unchanged criterion')
        if not any(len(s)==4 for s in stop_support):reasons.append('No final all-four contact-plus-FK sample on tread')
    rows.append({'name':name,'frames':len(trace),'fell':rr['failed'],'same_numeric_terrain_criteria_passed':not reasons,'failure_reasons':reasons,'terrain':terrain_metrics,'steady_window_seconds':[5,min(10,command_end)],'steady_mean_actual_vx_vy_wz':actual[steady].mean(0).tolist(),'steady_rmse_vx_vy_wz':rmse.tolist(),'parking':{'window_seconds':[command_end+3,case['duration']],'xy_drift_m':stop_drift,'linear_rms_mps':stop_xy_rms,'yaw_rate_rms_radps':stop_yaw_rms,'controller_modes':sorted(set(r['controller_mode'] for r,m in zip(trace,settled_stop) if m))},'actual_base_contact_max_newton':base_max,'base_contact_log_available':base_contact_available,'recorded_observation_actor_max_error':actor_error,'recorded_raw_scan_clip_vs_observation_error':scan_error,'actual_ray_hits_vs_exact_SDF_surface_max_error_m':mesh_ray_error,'max_abs_torque':float(np.max(abs(np.asarray([r['torque'] for r in trace]))))})
summary={'scope':'Real PhysX CPU one-env frozen Teacher, exact Gazebo static SDF fixtures imported for both physical collision and187-ray sensor. Nominal training robot mass16.087kg/material1 vs Gazebo Demo16.512kg/robot.7. No noise/randomization/push/training. Same numerical criteria do not establish identical dynamics or full Sim2Sim/navigation acceptance. Per-triangle surface contact attribution remains inferred, not directly recorded.','status':'completed','rows':rows,'all_cases_numeric_criteria_passed':all(r['same_numeric_terrain_criteria_passed'] for r in rows),'protocol_sha256':hashlib.sha256((out/'protocol.json').read_bytes()).hexdigest(),'analysis_criteria_sha256':hashlib.sha256((out/'analysis_criteria.json').read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(out/'terrain_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
