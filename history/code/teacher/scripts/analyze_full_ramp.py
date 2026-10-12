#!/usr/bin/env python3
"""Audit complete original-scene ramps from actual native physics/contact logs.

Writes an independent receipt. Never changes the historic evaluator, commands,
physics, or navigation; world height rise is diagnostic only.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def lines(path):return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]
def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,dict):return {str(k):clean(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [clean(x) for x in v]
    if isinstance(v,float) and not math.isfinite(v):return None
    return v
def check(value,**evidence):return {'status':'unverified'if value is None else'passed'if bool(value)else'failed','passed':None if value is None else bool(value),**evidence}
def array(rows,key,width=None):
    a=np.asarray([r.get(key) for r in rows],dtype=float)
    shape=(len(rows),) if width is None else (len(rows),width)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError(f'{key} is not finite {shape}: {a.shape}')
    return a
def module(path,prefix):
    name=prefix+sha(path)[:12];spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
def span(mask,t,gap):
    best=0.;begin=None;interval=None
    for i,valid in enumerate(mask):
        if not valid:begin=None;continue
        if begin is None or (i and t[i]-t[i-1]>gap):begin=float(t[i])
        length=float(t[i]-begin)
        if length>best:best=length;interval=[begin,float(t[i])]
    return best,interval
def rpy(q):
    w,x,y,z=q.T
    return np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def surface(world,name,helper):
    m=world.find(f"world/model[@name='{name}']")
    if m is None or m.findtext('static')!='true':raise ValueError(f'Missing actual static surface {name}')
    link=m.find('link');col=link.find('collision');mp,mr=helper.pose(m);lp,lr=helper.pose(link);cp,cr=helper.pose(col)
    size=np.fromstring(col.findtext('geometry/box/size',''),sep=' ')
    if size.shape!=(3,) or not np.isfinite(size).all():raise ValueError('Non-box ramp/landing evidence')
    return {'name':name,'collision':f'{name}::{link.get("name")}::{col.get("name")}','center':mp+mr@lp+mr@lr@cp,'rotation':mr@lr@cr,'half':size/2}


def support_masks(steps,centers,feet,surfaces,criteria):
    """Top face in each surface's actual rotated collision frame, not names alone."""
    masks={name:{leg:np.zeros(len(steps),dtype=bool) for leg in centers} for name in surfaces}
    missing=0;raw_points=0;qualified={name:0 for name in surfaces}
    for name,s in surfaces.items():
        local={leg:(centers[leg]-s['center'])@s['rotation'] for leg in centers}
        normal=s['rotation'][:,2]
        for i,row in enumerate(steps):
            for pair in row.get('contact_pairs',[]):
                if s['collision'] not in [pair.get('a'),pair.get('b')]:continue
                group=pair.get('group')
                if group not in range(1,5):continue
                leg=list(centers)[group-1];short={'FR':'rf','FL':'lf','RR':'rh','RL':'lh'}[leg]
                if not any(f'{short}_foot_link_collision' in str(pair.get(k,'')) for k in ['a','b']):continue
                count=pair.get('points');xyz=np.asarray(pair.get('positions_world',[]),dtype=float);n=np.asarray(pair.get('normals_world',[]),dtype=float)
                if not isinstance(count,int) or count<1 or xyz.shape!=(count,3) or n.shape!=(count,3) or not np.isfinite(xyz).all() or not np.isfinite(n).all() or (np.linalg.norm(n,axis=1)<1e-8).any():
                    missing+=1;continue
                raw_points+=count;point=(xyz-s['center'])@s['rotation'];normalized=n/np.linalg.norm(n,axis=1)[:,None]
                top=(np.abs(point[:,:2])<=s['half'][:2]-criteria['surface_local_xy_margin_m']).all(axis=1)&(abs(point[:,2]-s['half'][2])<=criteria['surface_normal_distance_tolerance_m'])&(abs(normalized@normal)>=criteria['minimum_absolute_normal_dot_surface_normal'])
                c=local[leg][i];radius=feet[leg]['radius_m']
                center_ok=(abs(c[:2])<=s['half'][:2]+radius).all() and abs(c[2]-s['half'][2]-radius)<=criteria['foot_center_height_tolerance_m']
                if top.any() and center_ok:masks[name][leg][i]=True;qualified[name]+=int(top.sum())
    return masks,{'missing_or_malformed_contact_geometry_rows':missing,'raw_points_examined':raw_points,'qualified_actual_top_points':qualified,'wrench_scope':'Raw wrenches are retained in actuator.jsonl; quantitative vertical bearing load is not inferred.'}


def evaluate(run, write_outputs=True):
    run=Path(run).resolve();checks={};metrics={};errors=[];sources={};schedule={};protocol={};legacy={}
    try:
        schedule=read(run/'independent_protocol.json');profile=schedule['profile_id']
        protocol_path=run/'sources/tests/ramp_full_protocol.json';protocol=read(protocol_path)
        checks['prospective_frozen_protocol']=check(sha(protocol_path)==schedule['acceptance_protocol_sha256'],actual_protocol_sha256=sha(protocol_path),scheduled_protocol_sha256=schedule['acceptance_protocol_sha256'])
        helper_path=run/'sources/scripts/evaluate_step_functional.py';helper=module(helper_path,'ramp_fk_')
        observation_path=run/'sources/policy/observation.py';observation=module(observation_path,'ramp_obs_')
        rows=lines(run/'telemetry.jsonl');native=lines(run/'actuator.jsonl');steps=[r for r in native if r.get('kind')=='physics_step'];contracts=[r for r in native if r.get('kind')=='actuator_contract']
        if len(contracts)!=1:raise ValueError('Need exactly one sole actuator contract')
        contract=contracts[0];meta=read(run/'policy_manifest.json');runtime=read(run/'runtime_manifest.json');result=read(run/'worker_result.json')
        legacy=read(run/'summary.json') if (run/'summary.json').exists() else {}
        offset=rows[0]['world_sim_time']-rows[0]['sim_time'];t=array(rows,'sim_time');nt=array(steps,'t')-offset;pos=array(steps,'position',3);q=array(steps,'quaternion_wxyz',4);rot=helper.quaternion_rotations(q);angles=rpy(q)
        nv=array(steps,'body_lin_vel_com',3);world_v=np.einsum('nij,nj->ni',rot,nv);origin_v=np.einsum('nij,nj->ni',rot,array(steps,'body_lin_vel_origin',3))
        actual_cmd=array(rows,'command',3);requested=array(rows,'requested',3);completion=result.get('physical_test_completion_time_s');gap=protocol['contact']['max_native_sample_gap_s'];ti=protocol['timing'];co=protocol['contact'];route=protocol['route'];sa=protocol['safety'];stopcriteria=protocol['stand_stop']
        if completion is None:
            recorded=[r.get('physical_test_completion_time_s') for r in rows if r.get('physical_test_completion_time_s') is not None]
            completion=recorded[0] if recorded else None
        if completion is not None and (not isinstance(completion,(float,int)) or not math.isfinite(completion)):raise ValueError('Nonfinite completion timestamp')
        active_end=min(float(completion) if completion is not None else ti['maximum_forward_end_s'],ti['maximum_forward_end_s'])
        active=(nt>=ti['command_start_s'])&(nt<active_end);pmactive=(t>=ti['command_start_s'])&(t<active_end)
        stop_start=(float(completion) if completion is not None else ti['maximum_forward_end_s'])+ti['stop_settling_s'];stop_end=stop_start+ti['stop_evaluation_s'];stop=(nt>=stop_start)&(nt<=stop_end+1e-8);pstop=(t>=stop_start)&(t<=stop_end+1e-8)
        code={p['role']:p.get('returncode') for p in runtime.get('owned_processes',[])}
        checks['complete_continuous_runtime']=check(result.get('completed') is True and not result.get('fault') and not runtime.get('error') and not any(r.get('kind')=='fault' for r in native) and not any(r.get('fault') for r in steps) and steps[-1].get('terminating') is True and code.get('worker')==0 and code.get('gazebo')==0 and np.all(np.diff(t)>0) and np.max(np.diff(t))<=1/protocol['policy_hz']+1e-8 and np.all(np.diff(nt)>0) and np.max(np.diff(nt))<=gap and t[-1]>=stop_end-.02,duration_s=t[-1],policy_samples=len(rows),native_samples=len(steps),maximum_native_gap_s=np.max(np.diff(nt)),main_exit_codes=code,completion_time_s=completion,required_stop_end_s=stop_end,worker_fault=result.get('fault'),runner_error=runtime.get('error'))
        world=ET.parse(run/'world.sdf').getroot();robot=world.find("world/model[@name='go2']");writers=[p for p in robot.findall('plugin') if 'TeacherActuator' in p.get('name','')];forbidden=[p.get('name') for p in robot.findall('plugin') if any(v in p.get('name','').lower() for v in ['control','champ','stabilizer','trajectory'])]
        with np.load(run/'observations_actions.npz') as saved:obs=saved['observations'];acts=saved['actions']
        policy_on=t>=ti['bootstrap_pd_s'];target_error=float(np.max(abs(array(rows,'q_target',12)[policy_on]-(np.asarray(contract['initial_q'])+.25*array(rows,'action',12)[policy_on]))))
        checks['continuous_cpu_teacher_exclusive_writer']=check(meta.get('inference_device')=='cpu' and meta.get('checkpoint_sha256')==protocol['checkpoint_sha256'] and obs.shape==(len(rows),247) and acts.shape==(len(rows),12) and np.isfinite(obs).all() and np.isfinite(acts).all() and target_error<1e-6 and not any(r.get('state') in ['support_hold','support_capture'] for r in rows) and len(writers)==1 and not forbidden and contract.get('body_pose_resets')==0 and contract.get('joint_position_reset_count')==12 and contract.get('kp')==25 and contract.get('kd')==.5 and contract.get('expected_dt')==protocol['physics_dt_s'] and contract.get('decimation')==4,target_action_max_error_rad=target_error,writer_contract=contract.get('writer'),writer_plugins=len(writers),other_control_plugins=forbidden,body_pose_resets=contract.get('body_pose_resets'),actor_device=meta.get('inference_device'))
        qd=array(steps,'qd',12);tau=array(steps,'tau',12);lo=np.maximum(23.5*(-1-qd/30),-23.5);hi=np.minimum(23.5*(1-qd/30),23.5);pd=25*(array(steps,'qtarget',12)-array(steps,'q',12))-.5*qd;enabled=np.asarray([r.get('mode')==0 and not r.get('terminating') for r in steps]);tau_error=float(np.max(abs(tau[enabled]-np.clip(pd[enabled],lo[enabled],hi[enabled]))))
        checks['native_dcmotor_execution']=check(contract.get('effort_limit')==23.5 and contract.get('saturation_effort')==23.5 and contract.get('velocity_limit')==30 and tau_error<1e-8,maximum_reconstructed_torque_error_Nm=tau_error,maximum_abs_applied_torque_Nm=abs(tau).max())
        required=np.array([protocol['command'][k] for k in ['body_vx_mps','body_vy_mps','yaw_radps']]);checks['scheduled_forward_then_teacher_zero']=check(pmactive.any() and np.allclose(requested[pmactive],required,atol=1e-8) and pstop.sum()>=ti['stop_evaluation_s']*protocol['policy_hz']-2 and pstop.any() and np.max(abs(actual_cmd[pstop]))<1e-6 and np.max(abs(requested[pstop]))<1e-6,active_window_s=[ti['command_start_s'],active_end],stop_window_s=[stop_start,stop_end],stop_samples=int(pstop.sum()),completion_source=meta.get('physical_test_completion_source'))
        surfaces={name:surface(world,name,helper) for name in [schedule['ramp'],schedule['start_landing'],schedule['destination_landing']]};ramp=surfaces[schedule['ramp']];fixture=protocol['fixtures'][schedule['ramp']]
        nominal=np.array(fixture['nominal_x_limits_m']);slope=fixture['slope_z_per_world_x'];n=ramp['rotation'][:,2];expected_normal=np.array([-slope,0,1.])/math.sqrt(1+slope*slope)
        expected_center_z=float(np.mean(fixture['floor_heights_m']))-.05/math.sqrt(1+slope*slope)
        landing_heights={fixture['floor_at_x_low']:fixture['floor_heights_m'][0],fixture['floor_at_x_high']:fixture['floor_heights_m'][1]}
        landing_ok=all(np.allclose(surfaces[name]['rotation'],np.eye(3),atol=1e-10) and abs(surfaces[name]['center'][2]+surfaces[name]['half'][2]-height)<1e-10 for name,height in landing_heights.items())
        checks['original_scene_complete_ramp_fixture']=check(np.allclose(n,expected_normal,atol=1e-10) and abs(ramp['center'][0]-8)<1e-10 and abs(ramp['center'][1]-fixture['center_y_m'])<1e-10 and abs(ramp['center'][2]-expected_center_z)<1e-10 and abs(ramp['half'][1]*2-fixture['width_m'])<1e-10 and abs(ramp['half'][0]*2-(math.hypot(12,1.2)+.04))<1e-10 and landing_ok,collision=ramp['collision'],ramp_center=ramp['center'],ramp_normal=n,nominal_horizontal_span_m=12,slope=slope,actual_landing_top_heights_m={name:surfaces[name]['center'][2]+surfaces[name]['half'][2] for name in landing_heights})
        centers,feet=helper.foot_centers(world,steps,contract['joint_order']);masks,contact_geometry=support_masks(steps,centers,feet,surfaces,co);metrics['contact_geometry']=contact_geometry
        checks['actual_contact_geometry_available']=check(contact_geometry['missing_or_malformed_contact_geometry_rows']==0,**contact_geometry)
        initial=(nt>=1.5)&(nt<ti['command_start_s']);legmetrics={};all_initial=True;all_ramp=True;all_final=True
        for leg in centers:
            first,firstpair=span(masks[schedule['start_landing']][leg]&initial,nt,gap);middle,middlepair=span(masks[schedule['ramp']][leg]&active,nt,gap);last,lastpair=span(masks[schedule['destination_landing']][leg]&stop,nt,gap)
            all_initial&=first>=co['each_foot_initial_landing_support_s']-1e-8;all_ramp&=middle>=co['each_foot_active_ramp_support_s']-1e-8;all_final&=last>=co['each_foot_final_landing_support_s']-1e-8
            legmetrics[leg]={'initial_landing_span_s':first,'initial_interval_s':firstpair,'active_ramp_span_s':middle,'ramp_interval_s':middlepair,'final_landing_span_s':last,'final_interval_s':lastpair,'final_collision_center_world':centers[leg][-1]}
        final_all=np.logical_and.reduce(list(masks[schedule['destination_landing']].values()));finalspan,finalpair=span(final_all&stop,nt,gap)
        checks['physical_entry_ramp_exit_each_named_foot']=check(all_initial and all_ramp and all_final and finalspan>=co['final_simultaneous_four_foot_landing_support_s']-1e-8 and bool(final_all[-1]),feet=legmetrics,simultaneous_destination_support_s=finalspan,simultaneous_interval_s=finalpair,all_four_terminal_destination_support=final_all[-1]);metrics['feet']=legmetrics
        ramp_any=np.logical_or.reduce(list(masks[schedule['ramp']].values()));binmetrics=[]
        for edge in np.arange(nominal[0],nominal[1],co['ramp_coverage_bin_m']):
            selected=active&ramp_any&(pos[:,0]>=edge)&(pos[:,0]<=edge+co['ramp_coverage_bin_m']);seconds=float(selected.sum()*protocol['physics_dt_s']);binmetrics.append({'x_limits_m':[edge,edge+co['ramp_coverage_bin_m']],'native_actual_support_seconds':seconds})
        checks['entire_twelve_meter_ramp_contact_coverage']=check(len(binmetrics)==12 and all(v['native_actual_support_seconds']>=co['each_coverage_bin_any_foot_support_s'] for v in binmetrics),bins=binmetrics)
        sign=schedule['world_forward_sign'];startpos=np.median(pos[initial],axis=0);lastactive=pos[np.flatnonzero(active)[-1]];progress=float(sign*(lastactive[0]-startpos[0]));forward=sign*world_v[:,0];indices=np.flatnonzero(active);duration=float(nt[indices[-1]]-nt[indices[0]])
        count=max(1,int(round(route['rolling_window_s']/protocol['physics_dt_s'])));rolling=np.convolve(forward[active],np.ones(count)/count,mode='valid') if active.sum()>=count else np.array([]);fraction=float((rolling>route['minimum_projected_world_forward_window_mean_mps']).mean()) if len(rolling) else None
        heading=np.arctan2(np.sin(angles[:,2]-schedule['spawn'][3]),np.cos(angles[:,2]-schedule['spawn'][3]));lateral=float(abs(pos[active,1]-fixture['center_y_m']).max());headingmax=float(abs(heading[active]).max());meanvx=float(forward[active].mean())
        crossed=bool((pos[active,0]<=nominal[0]).any() and (pos[active,0]>=nominal[1]).any());threshold=schedule['completion_stop']['threshold'];reached=lastactive[0]<=threshold+.02 if sign<0 else lastactive[0]>=threshold-.02
        metrics['route']={'initial_median_position_m':startpos,'last_active_position_m':lastactive,'projected_forward_progress_m':progress,'active_duration_s':duration,'mean_projected_world_COM_vx_mps':meanvx,'positive_rolling_window_fraction':fraction,'rolling_windows':len(rolling),'maximum_lateral_deviation_m':lateral,'maximum_heading_deviation_rad':headingmax,'nominal_both_seams_crossed_during_active_command':crossed,'landing_completion_time_s':completion}
        checks['complete_ramp_and_landing_progress']=check(completion is not None and completion<ti['maximum_forward_end_s'] and crossed and reached and progress>=route['minimum_forward_progress_m'] and lateral<=route['maximum_base_center_lateral_deviation_m'] and headingmax<=route['maximum_heading_deviation_rad'] and fraction is not None and fraction>=route['minimum_positive_window_fraction'] and meanvx>=required[0]*route['minimum_requested_axis_ratio'],**metrics['route'])
        any_support=np.logical_or.reduce([masks[name][leg] for name in surfaces for leg in centers]);unsupported,unsupportedpair=span((~any_support)&active,nt,gap)
        terrain=observation.TerrainHeightMap.from_sdf(run/'world.sdf');ground=terrain.height(pos[:,:2],ray_start_z=pos[:,2]);clearance=pos[:,2]-ground;safety=nt>=sa['audit_start_s'];contacts=array(steps,'contacts',5);maxrp=float(abs(angles[safety,:2]).max());minclear=float(clearance[safety].min())
        integral_error=float(abs(np.diff(pos,axis=0)-origin_v[1:]*np.diff(nt)[:,None]).max())
        metrics['safety']={'start_s':sa['audit_start_s'],'native_samples':int(safety.sum()),'max_abs_roll_pitch_rad':maxrp,'minimum_support_clearance_m':minclear,'body_contact_samples':int((contacts[safety,0]>0).sum()),'missing_contact_samples':int((contacts[safety]<0).sum()),'maximum_all_feet_unsupported_s':unsupported,'unsupported_interval_s':unsupportedpair,'maximum_pose_velocity_integral_error_m':integral_error,'attitude_source':'native actual quaternion at200Hz','clearance_definition':'actual base origin minus supporting collision directly below it, safety only'}
        body_indices=np.flatnonzero(safety&(contacts[:,0]>0));firstbody=int(body_indices[0]) if len(body_indices) else None
        edge={}
        for leg in centers:
            local=(centers[leg]-ramp['center'])@ramp['rotation'];margin=ramp['half'][1]-abs(local[:,1])-feet[leg]['radius_m'];outside=np.flatnonzero(active&(margin<0))
            edge[leg]={'minimum_active_lateral_sphere_clearance_to_ramp_edge_m':float(margin[active].min()),'first_outside_lateral_edge_s':float(nt[outside[0]]) if len(outside) else None,'collision_center_at_first_body_contact_world':centers[leg][firstbody] if firstbody is not None else None}
        metrics['failure_diagnostics']={'first_body_contact_s':float(nt[firstbody]) if firstbody is not None else None,'first_body_contact_position_m':pos[firstbody] if firstbody is not None else None,'first_body_actual_contact_pairs':[v for v in steps[firstbody].get('contact_pairs',[]) if v.get('group')==0] if firstbody is not None else [],'feet_lateral_edges':edge,'edge_scope':'Sphere extent relative to actual ramp lateral edges; airborne swing at an edge is diagnostic, actual contact and body safety remain the acceptance evidence.'}
        checks['unassisted_native_safety']=check(maxrp<=sa['max_abs_roll_pitch_rad'] and minclear>=sa['minimum_body_above_support_m'] and not(contacts[safety,0]>0).any() and not(contacts[safety]<0).any() and unsupported<=co['maximum_all_feet_unsupported_s'] and integral_error<=sa['pose_translation_integral_error_m'],**metrics['safety'])
        stoppos=pos[stop];stopyaw=np.unwrap(angles[:,2])[stop];drift=float(np.linalg.norm(stoppos[:,:2]-stoppos[0,:2],axis=1).max()) if len(stoppos) else None;yawdrift=float(abs(stopyaw-stopyaw[0]).max()) if len(stopyaw) else None
        stoplin=np.sqrt(np.mean(nv[stop,:2]**2,axis=0)) if stop.any() else np.full(2,np.inf);stopyawrate=float(np.sqrt(np.mean(array(steps,'body_ang_vel',3)[stop,2]**2))) if stop.any() else np.inf
        metrics['stop']={'window_s':[stop_start,stop_end],'translation_drift_m':drift,'yaw_drift_rad':yawdrift,'linear_rms_mps':stoplin,'yaw_rate_rms_radps':stopyawrate,'native_samples':int(stop.sum()),'strategy':'continuous learned Teacher with zero VELOCITY command; action remains inferred'}
        stop_verified=completion is not None and drift is not None and stop.sum()>=ti['stop_evaluation_s']/protocol['physics_dt_s']-2 and pstop.sum()>=ti['stop_evaluation_s']*protocol['policy_hz']-2
        stop_value=drift<=stopcriteria['translation_drift_m'] and yawdrift<=stopcriteria['yaw_drift_rad'] and max(stoplin)<=stopcriteria['linear_rms_mps'] and stopyawrate<=stopcriteria['yaw_rate_rms_radps'] and np.max(abs(actual_cmd[pstop]))<1e-6 and np.max(abs(requested[pstop]))<1e-6 if stop_verified else None
        checks['teacher_stop_on_destination_landing']=check(stop_value,**metrics['stop'],destination_completion_time_s=completion,reason='Actual destination landing and full continuousTeacher stop window'if stop_verified else'Destination not reached or complete stop window absent; parking not tested')
        metrics['height_change_diagnostic']={'body_world_z_gain_m':float(np.median(stoppos[:,2])-startpos[2]) if len(stoppos) else None,'support_surface_z_change_m':float(np.median(ground[stop])-np.median(ground[initial])) if stop.any() else None,'body_world_z_used_as_pass_gate':False,'height_scan_overhead_points_total':sum(r.get('height_scan_overhead_points',0) for r in rows),'frames_with_overhead_height_scan':sum(r.get('height_scan_overhead_points',0)>0 for r in rows)}
        manifest=read(run/'source_manifest.json');mismatches=[rel for rel,digest in manifest.items() if not(run/'sources'/rel).is_file() or sha(run/'sources'/rel)!=digest]
        checks['archived_source_integrity']=check(not mismatches,archived_files=len(manifest),mismatches=mismatches)
        sourcefiles=['world.sdf','actuator.jsonl','telemetry.jsonl','runtime_manifest.json','policy_manifest.json','worker_result.json','independent_protocol.json','sources/tests/ramp_full_protocol.json','sources/scripts/evaluate_step_functional.py','sources/policy/observation.py']
        sources={p:sha(run/p) for p in sourcefiles}
        if write_outputs:np.savez_compressed(run/'full_ramp_native_analysis.npz',sim_time=nt,base_position=pos,world_COM_velocity=world_v,support_clearance=clearance,**{f'{leg}_collision_center':centers[leg] for leg in centers},**{f'{name}_{leg}_top_support':masks[name][leg] for name in surfaces for leg in centers})
    except Exception as e:errors.append(f'{type(e).__name__}: {e}')
    status='failed'if errors or any(v['passed']is False for v in checks.values())else'passed'if checks and all(v['passed']is True for v in checks.values())else'unverified'
    receipt=clean({'schema_version':1,'profile_id':schedule.get('profile_id'),'scope':'Complete original physical ramp locomotion and landing transition audit, not SLAM navigation or stairs','status':status,'levels':{'full_ramp_locomotion':status,'navigation':'unverified','real_robot':'unverified'},'checks':checks,'metrics':metrics,'errors':errors,'protocol':protocol,'schedule':schedule,'source_hashes':sources,'analyzer_sha256':sha(__file__),'legacy_evaluation_diagnostic_only':legacy.get('levels'),'privileged_truth_use':'actor observation and physical-test end latch/evaluation only; no position servo or navigation claim'})
    if write_outputs:(run/'summary_full_ramp.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    return receipt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='+',type=Path);args=p.parse_args()
    for run in args.runs:
        result=evaluate(run);print(json.dumps({'run':str(run),'status':result['status'],'failed_checks':[k for k,v in result['checks'].items() if v['passed']is False],'unverified_checks':[k for k,v in result['checks'].items()if v['passed']is None],'errors':result['errors'],'metrics':result['metrics']},allow_nan=False),flush=True)
if __name__=='__main__':main()
