#!/usr/bin/env python3
"""Independent 200Hz continuous-turn plant evidence; no PID tracking verdict."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
sys.dont_write_bytecode=True


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    with Path(path).open() as f: return [json.loads(line) for line in f if line.strip()]


def field(data,key,width=None):
    out=np.asarray([x[key] for x in data],float)
    if not np.isfinite(out).all() or (width is not None and out.shape!=(len(data),width)):
        raise ValueError('Missing/nonfinite original '+key)
    return out


def rotations(q):
    if q.ndim!=2 or q.shape[1]!=4 or np.any(abs((q*q).sum(1)-1)>.02):raise ValueError('Invalid actual quaternion')
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.stack((1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),2*(x*y+w*z),1-2*(x*x+z*z),
        2*(y*z-w*x),2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),axis=1).reshape(-1,3,3)


def circle_fit(points):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<8 or not np.isfinite(points).all():raise ValueError('Incomplete circle fit input')
    origin=points.mean(0); xy=points-origin
    design=np.column_stack((2*xy[:,0],2*xy[:,1],np.ones(len(xy))))
    solution,_,rank,singular=np.linalg.lstsq(design,np.sum(xy*xy,axis=1),rcond=None)
    if rank<3:raise ValueError('Degenerate circle arc (in-place/straight is not a radius)')
    center=origin+solution[:2];radius2=float(solution[2]+solution[:2]@solution[:2])
    if radius2<=0:raise ValueError('Nonpositive fitted circle')
    radius=math.sqrt(radius2); distances=np.linalg.norm(points-center,axis=1)
    angle=np.unwrap(np.arctan2(points[:,1]-center[1],points[:,0]-center[0]))
    return {'center_xy_m':center.tolist(),'radius_m':radius,'radial_rmse_m':float(np.sqrt(np.mean((distances-radius)**2))),
        'signed_arc_rad':float(angle[-1]-angle[0]),'absolute_arc_rad':float(abs(angle[-1]-angle[0])),
        'fit_condition_number':float(singular[0]/singular[-1])}


def rolling_mean(t,values,window):
    """Every complete native-clock window; no padding across cruise/stop."""
    t=np.asarray(t,float);values=np.asarray(values,float)
    if len(t)<2 or np.any(np.diff(t)<=0):raise ValueError('Invalid rolling source time')
    integral=np.concatenate(([0.],np.cumsum(.5*(values[1:]+values[:-1])*np.diff(t))))
    ends=t[t>=t[0]+window-1e-9]
    if not len(ends):raise ValueError('No full prospective rolling window')
    return ends,(np.interp(ends,t,integral)-np.interp(ends-window,t,integral))/window


def serial(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return {k:serial(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [serial(x) for x in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def evaluate(run):
    run=Path(run).resolve(); checks={}; metrics={}; inputs={}; arrays={}
    if (run/'summary_radius_independent.json').exists() or (run/'radius_evidence_arrays.npz').exists():
        raise FileExistsError('Independent radius outputs are immutable; never replace an original verdict')
    def check(name,ok,detail):checks[name]={'status':'unverified' if ok is None else 'passed' if bool(ok) else 'failed','detail':serial(detail)}
    protocol_path=run/'sources/curvature/protocol.json'; plan_path=run/'curvature_plan.json'
    try:
        protocol=json.loads(protocol_path.read_text());plan=json.loads(plan_path.read_text());g=protocol['gates'];timing=protocol['timing']
        for name in ('actuator.jsonl','telemetry.jsonl','plant_commands.jsonl','observations_actions.npz','runtime_manifest.json','policy_manifest.json','worker_result.json','source_manifest.json','fixture_manifest.json','curvature_plan.json'):
            path=run/name
            if not path.exists():raise ValueError('Missing original input '+name)
            inputs[name]=sha(path)
        inputs['protocol.json']=sha(protocol_path)
        source=json.loads((run/'source_manifest.json').read_text());source_errors=[]
        for name,item in source.items():
            path=Path(item['path'])
            if not path.exists() or sha(path)!=item['sha256']:source_errors.append(name)
        check('frozen_sources_and_protocol',not source_errors and inputs['protocol.json']==plan['protocol_sha256'] and plan['timing']==timing,
              {'source_mismatches':source_errors,'protocol_sha256':inputs['protocol.json']})
        check('prospective_signed_plan_grid',plan['speed_mps'] in protocol['speeds_mps'] and plan['yaw_rate_radps'] in protocol['yaw_rates_radps'] and
              math.isclose(plan['command_radius_m'],plan['speed_mps']/abs(plan['yaw_rate_radps']),abs_tol=1e-12) and plan['fixture']==protocol['fixture'] and
              plan['model_sha256']==protocol['model_sha256'],{'speed_mps':plan['speed_mps'],'signed_yaw_rate_radps':plan['yaw_rate_radps'],'command_radius_m':plan['command_radius_m']})
        runtime=json.loads((run/'runtime_manifest.json').read_text());owned=runtime.get('owned_processes',[])
        actual_roles=[p['role'] for p in owned];expected=runtime['expected_owned_roles']
        check('complete_owned_runtime',runtime.get('error') is None and len(actual_roles)==len(set(actual_roles)) and set(actual_roles)==set(expected) and all(type(p.get('returncode')) is int and p['returncode']==0 for p in owned),owned)
        fixture=json.loads((run/'fixture_manifest.json').read_text())
        check('explicit_unobstructed_plant_fixture',fixture.get('fixture')==protocol['fixture'] and fixture.get('no_base_servo') is True and fixture.get('no_physics_math_change') is True and fixture.get('robot_changes')==['spawn pose only'] and fixture['world_sha256']==sha(run/'world.sdf'),fixture)
        all_native=rows(run/'actuator.jsonl');native=[r for r in all_native if r.get('kind')=='physics_step']
        contracts=[r for r in all_native if r.get('kind')=='actuator_contract'];owners=[r for r in all_native if r.get('kind')=='model_plugin_ownership']
        if len(contracts)!=1 or len(owners)!=1:raise ValueError('Missing unique native ownership/contract')
        contract=contracts[0];plant=protocol['plant']
        check('exclusive_native_PD_DC_motor_contract',owners[0].get('passed') is True and owners[0].get('teacher_writers')==1 and 'sole JointForceCmd writer' in contract.get('writer','') and all(abs(float(contract[k])-plant[p])<1e-10 for k,p in [('kp','kp'),('kd','kd'),('effort_limit','effort_limit_nm'),('saturation_effort','saturation_effort_nm'),('velocity_limit','velocity_limit_radps'),('expected_dt','physics_dt_s'),('decimation','decimation')]),{'contract':contract,'ownership':owners[0]})
        telemetry=rows(run/'telemetry.jsonl');commands=rows(run/'plant_commands.jsonl')
        if len(native)<2 or len(telemetry)<2 or len(commands)!=len(telemetry):raise ValueError('Incomplete native/policy/command samples')
        nt=field(native,'t');dt=field(native,'dt');state_t=nt-dt
        tt=field(telemetry,'sim_time');world_t=field(telemetry,'world_sim_time')
        origin_time=world_t[0]-.005; t=state_t-origin_time
        position=field(native,'position',3);q=field(native,'q',12);qd=field(native,'qd',12);tau=field(native,'tau',12)
        target=field(native,'qtarget',12);quat=field(native,'quaternion_wxyz',4);rot=rotations(quat)
        body_com=field(native,'body_lin_vel_com',3);omega=field(native,'body_ang_vel',3)
        offset=np.asarray(contract['base_com_offset_body_m'],float);body_origin=body_com-np.cross(omega,np.broadcast_to(offset,omega.shape))
        recorded_origin=field(native,'body_lin_vel_origin',3)
        origin_error=float(np.max(np.abs(body_origin-recorded_origin)))
        world_origin=np.einsum('nij,nj->ni',rot,body_origin)
        roll=np.arctan2(rot[:,2,1],rot[:,2,2]);pitch=np.arcsin(np.clip(-rot[:,2,0],-1,1));yaw=np.unwrap(np.arctan2(rot[:,1,0],rot[:,0,0]))
        contacts=field(native,'contacts',5);fault=field(native,'fault');mode=field(native,'mode')
        actual_cmd=field(telemetry,'command',3);request=field(telemetry,'requested',3)
        pair=np.searchsorted(nt,world_t);pair=np.clip(pair,0,len(nt)-1)
        phase_error=float(np.max(abs(nt[pair]-world_t)))
        position_error=float(np.max(abs(position[pair]-field(telemetry,'position',3))))
        state_error=float(np.max(abs(state_t[pair]-field(telemetry,'state_physics_world_time'))))
        check('original_worker_native_state_phase',phase_error<=1e-8 and state_error<=1e-8 and position_error<=2e-7 and
              np.all(np.diff(field(native,'iteration'))==1) and np.max(abs(field(commands,'world_sim_time')-world_t))<=1e-8 and
              np.max(abs(field(commands,'state_physics_world_time')-field(telemetry,'state_physics_world_time')))<=1e-8,
              {'native_publish_time_error_s':phase_error,'state_time_error_s':state_error,'position_pair_error_m':position_error})
        check('complete_monotonic_native_and_50Hz_policy',np.all(np.diff(t)>0) and np.max(np.diff(t))<=g['maximum_native_gap_s'] and np.all(abs(dt-.005)<1e-9) and np.all(np.diff(tt)>0) and np.max(np.diff(tt))<=.020001 and tt[-1]>=timing['duration_s']-.020001 and origin_error<=2e-7,
              {'native_samples':len(native),'policy_samples':len(telemetry),'end_sim_s':tt[-1],'max_native_gap_s':np.max(np.diff(t)),'max_policy_gap_s':np.max(np.diff(tt)),'origin_COM_reconstruction_max_error_mps':origin_error})
        policy=json.loads((run/'policy_manifest.json').read_text());result=json.loads((run/'worker_result.json').read_text())
        with np.load(run/'observations_actions.npz') as pack:
            obs=pack['observations'];action=pack['actions'];oa_ok=obs.shape==(len(telemetry),247) and action.shape==(len(telemetry),12) and np.isfinite(obs).all() and np.isfinite(action).all()
        check('frozen_CPU_Teacher_continuously_active',policy.get('checkpoint_sha256')==protocol['model_sha256'] and policy.get('inference_device')=='cpu' and policy.get('torch_threads')==1 and policy.get('command_slew_acceleration')==plant['command_slew_per_s'] and oa_ok and all(r.get('actor_inferred_this_frame') is True for r in telemetry) and policy.get('counts_as_SLAM_navigation') is False,
              {'inference_device':policy.get('inference_device'),'policy_samples':len(telemetry),'observation_action_shapes':[list(obs.shape),list(action.shape)],'privileged_observations':policy.get('observation_source')})
        scheduled=[]
        for ts in tt:
            if ts<3:scheduled.append([0,0,0])
            elif ts<6:scheduled.append([plan['speed_mps']*(ts-3)/3,0,plan['yaw_rate_radps']*(ts-3)/3])
            elif ts<26:scheduled.append([plan['speed_mps'],0,plan['yaw_rate_radps']])
            else:scheduled.append([0,0,0])
        scheduled=np.asarray(scheduled)
        applied=np.zeros(3);expected_cmd=[]
        for r in request:
            applied+=np.clip(r-applied,-np.asarray(plant['command_slew_per_s'])*.02,np.asarray(plant['command_slew_per_s'])*.02);expected_cmd.append(applied.copy())
        check('original_velocity_schedule_and_slew',np.max(abs(request-scheduled))<1e-7 and np.max(abs(actual_cmd-np.asarray(expected_cmd)))<1e-7 and np.max(abs(field(commands,'sim_time')-tt))<1e-8 and np.max(abs(field(commands,'requested_body_command',3)-request))<1e-7,
              {'requested_max_error':np.max(abs(request-scheduled)),'slew_max_error':np.max(abs(actual_cmd-np.asarray(expected_cmd))),'command_radius_m':plan['command_radius_m']})
        safe=(t>=g['safety_start_s']) & (t<=timing['duration_s']+1e-8)
        faults=[r for r in all_native if r.get('kind')=='fault' and r.get('reason') not in ('worker completed experiment',)]
        check('no_runtime_or_physical_fault',result.get('fault') is None and not faults and np.all(fault==0) and all(r.get('fault') is None for r in telemetry),{'worker_fault':result.get('fault'),'native_fault_events':faults})
        check('body_contact_clearance_and_attitude_safety',np.all(contacts[safe,0]==0) and np.all(contacts[safe]>=0) and np.max(np.abs(np.column_stack((roll,pitch))[safe]))<=g['maximum_roll_pitch_rad'] and np.min(position[safe,2])>=g['minimum_base_clearance_m'],
              {'body_contact_samples':np.sum(contacts[safe,0]>0),'max_roll_pitch_rad':np.max(np.abs(np.column_stack((roll,pitch))[safe])),'minimum_base_clearance_m':np.min(position[safe,2]),'ground_top_z_m':0.})
        check('all_native_torque_and_joint_speed_limits',np.max(abs(tau[safe]))<=g['maximum_abs_torque_nm'] and np.max(abs(qd[safe]))<=g['maximum_joint_velocity_radps'],{'max_abs_torque_nm':np.max(abs(tau[safe])),'max_abs_qd_radps':np.max(abs(qd[safe])),'torque_source':'Native JointForceCmd feed to physics, not measured motor torque'})
        raw=contract['kp']*(target-q)-contract['kd']*qd
        lower=np.maximum(contract['saturation_effort']*(-1-qd/contract['velocity_limit']),-contract['effort_limit'])
        upper=np.minimum(contract['saturation_effort']*(1-qd/contract['velocity_limit']),contract['effort_limit'])
        pd=np.minimum(np.maximum(raw,lower),upper);pd_mask=(mode==0)&safe
        pd_error=float(np.max(np.abs(tau[pd_mask]-pd[pd_mask])))
        check('actual_PD_speed_torque_curve_recomputed',pd_error<=g['pd_numeric_tolerance_nm'],{'max_abs_difference_nm':pd_error,'evaluated_native_samples':np.sum(pd_mask)})
        cruise=(t>=timing['ramp_end_s']-1e-9)&(t<=timing['cruise_end_s']+1e-9)
        ct=t[cruise];cp=position[cruise,:2];v=body_origin[cruise,0];w=omega[cruise,2]
        expected=int(round((timing['cruise_end_s']-timing['ramp_end_s'])/.005))+1
        check('full_continuous_cruise_coverage',len(ct)>=expected*g['minimum_native_coverage_fraction'] and abs(ct[0]-6)<=.005001 and abs(ct[-1]-26)<=.005001,{'samples':len(ct),'expected':expected,'start_s':ct[0],'end_s':ct[-1]})
        vmae=float(np.mean(abs(v-plan['speed_mps'])));wmae=float(np.mean(abs(w-plan['yaw_rate_radps'])))
        check('continuous_forward_and_yaw_response',vmae<=abs(plan['speed_mps'])*g['speed_mae_fraction'] and wmae<=abs(plan['yaw_rate_radps'])*g['yaw_rate_mae_fraction'],
              {'forward_mae_mps':vmae,'yaw_rate_mae_radps':wmae,'mean_origin_body_forward_mps':np.mean(v),'mean_body_yaw_rate_radps':np.mean(w),'speed_fraction_threshold':g['speed_mae_fraction'],'yaw_fraction_threshold':g['yaw_rate_mae_fraction']})
        rt,rv=rolling_mean(ct,v,g['forward_window_s'])
        check('never_in_place_or_intermittent_stop_in_cruise',np.min(rv)>=plan['speed_mps']*g['minimum_forward_rolling_mean_fraction'],
              {'window_s':g['forward_window_s'],'minimum_rolling_forward_mps':np.min(rv),'threshold_mps':plan['speed_mps']*g['minimum_forward_rolling_mean_fraction'],'instantaneous_forward_min_mps_diagnostic':np.min(v)})
        fit=circle_fit(cp);subfits=[]
        edges=np.linspace(ct[0],ct[-1],int(g['subfit_count'])+1)
        for a,b in zip(edges[:-1],edges[1:]):subfits.append(circle_fit(cp[(ct>=a-1e-9)&(ct<=b+1e-9)]))
        radii=np.array([f['radius_m'] for f in subfits]);cv=float(np.std(radii)/np.mean(radii))
        check('valid_stable_geometric_circle_arc',fit['absolute_arc_rad']>=g['minimum_arc_rad'] and fit['radial_rmse_m']<=g['maximum_circle_rmse_m'] and cv<=g['maximum_radius_subfit_cv'] and all(f['absolute_arc_rad']>=g['subfit_minimum_arc_rad'] for f in subfits) and fit['signed_arc_rad']*plan['yaw_rate_radps']>0,
              {'fit':fit,'subfits':subfits,'radius_cv':cv})
        park=(t>=timing['parking_start_s']-1e-9)&(t<=timing['parking_end_s']+1e-9)
        pk=position[park,:2];py=yaw[park];pv=np.linalg.norm(body_origin[park,:2],axis=1);pw=np.abs(omega[park,2])
        tp=(tt>=timing['parking_start_s']-1e-9)&(tt<=timing['parking_end_s']+1e-9)
        park_drift=float(np.max(np.linalg.norm(pk-pk[0],axis=1)));park_yaw=float(np.max(abs(py-py[0])))
        parking={'fixed_window_s':[28.,33.],'native_samples':int(np.sum(park)),'policy_samples':int(np.sum(tp)),
            'peak_planar_origin_mps':float(np.max(pv)),'peak_body_yaw_radps':float(np.max(pw)),
            'xy_max_drift_m':park_drift,'yaw_max_unwrapped_drift_rad':park_yaw}
        check('fixed_5s_Teacher_zero_velocity_parking',np.sum(park)>=1001*g['minimum_native_coverage_fraction'] and np.sum(tp)>=250*g['minimum_native_coverage_fraction'] and np.max(abs(request[tp]))<1e-9 and np.max(abs(actual_cmd[tp]))<1e-9 and np.max(pv)<=g['parking_planar_peak_mps'] and np.max(pw)<=g['parking_yaw_rate_peak_radps'] and park_drift<=g['parking_xy_drift_m'] and park_yaw<=g['parking_yaw_drift_rad'],parking)
        path_heading=np.arctan2(world_origin[cruise,1],world_origin[cruise,0]);slip=np.arctan2(np.sin(path_heading-yaw[cruise]),np.cos(path_heading-yaw[cruise]))
        metrics.update(command_radius_m=plan['command_radius_m'],actual_fitted_radius_m=fit['radius_m'],circle_fit=fit,
            actual_radius_subfit_cv=cv,mean_velocity_ratio_radius_m=float(abs(np.mean(v)/np.mean(w))) if abs(np.mean(w))>1e-6 else None,
            speed_forward_mae_mps=vmae,yaw_rate_mae_radps=wmae,heading_slip_p95_rad=float(np.percentile(abs(slip),95)),
            actual_origin_path_length_m=float(np.sum(np.linalg.norm(np.diff(cp,axis=0),axis=1))),
            actual_unwrapped_yaw_change_rad=float(yaw[cruise][-1]-yaw[cruise][0]),parking=parking,
            max_abs_roll_pitch_rad=float(np.max(np.abs(np.column_stack((roll,pitch))[safe]))),
            minimum_base_clearance_m=float(np.min(position[safe,2])),native_state_time_semantics='PreUpdate t-dt; origin velocity = COM body velocity minus omega cross actual COM offset')
        arrays={'native_state_time_s':t,'native_position':position,'native_body_origin_velocity':body_origin,'native_body_ang_velocity':omega,
                'native_unwrapped_yaw':yaw,'telemetry_time_s':tt,'command_body':actual_cmd,'requested_body':request,
                'cruise_time_s':ct,'cruise_forward_mps':v,'rolling_time_s':rt,'rolling_forward_mps':rv,'heading_slip_rad':slip}
    except Exception as exc:
        check('complete_original_evidence',None,{'error':repr(exc),'no_missing_evidence_pass':True})
    passed=bool(checks) and all(c['status']=='passed' for c in checks.values())
    summary={'schema':'teacher_continuous_turn_plant_independent/v1','status':'passed' if passed else 'failed','run':str(run),
        'checks':checks,'metrics':metrics,'input_hashes':inputs,'analyzer_sha256':sha(__file__),
        'levels':{'continuous_turn_plant': 'passed' if passed else 'failed','PID_curve_tracking':'unverified','SLAM_navigation':'unverified','multifloor':'unverified','real_robot':'unverified'},
        'claim':'Only the exact tested speed and signed rate on the new plane; commanded radius is not achieved radius. Boundary repeatability requires subsequent fixed repeats.',
        'minimum_radius_claim':'No per-run global minimum or untested-radius claim'}
    (run/'summary_radius_independent.json').write_text(json.dumps(serial(summary),indent=2,allow_nan=False)+'\n')
    if arrays:np.savez_compressed(run/'radius_evidence_arrays.npz',**arrays)
    print(json.dumps({'run':str(run),'status':summary['status'],'checks':{k:v['status'] for k,v in checks.items()}},allow_nan=False))
    return summary


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);evaluate(ap.parse_args().run)
