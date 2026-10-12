#!/usr/bin/env python3
"""Independent truth-feedback Teacher benchmark; never SLAM acceptance."""
from __future__ import annotations
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def rows(path):
    output=[]
    with Path(path).open() as stream:
        for index,line in enumerate(stream,1):
            if not line.strip():continue
            try:output.append(json.loads(line))
            except (ValueError,TypeError) as error:
                raise ValueError(f'{Path(path).name}:{index}: invalid original JSON') from error
    return output


def array(data,key,width=None):
    a=np.asarray([r[key]for r in data],dtype=float)
    if width is not None and a.shape!=(len(data),width):raise ValueError('Invalid telemetry array '+key)
    if not np.isfinite(a).all():raise ValueError('Nonfinite telemetry '+key)
    return a


def rotation_wxyz(q):
    if q.shape[1:]!=(4,)or np.any(abs(np.sum(q*q,axis=1)-1)>.02):raise ValueError('Invalid recorded quaternion')
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
        2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)


def angular_difference(a,b):return np.arctan2(np.sin(a-b),np.cos(a-b))


def polyline_distance(points,route):
    best=np.full(len(points),np.inf);segment=np.full(len(points),-1,dtype=int)
    for i,(a,b)in enumerate(zip(route[:-1,:2],route[1:,:2])):
        delta=b-a;length2=float(delta@delta)
        if length2<=1e-12:raise ValueError('Degenerate requested route XY leg')
        frac=np.clip((points[:,:2]-a)@delta/length2,0,1)
        distance=np.linalg.norm(points[:,:2]-(a+frac[:,None]*delta),axis=1)
        replace=distance<best;best[replace]=distance[replace];segment[replace]=i
    return best,segment


def control_rows(data):
    output=[]
    for row in data:
        tag=row.get('type',row.get('event',row.get('kind')))
        if tag=='controller_update'or(tag is None and 'control_t_s'in row):output.append(row)
    if not output:raise ValueError('No actual controller_update rows')
    for row in output:
        for field in ('control_t_s','feedback_time_s','reference_yaw'):
            if not math.isfinite(float(row[field])):raise ValueError('Nonfinite control '+field)
        for field,width in (('reference_xy',2),('reference_velocity_world',3),('command_body',3)):
            value=np.asarray(row[field],float)
            if field=='reference_velocity_world'and value.shape==(2,)and np.isfinite(value).all():continue
            if value.shape!=(width,)or not np.isfinite(value).all():raise ValueError('Invalid control '+field)
        if not isinstance(row['mode'],str):raise ValueError('Missing actual controller mode')
    return output


def evaluate_run(run,write=True):
    run=Path(run).resolve();protocol_path=HERE/'protocol.json'
    protocol=json.loads(protocol_path.read_text());profile_path=run/'truth_profile.json'
    output_path=run/'summary_truth_pid.json';arrays_path=run/'truth_pid_independent_arrays.npz'
    if write and (output_path.exists()or arrays_path.exists()):raise ValueError('Refusing to replace prior independent truth evidence')
    source_paths=[protocol_path,Path(__file__).resolve(),profile_path,run/'telemetry.jsonl',run/'control.jsonl']
    result={'schema':'independent_truth_teacher_pid_benchmark/v1','run':str(run),'status':'failed',
        'scope':'Simulator-feedback outer velocity PID benchmark only; not sensor-SLAM navigation',
        'navigation_ground_truth_used':True,'SLAM_navigation_verified':False,'real_robot_verified':False,
        'strict_terrain_motion_verified':False,'score':None,'checks':{},'limitations':[
            'Actual body COM velocities are rotated by the actual quaternion. Joint targets are not motion evidence.',
            'Safety is limited to recorded telemetry cadence; no claim about unrecorded native physics extrema.',
            'Truth feedback is an explicit tuning benchmark, not a SLAM navigation integration result.']}
    independent={}
    def check(name,passed,status=None,**values):
        result['checks'][name]={'status':status or('passed'if passed else'failed'),**values}
        return bool(passed)
    try:
        profile=json.loads(profile_path.read_text());data=rows(run/'telemetry.jsonl');control=control_rows(rows(run/'control.jsonl'))
        if len(data)<2:raise ValueError('Insufficient original telemetry')
        route=np.asarray(profile['route_world_xyz'],float)
        if route.ndim!=2 or route.shape[0]<2 or route.shape[1]!=3 or not np.isfinite(route).all():raise ValueError('Finite N>=2 route XYZ required')
        speed=float(profile['desired_speed']);duration=float(profile['duration']);feedback_hz=float(profile['feedback_hz'])
        if not all(math.isfinite(v)and v>0 for v in(speed,duration,feedback_hz)):raise ValueError('Invalid frozen profile speed/duration/frequency')
        terrain=str(profile['terrain']);result['profile']=profile
        world_t=array(data,'world_sim_time');t=array(data,'state_physics_world_time');pos=array(data,'position',3)
        quat=array(data,'quaternion_wxyz',4);R=rotation_wxyz(quat)
        body_velocity=array(data,'body_lin_vel',3);world_velocity=np.einsum('nij,nj->ni',R,body_velocity)
        body_omega=array(data,'body_ang_vel',3);rpy=array(data,'rpy',3);yaw=np.arctan2(R[:,1,0],R[:,0,0])
        q=array(data,'q',12);qd=array(data,'qd',12);tau=array(data,'applied_torque',12)
        clearance=array(data,'body_clearance');requested=array(data,'requested',3);command=array(data,'command',3)
        if any(type(row.get('contacts'))is not dict or 'body'not in row['contacts']for row in data):raise ValueError('Actual body-contact evidence missing')
        contacts=np.asarray([float(row['contacts']['body'])for row in data])
        if not np.isfinite(contacts).all()or np.any(contacts<0):raise ValueError('Unavailable/invalid actual body contacts')
        faults=[{'index':i,'physics_time_s':float(t[i]),'fault':row.get('fault')}for i,row in enumerate(data)if row.get('fault')not in(None,0,'')]
        bootstrap=float(protocol['time']['bootstrap_excluded_s']);audit=t>=bootstrap
        if not audit.any():raise ValueError('No physical data beyond bootstrap')
        maxgap=float(np.max(np.diff(t)));offset_error=float(np.max(abs(t-(world_t-.005))))
        end_t=float(t[-1]);elapsed_complete=end_t>=duration-.041
        check('physical_time_and_telemetry_coverage',np.all(np.diff(t)>0)and maxgap<=protocol['time']['maximum_telemetry_gap_s']and
            offset_error<=protocol['time']['pose_offset_tolerance_s']and elapsed_complete,
            first_physical_time_s=float(t[0]),last_physical_time_s=end_t,planned_duration_s=duration,
            maximum_gap_s=maxgap,actual_recorded_rows=len(data),measured_pose_offset_error_s=offset_error)
        ct=np.asarray([float(row['control_t_s'])for row in control]);ft=np.asarray([float(row['feedback_time_s'])for row in control])
        ref_yaw=np.asarray([float(row['reference_yaw'])for row in control]);rv=np.asarray([list(row['reference_velocity_world'])[:2]for row in control],float)
        check('original_control_clock_and_feedback_causality',np.all(np.diff(ct)>0)and np.all(ft<=ct+1e-8)and np.all(ct-ft>=-1e-8),
            controller_rows=len(control),feedback_hz_requested=feedback_hz,
            maximum_control_feedback_delay_s=float(np.max(ct-ft)),minimum_control_feedback_delay_s=float(np.min(ct-ft)))
        # A control computed at time c can only describe a physical pose at/after
        # c. Never match a later reference to an earlier PreUpdate cached pose.
        indexes=np.searchsorted(ct,t,side='right')-1;bound=np.clip(indexes,0,len(control)-1)
        age=t-ct[bound];valid=(indexes>=0)&(age>=-1e-8)&(age<=protocol['time']['maximum_causal_control_age_s'])
        speed_ref=np.linalg.norm(rv[bound],axis=1);mode=np.asarray([control[i]['mode']for i in bound])
        moving=valid&audit&(speed_ref>protocol['speed']['active_reference_min_mps'])
        check('causal_control_reference_coverage',np.all(valid[audit])and moving.any(),
            missing_or_old_reference_rows=int((~valid&audit).sum()),maximum_causal_age_s=float(age[valid].max())if valid.any()else None,
            drive_rows=int(moving.sum()),drive_modes=sorted(set(mode[moving].tolist())))
        route_distance,nearest_leg=polyline_distance(pos,route)
        check('flat_or_requested_route_distance',bool(moving.any())and float(route_distance[audit].max())<=protocol['route']['maximum_xy_distance_m']and
            float(np.sqrt(np.mean(route_distance[moving]**2)))<=protocol['route']['drive_xy_distance_rms_m'],
            maximum_xy_distance_m=float(route_distance[audit].max()),
            drive_xy_distance_rms_m=float(np.sqrt(np.mean(route_distance[moving]**2)))if moving.any()else None,
            maximum_allowed_m=protocol['route']['maximum_xy_distance_m'],rms_allowed_m=protocol['route']['drive_xy_distance_rms_m'])
        heading_error=abs(angular_difference(yaw,ref_yaw[bound]));heading_max=float(heading_error[moving].max())if moving.any()else None
        check('drive_heading',heading_max is not None and heading_max<=protocol['route']['maximum_drive_heading_error_rad'],
            maximum_error_rad=heading_max,limit_rad=protocol['route']['maximum_drive_heading_error_rad'],reference='causal recorded reference_yaw',
            maximum_error_time_s=float(t[np.flatnonzero(moving)[np.argmax(heading_error[moving])]])if moving.any()else None)
        direction=np.zeros_like(rv[bound]);np.divide(rv[bound],speed_ref[:,None],out=direction,where=speed_ref[:,None]>1e-12)
        actual_along=np.sum(world_velocity[:,:2]*direction,axis=1)
        first_move=float(t[np.flatnonzero(moving)[0]])if moving.any()else math.inf
        stable=moving&(t>=first_move+protocol['time']['speed_startup_excluded_s'])&(speed_ref>=speed*protocol['speed']['full_speed_reference_fraction_min'])
        dt=np.r_[np.diff(t),np.median(np.diff(t))];stable_duration=float(dt[stable].sum())
        mean_reference=float(np.mean(speed_ref[stable]))if stable.any()else None
        speed_error=float(np.mean(abs(actual_along[stable]-speed_ref[stable])))if stable.any()else None
        speed_limit=max(protocol['speed']['maximum_absolute_error_floor_mps'],protocol['speed']['maximum_relative_error_fraction']*mean_reference)if mean_reference is not None else None
        check('stable_real_COM_speed',stable_duration>=protocol['time']['minimum_stable_speed_duration_s']and speed_error is not None and speed_error<=speed_limit,
            desired_profile_speed_mps=speed,mean_reference_mps=mean_reference,
            mean_actual_forward_projection_mps=float(np.mean(actual_along[stable]))if stable.any()else None,
            mean_absolute_error_mps=speed_error,limit_mps=speed_limit,stable_duration_s=stable_duration,stable_rows=int(stable.sum()),
            velocity_source='actual COM body_lin_vel, actual quaternion body-to-world rotation, causal reference-velocity projection')
        rp_max=float(abs(rpy[audit,:2]).max());clear_min=float(clearance[audit].min());tau_max=float(abs(tau[audit]).max());qd_max=float(abs(qd[audit]).max())
        safe=(rp_max<=protocol['safety']['maximum_abs_roll_pitch_rad']and clear_min>=protocol['safety']['minimum_body_clearance_m']and
            tau_max<=protocol['safety']['maximum_abs_applied_torque_Nm']and qd_max<=protocol['safety']['maximum_abs_joint_velocity_radps']and
            not np.any(contacts[audit]>0)and not faults)
        check('as_recorded_physical_safety',safe,maximum_abs_roll_pitch_rad=rp_max,minimum_body_clearance_m=clear_min,
            maximum_abs_applied_torque_Nm=tau_max,maximum_abs_joint_velocity_radps=qd_max,
            body_contact_rows=int((contacts[audit]>0).sum()),faults=faults,measured_motor_torque=False,
            coverage='original telemetry rows only; not every native physics iteration')
        # Ordered waypoint visits prevent the initial origin from being called
        # completion of an outward-and-return route.
        cursor=int(np.flatnonzero(audit)[0]);visits=[]
        for number,goal in enumerate(route[1:],1):
            candidates=np.flatnonzero((np.arange(len(t))>=cursor)&(np.linalg.norm(pos[:,:2]-goal[:2],axis=1)<=protocol['route']['goal_xy_tolerance_m']))
            if not len(candidates):break
            index=int(candidates[0]);visits.append({'waypoint_index':number,'physics_time_s':float(t[index]),'telemetry_index':index,
                'actual_position':pos[index].tolist(),'goal':goal.tolist(),'XY_error_m':float(np.linalg.norm(pos[index,:2]-goal[:2])),
                'Z_error_m':float(pos[index,2]-goal[2])});cursor=index+1
        completed_route=len(visits)==len(route)-1;final_error=float(np.linalg.norm(pos[-1,:2]-route[-1,:2]))
        check('ordered_route_and_final_goal',completed_route and final_error<=protocol['route']['goal_xy_tolerance_m'],
            ordered_visits=visits,required_waypoints=len(route)-1,final_goal_xy_error_m=final_error,
            final_goal_z_error_m=float(pos[-1,2]-route[-1,2]),arrival_is_true_simulation_pose_not_SLAM=True)
        parking=None;park=None;zero=(np.max(abs(requested),axis=1)<=1e-9)&(np.max(abs(command),axis=1)<=1e-9)&(speed_ref<=1e-9)&valid
        if completed_route:
            ready=np.flatnonzero(zero&(np.arange(len(t))>=visits[-1]['telemetry_index'])&(np.linalg.norm(pos[:,:2]-route[-1,:2],axis=1)<=protocol['route']['goal_xy_tolerance_m']))
            if len(ready):
                first=int(ready[0]);start=float(t[first]);end=start+protocol['parking']['fixed_window_s'];park=(t>=start)&(t<=end+1e-9)
                drift=float(np.linalg.norm(pos[park,:2]-pos[first,:2],axis=1).max());yd=float(abs(np.unwrap(yaw[park])-yaw[first]).max())
                covered=end_t>=end-.00000001 and float(t[park][-1])>=end-.025
                parking={'window_s':[start,end],'rows':int(park.sum()),'complete_5s':covered,'xy_drift_m':drift,'yaw_drift_rad':yd,
                    'continuous_requested_and_actor_command_zero':bool(zero[park].all()),'window_selection':'first eligible zero final-goal row; never a later quiet window'}
        park_ok=(parking is not None and parking['complete_5s']and parking['continuous_requested_and_actor_command_zero']and
            parking['xy_drift_m']<=protocol['parking']['maximum_xy_drift_m']and parking['yaw_drift_rad']<=protocol['parking']['maximum_yaw_drift_rad'])
        check('fixed_final_goal_parking',park_ok,status='passed'if park_ok else'unverified'if parking is None or not parking['complete_5s']else'failed',
            parking=parking,xy_limit_m=protocol['parking']['maximum_xy_drift_m'],yaw_limit_rad=protocol['parking']['maximum_yaw_drift_rad'])
        is_flat=terrain in('flat','flat_short','flat_long','flat_roundtrip')
        if is_flat:
            check('strict_terrain_support_applicability',True,terrain=terrain,meaning='flat benchmark; no ramp/step support claim')
        else:
            check('strict_terrain_support_applicability',False,status='unverified',terrain=terrain,
                meaning='Actual foot contact XYZ/normal plus collision geometry and ordered support/continuation are not in this telemetry schema; no strict terrain motion pass',
                requested_route_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()),
                body_z_change_diagnostic_m=float(pos[-1,2]-pos[0,2]),world_z_gain_ratio_used=False)
            result['limitations'].append('Ramp 12m and low-step traversal remain unverified until independent actual foot geometry/contact coverage is evaluated; body rise is diagnostic only.')
        passed=all(row['status']=='passed'for row in result['checks'].values());result['status']='passed'if passed else'failed'
        if passed:
            score=100/(1+result['checks']['flat_or_requested_route_distance']['drive_xy_distance_rms_m']/.08+
                speed_error/speed_limit+heading_max/.2+parking['xy_drift_m']/.05+parking['yaw_drift_rad']/.1)
            result['score']=float(score)
        result['failed_checks']=[key for key,row in result['checks'].items()if row['status']=='failed']
        result['unverified_checks']=[key for key,row in result['checks'].items()if row['status']=='unverified']
        independent={'state_physics_world_time':t,'world_sim_time':world_t,'position':pos,'quaternion_wxyz':quat,
            'world_COM_velocity':world_velocity,'body_COM_velocity':body_velocity,'body_angular_velocity':body_omega,
            'route_world_xyz':route,'route_xy_distance':route_distance,'nearest_route_leg':nearest_leg,
            'causal_control_index':indexes,'causal_reference_age_s':age,'causal_reference_valid':valid,'drive_mask':moving,
            'stable_speed_mask':stable,'actual_forward_projection_mps':actual_along,'reference_speed_mps':speed_ref,
            'heading_error_rad':heading_error,'control_t_s':ct,'feedback_time_s':ft,
            'q':q,'qd':qd,'applied_torque':tau,'body_clearance':clearance,'requested':requested,'command':command}
    except (OSError,ValueError,KeyError,TypeError,IndexError)as error:
        check('complete_parseable_required_evidence',False,reason=type(error).__name__+': '+str(error))
        result['status']='failed';result['score']=None
    result['input_source_sha256']={str(path):sha(path)for path in source_paths if path.is_file()}
    if write:
        if independent:
            with arrays_path.open('xb')as stream:np.savez_compressed(stream,**independent)
            result['independent_arrays']={'path':str(arrays_path),'sha256':sha(arrays_path)}
        with output_path.open('x')as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();result=evaluate_run(args.run)
    print(json.dumps({'status':result['status'],'score':result['score'],'summary':str(args.run.resolve()/'summary_truth_pid.json'),
        'SLAM_navigation_verified':False},ensure_ascii=False))


if __name__=='__main__':main()
