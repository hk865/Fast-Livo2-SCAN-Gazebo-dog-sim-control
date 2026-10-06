#!/usr/bin/env python3
"""Append-only actual continuous-curve evidence; simulation truth, never SLAM.

Measurements, cadence, origin velocity, yaw, traversal and continuity are
reconstructed from original native/actor logs. Geometric formulas are shared
with the frozen StaticPath archive, not claimed as a fully independent formula.
No Controller is instantiated; no robot, simulator, process or ancestor write.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np


MODEL_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
CORE_SHA='82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d'
COMMON_SCHEMA='independent_truth_teacher_pid_benchmark/v2'
COMMON_CHECKS=('physical_time_and_telemetry_coverage','original_control_clock_and_feedback_causality',
    'causal_control_reference_coverage','native_physics_coverage_and_state_pairing',
    'flat_or_requested_route_distance','drive_heading','stable_real_COM_speed',
    'as_recorded_physical_safety','ordered_route_and_final_goal',
    'declared_completion_and_termination','fixed_final_goal_parking','actual_runtime_receipt')
OUTPUT='summary_curve_independent.json'
ARRAY_OUTPUT='curve_evidence_arrays.npz'


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def require_digest(path,expected):
    actual=sha(path)
    if actual!=expected:raise ValueError('SHA mismatch: '+str(path))
    return actual


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def common_pass(common):
    """Require the untouched original v2 flat receipt, including all gates."""
    checks=common.get('checks',{})
    return (common.get('schema')==COMMON_SCHEMA and common.get('status')=='passed' and
            common.get('navigation_ground_truth_used') is True and
            common.get('SLAM_navigation_verified') is False and
            common.get('real_robot_verified') is False and
            all(checks.get(k,{}).get('status')=='passed' for k in COMMON_CHECKS) and
            checks.get('strict_terrain_support_applicability',{}).get('status')=='passed' and
            bool(checks) and all(v.get('status')=='passed' for v in checks.values()))


def finite_array(data,key,width=None):
    a=np.asarray([x[key] for x in data],float)
    if width is not None and a.shape!=(len(data),width):raise ValueError('Invalid '+key+' shape')
    if not np.isfinite(a).all():raise ValueError('Nonfinite '+key)
    return a


def rotations(q):
    if q.ndim!=2 or q.shape[1]!=4 or np.any(abs(np.sum(q*q,axis=1)-1)>.02):
        raise ValueError('Invalid native quaternion')
    w,x,y,z=(q/np.linalg.norm(q,axis=1)[:,None]).T
    return np.stack((1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
        2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),axis=1).reshape(-1,3,3)


def native_arrays(native,offset):
    if len(native)<2:raise ValueError('No original native coverage')
    t=finite_array(native,'t');dt=finite_array(native,'dt');iteration=finite_array(native,'iteration')
    position=finite_array(native,'position',3);q=finite_array(native,'quaternion_wxyz',4)
    com=finite_array(native,'body_lin_vel_com',3);omega=finite_array(native,'body_ang_vel',3)
    R=rotations(q);origin_body=com-np.cross(omega,np.asarray(offset,float))
    origin_world=np.einsum('nij,nj->ni',R,origin_body)
    native_origin=finite_array(native,'body_lin_vel_origin',3)
    roll=np.arctan2(R[:,2,1],R[:,2,2]);pitch=np.arcsin(np.clip(-R[:,2,0],-1,1))
    if np.any(abs(np.cos(pitch))<.4):raise ValueError('Unsafe Euler yaw kinematics')
    yawdot=(np.sin(roll)*omega[:,1]+np.cos(roll)*omega[:,2])/np.cos(pitch)
    yaw=np.unwrap(np.arctan2(R[:,1,0],R[:,0,0]))
    return dict(t=t-dt,force_t=t,dt=dt,iteration=iteration,position=position,
                quaternion_wxyz=q,body_COM_velocity=com,body_omega=omega,
                origin_body_velocity=origin_body,origin_world_velocity=origin_world,
                recorded_native_origin_body_velocity=native_origin,
                Euler_yaw_rate=yawdot,unwrapped_yaw=yaw)


def native_coverage(a):
    if len(a['t'])<2:return False
    return bool(np.all(np.diff(a['iteration'])==1) and np.all(abs(a['dt']-.005)<1e-8) and
                np.all(abs(np.diff(a['force_t'])-.005)<1e-8) and np.all(np.diff(a['t'])>0))


def continuous_modes(mode,curved,valid):
    bad=sorted(set(mode[curved & np.isin(mode,['pre_turn','turn','settle'])].tolist()))
    return bool(curved.any() and not bad and np.all(valid[curved])),bad


def geometric_trace(path,position):
    progress=np.zeros(len(position));point=np.zeros_like(position)
    tangent=np.zeros_like(position);normal=np.zeros_like(position)
    heading=np.zeros(len(position));curvature=np.zeros(len(position))
    previous=0.
    for i,p in enumerate(position):
        previous,_=path.project(p,previous)
        progress[i]=previous
        point[i],tangent[i],normal[i],heading[i],curvature[i]=path.geometry(previous)
    delta=position-point
    return dict(progress=progress,point=point,tangent=tangent,normal=normal,
                reference_heading=heading,reference_curvature=curvature,
                signed_cross=np.sum(delta[:,:2]*normal[:,:2],axis=1),
                nearest_xy_distance=np.linalg.norm(delta[:,:2],axis=1))


def ordered_traversal(path,a,g,tolerance=.15):
    # Fixed checkpoints are geometric arc positions, not controller timestamps.
    checkpoints=np.unique(np.r_[np.arange(.25,path.length,.25),path.entry,
                                 path.entry+path.curve_length,path.length])
    cursor=0;visits=[]
    for s in checkpoints:
        target=path.position(s)
        candidates=np.flatnonzero((g['progress'][cursor:]>=s-tolerance) &
            (np.linalg.norm(a['position'][cursor:,:2]-target[:2],axis=1)<=tolerance))
        if not len(candidates):
            return False,{'required_checkpoints':len(checkpoints),'visited':visits,'missing_first_arc_m':float(s)}
        cursor+=int(candidates[0]);visits.append({'arc_m':float(s),'native_index':cursor,
                                               'state_physics_time_s':float(a['t'][cursor])})
    complete=bool(g['progress'][-1]>=path.length-.15)
    return complete,{'required_checkpoints':len(checkpoints),'visited':visits,
                     'actual_final_progress_m':float(g['progress'][-1]),'required_final_m':path.length-.15}


def rolling_forward(t,velocity,eligible,width_s=1.):
    """Every full original 1s sliding interval; never choose a quiet/best bin."""
    if len(t)<2:return dict(complete_windows=0,minimum_mean_mps=None,time=np.array([]),means=np.array([]))
    integrated=np.r_[0.,np.cumsum(.5*(velocity[1:]+velocity[:-1])*np.diff(t))]
    invalid=np.r_[0,np.cumsum(~eligible)]
    starts=np.arange(len(t));ends=np.searchsorted(t,t+width_s-1e-8,side='left')
    inside=ends<len(t);starts=starts[inside];ends=ends[inside]
    exact=abs(t[ends]-t[starts]-width_s)<=1e-8
    uninterrupted=(invalid[ends+1]-invalid[starts])==0
    starts=starts[exact&uninterrupted];ends=ends[exact&uninterrupted]
    means=(integrated[ends]-integrated[starts])/(t[ends]-t[starts]) if len(starts) else np.array([])
    return dict(complete_windows=len(starts),minimum_mean_mps=float(means.min()) if len(means) else None,
                time=t[starts],means=means)


def turning_segments(path,a,g):
    if path.kind=='circle':
        intervals=[(path.entry,path.entry+path.curve_length,path.sign)]
    else:
        e=path.entry;L=path.curve_length
        intervals=[(e,e+L/4,path.sign),(e+L/4,e+3*L/4,-path.sign),(e+3*L/4,e+L,path.sign)]
    output=[];passed=True
    for lower,upper,sign in intervals:
        indices=np.flatnonzero((g['progress']>=lower-1e-8)&(g['progress']<=upper+1e-8))
        if len(indices)<2:
            passed=False;output.append({'arc_interval_m':[lower,upper],'status':'unverified','native_samples':len(indices)})
            continue
        i,j=int(indices[0]),int(indices[-1])
        ref=float(g['reference_heading'][j]-g['reference_heading'][i])
        actual=float(a['unwrapped_yaw'][j]-a['unwrapped_yaw'][i])
        complete=g['progress'][j]>=upper-.15 and g['progress'][i]<=lower+.15
        good=complete and sign*ref>1e-6 and sign*actual>0
        if path.kind=='circle':good=good and sign*actual>=2*math.pi-.4
        passed=passed and good
        output.append({'arc_interval_m':[lower,upper],'direction_sign':sign,
                       'native_samples':len(indices),'state_time_interval_s':[float(a['t'][i]),float(a['t'][j])],
                       'reference_accumulated_heading_rad':ref,'actual_accumulated_Euler_yaw_rad':actual,
                       'status':'passed' if good else 'failed',
                       'method':'Original unwrapped body yaw accumulated over fixed signed-curvature geometric segment; gait oscillations retained'})
    return bool(passed),output


def motion_curvature_diagnostic(a,eligible):
    """True XY motion curvature from fixed 1s quadratic position fits.

    No local-circle match gate for varying-curvature S; no best-window search.
    These spatial diagnostics supplement, never replace, raw velocity/yaw logs.
    """
    t=a['t'];starts=np.arange(t[0],t[-1]-1+.000001,.5);output=[]
    for start in starts:
        mask=(t>=start-1e-8)&(t<=start+1+1e-8)
        if mask.sum()<201 or not np.all(eligible[mask]):continue
        tau=t[mask]-(start+.5);M=np.column_stack((np.ones(len(tau)),tau,tau*tau))
        coef=np.linalg.lstsq(M,a['position'][mask,:2],rcond=None)[0]
        v=coef[1];acc=2*coef[2];speed=np.linalg.norm(v)
        kappa=float((v[0]*acc[1]-v[1]*acc[0])/speed**3) if speed>.05 else None
        output.append({'center_time_s':float(start+.5),'fit_window_s':[float(start),float(start+1)],
                       'actual_geometric_motion_curvature_1pm':kappa,'fit_planar_speed_mps':float(speed),
                       'position_fit_rms_m':float(np.sqrt(np.mean((M@coef-a['position'][mask,:2])**2)))})
    return output


def _serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:_serial(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [_serial(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def evaluate(run,write=True):
    run=Path(run).resolve();output=run/OUTPUT;array_output=run/ARRAY_OUTPUT
    if write and (output.exists() or array_output.exists()):raise FileExistsError('Never replace existing curve evidence')
    checks={};verified={};arrays={};ancestor=None;metrics={}
    def check(name,ok,**details):
        checks[name]={'status':'unverified' if ok is None else 'passed' if bool(ok) else 'failed',**_serial(details)}
    def verify(path,expected=None):
        path=Path(path).resolve();actual=sha(path)
        if expected is not None:require_digest(path,expected)
        verified[str(path)]=actual;return actual
    try:
        names=('summary_truth_pid.json','truth_profile.json','source_manifest.json','runtime_manifest.json',
               'policy_manifest.json','worker_result.json','actuator.jsonl','telemetry.jsonl','control.jsonl',
               'observations_actions.npz','world.sdf')
        for name in names:verify(run/name)
        common=json.loads((run/'summary_truth_pid.json').read_text())
        profile=json.loads((run/'truth_profile.json').read_text())
        runtime=json.loads((run/'runtime_manifest.json').read_text())
        policy=json.loads((run/'policy_manifest.json').read_text())
        manifest=json.loads((run/'source_manifest.json').read_text())
        ancestor={'path':str(run/'summary_truth_pid.json'),'sha256':verified[str(run/'summary_truth_pid.json')],
                  'status':common.get('status'),'schema':common.get('schema'),'checks':common.get('checks',{})}
        check('untouched_original_flat_common_all_passed',common_pass(common) and profile['terrain']=='flat',
              required_original_checks=list(COMMON_CHECKS),original_status=common.get('status'),
              original_numerical_thresholds_changed=False)
        for path,expected in common['input_source_sha256'].items():verify(path,expected)
        verify(common['independent_arrays']['path'],common['independent_arrays']['sha256'])
        required_sources=('truth/core.py','truth/worker.py','truth/run.py','truth/evaluate.py','truth/protocol.json',
                          'truth/profiles.py','truth/pure_tests.py','truth/curve_receipt.py',
                          'truth/test_curve_receipt.py','truth/README_CURVE_RECEIPT.md',
                          'policy/worker.py','policy/observation.py','policy/contract.json',
                          'native/teacher_actuator.cpp','native/libteacher_actuator.so')
        if not set(required_sources)<=set(manifest):raise ValueError('Missing required frozen archive sources')
        for name,digest in manifest.items():
            verify(run/'sources'/name,digest if isinstance(digest,str) else digest['sha256'])
        verify(run/'source_manifest.json',runtime['source_manifest_sha256'])
        verify(run/'truth_profile.json',runtime['truth_profile_sha256'])
        verify(run/'sources/native/libteacher_actuator.so',runtime['native_plugin_sha256'])
        core_path=run/'sources/truth/core.py';verify(core_path,CORE_SHA)
        verify(run/'sources/truth/curve_receipt.py',sha(__file__))
        verify(policy['checkpoint'],MODEL_SHA)
        CPU=(policy['checkpoint_sha256']==MODEL_SHA and runtime['frozen_model_sha256']==MODEL_SHA and
             policy['inference_device']=='cpu' and policy['torch_threads']==1 and
             runtime['exclusive_writer']=='teacher_sim::TeacherActuator' and
             runtime['uses_truth_for_control'] is True and runtime['counts_as_SLAM_navigation'] is False and
             runtime['training_processes_signaled'] is False and runtime['real_robot'] is False)
        check('all_frozen_source_inputs_model_CPU_and_scope',CPU,
              model_sha256=MODEL_SHA,privileged_actor_observation_dimensions=232,actor_input_dimensions=247,
              known_command_and_previous_action_dimensions=15,source_manifest_verified=True)
        module_name='_curve_geometry_'+hashlib.sha256(str(run).encode()).hexdigest()[:12]
        spec=importlib.util.spec_from_file_location(module_name,core_path)
        module=importlib.util.module_from_spec(spec)
        previous=sys.dont_write_bytecode;sys.dont_write_bytecode=True
        try:spec.loader.exec_module(module)
        finally:sys.dont_write_bytecode=previous
        path=module.StaticPath(profile['path'])
        check('immutable_analytic_path_parameters',path.sha256==profile['path_parameters_sha256'],
              actual_parameter_sha256=path.sha256,kind=path.kind,path_length_m=path.length,
              geometry_independence='Measurement reconstruction is independent; StaticPath formulas are shared with hash-pinned archived controller geometry, not independently rederived')
        protocol=json.loads((run/'sources/truth/protocol.json').read_text())
        if (protocol['route']['maximum_xy_distance_m']!=.2 or protocol['route']['drive_xy_distance_rms_m']!=.08 or
            protocol['route']['maximum_drive_heading_error_rad']!=.2 or protocol['parking']['maximum_xy_drift_m']!=.05 or
            protocol['parking']['maximum_yaw_drift_rad']!=.1):raise ValueError('Original motion thresholds changed')
        all_native=rows(run/'actuator.jsonl');native=[r for r in all_native if r.get('kind')=='physics_step']
        control=rows(run/'control.jsonl');telemetry=rows(run/'telemetry.jsonl')
        with np.load(run/'observations_actions.npz',allow_pickle=False) as actor:
            observations=actor['observations'];actions=actor['actions']
            actor_coverage=(observations.shape==(len(telemetry),247) and actions.shape==(len(telemetry),12) and
                            np.isfinite(observations).all() and np.isfinite(actions).all())
        check('original_finite_247_by_12_actor_evidence',actor_coverage,
              observations_shape=list(observations.shape),actions_shape=list(actions.shape),
              original_actor_rows=len(telemetry),inference_reexecuted=False)
        a=native_arrays(native,profile['base_com_offset']);t=a['t']
        ct=finite_array(control,'control_t_s');ft=finite_array(control,'feedback_time_s')
        tt=finite_array(telemetry,'state_physics_world_time')
        check('every_native_200Hz_step_and_original_actor_control_coverage',
              native_coverage(a) and len(control)==len(telemetry) and len(telemetry)>1 and
              np.all(np.diff(ct)>0) and np.all(np.diff(tt)>0) and t[0]<=tt[0]+1e-8 and t[-1]>=tt[-1]-1e-8,
              native_rows=len(native),actor_rows=len(telemetry),maximum_native_gap_s=float(np.diff(t).max()))
        origin_error=float(np.max(abs(a['origin_body_velocity']-a['recorded_native_origin_body_velocity'])))
        check('body_origin_COM_and_frame_semantics_recomputed',origin_error<=1e-9,
              maximum_native_recorded_vs_recomputed_origin_velocity_mps=origin_error)
        g=geometric_trace(path,a['position'])
        bound=np.searchsorted(ct,t,side='right')-1;bi=np.clip(bound,0,len(ct)-1)
        age=t-ct[bi];valid=(bound>=0)&(age>=-1e-8)&(age<=protocol['time']['maximum_causal_control_age_s'])
        cmode=np.array([x['mode'] for x in control]);mode=cmode[bi]
        cprogress=np.array([x.get('arc_progress_m',np.nan) for x in control])
        if not np.isfinite(cprogress).all():raise ValueError('Missing original controller arc progress')
        curved=(g['progress']>=path.entry)&(g['progress']<=path.entry+path.curve_length)
        mode_ok,bad_modes=continuous_modes(mode,curved,valid)
        check('continuous_curved_drive_never_stop_turn',mode_ok,
              forbidden_modes=bad_modes,physical_curved_rows=int(curved.sum()))
        ordered,visits=ordered_traversal(path,a,g)
        check('complete_original_ordered_circle_or_S_traversal',ordered,**visits)
        drive=valid&(mode=='drive')&(t>=protocol['time']['bootstrap_excluded_s'])
        if not drive.any():raise ValueError('No original translating curve drive')
        first=t[np.flatnonzero(drive)[0]]
        eligible=(drive & (t>=first+protocol['time']['speed_startup_excluded_s']) &
                  (cprogress[bi]<=path.length-.8))
        actual_forward=np.sum(a['origin_world_velocity']*g['tangent'],axis=1)
        roll=rolling_forward(t,actual_forward,eligible)
        continuous=(roll['complete_windows']>0 and roll['minimum_mean_mps']>=.7*profile['desired_speed'])
        check('every_full_one_second_actual_forward_motion',continuous,
              complete_sliding_windows=roll['complete_windows'],minimum_mean_mps=roll['minimum_mean_mps'],
              required_minimum_mps=.7*profile['desired_speed'],last_arc_exclusion_m=.8,
              startup_exclusion_s=protocol['time']['speed_startup_excluded_s'],
              mask='Original causal mode/progress, native physical samples; each complete 1s sliding window, no quiet-window selection')
        direction_ok,direction_details=turning_segments(path,a,g)
        check('all_signed_curvature_segments_actual_turn_direction',direction_ok,segments=direction_details)
        # Original actor/controller cadence defines what reference existed at
        # this physical time. Native geometric trace defines actual path error.
        cyaw=np.array([x['reference_yaw'] for x in control])
        rv=np.array([x['reference_velocity_world'] for x in control],float)
        speeds=np.linalg.norm(rv[bi,:2],axis=1)
        moving=drive&(speeds>protocol['speed']['active_reference_min_mps'])
        stable=moving&(t>=first+protocol['time']['speed_startup_excluded_s'])&(
            speeds>=profile['desired_speed']*protocol['speed']['full_speed_reference_fraction_min'])
        direction=np.zeros((len(t),2));np.divide(rv[bi,:2],speeds[:,None],out=direction,where=speeds[:,None]>1e-12)
        world_com=np.einsum('nij,nj->ni',rotations(a['quaternion_wxyz']),a['body_COM_velocity'])
        com_forward=np.sum(world_com[:,:2]*direction,axis=1)
        error_yaw=abs(np.arctan2(np.sin(a['unwrapped_yaw']-cyaw[bi]),np.cos(a['unwrapped_yaw']-cyaw[bi])))
        route_rms=float(np.sqrt(np.mean(g['nearest_xy_distance'][moving]**2))) if moving.any() else None
        route_max=float(g['nearest_xy_distance'][t>=.1].max())
        heading_max=float(error_yaw[moving].max()) if moving.any() else None
        mae=float(np.mean(abs(com_forward[stable]-speeds[stable]))) if stable.any() else None
        mean_ref=float(np.mean(speeds[stable])) if stable.any() else None
        speed_limit=max(.05,.25*mean_ref) if mean_ref is not None else None
        check('native_exact_path_and_causal_heading_original_thresholds',
              route_rms is not None and route_rms<=.08 and route_max<=.2 and heading_max<=.2,
              drive_exact_path_rms_m=route_rms,maximum_exact_path_distance_m=route_max,
              maximum_causal_drive_heading_rad=heading_max,limits={'rms_m':.08,'maximum_m':.2,'heading_rad':.2})
        check('native_real_COM_speed_original_threshold',mae is not None and mae<=speed_limit and stable.sum()*.005>=.5,
              stable_native_rows=int(stable.sum()),mean_absolute_error_mps=mae,mean_reference_mps=mean_ref,
              limit_mps=speed_limit,metric='Actual native COM world velocity projected on causal reference velocity')
        fresh=[x for x in control if x.get('controller_updated') is True]
        ff_errors=[];geometry_errors=[];budget=[];progress_errors=[]
        for c in fresh:
            arc=float(c['arc_progress_m']);point,tangent,normal,heading,kappa=path.geometry(arc)
            speed=float(c['planned_tangential_speed_mps'])
            ff=float(c['yaw_feedforward_radps'])
            ff_errors.append(abs(ff-kappa*speed));budget.append(abs(ff))
            geometry_errors.append(max(abs(c['path_curvature_1pm']-kappa),
                abs(math.atan2(math.sin(c['reference_yaw']-heading),math.cos(c['reference_yaw']-heading))),
                float(np.max(abs(np.asarray(c['reference_xy'])-point[:2])))))
            j=int(np.argmin(abs(t-c['feedback_time_s'])))
            progress_errors.append(abs(arc-g['progress'][j]))
            if c.get('path_parameters_sha256')!=path.sha256:raise ValueError('Control path hash mismatch')
        check('original_kappa_v_feedforward_geometry_and_budget',bool(fresh) and
              max(ff_errors,default=math.inf)<=1e-9 and max(geometry_errors,default=math.inf)<=1e-9 and
              max(budget,default=math.inf)<=.64+1e-9 and max(progress_errors,default=math.inf)<=.15,
              fresh_rows=len(fresh),maximum_kappa_v_error_radps=max(ff_errors,default=None),
              maximum_recorded_geometry_error=max(geometry_errors,default=None),
              maximum_abs_feedforward_radps=max(budget,default=None),feedforward_budget_radps=.64,
              maximum_controller_vs_independent_native_progress_m=max(progress_errors,default=None))
        parking=common['checks']['fixed_final_goal_parking']['parking']
        window=parking['window_s'];mask=(t>=window[0]-1e-8)&(t<=window[1]+1e-8)
        if not mask.any():raise ValueError('No original parking physical window')
        xy=float(np.linalg.norm(a['position'][mask,:2]-a['position'][mask,:2][0],axis=1).max())
        yaw=float(abs(a['unwrapped_yaw'][mask]-a['unwrapped_yaw'][mask][0]).max())
        check('original_first_five_second_parking_unchanged',mask.sum()>=1001 and xy<=.05 and yaw<=.1 and
              parking['continuous_requested_and_actor_command_zero'] is True and
              abs(xy-parking['xy_drift_m'])<=1e-8 and abs(yaw-parking['yaw_drift_rad'])<=1e-8,
              original_window_s=window,native_rows=int(mask.sum()),actual_xy_drift_m=xy,actual_yaw_drift_rad=yaw)
        metrics={'native_body_origin_source':'Original body COM twist minus omega cross archived COM offset, rotated by original quaternion',
                 'signed_turn_segments':direction_details,'actual_motion_curvature_diagnostics':motion_curvature_diagnostic(a,eligible),
                 'motion_curvature_method':'Fixed complete 1s native XY-position quadratic fits every 0.5s; diagnostic only, no S local-circle match gate',
                 'drive_exact_path_rms_m':route_rms,'speed_mae_mps':mae,'maximum_heading_error_rad':heading_max,
                 'minimum_full_one_second_forward_mps':roll['minimum_mean_mps'],'parking_xy_m':xy,'parking_yaw_rad':yaw,
                 'actual_final_progress_m':float(g['progress'][-1]),'path_length_m':path.length}
        arrays={**a,**{f'geometry_{k}':v for k,v in g.items()},'causal_control_index':bound,
                'causal_control_age_s':age,'causal_reference_valid':valid,'actual_origin_forward_mps':actual_forward,
                'actual_COM_reference_forward_mps':com_forward,'original_reference_speed_mps':speeds,
                'drive_mask':drive,'continuous_window_eligible':eligible,'heading_error_rad':error_yaw,
                'rolling_one_second_start_time_s':roll['time'],'rolling_one_second_forward_mean_mps':roll['means']}
        check('complete_parseable_curve_evidence',True)
    except Exception as error:
        check('complete_parseable_curve_evidence',None,error=repr(error),missing_or_failed_evidence_never_passes=True)
    statuses=[x['status'] for x in checks.values()]
    status='failed' if 'failed' in statuses else 'passed' if statuses and all(x=='passed' for x in statuses) else 'unverified'
    result={'schema':'independent_truth_teacher_continuous_curve_tracking/v1','run':str(run),'status':status,
            'scope':'Actual continuous circle/S on isolated simulation plane with truth feedback; not SLAM5 navigation',
            'navigation_ground_truth_used':True,'SLAM_navigation_verified':False,'real_robot_verified':False,
            'privileged_actor_observation_dimensions':232,'actor_input_dimensions':247,
            'known_command_and_previous_action_dimensions':15,'continuous_curve_verified':status=='passed',
            'score':None,'ancestor_common':ancestor,'checks':checks,'metrics':metrics,
            'verified_input_source_sha256':verified,'analyzer_sha256':sha(__file__),
            'failed_checks':[k for k,v in checks.items() if v['status']=='failed'],
            'unverified_checks':[k for k,v in checks.items() if v['status']=='unverified'],
            'original_ancestor_or_run_inputs_modified':False,
            'limitations':['No real SLAM/IMU/cloud navigation input accepted by this truth experiment.',
                           'StaticPath geometry formulas are shared with frozen archived controller; physical measurements and checks are independently reconstructed.',
                           'Body-yaw accumulated signed turn and fitted true XY motion curvature are different quantities; no instantaneous yawdot/v is called the true path curvature.',
                           'No optimum gain/frequency/minimum-radius claim from one successful curve run.']}
    if write:
        if arrays:
            with array_output.open('xb') as stream:np.savez_compressed(stream,**arrays)
            result['independent_arrays']={'path':str(array_output),'sha256':sha(array_output)}
        with output.open('x') as stream:json.dump(_serial(result),stream,ensure_ascii=False,indent=2,allow_nan=False)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();result=evaluate(args.run)
    print(json.dumps({'status':result['status'],'failed_checks':result['failed_checks'],
                      'unverified_checks':result['unverified_checks'],'receipt':str(args.run/OUTPUT)}))
