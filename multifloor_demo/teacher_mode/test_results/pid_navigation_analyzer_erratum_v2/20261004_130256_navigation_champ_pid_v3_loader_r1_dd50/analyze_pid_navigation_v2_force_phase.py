#!/usr/bin/env python3
"""Independent PID/CHAMP campaign audit. Never imports ROS or publishes commands.

Only new per-run analysis files are written. Missing evidence is not success;
source, actual motion, SLAM arrivals, contacts and parking are separate gates.
"""
from __future__ import annotations
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
TRANSPORT = {'transport_write_started_monotonic_wall','transport_written_monotonic_wall',
             'transport_queue_delay_wall_s','transport_write_duration_wall_s','transport_superseded_envelopes'}


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1<<20),b''):h.update(chunk)
    return h.hexdigest()
def read(path):return json.loads(Path(path).read_text())
def lines(path):
    if not Path(path).is_file():return []
    with Path(path).open() as f:return [json.loads(s) for s in f if s.strip()]
def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,dict):return {str(k):clean(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [clean(x) for x in v]
    if isinstance(v,float) and not math.isfinite(v):return None
    return v
def check(v,**e):return {'status':'unverified' if v is None else 'passed' if bool(v) else 'failed','passed':None if v is None else bool(v),**clean(e)}
def status(checks,names):
    s=[checks.get(n,{}).get('status','unverified') for n in names]
    return 'failed' if 'failed' in s else 'unverified' if 'unverified' in s else 'passed'
def arr(rows,key,n=None):
    a=np.asarray([r.get(key) for r in rows],dtype=float)
    shape=(len(rows),) if n is None else (len(rows),n)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError(f'Invalid finite {key} shape {a.shape}, expected {shape}')
    return a
def module(path,prefix):
    name=prefix+sha(path)[:12];s=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(s);sys.modules[name]=m;s.loader.exec_module(m);return m
def longest(mask,t,gap):
    begin=None;best=0.;interval=None
    for i,v in enumerate(mask):
        if not v:begin=None;continue
        if begin is None or i and t[i]-t[i-1]>gap:begin=float(t[i])
        if t[i]-begin>best:best=float(t[i]-begin);interval=[begin,float(t[i])]
    return best,interval
def rotation(q):
    w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
                     2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
                     2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)
def rpy(q):
    w,x,y,z=q.T
    return np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])
def wrap(a):return np.arctan2(np.sin(a),np.cos(a))
def path_error(p,vertices):
    """Distance to bounded planned XY polyline; no fitted truth route."""
    d=[]
    for a,b in zip(vertices[:-1],vertices[1:]):
        v=b[:2]-a[:2];den=float(v@v)
        if den<1e-12:continue
        u=np.clip(((p[:,:2]-a[:2])@v)/den,0,1)
        d.append(np.linalg.norm(p[:,:2]-a[:2]-u[:,None]*v,axis=1))
    if not d:raise ValueError('No nondegenerate planned route segment')
    return np.min(d,axis=0)
def causal_index(times,query,max_gap):
    idx=np.searchsorted(times,query,side='right')-1;valid=idx>=0;idx=np.clip(idx,0,len(times)-1)
    valid&=(query-times[idx]>=-1e-9)&(query-times[idx]<=max_gap+1e-9)
    return idx,valid
def source_audit(run,freeze):
    manifest=read(run/'source_manifest.json');bad=[];missing=[]
    for name,h in manifest.items():
        p=run/'sources'/name
        if not p.is_file():missing.append(name)
        elif sha(p)!=h:bad.append(name)
    refs=freeze.get('source_hashes',{});refbad=[]
    pid_manifest=read(run/'navigation_pid_source_manifest.json').get('sources',{}) if (run/'navigation_pid_source_manifest.json').is_file() else {}
    for name,h in refs.items():
        p=run/'sources'/name
        absolute=str((ROOT/name).resolve()) if not Path(name).is_absolute() else name
        if absolute in pid_manifest:
            entry=pid_manifest[absolute];p=Path(entry['snapshot'])
            if not p.is_relative_to(run) or entry.get('sha256')!=h:refbad.append(name);continue
        if not p.is_file() or sha(p)!=h:refbad.append(name)
    externalbad=[]
    for name,e in freeze.get('source_snapshots',freeze.get('external_source_snapshots',{})).items():
        p=Path(e['snapshot'])
        if not p.is_relative_to(run) or not p.is_file() or sha(p)!=e.get('sha256'):externalbad.append(name)
    return check(bool(manifest) and bool(refs) and not bad and not missing and not refbad,
                 archive_files=len(manifest),frozen_refs=len(refs),archive_mismatches=bad,
                 missing_archive_files=missing,frozen_reference_mismatches=refbad,
                 external_snapshot_mismatches=externalbad) if not externalbad else check(False,external_snapshot_mismatches=externalbad)


def command_audit(execution,history,p):
    index={r['sequence']:r for r in history if isinstance(r.get('sequence'),int)}
    hashes={s:hashlib.sha256(json.dumps({k:v for k,v in r.items() if k not in TRANSPORT},sort_keys=True,allow_nan=False).encode()).hexdigest() for s,r in index.items()}
    bad=[];accepted=0;moving=0;expired=0;matched=0;lastseq=-1;age=[];lastcmd=None;lasttime=None;slewbad=[]
    for i,r in enumerate(execution):
        env=r.get('navigation_envelope') or {};out=env.get('read_status');req=np.asarray(r.get('requested'),float);cmd=np.asarray(r.get('command'),float)
        now=r.get('world_sim_time');wall=env.get('read_monotonic_wall')
        if req.shape!=(3,) or cmd.shape!=(3,) or not np.isfinite(req).all() or not np.isfinite(cmd).all() or not isinstance(now,(int,float)):
            bad.append({'frame':i,'reason':'Malformed actual execution command'});continue
        if r.get('command_expired') is True:
            expired+=1
            if np.max(abs(req))>1e-9:bad.append({'frame':i,'reason':'Expired source requested motion'})
        if lastcmd is not None:
            dt=now-lasttime
            if dt<=0:bad.append({'frame':i,'reason':'Execution command time not increasing'})
            else:
                limit=np.asarray(p['transport']['worker_slew_acceleration'])*min(dt,.02)
                expected=lastcmd+np.clip(req-lastcmd,-limit,limit)
                if np.max(abs(cmd-expected))>1e-8:slewbad.append(i)
        lastcmd=cmd;lasttime=now
        if out not in ['accepted','valid_but_unhealthy_or_stale']:continue
        seq=env.get('sequence');original=index.get(seq)
        if original is None or hashes.get(seq)!=env.get('hash'):
            bad.append({'frame':i,'reason':'Original consumer envelope missing or hash differs'});continue
        matched+=1
        if seq<lastseq:bad.append({'frame':i,'reason':'Consumer sequence moved backward'})
        lastseq=seq;sa=now-original['sim_time'];wa=env.get('wall_age_s')
        if not isinstance(wall,(int,float)) or not isinstance(wa,(int,float)) or abs(sa-env.get('sim_age_s',math.inf))>1e-8 or wa<wall-original['monotonic_wall']-1e-8:
            bad.append({'frame':i,'reason':'Causal consumer age evidence inconsistent'});continue
        if out=='accepted':
            accepted+=1;age.append([sa,wa])
            good=(p['transport']['minimum_source_age_s']<=sa<p['transport']['maximum_source_sim_age_s'] and
                  p['transport']['minimum_source_age_s']<=wa<p['transport']['maximum_source_wall_age_s'] and original.get('healthy') is True)
            if not good:bad.append({'frame':i,'reason':'Accepted nonfresh or unhealthy source'})
            if np.linalg.norm(req[:2])>.02:moving+=1
            local=r.get('actual_execution_health',{})
            protected_zero=local.get('ready') is False and np.max(abs(req))<1e-9
            if not protected_zero and not np.allclose(req,original['command'],atol=1e-9,rtol=0):bad.append({'frame':i,'reason':'Executed requested command differs from accepted original envelope'})
    return check(not bad and not slewbad and matched>0 and accepted>0 and moving>=25 if execution and history else None,
                 lineage_errors=bad,slew_error_frames=slewbad,actual_matched_reads=matched,
                 accepted_reads=accepted,accepted_forward_frames=moving,expired_or_unhealthy_frames=expired,
                 accepted_age_max_s=np.max(age,axis=0) if age else None,
                 limits=p['transport'],physical_TTL_stop_claim='unverified without a full deliberate dropout/stop event')


def arrival_audit(slam,request,status_rows,p,case):
    stamps=np.asarray([r.get('stamp_ns') for r in slam],dtype=np.int64);pos=arr(slam,'position',3)
    matching=[r for r in status_rows if r.get('request_id')==request.get('request_id')]
    latest=matching[-1] if matching else {};claimed=latest.get('region_arrivals',[]);out=[];last_end=None
    for j,goal in enumerate(request.get('goals',[])):
        receipt=next((r for r in claimed if r.get('goal_id')==goal.get('goal_id')),None)
        detail={'goal_id':goal.get('goal_id'),'claimed_receipt':receipt,'status':'failed' if matching else 'unverified'}
        if receipt:
            begin=receipt.get('start_stamp_ns');end=receipt.get('stamp_ns');exact=isinstance(begin,int) and isinstance(end,int)
            mask=(stamps>=begin)&(stamps<=end) if exact else np.zeros(len(stamps),bool)
            exact=bool(exact and mask.sum()>1 and stamps[mask][0]==begin and stamps[mask][-1]==end)
            gap=float(np.max(np.diff(stamps[mask]))/1e9) if mask.sum()>1 else None
            dwell=float((end-begin)/1e9) if exact else 0.;delta=pos[mask]-np.asarray(goal['center'])
            inside=(np.linalg.norm(delta[:,:2],axis=1)<=p['arrival']['control_radius_m']+1e-12)&(abs(delta[:,2])<=p['arrival']['height_half_span_m']+1e-12)
            activation=receipt.get('goal_activated_ros_clock_ns') or receipt.get('goal_activated_stamp_ns')
            if activation is None:
                active=[r for r in matching if r.get('state')=='running' and r.get('waypoint_index')==j]
                activation=int(round(active[0]['ros_sim_time']*1e9)) if active else None
                deadline_source='first actual running status snapshot; timing uncertainty <= status period'
            else:deadline_source='actual goal activation integer timestamp'
            elapsed=(end-activation)/1e9 if activation is not None and exact else None
            good=bool(exact and inside.all() and dwell>=p['arrival']['dwell_s']-1e-9 and gap<=p['arrival']['maximum_raw_pose_gap_s']+1e-9 and receipt.get('protected') is False and receipt.get('control_region_inside') is True and elapsed is not None and 0<=elapsed<=case['goal_deadline_s']+1e-9 and (last_end is None or begin>last_end))
            detail.update(check(good,raw_samples=int(mask.sum()),exact_raw_begin_end=exact,
                inside_original_control_region=bool(inside.all()),dwell_s=dwell,maximum_raw_pose_gap_s=gap,
                goal_elapsed_s=elapsed,goal_deadline_s=case['goal_deadline_s'],activation_source=deadline_source))
            last_end=end
        out.append(detail)
    return check(len(out)==len(request.get('goals',[])) and bool(out) and all(r['status']=='passed' for r in out) and latest.get('state')=='succeeded',
                 regions=out,actual_last_state=latest.get('state'),actual_last_waypoint=latest.get('waypoint_index')),out


def pid_terms_audit(records,profile,pose_index):
    cfg=profile.get('pid',{});bad=[];updated=0;maximum=0.
    if not cfg:return check(None,reason='Frozen PID coefficients absent')
    kp=np.array([cfg['position_kp'],cfg['position_kp'],cfg['yaw_kp']]);ki=np.array([cfg['position_ki'],cfg['position_ki'],cfg['yaw_ki']]);kd=np.array([cfg['position_kd'],cfg['position_kd'],cfg['yaw_kd']]);limits=np.array([profile['max_speed_mps'],profile['max_lateral_speed_mps'],profile['max_yaw_rate_radps']])
    for r in records:
        diag=r.get('pid',{});seq=r.get('sequence');source=pose_index.get(r.get('control_pose_stamp_ns'))
        if diag.get('updated') is not True or r.get('mode') not in ['drive','turn']:continue
        updated+=1
        try:
            yaw=float(r['yaw']);c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]]);pose=np.asarray(r['control_pose']);target=np.asarray(r['checked_target']);error=np.r_[target[:2]-pose[:2],wrap(r['heading']-yaw)]
            if r['mode']=='turn':error[:2]=0
            integral=np.asarray(diag['integral']);filtered=np.asarray(diag['filtered_measured_world_vxy_body_wz']);parts=[kp*error,ki*integral,-kd*filtered]
            if r['mode']=='turn':
                for v in parts:v[:2]=0
            raw=np.r_[R.T@sum(parts)[:2],sum(parts)[2]];clipped=np.clip(raw,-limits,limits)
            if r['mode']=='drive':clipped[0]=max(0.,clipped[0])
            if np.linalg.norm(clipped[:2])>limits[0]:clipped[:2]*=limits[0]/np.linalg.norm(clipped[:2])
            observed=[diag['P_world_xy_yaw'],diag['I_world_xy_yaw'],diag['D_world_xy_yaw'],diag['raw_body_command'],diag['clipped_body_command'],r['desired_body_command']]
            expected=[*parts,raw,clipped,clipped];errs=[float(np.max(abs(np.asarray(a)-b))) for a,b in zip(observed,expected)]
            qr=np.asarray(source['quaternion'])[[3,0,1,2]];srcR=rotation(qr[None,:])[0];velocity=srcR@np.asarray(source['body_velocity']);errs.append(float(np.max(abs(np.asarray(r['measured_world_velocity'])-velocity))))
            errs.append(float(np.max(abs(np.asarray(r['control_rotation'])-srcR))));maximum=max(maximum,max(errs));bounds=np.array([cfg['position_integral_limit']]*2+[cfg['yaw_integral_limit']]);dt=diag['header_dt_s'];age=(r['compute_ros_clock_ns']-r['control_pose_stamp_ns'])/1e9
            if max(errs)>1e-8 or np.any(abs(integral)>bounds+1e-10) or not 0<=dt<=cfg['header_gap_max_s']+1e-9 or not -.05<=age<.3:bad.append(seq)
        except (KeyError,ValueError,TypeError):bad.append(seq)
    return check(updated>0 and not bad if records else None,actual_updated_drive_or_turn_ticks=updated,term_replay_maximum_error=maximum,invalid_sequences=bad,integral_and_header_gap_limits=cfg)


def evaluate(run,write=True):
    run=Path(run).resolve();checks={};metrics={};errors=[];inputs={};arrays={};controller=None;case={};p={}
    try:
        for name in ['pid_navigation_protocol.json','pid_navigation_case.json','pid_navigation_freeze.json','navigation_scope.json','navigation_profile.json','navigation_anchor.json','navigation_request.json','runtime_manifest.json','source_manifest.json','world.sdf']:
            if not (run/name).is_file():raise FileNotFoundError('Missing actual campaign input '+name)
        p=read(run/'pid_navigation_protocol.json');case=read(run/'pid_navigation_case.json');freeze=read(run/'pid_navigation_freeze.json');scope=read(run/'navigation_scope.json');profile=read(run/'navigation_profile.json');anchor=read(run/'navigation_anchor.json');request=read(run/'navigation_request.json');runtime=read(run/'runtime_manifest.json')
        controller=scope.get('controller',scope.get('locomotion_controller',scope.get('controller_kind')));controller='teacher_pid' if controller=='teacher' else 'champ' if controller=='champ_pid' else controller;checks['prospective_case_and_scope']=check(p.get('schema')=='teacher_champ_pid_navigation_protocol/v1' and controller in p['controllers'] and scope.get('protocol_case_id')==case['case_id'] and scope.get('navigation_ground_truth_used') is False and not scope.get('acceptance_is_global',False),controller=controller,case_id=case.get('case_id'),experiment=scope.get('experiment'),truth_navigation=scope.get('navigation_ground_truth_used'))
        checks['source_provenance']=source_audit(run,freeze)
        refproto=run/'sources/tests/pid_navigation/protocol.json';refcases=run/'sources/tests/pid_navigation/cases.json'
        limits_exact=(profile.get('goal_timeout_sim_s')==case['goal_deadline_s'] and profile.get('max_speed_mps')==case['nominal_body_forward_mps'] and profile.get('max_lateral_speed_mps')==case['maximum_body_lateral_mps'] and profile.get('max_yaw_rate_radps')==case['maximum_yaw_rate_radps'])
        checks['frozen_protocol_and_case']=check(refproto.is_file() and refcases.is_file() and sha(refproto)==sha(run/'pid_navigation_protocol.json') and read(refcases).get(case['case_id'])==case and limits_exact,protocol_sha256=sha(run/'pid_navigation_protocol.json'),case_sha256=sha(run/'pid_navigation_case.json'),actual_profile_limits_match_case=limits_exact)
        slam=lines(run/'navigation_slam_poses.jsonl');nav=lines(run/'navigation_status.jsonl');pid=lines(run/'navigation_pid_history.jsonl');health=lines(run/'navigation_sensor_gate_history.jsonl');commands=lines(run/'navigation_command_history.jsonl');cloud=lines(run/'navigation_cloud_history.jsonl');guardfile='navigation_native_guard_trace.jsonl' if (run/'navigation_native_guard_trace.jsonl').is_file() else 'navigation_guard_history.jsonl';guard=lines(run/guardfile)
        if not slam:raise ValueError('No accepted raw SLAM stream; cannot validate a route')
        stamps=np.asarray([r.get('stamp_ns') for r in slam],dtype=np.int64);sp=arr(slam,'position',3);sq=arr(slam,'quaternion',4)
        checks['raw_sensor_slam_stream']=check(np.all(np.diff(stamps)>0) and all(r.get('frame_id')==p['localization']['frame_id'] and r.get('child_frame_id')=='demo_slam_body' for r in slam) and np.allclose(np.sum(sq*sq,1),1,atol=.02),samples=len(slam),maximum_pose_gap_s=np.max(np.diff(stamps))/1e9 if len(stamps)>1 else None)
        ready=[r for r in health if r.get('ready') is True];chains=all(r.get('ground_truth_navigation_used') is False and r.get('overview_used_for_vio') is False and r.get('publisher_graph') and all(v.get('passed') is True for v in r['publisher_graph'].values()) for r in ready)
        checks['actual_sensor_publisher_chain']=check(bool(ready) and chains if health else None,ready_samples=len(ready),truth_navigation_used=False if chains and ready else None)
        ai=np.flatnonzero(stamps==anchor.get('pose_stamp_ns'));origin=np.asarray(anchor.get('origin'),float);yaw=anchor.get('yaw');exact=bool(len(ai)==1 and origin.shape==(3,) and np.allclose(sp[ai[0]],origin,rtol=0,atol=1e-10))
        if not exact or not isinstance(yaw,(int,float)):raise ValueError('Immutable anchor does not exactly match raw SLAM')
        sr=rotation(sq[ai[0]][[3,0,1,2]][None,:])[0];anchoryaw=math.atan2(sr[1,0],sr[0,0]);rr=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
        desired=origin+np.asarray(case['relative_waypoints_m'])@rr.T;goals=request.get('goals',[]);goal_ok=len(goals)==len(desired) and all(np.allclose(g['center'],d,rtol=0,atol=1e-10) for g,d in zip(goals,desired))
        arrival_ok=all(g.get('arrival',{}).get('control_band',{}).get('radius_m')==p['arrival']['control_radius_m'] and g.get('arrival',{}).get('height_half_span_m')==p['arrival']['height_half_span_m'] and g.get('arrival',{}).get('dwell_sim_s')==p['arrival']['dwell_s'] and g.get('timeout_sim_s')==case['goal_deadline_s'] for g in goals)
        checks['immutable_raw_slam_route']=check(exact and abs(wrap(yaw-anchoryaw))<1e-10 and goal_ok and arrival_ok and anchor.get('source')==p['localization']['source'] and anchor.get('frozen_once') is True and anchor.get('ground_truth_navigation_used') is False and request.get('frame_id')==p['localization']['frame_id'],anchor_stamp_ns=int(stamps[ai[0]]),relative_waypoints=case['relative_waypoints_m'],actual_goal_centers=[g.get('center') for g in goals],arrival_contract_exact=arrival_ok)
        checks['all_raw_slam_regions'],arrivals=arrival_audit(slam,request,nav,p,case);metrics['arrivals']=arrivals
        own=runtime.get('owned_processes',[]);roles=[r.get('role') for r in own];child_paths=sorted(set(list(run.glob('*navigation*stack*.log'))+list(run.glob('*navstack*.log'))+list(run.glob('*champ*stack*.log'))))
        child_errors=[];cleanchildren=0
        for path in child_paths:
            for line in path.read_text(errors='replace').splitlines():
                if 'process has finished cleanly' in line:cleanchildren+=1
                if 'process has died' in line or 'failed to terminate' in line or 'escalating to SIGKILL' in line:child_errors.append(line)
        required=freeze.get('required_owned_roles',[]);expectedchildren=freeze.get('expected_launch_children')
        owner_complete=bool(own and required and set(required)==set(roles) and len(roles)==len(set(roles)))
        checks['complete_runtime_and_clean_exit']=check(owner_complete and all(r.get('returncode')==0 for r in own) and runtime.get('error') is None and bool(child_paths) and expectedchildren is not None and cleanchildren==expectedchildren and not child_errors,owned=own,required_owned_roles=required,launch_clean_children=cleanchildren,expected_launch_children=expectedchildren,child_errors=child_errors)
        if controller=='teacher_pid':
            raw=lines(run/'actuator.jsonl');meta=read(run/'policy_manifest.json');result=read(run/'worker_result.json');execution=lines(run/'telemetry.jsonl');contracts=[r for r in raw if r.get('kind')=='actuator_contract'];contract=contracts[0] if len(contracts)==1 else {};offset=-.005
            obsfile=run/'observations_actions.npz'
            with np.load(obsfile) as saved:obs=saved['observations'];actions=saved['actions']
            acts=arr(execution,'action',12);target=arr(execution,'q_target',12);et=arr(execution,'sim_time');enabled=et>=.1;err=float(np.max(abs(target[enabled]-(np.asarray(contract.get('initial_q'))+.25*acts[enabled]))))
            spec=read(run/'sources/policy/contract.json')['observations']['concatenation_order'][3];cmdinput=np.clip(arr(execution,'command',3),*spec['clip'])*spec['scale_after_clip'];cmdinputerror=float(np.max(abs(obs[:,9:12]-cmdinput.astype(np.float32))))
            checks['controller_identity_and_execution']=check(len(contracts)==1 and meta.get('checkpoint_sha256')==p['checkpoint_sha256'] and meta.get('inference_device')=='cpu' and meta.get('torch_threads')==1 and contract.get('body_pose_resets')==0 and contract.get('writer') in ['teacher_sim::TeacherActuator','teacher_sim::TeacherActuator sole JointForceCmd writer'] and obs.shape==(len(execution),247) and actions.shape==(len(execution),12) and np.isfinite(obs).all() and np.isfinite(actions).all() and np.array_equal(actions,acts) and err<1e-6 and cmdinputerror<1e-7 and np.all(np.diff(et)>0) and np.max(np.diff(et))<=.020001 and all(r.get('actor_inferred_this_frame') is True for r in execution if r.get('sim_time',0)>=.1) and result.get('fault') is None and result.get('completed') is True and meta.get('physical_test_completion_source') is None,controller=controller,actor_device=meta.get('inference_device'),torch_threads=meta.get('torch_threads'),samples=len(execution),target_action_error_rad=err,actor_command_observation_error=cmdinputerror,privileged_actor_inputs=meta.get('observation_source'),worker_fault=result.get('fault'),truth_navigation_completion_gate_present=meta.get('physical_test_completion_source'))
        elif controller=='champ':
            raw=lines(run/'actuator.jsonl');meta=read(run/'champ_contract.json');execution=lines(run/'telemetry.jsonl');monitor=read(run/'policy_metadata.json');ownership=[r for r in raw if r.get('kind')=='model_plugin_ownership'];contracts=[r for r in raw if r.get('kind') in ['champ_observer_contract','actuator_contract']];contract=contracts[0] if len(contracts)==1 else {};offset=contract.get('state_time_offset_s')
            if offset is None:raise ValueError('CHAMP native observer must explicitly declare PostUpdate state_time_offset_s')
            checks['controller_identity_and_execution']=check(len(contracts)==1 and meta.get('controller_kind')=='champ' and meta.get('body_servo') is False and meta.get('body_stabilizer') is False and len(ownership)==1 and ownership[0].get('passed') is True and ownership[0].get('ros2_control_writers')==1 and contract.get('body_pose_resets')==0 and monitor.get('completed') is True and monitor.get('fault') is None and monitor.get('actor_started') is False,controller=controller,manifest=meta,ownership=ownership,actual_phase_offset_s=offset,monitor=monitor)
        else:raise ValueError('Controller identity unavailable; cannot assign a baseline result')
        steps=[r for r in raw if r.get('kind')=='physics_step']
        if not steps:raise ValueError('No actual native physical steps')
        nt=arr(steps,'t')+offset;pos=arr(steps,'position',3);q=arr(steps,'quaternion_wxyz',4);rot=rotation(q);angles=rpy(q);vel=arr(steps,'body_lin_vel_com',3);wv=np.einsum('nij,nj->ni',rot,vel);omega=arr(steps,'body_ang_vel',3)
        qj=arr(steps,'q',12);qd=arr(steps,'qd',12);tau=arr(steps,'tau',12);counts=arr(steps,'contacts',5)
        gap=p['physical']['native_sample_gap_max_s'];goodtime=bool(np.all(np.diff(nt)>0) and np.max(np.diff(nt))<=gap+1e-10 and np.max(abs(np.sum(q*q,1)-1))<.02)
        originvel=arr(steps,'body_lin_vel_origin',3) if all('body_lin_vel_origin' in r for r in steps) else None
        if originvel is not None:
            worldorigin=np.einsum('nij,nj->ni',rot,originvel);integral_error=float(np.max(np.linalg.norm(np.diff(pos,axis=0)-worldorigin[1:]*np.diff(nt)[:,None],axis=1)))
        else:integral_error=None
        checks['continuous_native_200hz']=check(goodtime and integral_error is not None and integral_error<=p['physical']['pose_translation_integral_error_m'],native_samples=len(nt),state_time_offset_s=offset,maximum_gap_s=np.max(np.diff(nt)),start_world_time_s=nt[0],end_world_time_s=nt[-1],pose_increment_velocity_integral_error_m=integral_error)
        checks['causal_command_TTL_and_slew']=command_audit(execution,commands,p)
        world=ET.parse(run/'world.sdf').getroot();robot=world.find("world/model[@name='go2']")
        if robot is None:raise ValueError('Actual robot SDF model missing')
        actuator=[v.get('name') for v in robot.findall('plugin') if any(s in v.get('name','').lower() for s in ['teacheractuator','control','champ','stabilizer','trajectory']) and v.get('name')!='champ_compare::NativeObserver']
        expected_writers=1 if controller=='teacher_pid' else 1 if meta.get('exclusive_effort_writer')=='gz_ros2_control::GazeboSimROS2ControlPlugin' else None
        checks['exclusive_actuator_and_no_body_servo']=check(expected_writers is not None and len(actuator)==expected_writers and not any('stabilizer' in s.lower() for s in actuator) and contract.get('body_pose_resets')==0 and (controller!='champ' or meta.get('observer_writes_joint_force') is False),writer_plugins=actuator,expected=expected_writers,body_pose_resets=contract.get('body_pose_resets'))
        helperpath=run/'sources/scripts/evaluate_step_functional.py';ramppath=run/'sources/scripts/analyze_full_ramp.py';obspath=run/'sources/policy/observation.py'
        helper=module(helperpath,'pid_foot_');ramphelper=module(ramppath,'pid_ramp_');observation=module(obspath,'pid_height_')
        terrain=observation.TerrainHeightMap.from_sdf(run/'world.sdf');ground=terrain.height(pos[:,:2],ray_start_z=pos[:,2]);clearance=pos[:,2]-ground;audit=nt>=p['physical']['audit_start_s'];faults=[r for r in steps if r.get('fault',0)>0]
        safe=bool(audit.any() and np.isfinite(clearance[audit]).all() and np.min(clearance[audit])>=p['physical']['minimum_support_clearance_m'] and np.max(abs(angles[audit,:2]))<=p['physical']['maximum_abs_roll_pitch_rad'] and np.max(counts[audit,0])==0 and np.min(counts[audit])>=0 and not faults)
        checks['physical_safety_and_clearance']=check(safe,min_support_clearance_m=np.min(clearance[audit]),max_roll_rad=np.max(abs(angles[audit,0])),max_pitch_rad=np.max(abs(angles[audit,1])),body_contact_samples=int(np.sum(counts[audit,0]>0)),native_fault_samples=len(faults),world_height_is_ability_gate=False)
        unsupported,unsupported_interval=longest((np.sum(counts[:,1:],axis=1)==0)&audit,nt,gap)
        checks['continuous_actual_foot_contact']=check(unsupported<=p['contact']['maximum_all_feet_unsupported_s']+1e-9,maximum_all_feet_unsupported_s=unsupported,interval_s=unsupported_interval,foot_support_is_not_four_simultaneously=True)
        if controller=='teacher_pid':
            targetnative=arr(steps,'qtarget',12);lo=np.maximum(23.5*(-1-qd/30),-23.5);hi=np.minimum(23.5*(1-qd/30),23.5);pd=25*(targetnative-qj)-.5*qd;active=np.array([r.get('mode')==0 and not r.get('terminating') for r in steps]);torque_error=float(abs(tau[active]-np.clip(pd[active],lo[active],hi[active])).max())
            checks['actual_joint_execution']=check(contract.get('kp')==25 and contract.get('kd')==.5 and contract.get('effort_limit')==23.5 and contract.get('velocity_limit')==30 and torque_error<1e-8,reconstructed_torque_error_Nm=torque_error,maximum_tau_Nm=np.max(abs(tau)),outside_target_steps=sum(any(r.get('target_outside_hard_limits',[])) for r in steps),target_outside_limits_is_not_an_adapter_fault=True)
        else:
            targetevents=lines(run/'champ/joint_target_events.jsonl');available=arr(steps,'tau_available',12)
            checks['actual_joint_execution']=check(contract.get('torque_source') is not None and meta.get('pid') is not None and bool(targetevents) and np.all(available[audit]==1),torque_source=contract.get('torque_source'),controller_PD=meta.get('pid'),maximum_tau_Nm=np.max(abs(tau)),force_components_missing_after_bootstrap=int((available[audit]!=1).sum()),original_target_events=len(targetevents),original_target_zero_header_stamps=sum(r.get('original_header_stamp_ns')==0 for r in targetevents),requested_targets_source='Original ROS targets and real receive clocks, independent from native q/JointForceCmd; zero source headers are never repaired')
        checks['native_force_and_joint_velocity_limits']=check(np.max(abs(tau[audit]))<=p['physical']['maximum_actual_joint_forcecmd_abs_Nm'] and np.max(abs(qd[audit]))<=p['physical']['maximum_actual_joint_velocity_radps'],maximum_actual_forcecmd_abs_Nm=np.max(abs(tau[audit])),maximum_actual_joint_velocity_radps=np.max(abs(qd[audit])),force_limit_Nm=p['physical']['maximum_actual_joint_forcecmd_abs_Nm'],velocity_limit_radps=p['physical']['maximum_actual_joint_velocity_radps'],force_is_measured_motor_torque=False)
        if controller=='champ':
            # The frozen CHAMP observer reads in PostUpdate. Gazebo Physics
            # zeroes JointForceCmd after SetForce during Update, before this
            # observer. Component existence does not establish applied force.
            # Keep the physical q/qd/target evidence, but do not grant the
            # required 200Hz force gate from this emptied command buffer.
            old_execution=checks['actual_joint_execution']
            old_limits=checks['native_force_and_joint_velocity_limits']
            phase_reason='CHAMP PostUpdate JointForceCmd is the buffer after Gazebo Physics Update clearing; no complete after-controller-before-physics force samples. Presence flags and all-zero values cannot prove applied-command limits.'
            checks['actual_joint_execution']=check(None,reason=phase_reason,original_joint_target_events=old_execution.get('original_target_events'),joint_position_velocity_evidence='Actual PostUpdate q and qd remain available',postupdate_buffer_max_abs_Nm=np.max(abs(tau)))
            checks['native_force_and_joint_velocity_limits']=check(None,reason=phase_reason,postupdate_buffer_max_abs_Nm=np.max(abs(tau[audit])),maximum_actual_joint_velocity_radps=np.max(abs(qd[audit])),joint_velocity_bound_passed=bool(np.max(abs(qd[audit]))<=p['physical']['maximum_actual_joint_velocity_radps']),force_limit_Nm=p['physical']['maximum_actual_joint_forcecmd_abs_Nm'],velocity_limit_radps=p['physical']['maximum_actual_joint_velocity_radps'],force_is_measured_motor_torque=False)
        checks['actual_registered_cloud_inputs']=check(bool(cloud) and all(r.get('frame_id')==p['localization']['frame_id'] and isinstance(r.get('stamp_ns'),int) and isinstance(r.get('nearest_slam_pose_stamp_ns'),int) and abs(r['stamp_ns']-r['nearest_slam_pose_stamp_ns'])<=150000001 and r.get('filtered_points',0)>0 and len(r.get('filtered_xyz_float64_sha256',''))==64 for r in cloud),accepted_clouds=len(cloud),raw_geometry_replay='unverified where original XYZ is not archived')
        trajectory_files=sorted((run/'navigation_trajectories').glob('*.json'));trajectory_bad=[];trajectory_ids=set()
        for path in trajectory_files:
            r=read(path);a=path.parent/r.get('array_file','');trajectory_ids.add(r.get('trajectory_id'))
            if not a.is_file() or sha(a)!=r.get('array_sha256'):trajectory_bad.append(path.name);continue
            with np.load(a) as saved:c=saved['coefficients'];knots=saved['knots'];samples=saved['samples']
            if r.get('order')!=3 or c.ndim!=2 or c.shape[1]!=3 or len(knots)!=len(c)+4 or samples.ndim!=2 or samples.shape[1]!=3 or not np.isfinite(c).all() or not np.isfinite(knots).all() or not np.isfinite(samples).all():trajectory_bad.append(path.name);continue
            from scipy.interpolate import BSpline
            if np.max(abs(BSpline(knots,c,3)(np.linspace(knots[3],knots[-4],len(samples)))-samples))>1e-9:trajectory_bad.append(path.name)
        accepted={r['accepted_trajectory_id'] for r in nav if r.get('accepted_trajectory_id') is not None}
        checks['actual_checked_SCAN_trajectories']=check(bool(trajectory_files) and bool(accepted) and accepted.issubset(trajectory_ids) and not trajectory_bad,actual_archives=len(trajectory_files),accepted_ids=sorted(accepted),payload_errors=trajectory_bad,full_cloud_collision_replay='unverified without complete original XYZ')
        if pid:
            pt=np.asarray([r.get('compute_ros_clock_ns') for r in pid],dtype=np.int64)/1e9;ps=np.asarray([r.get('control_pose_stamp_ns') for r in pid]);seq=np.asarray([r.get('sequence') for r in pid]);pi={int(r['stamp_ns']):r for r in slam};bad=[]
            for r in pid:
                rawpose=pi.get(r.get('control_pose_stamp_ns'))
                pose=r.get('pose',r.get('control_pose'));frame=r.get('frame_id',r.get('control_frame_id'))
                # The exact header lookup binds the frame if the observer did
                # not duplicate it in its row; never bind by a nearby float.
                frame=rawpose.get('frame_id') if frame is None and rawpose is not None else frame
                if rawpose is None or pose is None or not np.allclose(pose,rawpose['position'],rtol=0,atol=1e-10) or frame!=p['localization']['frame_id'] or r.get('navigation_ground_truth_used') is not False:bad.append(r.get('sequence'))
            checks['PID_measured_source_and_execution']=check(np.all(np.diff(seq)>0) and np.all(np.diff(pt)>=0) and len(pid)>=25 and not bad,actual_ticks=len(pid),invalid_raw_slam_source_sequences=bad,PID_is_outer_velocity_controller=True)
            checks['PID_actual_terms_and_bounds']=pid_terms_audit(pid,profile,pi)
            receipt=read(run/'navigation_pid_writer_receipt.json') if (run/'navigation_pid_writer_receipt.json').is_file() else {}
            expected=receipt.get('expected_pid_records');expected_guard=receipt.get('expected_guard_records');guardseq=[r.get('sequence') for r in guard]
            checks['complete_PID_and_guard_recording']=check(receipt.get('status')=='drained' and not receipt.get('queue_error') and expected==len(pid) and expected_guard==len(guard) and list(seq)==list(range(1,len(pid)+1)) and guardseq==list(range(1,len(guard)+1)),actual_pid_records=len(pid),actual_guard_records=len(guard),writer_receipt=receipt)
            ngi,valid=causal_index(pt,nt,.3);ep_t=arr(execution,'world_sim_time');ep_cmd=arr(execution,'command',3);ei,ev=causal_index(ep_t,nt,.020001);drive=valid&ev&(np.linalg.norm(ep_cmd[ei,:2],axis=1)>.02)
            spawn=np.asarray(case['spawn']);yaw0=spawn[3];planned=np.asarray(case['relative_waypoints_m']);rz=np.array([[math.cos(yaw0),-math.sin(yaw0),0],[math.sin(yaw0),math.cos(yaw0),0],[0,0,1]])
            vertices=np.vstack([spawn[:3],spawn[:3]+planned@rz.T]);lateral=path_error(pos,vertices);head=np.zeros(len(nt));direction=np.zeros((len(nt),2));wp=np.asarray([pid[i].get('waypoint_index',-1) for i in ngi],int);validwp=(wp>=0)&(wp<len(vertices)-1);drive&=validwp & np.asarray([pid[i].get('mode')=='drive' for i in ngi], dtype=bool)
            for j in range(len(vertices)-1):
                d=vertices[j+1,:2]-vertices[j,:2];dn=d/np.linalg.norm(d);mask=wp==j;direction[mask]=dn;head[mask]=abs(wrap(angles[mask,2]-math.atan2(d[1],d[0])))
            movingprogress=float(np.max(np.linalg.norm(pos[:,:2]-pos[0,:2],axis=1)));lateralrms=float(np.sqrt(np.mean(lateral[drive]**2))) if drive.any() else None;maximum=float(np.max(lateral[audit]));maxheading=float(np.max(head[drive])) if drive.any() else None
            checks['physical_route_tracking_and_actual_motion']=check(bool(drive.any()) and movingprogress>=p['route']['minimum_measured_motion_m'] and maximum<=p['route']['maximum_lateral_error_m'] and lateralrms<=p['route']['maximum_active_lateral_rms_m'] and maxheading<=p['route']['maximum_drive_heading_error_rad'],world_planned_polyline=vertices,maximum_lateral_error_m=maximum,active_lateral_RMS_m=lateralrms,maximum_drive_heading_error_rad=maxheading,maximum_actual_XY_excursion_m=movingprogress,drive_native_samples=int(drive.sum()),truth_used_only_offline=True)
            projected=np.einsum('ij,ij->i',wv[:,:2],direction);rolling=[]
            for i in np.flatnonzero(drive):
                j=np.searchsorted(nt,nt[i]-p['route']['rolling_window_s']);mask=slice(j,i+1)
                if nt[i]-nt[j]>=p['route']['rolling_window_s']-gap and drive[mask].all() and (wp[mask]==wp[i]).all():rolling.append(float(projected[mask].mean()))
            fraction=float(np.mean(np.asarray(rolling)>p['route']['minimum_projected_world_forward_window_mean_mps'])) if rolling else None
            checks['forward_motion_during_drive']=check(fraction is not None and fraction>=p['route']['minimum_positive_window_fraction'],rolling_windows=len(rolling),positive_window_fraction=fraction,window_s=p['route']['rolling_window_s'],minimum_world_mean_mps=p['route']['minimum_projected_world_forward_window_mean_mps'])
            metrics.update(route=checks['physical_route_tracking_and_actual_motion'],PID_ticks=len(pid),truth_state_time_offset_s=offset)
            arrays.update(native_world_time_s=nt,native_position=pos,native_world_COM_velocity=wv,native_quaternion_wxyz=q,native_q=qj,native_qd=qd,native_applied_torque=tau,native_clearance=clearance,planned_world_polyline=vertices,lateral_error_m=lateral,drive_heading_error_rad=head,drive_mask=drive,SLAM_stamp_ns=stamps,SLAM_position=sp,execution_world_time_s=ep_t,execution_command=ep_cmd)
        else:
            checks['PID_measured_source_and_execution']=check(None,reason='No actual PID tick history')
            checks['PID_actual_terms_and_bounds']=check(None,reason='No actual PID tick history')
            checks['complete_PID_and_guard_recording']=check(None,reason='No actual PID tick history')
            checks['physical_route_tracking_and_actual_motion']=check(None,reason='No causal active route/command mapping')
            checks['forward_motion_during_drive']=check(None,reason='No observed drive intervals')
        pi={r.get('sequence'):r for r in pid};guardbad=[]
        for r in guard:
            original=pi.get(r.get('pid_sequence'))
            if original is not None and (r.get('pid_compute_ros_clock_ns')!=original.get('compute_ros_clock_ns') or r.get('pid_control_pose_stamp_ns')!=original.get('control_pose_stamp_ns') or r.get('control_pose_stamp_ns')!=original.get('control_pose_stamp_ns')):
                original=None
            desired_cmd=original.get('guard_world_direction',original.get('actual_pid_world_direction')) if original else None
            if desired_cmd is not None and np.linalg.norm(desired_cmd)<1e-9:
                # Zero motion retains the original checked route direction.
                desired_cmd=original.get('steering_direction')
            if desired_cmd is None or not np.allclose(desired_cmd,r.get('steering_direction'),rtol=0,atol=1e-10):guardbad.append(r.get('sequence'))
        checks['actual_PID_motion_corridor_guard']=check(not guardbad if guard else None,actual_guard_calls=len(guard),missing_or_different_PID_direction_sequences=guardbad,scope='Native checked goal corridor union actual PID movement direction; no sideways bypass')
        success=[r for r in nav if r.get('state')=='succeeded' and r.get('request_id')==request.get('request_id')]
        if success and arrivals and arrivals[-1].get('claimed_receipt'):
            # The fixed window follows first actually recorded success clock;
            # the distinct raw-SLAM arrival stamp is audited above.
            first_success=success[0]['ros_sim_time'];start=first_success+p['parking']['settling_s'];end=start+p['parking']['evaluation_s'];mask=(nt>=start-1e-10)&(nt<=end+1e-10);etimes=arr(execution,'world_sim_time');em=(etimes>=start-1e-10)&(etimes<=end+1e-10)
            complete=bool(mask.sum()>1 and nt[mask][-1]-nt[mask][0]>=p['parking']['evaluation_s']-gap and em.sum()>=p['parking']['evaluation_s']/.02-2)
            if complete:
                v=wv[mask,:2];pp=pos[mask,:2];yy=np.unwrap(angles[mask,2]);drift=float(np.linalg.norm(pp-pp[0],axis=1).max());yd=float(abs(yy-yy[0]).max());rms=np.sqrt(np.mean(v*v,axis=0));wr=float(np.sqrt(np.mean(omega[mask,2]**2)));zero=np.max(abs(arr(execution,'requested',3)[em]))<1e-6 and np.max(abs(arr(execution,'command',3)[em]))<1e-6;active=all(r.get('actual_execution_health',{}).get('ready') is True for r in np.asarray(execution,dtype=object)[em]) if controller=='champ' else all(r.get('actor_inferred_this_frame') is True and r.get('state') not in ['support_hold','support_capture'] for r in np.asarray(execution,dtype=object)[em])
                park=bool(drift<=p['parking']['maximum_translation_drift_m'] and yd<=p['parking']['maximum_yaw_drift_rad'] and rms.max()<=p['parking']['maximum_world_planar_component_rms_mps'] and wr<=p['parking']['maximum_body_yaw_rate_rms_radps'] and zero and active)
                checks['fixed_final_zero_command_parking']=check(park,world_window_s=[start,end],window_trigger='first measured successful navigation status; rawSLAM receipt independentlychecked',translation_drift_m=drift,yaw_drift_rad=yd,world_planar_component_rms_mps=rms,body_yaw_rate_rms_radps=wr,continuous_zero_requested_and_command=zero,continuous_controller=active,native_samples=int(mask.sum()))
            else:checks['fixed_final_zero_command_parking']=check(None,world_window_s=[start,end],reason='Incomplete fixed final parking window')
        else:checks['fixed_final_zero_command_parking']=check(None,reason='Destination not reached: final parking not tested')
        if case.get('fixture_kind')=='ramp':
            co=p['contact'];surfaces={name:ramphelper.surface(world,name,helper) for name in [case['start_landing'],case['ramp'],case['destination_landing']]};centers,feet=helper.foot_centers(world,steps,contract['joint_order']);masks,geometry=ramphelper.support_masks(steps,centers,feet,surfaces,co);checks['ramp_actual_contact_geometry']=check(geometry['missing_or_malformed_contact_geometry_rows']==0,**geometry)
            arrival_end=arrivals[-1].get('claimed_receipt',{}).get('stamp_ns') if arrivals and arrivals[-1].get('claimed_receipt') else None;movement=(nt>=anchor['pose_stamp_ns']/1e9)&(nt<arrival_end/1e9 if arrival_end is not None else True);initial=(nt>=1.5)&(nt<anchor['pose_stamp_ns']/1e9);final=(nt>=arrival_end/1e9)&(nt<=arrival_end/1e9+8) if arrival_end is not None else np.zeros(len(nt),bool)
            legs={};allgood=True
            for leg in centers:
                b,bp=longest(masks[case['start_landing']][leg]&initial,nt,gap);m,mp=longest(masks[case['ramp']][leg]&movement,nt,gap);f,fp=longest(masks[case['destination_landing']][leg]&final,nt,gap);allgood&=b>=co['each_foot_initial_landing_support_s']-1e-8 and m>=co['each_foot_active_ramp_support_s']-1e-8 and f>=co['each_foot_final_landing_support_s']-1e-8;legs[leg]={'initial_s':b,'ramp_s':m,'final_s':f,'intervals':[bp,mp,fp]}
            four=np.logical_and.reduce(list(masks[case['destination_landing']].values()));fs,fsi=longest(four&final,nt,gap);bins=[]
            anyfoot=np.logical_or.reduce(list(masks[case['ramp']].values()))
            for low in np.arange(2,14,co['ramp_coverage_bin_m']):
                coverage=np.zeros(len(nt),bool)
                for leg in centers:coverage|=masks[case['ramp']][leg]&(centers[leg][:,0]>=low)&(centers[leg][:,0]<low+co['ramp_coverage_bin_m'])
                s,interval=longest(coverage&movement,nt,gap);bins.append({'x_low_m':low,'duration_s':s,'interval_s':interval})
            progression=case['world_forward_x_sign']*(pos[:,0]-case['spawn'][0]);progress=float(progression.max());fixture=p['ramp_fixtures'][case['ramp']];ramp=surfaces[case['ramp']];expected_normal=np.array([-fixture['slope_z_per_world_x'],0,1.])/math.sqrt(1+fixture['slope_z_per_world_x']**2);geometry_ok=abs(2*ramp['half'][1]-fixture['width_m'])<1e-10 and np.allclose(ramp['rotation'][:,2],expected_normal,atol=1e-10) and abs(ramp['center'][0]-8)<1e-10 and abs(ramp['center'][1]-fixture['center_y_m'])<1e-10
            checks['complete_12m_ramp_and_both_transitions']=check(geometry_ok and allgood and progress>=case['minimum_total_route_progress_m'] and fs>=co['final_simultaneous_four_foot_landing_support_s']-1e-8 and all(v['duration_s']>=co['each_coverage_bin_any_foot_support_s']-1e-8 for v in bins),each_foot=legs,final_four_foot_simultaneous_s=fs,final_four_foot_interval_s=fsi,coverage_bins=bins,destination_reached=arrival_end is not None,actual_progress_m=progress,original_width_slope_and_center=geometry_ok,height_gain_is_gate=False)
        elif case.get('fixture_kind')=='continuous_ramps':
            checks['complete_12m_ramp_and_both_transitions']=check(None,reason='Continuous two-ramp stage requires a separate frozen per-segment contact audit before execution')
        for name in ['pid_navigation_protocol.json','pid_navigation_case.json','pid_navigation_freeze.json','navigation_scope.json','navigation_profile.json','navigation_anchor.json','navigation_request.json','navigation_slam_poses.jsonl','navigation_status.jsonl','navigation_pid_history.jsonl','navigation_command_history.jsonl','navigation_sensor_gate_history.jsonl','navigation_cloud_history.jsonl',guardfile,'telemetry.jsonl','actuator.jsonl','champ_contract.json','policy_metadata.json','runtime_manifest.json','source_manifest.json','world.sdf']:
            if (run/name).is_file():inputs[name]=sha(run/name)
    except Exception as e:errors.append(f'{type(e).__name__}: {e}')
    required=['prospective_case_and_scope','source_provenance','frozen_protocol_and_case','raw_sensor_slam_stream','actual_sensor_publisher_chain','immutable_raw_slam_route','all_raw_slam_regions','complete_runtime_and_clean_exit','controller_identity_and_execution','continuous_native_200hz','causal_command_TTL_and_slew','exclusive_actuator_and_no_body_servo','physical_safety_and_clearance','continuous_actual_foot_contact','actual_joint_execution','native_force_and_joint_velocity_limits','actual_registered_cloud_inputs','actual_checked_SCAN_trajectories','PID_measured_source_and_execution','PID_actual_terms_and_bounds','complete_PID_and_guard_recording','physical_route_tracking_and_actual_motion','forward_motion_during_drive','actual_PID_motion_corridor_guard','fixed_final_zero_command_parking']
    if case.get('fixture_kind') in ['ramp','continuous_ramps']:required+=['ramp_actual_contact_geometry','complete_12m_ramp_and_both_transitions']
    outcome='failed' if errors else status(checks,required)
    levels={'interface_and_sources':status(checks,['prospective_case_and_scope','source_provenance','frozen_protocol_and_case','controller_identity_and_execution','exclusive_actuator_and_no_body_servo']), 'actual_measured_navigation':status(checks,['raw_sensor_slam_stream','actual_sensor_publisher_chain','immutable_raw_slam_route','all_raw_slam_regions','actual_checked_SCAN_trajectories','actual_registered_cloud_inputs','PID_measured_source_and_execution']), 'physical_route_and_safety':status(checks,['continuous_native_200hz','physical_safety_and_clearance','physical_route_tracking_and_actual_motion','forward_motion_during_drive']), 'destination_parking':status(checks,['fixed_final_zero_command_parking']), 'clean_runtime':status(checks,['complete_runtime_and_clean_exit']), 'case_complete':outcome,'CHAMP_comparison':'unverified without actual matched CHAMP/Teacher runs','continuous_multifloor':'unverified','global_sim2sim':'unverified','real_robot':'unverified'}
    result=clean({'schema':'teacher_champ_pid_navigation_independent/v2-force-phase','run':str(run),'controller':controller,'case_id':case.get('case_id'),'status':outcome,'levels':levels,'checks':checks,'required_checks':required,'metrics':metrics,'errors':errors,'input_hashes':inputs,'analyzer_sha256':sha(__file__),'protocol':p,'original_receipts_overwritten':False,'truth_scope':'Post-run physical evaluation only. No truth-fed command, region arrival or PID.', 'limits':'No actual baseline or route result from a loaded model, stand-only run, historical camera Demo, missing payload, or static code presence.'})
    if write:
        (run/'summary_pid_navigation_independent.corrected_v2.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        if arrays:np.savez_compressed(run/'pid_navigation_independent_arrays.corrected_v2.npz',**arrays)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('runs',type=Path,nargs='+');parser.add_argument('--no-write',action='store_true');a=parser.parse_args()
    for run in a.runs:
        r=evaluate(run,not a.no_write);print(json.dumps({'run':str(run),'case_id':r['case_id'],'controller':r['controller'],'status':r['status'],'failed':[k for k,v in r['checks'].items() if v['status']=='failed'],'unverified':[k for k,v in r['checks'].items() if v['status']=='unverified'],'errors':r['errors']},ensure_ascii=False,allow_nan=False),flush=True)
if __name__=='__main__':main()
