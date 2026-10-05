#!/usr/bin/env python3
"""Independent read-only audit of a finite measured-SLAM Teacher route.

Only creates analysis outputs. It never starts ROS, publishes commands, alters
acceptance, or substitutes simulator truth into navigation arrival evidence.
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

ROOT=Path(__file__).resolve().parents[1]
SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def rows(p):return [json.loads(s)for s in Path(p).read_text().splitlines()if s.strip()]
def optional(run,*names):
    for name in names:
        if(run/name).is_file():return rows(run/name),name
    return [],None
def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,dict):return {str(k):clean(x)for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [clean(x)for x in v]
    if isinstance(v,float)and not math.isfinite(v):return None
    return v
def check(value,**evidence):
    return {'status':'unverified'if value is None else'passed'if bool(value)else'failed','passed':None if value is None else bool(value),**evidence}
def array(records,key,width=None):
    a=np.asarray([r.get(key)for r in records],float);shape=(len(records),)if width is None else(len(records),width)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError(f'{key}: invalid finite shape {a.shape}, expected {shape}')
    return a
def load(path,prefix):
    name=prefix+digest(path)[:12];spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
def status_of(checks,names):
    values=[checks.get(k,{}).get('status','unverified')for k in names]
    return 'failed'if'failed'in values else'unverified'if'unverified'in values else'passed'
def longest(mask,t,gap):
    start=None;best=0.;bestpair=None
    for i,valid in enumerate(mask):
        if not valid:start=None;continue
        if start is None or(i and t[i]-t[i-1]>gap):start=float(t[i])
        if t[i]-start>best:best=float(t[i]-start);bestpair=[start,float(t[i])]
    return best,bestpair


def evaluate(run, write_outputs=True):
    run=Path(run).resolve();checks={};metrics={};errors=[];scope={};profile={};sources={};telemetry=[];native=[];slam=[];status=[]
    try:
        scope=read(run/'navigation_scope.json');profile=read(run/'navigation_profile.json');meta=read(run/'policy_manifest.json');runtime=read(run/'runtime_manifest.json');result=read(run/'worker_result.json')
        telemetry=rows(run/'telemetry.jsonl');raw=rows(run/'actuator.jsonl');native=[r for r in raw if r.get('kind')=='physics_step'];contracts=[r for r in raw if r.get('kind')=='actuator_contract'];contract=contracts[0]
        helper=load(run/'sources/scripts/evaluate_step_functional.py','nav_native_');observation=load(run/'sources/policy/observation.py','nav_observation_')
        t=array(telemetry,'sim_time');wt=array(telemetry,'world_sim_time');nt=array(native,'t');pos=array(native,'position',3);q=array(native,'quaternion_wxyz',4);rot=helper.quaternion_rotations(q);cmd=array(telemetry,'command',3);requested=array(telemetry,'requested',3);vel=array(native,'body_lin_vel_com',3)
        acts=array(telemetry,'action',12);targets=array(telemetry,'q_target',12);policy=t>=.1;target_error=float(abs(targets[policy]-(np.asarray(contract['initial_q'])+.25*acts[policy])).max())
        with np.load(run/'observations_actions.npz')as n:obs=n['observations'];saved_actions=n['actions']
        robot=ET.parse(run/'world.sdf').getroot().find("world/model[@name='go2']");writer=[p for p in robot.findall('plugin')if'TeacherActuator'in p.get('name','')];other=[p.get('name')for p in robot.findall('plugin')if any(v in p.get('name','').lower()for v in['control','champ','stabilizer','trajectory'])]
        checks['cpu_teacher_and_exclusive_actuation']=check(len(contracts)==1 and meta.get('test')=='navigation'and meta.get('inference_device')=='cpu'and meta.get('checkpoint_sha256')==SHA and len(writer)==1 and not other and obs.shape==(len(t),247)and saved_actions.shape==(len(t),12)and np.isfinite(obs).all()and np.isfinite(saved_actions).all()and target_error<1e-6 and not any(r.get('state')in['support_hold','support_capture']for r in telemetry)and contract.get('body_pose_resets')==0,target_action_max_error_rad=target_error,policy_samples=len(t),actor_device=meta.get('inference_device'),joint_writer_plugins=len(writer),other_control_plugins=other,body_pose_resets=contract.get('body_pose_resets'),privileged_policy_observation=meta.get('observation_source'))
        command_spec=observation.CONTRACT['observations']['concatenation_order'][3];command_input=np.clip(cmd,*command_spec['clip'])*command_spec['scale_after_clip'];command_input_error=float(abs(obs[:,9:12]-command_input.astype(np.float32)).max());action_archive_error=float(abs(saved_actions-acts).max());teacher_frames=all(r.get('actor_inferred_this_frame')is True for r in telemetry if r['sim_time']>=.1)
        teacher_flags_present=all('actor_inferred_this_frame'in r for r in telemetry if r['sim_time']>=.1);continuous_value=False if command_input_error>=1e-7 or action_archive_error>=1e-7 else None if not teacher_flags_present else teacher_frames;checks['teacher_continuous_actor_and_command_input']=check(continuous_value,actor_inference_metadata_present=teacher_flags_present,actor_inferred_each_policy_frame=teacher_frames if teacher_flags_present else None,command_observation_max_error=command_input_error,saved_action_max_error=action_archive_error,meaning='Commands reach the actual247D actor input; action archive and actual PD targets remain linked after bootstrap; missing legacy inference flags remainunverified')
        code={p['role']:p.get('returncode')for p in runtime.get('owned_processes',[])}
        checks['actual_runtime_continuity']=check(result.get('completed')is True and not result.get('fault')and not runtime.get('error')and code.get('worker')==0 and code.get('gazebo')==0 and steps_terminate(native)and np.all(np.diff(t)>0)and np.max(np.diff(t))<=.020001 and np.all(np.diff(nt)>0)and np.max(np.diff(nt))<=.005001 and not any(r.get('kind')=='fault'for r in raw)and not any(r.get('fault')for r in native),duration_s=t[-1],worker_fault=result.get('fault'),runner_error=runtime.get('error'),owned_exit_codes=code,maximum_policy_gap_s=np.max(np.diff(t)),maximum_native_gap_s=np.max(np.diff(nt)))
        worldvel=np.einsum('nij,nj->ni',rot,vel);origin_worldvel=np.einsum('nij,nj->ni',rot,array(native,'body_lin_vel_origin',3));integral_error=float(abs(np.diff(pos,axis=0)-origin_worldvel[1:]*np.diff(nt)[:,None]).max())
        w,x,y,z=q.T;rp=np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1))]);terrain=observation.TerrainHeightMap.from_sdf(run/'world.sdf');clear=pos[:,2]-terrain.height(pos[:,:2],ray_start_z=pos[:,2]);contact=array(native,'contacts',5);safe=nt>=wt[0]+.1
        checks['native_physical_safety']=check(abs(rp[safe]).max()<=.65 and clear[safe].min()>=.18 and not(contact[safe,0]>0).any()and not(contact[safe]<0).any()and integral_error<=1e-4,max_abs_roll_pitch_rad=abs(rp[safe]).max(),minimum_support_clearance_m=clear[safe].min(),body_contact_samples=int((contact[safe,0]>0).sum()),missing_contact_samples=int((contact[safe]<0).sum()),pose_velocity_integral_max_error_m=integral_error,native_sample_hz=200,start_relative_sim_s=.1)
        nonzero=np.linalg.norm(cmd[:,:2],axis=1)>.015;motion=np.linalg.norm(worldvel[:,:2],axis=1)>.05;excursion=float(np.linalg.norm(pos[:,:2]-np.median(pos[(nt>=wt[0]+1.5)&(nt<=wt[0]+3),:2],axis=0),axis=1).max());active_seconds=float(nonzero.sum()*.02);measured_seconds=float(motion.sum()*.005)
        checks['teacher_received_and_executed_motion']=check(active_seconds>=.5 and measured_seconds>=.5 and excursion>=.10,nonzero_velocity_command_seconds=active_seconds,native_actual_motion_seconds=measured_seconds,maximum_actual_xy_excursion_m=excursion,meaning='CPU actor inference alone and zero-command standing do not establish route execution')
        checks['scope_is_finite_and_truth_not_navigation']=check(scope.get('status')=='experimental_unverified'and scope.get('navigation_is_verified')is False and scope.get('navigation_ground_truth_used')is False and profile.get('navigation_ground_truth_used')is False and scope.get('experiment')=='flat_relative_roundtrip_v1'and meta.get('navigation_truth_used')is False,global_levels_preserved=scope.get('global_levels_preserved'),excluded_scenarios=profile.get('excluded'),navigation_scope=scope.get('experiment'))
        frozen=read(run/'source_manifest.json');mismatch=[name for name,h in frozen.items()if not(run/'sources'/name).is_file()or digest(run/'sources'/name)!=h];refs=scope.get('references',{});refbad=[];archived_ref_count=0
        for name,h in refs.items():
            source=Path(name)
            try:rel=source.relative_to(ROOT)
            except ValueError:rel=None
            archived=run/'sources'/rel if rel is not None and rel.parts[0]!='runs' else None
            reference=archived if archived is not None and archived.is_file() else source
            archived_ref_count+=int(reference==archived)
            if not reference.is_file()or digest(reference)!=h:refbad.append(name)
        checks['source_integrity']=check(not mismatch and not refbad,archived_files=len(frozen),archived_mismatches=mismatch,scoped_runtime_reference_mismatches=refbad,scope_sha256=digest(run/'navigation_scope.json'),teacher_runtime_sources_verified_from_frozen_archive=archived_ref_count,note='Completed runs use their original archived Teacher sources, not a later hotfixed working copy; external binaries/shared sources remain hash checked')
        status,status_name=optional(run,'navigation_status.jsonl');slam,slam_name=optional(run,'navigation_slam_poses.jsonl');health,health_name=optional(run,'navigation_sensor_gate_history.jsonl');commands,commands_name=optional(run,'navigation_command_history.jsonl');trajectories,traj_name=optional(run,'navigation_trajectories.jsonl','navigation_trajectory_history.jsonl');clouds,cloud_name=optional(run,'navigation_cloud_inputs.jsonl','navigation_cloud_history.jsonl')
        trajectory_files=sorted((run/'navigation_trajectories').glob('*.json'))
        if trajectory_files:trajectories=[read(p)for p in trajectory_files];traj_name='navigation_trajectories/*.json+*.npz'
        metrics['recorded_inputs']={'navigation_status':status_name,'slam_poses':slam_name,'sensor_gate_history':health_name,'command_history':commands_name,'trajectory_history':traj_name,'cloud_history':cloud_name,'counts':{'status':len(status),'slam_poses':len(slam),'sensor_health':len(health),'commands':len(commands),'trajectories':len(trajectories),'cloud_inputs':len(clouds)}}
        ready=[r for r in health if r.get('ready')is True];ready_good=all(r.get('ground_truth_navigation_used')is False and r.get('overview_used_for_vio')is False and all(v.get('passed')is True for v in r.get('publisher_graph',{}).values())and bool(r.get('publisher_graph'))for r in ready)
        checks['actual_sensor_slam_publisher_chain']=check(bool(ready)and ready_good if health else None,ready_snapshots=len(ready),history_available=bool(health),required_sources=['actual vehicleRGB/CameraInfo','actual IMU/LiDAR','laserMapping registered clouds','demo_slam_odom_adapter'],last_snapshot_is_not_full_history=True)
        if slam:
            stamps=array(slam,'stamp_ns');sp=array(slam,'position',3);sq=array(slam,'quaternion',4);checks['measured_slam_pose_stream']=check(np.all(np.diff(stamps)>0)and all(r.get('frame_id')=='camera_init'and r.get('child_frame_id')=='demo_slam_body'for r in slam)and np.allclose(np.sum(sq*sq,axis=1),1,atol=.02),samples=len(slam),median_period_s=np.median(np.diff(stamps))/1e9,maximum_period_s=np.max(np.diff(stamps))/1e9)
        else:checks['measured_slam_pose_stream']=check(None,reason='No recorded accepted raw SLAM poses')
        accepted=[r for r in status if r.get('accepted_trajectory_id')is not None];trajectory_ids=sorted(set(r['accepted_trajectory_id']for r in accepted));metrics['planner']={'accepted_trajectory_ids':trajectory_ids,'maximum_replans':max([r.get('replans',0)for r in status],default=0),'maximum_reference_requests':max([r.get('reference_requests',0)for r in status],default=0),'maximum_degenerate_splines':max([r.get('degenerate_splines',0)for r in status],default=0),'last_rejection':status[-1].get('last_spline_rejected')if status else None}
        checks['actual_scan_trajectory_association']=check(bool(trajectory_ids)if status else None,**metrics['planner'],native_coefficients_metadata_recorded=bool(trajectories),limit='Accepted trajectory IDs/status prove an association occurred; independent native spline/collision replay requires original coefficients, knots and cloud snapshot')
        payload_errors=[];payload_metrics=[]
        for path,data in zip(trajectory_files,trajectories):
            archive=path.parent/data['array_file']
            if not archive.is_file()or digest(archive)!=data['array_sha256']:payload_errors.append(path.name+': array hash mismatch');continue
            with np.load(archive)as saved:c=saved['coefficients'];k=saved['knots'];s=saved['samples']
            valid=data.get('order')==3 and c.ndim==2 and c.shape[1]==3 and len(k)==len(c)+4 and s.ndim==2 and s.shape[1]==3 and len(s)>1 and np.isfinite(c).all()and np.isfinite(k).all()and np.isfinite(s).all()and np.all(np.diff(k)>=0)and k[-4]>k[3]
            metadata=data.get('metadata',{});payload=metadata.get('trajectory',{});declared_c=np.asarray(payload.get('pos_pts',[]),float);declared_k=np.asarray(payload.get('knots',[]),float)
            payload_equal=payload.get('traj_id')==data.get('trajectory_id')and payload.get('order')==3 and declared_c.shape==c.shape and declared_k.shape==k.shape and np.array_equal(c,declared_c)and np.array_equal(k,declared_k)and metadata.get('reference_stamp')==data.get('reference_stamp')
            curve_error=None
            if valid:
                from scipy.interpolate import BSpline
                rebuilt=BSpline(k,c,3)(np.linspace(k[3],k[-4],len(s)));curve_error=float(abs(rebuilt-s).max());valid&=curve_error<1e-9
            valid&=payload_equal
            if not valid:payload_errors.append(path.name+': invalid finite cubic spline arrays')
            payload_metrics.append({'file':str(path.relative_to(run)),'trajectory_id':data.get('trajectory_id'),'request_id':data.get('request_id'),'waypoint_index':data.get('waypoint_index'),'reference_stamp':data.get('reference_stamp'),'samples':len(s),'sample_path_length_m':float(np.linalg.norm(np.diff(s,axis=0),axis=1).sum()),'array_hash_verified':True,'metadata_payload_exactly_equal':payload_equal,'independent_spline_reconstruction_max_error_m':curve_error,'metadata':metadata})
        checks['native_scan_payload_available']=check(not payload_errors if trajectories else None,trajectory_records=len(trajectories),cloud_input_records=len(clouds),payload_errors=payload_errors,payloads=payload_metrics,note='Cloud stamp/count/hash prove recorded inputs; absent original XYZ buffers cannot support independent collision replay')
        cloud_bad=[]
        for r in clouds:
            stamp=r.get('stamp_ns');near=r.get('nearest_slam_pose_stamp_ns');h=r.get('filtered_xyz_float64_sha256','')
            if r.get('frame_id')!='camera_init'or not isinstance(stamp,int)or not isinstance(near,int)or abs(stamp-near)>150000001 or r.get('filtered_points',0)<1 or len(h)!=64:cloud_bad.append(stamp)
        checks['actual_registered_cloud_inputs']=check(not cloud_bad if clouds else None,processed_clouds=len(clouds),invalid_record_stamps=cloud_bad,note='Actual controller-accepted registered XYZ inputs retain stamp, nearest SLAM pose, point counts and buffer hash; raw XYZ not retained for independent collision recheck')
        checks['command_bridge_history']=check(True if commands else None,snapshots=len(commands),note='Final overwritten command file is insufficient to independently prove all source ages or sequences')
        if commands:
            seq=array(commands,'sequence');healthy=[r for r in commands if r.get('healthy')is True];bad=[]
            for r in healthy:
                if r.get('source')!='scan_slam'or r.get('mode')!='scan_slam'or r.get('observation_source',{}).get('navigation_ground_truth_used')is not False or not all(v is not None and -.05<=v<.3 for v in r.get('ages',{}).values())or r.get('acceptance',{}).get('sha256')!=digest(run/'navigation_scope.json')or r.get('publisher_graph',{}).get('slam')is not True or r.get('publisher_graph',{}).get('command')is not True:bad.append(r.get('sequence'))
            checks['healthy_command_source_ages']=check(np.all(np.diff(seq)>0)and bool(healthy)and not bad,healthy_snapshots=len(healthy),invalid_healthy_sequences=bad)
        else:checks['healthy_command_source_ages']=check(None,reason='No command envelope history')
        expirations=[r for r in telemetry if r.get('command_expired')is True];expired_nonzero=sum(np.linalg.norm(r.get('requested',[0,0,0]))>1e-8 for r in expirations);checks['expired_input_requests_zero_velocity']=check(expired_nonzero==0 if expirations else None,expired_policy_samples=len(expirations),expired_nonzero_requests=expired_nonzero,limit='Startup wait can demonstrate zero-input holding but not moving-to-stale physical stopping')
        moving_stale=[]
        for i,r in enumerate(telemetry):
            if r.get('command_expired')is not True:continue
            if i and telemetry[i-1].get('command_expired')is True:continue
            before=max(0,i-25)
            if np.linalg.norm(cmd[before:i,:2],axis=1).max()>.015 if i>before else False:moving_stale.append(float(r['sim_time']))
        metrics['TTL']={'moving_stale_transition_times_s':moving_stale,'expired_samples':len(expirations),'expired_zero_request_samples':len(expirations)-expired_nonzero,'nominal_timeout_s':.3}
        timing_helper=load(Path(__file__).with_name('audit_navigation_timing.py'),'nav_timing_');timing=timing_helper.evaluate(run,write=write_outputs);timing_checks=timing['checks'];checks['causal_worker_command_source']=check(timing_checks['worker_read_envelope_lineage']['status']=='passed' and timing_checks['observed_command_TTL_boundary']['status']=='passed' and timing_checks['actual_teacher_command_slew']['status']=='passed' and timing_checks['recorded_transport_timing']['status']!='failed',checks=timing_checks,read_status_counts=timing['metrics']['worker_read_status_counts'],accepted_nonzero_frames=timing['metrics']['accepted_nonzero_planar_velocity_frames'],timing_analyzer_sha256=timing['analyzer_sha256'])
        metrics['execution_timing']=timing['metrics'];moving_nonzero_frames=timing['metrics']['accepted_nonzero_planar_velocity_frames'];checks['nonzero_commands_have_accepted_slam_source']=check(moving_nonzero_frames>=25,accepted_nonzero_frames=moving_nonzero_frames,minimum_nonzero_source_frames=25,meaning='No route execution pass from stale input, unrelated bootstrap motion or stand-only actor loading')
        protocol_path=run/'sources/tests/protocol.json';stop_protocol=read(protocol_path)['stand_stop']if protocol_path.exists()else read(ROOT/'tests/protocol.json')['stand_stop'];settling=float(stop_protocol['settling_seconds']);physical_times=nt-.005;all_yaw=np.unwrap(np.arctan2(rot[:,1,0],rot[:,0,0]));omega=array(native,'body_ang_vel',3)
        def stop_window(start,seconds=3.):
            stopmask=(physical_times>=start-1e-10)&(physical_times<=start+seconds+1e-10);ptmask=(wt>=start-1e-10)&(wt<=start+seconds+1e-10);p=pos[stopmask];duration=float(physical_times[stopmask][-1]-physical_times[stopmask][0])if len(p)>1 else 0.;actorrows=[r for r in telemetry if start-1e-10<=r['world_sim_time']<=start+seconds+1e-10]
            if duration<seconds-.005001 or stopmask.sum()<599 or ptmask.sum()<149:return None,{'world_time_window_s':[start,start+seconds],'recorded_duration_s':duration,'native_samples':int(stopmask.sum()),'reason':'Insufficient continuous settled3s native/Teacher coverage'}
            rms=np.sqrt(np.mean(vel[stopmask,:2]**2,axis=0));yaw=all_yaw[stopmask];drift=float(np.linalg.norm(p[:,:2]-p[0,:2],axis=1).max());yawdrift=float(abs(yaw-yaw[0]).max());yawrate=float(np.sqrt(np.mean(omega[stopmask,2]**2)));zero_input=bool(np.max(abs(requested[ptmask]))<1e-6 and np.max(abs(cmd[ptmask]))<1e-6);active=bool(actorrows and all(r.get('actor_inferred_this_frame')is True and r.get('state')not in['support_hold','support_capture']for r in actorrows));passed=drift<=stop_protocol['translation_drift_m'] and yawdrift<=stop_protocol['yaw_drift_rad'] and rms.max()<=stop_protocol['linear_rms_mps'] and yawrate<=stop_protocol['yaw_rate_rms_radps'] and zero_input and active
            return passed,{'world_time_window_s':[start,start+seconds],'translation_drift_m':drift,'yaw_drift_rad':yawdrift,'linear_rms_mps':rms,'yaw_rate_rms_radps':yawrate,'native_samples':int(stopmask.sum()),'zero_requested_and_actor_velocity_command':zero_input,'continuous_teacher_inference':active,'limits':stop_protocol,'native_state_time':'PreUpdate t minus0.005s'}
        ttl_stop=[]
        for event in timing['metrics']['moving_to_stale_events']:
            begin=event['world_sim_time_s'];end=begin+settling+3.;transition=(wt>=begin-1e-10)&(wt<=end+1e-10);full=bool(transition.any()and wt[transition][-1]>=end-.020001);zero_held=bool(full and np.max(abs(requested[transition]))<1e-6)
            if not zero_held:ttl_stop.append({'event':event,'status':'unverified','reason':'Fresh nonzero command resumed or run ended before complete settling+3s stop interval'});continue
            passed,evidence=stop_window(begin+settling);ttl_stop.append({'event':event,**check(passed,**evidence)})
        ttl_status='failed'if any(r['status']=='failed'for r in ttl_stop)else'passed'if ttl_stop and all(r['status']=='passed'for r in ttl_stop)and checks['causal_worker_command_source']['passed']else'unverified';checks['moving_command_TTL_and_stop_observed']=check(True if ttl_status=='passed'else False if ttl_status=='failed'else None,events=ttl_stop,reason='Only actual motion-to-expired commands followed by a full continuousTeacher physical stop window can pass; transient recovery and standing remainunverified')
        request_path=run/'navigation_request.json';anchor_path=run/'navigation_anchor.json';arrival_records=[]
        if request_path.exists()and anchor_path.exists()and slam:
            request=read(request_path);anchor=read(anchor_path);origin=np.asarray(anchor['origin']);yaw=anchor['yaw'];goals=request['goals'];expected=origin+profile['distance_m']*np.array([np.cos(yaw),np.sin(yaw),0]);anchor_index=np.flatnonzero(stamps==anchor['pose_stamp_ns']);exact_anchor=bool(len(anchor_index)==1 and np.allclose(sp[anchor_index[0]],origin,rtol=0,atol=1e-10))
            anchor_yaw_error=None
            if exact_anchor:
                ax,ay,az,aw=sq[anchor_index[0]];raw_yaw=np.arctan2(2*(aw*az+ax*ay),1-2*(ay*ay+az*az));anchor_yaw_error=float(abs(np.arctan2(np.sin(yaw-raw_yaw),np.cos(yaw-raw_yaw))))
            region_contract=bool(len(goals)==2 and [g['goal_id']for g in goals]==['out_1m','return_origin'] and request.get('frame_id')=='camera_init')
            for goal in goals:
                a=goal['arrival'];b=a.get('control_band',{});region_contract&=a.get('type')=='disc_prism' and b.get('type')=='disc_prism' and a.get('radius_m')==profile['arrival_radius_m'] and b.get('radius_m')==profile['arrival_control_radius_m'] and a.get('height_half_span_m')==profile['arrival_height_half_span_m'] and b.get('height_half_span_m')==profile['arrival_height_half_span_m'] and a.get('dwell_sim_s')==profile['dwell_sim_s'] and goal.get('timeout_sim_s')==profile['goal_timeout_sim_s']
            checks['immutable_slam_relative_route_anchor']=check(anchor.get('source')=='/demo/slam/body_odom'and anchor.get('ground_truth_navigation_used')is False and anchor.get('frozen_once')is True and exact_anchor and anchor_yaw_error is not None and anchor_yaw_error<1e-10 and region_contract and np.allclose(goals[0]['center'],expected,rtol=0,atol=1e-10)and np.allclose(goals[1]['center'],origin,rtol=0,atol=1e-10),anchor_stamp_ns=anchor['pose_stamp_ns'],recorded_exact_anchor_pose=exact_anchor,anchor_yaw_error_against_raw_slam_rad=anchor_yaw_error,region_contract_matches_frozen_profile=region_contract,origin=origin,first_center=goals[0]['center'],return_center=goals[1]['center'])
            latest=next((r for r in reversed(status)if r.get('request_id')==request['request_id']),{});claimed=latest.get('region_arrivals',[])
            for goal in goals:
                evidence=next((r for r in claimed if r.get('goal_id')==goal['goal_id']),None);good=False;details={'goal_id':goal['goal_id'],'claimed_receipt':evidence}
                if evidence:
                    begin=evidence.get('start_stamp_ns');end=evidence.get('stamp_ns');mask=(stamps>=begin)&(stamps<=end);band=goal['arrival'].get('control_band',goal['arrival']);delta=sp[mask]-np.array(goal['center']);inside=np.linalg.norm(delta[:,:2],axis=1)<=band['radius_m']+1e-12;inside&=abs(delta[:,2])<=band['height_half_span_m']+1e-12
                    gap=float(np.max(np.diff(stamps[mask]))/1e9)if mask.sum()>1 else None;duration=float((stamps[mask][-1]-stamps[mask][0])/1e9)if mask.sum()>1 else 0.
                    exact_end=bool(mask.any()and stamps[mask][0]==begin and stamps[mask][-1]==end);good=exact_end and inside.all()and gap is not None and gap<=.200000001 and duration>=profile['dwell_sim_s']-1e-9 and evidence.get('protected')is False and evidence.get('control_region_inside')is True
                    details.update(independent_raw_slam_samples=int(mask.sum()),all_raw_poses_inside_control_region=bool(inside.all()),measured_stamp_dwell_s=duration,maximum_pose_gap_s=gap,exact_begin_end_poses_recorded=exact_end)
                details['passed']=bool(good);arrival_records.append(details)
            checks['both_regions_confirmed_by_raw_slam_dwell']=check(len(arrival_records)==2 and all(r['passed']for r in arrival_records)and latest.get('state')=='succeeded',regions=arrival_records,last_navigation_state=latest.get('state'),last_message=latest.get('message'),latest_waypoint_index=latest.get('waypoint_index'))
            metrics['arrivals']=arrival_records
            # Freeze one evaluation-only rigid alignment at the SLAM anchor.
            # Native PreUpdate state describes previous5ms Physics result.
            physical_times=nt-.005;anchor_world_s=anchor['pose_stamp_ns']/1e9;ni=np.searchsorted(physical_times,anchor_world_s,side='right')-1
            if 0<=ni<len(nt)and anchor_world_s-physical_times[ni]<=.005001:
                a=sq[anchor_index[0]];slamrot=observation.quaternion_rotation(a[[3,0,1,2]]);alignment=slamrot@rot[ni].T;translation=origin-alignment@pos[ni];match=np.searchsorted(physical_times,stamps/1e9,side='right')-1;valid=(match>=0)&(match<len(nt));match=np.clip(match,0,len(nt)-1);valid&=(stamps/1e9-physical_times[match]<=.005001);mapped=pos[match]@alignment.T+translation;error=np.linalg.norm(sp[valid]-mapped[valid],axis=1)
                metrics['independent_truth_error']={'used_for_navigation':False,'alignment':'one immutable rigid transform at SLAM anchor; no online correction','causal_matched_poses':int(valid.sum()),'median_error_m':float(np.median(error)),'p95_error_m':float(np.percentile(error,95)),'maximum_error_m':float(error.max()),'alignment_world_to_slam_rotation':alignment,'alignment_translation':translation,'native_state_time':'PreUpdate t minus0.005s','actual_anchor_world_position':pos[ni]}
                if write_outputs:np.savez_compressed(run/'navigation_independent_truth_error.npz',slam_stamp_ns=stamps[valid],slam_position=sp[valid],truth_aligned_position=mapped[valid],position_error_m=error,alignment_rotation=alignment,alignment_translation=translation)
            else:metrics['independent_truth_error']={'used_for_navigation':False,'status':'unverified','reason':'No causal native state at anchor stamp'}
        else:
            checks['immutable_slam_relative_route_anchor']=check(None,request_recorded=request_path.exists(),anchor_recorded=anchor_path.exists(),raw_slam_recorded=bool(slam));checks['both_regions_confirmed_by_raw_slam_dwell']=check(False if runtime.get('error')is None else None,reason='Actual finite route has no complete anchor/request/pose evidence',last_navigation_state=status[-1].get('state')if status else None)
        succeeded=[r for r in status if r.get('state')=='succeeded'];stopmetrics={}
        if succeeded:
            passed,stopmetrics=stop_window(succeeded[0]['ros_sim_time']+settling);checks['post_arrival_actual_teacher_stop']=check(passed,**stopmetrics)
        else:checks['post_arrival_actual_teacher_stop']=check(None,reason='No measured route success; final-arrival parking not yet reached')
        metrics['stop']=stopmetrics;metrics['last_navigation_status']=status[-1]if status else None
        for path in run.glob('*.json*'):
            if path.name in ['telemetry.jsonl','actuator.jsonl','navigation_status.jsonl','navigation_slam_poses.jsonl','navigation_scope.json','navigation_profile.json','navigation_request.json','navigation_anchor.json','navigation_command_history.jsonl','navigation_sensor_gate_history.jsonl','navigation_trajectories.jsonl','navigation_cloud_inputs.jsonl','worker_result.json','policy_manifest.json','runtime_manifest.json','source_manifest.json']:sources[path.name]=digest(path)
    except Exception as e:errors.append(f'{type(e).__name__}: {e}')
    levels={'interface':status_of(checks,['cpu_teacher_and_exclusive_actuation','source_integrity']),'actual_sensor_slam':status_of(checks,['actual_sensor_slam_publisher_chain','measured_slam_pose_stream']),'scan_planner':status_of(checks,['actual_scan_trajectory_association']),'teacher_physical_execution':status_of(checks,['teacher_received_and_executed_motion','teacher_continuous_actor_and_command_input','nonzero_commands_have_accepted_slam_source','causal_worker_command_source','native_physical_safety','actual_runtime_continuity']),'slam_region_arrival':status_of(checks,['immutable_slam_relative_route_anchor','both_regions_confirmed_by_raw_slam_dwell']),'post_arrival_stop':status_of(checks,['post_arrival_actual_teacher_stop']),'command_TTL_physical_stop':status_of(checks,['moving_command_TTL_and_stop_observed']),'global_sim2sim':scope.get('global_levels_preserved',{}).get('sim2sim','unverified'),'global_navigation':'unverified','dynamic_obstacle':'unverified','multifloor_navigation':'unverified','real_robot':'unverified'}
    required=['cpu_teacher_and_exclusive_actuation','teacher_continuous_actor_and_command_input','actual_runtime_continuity','native_physical_safety','teacher_received_and_executed_motion','nonzero_commands_have_accepted_slam_source','causal_worker_command_source','scope_is_finite_and_truth_not_navigation','source_integrity','actual_sensor_slam_publisher_chain','measured_slam_pose_stream','actual_registered_cloud_inputs','actual_scan_trajectory_association','native_scan_payload_available','healthy_command_source_ages','immutable_slam_relative_route_anchor','both_regions_confirmed_by_raw_slam_dwell','post_arrival_actual_teacher_stop']
    outcome=status_of(checks,required);outcome='failed'if errors else outcome;levels['finite_flat_navigation']=outcome
    receipt=clean({'schema_version':1,'scope':'One metre out/return measuredSLAM/SCAN finite integration audit only; privileged Teacher observations retained','status':outcome,'levels':levels,'checks':checks,'metrics':metrics,'errors':errors,'profile':profile,'source_hashes':sources,'analyzer_sha256':digest(__file__),'global_acceptance_overwritten':False,'truth_scope':'Only separate post-run causal error analysis; never used for NAV anchor, goals, planner, command, or arrival','limits':'Missing raw command/health/trajectory/TTL evidence remains unverified. Static code presence is not closed-loop pass.'})
    if write_outputs:(run/'summary_navigation_independent.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    return receipt


def steps_terminate(native):return bool(native and native[-1].get('terminating')is True)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');args=p.parse_args()
    for run in args.runs:
        result=evaluate(run);print(json.dumps({'run':str(run),'status':result['status'],'levels':result['levels'],'errors':result['errors'],'failed':[k for k,v in result['checks'].items()if v['status']=='failed'],'unverified':[k for k,v in result['checks'].items()if v['status']=='unverified']},allow_nan=False),flush=True)
if __name__=='__main__':main()
