"""Read-only bounded prefix facts, not the formal full-route acceptance replay."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np


def rows(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():yield json.loads(line)


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def stats(values):
    a=np.asarray([x for x in values if isinstance(x,(int,float)) and math.isfinite(x)],float)
    return None if not len(a) else dict(count=len(a),min=float(np.min(a)),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(np.max(a)))

def rate(stamps):
    a=np.asarray(stamps,dtype=float);d=np.diff(a);pos=d[d>0]
    return dict(samples=len(a),positive_advances=len(pos),duplicates=int(sum(d==0)),backward=int(sum(d<0)),
        span_s=None if len(a)<2 else float(a[-1]-a[0]),header_hz=None if len(a)<2 or a[-1]<=a[0] else float((len(a)-1)/(a[-1]-a[0])),
        positive_dt_s=stats(pos.tolist()))

def evaluate(run):
    result=json.loads((run/'run_result.json').read_text());worker=json.loads((run/'worker_result.json').read_text());profile=json.loads((run/'navigation_profile.json').read_text())
    count=0;faults=[];native_states={};maxroll=maxpitch=maxqd=maxtau=0.;bodycontacts=0;clearances=[];last=None;lastwall=None;cmdreject=[];active_cmdages=[];velocities=[]
    for r in rows(run/'telemetry.jsonl'):
        count+=1;last=r;native_states[r['state']]=native_states.get(r['state'],0)+1
        if r.get('fault') is not None:faults.append(dict(sim_time=r['sim_time'],fault=r['fault']))
        if r['state'] not in ('initializing','settling'):
            maxroll=max(maxroll,abs(r['rpy'][0]));maxpitch=max(maxpitch,abs(r['rpy'][1]));maxqd=max(maxqd,max(map(abs,r['qd'])));maxtau=max(maxtau,max(map(abs,r['applied_torque'])));clearances.append(r['body_clearance']);bodycontacts+=r['contacts']['body']>0
        d=r.get('closed_loop_read_evidence')or{};read=d.get('actual_read_attempt')or{}
        if read.get('read_monotonic_wall')is not None:lastwall=read['read_monotonic_wall']
        if d.get('rejected'):cmdreject.append(dict(sim_time=r['sim_time'],reason=d.get('reason')))
        if r.get('actor_inferred_this_frame') and r.get('state')=='tracking':
            active_cmdages.append(r.get('command_age_sim_s'));velocities.append(r['body_lin_vel'])
    endclock=last['world_sim_time'] if last else 0
    navcounts={};arrivals=[];firstfail=None;lastphysical=None;ages={k:[]for k in('pose_age','cloud_age','raw_imu_age')};nonzeromissing=0;truth=[];maxpatherr=[];maxyaw=[];bridge={};holds=0;total=0;cleanupstates=[]
    for r in rows(run/'navigation_status.jsonl'):
        physical=(r.get('ros_sim_time',0)<=endclock+1e-6 and (lastwall is None or r.get('monotonic_wall',float('inf'))<=lastwall))
        if not physical:
            cleanupstates.append(dict(state=r.get('state'),sim_time=r.get('ros_sim_time'),wall=r.get('monotonic_wall'),message=r.get('message')));continue
        total+=1;lastphysical=r;navcounts[r['state']]=navcounts.get(r['state'],0)+1;truth.append(r.get('navigation_ground_truth_used'))
        if r['state']=='failed' and firstfail is None:firstfail=dict(sim_time=r.get('ros_sim_time'),message=r.get('message'),waypoint_index=r.get('waypoint_index'))
        a=r.get('region_arrivals')or[]
        if len(a)>len(arrivals):arrivals=a
        cmd=r.get('command')or[0,0,0];nz=any(abs(v)>1e-9 for v in cmd)
        if r.get('state')=='running':
            for k in ages:
                if isinstance(r.get(k),(int,float)):ages[k].append(r[k])
                elif nz:nonzeromissing+=1
        c=r.get('cascade_parking')or{}
        if c.get('error_cross_m')is not None:maxpatherr.append(c['error_cross_m'])
        if c.get('error_yaw_rad')is not None:maxyaw.append(c['error_yaw_rad'])
        b=(r.get('execution_bridge_safety')or{}).get('state');bridge[b]=bridge.get(b,0)+1
        holds+=bool(r.get('tilt_hold')or r.get('obstacle_hold')or c.get('protection_active'))
    poses=[];poseages=[]
    for r in rows(run/'navigation_slam_poses.jsonl'):
        if lastwall is None or r['received_monotonic_wall']<=lastwall:
            poses.append(r['stamp_ns']/1e9);poseages.append(r.get('sim_age_at_callback_s'))
    mathstamps=[];modestats={};pidtruth=[];pidprotect=[];speederrors=[[],[],[]];firstpath=None;lastpath=None;pathIDs=set();PIDcount=0
    for r in rows(run/'navigation_pid_history.jsonl'):
        if lastwall is not None and r.get('compute_monotonic_wall',float('inf'))>lastwall:continue
        PIDcount+=1;c=r.get('cascade')or{};pidtruth.append(r.get('navigation_ground_truth_used'));modestats[r.get('mode')]=modestats.get(r.get('mode'),0)+1
        if c.get('controller_updated')is True:mathstamps.append(r['source_pose_stamp_ns']/1e9)
        if c.get('protection_active')or c.get('failure_latched'):pidprotect.append(dict(seq=r.get('sequence'),sim_time=r['compute_ros_clock_ns']/1e9,reason=c.get('reason')))
        for i,v in enumerate((c.get('velocity_PI')or{}).get('error')or[]):speederrors[i].append(abs(v))
        pr=r.get('path_receipt')or{}
        if pr:
            if firstpath is None:firstpath=pr
            lastpath=pr;pathIDs.add(r.get('trajectory_id'))
    loaded=json.loads((run/'slam_loaded_binary.json').read_text());scope=json.loads((run/'navigation_scope.json').read_text())
    refs={name:digest(run/name)for name in('run_result.json','worker_result.json','navigation_scope.json','navigation_profile.json','slam_loaded_binary.json','source_manifest.json','navigation_source_snapshots.json')}
    lifecycle=worker.get('pipeline')or{}
    out=dict(schema='teacher_v19_bounded_prefix_facts/v1',run_id=run.name,run_dir=str(run.resolve()),binding_sha256=refs,
        status='FACTS_RECORDED_FULL_ROUTE_UNVERIFIED',full_route_pass=False,formal_source_and_native_replay_performed=False,
        runner=result,native=dict(samples=count,worker_samples=worker.get('samples'),faults=faults,states=native_states,last_sim_time=last.get('sim_time'),last_world_time=endclock,
            max_abs_roll_rad=maxroll,max_abs_pitch_rad=maxpitch,max_abs_qd_radps=maxqd,max_abs_torque_Nm=maxtau,min_clearance_m=min(clearances)if clearances else None,
            body_contact_frames=bodycontacts,command_rejection_count=len(cmdreject),first_command_rejections=cmdreject[:5],tracking_command_age_sim_s=stats(active_cmdages)),
        physical_window=dict(last_native_command_read_monotonic_wall=lastwall,status_rows=total,nav_state_counts=navcounts,first_failed_status=firstfail,
            last_state=lastphysical.get('state')if lastphysical else None,last_waypoint_index=lastphysical.get('waypoint_index')if lastphysical else None,
            region_arrivals=len(arrivals),goal_total=lastphysical.get('total')if lastphysical else None,region_receipts=arrivals,
            protection_status_rows=holds,bridge_state_counts=bridge,source_ages_s={k:stats(v)for k,v in ages.items()},nonzero_missing_source_age_fields=nonzeromissing,
            navigation_truth_fields_all_false=all(x is False for x in truth+pidtruth),cleanup_status_count=len(cleanupstates),last_cleanup_status=cleanupstates[-1]if cleanupstates else None),
        control=dict(actual_SLAM_header=rate(poses),pose_callback_sim_age_s=stats(poseages),math_update_source_header=rate(mathstamps),pid_rows=PIDcount,modes=modestats,
            protection_rows=pidprotect,cross_track_error_max_abs_m=max(map(abs,maxpatherr))if maxpatherr else None,
            cross_track_error_rms_m=float(np.sqrt(np.mean(np.square(maxpatherr))))if maxpatherr else None,
            yaw_error_max_abs_rad=max(map(abs,maxyaw))if maxyaw else None,
            velocity_PI_absolute_error_mean=[float(np.mean(v))if v else None for v in speederrors],
            actual_SCAN_trajectory_ids=len(pathIDs),first_path_receipt=firstpath,last_path_receipt=lastpath),
        model_and_observation=dict(checkpoint_sha256=profile.get('checkpoint_sha256'),SLAM_binary_verified=loaded.get('verified'),
            navigation_ground_truth_used=profile.get('navigation_ground_truth_used'),policy_observations=profile.get('policy_observations'),pipeline=profile.get('pipeline')),
        limitations=['Prefix only; original46 mission is a separate run/profile.',
            'Facts are computed from actual serialized telemetry/navigation records, but full mathematical/geometry/source-hash acceptance replay is not performed here.',
            'No complete ramp, dynamic obstacle, full-route arrival or final5s parking success is asserted.',
            'End-of-physics classification uses last native actual command-read monotonic wall boundary, retaining later cleanup states separately.'])
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not (a.run/'run_result.json').exists():raise SystemExit('Refuse final evaluation before run_result')
    out=evaluate(a.run.resolve());a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x')as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(output=str(a.output),native=out['native'],physical={k:v for k,v in out['physical_window'].items()if k!='region_receipts'},control={k:v for k,v in out['control'].items()if k not in('first_path_receipt','last_path_receipt')}),ensure_ascii=False,allow_nan=False))
