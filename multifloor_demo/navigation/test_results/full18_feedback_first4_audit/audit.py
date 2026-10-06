"""Read-only first4 actual Adapter emit/SLAM/GT decomposition. No ROS init/control.

Published actual Adapter commands use cached float ROS-clock in native log; only
pose/GT/receipt timestamps are original integer header nanoseconds. Truth is
paired without extrapolation under the component's same single initial SE3.
"""
import argparse,bisect,hashlib,json,math,pathlib
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

HERE=pathlib.Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load_rows(p):
    rows=[];errors=[]
    with p.open() as f:
        for i,line in enumerate(f,1):
            try:rows.append(json.loads(line))
            except (ValueError,TypeError) as e:errors.append([i,str(e)])
    return rows,errors

def mode(c):
    if abs(c[0])+abs(c[1])>1e-8:return 'walk'
    if abs(c[5])>1e-8:return 'pure_turn'
    return 'zero'

def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi

class Series:
    def __init__(self,rows,max_gap_ns):
        self.rows=rows;self.stamps=[x['stamp_ns'] for x in rows];self.max_gap_ns=max_gap_ns
        self.origin=self.stamps[0]
        self.ordered=all(b>a for a,b in zip(self.stamps,self.stamps[1:]))
        if not self.ordered:raise ValueError('Unordered original native pose stamps')
        self.slerp=Slerp([(t-self.origin)/1e9 for t in self.stamps],Rotation.from_quat([r['q'] for r in rows]))
        self.cache={}
    def sample(self,t):
        if t in self.cache:return self.cache[t]
        i=bisect.bisect_left(self.stamps,t)
        if i<len(self.rows) and self.stamps[i]==t:r=self.rows[i];result=(np.asarray(r['p']),Rotation.from_quat(r['q']))
        elif not 0<i<len(self.rows) or not 0<self.stamps[i]-self.stamps[i-1]<=self.max_gap_ns:result=None
        else:
            a,b=self.rows[i-1],self.rows[i];fraction=(t-a['stamp_ns'])/(b['stamp_ns']-a['stamp_ns'])
            p=np.asarray(a['p'])+fraction*(np.asarray(b['p'])-a['p'])
            result=(p,self.slerp([(t-self.origin)/1e9])[0])
        self.cache[t]=result;return result

def empty():return dict(duration_s=0.,matched_duration_s=0.,command_integral=[0.,0.,0.],
    world_delta_gt=[0.,0.,0.],world_delta_slam=[0.,0.,0.],body_forward_gt_m=0.,body_forward_slam_m=0.,
    body_lateral_gt_m=0.,body_lateral_slam_m=0.,yaw_delta_gt_rad=0.,yaw_delta_slam_rad=0.,unpaired_intervals=0)

def add(total,item):
    for k in total:
        if isinstance(total[k],list):total[k]=[a+b for a,b in zip(total[k],item[k])]
        else:total[k]+=item[k]

def finalize(x):
    d=x['matched_duration_s'];x['mean_body_forward_gt_m_s']=x['body_forward_gt_m']/d if d else None
    x['mean_body_forward_slam_m_s']=x['body_forward_slam_m']/d if d else None
    x['mean_yaw_rate_gt_rad_s']=x['yaw_delta_gt_rad']/d if d else None
    x['mean_yaw_rate_slam_rad_s']=x['yaw_delta_slam_rad']/d if d else None
    return x

def analyze(run,out):
    original=json.loads((run/'first_four_result.json').read_text())
    commands,parse_errors=load_rows(run/'joint_stop_adapter.jsonl')
    actual=[x for x in commands if x.get('kind')=='actual_champ_command']
    times={};duplicates=[];reversals=[];previous=-1
    for row in actual:
        ns=round(row['sim']*1e9)
        if ns<previous:reversals.append([previous,ns])
        if ns in times:duplicates.append(dict(stamp_ns=ns,value_changed=times[ns]['value']!=row['value']))
        times[ns]=row;previous=ns
    if reversals:raise ValueError('Actual command cached clock reversal; refuse ZOH replay')
    cmd=sorted(times.items());cmd_times=[a for a,b in cmd]
    slam=Series(original['poses'],200_000_000);truth=Series(original['truth'],150_000_000)
    se3=original['region_evaluation'].get('initial_fixed_SE3_evaluation_only')
    if not se3:raise ValueError('No actual single initial evaluation transform')
    rotation=np.asarray(se3['rotation_world_from_slam']);translation=np.asarray(se3['translation'])
    origin=original['origin_stamp_ns'];terminal=original['terminal_stamp_ns']
    regions=original['region_evaluation']['regions']
    intervals=[]
    previous_receipt_end=origin
    for i,r in enumerate(regions):
        if r.get('receipt'):
            intervals.append((previous_receipt_end,r['receipt']['stamp_ns']))
            previous_receipt_end=r['receipt']['stamp_ns']
        else:
            actual_index=max((x['status'].get('waypoint_index',0) for x in original['statuses']
              if x['status'].get('request_id')==original['request_id']),default=-1)
            if actual_index==i and terminal>previous_receipt_end:intervals.append((previous_receipt_end,terminal))
            break
    statuses=[x for x in original['statuses'] if x['status'].get('request_id')==original['request_id']]
    status_times=[x['received_stamp_ns'] for x in statuses]
    def status(t):
        i=bisect.bisect_right(status_times,t)-1
        if i<0:return None
        n=statuses[i]['status'];st=n.get('steering');return dict(received_stamp_ns=status_times[i],
          waypoint_index=n.get('waypoint_index'),phase=n.get('alignment_phase'),locked_heading=n.get('locked_heading'),
          steering=st,trajectory_id=n.get('accepted_trajectory_id'),reference=n.get('trajectory_reference_stamp'))
    def item(a,b,c):
        dt=(b-a)/1e9;out=empty();out['duration_s']=dt;out['command_integral']=[c[0]*dt,c[1]*dt,c[5]*dt]
        s0,s1,g0,g1=slam.sample(a),slam.sample(b),truth.sample(a),truth.sample(b)
        if any(x is None for x in (s0,s1,g0,g1)):out['unpaired_intervals']=1;return out
        out['matched_duration_s']=dt
        for name,q0,q1 in [('gt',g0,g1),('slam',s0,s1)]:
            dp=q1[0]-q0[0];middle=q0[1]*(Rotation.from_rotvec((q0[1].inv()*q1[1]).as_rotvec()*.5))
            body=middle.inv().apply(dp)
            out['world_delta_'+name]=(dp if name=='gt' else rotation@dp).tolist()
            out['body_forward_'+name+'_m']=float(body[0]);out['body_lateral_'+name+'_m']=float(body[1])
            out['yaw_delta_'+name+'_rad']=wrap(float(q1[1].as_euler('xyz')[2]-q0[1].as_euler('xyz')[2]))
        return out
    goal_reports=[]
    for gi,(a,b) in enumerate(intervals):
        bounds=sorted(set([a,b]+[t for t in cmd_times if a<t<b]))
        totals={m:empty() for m in ('walk','pure_turn','zero')};segments=[]
        lastmode=None
        for begin,end in zip(bounds,bounds[1:]):
            ci=bisect.bisect_right(cmd_times,begin)-1
            if ci<0:raise ValueError('Actual emit log does not cover goal interval')
            row=cmd[ci][1];m=mode(row['value']);metric=item(begin,end,row['value']);add(totals[m],metric)
            if m!=lastmode:
                segments.append(dict(mode=m,start_ns=begin,end_ns=end,start_status=status(begin),metric=empty(),
                                     emitted_adapter_states=set(),max_abs_command=[0.,0.,0.]))
                lastmode=m
            segment=segments[-1];segment['end_ns']=end;add(segment['metric'],metric)
            segment['emitted_adapter_states'].add(row['state'])
            for j,k in enumerate((0,1,5)):segment['max_abs_command'][j]=max(segment['max_abs_command'][j],abs(row['value'][k]))
        for segment in segments:
            segment['end_status']=status(segment['end_ns']);segment['metric']=finalize(segment['metric'])
            segment['emitted_adapter_states']=sorted(segment['emitted_adapter_states'])
        net=empty()
        for v in totals.values():add(net,v)
        goal_reports.append(dict(index=gi,goal_id=regions[gi]['goal_id'],interval_ns=[a,b],
            original_receipt_window=([regions[gi]['receipt']['start_stamp_ns'],regions[gi]['receipt']['stamp_ns']]
                if regions[gi].get('receipt') else None),
            receipt_completed=bool(regions[gi].get('receipt')),
            original_raw_inner_gt_outer=regions[gi]['passed'],modes={k:finalize(v) for k,v in totals.items()},
            net=finalize(net),segments=segments))
    selected=[x for x in original['aggregate_execution_safety_events'] if origin<=x['received_stamp_ns']<=terminal]
    feedback_mode_checks=[x['safety'].get('body_feedback_required') is True and isinstance(x['safety'].get('body_feedback',{}).get('enabled'),bool)
        and x['safety']['body_feedback']['enabled']==original['selected_body_feedback_enabled'] for x in selected]
    zero_edges=[x for x in commands if x.get('kind') in ['zero_edge','native_zero_ack','return_complete','failure'] and origin<=round(x.get('sim',0)*1e9)<=terminal]
    identities=[]
    for x in statuses:
        s=x['status'];cur=s.get('current_goal');ps=s.get('planning_start_state');
        if s.get('state')=='running' and cur and ps:
            identities.append(dict(received_stamp_ns=x['received_stamp_ns'],goal_id=cur['goal_id'],
              trajectory=s.get('accepted_trajectory_id'),reference=s.get('trajectory_reference_stamp'),
              requested_goal=ps.get('body_goal')))
    files=['first_four_result.json','joint_stop_adapter.jsonl','source_manifest.json','runtime_manifest.json','first4_manifest.json','process_cleanup.json','driver_exit.json']
    result=dict(run=str(run.resolve()),scope='read-only actual Adapter emit and bounded raw SLAM/independent GT motion decomposition; no actuation',
      original_result_passed=original['passed'],original_failure=original['failure'],original_checks=original['checks'],
      selected_body_feedback_enabled=original['selected_body_feedback_enabled'],
      original_region_evaluation=original['region_evaluation'],initial_fixed_SE3=se3,
      native_active_interval_ns=[origin,terminal],fourth_segment_interval_ns=[original['fourth_segment_previous_receipt_stamp_ns'],terminal],
      goals=goal_reports,active_max_imu_tilt=original['active_max_imu_tilt'],active_bridge_holds=original['active_bridge_holds'],
      body_feedback_mode=dict(samples=len(selected),all_actual_mode_matching=bool(selected) and all(feedback_mode_checks)),
      adapter_zero_handshake_edges=zero_edges,native_command_log=dict(rows=len(actual),unique_cached_clock_times=len(cmd),
        duplicate_cached_times=len(duplicates),same_time_value_changes=sum(x['value_changed'] for x in duplicates),
        sim_clock_reversal=reversals,semantics='actual Adapter emit log, cached float sim rounded to ns for join; not native message Header or JTC receipt'),
      fourth_pose_coverage=dict(slam=[slam.stamps[0],slam.stamps[-1]],truth=[truth.stamps[0],truth.stamps[-1]],
        note='Motion intervals require bounded endpoints. Final driver terminal may follow last raw SLAM; receipt ends earlier. No extrapolation.'),
      parse_errors=parse_errors,input_sha256={f:sha(run/f) for f in files if (run/f).exists()},
      script_sha256=sha(pathlib.Path(__file__)),limitations=[
       'GT only evaluates, never produces control or requests; SLAM-world displacements use the same initial SE3 from original component result.',
       'Commands are actual Adapter emitted Twist log, not independent CHAMP receive or motor torque evidence.',
       'Status samples are lower-rate cached observations and can predate the command interval. Preserve embedded steering stamps.',
       'This decomposes recorded component trajectories; A/B routes and starting yaw can differ due to actual physics.',
       'CDR fourth-segment rawIMU/measuredJoint/JTC/4feet coverage is separately checked by sole physical owner.',
       'Component success does not erase Full18 failure or demonstrate complete multi-floor success.'])
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),sha256=sha(out),passed=original['passed'],goals=[dict(index=g['index'],
      duration=g['net']['duration_s'],walk_s=g['modes']['walk']['duration_s'],turn_s=g['modes']['pure_turn']['duration_s'],
      zero_s=g['modes']['zero']['duration_s'],turn_planar_net=g['modes']['pure_turn']['world_delta_gt'],
      unpaired=g['net']['unpaired_intervals']) for g in goal_reports])))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=pathlib.Path);p.add_argument('output',type=pathlib.Path);a=p.parse_args();analyze(a.run,a.output)
