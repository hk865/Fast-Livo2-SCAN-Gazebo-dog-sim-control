"""Stream a failed original46 region without changing run evidence or gates."""
import argparse,collections,hashlib,json,math
from pathlib import Path
import numpy as np

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def lines(path):
 offset=0
 with path.open('rb')as f:
  for number,line in enumerate(f,1):
   current=offset;offset+=len(line)
   if line.strip():yield json.loads(line),dict(source_file=str(path),source_offset=current,source_length=len(line),source_line=number,source_line_sha256=hashlib.sha256(line).hexdigest())
def stats(values):
 a=np.asarray(values,float)
 return None if not len(a)else dict(count=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))
def yaw(q):
 w,x,y,z=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def err(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))

def analyze(run,index,facts):
 bound=facts['physical_window']['last_native_command_read_monotonic_wall'];goalrows=[];events=[];firstfailed=None;lastkey=None;sourceages={k:[]for k in('pose_age','cloud_age','raw_imu_age')};statecounts=collections.Counter();phasedurations=collections.Counter();flags=collections.Counter();activation=None;previous=None;firstgoal=None
 for r,source in lines(run/'navigation_status.jsonl'):
  if r.get('monotonic_wall',float('inf'))>bound:continue
  if r.get('state')=='failed'and firstfailed is None:firstfailed=dict(sim_time=r['ros_sim_time'],message=r.get('message'),waypoint_index=r.get('waypoint_index'),goal_activated_ros_clock_ns=r.get('goal_activated_ros_clock_ns'),source=source)
  if r.get('waypoint_index')!=index:continue
  t=r['ros_sim_time'];tr=r.get('teacher_transition')or{};phase=tr.get('phase');activation=r['goal_activated_ros_clock_ns']/1e9
  firstgoal=firstgoal or r.get('current_goal');statecounts[r['state']]+=1
  reduced=dict(sim_time=t,state=r['state'],phase=phase,pose=r.get('pose'),command=r.get('command'),measured_stop=tr.get('measured_stop'),valid_stop_samples=tr.get('stop_valid_pose_samples'),stop_first_s=tr.get('stop_first_stamp_s'),stop_last_s=tr.get('stop_last_stamp_s'),locked_heading=r.get('locked_heading'),trajectory_id=r.get('accepted_trajectory_id'),replans=r.get('replans'),pose_age=r.get('pose_age'),cloud_age=r.get('cloud_age'),imu_age=r.get('raw_imu_age'),obstacle_hold=r.get('obstacle_hold'),tilt_hold=r.get('tilt_hold'),message=r.get('message'),source=source)
  if previous and previous['state']=='running':
   dt=t-previous['sim_time']
   if 0<=dt<=.2:phasedurations[previous['phase']]+=dt
  previous=reduced;goalrows.append(reduced)
  key=(r['state'],phase,bool(r.get('obstacle_hold')),bool(r.get('tilt_hold')))
  if key!=lastkey:events.append(reduced);lastkey=key
  for k in sourceages:
   if r['state']=='running'and isinstance(r.get(k),(int,float)):sourceages[k].append(r[k])
  for k in('obstacle_hold','tilt_hold','alignment_hold'):
   if r.get(k):flags[k]+=1
 pid=[];pathchanges=[];lastpath=None;reasons=collections.Counter();modes=collections.Counter();resetdiff=0;lastreset=None
 for r,source in lines(run/'navigation_pid_history.jsonl'):
  if r.get('compute_monotonic_wall',float('inf'))>bound or r.get('waypoint_index')!=index:continue
  c=r.get('cascade')or{};g=r.get('heading_gate_reference')or{};steering=g.get('steering')or{};feedback=r.get('feedback')or{};q=feedback.get('quaternion_wxyz');actualyaw=yaw(q)if q else None;reference=c.get('reference_yaw_rad');locked=g.get('locked_heading');path=r.get('trajectory_id')
  row=dict(sequence=r['sequence'],sim_time=r['compute_ros_clock_ns']/1e9,source_pose_time_s=r['source_pose_stamp_ns']/1e9,mode=c.get('mode'),phase=g.get('phase'),locked_heading=locked,current_SCAN_heading=steering.get('heading'),effective_reference=reference,reference_source=c.get('reference_yaw_source'),ungated_reference=c.get('ungated_reference_yaw_rad'),actual_yaw=actualyaw,external_locked_error=None if locked is None or actualyaw is None else err(locked,actualyaw),current_SCAN_error=None if steering.get('heading')is None or actualyaw is None else err(steering['heading'],actualyaw),reference_error=c.get('error_yaw_rad'),pose=r.get('control_pose'),actual_velocity=feedback.get('origin_velocity_body'),body_gyro=(r.get('imu')or{}).get('angular_velocity_body'),command=r.get('command_after_slew'),remaining_m=c.get('goal_distance_xy_m'),reason=c.get('reason'),updated=c.get('controller_updated'),trajectory_id=path,replan_count=c.get('replan_count'),reset_count=r.get('reset_count_after_publish'),source=source)
  pid.append(row);modes[row['mode']]+=1
  if row['reason']:reasons[row['reason']]+=1
  if path!=lastpath:pathchanges.append(row);lastpath=path
  if lastreset is not None and row['reset_count'] is not None:resetdiff+=max(0,row['reset_count']-lastreset)
  lastreset=row['reset_count']
 # Correlate each phase transition to the latest causal PID record, with exact
 # original serialized-line references. This is an observation, not replay.
 j=0
 for event in events:
  while j+1<len(pid)and pid[j+1]['sim_time']<=event['sim_time']:j+=1
  event['latest_causal_PID']=pid[j]if pid and pid[j]['sim_time']<=event['sim_time']else None
 selected=[];nextsample=-math.inf
 for row in pid:
  if row['sim_time']>=nextsample:selected.append(row);nextsample=row['sim_time']+1
 if pid and(selected[-1]['sequence']!=pid[-1]['sequence']):selected.append(pid[-1])
 return dict(schema='teacher_original46_failed_region_observation/v1',run=str(run),run_id=run.name,waypoint_index=index,goal_id='exploration:'+str(index),goal=firstgoal,activation_sim_s=activation,first_failed_navigation_status=firstfailed,observed_elapsed_until_failure_s=None if firstfailed is None else firstfailed['sim_time']-activation,physical_boundary_monotonic_wall=bound,goal_status_rows=len(goalrows),goal_state_counts=dict(statecounts),heading_phase_sim_duration_s=dict(phasedurations),phase_duration_scope='Only consecutive physical running status intervals <=0.2s; no gap extrapolation',flags=dict(flags),active_source_ages_s={k:stats(v)for k,v in sourceages.items()},phase_transitions=events,PID_modes=dict(modes),PID_protection_reasons=dict(reasons),reset_counter_increment=resetdiff,accepted_trajectory_changes=len(pathchanges),actual_path_acceptance_observations=pathchanges,one_second_PID_observations=selected,first_goal_pose=goalrows[0]['pose']if goalrows else None,last_goal_pose=goalrows[-1]['pose']if goalrows else None,full46_pass=False,full_PI_math_replayed=False,limits=['Streaming actual records with exact byte offsets and line hashes; selected observations cannot substitute for complete raw replay.','No causal attribution to a single policy/SLAM/SCAN tuning term is claimed solely from co-occurrence.','Physical runtime boundary excludes later cleanup source loss.'])

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--run',type=Path,required=True);a.add_argument('--facts',type=Path,required=True);a.add_argument('--index',type=int,default=8);a.add_argument('--output',type=Path,required=True);x=a.parse_args();out=analyze(x.run.resolve(),x.index,json.loads(x.facts.read_text()));out['evaluator_sha256']=digest(__file__);out['facts_binding_sha256']=digest(x.facts)
 with x.output.open('x')as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps({k:out[k]for k in('run_id','activation_sim_s','first_failed_navigation_status','observed_elapsed_until_failure_s','heading_phase_sim_duration_s','PID_modes','PID_protection_reasons','accepted_trajectory_changes','active_source_ages_s')},ensure_ascii=False))
