#!/usr/bin/env python3
"""Closed e8de one complete pass per selected log; never changes run or gates."""
from pathlib import Path
import collections,gzip,hashlib,io,json,os,re
import numpy as np
OUT=Path(__file__).resolve().parent
RUN=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_182824_closed_loop_cascade_clock_hold_terrain_event_V25_full46_r1_isolated_e8de')
RECEIPTS={};SUMMARY={};PATHS={}

def digest(b):return hashlib.sha256(b).hexdigest()
def dumps(d):return json.dumps(d,ensure_ascii=False,allow_nan=False,separators=(',',':'))
def keep(d,keys):return {k:d.get(k) for k in keys.split()}
def statid(p):
 s=p.stat();return [s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns]
def stream(name):
 p=RUN/name;before=statid(p);h=hashlib.sha256();offset=0;count=0
 with p.open('rb')as f:
  for number,raw in enumerate(f,1):
   h.update(raw);at=offset;offset+=len(raw)
   if not raw.strip():continue
   count+=1;yield json.loads(raw),dict(source_line=number,source_offset=at,source_bytes=len(raw),source_line_sha256=digest(raw))
 assert before==statid(p),'Input changed during read: '+name
 RECEIPTS[name]=dict(path=str(p),sha256=h.hexdigest(),bytes=offset,rows=count,full_sequential_passes=1,identity_before_after=before)
 print('DONE',name,offset,count,flush=True)
def small(p):
 p=Path(p);before=statid(p);raw=p.read_bytes();assert before==statid(p)
 RECEIPTS[str(p.relative_to(RUN))]=dict(path=str(p),sha256=digest(raw),bytes=len(raw),full_reads=1)
 return json.loads(raw)
def target(row):return row.get('waypoint_index')in(10,11)
def save(name,data):
 with (OUT/name).open('x')as f:f.write(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
assert (RUN/'run_result.json').is_file() and (RUN/'worker_result.json').is_file()
assert not (OUT/'E8DE_STREAM_RECEIPT.json').exists()

keys='sequence cascade_sequence control_pose_stamp_ns source_pose_stamp_ns paired_imu_stamp_ns control_stamp_ns compute_ros_clock_ns compute_monotonic_wall publish_ros_clock_ns request_id waypoint_index trajectory_id trajectory_archive_file mode checked_target goal control_pose control_quaternion feedback imu ack desired_body_command prepared_after_slew_command command_after_slew state obstacle_hold alignment_hold stopped reset_count_after_publish queue_error cascade heading_gate_reference'
modes=collections.Counter();reasons=collections.Counter();pid_n=0
with gzip.open(OUT/'e8de_regions11_12_pid.jsonl.gz','xt')as out:
 for row,loc in stream('navigation_pid_history.jsonl'):
  if not target(row):continue
  x=keep(row,keys);x.update(loc);out.write(dumps(x)+'\n');pid_n+=1
  c=row.get('cascade')or{};modes[(row['waypoint_index'],c.get('mode'))]+=1;reasons[(row['waypoint_index'],c.get('reason'))]+=1
  receipt=row.get('path_receipt')or{}
  if receipt.get('path_id') and receipt['path_id']not in PATHS:PATHS[receipt['path_id']]=dict(source=loc,receipt=receipt)
SUMMARY['pid']={'selected_rows':pid_n,'modes':[[*k,v]for k,v in modes.items()],'reasons':[[*k,v]for k,v in reasons.items()]}

statuskeys='request_id state goal_activated_ros_clock_ns segment_start_pose_stamp_ns current_goal region_arrival_evidence waypoint_index total message obstacle_hold obstacle_stops obstacle_resumes min_obstacle_clearance replans reference_requests degenerate_splines last_spline_rejected trajectory_reference_stamp trajectory_association_rejected last_trajectory_association_rejected planning_start_state steering accepted_trajectory_id obstacle_resume_pending aligned_obstacle_resumes pose_age cloud_age actual_cloud_message_stamp_ns obstacle_guard_cloud_stamp_ns pose command counts last_rejected tilt_hold tilt_stops max_tilt_rad tilt_source raw_imu_age raw_imu_stamp_age raw_imu_tilt_rad raw_imu_max_tilt_rad execution_bridge_safety alignment_hold filtered_yaw_rate alignment_phase locked_heading tracking_pose heading_deviation_since cascade_parking teacher_transition monotonic_wall ros_sim_time'
statecounts=collections.Counter();status_n=0;arrival_by_id={};last={}
with gzip.open(OUT/'e8de_regions11_12_status.jsonl.gz','xt')as out:
 for row,loc in stream('navigation_status.jsonl'):
  for receipt in row.get('region_arrivals',[]):arrival_by_id[receipt['goal_id']]=receipt
  if not target(row):continue
  x=keep(row,statuskeys);x.update(loc);out.write(dumps(x)+'\n');status_n+=1
  statecounts[(row['waypoint_index'],row.get('message'))]+=1;last[row['waypoint_index']]=x
SUMMARY['status']={'selected_rows':status_n,'messages':[[*k,v]for k,v in statecounts.items()],'arrivals':arrival_by_id,'last_by_waypoint':last}
save('E8DE_PID_PATH_RECEIPTS.json',PATHS)

metadata=[]
for p in sorted((RUN/'navigation_trajectories').glob('*.json')):
 d=small(p)
 # Keep the entire original small metadata, including every control point.
 metadata.append(dict(file=str(p.relative_to(RUN)),sha256=RECEIPTS[str(p.relative_to(RUN))]['sha256'],data=d))
save('E8DE_ALL_TRAJECTORY_METADATA.json',metadata)
geometry=[]
for p in sorted((RUN/'navigation_trajectories').glob('*.npz'))[-3:]:
 before=statid(p);raw=p.read_bytes();assert before==statid(p)
 arrays={}
 with np.load(io.BytesIO(raw),allow_pickle=False)as z:
  for key in z.files:
   a=z[key];arrays[key]=dict(dtype=str(a.dtype),shape=list(a.shape),array_bytes_sha256=digest(a.tobytes()),values=a.tolist())
 geometry.append(dict(path=str(p),sha256=digest(raw),bytes=len(raw),arrays=arrays))
 RECEIPTS[str(p.relative_to(RUN))]=dict(path=str(p),sha256=digest(raw),bytes=len(raw),full_reads=1,all_npz_members_preserved=True)
save('E8DE_LAST3_SCAN_GEOMETRY.json',geometry)

patterns=re.compile(r'Target.*occupied|Local target|Replan.*fail|Obstacle discovered|Close to goal|Unable.*refresh|emergency|Emergency|EMERGENCY|stop|Stop|collision|Collision|fail|Fail',re.I)
p=RUN/'navigation_stack.log';before=statid(p);h=hashlib.sha256();offset=0;matches=[];last_lines=collections.deque(maxlen=20)
with p.open('rb')as f:
 for number,raw in enumerate(f,1):
  h.update(raw);at=offset;offset+=len(raw);text=raw.decode('utf-8','replace').rstrip();last_lines.append(dict(source_line=number,source_offset=at,text=text))
  if patterns.search(text):matches.append(dict(source_line=number,source_offset=at,source_bytes=len(raw),source_line_sha256=digest(raw),text=text))
assert before==statid(p)
RECEIPTS[p.name]=dict(path=str(p),sha256=h.hexdigest(),bytes=offset,rows=number,full_sequential_passes=1)
with gzip.open(OUT/'e8de_SCAN_diagnostic_lines.json.gz','xt')as f:f.write(dumps(dict(matches=matches,last_lines=list(last_lines))))
print('DONE',p.name,offset,len(matches),flush=True)

# Preserve small fixed initial frame evidence before telemetry joins. No fixed frame is invented.
frame={n:small(RUN/n)for n in ['navigation_scene_axis_registration.json','navigation_anchor.json','mission46_initialization_receipt.json','mission46_initialization_evidence.json','sensor_contract.json','navigation_profile.json']}
save('E8DE_FRAME_INPUTS.json',frame)
nativekeys='sim_time world_sim_time state_physics_world_time command requested measured body_lin_vel body_ang_vel position quaternion_wxyz rpy contacts state fault body_clearance command_expired command_age_sim_s command_reason actor_inferred_this_frame outer_navigation_ground_truth_used'
native_n=0
with gzip.open(OUT/'e8de_native50Hz_compact.jsonl.gz','xt')as out:
 for row,loc in stream('telemetry.jsonl'):
  x=keep(row,nativekeys);x.update(loc);out.write(dumps(x)+'\n');native_n+=1
SUMMARY['native50Hz']={'rows':native_n,'position_semantics':'native base_link world origin, not COM position','time_semantics':'state_physics_world_time is actual represented physics time; world_sim_time request time is 5ms later','not_a_full200Hz_trace':True}
save('E8DE_EXTRACTION_SUMMARY.json',SUMMARY)
save('E8DE_STREAM_RECEIPT.json',dict(schema='e8de_onepass_diagnostic_extraction/v1',run=str(RUN),sources=RECEIPTS,source_script_sha256=digest(Path(__file__).read_bytes()),raw_mutated=False,acceptance_changed=False,all_after_diagnostics_use_only_compact=True,read_scope_note='Each selected JSONL/log had one complete sequential hash/extraction pass. Before extraction, one initial schema line was previewed for PID/status/cascade path/replan files; no prior complete pass. Cascade path streams otherwise not scanned. No full200Hz trace or cloud arrays read.'))
print('FINISHED',OUT,flush=True)
