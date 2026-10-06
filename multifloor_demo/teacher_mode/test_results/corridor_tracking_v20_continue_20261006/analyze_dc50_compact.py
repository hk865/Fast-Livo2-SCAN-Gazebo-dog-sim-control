#!/usr/bin/env python3
"""Offline diagnosis from already captured compact; never reads large run logs."""
import gzip,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
NAME='20261006_140607_closed_loop_cascade_clock_hold_v20_original46_r1_dc50'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def rot(q):
 w,x,y,z=map(float,q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
def write(path,data):
 with path.open('x')as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False,default=plain);f.write('\n')
def main():
 cp=HERE/(NAME+'_COMPACT_DIAGNOSTIC.json.gz');d=json.loads(gzip.decompress(cp.read_bytes()))
 run=Path(d['run']);s=d['series'];m=d['metadata'];a=m['navigation_anchor.json'];profile=m['navigation_profile.json']
 report=Path(d['formal_report']['file']);assert sha(report)==d['formal_report']['sha256']
 assert sha(HERE/'audit_dc50_once.py')==d['script_sha256']
 native_source=run/'sources/simulation/teacher_actuator.cpp'
 assert sha(native_source)==m['source_manifest.json']['simulation/teacher_actuator.cpp']
 fence_source=run/'sources/navigation/corridor_tracking_v20/route_fence.py'
 assert sha(fence_source)==m['source_manifest.json']['navigation/corridor_tracking_v20/route_fence.py']
 sp=importlib.util.spec_from_file_location('dc50_archived_pure_fence',fence_source);fence=importlib.util.module_from_spec(sp);sp.loader.exec_module(fence)
 selections=[]
 for item in d['exact_V20_snapshot_selections']:
  path=Path(item['snapshot']);expected=run/'sources/navigation/corridor_tracking_v20'/Path(item['source']).name
  ok=path==expected and sha(path)==item['sha256'] and (item['required_sha256']is None or item['required_sha256']==item['sha256'])
  if not ok:raise ValueError('Actual exact archive selection differs')
  selections.append(dict(**item,independently_rechecked=True))
 provenance=dict(schema='dc50_actual_evaluation_invocation_provenance/v1',run=str(run),run_id=NAME,
  report=dict(file=str(report),sha256=sha(report)),actual_wrapper=dict(file=str(HERE/'audit_dc50_once.py'),sha256=d['script_sha256']),
  prepared_adapter=dict(file=str(HERE/'evaluate_full46_v20.py'),sha256=d['adapter_sha256']),
  compact=dict(file=str(cp),sha256=sha(cp)),actual_exact_archive_selections=selections,
  selector_validation='Every actual V20 full archive path, SHA and explicit control pin independently checked; no claim of synthetic selector unit tests.',
  collection_history=d['collection_history'],completed_attempt_full_data_passes=d['full_data_passes'],
  original_numerical_acceptance_unchanged=True,formal_report_not_modified=True,source_bindings=d['source_bindings'])
 write(HERE/'DC50_AUDIT_PROVENANCE.json',provenance)
 R=np.asarray(a['scene_axis_registration']['heading_receipt']['rotation_camera_init_from_world']);yawreg=a['scene_axis_registration']['heading_receipt']['yaw_camera_init_from_world']
 origin=np.asarray(a['origin']);spawn=np.asarray(profile['spawn'][:3]);route=np.asarray(a['registered_route_camera_init_xyz'])
 # Physical scene segment from original regions 9 -> 10, using recorded registration only.
 wa=R.T@(route[9]-origin)+spawn;wb=R.T@(route[10]-origin)+spawn
 direction=(wb-wa)[:2];direction/=np.linalg.norm(direction);normal=np.array([-direction[1],direction[0]])
 native=s['telemetry.jsonl'];nt=np.array([r['state_physics_world_time']for r in native]);offset=np.asarray(d['actuator_contract']['base_com_offset_body_m'])
 poses=s['navigation_slam_poses.jsonl'];by_stamp={r['stamp_ns']:r for r in poses}
 def pair_pose(pose):
  t=pose['stamp_ns']*1e-9;i=int(np.argmin(abs(nt-t)));n=native[i];gap=nt[i]-t
  if abs(gap)>.0100001:raise ValueError('Unbounded SLAM/native time pair')
  p=np.asarray(pose['position']);world=R.T@(p-origin)+spawn
  v=np.asarray(n['body_lin_vel'])-np.cross(n['body_ang_vel'],offset)
  rq=rot(n['quaternion_wxyz']);sq=rot([pose['quaternion'][3],*pose['quaternion'][:3]])
  f=fence.tube_evidence(p.tolist(),route.tolist())
  return dict(slam_stamp_s=t,native_physics_stamp_s=nt[i],pair_gap_s=gap,
   slam_camera_init_xyz=p,slam_scene_prior_world_xy=world[:2],native_body_origin_world_xyz=n['position'],
   recorded_route_fence=f,slam_signed_lateral_m=float(np.dot(world[:2]-wa[:2],normal)),
   native_signed_lateral_m=float(np.dot(np.asarray(n['position'])[:2]-wa[:2],normal)),
   native_rpy_rad=n['rpy'],slam_world_yaw_rad=wrap(math.atan2(sq[1,0],sq[0,0])-yawreg),
   native_minus_slam_yaw_rad=wrap(n['rpy'][2]-(math.atan2(sq[1,0],sq[0,0])-yawreg)),
   native_body_COM_velocity_mps=n['body_lin_vel'],native_body_origin_velocity_mps=v,
   native_world_origin_velocity_mps=rq@v,native_body_angular_velocity_radps=n['body_ang_vel'],
   command_body=n['command'],native_fault=n['fault'],native_contacts=n['contacts'],
   slam_source=pose['_source'],native_source=n['_source'])
 envs=[r for r in native if (r.get('envelope')or{}).get('registered_route_fence')]
 fail=next(r for r in envs if r['envelope']['registered_route_fence']['inside']is False)
 event=pair_pose(by_stamp[fail['envelope']['slam_stamp_ns']]);event['first_observed_failed_command_envelope']=fail['envelope'];event['native_command_receipt_source']=fail['_source']
 last=pair_pose(poses[-1]);peakrow=max((r for r in poses if r['stamp_ns']>=170e9),key=lambda r:fence.tube_evidence(r['position'],route.tolist())['horizontal_error_m']);peak=pair_pose(peakrow)
 activation=s['navigation_segment_activation.jsonl'][-1];start=activation['source_pose_stamp_ns']*1e-9;end=event['slam_stamp_s']
 pids=[r for r in s['navigation_pid_history.jsonl']if r['waypoint_index']==9 and r['control_pose_stamp_ns']*1e-9<=end]
 phase_events=[];drive_exits=[];prev=None;changes=[]
 for row in pids:
  gate=row['heading_gate_reference'];ph=gate['phase'];t=row['control_pose_stamp_ns']*1e-9
  if prev and row['path_id']!=prev['path_id']:
   changes.append(dict(t=t,old_path=prev['path_id'],new_path=row['path_id'],old_phase=prev['heading_gate_reference']['phase'],new_phase=ph,
    heading_error_rad=gate['steering'].get('error'),previous_heading_error_rad=prev['heading_gate_reference']['steering'].get('error'),source=row['_source']))
  if prev is None or ph!=prev['heading_gate_reference']['phase']:
   ev=dict(t=t,old_phase=prev['heading_gate_reference']['phase']if prev else None,new_phase=ph,trajectory_id=row['trajectory_id'],
    same_record_new_path=bool(prev and row['path_id']!=prev['path_id']),heading_error_rad=gate['steering'].get('error'),source=row['_source'])
   phase_events.append(ev)
   if prev and prev['heading_gate_reference']['phase']=='drive'and ph!='drive':drive_exits.append(ev)
  prev=row
 updated=[r for r in pids if r['cascade'].get('controller_updated')is True and r['cascade'].get('feedback_pose_stamp_ns')==r['control_pose_stamp_ns']]
 metrics=[]
 for r in updated:
  t=r['control_pose_stamp_ns']*1e-9;n=native[int(np.argmin(abs(nt-t)))];c=r['cascade'];reference=c.get('reference_COM_velocity_body');measured=c.get('measured_COM_velocity_body')
  if reference is not None and measured is not None and c.get('mode')=='drive'and t>=181.6:
   metrics.append(dict(t=t,command=r['command_after_slew'],reference=reference,slam_COM=measured,native_COM=n['body_lin_vel'],
    slam_native_COM_error=np.asarray(measured)[:2]-np.asarray(n['body_lin_vel'])[:2],native_reference_COM_error=np.asarray(n['body_lin_vel'])[:2]-np.asarray(reference)[:2],
    reference_world=c.get('reference_velocity_world'),path_cross=c.get('control_error_cross_m'),heading_error=c.get('error_yaw_rad')))
 def stats(field):
  v=np.asarray([r[field]for r in metrics]);return dict(mean=np.mean(v,axis=0),rms=np.sqrt(np.mean(v*v,axis=0)),min=np.min(v,axis=0),max=np.max(v,axis=0))
 path_geometry=[]
 for tid in (256,257,258,259):
  receipt=next(v for v in d['path_receipts'].values()if v['trajectory_id']==tid);file=run/receipt['array_file']
  with np.load(file,allow_pickle=False)as z:points=z['samples'].copy()
  if hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()!=receipt['source_samples_float64_sha256']:raise ValueError('Actual SCAN samples hash mismatch')
  allf=[fence.tube_evidence(x.tolist(),route.tolist())for x in points];out=[i for i,f in enumerate(allf)if not f['inside']]
  path_geometry.append(dict(trajectory_id=tid,path_id=receipt['path_id'],file=str(file),file_sha256=sha(file),samples_sha256=receipt['source_samples_float64_sha256'],
   samples=len(points),maximum_original_fence_horizontal_error_m=max(f['horizontal_error_m']for f in allf),outside_original_route_fence_count=len(out),
   first_outside_sample_index=out[0]if out else None,original_endpoint_camera_init_xyz=points[-1],original_start_camera_init_xyz=points[0],
   SCAN_collision_replay=False,route_fence_sample_test_only=True))
 diagnosis=dict(schema='dc50_region10_offline_failure_diagnosis/v1',run=str(run),run_id=NAME,status='failed',passed=False,
  formal_report=dict(file=str(report),sha256=sha(report)),compact_sha256=sha(cp),analysis_script_sha256=sha(__file__),
  scope='Original46 failed with nine original exploration arrivals; region10 unfinished; no final parking or full46 PASS.',
  first_fence_failure=event,post_stop_peak_SLAM_fence=peak,final_SLAM_snapshot=last,
  coordinate_contract=dict(rotation_camera_init_from_world=R,yaw_camera_init_from_world=yawreg,SLAM_anchor_origin=origin,
   world_route_segment_XY=[wa[:2],wb[:2]],native_position_semantics='Native base.WorldPose link origin, NOT COM; no COM position subtraction.',
   native_velocity_semantics='body_lin_vel is COM; v_origin_body = v_COM_body - omega_body cross measured base COM offset.',COM_offset_body_m=offset,
   native_state_time_semantics='PreUpdate state at world_sim_time minus 0.005 s, paired to SLAM original header; nearest sample <=10ms.',
   translation_semantics='Recorded SLAM origin plus fixed scene spawn XY prior. No truth-fitted or per-trajectory alignment. Native physical lateral is directly measured against original scene XY segment.',
   vertical_limitation='Scene route Z is support-relative; native body-origin altitude is not treated as the same Z datum. No direct 3D truth/SLAM position residual asserted.',
   native_source_file=str(native_source),native_source_sha256=sha(native_source),pure_fence_file=str(fence_source),pure_fence_sha256=sha(fence_source)),
  region10=dict(activation_pose_s=start,first_fence_pose_s=end,elapsed_until_fence_s=end-start,arrival=False,
   path_changes=len(changes),phase_events=phase_events,drive_exits=drive_exits,drive_exit_count=len(drive_exits),
   drive_exits_same_record_new_path=sum(e['same_record_new_path']for e in drive_exits),new_path_changes_heading_error_over_point2=sum(abs(e['heading_error_rad'])>.2 for e in changes if e['heading_error_rad']is not None),
   note='Counts are recorded PID observations; same-record association establishes sequence correlation, not a sole causal mechanism. Direct drive->align transitions retained, not hidden by counting only pre_turn.'),
  late_drive_181point6_to_stop=dict(samples=len(metrics),SLAM_minus_native_COM_xy_mps=stats('slam_native_COM_error'),native_minus_reference_COM_xy_mps=stats('native_reference_COM_error'),
   reference_world_velocity_mps=stats('reference_world'),control_path_cross_m=stats('path_cross'),heading_error_rad=stats('heading_error')),
  selected_actual_SCAN_path_samples=path_geometry,
  established=['The execution bridge latched the unchanged original SLAM route fence and requested zero velocity; full46 is failed, not a resource-aborted successful mission.',
   'Native body-origin lateral displacement also exceeds the original 0.45 m centerline radius at the first SLAM fence breach; this is not supported as a purely SLAM-only false alarm.',
   'Accepted paths, heading phase transitions and local tracking reference are observed; source-consistent heading does not imply the path stays within the original route tube.'],
  unverified=['No counterfactual controller replay or causal isolation; cannot attribute failure solely to replanning, velocity PI, Teacher tracking, estimator, or curvature limits.',
   '50 Hz native snapshots are not the complete 200 Hz actuator/physics trace.',
   'No SCAN collision guard re-evaluation, continuous-between-sample path safety proof, or foot support/corridor-control certification.'],
  navigation_ground_truth_used=False,truth_use='Offline diagnosis only',controller_replay_performed=False,source_bindings=d['source_bindings'])
 write(HERE/'DC50_REGION10_FAILURE_DIAGNOSIS.json',diagnosis)
 # Export only numeric rows in the requested window, omitting giant repeated receipts.
 slim=dict(schema='dc50_region10_numeric_window/v1',run_id=NAME,window_world_sim_s=[170,190],
  source_compact_sha256=sha(cp),source_bindings=d['source_bindings'],registration=diagnosis['coordinate_contract'],
  native=[{k:r[k]for k in ('state_physics_world_time','position','quaternion_wxyz','body_lin_vel','body_ang_vel','command','requested','fault','contacts','_source')}for r in native if 170<=r['state_physics_world_time']<=190],
  SLAM=[{k:r[k]for k in ('stamp_ns','position','quaternion','body_velocity','body_angular_velocity','_source')}for r in poses if 170e9<=r['stamp_ns']<=190e9],
  PID=[dict(t=r['control_pose_stamp_ns']*1e-9,trajectory_id=r['trajectory_id'],gate=r['heading_gate_reference']['phase'],mode=r['mode'],
    reference_heading=r['cascade'].get('reference_yaw_rad'),heading_error=r['cascade'].get('error_yaw_rad'),cross=r['cascade'].get('control_error_cross_m'),
    command=r['command_after_slew'],reference_world_velocity=r['cascade'].get('reference_velocity_world'),SLAM_COM_velocity=r['cascade'].get('measured_COM_velocity_body'),
    controller_updated=r['cascade'].get('controller_updated'),feedback_pose_stamp_ns=r['cascade'].get('feedback_pose_stamp_ns'),source=r['_source'])for r in pids if 170e9<=r['control_pose_stamp_ns']<=190e9],
  status=[{k:r[k]for k in ('ros_sim_time','state','waypoint_index','message','alignment_phase','accepted_trajectory_id','pose','command','_source')}for r in s['navigation_status.jsonl']if 170<=r['ros_sim_time']<=190],
  original_status='failed',navigation_ground_truth_used=False)
 output=HERE/'DC50_REGION10_NUMERIC_WINDOW.json.gz'
 with output.open('xb')as f:
  with gzip.GzipFile(fileobj=f,mode='wb',mtime=0)as z:z.write(json.dumps(slim,separators=(',',':'),default=plain,allow_nan=False).encode())
 print(json.dumps(dict(first_cross=event,late_drive=diagnosis['late_drive_181point6_to_stop'],drive_exit_count=len(drive_exits),drive_exit_newpath=diagnosis['region10']['drive_exits_same_record_new_path'],paths=path_geometry,window_bytes=output.stat().st_size),default=plain,indent=2))

if __name__=='__main__':main()
