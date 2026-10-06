#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,math,datetime
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
RUN=ROOT/'runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70'
lo,hi=16.345,21.345
rows=[];prefix=hashlib.sha256();length=0
with (RUN/'actuator.jsonl').open('rb') as f:
 for raw in f:
  prefix.update(raw);length+=len(raw)
  v=json.loads(raw)
  if v.get('kind')!='physics_step':continue
  world=v['t']-.005
  if world>hi+1e-9:break
  if world>=lo-1e-9:rows.append((world,math.hypot(*v['body_lin_vel_com'][:2]),v))
physics_prefix={'path':str(RUN/'actuator.jsonl'),'bytes_through_first_after_window':length,'sha256_of_prefix_only':prefix.hexdigest()}
peak=max(rows,key=lambda x:x[1]);bad=[v for v in rows if v[1]>.03]
actor=[];prefix=hashlib.sha256();length=0
with (RUN/'telemetry.jsonl').open('rb') as f:
 for raw in f:
  prefix.update(raw);length+=len(raw);v=json.loads(raw)
  world=v['state_physics_world_time']
  if world>hi+1e-9:break
  if world>=lo-1e-9:actor.append(v)
telemetry_prefix={'path':str(RUN/'telemetry.jsonl'),'bytes_through_first_after_window':length,'sha256_of_prefix_only':prefix.hexdigest()}
comparison=RUN/'dynamic_failure_diagnostics/parking_peak_diagnostic.json'
d=json.loads(comparison.read_text())
x={'asof_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'run_id':RUN.name,'window_world_s':[lo,hi],'native_time_interpretation':'PreUpdate t minus .005; no window reselect','rows':len(rows),'peak_world_s':peak[0],'peak_planar_speed_mps':peak[1],'frozen_limit_mps':.03,'count_above_limit':len(bad),'first_last_violation_world_s':[bad[0][0],bad[-1][0]],'actor_rows':len(actor),'requested_and_actor_commands_zero':all(v['command']==[0,0,0] and v['requested']==[0,0,0] for v in actor),'physics_mode0_fault0':all(v[2]['mode']==0 and v[2]['fault']==0 for v in rows),'independent_parking_result':'failed','matches_gazebo_diagnostic':len(rows)==d['sample_count'] and len(bad)==d['samples_above_frozen_limit'] and abs(peak[1]-d['peak_planar_speed_mps'])<1e-12,'inputs':[physics_prefix,telemetry_prefix,{'path':str(comparison),'sha256':hashlib.sha256(comparison.read_bytes()).hexdigest()}],'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'no_original_log_write':True,'no_ROS_sim_or_signal':True,'limits':'Only independently cross-checks fixed-window planar-speed peak from original native COM physics. Does not assert unique cause or alter V43 failed navigation/guard conclusions.'}
(OUT/'review.json').write_text(json.dumps(x,indent=2)+'\n')
print(json.dumps(x))
