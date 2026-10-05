"""Limited frozen-source path projection/reset audit; not a full PI replay."""
import argparse,collections,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
from analyze_failed_region import lines,yaw,err,stats,digest

def main(run,output):
 profile=json.loads((run/'navigation_profile.json').read_text());snapshots=json.loads((run/'navigation_source_snapshots.json').read_text());source=next(Path(v['snapshot'])for k,v in snapshots.items()if Path(k).name=='cascade_core.py')
 if digest(source)!='ba139551805511fbf04b2fbc52fcc8feb9ea9bd60c0fa18251e7d7a37c93e9c9':raise ValueError('Not frozen revision03 core')
 spec=importlib.util.spec_from_file_location('limited_frozen_core',source);core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
 previous=None;events=[];dur=collections.Counter();transitions=collections.Counter();projection_errors=[];sources={str(source):digest(source)};rows_count=0;lastt=None
 for r,origin in lines(run/'navigation_pid_history.jsonl'):
  if r.get('waypoint_index')!=8:continue
  rows_count+=1;g=r['heading_gate_reference'];c=r['cascade'];s=g.get('steering')or{};q=r['feedback']['quaternion_wxyz'];actual_yaw=yaw(q);heading=s.get('heading');t=r['compute_ros_clock_ns']/1e9
  current=dict(sequence=r['sequence'],sim_time=t,phase=g['phase'],trajectory_id=r['trajectory_id'],heading=heading,actual_yaw=actual_yaw,actual_SCAN_yaw_error=err(heading,actual_yaw),command=r.get('command_after_slew'),pose=r['control_pose'],reason=c.get('reason'),source=origin)
  if previous:
   dt=t-previous['sim_time']
   if 0<=dt<=.2:dur[previous['phase']]+=dt
   if previous['phase']!=current['phase']:transitions[previous['phase']+'->'+current['phase']]+=1
   if previous['phase']=='drive'and current['phase']=='pre_turn':
    receipt=r['path_receipt'];p=run/r['trajectory_archive_file'];a=np.load(p,allow_pickle=False);points=a['samples'][receipt['source_sample_indices']]
    hash_ok=digest(p)==json.loads(p.with_suffix('.json').read_text())['array_sha256']and hashlib.sha256(a['samples'].astype('<f8').tobytes()).hexdigest()==receipt['source_samples_float64_sha256']and hashlib.sha256(points.astype('<f8').tobytes()).hexdigest()==receipt['points_float64_sha256']
    # For a newly accepted path, progress is reset to zero in actual production
    # set_path. We do not call update or replay PI/private controller state.
    fresh=core.Controller(profile['cascade'],c['fixed_goal']['position_world_xyz'],c['fixed_goal']['heading_rad'],c['fixed_goal']['goal_id']);fresh.set_path(points.tolist(),receipt['path_id'],receipt['stamp_ns'],receipt['received_wall_ns'],receipt['frame_id']);segment,point,tangent,normal,cross,grade,nearest_s=fresh._project(np.asarray(r['control_pose']))
    projected_heading=math.atan2(tangent[1],tangent[0]);observed=s['cascade_projection'];same=(segment==observed['segment']and np.array_equal(tangent,observed['tangent'])and abs(err(projected_heading,heading))<=1e-12)
    threshold=profile['teacher_transition']['drive_heading_max_rad'];threshold_trigger=abs(current['actual_SCAN_yaw_error'])>threshold;newpath=current['trajectory_id']!=previous['trajectory_id']
    if not(hash_ok and same and threshold_trigger and newpath):projection_errors.append(dict(sequence=r['sequence'],archive_valid=hash_ok,local_projection_equal=same,threshold_trigger=threshold_trigger,new_path=newpath))
    ahead=points[-1,:2]-points[0,:2];ahead_heading=math.atan2(ahead[1],ahead[0]);start=receipt['metadata'].get('start_state')or{}
    events.append(dict(previous=previous,current=current,path_archive=str(p),path_archive_sha256=digest(p),frozen_path_points_sha256=receipt['points_float64_sha256'],projection=dict(segment=segment,tangent=tangent.tolist(),heading=projected_heading,segment_horizontal_length_m=float(fresh.lengths[segment]),nearest_s_m=nearest_s,whole_path_horizontal_length_m=float(fresh.cumulative[-1]),whole_path_endpoint_heading=ahead_heading,local_vs_whole_heading_difference_rad=err(projected_heading,ahead_heading),archive_bytes_and_sample_indices_verified=hash_ok,projection_matches_actual_exactly=same),planner_start_state=start,drive_heading_max_rad=threshold,threshold_trigger=threshold_trigger,new_path=newpath))
  previous=current;lastt=t
 reseterrors=[e['current']['actual_SCAN_yaw_error']for e in events]
 out=dict(schema='teacher_original46_heading_reset_projection_audit/v1',run=str(run),run_id=run.name,limited_observation_checks_passed=bool(events)and not projection_errors,goal_id='exploration:8',region_PID_rows=rows_count,phase_transitions=dict(transitions),phase_durations_sim_s=dict(dur),duration_scope='Actual consecutive PID interval<=0.2s through last pre-failure PID; larger unknown intervals excluded',drive_to_pre_turn_count=len(events),new_path_reset_count=sum(e['new_path']for e in events),all_triggered_original_0_2rad_gate=all(e['threshold_trigger']for e in events),heading_error_on_reset_rad=stats([abs(x)for x in reseterrors]),all_selected_actual_archives_and_local_projection_verified=not projection_errors,errors=projection_errors,events=events,source_bindings=sources,evaluator_sha256=digest(__file__),full_PI_math_replayed=False,full46_pass=False,notes=['This independently verifies the actual new-path local tangent and original heading gate reset condition; it does not approve all trajectories or full PI arithmetic.','A new SCAN start segment can point differently from the remainder of an otherwise goal-directed path. Why the planner emits those initial segments remains a separate algorithm/input question.','No threshold, command limit, path geometry or runtime file was changed.'])
 with output.open('x')as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps({k:out[k]for k in('limited_observation_checks_passed','drive_to_pre_turn_count','new_path_reset_count','all_triggered_original_0_2rad_gate','phase_transitions','phase_durations_sim_s','heading_error_on_reset_rad','errors')}))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.run.resolve(),a.output)
