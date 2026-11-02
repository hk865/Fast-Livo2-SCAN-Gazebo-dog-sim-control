from pathlib import Path
import json,math,csv
import numpy as np
R=Path(__file__).resolve().parents[2];P=R;out={}
for run in sorted((P/'runs').iterdir()):
 if not(run/'experiment_exit.json').exists()or not(run/'truth_navigation_status.json').exists():continue
 if json.loads((run/'experiment_setup.json').read_text()).get('navigation_mode')=='slam':continue
 if (run/'truth_navigation_result.json').exists():
  cached=json.loads((run/'truth_navigation_result.json').read_text());setup0=json.loads((run/'experiment_setup.json').read_text());status0=json.loads((run/'truth_navigation_status.json').read_text());cached['adapter_version']=setup0.get('adapter_version','pre_world_twist_fix_v1');cached['SCAN_candidate_rejections']=status0['candidate_rejections'];out[run.name]=cached;continue
 status=json.loads((run/'truth_navigation_status.json').read_text());setup=json.loads((run/'experiment_setup.json').read_text());route=json.loads((run/'PCT_route.json').read_text());rows=[json.loads(l)for l in (run/'navigation_truth_poses.jsonl').open()];xyz=np.array([a['position']for a in rows]);q=np.array([a['quaternion_wxyz']for a in rows]);stamps=np.array([a['stamp_ns']for a in rows]);goal=np.array(route['world_body_points'][-1]);origin=np.array(setup['navigation_world_origin']);park=json.loads((run/'parking_10Hz_result.json').read_text())if(run/'parking_10Hz_result.json').exists()else None
 if park:
  last=park['window']['samples'][-1];final=np.array(last['position'])+origin;t0=park['T0_ns'];end_ns=park['window']['end_ns']
 else:final=xyz[-1];t0=end_ns=None
 physicsN=0;faults=set();torquebad=0;targetoutside=0;maxqd=0;maxtau=0;bodyepisodes=[];topheight={n:-1e9 for n in ['FR','FL','RR','RL']};firstbands={n:{}for n in topheight};last_contact_time=-100.;t0exchange=None;footmaxdepth=0;bodyN=0
 contactstream=(run/'foot_support_events.csv').open('w');writer=csv.writer(contactstream);writer.writerow(['sim_s','foot','world_x','world_y','world_z','normal_z','depth_m'])
 for line in (run/'actuator_physics.jsonl').open():
  a=json.loads(line)
  if a['kind']=='policy_exchange':
   if t0 is not None and abs(a['t']*1e9-t0)<1:t0exchange=a
   continue
  if a['kind']!='physics_step':continue
  physicsN+=1
  if a['fault']:faults.add(a['fault'])
  tau=np.array(a['tau']);lo=np.array(a['tau_lower']);hi=np.array(a['tau_upper']);torquebad+=int(np.count_nonzero((tau<lo-1e-8)|(tau>hi+1e-8)));targetoutside+=sum(a['target_outside_hard_limits']);maxqd=max(maxqd,max(abs(v)for v in a['qd']));maxtau=max(maxtau,max(abs(v)for v in a['tau']));t=a['t']
  if a['contacts'][0]>0:
   bodyN+=1;cs=[v for v in a['contact_pairs']if v['group']==0]
   if t-last_contact_time>.011:bodyepisodes.append({'start_s':t,'end_s':t,'first_body_position_world':a['position'],'collisions':sorted(set(v['a']for v in cs)),'max_depth_m':0.})
   else:bodyepisodes[-1]['end_s']=t
   bodyepisodes[-1]['max_depth_m']=max(bodyepisodes[-1]['max_depth_m'],max((v['max_depth_m']for v in cs),default=0.));last_contact_time=t
  for pair in a['contact_pairs']:
   g=pair['group']
   if not 1<=g<=4:continue
   name=['FR','FL','RR','RL'][g-1]
   for i,pt in enumerate(pair['positions_world']):
    normal=pair['normals_world'][i]if i<len(pair['normals_world'])else[0,0,0];depth=pair['depths_m'][i]if i<len(pair['depths_m'])else 0.;footmaxdepth=max(footmaxdepth,depth)
    if normal[2]<.95:continue
    topheight[name]=max(topheight[name],pt[2])
    for z in [1.45,2.96,4.41]:
     if pt[2]>=z:firstbands[name].setdefault(str(z),t)
    if physicsN%4==0:writer.writerow([t,name,*pt,normal[2],depth])
 contactstream.close()
 elapsed=(stamps[-1]-stamps[0])/1e9;planlength=route['length_m'];arclength=float(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1).sum());curves=[json.loads(l)for l in (run/'accepted_SCAN_curves.jsonl').open()];stop_math=None;nativeproof=None
 if park:
  w=park['window'];stop_math={k:w[k]for k in ['passed','complete','data_available','max_xy_from_anchor_m','max_abs_yaw_from_anchor_rad','missing','motion_failure']};stop_math.update(samples=len(w['samples']),T0_s=t0/1e9,T1_s=end_ns/1e9,capture_samples=len(park['capture']['samples']));nativeproof={'T0_simulation_ns':t0,'actual_actor_input_receipt':park['zero_ack'],'native_actuator_successful_IPC_exchange_at_same_sim_clock':t0exchange,'binding_scope':'Original native64/response16 RPC is synchronous: simulation remains at the same clock until actuator receives the sole actor response. Actor receipt is pre-send; native exchange completes it. Independently crosschecked offline; not relabeled as the legacy next-request ACK implementation.'};(run/'parking_native_exchange_validation.json').write_text(json.dumps(nativeproof,indent=2))
 worker=json.loads((run/'worker_result.json').read_text())if(run/'worker_result.json').exists()else None
 result={'adapter_version':setup.get('adapter_version','pre_world_twist_fix_v1'),'SCAN_candidate_rejections':status['candidate_rejections'],'case':setup['case'],'state':status['state'],'failure':status['failure'],'navigation_ground_truth_used':True,'PCT_planned_3D_length_m':planlength,'actual_XY_arclength_m':arclength,'duration_source_sim_s':elapsed,'goal_world_body':goal.tolist(),'final_world_body_at_T1_or_last':final.tolist(),'terminal_xy_error_m':float(np.linalg.norm(final[:2]-goal[:2])),'terminal_z_error_m':float(abs(final[2]-goal[2])),'final_yaw_error_rad':status['goal_yaw_error_rad'],'parking':stop_math,'parking_native_exchange_proof':nativeproof,'actual_foot_highest_horizontal_contact_z_world_m':topheight,'first_actual_foot_contact_above_world_bands':firstbands,'body_nonfoot_contact_duration_s':bodyN*.005,'body_contact_episodes':bodyepisodes,'maximum_actual_foot_penetration_m':footmaxdepth,'physics_samples':physicsN,'actuator_fault_codes':sorted(faults),'torque_envelope_violations':torquebad,'max_abs_joint_speed_rad_s':maxqd,'max_abs_joint_torque_Nm':maxtau,'raw_unclipped_target_outside_SDF_joint_limits_samples':targetoutside,'Student_worker':worker,'accepted_SCAN_curves':len(curves),'SCAN_curve_sample_Z_nav_minmax':None if not curves else[min(v['sample_z_minmax'][0]for v in curves),max(v['sample_z_minmax'][1]for v in curves)],'SCAN_source_scope':'Native SCAN cubic curves, PCT 3D distance<=.5m gate plus native occupancy/optimization; not time-scripted forward commands','controller_scope':'Independent truth-mode spatial lookahead and terminal pose adapter; original Student/velocity PID and original10Hz parking math reused. This is not a PASS for the unmodified legacy control/admission controller.','parking_frame_count_and_limits_unchanged':True,'formal_main_or_weights_changed':False,'execution_pass':bool(status['state']=='succeeded'and stop_math and stop_math['passed']and t0exchange and t0exchange['response_mode']==0 and not faults and(worker is None or worker.get('fault')is None)and float(np.linalg.norm(final[:2]-goal[:2]))<=.25 and abs(final[2]-goal[2])<=.1)}
 (run/'truth_navigation_result.json').write_text(json.dumps(result,indent=2));out[run.name]=result
(P/'runs/building_results.json').write_text(json.dumps(out,indent=2));print(json.dumps({n:{k:r[k]for k in ['case','state','execution_pass','PCT_planned_3D_length_m','terminal_xy_error_m','terminal_z_error_m','body_nonfoot_contact_duration_s','actual_foot_highest_horizontal_contact_z_world_m','accepted_SCAN_curves']}for n,r in out.items()},indent=2))
