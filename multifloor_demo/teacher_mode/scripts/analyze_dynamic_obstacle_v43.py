#!/usr/bin/env python3
"""Prospective V4.3 turn20 dynamic audit, exact per-guard replay and both regions required; never launches ROS or Gazebo.

Old flat runs remain inapplicable to this new fixture. Existing frozen analyses,
runtime files, original logs and global acceptance are never overwritten.
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
EXPERIMENT='finite_flat_dynamic_stop_resume_v1'
NAV_EXPERIMENT='finite_flat_dynamic_stop_resume_turn20_v1'
BASE_SHA='5ed1af467538af3eb96239d456d2e5f63bbf0df74a3a4cee908c635d917effc0'
CANDIDATE=ROOT/'test_results/dynamic_independent_candidate3_v43_20261004'

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def read(path):return json.loads(Path(path).read_text())
def rows(path):
    if not Path(path).exists():return []
    with Path(path).open()as f:return [json.loads(x)for x in f if x.strip()]
def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,dict):return {k:clean(x)for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [clean(x)for x in v]
    if isinstance(v,float)and not math.isfinite(v):return None
    return v
def check(v,**data):return {'status':'unverified'if v is None else'passed'if bool(v)else'failed','passed':None if v is None else bool(v),**data}
def verdict(values):return 'failed'if'failed'in values else'unverified'if'unverified'in values else'passed'
def load(path,prefix):
    s=importlib.util.spec_from_file_location(prefix+sha(path)[:8],path);m=importlib.util.module_from_spec(s);sys.modules[s.name]=m;s.loader.exec_module(m);return m
def reference_path(run,path):
    p=Path(path)
    try:
        rel=p.relative_to(ROOT);arch=run/'sources'/rel
        if arch.is_file():return arch
    except ValueError:pass
    return p
def first_span(mask,t,minimum,gap=.005001):
    first=None
    for i,good in enumerate(mask):
        if not good:first=None;continue
        if first is None or(i and t[i]-t[i-1]>gap):first=i
        if t[i]-t[first]>=minimum-1e-9:return first,i
    return None
def quat_rotation(q):
    w,x,y,z=q
    return np.asarray([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def evaluate(run,write=True):
    run=Path(run).resolve();checks={};metrics={};errors=[];input_hashes={};artifacts={}
    if write and(run/'summary_dynamic_obstacle_independent.json').exists():raise ValueError('Refusing to overwrite prior frozen dynamic receipt')
    originals={n:(run/n).read_bytes()for n in ['summary.json','summary_navigation_independent.json']if(run/n).is_file()}
    if not(run/'dynamic_scope.json').exists():
        return {'schema':1,'status':'inapplicable','scope':EXPERIMENT,'reason':'No new dynamic scope/fixture; old flat route is not a dynamic obstacle test','run_dir':str(run),'checks':{},'errors':[],'analyzer_sha256':sha(__file__),'global_acceptance_overwritten':False}
    protocol=read(run/'dynamic_protocol.json');profile=read(run/'navigation_profile.json');scope=read(run/'dynamic_scope.json');parent=read(run/'navigation_scope.json');asset=read(run/'asset_manifest.json')
    frozen=read(CANDIDATE/'freeze_manifest.json')
    runtime_freeze_path=CANDIDATE/'runtime_v43_frozen.json'
    if sha(runtime_freeze_path)!=frozen['runtime_freeze_sha256']:raise ValueError('Exact prospective V43 runtime freeze changed')
    runtime_freeze=read(runtime_freeze_path)
    runtime_source_errors=[];runtime_source_checks={}
    for name,expected in runtime_freeze['source_hashes'].items():
        source_path=reference_path(run,str(ROOT/name));actual=sha(source_path)if source_path.is_file()else None
        runtime_source_checks[name]={'actual_path':str(source_path),'expected_sha256':expected,'actual_sha256':actual}
        if actual!=expected:runtime_source_errors.append(name)
    checks['exact_prospective_v43_runtime_sources']=check(not runtime_source_errors,runtime_freeze_sha256=frozen['runtime_freeze_sha256'],checks=runtime_source_checks,mismatches=runtime_source_errors,authorized_navigation_experiment=NAV_EXPERIMENT)
    cleanup_path=ROOT/'scripts/audit_dynamic_runtime.py'
    if sha(cleanup_path)!=frozen['cleanup_analyzer_sha256']:raise ValueError('Strict cleanup evaluator hash changed')
    cleanup=load(cleanup_path,'dynamic_v43_cleanup_').evaluate(run,write=False)
    checks['all_owned_and_started_child_clean_exit']=check(True if cleanup['status']=='passed'else False if cleanup['status']=='failed'else None,strict_process_checks=cleanup['checks'],no_signal_exception_or_added_grace=True,worker_completed_without_fault=cleanup['levels']['worker_completed_without_fault'])
    exact_protocol=sha(run/'dynamic_protocol.json')==frozen['protocol_sha256']and profile==read(CANDIDATE/'profile_frozen.json')
    frozen_basis={row['run_id']:row['input_sha256']['summary_navigation_independent.json']for row in read(ROOT/'test_results/navigation_v4_independent_campaign/navigation_v4_independent_campaign.json')['runs']}
    basis=scope.get('finite_navigation_basis',[]);basis_errors=[]
    if len(basis)!=3 or{r.get('run_id')for r in basis}!=set(frozen_basis):basis_errors.append('Not the three preserved actual V4 runs')
    for r in basis:
        p=Path(r['path'])
        if not p.is_file()or sha(p)!=r.get('sha256')or sha(p)!=frozen_basis.get(r.get('run_id')):basis_errors.append(str(p)+': changed basis hash');continue
        s=read(p)
        if s.get('status')!='passed'or s.get('levels',{}).get('finite_flat_navigation')!='passed'or s.get('errors'):basis_errors.append(str(p)+': no actual finite pass')
    refs={**parent.get('references',{}),**scope.get('references',{})};ref_errors=[]
    for p,h in refs.items():
        rp=reference_path(run,p)
        if not rp.is_file()or sha(rp)!=h:ref_errors.append(p)
    exclusions={'original_origin','overhead_decks','ramps','low_steps','full_46_region_mission','real_robot','unbounded_dynamic_obstacle_mission'}
    mandatory={str((run/n).resolve())for n in ['world.sdf','asset_manifest.json','navigation_scope.json','dynamic_protocol.json']}
    scope_ok=exact_protocol and scope.get('schema')=='teacher_dynamic_scope/v1'and scope.get('experiment')==EXPERIMENT and scope.get('status')=='experimental_unverified'and scope.get('allowed')is True and scope.get('navigation_ground_truth_used')is False and scope.get('run_dir')==str(run)and parent.get('schema')=='teacher_navigation_scope/v1'and parent.get('experiment')==NAV_EXPERIMENT and scope.get('navigation_experiment')==NAV_EXPERIMENT and scope.get('navigation_max_yaw_rate_radps')==.2 and parent.get('allowed')is True and parent.get('run_dir')==str(run)and parent.get('profile')==profile and parent.get('checkpoint_sha256')=='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'and parent.get('status')=='experimental_unverified'and parent.get('navigation_is_verified')is False and parent.get('navigation_ground_truth_used')is False and parent.get('review_basis')=='explicit_user_continuation_and_root_scope_authorization'and exclusions<=set(profile.get('excluded',[]))and'dynamic_obstacle_success_claim'not in parent.get('excluded_scenarios',[])and mandatory<=set(refs)and not basis_errors and not ref_errors
    levels=scope.get('global_levels_preserved',{});scope_ok&=levels==parent.get('global_levels_preserved')and levels.get('sim2sim')=='failed'and levels.get('navigation')=='unverified'and levels.get('real_robot')=='unverified'
    checks['strict_explicit_new_scope_and_preserved_basis']=check(scope_ok,protocol_sha256=sha(run/'dynamic_protocol.json'),expected_frozen_protocol_sha256=frozen['protocol_sha256'],protocol_and_profile_exactly_frozen=exact_protocol,reference_hash_errors=ref_errors,basis_errors=basis_errors,global_levels_preserved=levels,excluded_scenarios=profile.get('excluded'),old_scope_not_generalized=True)
    old=ET.parse(run/'dynamic_original_world.sdf').getroot();new=ET.parse(run/'world.sdf').getroot();models=new.findall("world/model[@name='moving_obstacle']");unique=len(models)==1
    if unique:
        model=models[0];actualpose=np.asarray([float(x)for x in model.findtext('pose').split()]);size=np.asarray([float(x)for x in model.findtext('link/collision/geometry/box/size').split()]);originalpose=old.find("world/model[@name='moving_obstacle']/pose").text;model.find('pose').text=originalpose
        unchanged=ET.tostring(old)==ET.tostring(new);model.find('pose').text=' '.join(str(x)for x in actualpose)
    else:unchanged=False;actualpose=[];size=[]
    fixture=asset.get('dynamic_fixture',{})
    physics_ok=unique and unchanged and model.findtext('static')=='true'and np.array_equal(actualpose[:3],protocol['initial_position_world'])and np.array_equal(size,protocol['box_size_m'])and fixture.get('changed_fields')==['world/moving_obstacle/pose']and fixture.get('policy_terrain_scan_excludes_dynamic_entity')is True and fixture.get('navigation_ground_truth_used')is False and asset.get('world_sha256')==sha(run/'world.sdf')and fixture.get('original_world_sha256')==sha(run/'dynamic_original_world.sdf')and asset.get('spawn')==[6,-.7,.4,0]and asset.get('exclusive_writer')=='teacher_sim::TeacherActuator'
    checks['only_existing_physical_box_initial_pose_changed']=check(physics_ok,whole_xml_equal_after_single_pose_normalization=unchanged,actual_initial_pose=actualpose,actual_box_size=size,physics_and_robot_unchanged=True if physics_ok else False)
    basepath=ROOT/'scripts/analyze_navigation.py'
    if sha(basepath)!=BASE_SHA:raise ValueError('Frozen common evidence analyzer changed')
    base=load(basepath,'dynamic_common_');common=base.evaluate(run,write_outputs=False)
    required_common=['cpu_teacher_and_exclusive_actuation','teacher_continuous_actor_and_command_input','actual_runtime_continuity','native_physical_safety','teacher_received_and_executed_motion','source_integrity','actual_sensor_slam_publisher_chain','measured_slam_pose_stream','native_scan_payload_available','actual_registered_cloud_inputs','healthy_command_source_ages','causal_worker_command_source','nonzero_commands_have_accepted_slam_source','immutable_slam_relative_route_anchor']
    common_errors=common.get('errors',[])
    checks['actual_actor_slam_scan_source_and_safety']=check(not common_errors and all(common['checks'].get(k,{}).get('status')=='passed'for k in required_common),evidence={k:common['checks'].get(k)for k in required_common},common_errors=common_errors,old_flat_scope_check_inapplicable='Only the old flat experiment-name check is replaced by strict_explicit_new_scope_and_preserved_basis; all physical/source/SLAM/SCAN checks retained')
    checks['original_raw_slam_two_regions_and_final_stop']=check(common['checks'].get('both_regions_confirmed_by_raw_slam_dwell',{}).get('status')=='passed'and common['checks'].get('post_arrival_actual_teacher_stop',{}).get('status')=='passed',region_check=common['checks'].get('both_regions_confirmed_by_raw_slam_dwell'),arrival_evidence=common['metrics'].get('arrivals'),final_stop=common['metrics'].get('stop'))
    metrics['ordinary_diagnostics']={'levels':common.get('levels'),'errors':common_errors,'source_hashes':common.get('source_hashes'),'motion':common['checks'].get('teacher_received_and_executed_motion'),'truth_scope':'Post-run immutable alignment only; never navigation or fixture trigger'}
    motions=rows(run/'obstacle_motion_history.jsonl');poses=rows(run/'obstacle_actual_pose.jsonl');observed=[p for p in poses if p.get('status')=='actual_observed'];phase=[p for p in motions if p.get('kind')=='phase_change'];byphase={r['phase']:r for r in phase}
    failures=[r for r in motions if r.get('failure')or r.get('kind')=='service_reply'and r.get('success')is not True];commands=[r for r in motions if r.get('kind')=='service_request'];othercommands=[r for r in commands if r.get('model')!='moving_obstacle'or r.get('service')!=protocol['service']]
    actualids={p.get('entity_id')for p in observed};ot=np.asarray([p['stamp_ns']/1e9 for p in observed]);op=np.asarray([p['position']for p in observed]);tolerance=protocol['actual_pose_position_tolerance_m']
    blocking=byphase.get('blocking');leaving=byphase.get('leaving');clear=byphase.get('clear');entering=byphase.get('entering')
    bm=(ot>=blocking['sim_time']-1e-9)&(ot<leaving['sim_time']+1e-9)if blocking and leaving else np.zeros(len(ot),bool)
    block_seconds=leaving['sim_time']-blocking['sim_time']if blocking and leaving else None
    actualok=len(observed)>1 and len(actualids)==1 and np.all(np.diff(ot)>0)and np.max(np.diff(ot))<=.300000001 and not othercommands and not failures and all(r.get('navigation_input')is False for r in observed)
    actualok&=blocking is not None and leaving is not None and clear is not None and block_seconds>=protocol['blocking_duration_sim_s']-1e-8 and bm.any()and np.max(np.linalg.norm(op[bm]-np.asarray(protocol['blocked_position_world']),axis=1))<=tolerance and math.dist(clear.get('actual_model_pose',{}).get('position',[math.inf]*3),protocol['initial_position_world'])<=tolerance
    checks['actual_box_entry_block_and_withdrawal']=check(actualok,actual_model_ids=list(actualids),actual_pose_samples=len(observed),phase_times_s={k:r['sim_time']for k,r in byphase.items()},actual_observed_block_duration_s=block_seconds,requested_service_only_for_allowed_box=not othercommands,fixture_failures=failures,actual_model_pose_navigation_input=False)
    telemetry=rows(run/'telemetry.jsonl');native=[p for p in rows(run/'actuator.jsonl')if p.get('kind')=='physics_step'];slam=rows(run/'navigation_slam_poses.jsonl');status=rows(run/'navigation_status.jsonl');cloudhistory=rows(run/'navigation_cloud_history.jsonl');anchor=read(run/'navigation_anchor.json')
    st=np.asarray([p['stamp_ns']for p in slam],dtype=np.int64);sp=np.asarray([p['position']for p in slam]);rule=protocol['trigger']
    trigmask=(st>=int(entering.get('trigger_first_slam_stamp_s',0)*1e9)-1)&(st<=int(entering.get('trigger_last_slam_stamp_s',0)*1e9)+1)if entering else np.zeros(len(slam),bool)
    trigidx=np.flatnonzero(trigmask);along=(sp-np.asarray(anchor['origin']))[:,:2]@np.asarray([math.cos(anchor['yaw']),math.sin(anchor['yaw'])]);sv=np.asarray([p['body_velocity'][0]for p in slam])
    triggerok=entering is not None and len(trigidx)>=rule['min_distinct_slam_poses']and(st[trigidx[-1]]-st[trigidx[0]])/1e9>=rule['continuous_sim_s']-1e-8 and np.max(np.diff(st[trigidx]))/1e9<=.200000001 and np.min(sv[trigidx])>=rule['slam_body_forward_min_mps']and np.min(along[trigidx])>=rule['slam_along_min_m']-1e-9 and np.max(along[trigidx])<=rule['slam_along_max_m']+1e-9
    checks['entry_trigger_was_original_measured_slam_motion']=check(triggerok,raw_slam_samples=len(trigidx),first_last_stamp_ns=[int(st[trigidx[0]]),int(st[trigidx[-1]])]if len(trigidx)else None,actual_slam_forward_velocity_min_mps=float(sv[trigidx].min())if len(trigidx)else None,along_min_max_m=[float(along[trigidx].min()),float(along[trigidx].max())]if len(trigidx)else None,truth_trigger_used=False)
    corepath=ROOT.parent/'navigation/control_core.py';expected=parent.get('references',{}).get(str(corepath.resolve()))
    if expected!=sha(corepath):raise ValueError('Native obstacle geometry code hash not verified by run scope')
    core=load(corepath,'dynamic_core_');cloudidx={p['stamp_ns']:p for p in cloudhistory};poseidx={p['stamp_ns']:p for p in slam}
    event_results=[];native_event_errors=[];guard_times=[];native_route=None
    alignment=common['metrics'].get('independent_truth_error',{});a=np.asarray(alignment.get('alignment_world_to_slam_rotation'));at=np.asarray(alignment.get('alignment_translation'))
    for path in sorted((run/'navigation_events').glob('*.json')):
        document=read(path);npz=path.parent/document['cloud_file'];record={'file':str(path.relative_to(run)),'guard_compute_world_s':document.get('guard_compute_stamp'),'zero_command_stamp':document.get('zero_command_stamp')}
        if not npz.is_file()or sha(npz)!=document.get('cloud_sha256'):native_event_errors.append(path.name+': array hash mismatch');continue
        with np.load(npz)as saved:data={k:saved[k]for k in saved.files}
        result=core.steering_obstacle_ahead(data['cloud'],data['pose'],data['checked_target'],data['steering_direction'],data['route']);declared=document['guard_result'];cs=document['cloud_input']['message_stamp_ns'];ch=cloudidx.get(cs)
        bufferhash=hashlib.sha256(np.asarray(data['cloud'],dtype='<f8').tobytes()).hexdigest();samebuffer=ch is not None and ch['filtered_xyz_float64_sha256']==bufferhash
        equal=result[0]==declared[0]and result[2]==declared[2]and((result[1]is None and declared[1]is None)or result[1]is not None and declared[1]is not None and abs(result[1]-declared[1])<1e-9)
        ei=np.searchsorted(ot,cs/1e9,side='right')-1;boxpoints=None
        if ei>=0 and cs/1e9-ot[ei]<=.300000001 and a.shape==(3,3)and at.shape==(3,):
            world=(data['cloud']-at)@a;box=observed[ei];qxyzw=box['quaternion_xyzw'];br=quat_rotation(np.asarray([qxyzw[3],*qxyzw[:3]]));local=(world-np.asarray(box['position']))@br;half=np.asarray(protocol['box_size_m'])/2
            inside=np.all(abs(local)<=half+.07,axis=1);side=np.min(abs(abs(local[:,:2])-half[:2]),axis=1)<=.07;above=(world[:,2]>.15)&(world[:,2]<1.05);boxpoints=int(np.sum(inside&side&above))
        good=result[0]is True and equal and samebuffer and boxpoints is not None and boxpoints>=3 and document.get('protect_result')is False and np.max(abs(np.asarray(document.get('zero_command',[1,1,1]))))<1e-8
        if good:guard_times.append(float(document['obstacle_edge_stamp']));native_route=data['route'].copy()
        record.update(replayed_result=result,recorded_result=declared,exact_filtered_cloud_sha_verified=samebuffer,replay_matches=equal,actual_observed_box_surface_returns=boxpoints,matching_tolerance_m=.07,passed=good)
        event_results.append(record)
    checks['real_cloud_native_guard_detected_actual_box_and_zeroed']=check(bool(guard_times)and not native_event_errors,events=event_results,event_errors=native_event_errors,native_guard_minimum_points=3,box_geometry_match='Post-run fixed alignment; actual Pose_V sampled causally at cloud stamp. ≥3 vertical-side returns within7cm; never injected into navigation.')
    nt=np.asarray([r['t']-.005 for r in native]);npos=np.asarray([r['position']for r in native]);nv=np.asarray([r['body_lin_vel_com']for r in native]);nw=np.asarray([r['body_ang_vel']for r in native]);nq=np.asarray([r['quaternion_wxyz']for r in native]);contacts=np.asarray([r['contacts']for r in native]);yaw=np.unwrap(np.arctan2(2*(nq[:,0]*nq[:,3]+nq[:,1]*nq[:,2]),1-2*(nq[:,2]**2+nq[:,3]**2)))
    pt=np.asarray([r['world_sim_time']for r in telemetry]);cmd=np.asarray([r['command']for r in telemetry]);requested=np.asarray([r['requested']for r in telemetry]);park=protocol['parking'];parking=None
    if blocking and leaving and guard_times:
        policy_for_native=np.searchsorted(pt,nt,side='right')-1;safeidx=np.maximum(policy_for_native,0);zero=(np.max(abs(cmd[safeidx]),axis=1)<1e-8)&(np.max(abs(requested[safeidx]),axis=1)<1e-8)
        eligible=(nt>=max(blocking['sim_time'],min(guard_times)))&(nt<=leaving['sim_time'])&zero&(np.linalg.norm(nv[:,:2],axis=1)<=park['planar_speed_max_mps'])&(abs(nw[:,2])<=park['yaw_rate_max_radps'])
        span=first_span(eligible,nt,park['continuous_sim_s'])
        if span:
            begin=float(nt[span[1]]);end=begin+park['hold_duration_sim_s'];nm=(nt>=begin-1e-10)&(nt<=end+1e-10);tm=(pt>=begin-1e-10)&(pt<=end+1e-10)
            if end<=leaving['sim_time'] and nm.sum()>=int(park['hold_duration_sim_s']/.005)-1 and tm.sum()>=int(park['hold_duration_sim_s']/.02)-1:
                drift=float(np.linalg.norm(npos[nm,:2]-npos[nm,:2][0],axis=1).max());yd=float(abs(yaw[nm]-yaw[nm][0]).max());maxspeed=float(np.linalg.norm(nv[nm,:2],axis=1).max());maxwz=float(abs(nw[nm,2]).max());actor=all(r.get('actor_inferred_this_frame')is True and r.get('state')not in {'support_hold','support_capture'}for r,m in zip(telemetry,tm)if m)
                parking=check(drift<=park['xy_drift_max_m']and yd<=park['yaw_drift_max_rad']and maxspeed<=park['planar_speed_max_mps']and maxwz<=park['yaw_rate_max_radps']and actor and np.max(abs(cmd[tm]))<1e-8 and np.max(abs(requested[tm]))<1e-8 and not(contacts[nm,0]>0).any()and not(contacts[nm]<0).any(),world_window_s=[begin,end],selection='First1s continuous actual-stop span while real box blocked; next fixed5s window. Never choose the best late window.',native_samples=int(nm.sum()),actor_samples=int(tm.sum()),continuous_cpu_teacher=actor,body_contact_samples=int((contacts[nm,0]>0).sum()),missing_contact_samples=int((contacts[nm]<0).sum()),xy_drift_m=drift,yaw_drift_rad=yd,maximum_planar_speed_mps=maxspeed,maximum_abs_wz_radps=maxwz,limits=park)
                artifacts['parking_arrays']={'native_world_time_s':nt[nm],'position':npos[nm],'body_com_velocity':nv[nm],'body_angular_velocity':nw[nm],'quaternion_wxyz':nq[nm],'policy_world_time_s':pt[tm],'actor_command':cmd[tm],'requested':requested[tm]}
    checks['actual_continuous_teacher_parking_while_box_blocked']=parking or check(None,reason='No complete predetermined1s stop confirmation plus5s parking before physical withdrawal; event coverage unverified, never claim stand-only pass')
    sensor=rows(run/'dynamic_sensor_evidence/sensor_inputs.jsonl');inventory_path=run/'dynamic_sensor_evidence_manifest.json';inventory=read(inventory_path)if inventory_path.exists()else{};sensorfiles=inventory.get('files',{});sensor_errors=[]
    # Inventory schema is validated without treating absent/failed recorder evidence as success.
    for name,h in sensorfiles.items():
        p=run/name
        if not p.is_file()or sha(p)!=h:sensor_errors.append(name)
    cloud_replays=[];clear_times=[];clear_good=[]
    goal=np.asarray(read(run/'navigation_request.json')['goals'][0]['center']);route=native_route if native_route is not None else np.asarray([anchor['origin'],goal])
    for sr in sensor:
        if sr.get('source')!='cloud'or not sr.get('xyz_file'):continue
        p=run/'dynamic_sensor_evidence'/sr['xyz_file'];cs=sr['stamp_ns'];ch=cloudidx.get(cs)
        if not p.is_file()or ch is None:continue
        ps=poseidx.get(ch['nearest_slam_pose_stamp_ns'])
        if ps is None:continue
        with np.load(p)as data:xyz=data['xyz']
        filtered,count=core.remove_go2_self_returns(xyz,np.asarray(ps['position']),core.rotation_xyzw(ps['quaternion']));h=hashlib.sha256(np.asarray(filtered,dtype='<f8').tobytes()).hexdigest();exact=h==ch['filtered_xyz_float64_sha256']
        result=core.obstacle_ahead(filtered,np.asarray(ps['position']),goal,route)
        # Corroborate against the actual recorded steering corridor, not only a
        # hypothetical straight vector toward the goal. Control pose is selected
        # by steering's original odom stamp; never the simulator robot pose.
        snapshots=[r for r in status if r.get('actual_cloud_message_stamp_ns')==cs and r.get('waypoint_index')==0 and r.get('steering')and r.get('ros_sim_time',0)>=cs/1e9 and r.get('ros_sim_time',0)-cs/1e9<.3]
        actual_result=None;used_status=None
        if snapshots:
            used_status=min(snapshots,key=lambda r:r['ros_sim_time']);steer=used_status['steering'];cp=poseidx.get(int(round(steer['odom_stamp']*1e9)))
            if cp is not None:actual_result=core.steering_obstacle_ahead(filtered,np.asarray(cp['position']),np.asarray(steer['target']),np.asarray(steer['direction']),route)
        cr={'stamp_ns':cs,'phase':sr.get('fixture_phase'),'source':sr['xyz_file'],'actual_consumer_filtered_buffer_exact':exact,'replayed_goal_corridor':result,'replayed_recorded_steering_corridor':actual_result,'recorded_status_world_s':used_status.get('ros_sim_time')if used_status else None}
        cloud_replays.append(cr)
        # Sensor corridor clearance may start during leaving. Model returning to
        # its initial pose is a separate required final fixture condition above.
        if leaving and cs/1e9>=leaving['sim_time']and actual_result is not None:
            clear_times.append(cs/1e9);clear_good.append(exact and not result[0]and not actual_result[0])
    # Old 4Hz status pairing remains a diagnostic only. Prospective acceptance
    # replays each actual native guard input with original integer timestamps.
    guard_helper_path=ROOT/'scripts/replay_dynamic_native_guards.py'
    if sha(guard_helper_path)!=frozen['guard_replay_analyzer_sha256']:raise ValueError('Prospective exact guard evaluator hash changed')
    guard_helper=load(guard_helper_path,'dynamic_v43_guard_')
    guard_evidence=guard_helper.evaluate(run,leaving['sim_time']if leaving else math.inf,protocol['clear_corridor_sim_s'],maximum_gap_s=.300000001)
    guard_value=False if guard_evidence['errors']else True if guard_evidence['actual_guard_records']and guard_evidence['matched_original_xyz_guards']else None
    checks['actual_integer_stamped_native_guard_inputs_and_replay']=check(guard_value,evidence=guard_evidence,source='Every actual native guard invocation, exact integer SLAM pose/cloud stamps and independently filtered original registered XYZ; no status interpolation or truth geometry')
    clear_complete=guard_evidence['strict_clear_confirmation_world_s'];clear_verified=clear_complete is not None
    runtime_resumed=any(r.get('obstacle_resumes',0)>0 for r in status)
    checks['actual_registered_cloud_corridor_clear_after_withdrawal']=check(clear_verified and runtime_resumed if leaving and guard_evidence['actual_guard_records']else None,actual_model_withdrawal_complete_world_s=clear['sim_time']if clear else None,sensor_clear_can_start_during_leaving=True,first_continuous_clear_confirmation_world_s=clear_complete,required_clear_sim_s=protocol['clear_corridor_sim_s'],maximum_actual_compute_gap_s=.300000001,longest_corroborated_clear_s=guard_evidence['longest_corroborated_clear_s'],runtime_obstacle_resume_recorded=runtime_resumed,exact_consumer_buffer_replays=cloud_replays,raw_xyz_inventory_errors=sensor_errors,source='Original registered PointCloud2 XYZ, actual per-guard exact integer control/filter poses, native checked target/steering/route and both actual corridor results independently recomputed',model_pose_or_mover_phase_not_used_by_navigation=True,limitations='Missing/mismatched actual guard or XYZ, source age outside original0.3s bounds, recorded blockage or compute gap over0.3s resets independent clear continuity. No4Hz status coverage fallback or simulator geometry is used.')
    trajectories=[read(p)for p in sorted((run/'navigation_trajectories').glob('*.json'))];newplans=[p for p in trajectories if clear_complete is not None and p.get('waypoint_index')==0 and p.get('reference_stamp')is not None and(p['reference_stamp'][0]+p['reference_stamp'][1]/1e9)>=clear_complete]
    checks['new_checked_scan_after_actual_clear']=check(bool(newplans)if clear_complete is not None else None,clear_confirmation_world_s=clear_complete,new_trajectory_ids=[p['trajectory_id']for p in newplans],required_by_frozen_protocol=protocol['requires_new_scan_after_clear'])
    recovery=None
    if newplans:
        plan_stamp=min(p['reference_stamp'][0]+p['reference_stamp'][1]/1e9 for p in newplans);accepted=[i for i,r in enumerate(telemetry)if pt[i]>=plan_stamp and r['navigation_envelope'].get('read_status')=='accepted'and np.linalg.norm(requested[i,:2])>.015]
        if accepted:
            starttime=float(pt[accepted[0]]);nm=nt>=starttime;progress=float(np.linalg.norm(npos[nm,:2]-npos[nm,:2][0],axis=1).max());nativeactive=float((np.linalg.norm(nv[nm,:2],axis=1)>.05).sum()*.005)
            recovery=check(progress>=protocol['post_resume_progress_min_m']and len(accepted)*.02>=.5 and nativeactive>=.5,first_accepted_motion_world_s=starttime,actual_physical_xy_progress_m=progress,accepted_motion_seconds=len(accepted)*.02,native_actual_speed_above_point05_seconds=nativeactive,minimum_physical_progress_m=protocol['post_resume_progress_min_m'])
    checks['teacher_actual_motion_resumed_on_new_scan']=recovery or check(None,reason='No actual accepted Teacher motion after the required clear confirmation/newSCAN; recovery unverified')
    imagefiles={k:[]for k in ['vehicle','overview']}
    for sr in sensor:
        if sr.get('source')in imagefiles and sr.get('snapshot_file'):
            p=run/'dynamic_sensor_evidence'/sr['snapshot_file']
            if p.is_file():imagefiles[sr['source']].append(sr['snapshot_file'])
    checks['actual_dual_rgb_and_pointcloud_evidence_saved']=check(bool(sensorfiles)and not sensor_errors and not inventory.get('queue_error')and all(imagefiles.values())and bool(cloud_replays),image_files=imagefiles,inventory_records=len(sensorfiles),inventory_errors=sensor_errors,recorder_queue_error=inventory.get('queue_error'),registered_xyz_replays=len(cloud_replays),images_are_actual_ros_payloads=True)
    required=[v['status']for k,v in checks.items()];summary=clean({'schema':1,'scope':NAV_EXPERIMENT,'physical_fixture_scope':EXPERIMENT,'status':verdict(required),'checks':checks,'levels':{'new_scope':checks['strict_explicit_new_scope_and_preserved_basis']['status'],'actual_box_fixture':checks['actual_box_entry_block_and_withdrawal']['status'],'real_cloud_obstacle_stop':checks['real_cloud_native_guard_detected_actual_box_and_zeroed']['status'],'native_teacher_parking':checks['actual_continuous_teacher_parking_while_box_blocked']['status'],'actual_clear_and_newscan':verdict([checks['actual_registered_cloud_corridor_clear_after_withdrawal']['status'],checks['new_checked_scan_after_actual_clear']['status']]),'physical_motion_recovery':checks['teacher_actual_motion_resumed_on_new_scan']['status'],'raw_slam_two_regions_and_final_stop':checks['original_raw_slam_two_regions_and_final_stop']['status'],'complete_dynamic_run':verdict(required),'finite_roundtrip':checks['original_raw_slam_two_regions_and_final_stop']['status'],'dynamic_functional':verdict([v['status']for k,v in checks.items()if k!='original_raw_slam_two_regions_and_final_stop']),'all_owned_process_exit_zero':cleanup['levels']['all_owned_process_exit_zero'],'all_started_navigation_children_clean_exit':cleanup['levels']['all_started_navigation_children_clean_exit'],'exact_v43_runtime_sources':checks['exact_prospective_v43_runtime_sources']['status'],'global_sim2sim':'failed','global_navigation':'unverified','multifloor':'unverified','real_robot':'unverified'},'metrics':metrics,'errors':errors,'run_dir':str(run),'analyzer_sha256':sha(__file__),'common_frozen_analyzer_sha256':BASE_SHA,'runtime_freeze_sha256':frozen['runtime_freeze_sha256'],'guard_replay_analyzer_sha256':sha(guard_helper_path),'all_regions_required_for_main_verdict':True,'criteria_change':'Prospective turn20 exact source/profile only; same physical parking, clear1s/maxgap0.3s, route90s/control0.17m/0.6s dwell; original two SLAM regions and finalstop now explicitly required by main complete verdict','global_acceptance_overwritten':False,'truth_scope':'Gazebo robot/box poses used only in post-run diagnostics/physical safety; localization/commands/regions use original SLAM/cloud. Never navigation feedback.'})
    for name in ['world.sdf','dynamic_original_world.sdf','asset_manifest.json','dynamic_protocol.json','dynamic_scope.json','navigation_scope.json','navigation_profile.json','source_manifest.json','telemetry.jsonl','actuator.jsonl','navigation_slam_poses.jsonl','navigation_status.jsonl','navigation_cloud_history.jsonl','navigation_command_history.jsonl','obstacle_motion_history.jsonl','obstacle_actual_pose.jsonl','dynamic_sensor_evidence/sensor_inputs.jsonl','dynamic_sensor_evidence_manifest.json','navigation_guard_history.jsonl','navigation_guard_writer_receipt.json','runtime_manifest.json','worker_result.json','navigation_stack.log']:
        if(run/name).is_file():input_hashes[name]=sha(run/name)
    summary['input_sha256']=input_hashes;summary['original_summary_bytes_preserved']=all((run/n).read_bytes()==data for n,data in originals.items())
    if write:
        if artifacts.get('parking_arrays'):
            p=run/'dynamic_obstacle_native_parking.npz';np.savez_compressed(p,**artifacts['parking_arrays']);summary['parking_npz_sha256']=sha(p)
        (run/'summary_dynamic_obstacle_independent.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--read-only',action='store_true');a=p.parse_args()
    try:s=evaluate(a.run,write=not a.read_only)
    except (ValueError,KeyError,IndexError,FileNotFoundError,TypeError)as e:
        s={'schema':1,'status':'unverified','scope':EXPERIMENT,'checks':{},'errors':[str(e)],'analyzer_sha256':sha(__file__),'global_acceptance_overwritten':False}
        if not a.read_only:(a.run/'summary_dynamic_obstacle_independent.json').write_text(json.dumps(s,indent=2)+'\n')
    print(json.dumps({'status':s['status'],'errors':s['errors'],'failed':[k for k,v in s['checks'].items()if v['status']=='failed'],'unverified':[k for k,v in s['checks'].items()if v['status']=='unverified']},ensure_ascii=False))
