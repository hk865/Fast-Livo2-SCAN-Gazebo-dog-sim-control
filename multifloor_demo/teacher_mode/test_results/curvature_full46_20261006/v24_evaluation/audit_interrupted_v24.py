#!/usr/bin/env python3
"""Targeted partial arrival audit; never substitutes for the frozen 22 gates."""
import argparse,bisect,hashlib,json,math,os
from pathlib import Path
import evaluate_full46_v24 as adapter

HERE=Path(__file__).resolve().parent
ADAPTER_SHA='91198ade1b51972caab0677100e054ad022b7c70bb2288ecd4d31665ad679658'
sha=adapter.sha

def read(path):return json.loads(Path(path).read_text())

def rows_once(path,receipts):
    path=Path(path);before=path.stat();h=hashlib.sha256();count=0
    with path.open('rb')as stream:
        while True:
            offset=stream.tell();raw=stream.readline()
            if not raw:break
            h.update(raw)
            if not raw.strip():continue
            count+=1
            yield json.loads(raw),dict(source_file=str(path),source_offset=offset,source_line=count,raw_line_sha256=hashlib.sha256(raw).hexdigest())
    after=path.stat()
    if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):
        raise ValueError('Source changed during targeted audit: '+str(path))
    receipts[str(path)]=dict(sha256=h.hexdigest(),bytes=before.st_size,rows=count,read_passes=1,unchanged_during_read=True)

def verify_dwell(audit,raw,receipt,poses,stamps,request_id,definition_hash):
    goal=audit.parse_goal(raw);start=receipt['start_stamp_ns'];end=receipt['stamp_ns'];activation=receipt['goal_activated_ros_clock_ns']
    window=stamps[bisect.bisect_left(stamps,start):bisect.bisect_right(stamps,end)]
    gaps=[b-a for a,b in zip(window,window[1:])]
    valid_stamps=type(start)is int and type(end)is int and type(activation)is int
    terminal=poses.get(end)
    checks=dict(identity=receipt.get('request_id')==request_id and receipt.get('goal_id')==goal.goal_id and receipt.get('goals_definition_sha256')==definition_hash,
        original_arrival_definition=receipt.get('arrival_definition')==goal.definition()['arrival'] and receipt.get('control_arrival_definition')==goal.control_arrival_definition(),
        measured_arrival_flags=receipt.get('protected')is False and receipt.get('region_inside')is True and receipt.get('control_region_inside')is True and receipt.get('reason')=='arrived',
        endpoint_sources=valid_stamps and len(window)>=2 and window[0]==start and window[-1]==end,
        original_dwell=valid_stamps and receipt.get('dwell_ns')==end-start and end-start>=round(goal.dwell_sim_s*1e9),
        source_gap=bool(gaps)and max(gaps)<=200_000_000,
        original_3D_control_region=bool(window)and all(audit.contains_control(goal,poses[t]['position'])for t in window),
        exact_terminal_raw_pose=terminal is not None and terminal['position']==receipt.get('raw_position'),
        original_deadline=valid_stamps and 0<=start-activation<=end-activation<=round(goal.timeout_sim_s*1e9))
    axes=goal.control_goal().axes
    local=[]
    for t in window:
        delta=[a-b for a,b in zip(poses[t]['position'],goal.center)]
        local.append([sum(a*b for a,b in zip(axis,delta))for axis in axes])
    return dict(goal_id=goal.goal_id,ordinal=int(receipt['waypoint_index'])+1,passed=all(checks.values()),checks=checks,
        start_stamp_ns=start,end_stamp_ns=end,activation_ns=activation,dwell_s=(end-start)/1e9,
        activation_to_arrival_s=(end-activation)/1e9,original_timeout_s=goal.timeout_sim_s,
        actual_source_count=len(window),maximum_source_gap_ns=max(gaps)if gaps else None,
        control_geometry=goal.control_arrival_definition(),window_max_abs_local_xyz=[max(abs(x[i])for x in local)for i in range(3)]if local else None,
        terminal_raw_position=receipt.get('raw_position'),terminal_center_error_xyz=[a-b for a,b in zip(receipt['raw_position'],goal.center)],terminal_center_error_m=math.dist(receipt['raw_position'],goal.center),window_source_rows=[dict(stamp_ns=t,position=poses[t]['position'],**poses[t]['provenance'])for t in window])

def evaluate(run):
    if sha(Path(adapter.__file__))!=ADAPTER_SHA:raise ValueError('Frozen V24 full46 version adapter changed')
    audit=adapter.load_adapter();run=Path(run).resolve()
    if (run/'worker_result.json').exists():raise ValueError('This targeted audit is specifically for the missing-worker interrupted run')
    names=('navigation_scope.json','navigation_source_snapshots.json','source_manifest.json','navigation_profile.json','navigation_anchor.json','navigation_scene_axis_registration.json','runtime_manifest.json','run_result.json','mission46_status.json','navigation_status.json','mission46_requests/exploration_1.json')
    small={name:read(run/name)for name in names}
    profile=small['navigation_profile.json'];scope=small['navigation_scope.json'];runtime=small['runtime_manifest.json'];result=small['run_result.json'];anchor=small['navigation_anchor.json']
    if (runtime.get('run')!=str(run) or result.get('run')!=str(run) or result.get('status')!='failed' or
            not str(runtime.get('error','')).startswith('InterruptedError:') or
            result.get('runtime_manifest_sha256')!=sha(run/'runtime_manifest.json') or
            runtime.get('source_manifest_sha256')!=sha(run/'source_manifest.json') or
            runtime.get('scope_sha256')!=sha(run/'navigation_scope.json') or scope['profile']!=profile or runtime['profile']!=profile):
        raise ValueError('Interrupted runtime identity/source bindings differ')
    from full46_launch_contract import validate_full46
    from geometry_archive import verify_geometry_archive
    from navigation.goal_regions import definitions_sha256
    validate_full46(profile,adapter.V20)
    geometry=verify_geometry_archive(run,scope['references'],small['navigation_source_snapshots.json'],small['source_manifest.json'],adapter.V20)
    repair=adapter.repair_source_closure(audit,run,scope)
    def exact_source(source):
        source=Path(source).resolve();key=str(source);pin=sha(source);row=small['navigation_source_snapshots.json'].get(key)
        if not isinstance(row,dict)or row.get('sha256')!=pin:raise ValueError('Pure original geometry source missing: '+key)
        target=Path(row['snapshot']).resolve();relative=source.relative_to(adapter.TOP)if source.is_relative_to(adapter.TOP)else Path('external')/(pin[:16]+'_'+source.name)
        canonical=run/'sources'/relative
        if target!=canonical.resolve()or canonical.is_symlink()or not target.is_relative_to(run/'sources')or sha(target)!=pin or scope['references'].get(key)!=pin or scope['references'].get(str(target))!=pin or small['source_manifest.json'].get(key)!=pin or small['source_manifest.json'].get(str(relative))!=pin:raise ValueError('Pure source closure differs: '+key)
        return dict(source=key,snapshot=str(target),sha256=pin)
    sources=[exact_source(p)for p in (adapter.V20/'mission46_profile.py',adapter.TOP.parent/'mission/state_machine.py',adapter.TOP.parent/'mission/route_regions.py',adapter.TOP.parent/'navigation/goal_regions.py')]
    if anchor['scene_axis_registration']!=small['navigation_scene_axis_registration.json']or anchor['scene_axis_registration_file_sha256']!=sha(run/'navigation_scene_axis_registration.json')or anchor.get('ground_truth_navigation_used')is not False:raise ValueError('Original scene-axis registration differs')
    mission=audit.Mission();mission.scenario=profile['original_scenario'];mission.run_id=run.name;mission.origin=anchor['origin'];mission.heading_alignment=anchor['scene_axis_registration']['heading_receipt']
    action=mission.request_route('exploration',0)[0]
    expected=dict(schema_version=2,request_id=action['request_id'],frame_id=action['frame_id'],goals=action['goals'])
    request=small['mission46_requests/exploration_1.json']
    if request!=expected:raise ValueError('Original registered exploration goal geometry or identity differs')
    goals=[audit.parse_goal(g)for g in request['goals']];definition_hash=definitions_sha256(goals)
    streams={};poses={};conflicts=[];frame_errors=[]
    for row,provenance in rows_once(run/'navigation_slam_poses.jsonl',streams):
        stamp=row['stamp_ns'];position=row['position']
        if type(stamp)is not int or row.get('frame_id')!='camera_init'or row.get('child_frame_id')!='demo_slam_body'or len(position)!=3 or not all(math.isfinite(float(v))for v in position):frame_errors.append(provenance)
        if stamp in poses and poses[stamp]['position']!=position:conflicts.append(stamp)
        poses[stamp]=dict(position=position,provenance=provenance)
    best=None;best_source=None;status_errors=[];receipt_seen={};receipt_sources={}
    for row,provenance in rows_once(run/'navigation_status.jsonl',streams):
        if row.get('navigation_ground_truth_used')is not False:status_errors.append(dict(reason='ground_truth_flag',**provenance))
        if row.get('request_id')!=request['request_id']:continue
        if row.get('goals_definitions')!=request['goals']or row.get('goals_definition_sha256')!=definition_hash:status_errors.append(dict(reason='original_goal_definition',**provenance))
        receipts=row.get('region_arrivals')or[]
        for i,r in enumerate(receipts):
            if r.get('waypoint_index')!=i or i>=len(goals)or r.get('goal_id')!=goals[i].goal_id:status_errors.append(dict(reason='receipt_order',**provenance))
            key=r['goal_id']
            if key in receipt_seen and receipt_seen[key]!=r:status_errors.append(dict(reason='receipt_mutation',**provenance))
            if key not in receipt_seen:receipt_sources[key]=provenance
            receipt_seen[key]=r
        if best is None or len(receipts)>=len(best.get('region_arrivals')or[]):best=row;best_source=provenance
    if not best:raise ValueError('No source-bound exploration status')
    latest=small['navigation_status.json']
    latest_match=latest.get('request_id')==request['request_id']and latest.get('region_arrivals')==best.get('region_arrivals')
    checks=[verify_dwell(audit,raw,r,poses,sorted(poses),request['request_id'],definition_hash)for raw,r in zip(request['goals'],best.get('region_arrivals')or[])]
    for row in checks:row['first_receipt_status_source']=receipt_sources[row['goal_id']]
    source_ok=bool(poses)and not frame_errors and not conflicts and not status_errors and latest_match
    partial_pass=source_ok and len(checks)>0 and all(r['passed']for r in checks)
    small_sources={str(run/name):dict(sha256=sha(run/name),bytes=(run/name).stat().st_size)for name in names}
    return dict(schema='v24_interrupted_targeted_arrival_audit/v1',run=str(run),run_id=run.name,
        status='INTERRUPTED_PARTIAL_EVIDENCE_ONLY',full46_passed=False,functional46_limited_pass=False,
        full22_evaluation=dict(completed=False,status='UNVERIFIED_INCOMPLETE_EVALUATION',blocking_missing_file='worker_result.json',frozen_reader_or_run_files_modified=False),
        original_runtime_check=dict(status='failed',runtime_error=runtime.get('error'),all_owned_and_children_clean=runtime.get('all_owned_and_children_clean'),worker_result_present=False,
            worker_returncode=next((x.get('returncode')for x in runtime.get('owned_processes',[])if x.get('role')=='worker'),None)),
        signal_attribution=dict(sender_verified=False,exact_signal_verified=False,statement='Runner records a shared InterruptedError path. SIGINT versus SIGTERM and sender cannot be established from this audit.'),
        targeted_original_arrivals=dict(passed=partial_pass,verified_count=sum(r['passed']for r in checks)if source_ok else 0,recorded_count=len(checks),exploration_required=18,fullmission_required=46,source_checks_passed=source_ok,
            source_frame_errors=frame_errors[:10],conflicting_pose_stamps=conflicts[:10],status_errors=status_errors[:10],latest_atomic_status_matches=latest_match,request_exact_original_geometry=True,
            goals_definition_sha256=definition_hash,receipts=checks,tenth_region=next((r for r in checks if r['ordinal']==10),None),latest_status=dict(state=best.get('state'),waypoint_index=best.get('waypoint_index'),pose=best.get('pose'),source=best_source,active_original_goal=request['goals'][best['waypoint_index']],center_error_xyz=[a-b for a,b in zip(best['pose'],goals[best['waypoint_index']].center)],inside_original_control_region=bool(audit.contains_control(goals[best['waypoint_index']],best['pose'])),interpretation='Observed last source pose relative to uncompleted goal; not a demonstrated causal explanation for the external interruption or an original region timeout.')),
        unverified_scopes=['whole46 mission completion','remaining original regions','native/physical safety and faults','terrain switching causality','current-run RGB save','original dynamic obstacle recovery','final first5s parking','heading/PI/command replay','complete200Hz actuator trace and all SCAN geometry replay'],
        source_bindings=dict(small_files=small_sources,streams=streams,pure_geometry=sources,five_direct_helpers=geometry,repair_source_closure=repair,
            reader=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)),full46_adapter=dict(path=str(Path(adapter.__file__).resolve()),sha256=ADAPTER_SHA)),
        scope=dict(raw_streams_read=['navigation_slam_poses.jsonl','navigation_status.jsonl'],raw_stream_read_passes_each=1,PID_native_cloud_or_array_read=False,runtime_Controller_called=False,navigation_ground_truth_used=False,real_robot_verified=False))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists()or args.output.resolve().is_relative_to(args.run.resolve()):raise ValueError('Output must be new and outside immutable run')
    os.sched_setaffinity(0,{max(os.sched_getaffinity(0))});os.nice(15)
    report=evaluate(args.run);args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    t=report['targeted_original_arrivals'];print(json.dumps(dict(output=str(args.output),sha256=sha(args.output),targeted_arrivals_passed=t['passed'],verified_count=t['verified_count'],tenth=t['tenth_region'],full46_passed=False),ensure_ascii=False))
if __name__=='__main__':main()
