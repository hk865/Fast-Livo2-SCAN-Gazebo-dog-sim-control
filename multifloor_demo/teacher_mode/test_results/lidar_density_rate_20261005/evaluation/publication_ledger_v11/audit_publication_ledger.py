#!/usr/bin/env python3
"""Independent V11 prospective full-publication clock-hold audit.

No runtime controller/core imports or control outputs. Original independent PI
equations are replayed with explicit short-hold freeze / protected reset events.
Every other original common check and criterion remains exactly unchanged.
"""
from __future__ import annotations
import argparse,copy,hashlib,importlib.util,json,math,os,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
PI_SOURCE=HERE.parent/'clock_hold_v9/pi_replay.py'
sys.path.insert(0,str(PI_SOURCE.parent))
import pi_replay

CRITERIA_SHA='756ad4c3f78a68f42483c9d016a7caa49e2bcbfd22f41595de4ac1675acb480c'
PARENT_CRITERIA_SHA='9c2b5f2ff6a8f6525df662411ca51976ee3aa6196661f3b774b8a30bf4259f65'
COMMON_SHA='074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e'
SCHEMA='independent_actual_SLAM_SCAN_publication_ledger_navigation/v2'
REPLACED='actual_executor_ack_and_cascade_PI_COM_PD_replay'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb')as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())
def rows(p):return [json.loads(x)for x in Path(p).open()if x.strip()]
def canonical(d):return json.dumps(d,sort_keys=True,separators=(',',':'),allow_nan=False)
def check(v,**kw):return {'status':'unverified'if v is None else'passed'if v else'failed','passed':v,**kw}
def overall(checks):
    s=[c['status']for c in checks.values()]
    return 'failed'if'failed'in s else'unverified'if not s or'unverified'in s else'passed'
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
def exact_zero(d):return len(d)==3 and all(type(v)in(int,float)and math.isfinite(v)and v==0 for v in d)
def nearest_before(records,wall,key):
    found=None
    for d in records:
        if d[key]<=wall:found=d
        else:break
    return found

def bindings(run):
    snapshots=read(run/'navigation_source_snapshots.json');scope=read(run/'navigation_scope.json')
    mismatches=[];verified={};criterion=None;common=None
    for name,row in snapshots.items():
        p=Path(row['snapshot']).resolve()
        if not p.is_relative_to((run/'sources').resolve())or not p.is_file():mismatches.append(name+' missing/outside snapshot');continue
        actual=sha(p);verified[name]={'snapshot':str(p),'sha256':actual}
        if actual!=row['sha256']or scope['references'].get(name)!=actual:mismatches.append(name+' reference/hash')
        if actual==CRITERIA_SHA and Path(name).name in ('PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json','PROSPECTIVE_CRITERIA.json'):criterion=p
        if Path(name).name=='evaluate_closed_loop.py':common=p
    if criterion is None or common is None:raise ValueError('Prospective criterion/common snapshot missing')
    contract=read(criterion)
    if sha(common)!=COMMON_SHA:raise ValueError('Original common evaluator changed')
    if contract.get('parent_prospective_criteria_sha256')!=PARENT_CRITERIA_SHA:
        mismatches.append('Original prospective criterion lineage differs')
    if contract.get('ledger_receipt_schema')!=SCHEMA:mismatches.append('Prospective ledger receipt schema differs')
    expected={Path(k).name:v for k,v in contract['pre_test_source_sha256'].items()}
    # Match the new producer's prospective bytes. Numerical core/clock-hold
    # modules remain the original V9 bytes, checked in the frozen criterion.
    for name in expected:
        candidates=[(k,v)for k,v in verified.items()if Path(k).name==name]
        if len(candidates)!=1 or candidates[0][1]['sha256']!=expected[name]:mismatches.append(name+' differs from pre-test execution bytes')
    for name in ('controller.py','clock_hold.py'):
        candidates=[v for k,v in verified.items()if Path(k).name==name]
        if len(candidates)!=1:continue
        kept=[];inside=False;bad=False
        for line in Path(candidates[0]['snapshot']).read_text().splitlines(keepends=True):
            if 'PUBLICATION_OBSERVER_BEGIN'in line:
                if inside:bad=True
                inside=True;continue
            if 'PUBLICATION_OBSERVER_END'in line:
                if not inside:bad=True
                inside=False;continue
            if not inside:kept.append(line)
        actual=hashlib.sha256(''.join(kept).encode()).hexdigest()
        if inside or bad or actual!=contract['original_execution_sha256'][name]:mismatches.append(name+' original execution changed outside observational blocks')
    if read(run/'navigation_profile.json')['control_clock_contract']!=contract['frozen_control_clock_contract']:
        mismatches.append('Profile control-clock contract differs')
    return common,contract,check(not mismatches,criteria_sha256=CRITERIA_SHA,criterion_snapshot=str(criterion),
        common_evaluator_sha256=COMMON_SHA,source_mismatches=mismatches,verified_archived_sources=verified,
        copy_binding_by_bytes_not_old_absolute_host_paths=True)

def hold_audit(run,pid,holds,poses,guards,clouds,feedback,cloud_callbacks):
    errors=[];reset_errors=[];resume_errors=[];counts={};last_sequence=0;last_wall=-math.inf
    expected=read(run/'navigation_pid_writer_receipt.json').get('expected_control_clock_hold_records')
    if expected!=len(holds):errors.append(['writer expected/count',expected,len(holds)])
    ps={p['stamp_ns']:p for p in poses};cs={c['stamp_ns']:c for c in clouds};fs={f['stamp_ns']:f for f in feedback}
    accepted_cloud_receipts={c['accepted_stamp_ns']:c['accepted_cloud_received_monotonic_wall']
        for c in cloud_callbacks if c.get('accepted')is True}
    previous_receipts={};guard_order=sorted(guards,key=lambda g:g['compute_monotonic_wall'])
    for h in holds:
        seq=h['sequence'];wall=h['compute_monotonic_wall'];clock=h['compute_ros_clock_ns'];decision=h['decision'];counts[decision]=counts.get(decision,0)+1
        if seq!=last_sequence+1 or not math.isfinite(wall)or wall<last_wall:errors.append([seq,'Hold sequence/wall chronology differs'])
        last_sequence=seq;last_wall=wall
        if h.get('schema')!='teacher_control_clock_exact_zero_hold/v1'or h.get('navigation_ground_truth_used')is not False:errors.append([seq,'Hold schema/truth source differs'])
        for key,wanted in [('controller_math_called',False),('native_geometry_called',False),('source_receipts_refreshed',False),('native_guard_required_before_nonzero_resume',True),('original_300ms_TTL_preserved',True)]:
            if h.get(key)is not wanted:errors.append([seq,key+' differs'])
        if not exact_zero(h['command_after']):errors.append([seq,'Hold did not publish exact zero'])
        if type(clock)is not int or type(h['publish_ros_clock_ns'])is not int or h['publish_ros_clock_ns']<clock:errors.append([seq,'Hold original clocks invalid/backwards'])
        before=h['frozen_control_before'];after=h['frozen_control_after']
        actual_pid=sum(r['compute_monotonic_wall']<wall for r in pid)
        actual_guards=sum(g['compute_monotonic_wall']<wall for g in guards)
        if before['pid_records']!=actual_pid or before['actual_guard_records']!=actual_guards:errors.append([seq,'Hold counters do not join original preceding records'])
        if after['pid_records']!=before['pid_records']or after['actual_guard_records']!=before['actual_guard_records']:errors.append([seq,'Hold advanced math/native guard counters'])
        guard=nearest_before(guard_order,wall,'compute_monotonic_wall')
        expected_guard=None if guard is None else guard['compute_ros_clock_ns']
        if h['previous_actual_guard_clock_ns']!=expected_guard:errors.append([seq,'Previous actual geometry clock differs from raw guard history'])
        if decision in ('duplicate','stalled')and clock not in(h['previous_control_clock_ns'],h['previous_actual_guard_clock_ns']):errors.append([seq,'Same-clock decision lacks original duplicate clock'])
        progress=h['last_clock_progress_monotonic_wall']
        if decision=='stalled'and(progress is None or wall-progress<.3-1e-9):reset_errors.append([seq,'Stalled decision before300ms'])
        if decision=='duplicate'and progress is not None and wall-progress>=.3+1e-9:reset_errors.append([seq,'300ms clock stall treated as short duplicate'])
        if decision=='backwards'and h['state_after']!='failed':reset_errors.append([seq,'Backward clock did not latch failure'])
        p=ps.get(h['source_pose_stamp_ns']);c=cs.get(h['source_cloud_stamp_ns']);f=fs.get(h['source_pose_stamp_ns'])
        if p is None or c is None or f is None:errors.append([seq,'Original SLAM/cloud/paired source missing'])
        stale=[]
        if p is not None:
            if not(-50_000_000<=clock-p['stamp_ns']<300_000_000):stale.append('SLAM_sim_TTL')
            if not(0<=wall-p['received_monotonic_wall']<.3+1e-9):stale.append('SLAM_original_wall_TTL')
        if c is not None:
            receipt=c['received_monotonic_wall']
            if not(-50_000_000<=clock-c['stamp_ns']<300_000_000)or not(0<=wall-receipt<.3+1e-9):stale.append('cloud_TTL')
            accepted_receipt=accepted_cloud_receipts.get(h['source_cloud_stamp_ns'])
            if accepted_receipt is None or h['source_cloud_received_monotonic_wall']!=accepted_receipt:
                errors.append([seq,'Original accepted cloud receipt missing/refreshed'])
        if f is not None:
            if not(0<=clock-f['stamp_ns']<=300_000_000 and 0<=round(wall*1e9)-f['received_wall_ns']<=300_000_001):stale.append('feedback_TTL')
            imu=f.get('paired_imu')
            if imu is None or not(0<=clock-imu['stamp_ns']<=300_000_000 and 0<=round(wall*1e9)-imu['received_wall_ns']<=300_000_001):stale.append('paired_IMU_TTL')
        for key,stamp,receipt in [('SLAM',h['source_pose_stamp_ns'],h['source_pose_received_monotonic_wall']),('cloud',h['source_cloud_stamp_ns'],h['source_cloud_received_monotonic_wall'])]:
            old=previous_receipts.get(key)
            if old and stamp==old[0]and receipt!=old[1]:errors.append([seq,key+' repeated header refreshed receipt'])
            previous_receipts[key]=(stamp,receipt)
        if (stale or decision=='stalled')and not h['protected_or_stale_reset']:reset_errors.append([seq,'Stale/stalled hold did not protect reset',stale])
        if not h['protected_or_stale_reset']:
            if canonical(before)!=canonical(after):errors.append([seq,'Short unprotected hold changed frozen PI/source/dwell state'])
        else:
            for key in ('velocity_integral','hold_integral'):
                if after[key]is not None and not exact_zero(after[key]):reset_errors.append([seq,key+' did not protect reset'])
            for key in ('math_pose_stamp_ns','capture_dwell_pose_stamp_ns','turn_dwell_pose_stamp_ns','obstacle_clear_since_s','arrival_since_s','region_arrival_since_stamp_ns'):
                if after[key]is not None:reset_errors.append([seq,key+' did not protect reset'])
        resumed=next((r for r in pid if r['compute_monotonic_wall']>wall and not exact_zero(r['command_after_slew'])),None)
        if resumed is not None:
            eligible=[g for g in guards if wall<g['compute_monotonic_wall']and g.get('pid_sequence')==resumed['sequence']]
            if len(eligible)!=1:resume_errors.append([seq,'Next nonzero publication lacks actual geometry for that tick'])
            elif eligible[0]['compute_ros_clock_ns']<=h['publish_ros_clock_ns']:resume_errors.append([seq,'Resume actual guard did not advance original clock'])
    return {
        'actual_clock_hold_event_integrity_and_short_freeze':check(not errors,hold_records=len(holds),decisions=counts,errors=errors,
            filter_evidence='Filter freeze is subsequently checked through original independent PI recurrence, not directly present in hold snapshots'),
        'actual_clock_hold_stall_stale_reset_and_failure':check(not reset_errors,errors=reset_errors,
            actual_stalled_hold_records=counts.get('stalled',0),actual_backwards_hold_records=counts.get('backwards',0)),
        'actual_clock_hold_nonzero_resume_requires_native_geometry':check(not resume_errors,errors=resume_errors,
            independent_guard_freshness_geometry_remain_in_unchanged_common_check=True)}

def publication_slew_PID_hold_only(pid,holds):
    events=sorted([(r['compute_monotonic_wall'],'pid',r)for r in pid]+[(r['compute_monotonic_wall'],'hold',r)for r in holds],key=lambda x:x[0])
    previous_command=np.zeros(3);previous_publish=None;maximum=0.;errors=[];checked=0
    for wall,kind,r in events:
        if kind=='hold':
            if not np.allclose(r['command_before'],previous_command,atol=1e-12,rtol=0):errors.append(['hold',r['sequence'],'Command before hold differs from prior publication'])
            previous_command=np.zeros(3);previous_publish=r['publish_ros_clock_ns'];continue
        c=r['cascade'];desired=np.asarray(r['desired_body_command']);observed=np.asarray(r['prepared_after_slew_command'])
        if c['mode']in('protect','recovering','pre_turn','settle','path_end_hold'):expected=np.zeros(3)
        elif previous_publish is None:
            if not exact_zero(observed):errors.append(['pid',r['sequence'],'First nonzero publication lacks prior publication clock'])
            expected=observed.copy()
        else:
            dt=(r['compute_ros_clock_ns']-previous_publish)*1e-9
            if dt<0:errors.append(['pid',r['sequence'],'Publication clock went backwards'])
            step=np.array([.6,.6,.8])*max(0.,min(dt,.1));expected=previous_command+np.clip(desired-previous_command,-step,step)
            if np.linalg.norm(desired[:2])<1e-9:expected[:2]=0.
            checked+=1
        maximum=max(maximum,float(np.max(np.abs(expected-observed))))
        after=np.asarray(r['command_after_slew'])
        expected_after=np.zeros(3)if r.get('stopped')else observed
        if not np.allclose(after,expected_after,atol=1e-12,rtol=0):errors.append(['pid',r['sequence'],'Actual published output differs from stopped/prepared output'])
        if r['publish_ros_clock_ns']<r['compute_ros_clock_ns']:errors.append(['pid',r['sequence'],'Publication before original compute clock'])
        previous_command=after;previous_publish=r['publish_ros_clock_ns']
    if maximum>1e-8:errors.append(['maximum original controller slew replay error',maximum])
    return check(not errors and bool(events),events=len(events),normal_slew_replays=checked,
        maximum_replay_absolute_error=maximum,errors=errors,
        previous_command_and_clock_include_actual_hold_zero_publications=True)

def publication_ledger_audit(run,pid,holds,publications):
    errors=[];math_errors=[];maximum=0.;checked=0
    required_fields=read(HERE/'PROSPECTIVE_CRITERIA.json')['publication_ledger_gates']['producer_fields']
    writer=read(run/'navigation_pid_writer_receipt.json')
    expected=writer.get('expected_command_publication_records')
    total=writer.get('final_commands_published_count')
    if expected!=len(publications)or total!=len(publications):
        errors.append(['Producer ledger/writer/original published counter differ',len(publications),expected,total])
    byseq={r['sequence']:r for r in pid};associated={};prior=None;hold_publications=[];outside=0
    for index,pub in enumerate(publications):
        missing_fields=[name for name in required_fields if name not in pub]
        if missing_fields:errors.append([pub.get('sequence'),'Producer required fields absent',missing_fields])
        seq=pub.get('sequence');entry=pub.get('entry_monotonic_wall_ns');start=pub.get('publish_started_monotonic_wall_ns');finish=pub.get('monotonic_wall_ns');clock=pub.get('publish_ros_clock_ns')
        if seq!=index+1:errors.append([seq,'Publication sequence is incomplete'])
        if pub.get('commands_published_count')!=seq:errors.append([seq,'Original published counter differs from ledger sequence'])
        if pub.get('schema')!='teacher_actual_command_publication/v1':errors.append([seq,'Original publisher schema differs'])
        if pub.get('navigation_ground_truth_used')is not False:errors.append([seq,'Original publisher truth disclosure invalid'])
        if type(pub.get('stopped_argument'))is not bool:errors.append([seq,'Original publisher stopped argument absent/invalid'])
        if any(type(v)is not int for v in (entry,start,finish,clock)):
            errors.append([seq,'Original publisher clocks are absent or invalid']);continue
        if not entry<=start<=finish or prior is not None and entry<prior['monotonic_wall_ns']:
            errors.append([seq,'Publisher wall chronology differs'])
        before=np.asarray(pub.get('command_before',[]),float);after=np.asarray(pub.get('command_after',[]),float)
        if before.shape!=(3,)or after.shape!=(3,)or not np.isfinite(np.r_[before,after]).all():
            errors.append([seq,'Publisher actual commands invalid']);continue
        if pub.get('last_command_time_after')!=clock/1e9:
            errors.append([seq,'Producer publication anchor differs from actual last_command_time'])
        if prior is not None:
            if not np.allclose(before,prior['command_after'],atol=1e-12,rtol=0):errors.append([seq,'Producer command chain changed outside recorded publication'])
            if pub.get('last_publication_ros_clock_ns_before')!=prior['publish_ros_clock_ns']:
                errors.append([seq,'Producer preceding publication ROS anchor differs'])
            if pub.get('last_command_time_before')!=prior['last_command_time_after']:
                errors.append([seq,'Producer preceding last_command_time differs'])
            if clock<prior['publish_ros_clock_ns']:errors.append([seq,'Producer ROS publication clock moved backwards'])
        if not isinstance(pub.get('trigger'),str)or not pub['trigger']:errors.append([seq,'Original callback trigger absent'])
        for name in ('protection_flags_before','protection_flags_after'):
            if not isinstance(pub.get(name),dict):errors.append([seq,name+' absent'])
        if pub.get('state_before')!=pub.get('state_after'):errors.append([seq,'Publisher modified controller state'])
        pidseq=pub.get('associated_pid_sequence')
        if pidseq is None:
            outside+=1
            if not exact_zero(pub['command_after']):errors.append([seq,'Publication without actual controller row is nonzero'])
            if pub.get('trigger')=='before_control':hold_publications.append(pub)
        else:
            if pidseq in associated:errors.append([seq,'A PID row was published twice'])
            associated[pidseq]=seq;row=byseq.get(pidseq)
            if row is None:errors.append([seq,'Original associated PID row missing'])
            else:
                if pub['stopped_argument']is not row.get('stopped'):errors.append([seq,'Original stopped argument differs from linked PID rule'])
                if clock!=row['publish_ros_clock_ns']or start/1e9<row['compute_monotonic_wall']-1e-9:
                    errors.append([seq,'Original PID/publication time identity differs'])
                if not np.allclose(after,row['command_after_slew'],atol=1e-12,rtol=0):errors.append([seq,'Original PID/publication command identity differs'])
                mode=row['cascade']['mode'];desired=np.asarray(row['desired_body_command']);observed=np.asarray(row['prepared_after_slew_command'])
                if mode in('protect','recovering','pre_turn','settle','path_end_hold'):replayed=np.zeros(3)
                elif prior is None:
                    replayed=observed.copy()
                    if not exact_zero(observed):math_errors.append([pidseq,'First nonzero PID has no recorded prior publication'])
                else:
                    dt=row['compute_ros_clock_ns']/1e9-prior['publish_ros_clock_ns']/1e9
                    if dt<0:math_errors.append([pidseq,'PID clock predates preceding original publication'])
                    step=np.array([.6,.6,.8])*max(0.,min(dt,.1));replayed=np.asarray(prior['command_after'])+np.clip(desired-np.asarray(prior['command_after']),-step,step)
                    if np.linalg.norm(desired[:2])<1e-9:replayed[:2]=0.
                    checked+=1
                error=float(np.max(abs(replayed-observed)));maximum=max(maximum,error)
                if error>1e-8:math_errors.append([pidseq,'Exact original publication slew replay differs',error])
                if not np.allclose(after,np.zeros(3)if row.get('stopped')else observed,atol=1e-12,rtol=0):math_errors.append([pidseq,'Actual output differs from unchanged prepared/stopped rule'])
        prior=pub
    missing=set(byseq)-set(associated)
    if missing:errors.append(['PID rows missing original publication',sorted(missing)])
    control_walls=sorted(r['compute_monotonic_wall']for r in pid+holds)
    used=[]
    for h in holds:
        later=[w for w in control_walls if w>h['compute_monotonic_wall']]
        bound=min(later)if later else math.inf
        matches=[p for p in hold_publications if p['sequence']==h.get('command_publication_sequence')and p['publish_ros_clock_ns']==h['publish_ros_clock_ns']and h['compute_monotonic_wall']<=p['publish_started_monotonic_wall_ns']/1e9<bound]
        if len(matches)!=1:errors.append(['hold',h['sequence'],'Hold lacks unique actual zero publication',len(matches)]);continue
        p=matches[0];used.append(p['sequence'])
        if not np.allclose(p['command_before'],h['command_before'],atol=1e-12,rtol=0):errors.append(['hold',h['sequence'],'Hold original command-before identity differs'])
    if len(used)!=len(hold_publications)or len(set(used))!=len(used):errors.append(['Original before_control publications do not biject hold records'])
    integrity=check(bool(publications)and not errors,actual_publication_records=len(publications),writer_expected_records=expected,
        original_commands_published_count=total,associated_PID_records=len(associated),outside_PID_publications=outside,
        associated_hold_records=len(used),errors=errors,
        actual_original_producer_clocks_used=True,bridge_receive_clock_substitution_used=False)
    slew=check(bool(checked)and not math_errors and integrity['passed']is True,
        normal_slew_replays=checked,maximum_replay_absolute_error=maximum,errors=math_errors,
        publication_integrity_status=integrity['status'],
        previous_command_and_clock_include_all_actual_PID_hold_and_callback_zero_publications=True,
        original_1e_minus8_numerical_gate_unchanged=True)
    return integrity,slew

def evaluate(run):
    run=run.resolve();common_path,criterion,binding=bindings(run)
    common=module(common_path,'_original_clock_hold_common')
    if common.CRITERIA!=criterion['unchanged_common_numerical_criteria']:raise ValueError('Original common numerical gates differ from prospective criterion')
    raw_path=run/'summary_closed_loop_cascade_independent.json';raw=read(raw_path)
    if Path(raw['run']).resolve()!=run:raise ValueError('Foreign-run raw common receipt')
    pid=rows(run/'navigation_pid_history.jsonl');poses=rows(run/'navigation_slam_poses.jsonl');guards=rows(run/'navigation_guard_history.jsonl')
    hpath=run/'navigation_control_clock_hold.jsonl';holds=rows(hpath)if hpath.exists()else[]
    clouds=rows(run/'navigation_cloud_history.jsonl');feedback=rows(run/'navigation_feedback_history.jsonl')
    cloud_callbacks=rows(run/'navigation_cloud_callbacks.jsonl')
    publication_path=run/'navigation_command_publications.jsonl'
    publications=rows(publication_path)if publication_path.exists()else[]
    execution=rows(run/'telemetry.jsonl');profile=read(run/'navigation_profile.json')
    worker=read(run/'worker_result.json');runtime=read(run/'runtime_manifest.json')
    if worker.get('completed')is not True or runtime.get('all_owned_and_children_clean')is not True:raise ValueError('Actual run/owned cleanup incomplete')
    replay_helpers=['rows','original_index','ns','finite','rotation','angles','wrap','bounded_command','check','MissingEvidence']
    for name in replay_helpers:setattr(pi_replay,name,getattr(common,name))
    extras=hold_audit(run,pid,holds,poses,guards,clouds,feedback,cloud_callbacks)
    extras['prospective_clock_hold_criteria_and_archived_sources']=binding
    integrity,slew=publication_ledger_audit(run,pid,holds,publications)
    extras['actual_control_publication_ledger_complete_and_original']=integrity
    extras['actual_publication_slew_with_hold_chronology']=slew
    try:pi=pi_replay.ack_and_PI_hold_audit(run,pid,poses,execution,profile,holds)
    except Exception as e:pi=check(False,error=type(e).__name__+': '+str(e))
    extras['actual_executor_ack_and_PI_replay_with_explicit_hold_resets']=pi
    checks=copy.deepcopy(raw['checks']);checks[REPLACED]=copy.deepcopy(pi);checks.update(extras)
    untouched={k:v for k,v in checks.items()if k in raw['checks']and k!=REPLACED}
    originals={k:v for k,v in raw['checks'].items()if k!=REPLACED}
    equal=canonical(untouched)==canonical(originals)
    hashes={name:sha(run/name)for name in ['navigation_pid_history.jsonl','navigation_pid_writer_receipt.json','navigation_slam_poses.jsonl','navigation_guard_history.jsonl','navigation_cloud_history.jsonl','navigation_cloud_callbacks.jsonl','navigation_feedback_history.jsonl','navigation_profile.json','navigation_source_snapshots.json','runtime_manifest.json','worker_result.json','closed_loop_executor_ack.jsonl','telemetry.jsonl']}
    if hpath.exists():hashes[hpath.name]=sha(hpath)
    if publication_path.exists():hashes[publication_path.name]=sha(publication_path)
    result={'schema':SCHEMA,'run':str(run),'status':overall(checks),'passed':overall(checks)=='passed',
        'simulation_only':True,'navigation_ground_truth_used':False,'criteria_sha256':CRITERIA_SHA,
        'source_bindings_verified':binding['passed']is True,
        'raw_common_receipt':str(raw_path),'raw_common_receipt_sha256':sha(raw_path),'raw_common_status':raw['status'],
        'unchanged_common_checks_verified':equal,'explicit_replaced_common_checks':[REPLACED],
        'unchanged_common_numerical_criteria':copy.deepcopy(common.CRITERIA),
        'checks':checks,'raw_common_checks':raw['checks'],'holds':{'count':len(holds),'events_chronology':'PID/hold original monotonic compute wall time; slew uses actual publish ROS stamp'},
        'source_bindings':{name:sha(run/name)for name in ['navigation_scope.json','source_manifest.json','navigation_source_snapshots.json','navigation_profile.json','runtime_manifest.json','slam_loaded_binary.json']},
        'criteria_binding_source':binding['criterion_snapshot'],
        'verified_input_source_sha256':copy.deepcopy(raw['verified_input_source_sha256']),
        'clock_hold_verified_input_source_sha256':hashes,
        'scope':copy.deepcopy(raw.get('scope')),'criteria':copy.deepcopy(raw.get('criteria')),
        'parent_prospective_criteria_sha256':PARENT_CRITERIA_SHA,
        'evaluator_sha256':sha(__file__),'pi_replay_sha256':sha(PI_SOURCE),
        'meaning':'Parallel explicit clock-hold bookkeeping audit; preserves original common source, geometry, route,32-region, safety, speed, parking and nonflat gates. Partial or missing gates cannot become PASS.'}
    if not equal:result['status']='failed';result['passed']=False
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path);a=p.parse_args()
    result=evaluate(a.run);out=a.output or a.run/'summary_closed_loop_clock_hold_independent.json'
    # Temporary file plus exclusive hard link exposes complete JSON atomically.
    temp=out.with_name(out.name+'.building');temp.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    try:os.link(temp,out)
    finally:temp.unlink(missing_ok=True)
    print(json.dumps({'status':result['status'],'receipt':str(out),'source_bindings_verified':result['source_bindings_verified'],
        'checks':{k:v['status']for k,v in result['checks'].items()}},indent=2))

if __name__=='__main__':main()
