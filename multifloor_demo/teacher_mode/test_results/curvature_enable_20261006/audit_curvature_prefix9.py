#!/usr/bin/env python3
"""Frozen prefix9 acceptance plus a same-pass OFF/ON diagnostic observer.

Never audits a live run. Never edits run files or frozen acceptance. Comparison
reads only already produced small diagnostics. No default V19 baseline scan.
"""
import argparse
from collections import Counter
import copy
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
LEGACY=HERE.parent/'corridor_tracking_v20_continue_20261006/audit_prefix9_with_series.py'
LEGACY_SHA='274d494a9e5624a8c3c5063de550727fd59e6515ddad0102514d0f70ae1bfbee'
EVALUATOR=HERE.parent/'corridor_tracking_v20_20261006/evaluation/evaluate_prefix9.py'
EVALUATOR_SHA='812946d475141e2bd2c71fb925cadb75b6b1086027167fb53d8a318b25c3481b'

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for raw in iter(lambda:f.read(1<<20),b''):h.update(raw)
    return h.hexdigest()
def load():
    if sha(LEGACY)!=LEGACY_SHA or sha(EVALUATOR)!=EVALUATOR_SHA:raise RuntimeError('Frozen independent evaluator/helper changed')
    spec=importlib.util.spec_from_file_location('curvature_frozen_prefix9_helpers',LEGACY)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,module.audit
def obj(value):return value if isinstance(value,dict)else {}
def finite(value):return value if type(value)in(int,float)and math.isfinite(value)else None
def vec(value,n=3):return [finite(x)for x in value]if isinstance(value,(list,tuple))and len(value)==n else [None]*n
def refcopy(ref):return {k:ref[k]for k in ('line','offset','length','sha256')}
def save(path,value):
    with Path(path).open('x')as f:json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')

class Collector:
    def __init__(self,legacy):
        self.legacy=legacy;self.base=legacy.Collector()
        self.pid=[];self.native=[];self.status=[];self.activations=[];self.replans=[]
        self.errors=[];self.row_counts=Counter();self.contract=None;self.receipts={}
    def observe(self,path,row,ref):
        name=Path(path).name;self.row_counts[name]+=1
        r=refcopy(ref)
        if name=='navigation_slam_poses.jsonl':self.base.observe(Path(path),row,ref)
        elif name=='navigation_pid_history.jsonl':
            c=obj(row.get('cascade'));f=obj(row.get('feedback'));g=obj(row.get('heading_gate_reference'));st=obj(g.get('steering'))
            fresh=c.get('controller_updated')is True and c.get('feedback_pose_stamp_ns')==f.get('stamp_ns')
            d=c if fresh else {};sp=obj(d.get('spatial_reference'));b=obj(d.get('curvature_speed_supervisor'));pi=obj(d.get('velocity_PI'))
            self.pid.append(dict(clock_ns=row.get('control_stamp_ns'),pose_stamp_ns=f.get('stamp_ns'),goal=row.get('waypoint_index'),
                request_id=row.get('request_id'),trajectory_id=row.get('trajectory_id'),path_id=c.get('path_id'),mode=c.get('mode'),phase=g.get('phase'),
                controller_updated=c.get('controller_updated'),fresh_math=fresh,math_pose_stamp_ns=c.get('feedback_pose_stamp_ns'),
                inner_heading_error_rad=finite(d.get('error_yaw_rad')),outer_heading_error_rad=finite(st.get('heading_error')) if finite(st.get('heading_error'))is not None else finite(st.get('error')),
                outer_heading_error_source_key='heading_error'if finite(st.get('heading_error'))is not None else 'error'if finite(st.get('error'))is not None else None,
                locked_heading_rad=finite(g.get('locked_heading')),reference_yaw_rad=finite(d.get('reference_yaw_rad')),
                cross_raw_m=finite(d.get('error_cross_m')),cross_control_m=finite(d.get('control_error_cross_m')),
                kappa_per_m=finite(sp.get('curvature_per_m')),kappa_rate_per_m2=finite(sp.get('curvature_rate_per_m2')),
                curvature_valid=sp.get('curvature_valid'),fallback_reason=sp.get('fallback_reason'),heading_source=sp.get('heading_source'),
                curvature_FF_raw_radps=finite(obj(d.get('yaw_PD')).get('curvature_FF')),
                yaw_P_radps=finite(obj(d.get('yaw_PD')).get('P')),yaw_D_radps=finite(obj(d.get('yaw_PD')).get('D')),
                speed_supervisor=copy.deepcopy(b),integral_dt_s=finite(d.get('integral_dt_s')),header_dt_s=finite(d.get('header_dt_s')),
                reference_world_velocity=vec(d.get('reference_velocity_world')),reference_COM_vx_vy_wz=vec(d.get('reference_COM_velocity_body')),
                measured_SLAM_COM_velocity=vec(d.get('measured_COM_velocity_body')),measured_SLAM_origin_velocity=vec(f.get('origin_velocity_body')),
                measured_body_omega=vec(d.get('measured_body_omega')),measured_Euler_yawrate=finite(d.get('measured_Euler_yawrate_radps')),
                command=vec(row.get('command_after_slew')),raw_command=vec(d.get('raw_command_body')),saturated=d.get('command_saturated'),
                PI_error=vec(pi.get('error')),PI_integral_state=vec(pi.get('integral_state')),translation_integral_frozen=pi.get('translation_integral_frozen'),
                remaining_arc_m=finite(d.get('remaining_horizontal_arc_m')),goal_distance_xy_m=finite(d.get('goal_distance_xy_m')),
                position=vec(row.get('control_pose')),source=r))
        elif name=='telemetry.jsonl':
            self.native.append(dict(physics_world_s=finite(row.get('state_physics_world_time')),world_s=finite(row.get('world_sim_time')),
                command=vec(row.get('command')),requested=vec(row.get('requested')),COM_velocity=vec(row.get('body_lin_vel')),
                body_omega=vec(row.get('body_ang_vel')),position=vec(row.get('position')),quaternion_wxyz=vec(row.get('quaternion_wxyz'),4),
                rpy=vec(row.get('rpy')),fault=row.get('fault'),body_contact=obj(row.get('contacts')).get('body'),source=r))
        elif name=='navigation_status.jsonl':
            self.status.append(dict(clock_s=finite(row.get('ros_sim_time')),state=row.get('state'),goal=row.get('waypoint_index'),
                request_id=row.get('request_id'),message=row.get('message'),replans=row.get('replans'),reference_requests=row.get('reference_requests'),
                phase=row.get('alignment_phase'),trajectory_id=row.get('accepted_trajectory_id'),source=r))
            for receipt in row.get('region_arrivals')or[]:
                if isinstance(receipt,dict):self.receipts.setdefault(receipt.get('goal_id'),copy.deepcopy(receipt))
        elif name=='actuator.jsonl'and row.get('kind')=='actuator_contract'and self.contract is None:
            self.contract=dict(data=copy.deepcopy(row),source=r)
        elif name=='navigation_segment_activation.jsonl':self.activations.append(dict(data=copy.deepcopy(row),source=r))
        elif name=='reference_replan_events.jsonl':self.replans.append(dict(data=copy.deepcopy(row),source=r))
    def observe_safe(self,path,row,ref):
        try:self.observe(path,row,ref)
        except Exception as e:self.errors.append(dict(file=str(path),source=refcopy(ref),error=type(e).__name__+': '+str(e)))
    def summarize_rows(self,rows):
        durations=Counter();uncovered=0.;exits=[];changes=[];holds=[]
        for old,new in zip(rows,rows[1:]):
            if type(old['clock_ns'])is not int or type(new['clock_ns'])is not int:continue
            dt=(new['clock_ns']-old['clock_ns'])*1e-9
            same_goal=(old['request_id'],old['goal'])==(new['request_id'],new['goal'])
            if 0<dt<=.200000001 and same_goal:durations[str(old['mode'])]+=dt
            elif dt>0:uncovered+=dt
            changed=same_goal and old['path_id']!=new['path_id']
            if changed:changes.append(dict(clock_ns=new['clock_ns'],goal=new['goal'],from_id=old['path_id'],to_id=new['path_id'],source=new['source']))
            if same_goal and old['phase']=='drive'and new['phase']!='drive':exits.append(dict(clock_ns=new['clock_ns'],goal=new['goal'],to_phase=new['phase'],new_path=changed,source=new['source']))
            if new['mode']=='reference_constraint_hold'and old['mode']!='reference_constraint_hold':holds.append(dict(clock_ns=new['clock_ns'],goal=new['goal'],source=new['source']))
        fresh=[r for r in rows if r['fresh_math']];budgets=[r['speed_supervisor']for r in fresh if r['speed_supervisor'].get('enabled')]
        ff=[abs(r['curvature_FF_raw_radps'])for r in fresh if r['curvature_FF_raw_radps']is not None]
        return dict(rows=len(rows),fresh_math_rows=len(fresh),mode_counts=dict(Counter(str(r['mode'])for r in rows)),
            observed_mode_duration_s=dict(durations),uncovered_positive_gap_s=uncovered,
            duration_scope='Only adjacent same-goal PID rows with 0 < compute-clock gap <=0.2 s; no held-time inference across gaps.',
            drive_exit_count=len(exits),drive_exits=exits,new_path_count=len(changes),new_path_events=changes,
            constraint_hold_entry_count=len(holds),constraint_hold_entries=holds,
            enabled_speed_budget_rows=len(budgets),infeasible_budget_rows=sum(b.get('reference_budget_feasible')is False for b in budgets),
            limited_speed_rows=sum(type(b.get('limited_speed_mps'))in(int,float)and type(b.get('requested_speed_mps'))in(int,float)and b['limited_speed_mps']<b['requested_speed_mps']-1e-9 for b in budgets),
            nonzero_curvature_FF_rows=sum(x>1e-12 for x in ff),maximum_abs_logged_v_kappa_radps=max(ff,default=None),
            curvature_unavailable_drive_or_constraint_rows=sum(r['curvature_valid']is False and r['mode']in('drive','reference_constraint_hold')and r['heading_source']!='fixed_goal_tail'for r in fresh),
            fixed_goal_tail_math_rows=sum(r['heading_source']=='fixed_goal_tail'for r in fresh),
            saturated_command_rows=sum(isinstance(r['saturated'],list)and any(r['saturated'])for r in fresh))
    def ninth(self,run):
        acts=[r for r in self.activations if r['data'].get('waypoint_index')==8]
        receipt=self.receipts.get('exploration:8');rows=[r for r in self.pid if r['goal']==8]
        result=dict(region9_exposed=bool(rows),original_arrival=receipt is not None,activation_source=acts[0]if len(acts)==1 else None,
            activation_count=len(acts),arrival_receipt=receipt,full46_pass=False)
        if len(acts)!=1:return dict(**result,comparable=False,reason='One exact region9 activation journal required; stale status activation is not substituted.')
        start=acts[0]['data']['control_stamp_ns'];end=receipt.get('stamp_ns')if receipt else max((r['clock_ns']for r in rows if type(r['clock_ns'])is int),default=None)
        selected=[r for r in rows if type(r['clock_ns'])is int and end is not None and start<=r['clock_ns']<=end]
        result.update(comparable=bool(selected),activation_control_ns=start,terminal_pose_or_last_control_ns=end,
            elapsed_s=(end-start)*1e-9 if end is not None else None,terminal_kind='original_arrival_pose_stamp'if receipt else'last_observed_PID_compute_clock',
            before_arrival_or_last_observation=self.summarize_rows(selected),post_arrival_rows_excluded=sum(r['clock_ns']>end for r in rows)if receipt else 0)
        poses=self.base.runs.get(str(Path(run).resolve()),{}).get('poses',[])
        points=[p for p in poses if end is not None and p[0]is not None and start*1e-9<=p[0]<=end*1e-9 and all(x is not None for x in p[1:4])]
        if points:
            length=0.;gaps=0
            for a,b in zip(points,points[1:]):
                if 0<b[0]-a[0]<=.200000001:length+=math.sqrt(sum((y-x)**2 for x,y in zip(a[1:4],b[1:4])))
                else:gaps+=1
            result.update(observed_SLAM_3d_path_length_m=length,excluded_pose_gaps=gaps,pose_samples=len(points),
                net_SLAM_xy_displacement_m=math.hypot(points[-1][1]-points[0][1],points[-1][2]-points[0][2]))
        return result

def require_finished(run,audit):
    if not(run/'runtime_manifest.json').is_file()or not(run/'run_result.json').is_file():raise RuntimeError('Completed manifest and run_result required; live-run audit refused')
    runtime=json.loads((run/'runtime_manifest.json').read_text())
    stopped,evidence=audit.owned_processes_stopped(runtime)
    if not stopped:raise RuntimeError('Owned-process identities must be stopped before reading raw logs')
    return dict(runtime_manifest_sha256=sha(run/'runtime_manifest.json'),run_result_sha256=sha(run/'run_result.json'),owned_processes_stopped=True)

def run_once(run,condition,output_dir):
    legacy,audit=load();run=Path(run).resolve();out=Path(output_dir).resolve()
    if condition not in ('OFF','ON'):raise ValueError('Condition must be OFF or ON')
    if out.is_relative_to(run):raise ValueError('Outputs cannot be written into the run archive')
    paths={k:out/(run.name+'_'+suffix)for k,suffix in dict(report='PREFIX9_ACTUAL_EVALUATION.json',series='CURVATURE_SERIES.json.gz',diagnostic='CURVATURE_DIAGNOSTICS.json',receipt='AUDIT_RECEIPT.json').items()}
    if any(p.exists()for p in paths.values()):raise ValueError('Refusing to overwrite independent evidence; choose a new output directory')
    completion=require_finished(run,audit)
    profile=json.loads((run/'navigation_profile.json').read_text());settings=obj(obj(profile.get('cascade')).get('spatial_reference'))
    expected=condition=='ON';flags_match=all(settings.get(k)is expected for k in ('curvature_feedforward_enabled','curvature_speed_limit_enabled'))
    if not flags_match:raise ValueError('Requested OFF/ON identity differs from actual archived profile flags')
    collector=Collector(legacy);original=audit.Sources;instances=[];passes=Counter();report=None;error=None;extra=[]
    class ObservedSources(original):
        def __init__(self):super().__init__();instances.append(self)
        def rows(self,path):
            key=str(Path(path).resolve());passes[key]+=1
            if passes[key]!=1:raise RuntimeError('Repeated full JSONL read refused: '+key)
            for row,ref in super().rows(path):
                collector.observe_safe(Path(path),row,ref)
                yield row,ref
    audit.Sources=ObservedSources
    out.mkdir(parents=True,exist_ok=True)
    try:
        report=audit.audit(run,baseline=None) # Optional V19 comparison is not an acceptance gate.
        save(paths['report'],report) # Preserve original report exactly; diagnostics remain separate.
        aux=ObservedSources()
        for name in ('navigation_segment_activation.jsonl','reference_replan_events.jsonl'):
            if(run/name).is_file():
                for _ in aux.rows(run/name):pass
            else:extra.append(dict(file=name,available=False))
    except Exception as e:error=type(e).__name__+': '+str(e)
    finally:
        audit.Sources=original
        bindings={k:v for source in instances for k,v in source.bindings.items()}
        source_errors=[r for source in instances for r in source.errors]
        profile_binding=dict(file=str(run/'navigation_profile.json'),sha256=sha(run/'navigation_profile.json'))
        payload=dict(schema='curvature_prefix9_same_pass_series/v1',run=str(run),run_id=run.name,condition=condition,
            frozen_evaluator_sha256=EVALUATOR_SHA,legacy_helper_sha256=LEGACY_SHA,wrapper_sha256=sha(__file__),
            profile=profile_binding,settings=settings,pid=collector.pid,native=collector.native,status=collector.status,
            pose_columns=legacy.POSE_COLUMNS,poses=collector.base.runs.get(str(run),{}).get('poses',[]),
            activation_events=collector.activations,replan_events=collector.replans,actuator_contract=collector.contract,
            source_bindings=list(bindings.values()),full_JSONL_read_counts=dict(passes),diagnostic_errors=collector.errors,
            source_read_errors=source_errors,evaluation_error=error,missing='null is unavailable; no fill-forward. Curvature/math fields null unless same-row controller update and feedback header match.',
            native_velocity='Recorded body_lin_vel is COM velocity. Actual contract offset and omega are included for offline COM->origin conversion; native position already is body origin.',
            navigation_ground_truth_used=False,full46_pass=False,whole_200hz_physical_acceptance=False)
        with paths['series'].open('xb')as f:
            with gzip.GzipFile(fileobj=f,mode='wb',mtime=0,compresslevel=3)as z:z.write(json.dumps(payload,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode())
        diagnostic=dict(schema='curvature_prefix9_diagnostic/v1',run=str(run),run_id=run.name,condition=condition,
            profile=profile_binding,settings=settings,comparison_profile=profile,
            original_acceptance=dict(report=str(paths['report'])if report is not None else None,sha256=sha(paths['report'])if paths['report'].exists()else None,
                prefix9_limited_pass=report['prefix9_limited_pass']if report is not None else False,
                result_status=report.get('result_status')if report is not None else'evaluation_exception',checks=report.get('checks')if report is not None else None),
            series=dict(file=str(paths['series']),sha256=sha(paths['series'])),summary=collector.summarize_rows(collector.pid),
            ninth_region=collector.ninth(run),actual_reference_replan_events=collector.replans,auxiliary_availability=extra,
            diagnostic_complete=error is None and not collector.errors and not source_errors,evaluation_error=error,
            diagnostic_errors=collector.errors,source_bindings=list(bindings.values()),
            legacy_ninth_diagnostic_note='Original acceptance report unchanged. Its ninth diagnostic uses index8 statuses and can include stale activation/post-arrival rows; use this separate activation-journal bounded diagnostic for comparisons.',
            full46_pass=False,whole_200hz_physical_acceptance=False,corridor_control_verified=False,navigation_ground_truth_used=False)
        save(paths['diagnostic'],diagnostic)
        save(paths['receipt'],dict(schema='curvature_prefix9_audit_invocation/v1',run=str(run),condition=condition,completion=completion,
            wrapper=dict(file=str(Path(__file__).resolve()),sha256=sha(__file__)),frozen_evaluator=dict(file=str(EVALUATOR),sha256=EVALUATOR_SHA),
            reused_helper=dict(file=str(LEGACY),sha256=LEGACY_SHA),full_JSONL_read_counts=dict(passes),row_counts=dict(collector.row_counts),
            outputs={k:dict(file=str(p),sha256=sha(p),bytes=p.stat().st_size)for k,p in paths.items()if k!='receipt'and p.exists()},
            evaluation_error=error,diagnostic_errors=collector.errors,source_read_errors=source_errors,
            baseline_raw_read=False,acceptance_logic_unchanged=True,controller_replay=False,raw_run_modified=False))
    return dict(paths={k:str(v)for k,v in paths.items()},prefix9_limited_pass=report['prefix9_limited_pass']if report is not None else False,error=error)

def compare(off,on,output):
    records=[]
    for path,condition in ((Path(off),'OFF'),(Path(on),'ON')):
        d=json.loads(path.read_text())
        if d.get('schema')!='curvature_prefix9_diagnostic/v1'or d.get('condition')!=condition:raise ValueError('Foreign diagnostic schema/condition')
        if not d.get('diagnostic_complete'):raise ValueError('Incomplete diagnostics cannot establish A/B comparison')
        r=d['original_acceptance'];rp=Path(r['report'])
        if sha(rp)!=r['sha256']:raise ValueError('Independent acceptance report binding changed')
        original=json.loads(rp.read_text())
        if original.get('schema')!='independent_original46_prefix9_actual_run/v1'or original.get('run')!=d['run']or original.get('run_id')!=d['run_id']:raise ValueError('Foreign report identity')
        records.append(d)
    if records[0]['run']==records[1]['run']:raise ValueError('OFF and ON must be different actual runs')
    diffs=[]
    def walk(a,b,path=''):
        if isinstance(a,dict)and isinstance(b,dict):
            for k in sorted(set(a)|set(b)):walk(a.get(k),b.get(k),path+'.'+k)
        elif a!=b:diffs.append(dict(field=path.lstrip('.'),OFF=a,ON=b))
    walk(records[0]['comparison_profile'],records[1]['comparison_profile'])
    result=dict(schema='curvature_prefix9_OFF_ON_comparison/v1',sources=[dict(file=str(Path(p).resolve()),sha256=sha(p))for p in (off,on)],
        OFF={k:records[0][k]for k in ('run','original_acceptance','summary','ninth_region')},ON={k:records[1][k]for k in ('run','original_acceptance','summary','ninth_region')},
        profile_differences=diffs,causal_attribution_verified=False,
        note='Two independently evaluated actual runs; all profile differences are shown. Comparison does not upgrade either acceptance or establish a single-cause benefit.',
        raw_logs_reread=False,full46_pass=False,whole_200hz_physical_acceptance=False)
    save(output,result);return dict(output=str(output),profile_difference_count=len(diffs))

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    run=sub.add_parser('audit');run.add_argument('--run',type=Path,required=True);run.add_argument('--condition',choices=['OFF','ON'],required=True);run.add_argument('--output-dir',type=Path,default=HERE/'actual')
    comp=sub.add_parser('compare');comp.add_argument('--off',type=Path,required=True);comp.add_argument('--on',type=Path,required=True);comp.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.action=='audit':
        if hasattr(os,'sched_setaffinity'):os.sched_setaffinity(0,{max(os.sched_getaffinity(0))})
        os.nice(15);result=run_once(a.run,a.condition,a.output_dir)
    else:result=compare(a.off,a.on,a.output)
    print(json.dumps(result,indent=2,ensure_ascii=False));return 1 if result.get('error')else 0

if __name__=='__main__':sys.exit(main())
