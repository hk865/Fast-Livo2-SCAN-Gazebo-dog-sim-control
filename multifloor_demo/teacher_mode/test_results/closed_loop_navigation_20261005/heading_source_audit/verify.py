#!/usr/bin/env python3
"""Append-only original SLAM/SCAN heading-source lineage supplement.

No ROS, simulation, Actor inference, velocity publishing or process signals.
This verifies source agreement only; a source PASS cannot upgrade a failed
route, physical safety, arrival, first parking window or ramp receipt.
"""
from __future__ import annotations
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np

sys.dont_write_bytecode=True
SCHEMA='independent_actual_SLAM_SCAN_heading_source/v1'
CORE_SHA='a195c5c2c0d5494d72184c988cd3fea3daa7c5eebd0ba9ac4b9e4186e11e9415'
COMMON_SHA='074b468581de568264f558e38f82ce25e809094843c6f86e2816130babf8099e'
COMMON_PATH=Path(__file__).resolve().parents[1]/'acceptance_audit/evaluate_closed_loop.py'
TOL=1e-8


class MissingEvidence(Exception):pass


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


def read(path):
    if not Path(path).is_file():raise MissingEvidence('Missing '+str(path))
    return json.loads(Path(path).read_text())


def rows(path):
    if not Path(path).is_file():raise MissingEvidence('Missing '+str(path))
    with Path(path).open() as stream:return [json.loads(v) for v in stream if v.strip()]


def clean(value):
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def check(value,**details):
    return {'status':'unverified' if value is None else 'passed' if value else 'failed',
            'passed':None if value is None else bool(value),**clean(details)}


def overall(checks):
    states=[v['status'] for v in checks.values()]
    return 'failed' if 'failed' in states else 'unverified' if not states or 'unverified' in states else 'passed'


def wrap(value):return math.atan2(math.sin(value),math.cos(value))


def finite(value,width):
    a=np.asarray(value,float)
    if a.shape!=(width,) or not np.isfinite(a).all():raise ValueError('Missing finite original vector')
    return a


def stamp(value):
    if type(value) is not int or value<0:raise ValueError('Original integer source header required')
    return value


def indexed(data,key='stamp_ns'):
    out={}
    for row in data:
        i=stamp(row[key])
        if i in out and out[i]!=row:raise ValueError('Repeated source identity changed original data')
        out[i]=row
    return out


def projection(points,position,progress=None):
    """Independent exact bounded directed xyz projection used by frozen core."""
    points=np.asarray(points,float);position=finite(position,3)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or not np.isfinite(points).all():raise ValueError('Invalid directed SCAN path')
    delta=np.diff(points,axis=0);lengths=np.linalg.norm(delta[:,:2],axis=1)
    if np.any(lengths<1e-5):raise ValueError('Degenerate horizontal SCAN segment')
    cumulative=np.r_[0.,lengths.cumsum()]
    low=0. if progress is None else max(0.,progress-.15)
    high=cumulative[-1] if progress is None else min(cumulative[-1],progress+.8)
    best=None
    for i,(a,d,L) in enumerate(zip(points[:-1],delta,lengths)):
        if cumulative[i+1]<low or cumulative[i]>high:continue
        u=float(np.clip((position-a)@d/(d@d),max(0.,(low-cumulative[i])/L),min(1.,(high-cumulative[i])/L)))
        point=a+u*d;along=cumulative[i]+u*L
        candidate=(float((position-point)@(position-point)),float(along),i,point)
        if best is None or candidate[:3]<best[:3]:best=candidate
    if best is None:raise ValueError('No bounded directed source projection')
    _,nearest,i,point=best;updated=max(0. if progress is None else progress,nearest)
    tangent=delta[i,:2]/lengths[i]
    return dict(segment=i,point=point,tangent=tangent,nearest_s=nearest,
                remaining=float(cumulative[-1]-updated),progress=updated)


def selected_yaw(projected,points,goal,yaw,mode=None):
    if mode in ('capture','active_hold') or (np.linalg.norm(np.asarray(points)[-1]-np.asarray(goal))<=.2 and projected['remaining']<=.15):
        return float(yaw),'immutable_actual_segment_endpoint_yaw'
    return math.atan2(projected['tangent'][1],projected['tangent'][0]),'original_SCAN_nearest_directed_projection'


def same_geometry(observed,expected):
    if observed.get('segment')!=expected['segment']:return False,math.inf
    error=max(float(abs(finite(observed['point'],3)-expected['point']).max()),
              float(abs(finite(observed['tangent'],2)-expected['tangent']).max()),
              abs(float(observed['nearest_s'])-expected['nearest_s']),abs(float(observed['remaining'])-expected['remaining']))
    return error<=TOL,error


def load_module(path):
    name='_heading_source_'+sha(path)[:12]
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec)
    sys.modules[name]=mod;spec.loader.exec_module(mod);return mod


def verify(run,common_name='summary_closed_loop_cascade_independent.json'):
    run=Path(run).resolve();checks={};inputs={};ancestor={};metrics={}
    def audit(name,function):
        try:checks[name]=function()
        except (MissingEvidence,KeyError,FileNotFoundError) as e:checks[name]=check(None,reason=type(e).__name__+': '+str(e))
        except (ValueError,TypeError,IndexError,AttributeError,ImportError,OSError) as e:checks[name]=check(False,reason=type(e).__name__+': '+str(e))
    try:
        common_path=run/common_name;common=read(common_path)
        if common.get('schema')!='independent_actual_SLAM_SCAN_cascade_navigation/v1' or common.get('evaluator_sha256')!=COMMON_SHA or Path(common['run']).resolve()!=run:
            raise ValueError('Different immutable original common ancestor')
        hashes=common.get('verified_input_source_sha256')
        if not hashes:raise MissingEvidence('Original common source hashes absent')
        changed=[]
        for path,digest in hashes.items():
            p=Path(path).resolve()
            if run not in p.parents:raise ValueError('Ancestor source path outside this run')
            if not p.is_file():raise MissingEvidence('Missing original ancestor input '+path)
            if sha(p)!=digest:changed.append(path)
        checks['unchanged_common_ancestor_bytes']=check(not changed,changed_paths=changed,original_status=common['status'],
            failed_common_motion_gates=[k for k,v in common['checks'].items() if v['status']=='failed'],
            ancestor_failure_is_not_upgraded_by_source_receipt=True)
        ancestor={'common':{'path':str(common_path),'sha256':sha(common_path),'status':common['status'],'checks':common['checks']}}
        source_names=('frozen_scope_and_archived_sources','actual_causal_SLAM_IMU_cascade_updates',
            'actual_checked_SCAN_trajectory_payloads','fixed_goal_and_original_SCAN_bounded_projection',
            'actual_executor_ack_and_cascade_PI_COM_PD_replay','actual_raw_cloud_bytes_fields_filtering',
            'actual_cascade_movement_guard_raw_geometry','actual_command_original_read_dual_TTL_slew')
        source_statuses={k:common['checks'].get(k,{}).get('status','unverified') for k in source_names}
        checks['original_causal_source_PI_TTL_and_real_guard_gates']=check(
            False if 'failed' in source_statuses.values() else None if 'unverified' in source_statuses.values() else True,
            original_checks=source_statuses,motion_failure_remains_separate_and_not_upgraded=True)
        cp=run/'sources/acceptance/evaluate_closed_loop.py'
        if not cp.exists():cp=COMMON_PATH
        if sha(cp)!=COMMON_SHA:raise ValueError('Changed common geometry/source helper')
        base=load_module(cp);scope=read(run/'navigation_scope.json')
        checks['archived_executed_source_integrity']=base.source_audit(run,scope)
        cores=[p for p in (run/'sources').rglob('cascade_core.py') if sha(p)==CORE_SHA]
        if not cores:raise MissingEvidence('Exact unchanged a195 cascade core not archived')
        checks['unchanged_core_geometry_and_controller_math_identity']=check(True,archived_core_sha256=CORE_SHA,
            archived_matching_core_paths=[str(p) for p in cores],projection_independent_reconstruction=True)
        pid=rows(run/'navigation_pid_history.jsonl');poses=rows(run/'navigation_slam_poses.jsonl')
        pose_index=indexed(poses);feedback_index=indexed(rows(run/'navigation_feedback_history.jsonl'))
        heading=rows(run/'navigation_heading_projection.jsonl');paths=rows(run/'navigation_cascade_paths.jsonl')
        references=rows(run/'navigation_fixed_goal_references.jsonl');request=read(run/'navigation_request.json')
        if not heading or not pid:raise MissingEvidence('No new original heading/projection lineage records')
        checks['original_committed_SCAN_Bspline_and_payloads']=base.trajectories(run,rows(run/'navigation_status.jsonl'),pid)[0]
        checks['original_fixed_goal_and_bounded_core_projection']=base.goal_and_path_geometry(run,pid,poses)
        goals={};path_index={};missing=[];errors=[];maximum_geometry_error=0.;maximum_yaw_error=0.;maximum_locked_error=0.
        for r in references:
            key=(r['request_id'],r['waypoint_index']);goal=r['fixed_goal']
            if key in goals and goals[key]!=r:raise ValueError('Immutable activated goal changed')
            if hashlib.sha256(canonical(goal).encode()).hexdigest()!=r['fixed_goal_sha256']:raise ValueError('Original fixed-goal SHA differs')
            goals[key]=r
        for p in paths:
            if p['path_id'] in path_index and path_index[p['path_id']]!=p:raise ValueError('Original immutable core path changed')
            path_index[p['path_id']]=p
        for i,h in enumerate(heading):
            clock=stamp(h['control_stamp_ns']);s=stamp(h['source_pose_stamp_ns']);fb=h['feedback'];original=feedback_index.get(s)
            if original is None or s not in pose_index:missing.append(['heading',i,'Original feedback/source header']);continue
            expected={k:v for k,v in original.items() if k not in ('paired_imu','navigation_ground_truth_used')}
            if fb!=expected or s!=stamp(fb['stamp_ns']) or s>clock or clock-s>300_000_000:
                errors.append(['heading',i,'Changed/future/stale original SLAM feedback'])
            if h.get('navigation_ground_truth_used') is not False or h['steering']['cascade_projection'].get('navigation_ground_truth_used') is not False:
                errors.append(['heading',i,'Navigation source boundary differs'])
            if not np.allclose(fb['position_world_xyz'],pose_index[s]['position'],atol=1e-12,rtol=0):errors.append(['heading',i,'Position is not original raw SLAM'])
        # The preview never commits progress. A math-updated PID row commits the
        # same independent projection once; timer duplicates are never integrated.
        state={};fresh=0;matched=0;fixed_count=0;locked_count=0;protected=0
        for r in pid:
            key=(r['request_id'],r['waypoint_index']);c=r['cascade'];pathid=c.get('path_id');clock=stamp(r['compute_ros_clock_ns']);s=stamp(r['source_pose_stamp_ns'])
            if key not in goals or pathid not in path_index:missing.append(['PID',r['sequence'],'Original path/fixed goal']);continue
            path=path_index[pathid];ref=goals[key];points=np.asarray(path['points_xyz'],float);g=ref['fixed_goal'];fb=r.get('feedback')
            original=feedback_index.get(s)
            if fb is None or original is None:missing.append(['PID',r['sequence'],'Original source feedback']);continue
            if fb!={k:v for k,v in original.items() if k not in ('paired_imu','navigation_ground_truth_used')} or s>clock:
                errors.append(['PID',r['sequence'],'Feedback differs or is future'])
            if r.get('navigation_ground_truth_used') is not False or c.get('navigation_ground_truth_used') is not False:errors.append(['PID',r['sequence'],'Truth-navigation boundary differs'])
            projected=projection(points,fb['position_world_xyz'],state.get((key,pathid)))
            mode=c.get('mode',r['mode']);protected_now=c.get('protection_active') or mode in ('protect','recovering')
            gate=r.get('heading_gate_reference')
            if not gate:missing.append(['PID',r['sequence'],'New parent heading source']);continue
            if protected_now:
                protected+=1
                if np.any(finite(c['command_body'],3)):errors.append(['PID',r['sequence'],'Protected core sends nonzero correction'])
            fixed_branch=(mode in ('capture','active_hold') or c.get('parking_capture_pose_stamp_ns') is not None)
            wanted,branch=selected_yaw(projected,points,g['position_world_xyz'],g['heading_rad'],mode if fixed_branch else None)
            steering=gate.get('steering') or {}
            if 'heading' not in steering:missing.append(['PID',r['sequence'],'Parent selected reference yaw']);continue
            mismatch=abs(wrap(float(steering['heading'])-wanted));maximum_yaw_error=max(maximum_yaw_error,mismatch)
            if mismatch>TOL:errors.append(['PID',r['sequence'],'Parent reference is not the same directed SCAN/fixed-goal yaw',mismatch])
            if not fixed_branch:
                matching=[h for h in heading if h['request_id']==key[0] and h['waypoint_index']==key[1] and h['source_pose_stamp_ns']==s
                    and h['steering']['cascade_projection'].get('path_id')==pathid and 0<=clock-h['control_stamp_ns']<=50_000_000]
                if not matching:missing.append(['PID',r['sequence'],'No causal original parent preview']);continue
                h=max(matching,key=lambda x:x['control_stamp_ns']);hp=h['steering']['cascade_projection'];matched+=1
                if hp.get('path_sha256')!=path['path_sha256'] or hp.get('actual_pose_stamp_ns')!=s:errors.append(['PID',r['sequence'],'Projection path/header hash differs'])
                good,error=same_geometry(hp,projected);maximum_geometry_error=max(maximum_geometry_error,error)
                if not good:errors.append(['PID',r['sequence'],'Parent preview differs from original bounded _project',error])
                for k in ('heading','error','cascade_projection','heading_reference','original_lookahead_steering'):
                    if steering.get(k)!=h['steering'].get(k):errors.append(['PID',r['sequence'],'Selected parent reference differs from original preview',k])
            else:
                fixed_count+=1
                if abs(wrap(float(steering['heading'])-g['heading_rad']))>TOL:errors.append(['PID',r['sequence'],'Fixed endpoint branch changed target yaw'])
            if gate.get('phase')=='align' and not protected_now:
                locked=gate.get('locked_heading')
                if locked is None:missing.append(['PID',r['sequence'],'Align has no parent locked heading'])
                else:
                    locked_count+=1;diff=abs(wrap(float(locked)-wanted));maximum_locked_error=max(maximum_locked_error,diff)
                    if diff>TOL:errors.append(['PID',r['sequence'],'Parent locked align yaw differs from selected current source',diff])
            if c.get('controller_updated') is True:
                fresh+=1
                if c.get('feedback_pose_stamp_ns')!=s or c.get('path_sha256')!=path['path_sha256']:errors.append(['PID',r['sequence'],'Core math source stamp/path differs'])
                if abs(wrap(float(c['reference_yaw_rad'])-wanted))>TOL:errors.append(['PID',r['sequence'],'Fresh core yaw disagrees with parent selected source'])
                # A fresh core uses _project even during capture/active hold.
                numerical=max(float(abs(finite(c['nearest_projection_xyz'],3)-projected['point']).max()),
                    float(abs(finite(c['local_horizontal_tangent'],2)-projected['tangent']).max()),
                    abs(float(c['progress_m'])-projected['progress']))
                maximum_geometry_error=max(maximum_geometry_error,numerical)
                if numerical>TOL:errors.append(['PID',r['sequence'],'Fresh core projection recurrence differs',numerical])
                state[(key,pathid)]=projected['progress']
        checks['causal_original_heading_projection_and_selected_yaw']=check(False if errors else None if missing else bool(fresh and matched),
            actual_raw_parent_projection_rows=len(heading),actual_math_updated_rows=fresh,matched_PID_parent_previews=matched,
            immutable_fixed_endpoint_phase_rows=fixed_count,actual_align_locked_reference_rows=locked_count,
            protection_zero_rows=protected,maximum_geometry_replay_error=maximum_geometry_error,
            maximum_parent_core_selected_yaw_error_rad=maximum_yaw_error,
            maximum_parent_locked_selected_yaw_error_rad=maximum_locked_error,
            errors=errors[:100],error_count=len(errors),missing_evidence=missing[:100],missing_count=len(missing),
            scope='Lineage/equality only; no changed motion/heading/arrival/safety thresholds')
        guardfile=run/'navigation_native_guard_trace.jsonl'
        if not guardfile.exists():guardfile=run/'navigation_guard_history.jsonl'
        guards=rows(guardfile);ledger=indexed(rows(run/'navigation_cloud_xyz.jsonl'));byseq={r['sequence']:r for r in pid}
        bad=[];associated=0
        for guard in guards:
            seq=guard.get('pid_sequence',guard.get('cascade_sequence'))
            if seq is None:continue  # Independent non-cascade prewarmup guards.
            if seq not in byseq:bad.append([seq,'Guard has no actual cascade tick']);continue
            r=byseq[seq];stamp_ns=guard.get('pid_control_pose_stamp_ns');clock=guard.get('pid_compute_ros_clock_ns')
            if stamp_ns!=r['source_pose_stamp_ns'] or clock!=r['compute_ros_clock_ns']:bad.append([seq,'Guard uses a different feedback/tick'])
            cloud=ledger.get(guard.get('cloud_stamp_ns',guard.get('cloud_message_stamp_ns')))
            # Actual production key is also retained below; an absent original
            # header/source ledger is UNVERIFIED, never inferred from geometry.
            if cloud is None:
                candidates=[x for x in ledger.values() if x['array_file']==guard.get('cloud_array_file')]
                cloud=candidates[0] if len(candidates)==1 else None
            if cloud is None:raise MissingEvidence('Actual guard original raw cloud association missing')
            for field,key in (('array_file','array_sha256'),('raw_payload_file','raw_payload_sha256')):
                p=(run/cloud[field]).resolve()
                if run not in p.parents or not p.is_file():raise MissingEvidence('Original guard cloud bytes absent')
                inputs[str(p)]=sha(p)
                if inputs[str(p)]!=cloud[key]:bad.append([seq,'Guard actual raw payload/array SHA differs'])
            associated+=1
        checks['actual_selected_source_has_original_native_cloud_guard']=check(bool(associated) and not bad,associated_actual_guard_rows=associated,errors=bad,
            original_raw_guard_geometry_remains_separately_required=True)
        metrics=dict(selected_source_feedback_hz='Original math-updated source headers; never Actor50Hz count',
                     producer_clock_phase_tolerance_s=.05,strict_feedback_future_tolerance_s=0.)
    except (MissingEvidence,KeyError,FileNotFoundError) as e:checks['complete_original_heading_source_evidence']=check(None,reason=type(e).__name__+': '+str(e))
    except (ValueError,TypeError,IndexError,AttributeError,ImportError,OSError) as e:checks['valid_original_heading_source_evidence']=check(False,reason=type(e).__name__+': '+str(e))
    for name in (common_name,'navigation_scope.json','navigation_source_snapshots.json','source_manifest.json','navigation_heading_projection.jsonl','navigation_pid_history.jsonl','navigation_cascade_paths.jsonl','navigation_fixed_goal_references.jsonl','navigation_request.json','navigation_slam_poses.jsonl','navigation_feedback_history.jsonl','navigation_imu_history.jsonl','navigation_native_guard_trace.jsonl','navigation_guard_history.jsonl','navigation_cloud_xyz.jsonl'):
        p=run/name
        if p.is_file():inputs[str(p)]=sha(p)
    return clean(dict(schema=SCHEMA,status=overall(checks),run=str(run),checks=checks,metrics=metrics,score=None,
        evaluator_sha256=sha(__file__),verified_input_source_sha256=inputs,ancestors=ancestor,
        scope=dict(source_lineage_only=True,navigation_ground_truth_used=False,Actor_privileged_dimensions=232,
                   navigation_motion_arrival_parking_result='Not upgraded; retain all original common and ramp gates',
                   dynamic_obstacle_stop_resume='unverified',full_multifloor='unverified',hardware='unverified')))


def selftest():
    assert overall({'missing':check(None)})=='unverified' and overall({'failed':check(False),'missing':check(None)})=='failed'
    points=np.array([[0,0,0],[1,0,.1],[1,1,.1]],float)
    p=projection(points,[.4,.2,.04]);assert p['segment']==0 and np.allclose(p['tangent'],[1,0])
    q=projection(points,[1,.7,.1],p['progress']);assert q['progress']>=p['progress']
    bounded=projection(points,[1,1,.1],.1);assert bounded['nearest_s']<=.9+1e-12
    reversed_path=projection(points[::-1],[.4,.2,.04]);assert abs(wrap(selected_yaw(reversed_path,points[::-1],[-1,0,0],0)[0]))>1.
    end=projection(points,[1,.95,.1]);assert selected_yaw(end,points,[1,1,.1],.3)[0]==.3
    assert selected_yaw(p,points,[9,9,0],-.5,'active_hold')[0]==-.5
    stacked=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,1],[0,0,1],[1,0,1]],float)
    upper=projection(stacked,[.5,0,1]);assert upper['segment']==4
    for bad in ([[0,0,0],[0,0,1]],[[0,0,0],[math.nan,0,0]]):
        try:projection(bad,[0,0,0])
        except ValueError:pass
        else:raise AssertionError('Invalid directed path accepted')
    transformed=points+np.array([3,-5,2]);pp=projection(transformed,np.array([.4,.2,.04])+[3,-5,2])
    assert pp['segment']==p['segment'] and np.allclose(pp['tangent'],p['tangent'])
    observed={k:clean(p[k]) for k in ('segment','point','tangent','nearest_s','remaining')}
    assert same_geometry(observed,p)[0]
    assert not same_geometry({**observed,'tangent':[0,1]},p)[0]
    assert stamp(123)==123
    try:stamp(123.)
    except ValueError:pass
    else:raise AssertionError('Float original source stamp accepted')
    return dict(status='passed',meaningful_groups=12,live_ROS_Gazebo_Actor_operations=0)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path)
    p.add_argument('--common-receipt',default='summary_closed_loop_cascade_independent.json')
    p.add_argument('--receipt-suffix');p.add_argument('--no-write',action='store_true');p.add_argument('--self-test',action='store_true')
    a=p.parse_args()
    if a.self_test:print(json.dumps(selftest()));return
    if a.run is None:p.error('--run required')
    if a.receipt_suffix and not a.receipt_suffix.replace('_','').replace('-','').isalnum():p.error('Invalid receipt suffix')
    dest=a.run.resolve()/('summary_heading_source_independent'+('.'+a.receipt_suffix if a.receipt_suffix else '')+'.json')
    if not a.no_write and dest.exists():p.error('Refusing to overwrite prior receipt')
    result=verify(a.run,a.common_receipt)
    if not a.no_write:
        with dest.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(status=result['status'],receipt=None if a.no_write else str(dest),checks={k:v['status'] for k,v in result['checks'].items()}),indent=2))


if __name__=='__main__':main()
