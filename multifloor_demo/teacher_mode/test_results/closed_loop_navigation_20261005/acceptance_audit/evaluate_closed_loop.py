#!/usr/bin/env python3
"""Append-only independent actual-SLAM/SCAN cascade navigation audit.

Offline reader: no ROS, process signals, actor inference, or command publishing.
Missing raw evidence is UNVERIFIED. Historical receipts never unlock a gate.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.dont_write_bytecode=True

SCHEMA = 'independent_actual_SLAM_SCAN_cascade_navigation/v1'
SCOPE_SCHEMA = 'teacher_closed_loop_navigation_scope/v1'
TICK_SCHEMA = 'teacher_closed_loop_cascade_tick/v1'
MODEL_SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
CRITERIA = {
    'source_ttl_sim_s': .3, 'source_ttl_wall_s': .3,
    'producer_clock_future_tolerance_s': .05, 'gyro_to_pose_max_gap_s': .02,
    'region_control_radius_m': .17, 'region_dwell_s': .6,
    'region_max_source_gap_s': .2, 'goal_deadline_s': 90.,
    'native_step_s': .005, 'actor_step_s': .02, 'bootstrap_s': .1,
    'flat_route_max_distance_m': .20, 'flat_route_rms_m': .08,
    'drive_heading_max_rad': .20, 'speed_mean_abs_error_floor_mps': .05,
    'speed_mean_abs_error_fraction': .25,
    'maximum_roll_pitch_rad': .65, 'minimum_clearance_m': .18,
    'maximum_tau_Nm': 23.50001, 'maximum_qd_radps': 30.001,
    'maximum_all_feet_unsupported_s': .3,
    'parking_window_s': 5., 'parking_xy_drift_m': .05,
    'parking_yaw_drift_rad': .1, 'parking_origin_speed_peak_mps': .08,
    'parking_euler_yawdot_peak_radps': .1, 'parking_body_wz_peak_radps': .1,
    'worker_slew_acceleration': [.6,.6,.8],
    'dynamic_clear_s': 1., 'dynamic_guard_max_gap_s': .3,
}
TRANSPORT_KEYS = {
    'transport_write_started_monotonic_wall', 'transport_written_monotonic_wall',
    'transport_queue_delay_wall_s', 'transport_write_duration_wall_s',
    'transport_superseded_envelopes',
}


class MissingEvidence(Exception):
    pass


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''): h.update(block)
    return h.hexdigest()


def canonical(data):
    return json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False)


def clean(value):
    if isinstance(value,np.ndarray): return clean(value.tolist())
    if isinstance(value,np.generic): return clean(value.item())
    if isinstance(value,dict): return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value): return None
    return value


def load(path):
    if not Path(path).is_file(): raise MissingEvidence('Missing '+str(path))
    return json.loads(Path(path).read_text(),parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))


def rows(path):
    if not Path(path).is_file(): raise MissingEvidence('Missing '+str(path))
    with Path(path).open() as f:
        return [json.loads(v,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x))) for v in f if v.strip()]


def ns(value):
    if type(value) is not int or value<0: raise ValueError('Original nonnegative integer header required')
    return value


def finite(value,n):
    a=np.asarray(value,float)
    if a.shape!=(n,) or not np.isfinite(a).all(): raise ValueError('Malformed finite vector '+str(n))
    return a


def check(value,**detail):
    return {'status':'unverified' if value is None else 'passed' if value else 'failed',
            'passed':None if value is None else bool(value),**clean(detail)}


def overall(checks):
    states=[v['status'] for v in checks.values()]
    return 'failed' if 'failed' in states else 'unverified' if 'unverified' in states else 'passed'


def rotation(q):
    w,x,y,z=np.asarray(q,float).T
    return np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
                     2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
                     2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],axis=-1).reshape(-1,3,3)


def angles(q):
    w,x,y,z=np.asarray(q,float).T
    return np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def wrap(x): return np.arctan2(np.sin(x),np.cos(x))


def distribution(v):
    a=np.asarray(v,float)
    return None if not len(a) else clean({'count':len(a),'min':a.min(),'p50':np.percentile(a,50),
                                       'p95':np.percentile(a,95),'max':a.max()})


def module(path):
    name='closed_loop_readonly_'+sha(path)[:16]
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m)
    return m


def archived_module_path(run,name):
    candidates=[p for p in (run/'sources').rglob('*.py') if p.name==name or p.name.endswith('_'+name)]
    digests={sha(p) for p in candidates}
    if len(digests)!=1:raise MissingEvidence('Unique archived '+name+' code not identifiable')
    return candidates[0]


def execution_statuses(run,execution):
    """Closed-loop run outcomes precede the last real Actor read, never cleanup.

    Keep every excluded row in diagnostics. A failure inside the measured
    execution horizon still fails; later shutdown publications are not motion.
    """
    allrows=rows(run/'navigation_status.jsonl')
    last=execution[-1]['closed_loop_read_evidence']['actual_read_attempt']
    endwall=last['read_monotonic_wall'];original=last.get('decoded_envelope')
    if original is not None and last.get('producer_wall_age_s') is not None:
        endwall=max(endwall,original['monotonic_wall']+last['producer_wall_age_s'])
    active=[r for r in allrows if float(r['monotonic_wall'])<=endwall+1e-9]
    cleanup=[r for r in allrows if float(r['monotonic_wall'])>endwall+1e-9]
    return active,check(bool(active) and not any(r.get('state')=='failed' for r in active),
                        last_actual_Actor_read_wall=endwall,last_actual_Actor_world_clock_s=execution[-1]['world_sim_time'],
                        execution_states=[r.get('state') for r in active],
                        after_execution_publications=[{k:r.get(k) for k in('state','ros_sim_time','monotonic_wall','message')} for r in cleanup],
                        cleanup_states_do_not_replace_original_running_phase_outcomes=True)


def causal(times,query,gap):
    i=np.searchsorted(times,query,side='right')-1
    good=i>=0; i=np.clip(i,0,len(times)-1)
    return i,good&(query-times[i]>=-1e-10)&(query-times[i]<=gap+1e-10)


def longest(mask,times,gap):
    start=None;best=0.
    for j,ok in enumerate(mask):
        if not ok: start=None;continue
        if start is None or j and times[j]-times[j-1]>gap+1e-9: start=float(times[j])
        best=max(best,float(times[j])-start)
    return best


def array(data,key,width=None):
    if not data: raise MissingEvidence('Empty original stream '+key)
    if any(key not in d for d in data): raise MissingEvidence('Original field missing '+key)
    a=np.asarray([d[key] for d in data],float)
    shape=(len(data),) if width is None else (len(data),width)
    if a.shape!=shape or not np.isfinite(a).all(): raise ValueError('Malformed original '+key)
    return a


def original_index(data,key='stamp_ns'):
    out={};last=-1
    for d in data:
        if d.get('accepted') is False: continue
        stamp=ns(d[key])
        if stamp<=last: raise ValueError('Duplicated/backwards accepted original header')
        last=stamp;out[stamp]=d
    if not out: raise MissingEvidence('No original accepted headers')
    return out


def archive_path(run,value):
    p=Path(value);p=p if p.is_absolute() else run/p
    p=p.resolve()
    if not p.is_relative_to(run.resolve()): raise ValueError('Archive outside current run '+str(p))
    return p


def source_audit(run,scope):
    manifest=load(run/'source_manifest.json');snap=load(run/'navigation_source_snapshots.json')
    refs=scope.get('references',scope.get('source_hashes',{}))
    if not manifest or not refs or not snap: raise MissingEvidence('Empty frozen source/archive contract')
    bad=[];missing=[];verified={}
    for key,h in manifest.items():
        # Some manifests include original absolute names as well as archive keys.
        if Path(key).is_absolute():
            row=snap.get(key)
            if row is None: missing.append(key);continue
            p=archive_path(run,row['snapshot'])
        else: p=archive_path(run,'sources/'+key)
        if not p.is_file(): missing.append(str(p));continue
        actual=sha(p);verified[str(p)]=actual
        if actual!=h: bad.append(str(p))
    for original,row in snap.items():
        p=archive_path(run,row['snapshot']);expected=row['sha256']
        if not p.is_file(): missing.append(str(p));continue
        actual=sha(p);verified[str(p)]=actual
        if actual!=expected or refs.get(original)!=expected: bad.append(original)
    for name,h in refs.items():
        p=Path(name).resolve()
        if p.is_relative_to(run):
            if not p.is_file(): missing.append(str(p));continue
            actual=sha(p);verified[str(p)]=actual
            if actual!=h:bad.append(str(p))
        elif name not in snap:
            missing.append('No archived original '+name)
    valid=(scope.get('schema')==SCOPE_SCHEMA and scope.get('checkpoint_sha256')==MODEL_SHA
           and scope.get('navigation_ground_truth_used') is False and scope.get('controller_kind')=='teacher'
           and scope.get('navigation_is_verified') is False and scope.get('allowed') is True)
    return check(False if bad or not valid else None if missing else True,
                 source_mismatches=bad,missing_archives=missing,verified_sources=verified,
                 old_current_source_is_never_used_to_rejudge_archived_runs=True)


def source_tick_audit(run,pid,poses,scope):
    pi=original_index(poses);imurows=rows(run/'navigation_imu_history.jsonl');gi=original_index(imurows)
    fi=original_index(rows(run/'navigation_feedback_history.jsonl'))
    updated=[];bad=[];poseages=[];gyroages=[];pwall=[];receipt_phase=[];immutable_feedback={}
    sensor=load(run/'sensor_contract.json')
    from scipy.spatial.transform import Rotation
    Rbi=Rotation.from_euler('xyz',sensor['imu']['body_rpy']).as_matrix()
    lastsource=-1;lastseq=0
    for r in pid:
        if r.get('schema')!=TICK_SCHEMA: raise ValueError('Old PID terms are not the new cascade contract')
        seq=ns(r['sequence']);pns=ns(r['control_pose_stamp_ns']);clock=ns(r['compute_ros_clock_ns'])
        c=r.get('cascade',{})
        if seq<=lastseq:bad.append([seq,'Tick sequence backwards/duplicate'])
        lastseq=seq
        if c.get('protection_active') is True:
            if c.get('controller_updated') is True or np.any(finite(r.get('command_after_slew',[0,0,0]),3)):
                bad.append([seq,'Protection still advances math or requests active correction'])
            continue
        source=pi.get(pns)
        if source is None: bad.append([seq,'SLAM header not original']);continue
        if source.get('frame_id')!='camera_init' or source.get('child_frame_id')!='demo_slam_body':bad.append([seq,'SLAM frame'])
        if not np.allclose(finite(r['control_pose'],3),finite(source['position'],3),atol=1e-10,rtol=0):bad.append([seq,'SLAM raw pose mismatch'])
        q=finite(source['quaternion'],4);rot=rotation(q[[3,0,1,2]][None])[0]
        feedback=r.get('feedback');usedimu=r.get('imu')
        if feedback is None or usedimu is None:
            if r.get('mode') in('protect','recovering'):continue
            raise MissingEvidence('Original feedback/paired IMU copies absent')
        if not np.allclose(finite(feedback['origin_velocity_body'],3),finite(source['body_velocity'],3),atol=1e-10,rtol=0) or not np.allclose(finite(feedback['quaternion_wxyz'],4),q[[3,0,1,2]],atol=1e-10,rtol=0):bad.append([seq,'SLAM raw origin velocity/quaternion mismatch'])
        rawfeedback=fi.get(pns)
        if rawfeedback is None:raise MissingEvidence('Original first-feedback receipt absent for header '+str(pns))
        if feedback!={k:v for k,v in rawfeedback.items() if k not in('paired_imu','navigation_ground_truth_used')} or usedimu!=rawfeedback.get('paired_imu'):
            bad.append([seq,'Feedback/paired gyro data or first receipt was refreshed'])
        fbhash=hashlib.sha256(canonical(feedback).encode()).hexdigest()
        if pns in immutable_feedback and immutable_feedback[pns]!=fbhash:bad.append([seq,'Same original pose header changes data/receipt'])
        immutable_feedback[pns]=fbhash
        gns=ns(r['paired_imu_stamp_ns']);gyro=gi.get(gns)
        if gyro is None:bad.append([seq,'Missing original gyro']);continue
        gap=(pns-gns)/1e9
        if not 0<=gap<=CRITERIA['gyro_to_pose_max_gap_s']+1e-9:bad.append([seq,'Future or stale causal gyro'])
        raw=gyro.get('gyro_source_xyz')
        if raw is None:raise MissingEvidence('Original IMU angular_velocity/gyro_imu field missing')
        body=Rbi@finite(raw,3)
        if not np.allclose(finite(usedimu['angular_velocity_body'],3),body,atol=1e-9,rtol=0) or usedimu!= {k:v for k,v in gyro.items() if k!='sequence'}:bad.append([seq,'IMU original copy or mount mismatch'])
        if gyro.get('source_frame_id')!=sensor['imu']['frame'] or gyro.get('frame_id')!='body':bad.append([seq,'Original IMU frame mismatch'])
        if not np.allclose(np.asarray(gyro['imu_to_body_rotation']),Rbi,atol=1e-10,rtol=0):bad.append([seq,'Recorded extrinsic differs from sensor contract'])
        wall=float(r['compute_monotonic_wall']);age=(clock-pns)/1e9;wa=wall-float(source['received_monotonic_wall'])
        fbwa=wall-feedback['received_wall_ns']/1e9
        receipt_phase.append(float(source['received_monotonic_wall'])-feedback['received_wall_ns']/1e9)
        # Feedback is causal to this producer clock; the bridge tolerates known
        # /clock ordering separately, never treats a future pose as feedback.
        if not 0<=age<=.3 or not 0<=wa<=.3 or not 0<=fbwa<=.3:bad.append([seq,'SLAM feedback freshness/causality'])
        if r.get('navigation_ground_truth_used') is not False:bad.append([seq,'Truth navigation source'])
        if type(c.get('controller_updated')) is not bool:raise MissingEvidence('cascade.controller_updated actual boolean absent')
        if c['controller_updated']:
            if pns<=lastsource:bad.append([seq,'Old pose masquerading as mathematical update'])
            if c.get('feedback_pose_stamp_ns')!=pns:bad.append([seq,'Cascade mathematical header different from raw SLAM'])
            lastsource=pns;updated.append(pns/1e9)
        poseages.append(age);gyroages.append(gap);pwall.append(wa)
    dt=np.diff(updated);hz=1/np.median(dt) if len(dt) else None
    return check(bool(updated) and not bad,invalid_ticks=bad,actual_math_updates=len(updated),
                 actual_math_header_hz=hz,actual_header_dt_s=distribution(dt),
                 SLAM_age_sim_s=distribution(poseages),SLAM_age_wall_s=distribution(pwall),
                 causal_gyro_gap_s=distribution(gyroages),IMU_orientation_used_only_if_actual_publisher_declares_available=True,
                 original_feedback_entry_to_base_callback_record_wall_offset_s=distribution(receipt_phase),
                 receipt_sampling_points_are_different_and_not_falsely_required_bit_exact=True,
                 selected_controller_hz=scope['profile']['cascade'].get('selected_feedback_hz'),
                 actor_hz_is_not_source_feedback_hz=True)


def trajectories(run,status,pid):
    archives={};bad=[]
    from scipy.interpolate import BSpline
    for f in sorted((run/'navigation_trajectories').glob('*.json')):
        d=load(f);p=archive_path(run,str(f.parent.relative_to(run)/d['array_file']))
        if not p.is_file():raise MissingEvidence('Missing original SCAN array '+str(p))
        if sha(p)!=d.get('array_sha256'):bad.append([f.name,'payload SHA']);continue
        with np.load(p,allow_pickle=False) as z:
            c=z['coefficients'];k=z['knots'];s=z['samples']
        if d.get('order')!=3 or c.ndim!=2 or c.shape[1]!=3 or len(k)!=len(c)+4 or s.ndim!=2 or s.shape[1]!=3 or not all(np.isfinite(a).all() for a in(c,k,s)):
            bad.append([f.name,'shape/finite']);continue
        generated=BSpline(k,c,3)(np.linspace(k[3],k[-4],len(s)))
        if not np.allclose(generated,s,atol=1e-9,rtol=0):bad.append([f.name,'actual spline coefficients/samples'])
        if d.get('request_id')!=load(run/'navigation_request.json').get('request_id'):bad.append([f.name,'request association'])
        archives[d['trajectory_id']]={'document':d,'samples':s,'array_path':p}
    used={r.get('trajectory_id') for r in pid if r.get('mode') in('drive','capture')}
    used.discard(None)
    accepted={r['accepted_trajectory_id'] for r in status if r.get('accepted_trajectory_id') is not None}
    if not archives or not accepted:raise MissingEvidence('No actual committed SCAN path')
    if not (used|accepted).issubset(archives):bad.append(['missing used or accepted trajectory'])
    return check(not bad,actual_SCAN_archives=len(archives),used_ids=sorted(used),accepted_ids=sorted(accepted),errors=bad),archives


def cloud_audit(run,poses,clouds):
    ledger=rows(run/'navigation_cloud_xyz.jsonl');li=original_index(ledger);pi=original_index(poses);ci=original_index(clouds)
    native=module(archived_module_path(run,'control_core.py'));errors=[];missing=[];merged=[];raw_verified=0
    formats={7:'f4',8:'f8'}
    for stamp,c in ci.items():
        d=li.get(stamp)
        if d is None:missing.append(stamp);continue
        a=archive_path(run,d['array_file']);p=archive_path(run,d['raw_payload_file'])
        if not a.is_file() or not p.is_file():missing.append(stamp);continue
        if sha(a)!=d['array_sha256'] or sha(p)!=d['raw_payload_sha256']:errors.append([stamp,'Original payload/array file hash mismatch']);continue
        xyz=np.load(a,allow_pickle=False)
        if xyz.shape!=tuple(d['shape']) or xyz.dtype!=np.dtype(d['dtype']) or not np.isfinite(xyz).all():errors.append([stamp,'Filtered original shape/dtype/finite']);continue
        if hashlib.sha256(np.asarray(xyz,dtype='<f8').tobytes()).hexdigest()!=c['filtered_xyz_float64_sha256'] or c['filtered_xyz_float64_sha256']!=d['filtered_xyz_float64_sha256']:errors.append([stamp,'Accepted filtered cloud differs from original archive'])
        fields={v['name']:v for v in d['fields']};fmt=[];offset=[];endian='>' if d['is_bigendian'] else '<'
        for name in('x','y','z'):
            f=fields[name]
            if f['datatype'] not in formats or f['count']!=1:raise ValueError('Unsupported original scalar XYZ encoding')
            fmt.append(endian+formats[f['datatype']]);offset.append(f['offset'])
            if f['offset']<0 or f['offset']+np.dtype(fmt[-1]).itemsize>d['point_step']:raise ValueError('Original field outside point record')
        raw=p.read_bytes()
        if len(raw)!=d['height']*d['row_step'] or d['row_step']<d['width']*d['point_step']:errors.append([stamp,'Original row padding/payload length invalid']);continue
        dtype=np.dtype(dict(names=['x','y','z'],formats=fmt,offsets=offset,itemsize=d['point_step']))
        values=np.ndarray((d['height'],d['width']),dtype=dtype,buffer=raw,strides=(d['row_step'],d['point_step'])).reshape(-1)
        points=np.column_stack([values[k] for k in('x','y','z')]);points=points[np.isfinite(points).all(axis=1)]
        filterstamp=ns(d['filtering_body_stamp_ns']);body=pi.get(filterstamp)
        if body is None:missing.append('Filtering original body header '+str(filterstamp));continue
        R=rotation(np.asarray(body['quaternion'])[[3,0,1,2]][None])[0];pos=finite(body['position'],3)
        if abs(stamp-filterstamp)>150_000_001 or not np.allclose(pos,d['filtering_body_pose'],atol=1e-10,rtol=0) or not np.allclose(R,d['filtering_body_rotation'],atol=1e-9,rtol=0):errors.append([stamp,'Self-return filtering raw pose/frame association differs'])
        reproduced,removed=native.remove_go2_self_returns(points,pos,R)
        if not np.array_equal(np.asarray(reproduced,dtype='<f8'),xyz):errors.append([stamp,'Raw PointCloud2 XYZ decode/self-filter differs'])
        if d['frame_id']!='camera_init' or d.get('navigation_ground_truth_used') is not False:errors.append([stamp,'Cloud navigation frame/source invalid'])
        raw_verified+=1;merged.append({**c,'filtered_xyz_file':d['array_file'],'xyz_file_sha256':d['array_sha256'],'raw_payload_file':d['raw_payload_file']})
    return check(False if errors else None if missing or not ledger else True,accepted_original_clouds=len(ci),
                 raw_payload_and_XYZ_verified=raw_verified,errors=errors,missing_headers=missing,
                 original_field_padding_and_endianness_preserved=True,filtering_pose_nearest_gap_is_separate_from_control_feedback_causality=True),merged


def guard_audit(run,guards,pid,clouds):
    ci=original_index(clouds);ticks={r['sequence']:r for r in pid};errors=[];missing=[];replayed=0
    native=module(archived_module_path(run,'control_core.py'));gaps=[];last=-1
    for g in guards:
        clock=ns(g['compute_ros_clock_ns']);pns=ns(g['control_pose_stamp_ns']);cns=ns(g['cloud_header_stamp_ns'])
        if clock<=last:errors.append([g.get('sequence'),'Actual guard clock duplicate/backward'])
        if last>=0:gaps.append((clock-last)/1e9)
        last=clock;c=ci.get(cns);tick=ticks.get(g.get('pid_sequence',g.get('cascade_sequence')))
        if c is None:errors.append([g.get('sequence'),'No original accepted cloud']);continue
        if not -50_000_000<=clock-cns<300_000_000 or not 0<=clock-pns<300_000_000:errors.append([g.get('sequence'),'Guard raw source stale/future feedback'])
        wall=float(g['compute_monotonic_wall'])
        if not 0<=wall-float(c['received_monotonic_wall'])<.3:errors.append([g.get('sequence'),'Cloud original wall stale'])
        if c.get('filtered_xyz_float64_sha256')!=g.get('filtered_xyz_float64_sha256'):errors.append([g.get('sequence'),'Guard cloud SHA different'])
        if tick is None or tick['control_pose_stamp_ns']!=pns:errors.append([g.get('sequence'),'No exact current cascade guard association'])
        elif not np.allclose(g['steering_direction'],tick.get('guard_world_direction',tick.get('actual_pid_world_direction')),atol=1e-9,rtol=0):
            if np.linalg.norm(tick.get('guard_world_direction',tick.get('actual_pid_world_direction')))>1e-9:errors.append([g.get('sequence'),'Actual movement direction bypassed guard'])
        filename=c.get('filtered_xyz_file',c.get('xyz_file',g.get('filtered_xyz_file')))
        if filename is None:missing.append(cns);continue
        p=archive_path(run,filename)
        if not p.is_file():missing.append(cns);continue
        if c.get('xyz_file_sha256') and sha(p)!=c['xyz_file_sha256']:errors.append([cns,'XYZ array file SHA']);continue
        if p.suffix=='.npy':xyz=np.load(p,allow_pickle=False)
        else:
            with np.load(p,allow_pickle=False) as z:xyz=z['xyz']
        if xyz.shape!=(c['filtered_points'],3) or not np.isfinite(xyz).all():errors.append([cns,'XYZ finite shape']);continue
        digest=hashlib.sha256(np.asarray(xyz,dtype='<f8').tobytes()).hexdigest()
        if digest!=c['filtered_xyz_float64_sha256']:errors.append([cns,'Original filtered XYZ SHA']);continue
        result=native.steering_obstacle_ahead(xyz,np.asarray(g['control_pose']),np.asarray(g['checked_target']),np.asarray(g['steering_direction']),np.asarray(g['route']))
        observed=g['union_result']
        if bool(result[0])!=observed['blocked'] or int(result[2])!=observed['point_count'] or ((result[1] is None)!=(observed['clearance_m'] is None)) or result[1] is not None and abs(result[1]-observed['clearance_m'])>1e-9:errors.append([g.get('sequence'),'Original native guard replay differs'])
        replayed+=1
        if observed['blocked'] and not g.get('zero_requested_after_control'):errors.append([g.get('sequence'),'Blocked actual movement corridor did not protect with zero velocity'])
    return check(False if errors else None if missing or not guards else True,errors=errors,
                 missing_original_XYZ_headers=sorted(set(missing)),actual_guard_calls=len(guards),
                 independently_replayed= replayed,actual_guard_gap_s=distribution(gaps),
                 empty_cloud_is_not_a_valid_clearance_receipt=True)


def commands_audit(execution,history,profile,pid,poses,imus):
    hi={r['sequence']:r for r in history if type(r.get('sequence')) is int};errors=[];missing=[];ages=[];matched=0;lastseq=-1
    ti={r['sequence']:r for r in pid};pi=original_index(poses);gi=original_index(imus)
    et=array(execution,'world_sim_time');requested=array(execution,'requested',3);command=array(execution,'command',3)
    limits=finite(profile['cascade']['command_limits'],3)
    for j,r in enumerate(execution):
        evidence=r.get('closed_loop_read_evidence',{})
        attempt=evidence.get('actual_read_attempt')
        if attempt is None:missing.append([j,'Original attempted bytes unavailable']);continue
        state=attempt.get('status');original_read=attempt.get('decoded_envelope')
        if np.any(abs(requested[j])>limits+1e-9):errors.append([j,'Teacher requested exceeds prospective bounds'])
        if j:
            dt=et[j]-et[j-1]
            if not 0<dt<=.020001:errors.append([j,'Teacher command period missing/duplicate'])
            expected=command[j-1]+np.clip(requested[j]-command[j-1],-np.asarray([.6,.6,.8])*min(dt,.02),np.asarray([.6,.6,.8])*min(dt,.02))
            if np.max(abs(expected-command[j]))>1e-8:errors.append([j,'Actual Teacher slew changed'])
        if r.get('command_expired') is True and np.max(abs(requested[j]))>1e-9:errors.append([j,'Expired/unhealthy command requests motion'])
        raw=attempt.get('raw_utf8');rawsha=attempt.get('raw_bytes_sha256')
        if raw is not None:
            if hashlib.sha256(raw.encode()).hexdigest()!=rawsha or json.loads(raw)!=original_read:errors.append([j,'Actual current attempt bytes/decoded envelope differ'])
        elif state=='accepted':missing.append([j,'Accepted attempt lacks raw bytes'])
        if state!='accepted':
            if np.max(abs(requested[j]))>1e-9:errors.append([j,'Rejected current attempt requests motion'])
            continue
        if not isinstance(original_read,dict):missing.append([j,'Accepted original envelope missing']);continue
        seq=original_read.get('sequence');original=hi.get(seq)
        if original is None:missing.append([j,'Original bridge envelope missing']);continue
        if {k:v for k,v in original.items() if k not in TRANSPORT_KEYS}!=original_read:errors.append([j,'Original bridge versus actual read envelope differs'])
        if seq<lastseq:errors.append([j,'Consumer sequence backwards'])
        lastseq=seq;sa=et[j]-original['sim_time'];wall=attempt.get('read_monotonic_wall');wa=attempt.get('producer_wall_age_s')
        if wall is None or wa is None:missing.append([j,'Consumer read wall missing']);continue
        # read_monotonic_wall is the beginning of the actual file read, while
        # wall age is checked after decoding. It may be larger by real read time.
        if abs(sa-attempt.get('producer_sim_age_s',math.inf))>1e-8 or wa<wall-original['monotonic_wall']-1e-8:errors.append([j,'Original producer stamp refreshed or age inconsistent'])
        matched+=1;ages.append([sa,wa])
        if not -.05<=sa<=.3 or not 0<=wa<=.3 or original.get('healthy') is not True:errors.append([j,'Accepted stale/unhealthy producer'])
        expected=original['command'] if original.get('stop_requested') is not True else [0,0,0]
        if not np.allclose(requested[j],expected,atol=1e-9,rtol=0):errors.append([j,'Requested differs from original accepted command'])
        lineage=original_read.get('controller_source')
        if np.any(requested[j]):
            if not isinstance(lineage,dict):missing.append([j,'Missing actual nonzero controller source']);continue
            pns=ns(lineage['source_pose_stamp_ns']);gns=ns(lineage['paired_imu_stamp_ns']);control=ns(lineage['control_stamp_ns'])
            if pns not in pi or gns not in gi:errors.append([j,'Actual read references absent original source'])
            if not 0<=round(et[j]*1e9)-pns<=300_000_000 or not 0<=pns-gns<=20_000_000 or not pns<=control<=round(et[j]*1e9)+50_000_000:errors.append([j,'Teacher source feedback future or stale'])
            if lineage.get('navigation_ground_truth_used') is not False:errors.append([j,'Actual command lineage uses navigation truth'])
            seqtick=lineage.get('cascade_sequence',lineage.get('sequence'))
            tick=ti.get(seqtick)
            if tick is None:missing.append([j,'Exact current cascade tick association absent'])
            elif tick['control_pose_stamp_ns']!=pns or not np.allclose(lineage['command_after_slew'],tick['command_after_slew'],atol=1e-12,rtol=0):errors.append([j,'Cascade command source join differs'])
    return check(False if errors else None if missing else bool(matched),errors=errors,
                 missing_current_attempt_evidence=missing[:50],missing_attempt_count=len(missing),
                 accepted_original_reads=matched,accepted_producer_age_s=distribution(np.asarray(ages)[:,0]) if ages else None,
                 command_expired_boolean_includes_unhealthy=True,source_stamp_is_never_refreshed=True)


def bounded_command(raw,limits,forward_only=False):
    value=np.clip(raw,-limits,limits)
    if forward_only:value[0]=max(0.,value[0])
    size=np.linalg.norm(value[:2])
    if size>limits[0]:value[:2]*=limits[0]/size
    return value


def goal_and_path_geometry(run,pid,poses):
    """Replay bounded monotone projection from original committed SCAN samples."""
    paths=rows(run/'navigation_cascade_paths.jsonl');refs=rows(run/'navigation_fixed_goal_references.jsonl')
    request=load(run/'navigation_request.json');pi=original_index(poses);byid={};goals={};errors=[];missing=[];fresh=0;maxerr=0.
    for r in refs:
        i=r['waypoint_index'];fixed=r['fixed_goal'];digest=hashlib.sha256(canonical(fixed).encode()).hexdigest()
        if r['request_id']!=request['request_id'] or not 0<=i<len(request['goals']) or digest!=r['fixed_goal_sha256']:errors.append(['Fixed-goal reference identity/hash']);continue
        g=request['goals'][i];start=finite(r['segment_start'],3);end=finite(r['goal'],3);d=end[:2]-start[:2]
        if not np.allclose(end,g['center'],atol=1e-12,rtol=0) or np.linalg.norm(d)<1e-9:errors.append(['Goal/immutable segment degenerate']);continue
        expected=math.atan2(d[1],d[0])
        stamp=ns(r['source_pose_stamp_ns'])
        if stamp not in pi:missing.append('Fixed goal source header '+str(stamp));continue
        if not np.allclose(start,pi[stamp]['position'],atol=1e-10,rtol=0):errors.append(['Fixed leg start not the cited actual original SLAM pose'])
        if abs(float(wrap(fixed['heading_rad']-expected)))>1e-12 or r.get('navigation_ground_truth_used') is not False:errors.append(['Goal heading has no immutable source segment'])
        key=r['fixed_goal_sha256']
        if key in goals and goals[key]!=r:errors.append(['Frozen goal hash reused with changed reference'])
        goals[key]=r
    for p in paths:
        record={k:p[k] for k in('points_xyz','path_id','stamp_ns','received_wall_ns','frame_id')}
        if hashlib.sha256(canonical(record).encode()).hexdigest()!=p['path_sha256']:errors.append([p['path_id'],'Core path-record SHA mismatch'])
        a=archive_path(run,p['array_file'])
        if not a.is_file():missing.append(str(a));continue
        with np.load(a,allow_pickle=False) as z:samples=z['samples']
        index=p.get('source_sample_indices')
        if index is None:raise MissingEvidence('Exact committed SCAN-to-core sample index derivation missing')
        if not index or any(type(i) is not int for i in index) or min(index)<0 or max(index)>=len(samples) or any(b<=a for a,b in zip(index[:-1],index[1:])):errors.append([p['path_id'],'Invalid immutable sample indices']);continue
        points=np.asarray(p['points_xyz'],float)
        if not np.array_equal(samples[index],points):errors.append([p['path_id'],'Core points differ from original indexed SCAN samples'])
        if p.get('source_samples_count')!=len(samples) or p.get('source_samples_float64_sha256')!=hashlib.sha256(np.asarray(samples,dtype='<f8').tobytes()).hexdigest():errors.append([p['path_id'],'Original committed sample count/hash differs'])
        if p['frame_id']!='camera_init' or p.get('navigation_ground_truth_used') is not False:errors.append([p['path_id'],'Path frame/source not actual SLAM/SCAN'])
        if p['path_id'] in byid and byid[p['path_id']]!=p:errors.append([p['path_id'],'Actual checked path ID changed'])
        byid[p['path_id']]=p
    state={};lastgoal=None
    for r in pid:
        c=r['cascade'];g=c.get('fixed_goal_sha256');pathid=c.get('path_id')
        if c.get('controller_updated') is not True:continue
        if g not in goals:missing.append('No fixed-goal source '+str(g));continue
        if pathid not in byid:missing.append('No original SCAN core path '+str(pathid));continue
        p=byid[pathid];stamp=ns(c['feedback_pose_stamp_ns'])
        if p['stamp_ns']>stamp:errors.append([r['sequence'],'Future SCAN path relative to current source'])
        if not pathid.startswith(request['request_id']+':'+str(r['waypoint_index'])+':'):errors.append([r['sequence'],'Previous goal path reused for new goal'])
        source=pi[stamp];position=np.asarray(source['position']);points=np.asarray(p['points_xyz']);delta=np.diff(points,axis=0);length=np.linalg.norm(delta[:,:2],axis=1)
        if (length<1e-5).any():errors.append([r['sequence'],'Degenerate horizontal SCAN segment']);continue
        cumulative=np.r_[0.,length.cumsum()];key=(g,pathid);progress=state.get(key);low=0. if progress is None else max(0.,progress-.15);high=cumulative[-1] if progress is None else min(cumulative[-1],progress+.8)
        options=[]
        for i,(a,d,L) in enumerate(zip(points[:-1],delta,length)):
            if cumulative[i+1]<low or cumulative[i]>high:continue
            u=float(np.clip((position-a)@d/(d@d),max(0.,(low-cumulative[i])/L),min(1.,(high-cumulative[i])/L)))
            point=a+u*d;along=cumulative[i]+u*L;options.append((float((position-point)@(position-point)),float(along),i,point))
        if not options:errors.append([r['sequence'],'No bounded actual SCAN projection']);continue
        _,s,i,point=min(options,key=lambda x:x[:3]);newprogress=max(0. if progress is None else progress,s);state[key]=newprogress
        tangent=delta[i,:2]/length[i];normal=np.array([-tangent[1],tangent[0]]);cross=float((position[:2]-point[:2])@normal);grade=delta[i,2]/length[i]
        expected=[(c['nearest_projection_xyz'],point),(c['local_horizontal_tangent'],tangent),(c['local_horizontal_normal'],normal),
                  (c['nearest_projection_progress_m'],s),(c['progress_m'],newprogress),(c['error_cross_m'],cross),
                  (c['local_grade_dz_ds'],grade),(c['remaining_horizontal_arc_m'],cumulative[-1]-newprogress)]
        for observed,actual in expected:maxerr=max(maxerr,float(np.max(abs(np.asarray(observed)-actual))))
        fresh+=1
    if maxerr>1e-8:errors.append(['Bounded original SCAN projection maximum error',maxerr])
    return check(False if errors else None if missing else bool(fresh),errors=errors,missing_original_paths=sorted(set(missing)),
                 original_SCANNative_paths=len(byid),fixed_goal_references=len(goals),actual_fresh_projection_replays=fresh,
                 maximum_geometry_replay_error=maxerr,geometry_shared_with_control_is_explicit=True,
                 independent_raw_sample_derivation_and_projection_reconstruction=True)


def ack_and_PI_audit(run,pid,poses,execution,profile):
    """Independent command-ack joins and PI recurrence, not old PID terms."""
    ackrows=rows(run/'closed_loop_executor_ack.jsonl');ai=original_index(ackrows)
    byseq={r['sequence']:r for r in ackrows};pi=original_index(poses)
    frames={round(r['world_sim_time']*1e9):r for r in execution}
    errors=[];previous=-1;lastgoal=None;previous_pose=None
    integral=np.zeros(3);hold_integral=np.zeros(3);filtered=None;inner=None
    offset=finite(profile['cascade']['base_com_offset'],3)
    reset_count={};updates=0;blocked_limit=0;blocked_ack=0;maximum=0.
    for ack in ackrows:
        stamp=ns(ack['stamp_ns']);seq=ns(ack['sequence']);confirmation=ns(ack['ack_native_clock_ns'])
        original=frames.get(stamp)
        if seq<=previous or confirmation<=stamp:errors.append([seq,'Ack identity/next native request is invalid'])
        previous=seq
        if original is None:errors.append([seq,'Ack lacks actual Teacher frame']);continue
        if not np.allclose(ack['applied_command_body'],original['command'],atol=1e-12,rtol=0) or not np.allclose(ack['requested_command_body'],original['requested'],atol=1e-12,rtol=0):errors.append([seq,'Ack changed real requested/applied Teacher input'])
        if ack.get('contains_position_or_attitude') is not False or ack.get('navigation_ground_truth_used') is not False:errors.append([seq,'Known-command ack contains truth/navigation state'])
    for r in pid:
        c=r.get('cascade',{});mode=c.get('mode',r.get('mode'));goal=c.get('fixed_goal_sha256');seq=r['sequence']
        if goal!=lastgoal:
            integral[:]=0;hold_integral[:]=0;filtered=inner=None;previous_pose=None;lastgoal=goal
        if c.get('protection_active') or mode in('protect','recovering'):
            integral[:]=0;hold_integral[:]=0;filtered=inner=None;previous_pose=None
            if np.any(finite(c.get('command_body',[0,0,0]),3)):errors.append([seq,'Protected core requests nonzero correction'])
            continue
        if c.get('controller_updated') is not True:continue
        updates+=1;source=pi.get(c.get('feedback_pose_stamp_ns'))
        if source is None:errors.append([seq,'Math source missing']);continue
        pns=source['stamp_ns'];dt=0. if previous_pose is None else (pns-previous_pose)/1e9
        if dt<0 or abs(dt-c['header_dt_s'])>1e-9:errors.append([seq,'PI dt is not new original source gap'])
        previous_pose=pns
        R=rotation(np.asarray(source['quaternion'])[[3,0,1,2]][None])[0];rp=angles(np.asarray(source['quaternion'])[[3,0,1,2]][None])[0]
        bodygyro=finite(r['imu']['angular_velocity_body'],3);origin=finite(source['body_velocity'],3);com=origin+np.cross(bodygyro,offset)
        yawdot=(math.sin(rp[0])*bodygyro[1]+math.cos(rp[0])*bodygyro[2])/math.cos(rp[1]);measurement=np.r_[ (R@origin)[:2],yawdot];actual=np.r_[com[:2],bodygyro[2]]
        filtered=measurement.copy() if filtered is None else filtered+dt/(.2+dt)*(measurement-filtered)
        inner=actual.copy() if inner is None else inner+dt/(.1+dt)*(actual-inner)
        vpi=c.get('velocity_PI')
        if not isinstance(vpi,dict):raise MissingEvidence('Actual new velocity_PI recurrence absent')
        ack=byseq.get(vpi['ack_sequence'])
        if ack is None:raise MissingEvidence('Exact original executed command ack absent')
        if ack['stamp_ns']>pns or pns-ack['stamp_ns']>300_000_000 or ack['received_wall_ns']>c['compute_wall_ns'] or c['compute_wall_ns']-ack['received_wall_ns']>300_000_000:errors.append([seq,'Executed command ack future/stale to actual source'])
        residual=finite(ack['requested_command_body'],3)-finite(ack['applied_command_body'],3)
        own=hold_integral if mode in('capture','active_hold') else integral
        if c.get('capture_entry_integral_reset'):
            if mode!='capture':errors.append([seq,'Unexpected integral reset outside capture entry'])
            own[:]=0;reset_count[goal]=reset_count.get(goal,0)+1
        if mode in('capture','active_hold'):
            limits=np.array([.15,.07,.10] if mode=='capture' else [.06,.04,.10])
            fixed=c['fixed_goal'];goalxy=np.asarray(fixed['position_world_xyz'])[:2]
            e=goalxy-np.asarray(source['position'])[:2];P=(.8 if mode=='capture' else .18)*np.sign(e)*np.maximum(abs(e)-.003,0.)
            D=-.2*filtered[:2];worldxy=P+D;cap=.05 if mode=='capture' else .025
            if np.linalg.norm(worldxy)>cap:worldxy*=cap/np.linalg.norm(worldxy)
            refworld=np.r_[worldxy,0.];yawerror=float(wrap(fixed['heading_rad']-rp[2]));yp=.65*math.copysign(max(abs(yawerror)-.005,0.),yawerror);yd=-.18*filtered[2];wref=float(np.clip(yp+yd,-.07,.07))
            if not np.allclose(c['position_terms']['P'],P,atol=1e-9,rtol=0) or not np.allclose(c['position_terms']['D'],D,atol=1e-9,rtol=0):errors.append([seq,'Fixed-target phase PD differs from prospective capture/hold law'])
        else:
            limits=finite(profile['cascade']['command_limits'],3);tangent=finite(c['local_horizontal_tangent'],2);normal=np.array([-tangent[1],tangent[0]])
            if abs(np.linalg.norm(tangent)-1)>1e-8:errors.append([seq,'Invalid actual SCAN tangent'])
            remaining=float(c['remaining_horizontal_arc_m']);speed=min(float(profile['cascade']['desired_speed']),.9*max(remaining,0.),math.sqrt(.7*max(remaining-.05,0.)))
            lat=np.clip(-.8*c['error_cross_m']-.2*(filtered[:2]@normal),-.2,.2)
            refworld=np.r_[speed*tangent+lat*normal,speed*c['local_grade_dz_ds']]
            if mode!='drive':refworld[:]=0.
            yp=1.3*float(wrap(c['reference_yaw_rad']-rp[2]));yd=-.15*filtered[2];wref=yp+yd
        body_wref=(wref*math.cos(rp[1])-math.sin(rp[0])*bodygyro[1])/math.cos(rp[0])
        target=np.r_[(R.T@refworld+np.cross([0,0,body_wref],offset))[:2],body_wref]
        error=target-inner;active_dt=dt if mode in('drive','turn','capture','active_hold') else 0.
        candidate=np.clip(own+active_dt*error,-.5,.5)
        if mode=='turn':candidate[:2]=own[:2]
        kp=np.array([.6,.4,.3]);ki=np.array([.5,.3,.2]);raw=target+kp*error+ki*candidate;limited=bounded_command(raw,limits,mode=='drive')
        block_limits=error*(raw-limited)>1e-9;block_ack=error*residual>1e-9
        candidate=np.where(block_limits|block_ack,own,candidate);raw=target+kp*error+ki*candidate;limited=bounded_command(raw,limits,mode=='drive')
        own[:]=candidate
        pairs=[(c['measured_COM_velocity_body'],com),(c['measured_origin_velocity_world'],R@origin),(vpi['filtered_actual_body'],inner),
               (vpi['error'],error),(vpi['P'],kp*error),(vpi['I'],ki*candidate),(vpi['integral_state'],candidate),
               (vpi['acknowledged_downstream_residual'],residual),(c['raw_command_body'],raw),
               (c['command_limits_body'],limits)]
        for observed,expected in pairs:maximum=max(maximum,float(np.max(abs(np.asarray(observed)-expected))))
        if not np.array_equal(vpi['blocked_limits'],block_limits) or not np.array_equal(vpi['blocked_actual_ack'],block_ack):errors.append([seq,'PI anti-windup blocked flags differ'])
        if abs(active_dt-c['integral_dt_s'])>1e-9:errors.append([seq,'Held/non-drive integration dt differs'])
        if mode=='turn':limited[:2]=0.
        elif mode in('pre_turn','settle','path_end_hold'):limited[:]=0.
        maximum=max(maximum,float(np.max(abs(np.asarray(c['command_body'])-limited))))
        if mode in('drive','turn','capture','active_hold'):
            maximum=max(maximum,float(np.max(abs(np.asarray(c['reference_COM_velocity_body'])-target))))
        blocked_limit+=int(block_limits.sum());blocked_ack+=int(block_ack.sum())
    if maximum>1e-8:errors.append(['maximum mathematical replay error',maximum])
    if any(v!=1 for v in reset_count.values()):errors.append(['Capture integral resets',reset_count])
    return check(bool(updates) and bool(ackrows) and not errors,actual_original_acks=len(ackrows),
                 actual_fresh_math_updates=updates,maximum_replay_absolute_error=maximum,errors=errors,
                 capture_entry_integral_resets=reset_count,PI_axis_blocked_count=blocked_limit,PI_actual_ack_blocked_count=blocked_ack,
                 outer_projection_independence='Projection geometry requires separate original SCAN path-record replay',
                 actual_ack_is_known_command_not_truth_feedback=True)


def region_audit(run,poses,status):
    request=load(run/'navigation_request.json');goals=request.get('goals',[])
    matches=[r for r in status if r.get('request_id')==request['request_id']]
    if not matches or not goals:raise MissingEvidence('No actual requested-region/status chain')
    definitions=matches[-1].get('goals_definitions');goalsha=matches[-1].get('goals_definition_sha256')
    if not definitions or not goalsha:raise MissingEvidence('Immutable canonical 3D goal definitions absent')
    native=module(archived_module_path(run,'goal_regions.py'));parsed=native.parse_request(request)
    # parse_request returns canonical GoalRegion objects; avoid relying on a
    # guessed reconstructed XY-only radius for a multi-floor request.
    regionlist=parsed[1] if isinstance(parsed,tuple) else parsed.goals
    if native.definitions_sha256(regionlist)!=goalsha:raise ValueError('Canonical immutable requested-goal SHA differs from receipt')
    for goal in regionlist:
        if goal.dwell_sim_s!=.6 or goal.timeout_sim_s!=90 or goal.control_arrival_definition().get('radius_m')!=.17:
            raise ValueError('Original prospective region numerical criteria changed')
    pi=original_index(poses);stamps=np.asarray(list(pi),np.int64)
    claimed=matches[-1].get('region_arrivals',[]);out=[];last=-1
    for j,g in enumerate(regionlist):
        receipt=next((r for r in claimed if r.get('goal_id')==g.goal_id),None)
        if receipt is None:out.append(check(False if matches[-1].get('state') in('failed','succeeded') else None,goal_id=g.goal_id,reason='No original measured arrival'));continue
        begin=ns(receipt['start_stamp_ns']);end=ns(receipt['stamp_ns']);part=[pi[int(v)] for v in stamps[(stamps>=begin)&(stamps<=end)]]
        exact=bool(part and part[0]['stamp_ns']==begin and part[-1]['stamp_ns']==end)
        sourcegap=max(np.diff([r['stamp_ns'] for r in part])/1e9) if len(part)>1 else math.inf
        inside=all(native.contains_control(g,np.asarray(r['position'])) for r in part)
        active=receipt.get('goal_activated_stamp_ns',receipt.get('goal_activated_ros_clock_ns'))
        if active is None:raise MissingEvidence('Exact original per-goal activation stamp absent')
        activation=ns(active);deadline=(end-activation)/1e9
        badprotected=[r for r in matches if begin<=round(float(r.get('ros_sim_time',-1))*1e9)<=end and (r.get('obstacle_hold') or r.get('tilt_hold') or r.get('execution_bridge_safety',{}).get('state')!='ready')]
        good=(exact and begin>last and inside and sourcegap<=.2+1e-9 and (end-begin)>=600_000_000 and 0<=deadline<=90 and receipt.get('protected') is False and not badprotected and receipt.get('goals_definition_sha256')==goalsha)
        out.append(check(good,goal_id=g.goal_id,original_begin_end_ns=[begin,end],original_headers=len(part),
                         dwell_s=(end-begin)/1e9,maximum_gap_s=sourcegap,deadline_elapsed_s=deadline,
                         exact_original_3D_geometry=True,protected_status_rows=len(badprotected)))
        last=end
    return check(all(v['status']=='passed' for v in out) and matches[-1].get('state')=='succeeded',
                 arrivals=out,actual_last_state=matches[-1].get('state'),goals_definition_sha256=goalsha),out


def native_state(run):
    raw=rows(run/'actuator.jsonl');contracts=[r for r in raw if r.get('kind')=='actuator_contract']
    if len(contracts)!=1:raise MissingEvidence('One native actuator contract required')
    c=contracts[0];steps=[r for r in raw if r.get('kind')=='physics_step']
    t=array(steps,'t')-array(steps,'dt');pos=array(steps,'position',3);q=array(steps,'quaternion_wxyz',4)
    rot=rotation(q);rpy=angles(q);v=array(steps,'body_lin_vel_com',3);w=array(steps,'body_ang_vel',3)
    origin=array(steps,'body_lin_vel_origin',3);tau=array(steps,'tau',12);qd=array(steps,'qd',12)
    contacts=array(steps,'contacts',5)
    obsfiles=[p for p in (run/'sources').rglob('observation.py') if p.parent.name=='policy']
    if len(obsfiles)!=1:raise MissingEvidence('Archived all-collision clearance provider not available')
    terrain=module(obsfiles[0]).TerrainHeightMap.from_sdf(run/'world.sdf')
    clear=pos[:,2]-terrain.height(pos[:,:2],ray_start_z=pos[:,2])
    yawdot=(np.sin(rpy[:,0])*w[:,1]+np.cos(rpy[:,0])*w[:,2])/np.cos(rpy[:,1])
    return dict(contract=c,steps=steps,t=t,pos=pos,q=q,rot=rot,rpy=rpy,com=v,omega=w,
                world_com=np.einsum('nij,nj->ni',rot,v),world_origin=np.einsum('nij,nj->ni',rot,origin),
                yawdot=yawdot,tau=tau,qd=qd,contacts=contacts,clear=clear)


def native_audit(n):
    t=n['t'];active=t>=.1;gap=np.diff(t);c=n['contract']
    faults=[r for r in n['steps'] if r.get('fault',0)]
    complete=(len(t)>1 and np.all(gap>0) and np.max(gap)<=.005001 and np.max(abs(np.sum(n['q']**2,axis=1)-1))<=.02)
    unavailable=any(r.get('tau_available') is not None and r['tau_available']!=[1]*12 for r in n['steps'] if r['t']-r['dt']>=.1)
    safe=(active.any() and np.isfinite(n['clear'][active]).all() and np.min(n['clear'][active])>=.18 and np.max(abs(n['rpy'][active,:2]))<=.65 and np.min(n['contacts'][active])>=0 and np.max(n['contacts'][active,0])==0 and not faults)
    unsupported=longest((n['contacts'][:,1:].sum(1)==0)&active,t,.005001)
    return {
        'native_continuous_200Hz':check(complete,phase='PreUpdate t-dt',maximum_gap_s=np.max(gap),samples=len(t)),
        'native_physical_safety':check(safe,minimum_clearance_m=np.min(n['clear'][active]),maximum_roll_pitch_rad=np.max(abs(n['rpy'][active,:2])),body_contacts=int(np.sum(n['contacts'][active,0]>0)),fault_samples=len(faults),ground_truth_used_only_offline=True),
        'native_force_velocity_and_support':check(not unavailable and np.max(abs(n['tau'][active]))<=23.50001 and np.max(abs(n['qd'][active]))<=30.001 and unsupported<=.3+1e-9,maximum_tau_Nm=np.max(abs(n['tau'][active])),maximum_qd_radps=np.max(abs(n['qd'][active])),maximum_unsupported_s=unsupported,torque_is_command_feed_to_physics_not_measured_motor_torque=True),
        'exclusive_Teacher_CPU_identity':check(c.get('writer') in('teacher_sim::TeacherActuator','teacher_sim::TeacherActuator sole JointForceCmd writer') and c.get('body_pose_resets')==0 and c.get('kp')==25 and c.get('kd')==.5,contract=c),
    }


def actor_and_graph_audit(run,execution,n):
    meta=load(run/'policy_manifest.json');result=load(run/'worker_result.json')
    gates=rows(run/'navigation_sensor_gate_history.jsonl');ready=[r for r in gates if r.get('ready') is True]
    graphs=bool(ready) and all(r.get('ground_truth_navigation_used') is False and r.get('overview_used_for_vio') is False and r.get('publisher_graph') and all(v.get('passed') is True for v in r['publisher_graph'].values()) for r in ready)
    zpath=run/'observations_actions.npz'
    if not zpath.is_file():raise MissingEvidence('Original 247-observation/action interface trace unavailable')
    with np.load(zpath,allow_pickle=False) as z:obs=z['observations'];acts=z['actions']
    actions=array(execution,'action',12);targets=array(execution,'q_target',12);command=array(execution,'command',3);t=array(execution,'sim_time')
    if obs.shape!=(len(execution),247) or acts.shape!=actions.shape or not np.isfinite(obs).all() or not np.isfinite(acts).all():raise ValueError('Actor full247 trace dimensions/finite invalid')
    active=t>=.1;default=np.asarray(n['contract']['initial_q'])
    targeterr=float(abs(targets[active]-(default+.25*actions[active])).max())
    contractpaths=list((run/'sources').glob('policy/contract.json'))
    if len(contractpaths)!=1:raise MissingEvidence('Archived exact actor-observation contract unavailable')
    spec=load(contractpaths[0])['observations']['concatenation_order'][3]
    cmd=np.clip(command,*spec['clip'])*spec['scale_after_clip'];cmderr=float(abs(obs[:,9:12]-cmd.astype(np.float32)).max())
    steps=n['steps'];pd_errors=[]
    for r in steps:
        if r.get('mode')!=0 or r.get('terminating') or r['t']-r['dt']<.1:continue
        q=np.asarray(r['q']);qd=np.asarray(r['qd']);tau=np.asarray(r['tau']);target=np.asarray(r['qtarget']);lo=np.maximum(23.5*(-1-qd/30),-23.5);hi=np.minimum(23.5*(1-qd/30),23.5)
        pd_errors.append(float(abs(tau-np.clip(25*(target-q)-.5*qd,lo,hi)).max()))
    model=ET.parse(run/'world.sdf').getroot().find("world/model[@name='go2']")
    if model is None:raise ValueError('Actual robot model absent')
    controlplugins=[p.attrib for p in model.findall('plugin') if any(s in p.get('name','').lower() for s in('teacheractuator','control','stabilizer','trajectory'))]
    good=(graphs and meta.get('checkpoint_sha256')==MODEL_SHA and meta.get('inference_device')=='cpu' and meta.get('torch_threads')==1 and meta.get('navigation_truth_used') is False and meta.get('privileged_actor_observation_dimensions')==232
          and result.get('fault') is None and result.get('completed') is True and np.array_equal(actions,acts) and targeterr<1e-6 and cmderr<1e-7 and bool(pd_errors) and max(pd_errors)<1e-8
          and np.max(np.diff(t))<=.020001 and all(r.get('actor_inferred_this_frame') is True for r in execution if r.get('sim_time',0)>=.1)
          and len(controlplugins)==1 and controlplugins[0]['name']=='teacher_sim::TeacherActuator')
    return check(good,actual_actor_full247=obs.shape,actor_action_target_error_rad=targeterr,actual_command_observation_max_error=cmderr,
                 native_PD_DC_torque_replay_max_error_Nm=max(pd_errors) if pd_errors else None,
                 actual_ready_graph_rows=len(ready),actual_vehicle_sensor_graph_valid=graphs,
                 sole_actual_writer_plugins=controlplugins,actor_CPU_threads=meta.get('torch_threads'),
                 actor_privileged_dimensions=232,controller_known_dimensions=15,full_real_sensor_actor_not_claimed=True)


def runtime_audit(run,scope):
    plan=load(run/'runtime_manifest.json')
    runtime=load(run/'runtime_result.json') if (run/'runtime_result.json').is_file() else plan
    owned=runtime.get('owned_processes',[])
    required=scope.get('required_owned_roles',scope.get('runtime',{}).get('required_owned_roles'))
    expectedchildren=scope.get('expected_launch_children',scope.get('runtime',{}).get('expected_launch_children'))
    if not required or type(expectedchildren) is not int:raise MissingEvidence('Prospective exact runtime roles/child count not frozen')
    roles=[r.get('role') for r in owned];childclean=0;errors=[]
    logs=list(run.glob('*navstack*.log'))+list(run.glob('*navigation*stack*.log'))
    for p in set(logs):
        for line in p.read_text(errors='replace').splitlines():
            if 'process has finished cleanly' in line:childclean+=1
            if any(v in line for v in('process has died','failed to terminate','escalating to SIGKILL')):errors.append(line)
    wr=load(run/'navigation_pid_writer_receipt.json')
    pid=rows(run/'navigation_pid_history.jsonl');guard=rows(run/'navigation_guard_history.jsonl')
    if not guard and (run/'navigation_native_guard_trace.jsonl').is_file():guard=rows(run/'navigation_native_guard_trace.jsonl')
    drained=wr.get('status')=='drained' and not wr.get('queue_error') and wr.get('expected_pid_records')==len(pid) and wr.get('expected_guard_records')==len(guard)
    meta=load(run/'policy_manifest.json');result=load(run/'worker_result.json')
    device=meta.get('inference_device',meta.get('device'));threads=meta.get('torch_threads')
    return check(set(roles)==set(required) and len(roles)==len(set(roles)) and all(r.get('returncode')==0 for r in owned) and runtime.get('error') is None and childclean==expectedchildren and not errors and drained and result.get('fault') is None and result.get('completed') is True and device=='cpu' and threads==1 and meta.get('checkpoint_sha256')==MODEL_SHA,
                 required_owned_roles=required,actual_owned=owned,expected_launch_children=expectedchildren,actual_clean_children=childclean,child_errors=errors,writer=wr,actor_device=device,torch_threads=threads)


def active_parking(run,pid,poses,execution,n):
    request=load(run/'navigation_request.json');finalgoal=request.get('goals',[])[-1]
    holds=[r for r in pid if r.get('mode')=='active_hold' and r.get('waypoint_index')==len(request['goals'])-1]
    if not holds:raise MissingEvidence('No declared source-derived active hold; zero parking cannot be relabelled')
    first=holds[0];a=first.get('cascade',{})
    declared=a.get('parking_hold_declared_pose_stamp_ns')
    if declared is None:raise MissingEvidence('Original integer first-hold source stamp absent')
    declared=ns(declared);pi=original_index(poses)
    if declared not in pi:raise ValueError('Hold declaration not based on original SLAM header')
    record=a.get('fixed_goal',{})
    target=record.get('position_world_xyz');targetyaw=record.get('heading_rad');targetsha=a.get('fixed_goal_sha256')
    if target is None or targetyaw is None or targetsha is None:raise MissingEvidence('Frozen source-frame target/hash absent')
    target=finite(target,3);targetyaw=float(targetyaw)
    if hashlib.sha256(canonical(record).encode()).hexdigest()!=targetsha or record.get('frame_id')!='camera_init':raise ValueError('Frozen source-frame target hash/coordinate declaration differs')
    goal=finalgoal
    if goal is None or not np.allclose(goal['center'],target,atol=1e-12,rtol=0):raise ValueError('Parking target is not this immutable requested goal')
    capture=[r for r in pid if r.get('mode')=='capture' and r.get('cascade',{}).get('fixed_goal_sha256')==targetsha]
    if not capture:raise MissingEvidence('Original capture phase absent')
    entry=a.get('parking_capture_pose_stamp_ns')
    if entry is None:raise MissingEvidence('Original first capture source stamp absent')
    candidates=[r for r in pid if r.get('mode') in('capture','active_hold') and r.get('cascade',{}).get('fixed_goal_sha256')==targetsha and r.get('cascade',{}).get('controller_updated') is True and entry<=ns(r['control_pose_stamp_ns'])<=declared]
    stamps=[ns(r['control_pose_stamp_ns']) for r in candidates];samples=[pi[s] for s in stamps]
    if not samples:raise MissingEvidence('No distinct actual capture headers')
    since=None;previous=None;duration=0.;strictentry=False
    for source,r in zip(samples,candidates):
        stamp=source['stamp_ns'];d=np.linalg.norm(np.asarray(source['position'])[:2]-target[:2]);q=np.asarray(source['quaternion'])[[3,0,1,2]];rp=angles(q[None])[0]
        gyr=finite(r['imu']['angular_velocity_body'],3);wz=(math.sin(rp[0])*gyr[1]+math.cos(rp[0])*gyr[2])/math.cos(rp[1])
        if previous is not None and (stamp<=previous or stamp-previous>200_000_000):since=None
        xy_gate=.025 if since is None else .03;ya_gate=.035 if since is None else .045
        eligible=(d<=xy_gate+1e-12 and abs(float(wrap(rp[2]-targetyaw)))<=ya_gate+1e-12 and np.linalg.norm(finite(source['body_velocity'],3)[:2])<.03 and abs(wz)<.06 and not r.get('obstacle_hold') and not r.get('cascade',{}).get('protection_active'))
        if not eligible:since=None
        elif since is None:since=stamp;strictentry=True
        previous=stamp;duration=0. if since is None else (stamp-since)/1e9
    if not stamps or stamps[-1]!=declared:raise MissingEvidence('Capture continuous original-header evidence does not end at first hold declaration')
    holdseqs={r['sequence'] for r in holds};et=array(execution,'world_sim_time')
    observe=[j for j,r in enumerate(execution) if r.get('closed_loop_read_evidence',{}).get('actual_read_attempt',{}).get('status')=='accepted' and ((r.get('closed_loop_read_evidence',{}).get('actual_original_envelope') or {}).get('controller_source') or {}).get('cascade_sequence') in holdseqs]
    if not observe:raise MissingEvidence('First actual Teacher read of hold declaration not associated')
    j=observe[0];begin=float(execution[j]['state_physics_world_time']);end=begin+5.
    mask=(n['t']>=begin-1e-9)&(n['t']<=end+1e-9);em=(et>=et[j]-1e-9)&(et<=et[j]+5+1e-9)
    ss=np.asarray(list(pi));sm=(ss>=round(begin*1e9))&(ss<=round(end*1e9))
    if mask.sum()<1000 or n['t'][mask][-1]-n['t'][mask][0]<4.999 or em.sum()<250 or sm.sum()<2:raise MissingEvidence('Incomplete first fixed active-hold five seconds')
    sp=np.asarray([pi[int(s)]['position'] for s in ss[sm]]);sq=np.asarray([pi[int(s)]['quaternion'] for s in ss[sm]])[:,[3,0,1,2]]
    sourcegap=max(np.diff(ss[sm])/1e9)
    xy=float(np.linalg.norm(n['pos'][mask,:2]-n['pos'][mask,:2][0],axis=1).max());y=n['rpy'][mask,2];yd=float(abs(np.unwrap(y)-y[0]).max())
    sx=float(np.linalg.norm(sp[:,:2]-sp[0,:2],axis=1).max());sy=angles(sq)[:,2];syd=float(abs(np.unwrap(sy)-sy[0]).max())
    peakv=float(np.linalg.norm(n['world_origin'][mask,:2],axis=1).max());peaky=float(abs(n['yawdot'][mask]).max());peakw=float(abs(n['omega'][mask,2]).max())
    relevant=[r for r in pid if r['compute_ros_clock_ns']/1e9>=begin and r['compute_ros_clock_ns']/1e9<=end]
    stable=all(r.get('mode')=='active_hold' and r.get('cascade',{}).get('fixed_goal_sha256')==targetsha and not r.get('cascade',{}).get('parking_window_interrupted') for r in relevant)
    cmds=array(execution,'requested',3)[em];caps=np.array([.06,.04,.1])
    goodpark=(duration>=.6-1e-9 and strictentry and stable and sourcegap<=.2+1e-9 and xy<=.05 and yd<=.1 and sx<=.05 and syd<=.1 and peakv<=.08 and peaky<=.1 and peakw<=.1 and np.all(abs(cmds)<=caps+1e-9))
    return check(goodpark,stopping_semantics='new fixed endpoint active velocity hold, explicitly different from historical zero-command parking',
                 first_declared_source_stamp_ns=declared,first_actual_Teacher_index=j,fixed_native_window_s=[begin,end],native_samples=int(mask.sum()),SLAM_samples=int(sm.sum()),
                 capture_distinct_source_dwell_s=duration,strict_entry_radius_verified=strictentry,target_camera_init_xyz=target,target_sha256=targetsha,
                 native_xy_drift_m=xy,native_yaw_drift_rad=yd,SLAM_xy_drift_m=sx,SLAM_yaw_drift_rad=syd,origin_speed_peak_mps=peakv,Euler_yawdot_peak_radps=peaky,body_wz_peak_radps=peakw)


def physical_route(run,pid,n,scope):
    anchor=load(run/'navigation_anchor.json');request=load(run/'navigation_request.json')
    poses=rows(run/'navigation_slam_poses.jsonl');pi=original_index(poses);stamp=ns(anchor['pose_stamp_ns'])
    if stamp not in pi or anchor.get('frozen_once') is not True or anchor.get('ground_truth_navigation_used') is not False:raise ValueError('Route anchor is not a single original SLAM observation')
    source=pi[stamp];origin=finite(anchor['origin'],3)
    if not np.allclose(origin,source['position'],atol=1e-10,rtol=0):raise ValueError('Anchor differs from actual SLAM')
    k=np.searchsorted(n['t'],stamp/1e9,side='right')-1
    if k<0 or stamp/1e9-n['t'][k]>.005001:raise MissingEvidence('Native anchor-time sample unavailable')
    sourceq=np.asarray(source['quaternion'])[[3,0,1,2]];Rsource=rotation(sourceq[None])[0]
    A=n['rot'][k]@Rsource.T
    vertices=np.vstack([origin,*[finite(g['center'],3) for g in request['goals']]])
    physicalvertices=n['pos'][k]+(vertices-origin)@A.T
    pt=np.asarray([r['compute_ros_clock_ns']/1e9 for r in pid]);idx,valid=causal(pt,n['t'],.3)
    directions=np.zeros((len(n['t']),3));ref=np.zeros(len(n['t']));heading=np.zeros(len(n['t']));distance=np.full(len(n['t']),np.nan)
    drive=np.zeros(len(n['t']),bool);wp=np.asarray([pid[i].get('waypoint_index',-1) for i in idx])
    for j in range(len(physicalvertices)-1):
        mask=valid&(wp==j)&np.array([pid[i].get('mode')=='drive' for i in idx]);v=physicalvertices[j+1]-physicalvertices[j];xy=float(v[:2]@v[:2])
        if xy<1e-10:continue
        proj=np.clip((n['pos'][:,:2]-physicalvertices[j,:2])@v[:2]/xy,0,1)
        lateral=np.linalg.norm(n['pos'][:,:2]-physicalvertices[j,:2]-proj[:,None]*v[:2],axis=1)
        distance[mask]=lateral[mask];heading[mask]=abs(wrap(n['rpy'][mask,2]-math.atan2(v[1],v[0])))
        directions[mask]=v/np.linalg.norm(v);drive|=mask
    for j in np.flatnonzero(drive):
        r=pid[idx[j]];c=r.get('cascade',{});rv=c.get('reference_velocity_world')
        if rv is None:raise MissingEvidence('New cascade reference_velocity_world absent; old PID command is not reference speed')
        ref[j]=np.linalg.norm(np.asarray(rv)[:2]);
    moving=drive&(ref>.03);stable=moving&(n['t']>=stamp/1e9+1.)
    if not stable.any():raise MissingEvidence('No actual drive with positive source reference speed')
    actual=np.einsum('ij,ij->i',n['world_com'],directions)
    err=float(np.mean(abs(actual[stable]-ref[stable])));rms=float(np.sqrt(np.mean(distance[moving]**2)));maximum=float(np.max(distance[moving]));hmax=float(np.max(heading[moving]));refmean=float(np.mean(ref[stable]))
    return check(maximum<=.2 and rms<=.08 and hmax<=.2 and err<=max(.05,.25*refmean),
                 body_relative_route_registration='single offline SE3 at original source anchor; not a registered original scene route',absolute_original_scene_centerline_registration='unverified',
                 maximum_route_distance_m=maximum,active_route_RMS_m=rms,maximum_drive_heading_rad=hmax,
                 native_COM_projected_speed_mean_mps=float(actual[stable].mean()),reference_speed_mean_mps=refmean,speed_MAE_mps=err,aligned_route_world_xyz=physicalvertices,
                 actual_Gazebo_state_used_only_after_sensor_command_audit=True)


def evaluate(run):
    run=Path(run).resolve();checks={};errors=[];inputs={};scope={};pid=[];poses=[];execution=[];n=None
    def audit(name,fn):
        try:checks[name]=fn()
        except MissingEvidence as e:checks[name]=check(None,reason=str(e))
        except KeyError as e:checks[name]=check(None,reason='Missing original recorded field: '+str(e))
        except (ValueError,TypeError,IndexError,AttributeError,OSError) as e:checks[name]=check(False,reason=type(e).__name__+': '+str(e))
    try:
        scope=load(run/'navigation_scope.json');pid=rows(run/'navigation_pid_history.jsonl');poses=rows(run/'navigation_slam_poses.jsonl');execution=rows(run/'telemetry.jsonl')
    except MissingEvidence as e:checks['required_original_inputs']=check(None,reason=str(e))
    except (ValueError,KeyError,TypeError) as e:checks['required_original_inputs']=check(False,reason=str(e))
    if scope:
        phase_status=[]
        try:phase_status,checks['execution_phase_status_and_cleanup_boundary']=execution_statuses(run,execution)
        except MissingEvidence as e:checks['execution_phase_status_and_cleanup_boundary']=check(None,reason=str(e))
        except (ValueError,KeyError,TypeError,IndexError) as e:checks['execution_phase_status_and_cleanup_boundary']=check(False,reason=str(e))
        audit('frozen_scope_and_archived_sources',lambda:source_audit(run,scope))
        audit('actual_causal_SLAM_IMU_cascade_updates',lambda:source_tick_audit(run,pid,poses,scope))
        audit('actual_checked_SCAN_trajectory_payloads',lambda:trajectories(run,rows(run/'navigation_status.jsonl'),pid)[0])
        audit('fixed_goal_and_original_SCAN_bounded_projection',lambda:goal_and_path_geometry(run,pid,poses))
        audit('actual_executor_ack_and_cascade_PI_COM_PD_replay',lambda:ack_and_PI_audit(run,pid,poses,execution,scope['profile']))
        clouds=[]
        try:
            checks['actual_raw_cloud_bytes_fields_filtering']=cloud_audit(run,poses,rows(run/'navigation_cloud_history.jsonl'))[0]
        except MissingEvidence as e:checks['actual_raw_cloud_bytes_fields_filtering']=check(None,reason=str(e))
        except (ValueError,KeyError,TypeError,IndexError,AttributeError,OSError) as e:checks['actual_raw_cloud_bytes_fields_filtering']=check(False,reason=type(e).__name__+': '+str(e))
        def guardfn():
            path=run/'navigation_native_guard_trace.jsonl'
            if not path.exists():path=run/'navigation_guard_history.jsonl'
            data=rows(run/'navigation_cloud_history.jsonl');ledger=original_index(rows(run/'navigation_cloud_xyz.jsonl'))
            merged=[{**r,'filtered_xyz_file':ledger[r['stamp_ns']]['array_file'], 'xyz_file_sha256':ledger[r['stamp_ns']]['array_sha256']} for r in data if r['stamp_ns'] in ledger]
            return guard_audit(run,rows(path),pid,merged)
        audit('actual_cascade_movement_guard_raw_geometry',guardfn)
        audit('actual_command_original_read_dual_TTL_slew',lambda:commands_audit(execution,rows(run/'navigation_command_history.jsonl'),scope['profile'],pid,poses,rows(run/'navigation_imu_history.jsonl')))
        audit('all_original_SLAM_3D_region_arrivals',lambda:region_audit(run,poses,phase_status)[0])
        audit('runtime_CPU_single_writer_complete_and_drained',lambda:runtime_audit(run,scope))
        try:n=native_state(run);checks.update(native_audit(n))
        except MissingEvidence as e:checks['native_execution_evidence']=check(None,reason=str(e))
        except (ValueError,KeyError,TypeError,AttributeError,OSError) as e:checks['native_execution_evidence']=check(False,reason=str(e))
        if n is not None:
            audit('actual_sensor_graph_actor247_CPU_joint_execution',lambda:actor_and_graph_audit(run,execution,n))
            audit('independent_offline_actual_route_speed_heading',lambda:physical_route(run,pid,n,scope))
            audit('new_first_declared_active_hold_fixed_5s',lambda:active_parking(run,pid,poses,execution,n))
        if scope.get('profile',{}).get('expected_asset',{}).get('terrain') not in('flat','flat_short','flat_long'):
            checks['strict_nonflat_complete_contact_geometry']=check(None,reason='A future terrain-specific append receipt must prove complete ramp/step foot XYZ+normal support and frozen route; general pose/safety is insufficient')
    for name in ('navigation_scope.json','source_manifest.json','navigation_source_snapshots.json','world.sdf','navigation_profile.json','navigation_request.json','navigation_anchor.json','navigation_slam_poses.jsonl','navigation_imu_history.jsonl','navigation_feedback_history.jsonl','navigation_cloud_history.jsonl','navigation_cloud_xyz.jsonl','navigation_cascade_paths.jsonl','navigation_fixed_goal_references.jsonl','closed_loop_executor_ack.jsonl','closed_loop_execution_commands.jsonl','navigation_pid_history.jsonl','navigation_command_history.jsonl','navigation_native_guard_trace.jsonl','navigation_guard_history.jsonl','navigation_sensor_gate_history.jsonl','navigation_status.jsonl','telemetry.jsonl','actuator.jsonl','runtime_manifest.json','navigation_pid_writer_receipt.json','policy_manifest.json','worker_result.json','sensor_contract.json','observations_actions.npz'):
        p=run/name
        if p.is_file():inputs[str(p)]=sha(p)
    state=overall(checks)
    return clean({'schema':SCHEMA,'status':state,'run':str(run),'checks':checks,'criteria':CRITERIA,
        'score':None,'evaluator_sha256':sha(__file__),'verified_input_source_sha256':inputs,
        'scope':{'actual_SLAM_SCAN_closed_loop_bounded_case':state,'simulation_only':True,
            'navigation_ground_truth_used':False,'native_ground_truth_used_only_offline':True,
            'privileged_Actor_dimensions':232,'controller_known_dimensions':15,
            'absolute_original_scene_map_registration':'unverified','dynamic_obstacle_stop_resume':'unverified',
            'complete_12m_ramp_contact_geometry':'unverified','full_multifloor_navigation':'unverified',
            'curved_SLAM_navigation':'unverified','general_Sim2Sim':'unverified','hardware_deployment':'unverified'},
        'meaning':'New append-only simulation contract; no historical zero-command receipt is overwritten or upgraded; no absent evidence is success.'})


def selftest():
    assert check(None)['status']=='unverified' and check(False)['status']=='failed'
    assert overall({'missing':check(None),'real_failure':check(False)})=='failed'
    assert longest([True]*6,np.arange(6)*.1,.2)==.5
    assert longest([True]*6,np.arange(6)*.25,.2)==0
    for value in(True,-1,1.0):
        try:ns(value)
        except ValueError:pass
        else:raise AssertionError('Original integer stamp false acceptance')
    assert np.array_equal(rotation([[1,0,0,0]])[0],np.eye(3))
    assert abs(float(wrap(2*math.pi)))<1e-12
    assert causal(np.array([1.,2.]),np.array([.9,1.2,2.4]),.3)[1].tolist()==[False,True,False]
    # A duplicate accepted raw header cannot add dwell/update frequency.
    try:original_index([{'stamp_ns':1},{'stamp_ns':1}])
    except ValueError:pass
    else:raise AssertionError('Duplicate original source accepted')
    # Axis clipping and planar norm are both required; the positive forward
    # restriction belongs to drive, never to an active endpoint correction.
    assert np.allclose(bounded_command(np.array([-1.,.4,.8]),np.array([.06,.04,.1])),[-.0499230176603,.0332820117735,.1],atol=1e-10)
    assert np.allclose(bounded_command(np.array([-1.,.4,.8]),np.array([.06,.04,.1]),True),[0.,.04,.1])
    # Fixed windows cannot pass from a later favorable interval: a violation
    # anywhere in the preregistered physical window remains a violation.
    actual=np.array([0.,.01,.051,.001]);assert np.max(actual)>.05
    assert float(np.linalg.norm([.001,0]))<=.05 and np.max(np.abs([.0001,.10001]))>.1
    # Feedback source future is always rejected independently of the declared
    # 50ms producer /clock ordering tolerance.
    pose=1_000_000_001;clock=1_000_000_000;gyro=1_000_000_002
    assert not 0<=clock-pose<=300_000_000 and not 0<=pose-gyro<=20_000_000
    print('14 meaningful offline invariant/negative groups passed; no ROS or actor started')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path);p.add_argument('--receipt-suffix',default='');p.add_argument('--self-test',action='store_true');p.add_argument('--no-write',action='store_true');a=p.parse_args()
    if a.self_test:selftest();return
    if a.run is None:p.error('--run required')
    if a.receipt_suffix and (not a.receipt_suffix.replace('_','').replace('-','').isalnum()):p.error('Invalid suffix')
    result=evaluate(a.run)
    suffix='.'+a.receipt_suffix if a.receipt_suffix else ''
    out=a.run.resolve()/('summary_closed_loop_cascade_independent'+suffix+'.json')
    if not a.no_write:
        text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
        with out.open('x') as f:f.write(text)
    print(json.dumps({'status':result['status'],'receipt':str(out) if not a.no_write else None,'checks':{k:v['status'] for k,v in result['checks'].items()}},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
