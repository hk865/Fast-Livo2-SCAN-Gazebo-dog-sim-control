"""Excluded component acceptance, with unchanged production region geometry.

GT is only an argument to final evaluation. No control/request/ROS code here.
One initial SE3, native integer timestamps, exact real NAV receipt windows.
"""
import bisect
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from navigation.goal_regions import contains,contains_control,definitions_sha256


def bounded_pair(stamp_ns,truth,max_gap_ns=150_000_000):
    if type(stamp_ns) is not int or not truth:return None
    stamps=[x['stamp_ns'] for x in truth]
    i=bisect.bisect_left(stamps,stamp_ns)
    if i<len(truth) and stamps[i]==stamp_ns:
        return np.asarray(truth[i]['p']),Rotation.from_quat(truth[i]['q']).as_matrix(),(i,i)
    if i==0 or i==len(truth):return None
    gap=stamps[i]-stamps[i-1]
    if not 0<gap<=max_gap_ns:return None
    fraction=(stamp_ns-stamps[i-1])/gap
    p=np.asarray(truth[i-1]['p'])+fraction*(np.asarray(truth[i]['p'])-truth[i-1]['p'])
    rotation=Slerp([0.,1.],Rotation.from_quat([truth[i-1]['q'],truth[i]['q']]))([fraction]).as_matrix()[0]
    return p,rotation,(i-1,i)


def evaluate_regions(goals,request_id,status_rows,poses,truth,origin_stamp_ns,terminal_stamp_ns):
    errors=[];checks={};precision=[];coverage_missing=[];regions=[]
    goal_hash=definitions_sha256(goals);definitions=[g.definition() for g in goals]
    for name,rows in [('slam',poses),('truth',truth)]:
        if any(type(x.get('stamp_ns')) is not int or x['stamp_ns']<0
               or not np.isfinite(x.get('p',[])).all() or len(x.get('p',[]))!=3
               or len(x.get('q',[]))!=4 or not np.isfinite(x['q']).all()
               or np.linalg.norm(x.get('q',[]))<1e-8 for x in rows):
            errors.append(name+' contains missing native stamp or invalid measured pose')
        if all(type(x.get('stamp_ns')) is int for x in rows) and any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])):
            errors.append(name+' native timestamps are duplicated or reversed')
    if errors or type(origin_stamp_ns) is not int or type(terminal_stamp_ns) is not int:
        return dict(passed=False,errors=errors or ['missing actual origin/terminal time'],checks={},regions=[],coverage={'passed':False})
    active=[x for x in poses if origin_stamp_ns<=x['stamp_ns']<=terminal_stamp_ns]
    origin=next((x for x in poses if x['stamp_ns']==origin_stamp_ns),None)
    first=bounded_pair(origin_stamp_ns,truth)
    if origin is None or first is None:
        return dict(passed=False,errors=['initial measured SLAM/GT has no bounded pair'],checks={},regions=[],coverage={'passed':False})
    rotation=first[1]@Rotation.from_quat(origin['q']).as_matrix().T
    translation=first[0]-rotation@np.asarray(origin['p'])
    paired={}
    for x in active:
        gt=bounded_pair(x['stamp_ns'],truth)
        if gt is None:
            coverage_missing.append(x['stamp_ns']);continue
        paired[x['stamp_ns']]=gt[0]
        precision.append(float(np.linalg.norm(rotation@np.asarray(x['p'])+translation-gt[0])))
    matching=[x for x in status_rows if x['status'].get('request_id')==request_id]
    definition_ok=bool(matching) and all(x['status'].get('goals_definition_sha256')==goal_hash
        and x['status'].get('goals_definitions')==definitions and x['status'].get('total')==len(goals)
        for x in matching)
    complete=[x for x in matching if x['status'].get('state')=='succeeded'
        and x['status'].get('waypoint_index')==len(goals)
        and isinstance(x['status'].get('region_arrivals'),list)
        and len(x['status']['region_arrivals'])==len(goals)]
    observed=[x['status']['region_arrivals'] for x in matching if isinstance(x['status'].get('region_arrivals'),list)]
    receipts=complete[0]['status']['region_arrivals'] if complete else max(observed,key=len,default=[])
    immutable=all(x==receipts[:len(x)] for x in observed) and len(receipts)<=len(goals)
    previous=-1;blocked=not definition_ok or not immutable
    for index,g in enumerate(goals):
        reasons=[];receipt=receipts[index] if index<len(receipts) else None
        entry=dict(index=index,goal_id=g.goal_id,definition=g.definition(),passed=False,receipt=receipt,observations=[])
        regions.append(entry)
        if not isinstance(receipt,dict):
            entry['errors']=['missing actual NAV region receipt'];blocked=True;continue
        start,end,dwell=[receipt.get(k) for k in ['start_stamp_ns','stamp_ns','dwell_ns']]
        valid_time=all(type(v) is int and v>=0 for v in [start,end,dwell])
        if not valid_time or start<=previous or end<start or dwell!=end-start or dwell<round(g.dwell_sim_s*1e9):
            reasons.append('invalid, overlapping or insufficient native dwell')
        if any(receipt.get(k)!=v for k,v in dict(request_id=request_id,goal_id=g.goal_id,
             waypoint_index=index,goals_definition_sha256=goal_hash,protected=False,region_inside=True,
             control_region_inside=True,reason='arrived',max_observation_gap_ns=200_000_000,
             arrival_definition=g.definition()['arrival'],control_arrival_definition=g.control_arrival_definition()).items()):
            reasons.append('receipt identity, geometry, protection or dwell contract differs')
        if receipt.get('protected') is not False or receipt.get('region_inside') is not True or receipt.get('control_region_inside') is not True:
            reasons.append('receipt protection/inside flags must be actual booleans')
        window=[x for x in active if valid_time and start<=x['stamp_ns']<=end]
        ns=[x['stamp_ns'] for x in window]
        if not ns or ns[0]!=start or ns[-1]!=end or any(b-a>200_000_000 for a,b in zip(ns,ns[1:])):
            reasons.append('raw measured SLAM does not cover complete dwell')
        raw=receipt.get('raw_position')
        try:
            raw_ok=np.shape(raw)==(3,) and np.isfinite(raw).all() and contains_control(g,raw)
            expected_error=float(np.linalg.norm(np.asarray(raw)-g.center)) if raw_ok else None
        except (ValueError,TypeError):raw_ok=False;expected_error=None
        if not raw_ok or not window or np.linalg.norm(np.asarray(window[-1]['p'])-raw)>1e-8:
            reasons.append('receipt raw endpoint differs or is outside inner band')
        reported=receipt.get('center_error_m')
        if expected_error is None or type(reported) not in (float,int) or not math.isfinite(reported) or abs(reported-expected_error)>1e-8:
            reasons.append('receipt center-error evidence differs')
        for x in window:
            ns=x['stamp_ns'];gt=paired.get(ns)
            if gt is None:
                reasons.append('same-time bounded independent truth missing');continue
            gt_slam=rotation.T@(gt-translation)
            inner=contains_control(g,x['p']);outer=contains(g,gt_slam)
            entry['observations'].append(dict(stamp_ns=ns,raw_slam=x['p'],truth_world=gt.tolist(),
                truth_in_slam=gt_slam.tolist(),raw_inner=inner,truth_outer=outer))
            if not inner or not outer:reasons.append('raw-inner or independent truth-outer failed in original receipt window')
        if blocked:reasons.append('earlier ordered prefix or definition failed')
        entry['errors']=list(dict.fromkeys(reasons));entry['passed']=not reasons
        if reasons:blocked=True
        else:previous=end
    checks.update(actual_matching_goal_hash_and_definitions=definition_ok,
        native_receipts_immutable=immutable,actual_NAV_succeeded=bool(complete),
        ordered_four_original_receipt_windows=len(goals)==4 and len(regions)==4 and all(x['passed'] for x in regions),
        independent_truth_full_active_pose_coverage=bool(active) and not coverage_missing,
        native_sensor_timestamps_valid=not errors)
    return dict(passed=all(checks.values()),checks=checks,errors=errors,regions=regions,
        goals_definition_sha256=goal_hash,initial_fixed_SE3_evaluation_only=dict(stamp_ns=origin_stamp_ns,
            rotation_world_from_slam=rotation.tolist(),translation=translation.tolist()),
        coverage=dict(passed=bool(active) and not coverage_missing,active_interval_ns=[origin_stamp_ns,terminal_stamp_ns],
            active_pose_rows=len(active),matched_pose_rows=len(paired),unpaired_native_stamps=coverage_missing,
            max_truth_interpolation_gap_ns=150_000_000,missing_tail_never_extrapolated=True),
        precision=dict(rmse=float(np.sqrt(np.mean(np.square(precision)))) if precision else None,max=max(precision,default=None)))
