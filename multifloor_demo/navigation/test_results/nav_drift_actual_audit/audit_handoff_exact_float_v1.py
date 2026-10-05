"""Offline original-data NAV interruption audit. No ROS/node/control or GT input.

Independent truth appears only in the final region evaluator, never in the
interruption/cmd/handshake analysis. Adapter emission time is its cached sim
clock, not a Twist Header or measured CHAMP callback.
"""
import ast
import bisect
import hashlib
import importlib.util
import json
import pathlib
import sys
import numpy as np
from scipy.interpolate import BSpline

HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(l) for l in p.open()]
def st(r):return round(r['sim']*1e9)

def audit(run):
    original=json.loads((run/'first_four_result.json').read_text())
    guard=json.loads((run/'nav_drift_guard.json').read_text())
    events=guard['events']
    adapter=rows(run/'joint_stop_adapter.jsonl')
    edges=[i for i,r in enumerate(adapter) if r['kind']=='zero_edge']
    trajectories=rows(run/'feedback_navigation_trajectories.jsonl')
    poses={r['stamp_ns']:r for r in original['poses']}
    stopindices=[i for i,r in enumerate(events) if r['event']=='exact_zero_published']
    reports=[]
    for count,i in enumerate(stopindices):
        stop=events[i];trigger=stop['trigger'];limit=stopindices[count+1] if count+1<len(stopindices) else len(events)
        later=events[i+1:limit]
        ref=next((r for r in later if r['event']=='fresh_reference_requested'),None)
        accepted=next((r for r in later if r['event']=='fresh_checked_path_accepted'),None)
        anchor=trigger['raw_anchor'];ctx=trigger['context'];since=trigger['stamp_ns']-trigger['persistence_ns']
        obs=[r for r in events[:i] if r['event']=='observe' and r['request_id']==stop['request_id']
             and r['waypoint_index']==stop['waypoint_index'] and since<=r['pose_stamp_ns']<=trigger['stamp_ns']]
        stamps=[r['pose_stamp_ns'] for r in obs]
        native_pos_match=(trigger['stamp_ns'] in poses and np.array_equal(poses[trigger['stamp_ns']]['p'],trigger['raw_position']))
        qualifying=all(t in poses and np.linalg.norm((np.asarray(poses[t]['p'])-anchor)[:2])>=.15
            and r['adapter_state']=='walk' and r['planned_command'][:2]==[0.,0.] and r['planned_command'][2]!=0.
            and r['bridge_safe'][:2]==[0.,0.] and r['bridge_safe'][2]!=0.
            and r['bridge_age_wall_s']<=.25 for t,r in zip(stamps,obs))
        edge_i=edges[stop['adapter_stop_count_before']]
        edge=adapter[edge_i]
        end_i=edges[stop['adapter_stop_count_before']+1] if stop['adapter_stop_count_before']+1<len(edges) else len(adapter)
        section=adapter[edge_i:end_i]
        preceding=next((r for r in reversed(adapter[:edge_i]) if r['kind']=='actual_champ_command'),None)
        ack=next((r for r in section if r['kind']=='native_zero_ack'),None)
        idle=next((r for r in section if r['kind']=='return_complete'),None)
        subsequent=adapter[edge_i:]
        first_motion_i=next((j for j,r in enumerate(subsequent) if r['kind']=='actual_champ_command' and any(r['value'])),None)
        first_motion=None if first_motion_i is None else subsequent[first_motion_i]
        held=subsequent if first_motion_i is None else subsequent[:first_motion_i]
        zeros=[r for r in held if r['kind']=='actual_champ_command']
        metadata=[];path=[];path_error=None;handoff_match=False
        if ref and accepted:
            identity=accepted['identity']
            metadata=[r for r in trajectories if r['kind']=='scan_metadata'
                and r['metadata']['reference_stamp']==ref['reference_stamp']
                and r['metadata']['trajectory']['traj_id']==identity['traj_id']]
            if metadata:
                m=metadata[0]['metadata'];v=m['trajectory'];k=np.asarray(v['knots']);c=np.asarray(v['pos_pts']);order=v['order']
                points=BSpline(k,c,order)(np.linspace(k[order],k[-order-1],min(2000,max(2,int((k[-order-1]-k[order])/.08)+1))))
                path=[r for r in trajectories if r['kind']=='accepted_path' and r['points']
                      and abs(round(r['receipt_sim_time']*1e9)-accepted['callback_clock_ns'])<=30000000
                      and np.shape(r['points'])==points.shape]
                if path:path_error=float(np.max(np.abs(np.asarray(path[0]['points'])-points)))
                handoff_match=(m['reference_stamp']==identity['reference_stamp']==ref['reference_stamp']
                    and m['body_goal']==identity['body_goal']==ref['handoff']['body_goal']
                    and tuple(ref['reference_stamp'])>tuple(ctx[3]) and identity['traj_id']>ctx[2])
        statuses=[r for r in original['statuses'] if accepted and r['sim']*1e9>=accepted['callback_clock_ns']
            and r['status']['waypoint_index']==stop['waypoint_index'] and r['status'].get('accepted_trajectory_id')==accepted['identity']['traj_id']]
        phases=[]
        for r in statuses:
            phase=r['status']['alignment_phase']
            if not phases or phases[-1]['phase']!=phase:phases.append(dict(phase=phase,receipt_sim=r['sim']))
        receipt=original['region_evaluation']['regions'][stop['waypoint_index']]['receipt']
        prior=original['origin_stamp_ns'] if stop['waypoint_index']==0 else original['region_evaluation']['regions'][stop['waypoint_index']-1]['receipt']['stamp_ns']
        checks=dict(trigger_matches_original_native_pose=native_pos_match,
            three_distinct_raw_samples=len(set(stamps))>=3 and len(stamps)==len(set(stamps)),
            actual_raw_200ms_window=bool(stamps) and stamps[0]==since and stamps[-1]==trigger['stamp_ns']
                and stamps[-1]-stamps[0]>=200000000 and all(0<b-a<=200000000 for a,b in zip(stamps,stamps[1:])),
            pure_actual_walk_protocol_and_raw_threshold=qualifying,
            original_exact_zero_first=stop['command']==[0.,0.,0.],
            actual_native_adapter_zero_before_edge=preceding is not None and not any(preceding['value']),
            native_zero_ack_and_return=ack is not None and idle is not None,
            all_actual_adapter_emits_until_resume_zero=bool(zeros) and all(not any(r['value']) for r in zeros),
            postSTOP_fresh_idle_ack=bool(ref) and ref['actual_adapter_ack']['state']=='idle'
                and ref['actual_adapter_ack']['nominal_calibrated'] is True
                and ref['actual_adapter_ack']['counters']['stops']>stop['adapter_stop_count_before']
                and round(ref['actual_adapter_ack']['sim']*1e9)>stop['actual_zero_command_clock_ns'],
            actual_idle_one_sim_second=bool(ref) and ref['callback_clock_ns']-ref['handoff']['actual_zero_since_ns']>=1000000000
                and idle is not None and st(idle)<=ref['handoff']['actual_zero_since_ns'],
            fresh_reference_and_same_goal_complete_metadata=handoff_match,
            accepted_path_matches_actual_full_spline=path_error is not None and path_error<=1e-12,
            native_motion_after_checked_accept_and_original_preturn=bool(first_motion) and bool(accepted)
                and st(first_motion)>=accepted['callback_clock_ns']+1000000000,
            same_current_goal_until_handoff=bool(ref) and bool(accepted)
                and ref['request_id']==accepted['request_id']==stop['request_id']
                and ref['waypoint_index']==accepted['waypoint_index']==stop['waypoint_index'],
            goal_receipt_elapsed_under_original_90s=(receipt['stamp_ns']-prior)<=90000000000)
        reports.append(dict(index=count,goal_index=stop['waypoint_index'],checks=checks,passed=all(checks.values()),
            raw_trigger_stamp_ns=trigger['stamp_ns'],stop_callback_clock_ns=stop['callback_clock_ns'],
            raw_planar_drift_m=trigger['raw_planar_drift_m'],raw_stamps=stamps,
            trigger_measurement_to_zero_callback_ns=stop['callback_clock_ns']-trigger['stamp_ns'],
            adapter_zero_edge= {k:edge[k] for k in ['sim','wall','state']},
            native_actual_zero_emit=None if preceding is None else {k:preceding[k] for k in ['sim','wall','state','value']},
            native_zero_ack=ack,native_return_complete=idle,
            actual_zero_emissions_before_resume=len(zeros),fresh_reference=ref,checked_accept=accepted,
            actual_first_resumed_emit=first_motion,path_max_error=path_error,observed_original_phases=phases,
            original_receipt_window_ns=[receipt['start_stamp_ns'],receipt['stamp_ns']],
            elapsed_from_prior_original_receipt_s=(receipt['stamp_ns']-prior)/1e9))
    spec=importlib.util.spec_from_file_location('frozen_region_evaluator',run/'staging/first_four_region_contract.py')
    evaluator=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluator)
    regions=evaluator.evaluate_regions(tuple(parse_goal(g) for g in original['goals_definitions']),original['request_id'],
        original['statuses'],original['poses'],original['truth'],original['origin_stamp_ns'],original['terminal_stamp_ns'])
    source=ast.parse((run/'staging/nav_drift_controller.py').read_text())
    written=[]
    for n in ast.walk(source):
        if isinstance(n,ast.Attribute) and isinstance(n.ctx,ast.Store):written.append(n.attr)
    checks=dict(four_interruptions=len(reports)==4 and guard['interruptions']==4,
        no_guard_ring_loss=guard['dropped']==0,
        all_original_native_handoffs=bool(reports) and all(r['passed'] for r in reports),
        region_eval_same_original_receipts=regions['passed'],
        wrapper_no_deadline_or_region_geometry_write=not any(n in written for n in ['segment_start','segment_started_ros','goals','waypoints','goals_definition_sha256','waypoint_index']),
        original_component_driver_passed=original['passed'])
    files=['nav_drift_guard.json','first_four_result.json','joint_stop_adapter.jsonl','feedback_navigation_trajectories.jsonl',
        'staging/nav_drift_controller.py','staging/turn_drift.py','staging/first_four_region_contract.py','staging/probe_first_four_regions.py']
    result=dict(passed=all(checks.values()),checks=checks,run=str(run),handoffs=reports,
        independently_recomputed_original_region_evaluation=regions,
        input_sha256={n:sha(run/n) for n in files},audit_sha256=sha(pathlib.Path(__file__)),
        limitations=['GT only evaluates original receipt windows with one initial SE3; never enters interruption analysis or control.',
            'Adapter emitted command is native publisher evidence, not CHAMP receive/torque or no-slip execution.',
            'Adapter actual emit log sim is its cached float clock; file event order/counter binds the stop, clock latency is not inferred as exact DDS delay.',
            'Protected waits keep the original deadline. Actual segment start may slightly follow the preceding receipt, so elapsed receipt bounds are conservative.',
            'Four flat-region component success does not prove original Full18 counterfactual stability, slope success or full multi-floor success.'])
    (HERE/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=checks,handoffs=[dict(index=r['index'],checks=r['checks'],trigger=r['raw_trigger_stamp_ns']/1e9,
        stop=r['stop_callback_clock_ns']/1e9,zero_edge=r['adapter_zero_edge']['sim'],ref=r['fresh_reference']['callback_clock_ns']/1e9,
        accepted=r['checked_accept']['callback_clock_ns']/1e9,resume=r['actual_first_resumed_emit']['sim']) for r in reports]),indent=2))

if __name__=='__main__':audit(ROOT/'simulation/test_results/20261002_nav_drift_first4_disabled')
