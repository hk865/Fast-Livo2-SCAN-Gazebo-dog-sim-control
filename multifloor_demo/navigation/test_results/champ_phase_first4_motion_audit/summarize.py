"""Summarize the two existing lightweight motion audits; no CDR/ROS/control."""
import hashlib,json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUN=ROOT/'multifloor_demo/simulation/test_results/20261002_champ_phase_first4_candidate_v2'
BASE=ROOT/'multifloor_demo/simulation/test_results/20261002_classic_lowD_align_first4_candidate'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    motion=json.loads((HERE/'motion.json').read_text())
    phase=json.loads((HERE/'phase_motion.json').read_text())
    data=json.loads((RUN/'first_four_result.json').read_text())
    guard=json.loads((RUN/'nav_drift_guard.json').read_text())
    config=json.loads((RUN/'actual_nav_align_configuration.json').read_text())
    sources={n:{'current':sha(RUN/'staging'/n),'prior':sha(BASE/'staging'/n)}
             for n in ['nav_align_controller.py','turn_drift.py','align_translation.py']}
    reasons={}
    for e in guard['events']:
        if e['event']=='exact_zero_published':
            reason=e['trigger']['details'].get('reason',e['trigger']['event'])
            reasons[reason]=reasons.get(reason,0)+1
    emits=[]
    with (RUN/'joint_stop_adapter.jsonl').open() as f:
        for line in f:
            r=json.loads(line)
            if r['kind']=='actual_champ_command' and data['origin_stamp']<=r['sim']<=data['terminal_sim']:
                emits.append(r['value'])
    bounds={'actual_emission_count':len(emits),'vx_abs_max':max(abs(c[0]) for c in emits),
        'vy_all_zero':all(c[1]==0 for c in emits),'yaw_abs_max':max(abs(c[5]) for c in emits),
        'mixed_yaw_abs_max':max(abs(c[5]) for c in emits if c[0]!=0 or c[1]!=0)}
    receipts=[r['receipt'] for r in data['region_evaluation']['regions']]
    deadlines=[];last=data['origin_stamp_ns']
    for i,r in enumerate(receipts):
        deadlines.append({'goal_index':i,'goal_id':r['goal_id'],'prior_receipt_or_origin_ns':last,
            'original_receipt_start_ns':r['start_stamp_ns'],'original_receipt_end_ns':r['stamp_ns'],
            'original_dwell_ns':r['dwell_ns'],'elapsed_from_prior_receipt_sim_s':(r['stamp_ns']-last)/1e9,
            'declared_timeout_sim_s':data['goals_definitions'][i]['timeout_sim_s']})
        last=r['stamp_ns']
    comparisons={}
    for key,label in [('candidate','private_phase_candidate'),('original_lowD4','previous_ALIGN_first4')]:
        p=phase[key];g=p['groups']; align=[v for k,v in g.items() if k.startswith('ALIGN_')]
        drive=g['DRIVE_TRANSLATION']
        comparisons[label]={'supported_phase_seconds':p['phase_support_duration_s'],
            'supported_ALIGN_total_GT_forward_m':sum(v['GT_evaluation']['body_forward_m'] for v in align),
            'supported_ALIGN_mixed':g['ALIGN_TRANSLATION'],'supported_ALIGN_pure':g['ALIGN_ROTATION'],
            'supported_DRIVE':drive,'supported_DRIVE_GT_forward_speed_m_s':drive['GT_evaluation']['body_forward_m']/drive['duration_sim_s'],
            'reported_guard_interruptions':p['drift_stops'],'active_peak_raw_imu_tilt':p['active_peak_imu'],
            'native_pose_observed_interval_ns':p['observed_interval_ns'],
            'excluded_terminal_suffix_ns':p['excluded_terminal_suffix_ns']}
    checks={'original_component_result_20checks_retained':data['passed'] and len(data['checks'])==20 and all(data['checks'].values()),
        'all_motion_handoff_checks':all(motion['checks'].values()),
        'one_source_identity_stop_not_displacement_or_corridor':reasons=={'foreign_identity':1},
        'three_NAV_source_bytes_same_as_previous_ALIGN':all(v['current']==v['prior'] for v in sources.values()),
        'all_live_ALIGN_configuration_observations_match':config['final_matches'] and config['all_received_statuses_match'],
        'actual_ALIGN_configuration_same_as_previous':phase['candidate']['PD_actual_configuration']==phase['original_lowD4']['PD_actual_configuration'],
        'actual_emitted_vx_original_cap':bounds['vx_abs_max']<=.12+1e-12,
        'actual_mixed_yaw_original_cap':bounds['mixed_yaw_abs_max']<=.08+1e-12,
        'actual_pure_yaw_original_cap':bounds['yaw_abs_max']<=.12+1e-12,
        'actual_vy_zero':bounds['vy_all_zero'],
        'four_receipts_original_dwell_and_order':len(receipts)==4 and all(r['dwell_ns']>=400000000 for r in receipts)
            and all(a['stamp_ns']<b['start_stamp_ns'] for a,b in zip(receipts,receipts[1:])),
        'all_declared_original90s_and_observed_receipts_within':all(r['declared_timeout_sim_s']==90 and r['elapsed_from_prior_receipt_sim_s']<90 for r in deadlines),
        'original_native_region_evaluation_unchanged':motion['original_native_region_evaluation']==data['region_evaluation'],
        'startup_selected_private_gait_actual_verified':json.loads((RUN/'startup_gait_runtime.json').read_text())['passed'],
        'actual_lowD_CM_JTC_parameter_gate_passed':json.loads((RUN/'control_parameter_readback.json').read_text())['passed']}
    proof={'scope':'Owned-clean original phase first4 NAV motion/handoff audit. One initial SE3 evaluates GT, never controls. CDR is not decoded here.',
        'run':str(RUN),'source_identity_stop_count':1,'displacement_stop_count':0,'corridor_stop_count':0,
        'original_component_result_passed':data['passed'],'original_failure':data['failure'],
        'original_receipt_deadlines':deadlines,'original_raw_peak_tilt':data['active_max_imu_tilt'],
        'native_emit_bounds':bounds,'comparisons_descriptive_only':comparisons,
        'all_original_handoffs':motion['handoffs'],'observation_boundary':motion['observation_boundary'],
        'original_SINGLE_SE3_GT_evaluation_only':motion['single_original_fixed_SE3_evaluation_only'],
        'original_NAV_source_pair_sha256':sources,'checks':checks,'audit_ready':all(checks.values()),
        'input_sha256':{n:sha(RUN/n) for n in ['first_four_result.json','nav_drift_guard.json','joint_stop_adapter.jsonl',
            'feedback_navigation_trajectories.jsonl','actual_nav_align_configuration.json','startup_gait_runtime.json',
            'actual_gait_process_prestop_paths.json','control_parameter_readback.json','profile_manifest.json','process_cleanup.json']},
        'audit_sha256':{n:sha(HERE/n) for n in ['audit_motion.py','audit_phase.py','motion.json','phase_motion.json','summarize.py']},
        'limits':['ALIGN with nonzero vx is identified from same-identity native arm/observe support; it is not reclassified as ordinary DRIVE.',
            'Matching low-rate status supports DRIVE/pre/settle only over bounded equal neighboring status intervals;7.316s unsupported/conflicting phase boundaries remain UNCERTAIN.',
            'Native Adapter command log is publisher evidence, not a CHAMP callback or applied force measurement; its sim field is a cached floating clock.',
            'Last native pose is132.3s; original terminal132.408 remains unchanged, but108ms motion tail is not extrapolated.',
            'The original90s declarations and receipt boundaries remain unchanged; successful4 cases do not independently exercise timeout failure paths.',
            'Aggregated signed yaw/velocity response is not a constant-input system identification or guarantee of accurate execution.',
            'Previous and current runs have different physical startup/trajectory histories. Descriptive differences are not a general plant or causal stability proof.',
            'No full-demo, slope/8-goal or new dynamic-obstacle PASS is inferred. Raw/JTC restart short-sample analysis is delegated to the single peer decode.']}
    output=HERE/'final_receipt.json';assert not output.exists()
    output.write_text(json.dumps(proof,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'audit_ready':proof['audit_ready'],'checks':len(checks),'final_receipt_sha256':sha(output),'stop_reasons':reasons}))
if __name__=='__main__':main()
