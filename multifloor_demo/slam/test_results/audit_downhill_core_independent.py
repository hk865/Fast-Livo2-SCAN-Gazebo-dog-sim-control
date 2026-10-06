"""Minimal immutable downhill A/B evidence. No ROS initialization or CDR scan."""
from pathlib import Path
import hashlib, importlib.util, json, sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal,definitions_sha256
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

spec=importlib.util.spec_from_file_location('downhill_existing_helpers',ROOT/'slam/test_results/analyze_feedback_first4_ab_independent.py')
H=importlib.util.module_from_spec(spec);spec.loader.exec_module(H)
frozen=load(ROOT/'test_results/full19_freeze/source_manifest.json')['sha256']
cases=[]
for name,enabled in [('20261002_downhill_a_disabled_v2',False),('20261002_downhill_b_enabled_v2',True)]:
    run=ROOT/'simulation/test_results'/name;r=load(run/'downhill_result.json');m=load(run/'fixture_manifest.json')
    terminal=load(run/'terminal_receipt.json');runtime=load(run/'full_control_runtime_evidence.json')
    body=[json.loads(l) for l in (run/'body_feedback.jsonl').read_text().splitlines() if l.strip()]
    firstfailed=next((x for x in body if x.get('failed') is True),None)
    mode=[x for x in body if type(x.get('enabled')) is bool]
    start,end=r['origin_stamp_ns'],r['terminal_stamp_ns'];evaluation=None;orientation=[];crossings={};native=None;rawgap=[]
    if start is not None:
        spec=importlib.util.spec_from_file_location('downhill_archived_contract_'+name,run/'staging/downhill_region_contract.py')
        C=importlib.util.module_from_spec(spec);spec.loader.exec_module(C)
        goals=tuple(parse_goal(x) for x in r['goals_definitions'])
        evaluation=C.evaluate_regions(goals,r['request_id'],r['statuses'],r['poses'],r['truth'],start,end)
        R=np.asarray(evaluation['initial_fixed_SE3_evaluation_only']['rotation_world_from_slam'])
        for p in r['poses']:
            if not start<=p['stamp_ns']<=end:continue
            gt=C.bounded_pair(p['stamp_ns'],r['truth'])
            if gt is None:continue
            actual=Rotation.from_matrix(gt[1]);estimated=Rotation.from_matrix(R)*Rotation.from_quat(p['q'])
            orientation.append(dict(stamp_ns=p['stamp_ns'],error_rad=float((actual.inv()*estimated).magnitude()),
                GT_Euler_metric_rad=H.metric(actual.as_quat())[0],SLAM_fixed_Euler_metric_rad=H.metric(estimated.as_quat())[0]))
        raw=[x for x in r['raw_imu'] if start<=x['stamp_ns']<=end]
        rawgap=[dict(previous_ns=a['stamp_ns'],next_ns=b['stamp_ns'],gap_ns=b['stamp_ns']-a['stamp_ns']) for a,b in zip(raw,raw[1:]) if b['stamp_ns']-a['stamp_ns']!=1000000]
        for limit in [.30,.50]:
            row=next((x for x in raw if H.metric(x['quaternion'])[0]>=limit),None)
            if row is None:crossings[str(limit)]=None;continue
            gt=C.bounded_pair(row['stamp_ns'],r['truth'])
            crossings[str(limit)]=dict(stamp_ns=row['stamp_ns'],raw_Euler_metric_rad=H.metric(row['quaternion'])[0],
                actual_same_stamp_GT_Euler_metric_rad=None if gt is None else H.metric(Rotation.from_matrix(gt[1]).as_quat())[0],
                GT_bracket_indices=None if gt is None else list(gt[2]))
        native=H.audit_native(run,r)
    observer=load(run/'actuator/observer_result.json')
    actual_return0=r['region_evaluation'].get('regions',[{}])[0] if r['region_evaluation'].get('regions') else None
    case=dict(run_id=name,original_driver_passed=r['passed'],original_driver_failure=r['failure'],
        original_checks=r['checks'],original_missing=r['missing_acceptance'],original_result_sha256=sha(run/'downhill_result.json'),
        selected_feedback_enabled=r['selected_body_feedback_enabled'],actual_body_log_mode_count=len(mode),
        actual_body_log_all_modes_match=bool(mode) and all(x['enabled'] is enabled for x in mode),
        origin_stamp_ns=start,terminal_stamp_ns=end,original_first_region=actual_return0,
        independent_original_regions=evaluation,raw_original_metric_peak=r['active_max_imu_tilt'],
        active_bridge_holds=r['active_bridge_holds'],active_raw_unexpected_1ms_gaps=rawgap,
        active_first_Euler_crossings_same_time_GT=crossings,
        one_initial_SE3_orientation=dict(samples=len(orientation),RMSE_rad=float(np.sqrt(np.mean([x['error_rad']**2 for x in orientation]))) if orientation else None,
            max_rad=max((x['error_rad'] for x in orientation),default=None),last_twelve=orientation[-12:]),
        native_IMU_and_generic=native,actual_first_failed_body_tick=firstfailed,
        actual_startup_gates={n:load(run/n)['passed'] for n in ['controller_ready_gate.json','control_parameter_readback.json','startup_control_runtime.json','startup_gait_runtime.json','startup_gate.json']},
        actual_terminal_runtime=runtime,terminal_receipt=terminal,
        same_source329_manifest=load(run/'source_manifest.json')['sha256']==frozen==m['source_sha256'],
        all5_excluded_executed_sources_match=all(sha(run/'staging'/n)==h for n,h in m['executed_staging_sha256'].items()),
        recording_footer_only=observer,
        no_original_CDR_scan=True,
        input_sha256={n:sha(run/n) for n in ['fixture_manifest.json','source_manifest.json','runtime_manifest.json','downhill_result.json',
            'terminal_receipt.json','full_control_runtime_evidence.json','body_feedback.jsonl','actuator/observer_result.json']})
    if start is None:
        case['unstarted_scope']='No origin, sensor heading calibration/request/region goals or motion windows. No evaluation SE3 invented.'
    cases.append(case)
a,b=cases
checks={
    'both_original_FAIL_unchanged':not a['original_driver_passed'] and not b['original_driver_passed'],
    'A_first_original_window_rawinner_GTouter_passed':a['independent_original_regions']['regions'][0]['passed'] is True,
    'A_second_original_window_absent_kept_failed':a['independent_original_regions']['regions'][1]['receipt'] is None and a['independent_original_regions']['regions'][1]['passed'] is False,
    'A_all_active_GT_bounded_under_one_initial_SE3':a['independent_original_regions']['coverage']['passed'],
    'A_raw050_and_trueGT_tilt_confirmed':a['active_first_Euler_crossings_same_time_GT']['0.5']['raw_Euler_metric_rad']>=.50 and a['active_first_Euler_crossings_same_time_GT']['0.5']['actual_same_stamp_GT_Euler_metric_rad']>=.49,
    'B_no_origin_or_motion_windows_preserved':b['origin_stamp_ns'] is None and b['independent_original_regions'] is None,
    'B_failed_original_100ms_wall_freshness_with_valid_integer_pair':b['actual_first_failed_body_tick']['predicates']['checks']['complete_joint_imu_pair'] is True and b['actual_first_failed_body_tick']['predicates']['checks']['joint_sim_fresh'] is True and b['actual_first_failed_body_tick']['predicates']['checks']['imu_wall_fresh'] is False and b['actual_first_failed_body_tick']['predicates']['checks']['joint_wall_fresh'] is False,
    'actual_feedback_modes_A0_B1_observed':a['actual_body_log_all_modes_match'] and b['actual_body_log_all_modes_match'],
    'both_early_actual5_gates_allpass':all(all(c['actual_startup_gates'].values()) for c in cases),
    'both_terminal_ownedclean_library_source_runtime16_allpass':all(c['terminal_receipt']['checks']['ownedclean'] and c['actual_terminal_runtime']['passed'] and len(c['actual_terminal_runtime']['checks'])==16 and all(c['actual_terminal_runtime']['checks'].values()) for c in cases),
    'both_original329_and_excluded5_SHA_same':all(c['same_source329_manifest'] and c['all5_excluded_executed_sources_match'] for c in cases),
    'current329_original_source_bytes_still_frozen':all(sha(ROOT/n)==h for n,h in frozen.items()),
    'both_original_recorders_no_capture_failure_or_overwrite':all(c['recording_footer_only']['failure'] is None and c['recording_footer_only']['overwritten_events']==0 for c in cases),
}
result=dict(scope='Independent core audit of fresh F3 canonical two-region A0/B1. No physical feedback-effect comparison can be inferred.',
    checks=checks,all_evidence_checks_true=all(checks.values()),check_count=len(checks),A=a,B=b,
    independent_initial_frame_only_A=a['independent_original_regions']['initial_fixed_SE3_evaluation_only'],
    original_329_source_manifest_SHA256=sha(ROOT/'test_results/full19_freeze/source_manifest.json'),
    verdict='A1/2 fails on actual tilt; B0/2 fails before motion on original input TTL. Both original FAILs retained; active-feedback downhill efficacy remains unmeasured.',
    limits=['GT only evaluates original windows under one actual initial SE3, and never constructs target or heading.',
        'The two fresh births share source/runtime/declared spawn but not microscopic physical state or scheduling history.',
        'B has no motion/request and cannot be compared with A as an enabled-vs-disabled balance-effect experiment.',
        'Recorder footer says all received messages retained; no original payload/header scan was performed and no DDS-complete stream is claimed.',
        'A raw/FAST omissions remain explicit and are neither repaired nor assigned as the unique tilt cause.'])
p=ROOT/'slam/test_results/downhill_AB_core_independent_final.json';p.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps(dict(report=str(p),sha256=sha(p),count=len(checks),alltrue=all(checks.values()),
    A_regions=[x['passed'] for x in a['independent_original_regions']['regions']],A_GT=a['independent_original_regions']['coverage'],
    A_crossings=a['active_first_Euler_crossings_same_time_GT'],B_firstfailure=b['actual_first_failed_body_tick']),ensure_ascii=False)[:4200])
