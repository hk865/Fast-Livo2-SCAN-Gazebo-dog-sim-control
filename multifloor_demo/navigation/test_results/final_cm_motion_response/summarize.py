#!/usr/bin/env python3
"""Summarize passive observations; no control policy/gain fitted to truth."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'navigation/test_results/motion_window_comparison'))
from compare_windows import stats,sha

report=dict(scope=__doc__,components={},full_demo_passed=False,
    same_nominal_gait_cycle_ns=600000000,actual_contact_phase_inferred=False,
    no_ROS_init_no_nodes_no_physics_no_production_edit=True,
    no_GT_control_or_gain_fit=True,
    caution='This is the same diagnostic definition, not matched plant trials: Full15 and three component routes, starts and durations differ.')
for kind in ['fourturn_v2','slope','dynamic']:
    out=HERE/('comparison_'+kind)
    original=json.loads((out/'result.json').read_text())
    provenance=json.loads((HERE/('native_inputs_'+kind)/'normalization_receipt.json').read_text())
    component=dict(active_interval_ns=[provenance['active_start_ns'],provenance['active_end_ns']],
        initial_SE3=original['initial_SE3'],input_sha256=provenance['input_sha256'],
        unchanged_observer_sha256=original['observer_source_sha256'],
        command_time_contract=provenance['actual_command_time'],windows={},
        report_sha256=sha(out/'result.json'),normalization_sha256=sha(HERE/('native_inputs_'+kind)/'normalization_receipt.json'))
    for label,base in original['windows'].items():
        snaps=[json.loads(line) for line in (out/f'snapshots_{label}.jsonl').open()]
        comparable=[];opposite=[];fit_rejected=[];long=[]
        for row in snaps:
            s,g,c=row['slam'],row['GT_evaluation_only'],row['command_window_quality']
            if not s.get('available') or s['execution_mode']!='walk':continue
            if not s['valid']:
                if g and g.get('available'):
                    r=g['fit_residuals']
                    fit_rejected.append(r['position_rms_m']>.02 or max(r['rotation_rms_rad'],r['heading_rms_rad'])>.04)
                continue
            if c:
                latest=c['latest_applied_command']
                a=[max(0.,.12-latest[0]),max(0.,.08-abs(latest[2]))]
                comparable.append(a)
                if row['classifications']['opposite_body_GT_confirmed_consistent_command_sign']:
                    opposite.append(a)
        eps=json.loads((out/f'episodes_{label}.json').read_text())
        long=[event for event in eps['opposite_body_GT_confirmed_consistent_command_sign']['episodes'] if event['at_least_one_nominal_gait_cycle']]
        component['windows'][label]=dict(
            raw_counts=base['counts'],fit_and_transition_reasons=base['reasons'],transition_causes=base['transition_causes'],
            opposite_summaries=base['opposite_episodes_summary'],long_consistent_opposite_episodes=long,
            current_same_direction_cap_headroom_vx_yaw=stats(comparable),
            current_consistent_opposite_cap_headroom_vx_yaw=stats(opposite),
            fit_rejected_walk_GT_same_window_count=len(fit_rejected),
            fit_rejected_walk_GT_also_over_same_gates=sum(fit_rejected),
            valid_walk_statistics=base['statistics'].get('walk_valid'),
            valid_turn_statistics=base['statistics'].get('turn_valid'),
            valid_zero_statistics=base['statistics'].get('zero_valid'))
    report['components'][kind]=component
old=ROOT/'navigation/test_results/motion_window_comparison/full15_v2/result.json'
oldr=json.loads(old.read_text())
report['prior_full15_reference']=dict(result_sha256=sha(old),
    observer_sha256=oldr['observer_source_sha256'],
    same_observer_source=all(x['unchanged_observer_sha256']==oldr['observer_source_sha256'] for x in report['components'].values()),
    windows={label:dict(raw_counts=w['counts'],fit_and_transition_reasons=w['reasons'],
            consistent_opposite_summary=w['opposite_episodes_summary']['opposite_body_GT_confirmed_consistent_command_sign'])
            for label,w in oldr['windows'].items()})
report['conclusions_and_limits']=[
    'At .6s, all three components have short SLAM+GT-confirmed opposite-sign walk windows; none stays continuously valid/opposite with the same context and command sign for >=.60s.',
    'This does not prove absence of sustained wrong response: fit rejection/transition gates interrupt coverage; longer windows have still fewer valid walk estimates.',
    'Valid .6s body-forward velocity remains below actual commanded vx and latest vx has almost no positive .12cap headroom.',
    'Valid .6s pure-turn body-forward velocity is negative in fourturn/dynamic; zero-vx intent is not zero translation.',
    'SLAM and independent GT body rates closely agree on qualified windows; no no-slip assumption or GT feedback was used.',
    'Window effective age is a finite-window delay only; comparison replays ideal receipt freshness, not proof of real-time estimator/transport age.',
    'The unchanged observer expects driving while actual adapter state is walk; observed_motion_ready is false and feedback_eligible remains hard false. Future actuation needs a separate interface contract.',
    'Do not infer CM causality or full-route success from three different bounded component routes or from these overlapping fit counts.',
    'Next use a frozen real-time read-only motion observer with native body Header ns, actual applied commands and quality/phase tags. Only after a reproducible cross-cycle unsaturated yaw discrepancy should a separately verified bounded low-frequency correction be considered.',
    'No forward P gain can create missing headroom under the original .12cap; pure-turn retreat also cannot be cancelled by walk-only feedback without a separate physically verified strategy.'
]
(HERE/'comparison_summary.json').write_text(json.dumps(report,indent=2)+'\n')
receipt=dict(summary_sha256=sha(HERE/'comparison_summary.json'),scripts={p.name:sha(p) for p in [Path(__file__),HERE/'compare_components.py']},
    outputs={kind:{p.name:sha(p) for p in (HERE/('comparison_'+kind)).iterdir() if p.is_file()} for kind in report['components']})
(HERE/'final_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
for kind,c in report['components'].items():
    print(kind)
    for label,w in c['windows'].items():
        e=w['opposite_summaries']['opposite_body_GT_confirmed_consistent_command_sign']
        print(label,w['raw_counts'].get('valid_walk',0),e['count'],e['max_duration_s'],e['at_least_one_nominal_cycle_count'])
