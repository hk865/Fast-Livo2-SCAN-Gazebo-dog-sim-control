"""Offline existing observer, unchanged fit gates; never controls or consumes GT in SLAM observer."""
import hashlib,importlib.util,json,pathlib
HERE=pathlib.Path(__file__).resolve().parent
ROOT=HERE.parents[2]
COMMON=ROOT/'navigation/test_results/motion_window_comparison/compare_windows.py'
spec=importlib.util.spec_from_file_location('prior_unchanged_window_analysis',COMMON);shared=importlib.util.module_from_spec(spec);spec.loader.exec_module(shared)
reports={}
for suffix in ('a_disabled','b_enabled'):
    original=ROOT/'simulation/test_results'/('20261002_feedback_first4_'+suffix)
    result=json.loads((original/'first_four_result.json').read_text())
    normalized=HERE/('observer_input_'+suffix);normalized.mkdir(exist_ok=False)
    start,end=result['origin_stamp_ns'],result['terminal_stamp_ns']
    alignment=dict(result['region_evaluation']['initial_fixed_SE3_evaluation_only'])
    alignment['translation_world_from_slam']=alignment.pop('translation')
    (normalized/'alignment.json').write_text(json.dumps({'initial_SE3':alignment},indent=2)+'\n')
    with (normalized/'pose_audit.jsonl').open('x') as h:
        for source,key in [('slam','poses'),('truth','truth')]:
            for p in result[key]:h.write(json.dumps(dict(p,source=source,
                stage='navigating' if start<=p['stamp_ns']<=end else 'outside_component'))+'\n')
    with (normalized/'navigation_audit.jsonl').open('x') as h:
        for n in result['statuses']:h.write(json.dumps(n)+'\n')
    for name in ['joint_stop_adapter.jsonl','source_snapshot.tar.gz']:(normalized/name).symlink_to(original/name)
    report=shared.compare(normalized,HERE/('observer_'+suffix),normalized/'alignment.json')
    extra={}
    for label,w in report['windows'].items():
        snapshots=list(shared.rows(HERE/('observer_'+suffix)/('snapshots_'+label+'.jsonl')))
        labels=[];backward=[]
        for r in snapshots:
            s=r['slam'];g=r['GT_evaluation_only'];c=s.get('actual_command_average',[0.,0.,0.]);mode=s.get('execution_mode')
            a=dict(stage=r['stage'],stamp_ns=s['stamp_ns'],context=s.get('context_id'),command_sign=0,
                backward=False,backward_gt=False)
            if s['valid'] and mode in ('walk','turn'):
                velocity=s['window_body_twist']['linear'][0]
                a['backward']=velocity<-.02
                if g and g.get('valid') and g.get('window_start_ns')==s['window_start_ns']:
                    a['backward_gt']=a['backward'] and g['window_body_twist']['linear'][0]<-.02
                if a['backward']:backward.append(dict(stamp_ns=s['stamp_ns'],mode=mode,
                    measured_vx=velocity,command_vx=c[0],effective_age_s=s['effective_measurement_age_ns']/1e9,
                    GT_confirmed=a['backward_gt'],position_fit_rms=s['fit_residuals']['position_rms_m'],
                    observed_motion_ready=s.get('observed_motion_ready'),feedback_eligible=s['feedback_eligible']))
            labels.append(a)
        episodes=shared.episodes(labels,'backward_gt')
        extra[label]=dict(valid_backward_samples=len(backward),valid_GT_confirmed_backward=sum(r['GT_confirmed'] for r in backward),
            longest_same_context_continuous_confirmed_backward_s=max((r['duration_ns']/1e9 for r in episodes),default=0),
            actual_full_cycle_confirmed_backward_episodes=[r for r in episodes if r['at_least_one_nominal_gait_cycle']],
            backward_samples=backward)
    reports[suffix]=dict(window_result_sha256=shared.sha(HERE/('observer_'+suffix)/'result.json'),
        original_result_sha256=shared.sha(original/'first_four_result.json'),
        original_input_header_contract='original SLAM/GT integer stamps preserved; actual command cached sim float',
        ideal_wall_age_only=True,original_fixed_SE3=alignment,windows={k:dict(counts=v['counts'],reasons=v['reasons'],
            statistics=v['statistics'],opposite_episodes_summary=v['opposite_episodes_summary'],backward=extra[k]) for k,v in report['windows'].items()})
output=dict(scope=__doc__,groups=reports,observer_sha256=shared.sha(shared.OBSERVER),shared_analysis_sha256=shared.sha(COMMON),script_sha256=shared.sha(pathlib.Path(__file__)),
    limits=['Original fit gates .02m/.04rad, command gap .1s, pose gap .15s preserved; none used to generate a control output.',
       'Callback wall freshness is not present in driver pose array; this analysis sets ideal receipt age and does not validate a live observer.',
       'Existing observed_motion_ready expects driving while actual Adapter uses walk; feedback_eligible remains false.',
       'Nominal gait cycle .60s comes from original CHAMP phase config, not observed foot phase.',
       'A valid sustained backwards episode is a diagnostic fact, not a validated brake threshold or a joint controller replacement.'])
(HERE/'observer_strategy.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(dict(output=str(HERE/'observer_strategy.json'),sha256=shared.sha(HERE/'observer_strategy.json'),
    backward={k:{w:v['backward']['longest_same_context_continuous_confirmed_backward_s'] for w,v in r['windows'].items()} for k,r in reports.items()})))
