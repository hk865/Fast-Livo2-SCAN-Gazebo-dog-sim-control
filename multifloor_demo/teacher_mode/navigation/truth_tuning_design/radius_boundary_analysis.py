#!/usr/bin/env python3
"""New boundary 3-repeat receipts; never rewrite the original sweep minimum."""
import argparse
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
HELPER_SHA='fe3a6f81a3085a91a657a7dbc73c73523924fad733c328ab78faa1a1f88d4e87'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def helper():
    path=HERE/'radius_campaign_analysis.py'
    if sha(path)!=HELPER_SHA:raise ValueError('Frozen original receipt auditor changed')
    spec=importlib.util.spec_from_file_location('_frozen_radius_boundary_receipt_auditor',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def summarize_group(speed,sign,original,new_cases,finished):
    cases=[original]+new_cases
    repetitions=[c.get('repetition') for c in cases]
    identity=all(c.get('desired_speed_mps')==speed and c.get('direction_sign')==sign and
                 c.get('signed_yaw_rate_radps')==original.get('signed_yaw_rate_radps') for c in cases)
    pass_cases=[c for c in cases if c.get('integrity_status')=='passed' and c.get('plant_status')=='passed']
    triple=len(cases)==3 and sorted(repetitions)==[1,2,3] and identity
    ok=finished and triple and len(pass_cases)==3
    result={'schema':'teacher_tested_turn_boundary_three_repeat/v1','status':'passed' if ok else 'pending' if not finished else 'failed',
            'desired_speed_mps':speed,'direction_sign':sign,'requested_yaw_rate_radps':original.get('signed_yaw_rate_radps'),
            'repetitions':repetitions,'three_repeat_identity_checked':triple,'verified_pass_repeats':len(pass_cases),
            'original_sweep_minimum_receipt_unchanged':{'path':original.get('summary_path'),'sha256':original.get('summary_sha256')},
            'original_first_sweep_actual_radius_m':original.get('metrics',{}).get('actual_fitted_radius_m'),
            'claim':'3/3 of one previously selected minimum-tested command at this speed/sign; not a global minimum, nor PID/SLAM curve tracking',
            'global_minimum_claim':False,'PID_curve_tracking':'unverified','SLAM_navigation':'unverified',
            'footprint_and_obstacle_inflation':'Must be evaluated separately from the centerline-radius margin',
            'source_receipts':[]}
    for c in cases:
        metrics=c.get('metrics',{})
        result['source_receipts'].append({'repetition':c.get('repetition'),'run':c.get('run'),
            'source_path':c.get('summary_path'),'source_sha256':c.get('summary_sha256'),
            'integrity_status':c.get('integrity_status'),'plant_status':c.get('plant_status'),
            'actual_fitted_radius_m':metrics.get('actual_fitted_radius_m'),
            'circle_radial_RMSE_m':metrics.get('circle_fit',{}).get('radial_rmse_m'),
            'radius_subfit_CV':metrics.get('actual_radius_subfit_cv'),
            'forward_MAE_mps':metrics.get('speed_forward_mae_mps'),'yaw_rate_MAE_radps':metrics.get('yaw_rate_mae_radps'),
            'fixed_parking':metrics.get('parking'),'original_input_hashes':c.get('input_hashes'),
            'nonpassing_gates':c.get('nonpassing_gates'),'verification_error':c.get('verification_error')})
    if ok:
        radii=[c['metrics']['actual_fitted_radius_m'] for c in cases]
        worst=max(cases,key=lambda c:c['metrics']['actual_fitted_radius_m'])
        result.update(actual_radius_range_m=[min(radii),max(radii)],
            worst_passed_actual_radius_m=max(radii),prospective_centerline_margin_factor=1.5,
            suggested_planning_centerline_radius_at_least_m=1.5*max(radii),
            conservative_planning_source_path=worst['summary_path'],conservative_planning_source_sha256=worst['summary_sha256'],
            suggestion_status='Prospective geometry only; actual circle/S tracking remains unverified')
    return result


def analyze(campaign,final=False,output=None):
    audit=helper();campaign=Path(campaign).resolve()
    prospective_path=campaign/'phase_boundary_v3_plan.json';result_path=campaign/'phase_boundary_v3_results.jsonl'
    if not result_path.exists():result_path=campaign/'results.jsonl'
    prospective=audit.read(prospective_path);registry=list(audit.lines(result_path))
    if prospective.get('schema')!='teacher_tested_turn_boundary_prospective_repeat/v1' or len(prospective.get('plans',[]))!=12 or len(prospective.get('selection',[]))!=6:
        raise ValueError('Original boundary plan must be 12 new cases and six selected originals')
    if prospective.get('total_repeats_including_sweep')!=3 or prospective.get('global_minimum_claim') is not False:
        raise ValueError('Boundary scope changed')
    indexed={}
    for row in registry:
        i=row.get('index')
        if type(i) is not int or not 0<=i<12 or i in indexed:raise ValueError('Duplicate/out-of-scope boundary result index')
        indexed[i]=row
    if final and len(indexed)!=12:raise ValueError('Final needs all 12 new results; no output written')
    sweep_results_path=campaign/'phase_radius_v3_results.jsonl';sweep_plan_path=campaign/'phase_radius_v3_plan.json'
    if sha(sweep_results_path)!=prospective['source_sweep_sha256']:raise ValueError('Original sweep registry changed')
    sweep_plan=audit.read(sweep_plan_path);sweep_registry=list(audit.lines(sweep_results_path))
    if len(sweep_registry)!=24 or len(sweep_plan['plans'])!=24:raise ValueError('Original complete sweep is missing')
    sweep_by_index={r['index']:r for r in sweep_registry}
    if set(sweep_by_index)!=set(range(24)):raise ValueError('Original sweep identity is incomplete')
    originals=[]
    for selected in prospective['selection']:
        source=Path(selected['source_path']).resolve()
        if sha(source)!=selected['source_sha256']:raise ValueError('Selected original receipt SHA mismatch')
        matches=[(i,r) for i,r in sweep_by_index.items() if Path(r['run']).resolve()==source.parent]
        if len(matches)!=1:raise ValueError('Selected receipt does not have one original sweep case')
        i,row=matches[0];case=audit.verify_case(i,sweep_plan['plans'][i],row)
        if case['integrity_status']!='passed' or case['plant_status']!='passed':raise ValueError('Selected first repetition is not a verified original PASS')
        p=audit.read(Path(case['run'])/'curvature_plan.json');case['repetition']=p['repetition']
        if case['repetition']!=1 or case['desired_speed_mps']!=selected['speed_mps'] or case['direction_sign']!=selected['sign'] or case['signed_yaw_rate_radps']!=selected['yaw_rate_radps'] or case['metrics']['actual_fitted_radius_m']!=selected['actual_radius_m']:
            raise ValueError('Selected original command/radius/repetition mismatch')
        originals.append(case)
    keys={(c['desired_speed_mps'],c['direction_sign']) for c in originals}
    if keys!={(v,s) for v in [.3,.5,1.] for s in [-1,1]}:raise ValueError('Selected six speed/sign identities changed')
    repeats=[]
    for i,reference in enumerate(prospective['plans']):
        case=audit.verify_case(i,reference,indexed.get(i))
        p=audit.read(reference['path']);case['repetition']=p['repetition']
        if case['repetition'] not in (2,3):raise ValueError('New case is not fixed repetition2/3')
        original=next((c for c in originals if c['desired_speed_mps']==case.get('desired_speed_mps') and c['direction_sign']==case.get('direction_sign')),None)
        if original is None or p['yaw_rate_radps']!=original['signed_yaw_rate_radps']:
            raise ValueError('Repeat command is different from selected boundary')
        op=audit.read(Path(original['run'])/'curvature_plan.json')
        # A repetition label is the only allowed plan semantic change. Source
        # hashes, timing, fixture and Actor are required to match exactly.
        semantic_new={k:v for k,v in p.items() if k!='repetition'}
        semantic_old={k:v for k,v in op.items() if k!='repetition'}
        if semantic_new!=semantic_old:raise ValueError('Repeat changed frozen command/scene/runtime source semantics')
        repeats.append(case)
    repeat_keys=[(c.get('desired_speed_mps'),c.get('direction_sign'),c['repetition']) for c in repeats]
    if len(set(repeat_keys))!=12:raise ValueError('Duplicate/missing prospective repeat identities')
    finished=len(indexed)==12
    boundaries=[]
    for original in originals:
        v=original['desired_speed_mps'];sign=original['direction_sign']
        relevant=[c for c in repeats if c.get('desired_speed_mps')==v and c.get('direction_sign')==sign]
        boundaries.append(summarize_group(v,sign,original,relevant,finished))
    integrity=all(c.get('integrity_status')=='passed' for c in repeats+originals)
    all_pass=finished and integrity and all(b['status']=='passed' for b in boundaries)
    summary={'schema':'teacher_turn_boundary_campaign_analysis/v1','status':'passed' if all_pass else 'pending' if not finished else 'failed',
        'completed_new_cases':len(indexed),'expected_new_cases':12,'selected_original_cases':6,
        'verified_original_and_new_receipts':sum(c.get('integrity_status')=='passed' for c in repeats+originals),
        'all_18_gates_passed_each_of_18_runs':all_pass,'boundaries':boundaries,
        'source_hashes':{'boundary_prospective_plan':sha(prospective_path),'boundary_results':sha(result_path),
            'original_sweep_plan':sha(sweep_plan_path),'original_sweep_results':sha(sweep_results_path),
            'frozen_receipt_auditor':HELPER_SHA,'analysis_script':sha(__file__)},
        'original_sweep_minimum_receipts_rewritten':False,'global_minimum_claim':False,
        'outer_sweep_launcher_143_is_separate_preserved_note':bool(prospective.get('outer_sweep_launcher_exit143_after_SWEEP_COMPLETE_preserved')),
        'scope':'Three repetitions at each speed/sign of the previously selected minimum-tested command; no tighter radius is tested',
        'planning_advice':'Centerline R >=1.5*worst actual radius among three PASS repetitions; footprint, obstacle inflation, circle/S closed-loop feasibility separate'}
    if output is None:
        stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        output=campaign/'boundary_analysis'/('final_' if final else 'partial_')/stamp
    output=Path(output).resolve()
    if not output.is_relative_to(campaign/'boundary_analysis'):raise ValueError('Output must stay inside the new boundary_analysis directory')
    output.mkdir(parents=True,exist_ok=False)
    for name,content in [('summary_radius_boundary_campaign.json',summary),('boundary_receipts.json',boundaries)]:
        with (output/name).open('x') as f:json.dump(audit.clean(content),f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(output),'status':summary['status'],'new_cases':len(indexed),'verified_receipts':summary['verified_original_and_new_receipts']}))
    return summary


def self_test():
    def case(rep,radius,status='passed'):
        return {'repetition':rep,'desired_speed_mps':.3,'direction_sign':1,'signed_yaw_rate_radps':.8,
                'integrity_status':'passed','plant_status':status,'metrics':{'actual_fitted_radius_m':radius},
                'summary_path':'PURE_FIXTURE_ONLY','summary_sha256':'fixture'}
    b=summarize_group(.3,1,case(1,.35),[case(2,.37),case(3,.36)],True)
    assert b['status']=='passed' and b['actual_radius_range_m']==[.35,.37]
    assert b['suggested_planning_centerline_radius_at_least_m']==1.5*.37
    assert b['conservative_planning_source_path']=='PURE_FIXTURE_ONLY'
    assert summarize_group(.3,1,case(1,.35),[case(2,.37),case(3,.36,'failed')],True)['status']=='failed'
    assert summarize_group(.3,1,case(1,.35),[case(2,.37)],False)['status']=='pending'
    assert summarize_group(.3,1,case(1,.35),[case(2,.37),case(2,.36)],True)['status']=='failed'
    changed=case(3,.36);changed['signed_yaw_rate_radps']=.6
    assert summarize_group(.3,1,case(1,.35),[case(2,.37),changed],True)['status']=='failed'
    print('SELF_TEST PASS: repetitions/command identity, FAIL and partial exclusions, worst-pass 1.5 planning margin')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--campaign',type=Path);ap.add_argument('--final',action='store_true')
    ap.add_argument('--output',type=Path);ap.add_argument('--self-test',action='store_true');a=ap.parse_args()
    if a.self_test:self_test()
    else:
        if a.campaign is None:ap.error('--campaign required')
        analyze(a.campaign,a.final,a.output)
