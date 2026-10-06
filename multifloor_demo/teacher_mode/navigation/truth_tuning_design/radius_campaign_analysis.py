#!/usr/bin/env python3
"""Read-only V3 plant receipts, with new exclusive campaign artifacts only."""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
os.environ['OMP_NUM_THREADS']='1';os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['MKL_NUM_THREADS']='1'
sys.dont_write_bytecode=True
import numpy as np

APPROVED_ANALYZER_SHA='b2663570a8ae9dcc53b26638a81e9b86389087e9cd488cbe22aa4e48a70ffc07'
APPROVED_PROTOCOL_SHA='2ad596f0c3815c8a12ba7f919b24f8087115ea9a510aa4fa7a8301300f459847'
MODEL_SHA='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
GATES=('frozen_sources_and_protocol','prospective_signed_plan_grid','complete_owned_runtime',
       'explicit_unobstructed_plant_fixture','exclusive_native_PD_DC_motor_contract','original_worker_native_state_phase',
       'complete_monotonic_native_and_50Hz_policy','frozen_CPU_Teacher_continuously_active',
       'original_velocity_schedule_and_slew','no_runtime_or_physical_fault',
       'body_contact_clearance_and_attitude_safety','all_native_torque_and_joint_speed_limits',
       'actual_PD_speed_torque_curve_recomputed','full_continuous_cruise_coverage','continuous_forward_and_yaw_response',
       'never_in_place_or_intermittent_stop_in_cruise','valid_stable_geometric_circle_arc',
       'fixed_5s_Teacher_zero_velocity_parking')
INPUTS=('actuator.jsonl','telemetry.jsonl','plant_commands.jsonl','observations_actions.npz',
        'runtime_manifest.json','policy_manifest.json','worker_result.json','source_manifest.json',
        'fixture_manifest.json','curvature_plan.json','protocol.json')


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def read(path):
    def invalid(token):raise ValueError('Nonfinite JSON '+token)
    return json.loads(Path(path).read_text(),parse_constant=invalid)


def lines(path):
    with Path(path).open() as f:
        for n,line in enumerate(f,1):
            if line.strip():
                try:yield json.loads(line,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
                except ValueError as exc:raise ValueError(f'{path}:{n}: {exc}') from exc


def clean(value):
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def check_receipt_status(summary):
    checks=summary.get('checks',{});missing=[name for name in GATES if name not in checks]
    invalid=[name for name,item in checks.items() if not isinstance(item,dict) or item.get('status') not in ('passed','failed','unverified')]
    conjunction=not missing and not invalid and all(item['status']=='passed' for item in checks.values())
    actual=summary.get('status')
    if actual not in ('passed','failed') or (actual=='passed')!=conjunction:
        raise ValueError('Receipt status is inconsistent with required 18 gates')
    return missing,[name for name,item in checks.items() if item.get('status')!='passed']


def verify_case(index,reference,result):
    output={'index':index,'prospective_plan':reference,'integrity_status':'unverified','plant_status':'unverified'}
    try:
        path=Path(reference['path']).resolve()
        if sha(path)!=reference['sha256']:raise ValueError('Prospective plan SHA mismatch')
        plan=read(path);output.update(desired_speed_mps=plan['speed_mps'],signed_yaw_rate_radps=plan['yaw_rate_radps'],
            direction_sign=1 if plan['yaw_rate_radps']>0 else -1,command_radius_m=plan['command_radius_m'])
        if result is None:
            output['integrity_status']='pending';output['plant_status']='pending';return output
        if result.get('index')!=index or result.get('plan')!=reference:raise ValueError('Result is not the original matching plan/index')
        run=Path(result['run']).resolve();summary_path=run/'summary_radius_independent.json';summary=read(summary_path)
        if summary.get('schema')!='teacher_continuous_turn_plant_independent/v1' or Path(summary['run']).resolve()!=run:
            raise ValueError('Original independent receipt identity mismatch')
        if sha(run/'curvature_plan.json')!=reference['sha256']:raise ValueError('Run used a different plan')
        if summary.get('analyzer_sha256')!=APPROVED_ANALYZER_SHA or sha(run/'sources/curvature/evaluate_radius.py')!=APPROVED_ANALYZER_SHA:
            raise ValueError('Independent evaluator is not frozen V3')
        if plan.get('protocol_sha256')!=APPROVED_PROTOCOL_SHA or sha(run/'sources/curvature/protocol.json')!=APPROVED_PROTOCOL_SHA:
            raise ValueError('Prospective protocol is not approved')
        if plan.get('model_sha256')!=MODEL_SHA:raise ValueError('Frozen Teacher mismatch')
        missing,nonpassing=check_receipt_status(summary)
        if result.get('status')!=summary['status']:raise ValueError('Results registry status differs from original receipt')
        if not isinstance(summary.get('input_hashes'),dict) or set(summary['input_hashes'])!=set(INPUTS):
            raise ValueError('Original 11 input hashes are incomplete/ambiguous')
        input_hashes={}
        for name,digest in summary['input_hashes'].items():
            original=run/'sources/curvature/protocol.json' if name=='protocol.json' else run/name
            actual=sha(original);input_hashes[name]=actual
            if actual!=digest:raise ValueError('Original input hash mismatch: '+name)
        manifest=read(run/'source_manifest.json');source_errors=[]
        for key,item in manifest.items():
            snapshot=Path(item['path'])
            if not snapshot.is_absolute() or not snapshot.is_relative_to(run) or not snapshot.is_relative_to(run/'sources') and not item.get('generated_input'):
                source_errors.append(key+':not archived source/input');continue
            if sha(snapshot)!=item['sha256']:source_errors.append(key+':SHA')
            expected=item.get('frozen_plan_digest')
            if expected and expected!=item['sha256']:source_errors.append(key+':plan digest')
        if source_errors:raise ValueError('Archived source mismatch: '+','.join(source_errors))
        # Source entries describe the executed archives, not a post-run current
        # checkout. Every planned original source must have its archived copy.
        for origin,digest in plan['source_hashes'].items():
            candidates=[item for item in manifest.values() if item.get('original_path')==origin]
            if not candidates or not any(item['sha256']==digest and item.get('original_sha256')==digest for item in candidates):
                raise ValueError('Plan source lacks exact original archive: '+origin)
        output.update(run=str(run),summary_path=str(summary_path),summary_sha256=sha(summary_path),
            integrity_status='passed',plant_status=summary['status'],missing_gates_in_failed_receipt=missing,
            nonpassing_gates=nonpassing,checks=summary['checks'],metrics=summary.get('metrics',{}),
            input_hashes=input_hashes,archived_source_count=len(manifest),analyzer_sha256=summary['analyzer_sha256'])
        if summary['status']=='passed':
            radius=summary['metrics']['actual_fitted_radius_m']
            if not isinstance(radius,(int,float)) or not math.isfinite(radius) or radius<=0:
                raise ValueError('PASS receipt lacks finite actual fitted radius')
        return output
    except Exception as exc:
        output['integrity_status']='failed';output['verification_error']=repr(exc);return output


def select_minimum(cases,speed,sign,complete,hashes):
    relevant=[c for c in cases if c.get('desired_speed_mps')==speed and c.get('direction_sign')==sign]
    passed=[c for c in relevant if c['integrity_status']=='passed' and c['plant_status']=='passed']
    status='passed' if complete and passed else 'pending' if not complete else 'unverified'
    answer={'status':status,'desired_speed_mps':speed,'direction_sign':sign,
            'scope':'Smallest fitted actual radius among passed tested commands at this speed/sign; not a global physical limit',
            'tested_case_indices':[c['index'] for c in relevant],'passed_tested_case_indices':[c['index'] for c in passed],
            'three_repeat_boundary_validation':'unverified','is_global_minimum':False,**hashes}
    if passed:
        chosen=min(passed,key=lambda c:(c['metrics']['actual_fitted_radius_m'],c['index']))
        answer.update(minimum_of_passed_tested_actual_radii_m=chosen['metrics']['actual_fitted_radius_m'],
            source_path=chosen['summary_path'],source_sha256=chosen['summary_sha256'],selected_case_index=chosen['index'],
            requested_yaw_rate_radps=chosen['signed_yaw_rate_radps'],command_radius_m=chosen['command_radius_m'])
    else:answer['reason']='No complete verified PASS case at this speed/sign'
    return answer


def native_curves(case):
    run=Path(case['run']);first=next(lines(run/'telemetry.jsonl'))
    origin=first['world_sim_time']-.005
    native=[r for r in lines(run/'actuator.jsonl') if r.get('kind')=='physics_step']
    t=np.array([r['t']-r['dt']-origin for r in native]);position=np.array([r['position'] for r in native])
    com=np.array([r['body_lin_vel_com'] for r in native]);omega=np.array([r['body_ang_vel'] for r in native])
    contract=next(r for r in lines(run/'actuator.jsonl') if r.get('kind')=='actuator_contract')
    body_origin=com-np.cross(omega,np.broadcast_to(contract['base_com_offset_body_m'],omega.shape))
    cruise=(t>=6-1e-9)&(t<=26+1e-9)
    if not cruise.any():return None
    return {'t':t[cruise],'xy':position[cruise,:2],'v':body_origin[cruise,0],'w':omega[cruise,2]}


def figures(output,cases,minima):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={.2:'#2474B4',.4:'#ED8A24',.6:'#3B9467',.8:'#BE4C65'}
    xyfig,xyaxes=plt.subplots(3,2,figsize=(12,15),constrained_layout=True)
    curvefig,curveaxes=plt.subplots(3,4,figsize=(19,10),constrained_layout=True)
    radiusfig,radiusaxes=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    plot_sources=[]
    for case in cases:
        if case['integrity_status']!='passed':continue
        data=native_curves(case)
        if data is None:continue
        v=case['desired_speed_mps'];signed=case['signed_yaw_rate_radps'];ridx=[.3,.5,1.].index(v);signcol=0 if signed<0 else 1;color=colors[abs(signed)]
        label=f"w={signed:+.1f}, {case['plant_status'].upper()}"
        ax=xyaxes[ridx,signcol];ax.plot(data['xy'][:,0],data['xy'][:,1],color=color,lw=1.2,label=label)
        ax.plot(*data['xy'][0],marker='o',color=color,ms=3);ax.plot(*data['xy'][-1],marker='x',color=color,ms=4)
        fit=case['metrics'].get('circle_fit')
        if fit:
            center=np.asarray(fit['center_xy_m']);rad=fit['radius_m'];a=np.linspace(0,2*math.pi,301)
            ax.plot(center[0]+rad*np.cos(a),center[1]+rad*np.sin(a),color=color,ls='--',lw=.7,alpha=.65)
        base=signcol*2
        curveaxes[ridx,base].plot(data['t'],data['v'],color=color,lw=.7,label=label)
        curveaxes[ridx,base+1].plot(data['t'],data['w'],color=color,lw=.7,label=label)
        curveaxes[ridx,base+1].axhline(signed,color=color,ls='--',lw=.6)
        fit_radius=case['metrics'].get('actual_fitted_radius_m')
        if fit_radius is not None:
            radiusaxes[ridx].plot(abs(signed),fit_radius,marker='o' if case['plant_status']=='passed' else 'x',
                                  color='#2474B4' if signed>0 else '#BE4C65',ms=7)
        plot_sources.append({'case_index':case['index'],'actuator_sha256':case['input_hashes']['actuator.jsonl'],
                             'telemetry_sha256':case['input_hashes']['telemetry.jsonl'],'plotted_cruise_samples':len(data['t'])})
    for ridx,v in enumerate([.3,.5,1.]):
        for scol,sign in enumerate([-1,1]):
            ax=xyaxes[ridx,scol];ax.set_aspect('equal',adjustable='datalim');ax.set(title=f'v={v:.1f} m/s, {"right" if sign<0 else "left"} turn: actual origin XY',xlabel='Gazebo X (m)',ylabel='Gazebo Y (m)');ax.grid(alpha=.2)
            if ax.lines:ax.legend(fontsize=8)
            for j in (0,1):
                ca=curveaxes[ridx,scol*2+j];ca.grid(alpha=.2);ca.set(xlim=(6,26),xlabel='Effective physics time (s)',ylabel='Origin body forward (m/s)' if j==0 else 'Body yaw rate (rad/s)',title=f'v={v:.1f}, {"right" if sign<0 else "left"}: {"forward" if j==0 else "yaw"}')
                if j==0:ca.axhline(v,color='black',ls='--',lw=.8)
                if ca.lines:ca.legend(fontsize=6.5)
        ax=radiusaxes[ridx];w=np.array([.2,.4,.6,.8]);ax.plot(w,v/w,ls=':',color='gray',label='command v/|w|')
        for sign,color in [(1,'#2474B4'),(-1,'#BE4C65')]:
            selected=next(x for x in minima if x['desired_speed_mps']==v and x['direction_sign']==sign)
            if selected.get('minimum_of_passed_tested_actual_radii_m') is not None:
                ax.plot(abs(selected['requested_yaw_rate_radps']),selected['minimum_of_passed_tested_actual_radii_m'],marker='*',ms=15,color=color,label=f'{"left" if sign>0 else "right"} smallest tested PASS')
        ax.set(title=f'v={v:.1f} m/s',xlabel='Requested |body yaw rate| (rad/s)',ylabel='Radius (m)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    xyfig.suptitle('Actual cruise trajectories; dashed geometric fits. New plane, not a PID/SLAM route test.');curvefig.suptitle('Actual native 200Hz responses; dashed requests. Full fixed 6–26s cruise.');radiusfig.suptitle('Actual fitted radii: circle=PASS, x=FAIL; no global minimum or repeatability claim.')
    files=[]
    for fig,name in [(xyfig,'actual_xy_circle_arcs.png'),(curvefig,'actual_forward_yaw_responses.png'),(radiusfig,'tested_radius_by_speed_sign.png')]:
        p=output/name
        with p.open('xb') as handle:fig.savefig(handle,format='png',dpi=160)
        files.append({'path':str(p),'sha256':sha(p)});plt.close(fig)
    return {'figures':files,'original_plot_sources':plot_sources}


def analyze(campaign,final=False,output=None,plots=True):
    campaign=Path(campaign).resolve();plan_path=campaign/'phase_radius_v3_plan.json'
    results_path=campaign/'phase_radius_v3_results.jsonl'
    if not results_path.exists():results_path=campaign/'results.jsonl'
    prospective=read(plan_path);registry=list(lines(results_path));references=prospective['plans']
    if prospective.get('schema')!='teacher_turn_radius_prospective_sweep/v1' or len(references)!=24:
        raise ValueError('Prospective campaign must contain original 24 cases')
    if len({r['path'] for r in references})!=24:raise ValueError('Duplicate prospective plans')
    indexed={}
    for result in registry:
        i=result.get('index')
        if type(i) is not int or i<0 or i>=24 or i in indexed:raise ValueError('Duplicate/out-of-scope result index')
        indexed[i]=result
    if final and len(indexed)!=24:raise ValueError('Final requires all 24 registered results; no output written')
    hashes={'campaign_plan_sha256':sha(plan_path),'campaign_results_sha256':sha(results_path)}
    cases=[verify_case(i,ref,indexed.get(i)) for i,ref in enumerate(references)]
    grid={(c.get('desired_speed_mps'),c.get('signed_yaw_rate_radps')) for c in cases}
    expected={(v,w) for v in [.3,.5,1.] for w in [-.8,-.6,-.4,-.2,.2,.4,.6,.8]}
    if grid!=expected:raise ValueError('Prospective grid differs from 24 signed cases')
    integrity=all(c['integrity_status']=='passed' for c in cases);complete=len(indexed)==24 and integrity
    minima=[select_minimum(cases,v,sign,complete,hashes) for v in [.3,.5,1.] for sign in [-1,1]]
    if output is None:
        stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        output=campaign/'analysis'/('final_' if final else 'partial_')/stamp
    output=Path(output).resolve()
    if not output.is_relative_to(campaign/'analysis'):raise ValueError('New output must stay inside campaign/analysis')
    output.mkdir(parents=True,exist_ok=False)
    summary={'schema':'teacher_turn_radius_campaign_analysis/v1','status':'passed' if complete else 'pending' if len(indexed)<24 else 'failed',
        'meaning_of_status':'Campaign coverage/source/receipt verification; does not require every plant command to pass',
        'final_requested':final,'completed_registered_cases':len(indexed),'expected_cases':24,
        'integrity_passed_cases':sum(c['integrity_status']=='passed' for c in cases),
        'plant_passed_cases':sum(c['plant_status']=='passed' and c['integrity_status']=='passed' for c in cases),
        'case_results':cases,'selected_tested_minima':minima,**hashes,
        'campaign_plan_path':str(plan_path),'campaign_results_path':str(results_path),'script_sha256':sha(__file__),
        'approved_original_analyzer_sha256':APPROVED_ANALYZER_SHA,'approved_protocol_sha256':APPROVED_PROTOCOL_SHA,
        'claim':'Per-speed/sign smallest geometric radius among passed tested commands only; first sweep has no three-repeat boundary validation',
        'PID_circle_tracking':'unverified','SLAM_navigation':'unverified','original_multifloor':'unverified','real_robot':'unverified'}
    with (output/'summary_radius_campaign.json').open('x') as f:json.dump(clean(summary),f,indent=2,allow_nan=False);f.write('\n')
    with (output/'selected_minimum_receipts.json').open('x') as f:json.dump(clean(minima),f,indent=2,allow_nan=False);f.write('\n')
    if plots:
        plot_receipt=figures(output,cases,minima)
        with (output/'figure_provenance.json').open('x') as f:json.dump(plot_receipt,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(output),'status':summary['status'],'registered':len(indexed),'verified':summary['integrity_passed_cases'],'plant_pass':summary['plant_passed_cases']}))
    return summary


def self_test():
    base={'status':'passed','checks':{k:{'status':'passed'} for k in GATES}}
    assert check_receipt_status(base)==([],[])
    missing=readback=json.loads(json.dumps(base));missing['checks'].pop(GATES[-1])
    try:check_receipt_status(missing)
    except ValueError:pass
    else:raise AssertionError('Missing gate passed')
    wrong=json.loads(json.dumps(base));wrong['checks'][GATES[0]]['status']='failed'
    try:check_receipt_status(wrong)
    except ValueError:pass
    else:raise AssertionError('Inconsistent PASS accepted')
    failed=json.loads(json.dumps(wrong));failed['status']='failed';assert GATES[0] in check_receipt_status(failed)[1]
    cases=[{'index':0,'desired_speed_mps':.3,'direction_sign':1,'plant_status':'passed','integrity_status':'passed','metrics':{'actual_fitted_radius_m':.7},'summary_path':'fixture-only','summary_sha256':'test','signed_yaw_rate_radps':.4,'command_radius_m':.75},
           {'index':1,'desired_speed_mps':.3,'direction_sign':1,'plant_status':'failed','integrity_status':'passed','metrics':{'actual_fitted_radius_m':.2}},
           {'index':2,'desired_speed_mps':.3,'direction_sign':1,'plant_status':'passed','integrity_status':'failed','metrics':{'actual_fitted_radius_m':.1}}]
    selected=select_minimum(cases,.3,1,True,{})
    assert selected['selected_case_index']==0 and selected['minimum_of_passed_tested_actual_radii_m']==.7
    assert select_minimum(cases,.3,1,False,{})['status']=='pending'
    assert select_minimum(cases,.3,-1,True,{})['status']=='unverified'
    print('SELF_TEST passed: mandatory gates, receipt conjunction, FAIL/integrity exclusions, partial and no-PASS selection')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--campaign',type=Path);ap.add_argument('--final',action='store_true')
    ap.add_argument('--partial',action='store_true');ap.add_argument('--no-plots',action='store_true');ap.add_argument('--output',type=Path)
    ap.add_argument('--self-test',action='store_true');args=ap.parse_args()
    if args.self_test:self_test()
    else:
        if args.campaign is None:ap.error('--campaign required')
        if args.final and args.partial:ap.error('Choose final or partial')
        analyze(args.campaign,args.final,args.output,not args.no_plots)
