"""Observed effective effort response, not a Go2 plant fit or adopted gain."""
from pathlib import Path
import hashlib,json
import numpy as np
HERE=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
attempt=Path((HERE/'latest_attempt.txt').read_text().strip())
execution=json.loads((attempt/'execution_receipt.json').read_text())
checks={};reports={};loaded={};data={}

def fit(rows,key):
    x=np.asarray([[r['preset_error'], -r['preset_velocity'],1.] for r in rows])
    y=np.asarray([r[key] for r in rows]);c=np.linalg.lstsq(x,y,rcond=None)[0]
    return dict(P=float(c[0]),D=float(c[1]),intercept=float(c[2]),
                max_abs_residual=float(np.max(np.abs(x@c-y))),conditions=len(rows),rank=int(np.linalg.matrix_rank(x)))

for flag in ('false','true'):
    allrows=[];batches=[];mapset=[]
    for batch in range(3):
        label=flag+'_batch'+str(batch)
        stream=[json.loads(line) for line in (attempt/(label+'.jsonl')).open()]
        config=next(x for x in stream if x['kind']=='configuration')
        service=next(x for x in stream if x['kind']=='parameter_service')
        preset=next(x for x in stream if x['kind']=='prescribed_conditions')
        samples=[x for x in stream if x['kind']=='sample' and x['stage']=='quasistatic']
        inputs=[x for x in stream if x['kind']=='actual_input']
        initial=next(x for x in stream if x['kind']=='sample' and x['stage']=='activation')
        base=initial['time_ns'];i_component=np.zeros(12);anchor_i=np.zeros(12)
        residuals=[];ffresiduals=[];errors=[];i_errors=[]
        for row in samples:
            pe=np.asarray(row['actual_pid_p_error']);de=np.asarray(row['actual_pid_d_error'])
            ie=np.asarray(row['actual_pid_i_weighted']);pid=np.asarray(row['actual_pid_command'])
            ff=np.asarray(row['next_effort']);out=np.asarray(row['output_effort'])
            before=np.asarray(row['before_point_effort_from_observed_source_branch'])
            if row['actual_trajectory_pointer_changed']:anchor_i=i_component.copy()
            alpha=float(np.dot(ff,before)/np.dot(before,before)) if np.dot(before,before)>1e-24 else 0.
            ffresiduals.append(float(np.max(np.abs(ff-alpha*before))))
            # Exact observed linear effort carry + actual weighted integral,
            # separated from P/D. No JTC private state is altered or reset here.
            i_component=alpha*anchor_i+ie
            residuals.append(max(float(np.max(np.abs(out-ff-pid))),float(np.max(np.abs(pid-100*pe-de-ie)))))
            errors.append(float(np.max(np.abs(np.asarray(row['actual_position'])-row['hardware_position']))))
            i_errors.append(float(np.max(np.abs(ie))))
            for joint in range(12):
                case=batch*12+joint
                if case>=35:continue
                allrows.append(dict(condition=case,batch=batch,joint=joint,time_ns=row['time_ns'],
                    phase_ns=(row['time_ns']-base)%20000000,preset_error=preset['target_minus_measured_position'][joint],
                    preset_velocity=preset['preset_measured_velocity'][joint],output=float(out[joint]),
                    output_without_integral_carry=float(out[joint]-i_component[joint]),
                    actual_pid_weighted_i=float(ie[joint]),total_integral_carry_component=float(i_component[joint]),
                    actual_feedforward=float(ff[joint]),actual_pid=float(pid[joint]),elapsed_ns=row['time_ns']-base))
        checks[label+'_actual_bool_closed_loop_gains']=config['actual_parameter']==(flag=='true') and config['closed_loop_effort_pid'] is True and config['open_loop_control'] is False and config['is_async'] is False and service['p']==100 and service['i']==.2 and service['d']==1
        checks[label+'_200Hz_fresh_real_position_only_inputs']=len(inputs)==241 and all(x['header_ns']==0 and x['horizon_ns']==16666666 and x['new_callback_observed'] for x in inputs) and all(b['time_ns']-a['time_ns']==5000000 for a,b in zip(inputs,inputs[1:]))
        checks[label+'_250Hz_real_returned_periods']=len(samples)==300 and all(x['actual_returned_success'] and x['actual_returned_period_ns']==4000000 for x in samples) and all(b['time_ns']-a['time_ns']==4000000 for a,b in zip(samples,samples[1:]))
        checks[label+'_preset_actual_memory_states_unchanged']=max(errors)<1e-12 and all(x['hardware_position']==initial['hardware_position'] and x['hardware_velocity']==initial['hardware_velocity'] for x in samples)
        checks[label+'_PID_plus_actual_FF_reconstructed']=max(residuals)<1e-11
        checks[label+'_observed_linear_effort_carry']=max(ffresiduals)<1e-11
        checks[label+'_activation_command_zero']=all(v==0 for v in initial['output_effort'])
        if flag=='true':checks[label+'_actual_reference_effort_FF_always_zero']=all(v==0 for x in [initial,*samples] for v in x['next_effort'])
        batches.append(dict(batch=batch,samples=len(samples),inputs=len(inputs),config=config,actual_service=service,
            max_pid_plus_ff_residual=max(residuals),max_observed_ff_carry_residual=max(ffresiduals),
            weighted_integral_abs_peak=max(i_errors),actual_next_effort_abs_peak=float(np.max(np.abs([x['next_effort'] for x in samples]))),
            actual_ff_velocity_scale=preset['actual_ff_velocity_scales'],actual_pointer_changes=sum(x['actual_trajectory_pointer_changed'] for x in samples)))
        files=set()
        for line in (attempt/(label+'.jsonl.maps')).read_text().splitlines():
            path=line.split()[-1]
            if path.startswith('/') and any(n in path for n in ('libjoint_trajectory_controller.so','libcontroller_interface.so','libcontrol_toolbox.so')):files.add(path)
        lm={p:sha(Path(p)) for p in sorted(files)};mapset.append(lm);loaded[label]=lm
    steady=[x for x in allrows if x['elapsed_ns']>400000000]
    means=[]
    for case in range(35):
        rr=[x for x in steady if x['condition']==case]
        a={k:rr[0][k] for k in ('condition','preset_error','preset_velocity')}
        a.update({k:float(np.mean([r[k] for r in rr])) for k in ('output','output_without_integral_carry','actual_pid_weighted_i','total_integral_carry_component','actual_feedforward','actual_pid')});a['samples']=len(rr);means.append(a)
    phases={str(p):fit([x for x in steady if x['phase_ns']==p],'output_without_integral_carry') for p in sorted(set(x['phase_ns'] for x in steady))}
    reports[flag]=dict(batches=batches,quasistatic_means=means,steady_window_ns=[400000000,1200000000],
        total_command_with_initial_I_zero_fit=fit(means,'output'),effort_carry_integral_removed_fit=fit(means,'output_without_integral_carry'),phasewise_integral_removed_fit=phases,
        actual_command_abs_peak=max(abs(x['output']) for x in allrows),source_branch='measured state/old command effort' if flag=='false' else 'last commanded reference state/effort')
    data[flag]=allrows
    checks[flag+'_35_cases_all5_phases_200samples']=len(means)==35 and all(m['samples']==200 for m in means) and len(phases)==5
    checks[flag+'_full_rank_steady_linear_fit']=reports[flag]['effort_carry_integral_removed_fit']['rank']==3 and reports[flag]['effort_carry_integral_removed_fit']['max_abs_residual']<1e-10
checks['owned77_clean']=all(x['exit_code']==0 and not x['timeout'] and x['owned_group_clean'] for x in execution['runs'])
checks['production269_unchanged']=execution['source_269_match_before'] and execution['source_269_match_after'] and not execution['source_changes']
checks['all6_same_installed_DSO']=all(x==next(iter(loaded.values())) for x in loaded.values())
checks={k:bool(v) for k,v in checks.items()}
result=dict(passed=all(checks.values()),checks=checks,scope=__doc__,reports=reports,execution=execution,
    source_sha256=sha(attempt/'fixture.cpp'),binary_sha256=sha(attempt/'jtc_effective_response_native'),actual_loaded_dso=loaded,
    cadence=dict(producer_period_ns=5000000,update_period_ns=4000000,tie_order='producer callback confirmation before update',clock='original integer controlled event stamp',wall_cadence_not_asserted=True),
    limitations=['Prescribed constant p and independently preset nonzero v are interface snapshots, not a physically consistent Go2 trajectory or no-slip model.',
      'These effective P/D values describe repeated fixed position-only targets at the declared dual cadence and horizon; changed producer cadence/trajectory shape alters them.',
      'Actual weighted I and the observed linear carry of prior total effort are separated without writing controller private state; all batches start with actual interface command zero and normal activation.',
      'True FF-zero is limited to this zero-initial-command, position-only fixed-target fixture. It does not prove all existing physical activation/stop cases have zero FF.',
      'No gain or flag adoption. No ResourceManager clamp is exercised; commanded interface effort is not independently measured applied torque.'],
    input_sha256={p.name:sha(p) for p in sorted(attempt.glob('*.jsonl'))},analyzer_sha256=sha(Path(__file__)))
(attempt/'result.json').write_text(json.dumps(result,indent=2)+'\n')
(attempt/'analysis_used.py').write_bytes(Path(__file__).read_bytes())
(HERE/'final_receipt.json').write_text(json.dumps(dict(passed=result['passed'],checks=checks,result=str(attempt/'result.json'),result_sha256=sha(attempt/'result.json'),source_sha256=result['source_sha256'],binary_sha256=result['binary_sha256']),indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],checks=checks,fit={f:reports[f]['effort_carry_integral_removed_fit'] for f in reports},result_sha256=sha(attempt/'result.json')),indent=2))
raise SystemExit(not result['passed'])
