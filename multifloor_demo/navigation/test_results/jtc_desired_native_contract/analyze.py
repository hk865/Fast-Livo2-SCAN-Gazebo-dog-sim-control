"""Actual installed JTC acceptance and mechanism report, not a Go2 success."""
import json,hashlib,subprocess
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
attempt=Path((HERE/'latest_attempt.txt').read_text().strip())
execution=json.loads((attempt/'execution_receipt.json').read_text())
streams={flag:[json.loads(l) for l in (attempt/(flag+'.jsonl')).open()] for flag in ['false','true']}
samples={flag:[r for r in data if r['kind']=='sample'] for flag,data in streams.items()}
inputs={flag:[r for r in data if r['kind']=='actual_input'] for flag,data in streams.items()}
checks={};reports={}
def peak(rows,key):
    values=[v for x in rows for v in x[key]]
    return float(np.max(np.abs(values))) if values else None
for flag,data in streams.items():
    setting=flag=='true';config=data[0];service=data[1];active=samples[flag][1:]
    expected_pid_integral=np.zeros(12);pid_errors=[];error_errors=[];carry_errors=[]
    previous=np.asarray(samples[flag][0]['output_effort'])
    for row in active:
        p=np.asarray(row['error_position']);v=np.asarray(row['error_velocity'])
        expected_pid_integral=np.clip(expected_pid_integral+.004*p,-12.5,12.5)
        pid_errors.append(float(np.max(np.abs(np.asarray(row['output_effort'])-row['next_effort']-100*p-v-.2*expected_pid_integral))))
        error_errors.append(max(float(np.max(np.abs(p-(np.asarray(row['reference_position'])-row['hardware_position'])))),
            float(np.max(np.abs(v-(np.asarray(row['reference_velocity'])-row['hardware_velocity']))))))
        carry_errors.append(float(np.max(np.abs(np.asarray(row['actual_effort_from_command_interface'])-previous))))
        previous=np.asarray(row['output_effort'])
    checks[flag+'_actual_bool_service']=(service['flag_type']==1 and service['flag_bool'] is setting
        and config['actual_parameter'] is setting)
    checks[flag+'_closed_loop_pid_not_open_loop']=config['closed_loop_effort_pid'] is True and config['open_loop_control'] is False and service['open_loop_bool'] is False
    checks[flag+'_original_gains']=service['p']==100 and service['i']==.2 and service['d']==1
    checks[flag+'_actual_dds_269_distinct_callbacks']=len(inputs[flag])==269 and all(x['new_callback_observed'] is True for x in inputs[flag])
    checks[flag+'_original_position_only_header0_horizon']=all(x['header_ns']==0 and x['horizon_ns']==16666666 and len(x['target'])==12 for x in inputs[flag])
    checks[flag+'_actual_sync_trigger_4ms']=config['is_async'] is False and len(active)==293 and all(x['actual_returned_success'] is True and x['actual_returned_period_ns']==4000000 for x in active)
    checks[flag+'_real_measured_state_each_update']=all(x['actual_position']==x['hardware_position'] and x['actual_velocity']==x['hardware_velocity'] for x in active)
    checks[flag+'_finite_all_measured_reference_output']=all(np.isfinite(x[k]).all() for x in active for k in ['hardware_position','hardware_velocity','reference_position','reference_velocity','output_effort','next_effort'])
    checks[flag+'_pid_error_is_current_measured_error']=max(error_errors)<1e-12
    checks[flag+'_pid_plus_ff_reconstructed']=max(pid_errors)<1e-12
    checks[flag+'_state_effort_is_old_command_not_measured_force']=max(carry_errors)<1e-12
    stages={}
    for stage in dict.fromkeys(x['stage'] for x in samples[flag]):
        ss=[x for x in samples[flag] if x['stage']==stage]
        stages[stage]=dict(samples=len(ss),reference_velocity_abs_peak=peak(ss,'reference_velocity'),
            next_velocity_abs_peak=peak(ss,'next_velocity'),
            commanded_effort_abs_peak=peak(ss,'output_effort'),
            next_reference_effort_abs_peak=peak(ss,'next_effort'))
    reports[flag]=dict(configuration=config,actual_parameter_service=service,stages=stages,
        max_pid_plus_ff_residual=max(pid_errors),max_current_state_error_residual=max(error_errors),max_old_command_readback_residual=max(carry_errors))
checks['whole_producer_input_stream_identical']=inputs['false']==inputs['true']
fixed_stages=['activation','first_reference','constant_target_measured_drift','nominal_stop_return','idle_constant_nominal','rapid_restart_after_nominal']
shared={}
for stage in fixed_stages:
    a=[x for x in samples['false'] if x['stage']==stage];b=[x for x in samples['true'] if x['stage']==stage]
    shared[stage]=dict(samples=len(a),hardware_pv_identical=len(a)==len(b) and all(x['hardware_position']==y['hardware_position'] and x['hardware_velocity']==y['hardware_velocity'] for x,y in zip(a,b)))
checks['same_fixed_state_stop_return_and_restart']=all(x['hardware_pv_identical'] for x in shared.values())
checks['first_reference_after_activation_same']=samples['false'][1]==samples['true'][1]
checks['old_false_reanchors_each_repeat']=np.max(np.abs([x['error_position'] for x in samples['false'] if x['stage']=='constant_target_measured_drift']))<1e-12
checks['candidate_still_reacts_to_measured_error']=np.max(np.abs([x['error_position'] for x in samples['true'] if x['stage']=='constant_target_measured_drift']))>.001
checks['owned_exit_and_clean']=all(x['exit_code']==0 and x['owned_group_clean'] and not x['timeout'] for x in execution['runs'])
checks['production_269_unchanged']=execution['source_269_match_before'] and execution['source_269_match_after'] and not execution['source_changes']
loaded={}
for flag in streams:
    paths=set()
    for line in (attempt/(flag+'.jsonl.maps')).read_text().splitlines():
        path=line.split()[-1]
        if path.startswith('/') and any(x in path for x in ['libjoint_trajectory_controller.so','libcontroller_interface.so','libcontrol_toolbox.so']):paths.add(path)
    loaded[flag]={x:sha(Path(x)) for x in sorted(paths)}
checks['actual_installed_jtc_dso_loaded']=all('/opt/ros/jazzy/lib/libjoint_trajectory_controller.so' in loaded[k] for k in loaded)
checks['same_loaded_dependency_dso']=loaded['false']==loaded['true']
checks={k:bool(v) for k,v in checks.items()}
versions=subprocess.run(['dpkg-query','-W','ros-jazzy-joint-trajectory-controller','ros-jazzy-controller-interface','ros-jazzy-control-toolbox'],capture_output=True,text=True).stdout
result=dict(passed=all(checks.values()),checks=checks,scope=__doc__,attempt=str(attempt),execution=execution,
    reports=reports,same_prescribed_measured_input_stages=shared,actual_loaded_dso=loaded,installed_packages=versions,
    original_flag_default=False,candidate_flag=True,controlled_call_period_ns=4000000,
    input_sha256={n:sha(attempt/n) for n in ['false.jsonl','true.jsonl','false.jsonl.maps','true.jsonl.maps','fixture.cpp','parameters.yaml','jtc_desired_native_contract','execution_receipt.json']},
    analyzer_sha256=sha(Path(__file__)),
    mechanisms=['New reference false: actual measured position/velocity and old effort command form the interpolation before-point.',
        'New reference true: last_commanded_state/time form the interpolation before-point. Effort-only current-state PID remains enabled and reads measured state.',
        'Reference effort FF and PID correction both change; the flag is not only a position anchor change. Output is commanded interface effort, not independently measured applied torque.',
        'Each repeated identical ROS target waits for a distinct live RT shared_ptr while holding the previous pointer; 269 fresh callbacks each trial.'],
    limitations=['Controlled 12-joint memory interfaces and a declared toy effort-feedback plant are not Gazebo/Go2 physics.',
        'Moving-target toy feedback is genuinely output-dependent; measured streams therefore differ. Stop/idle/restart isolation uses identical prescribed measured inputs and identical producer q0/target stream in both trials.',
        'Quintic .3s stop return has the production StopReturn geometry, but does not execute native Adapter ACK/return handshake; physical test must prove that independently.',
        'Lower controlled reference/command peaks are not an acceptance of physical stability or causation of the missing first8 JTC193sim data.',
        'CM is not exercised by this direct installed synchronous trigger_update fixture; final physical CM time wiring was independently tested earlier.'])
(attempt/'result.json').write_text(json.dumps(result,indent=2)+'\n')
(HERE/'final_receipt.json').write_text(json.dumps(dict(passed=result['passed'],result=str(attempt/'result.json'),result_sha256=sha(attempt/'result.json'),checks=checks,source_sha256=sha(attempt/'fixture.cpp'),binary_sha256=sha(attempt/'jtc_desired_native_contract'),actual_loaded_dso=loaded),indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],checks=checks,result=str(attempt/'result.json')),indent=2))
raise SystemExit(not result['passed'])
