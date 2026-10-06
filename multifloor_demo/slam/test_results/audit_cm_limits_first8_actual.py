#!/usr/bin/env python3
"""Offline CM-limit first8 audit. Never changes an original result or starts ROS."""
from pathlib import Path
import argparse, importlib.util, json, hashlib
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('actual_helpers', HERE/'analyze_feedback_first4_ab_independent.py')
H = importlib.util.module_from_spec(spec); spec.loader.exec_module(H)

def audit(name, defer_cdr=False):
    run = H.ROOT/'simulation/test_results'/name
    result = H.load(run/'first_eight_result.json')
    spec = importlib.util.spec_from_file_location('actual_eight', run/'staging/eight_region_contract.py')
    C = importlib.util.module_from_spec(spec); spec.loader.exec_module(C)
    goals = tuple(H.parse_goal(g) for g in result['goals_definitions'])
    evaluation = C.evaluate_regions(goals, result['request_id'], result['statuses'], result['poses'], result['truth'], result['origin_stamp_ns'], result['terminal_stamp_ns'])
    start, end = result['origin_stamp_ns'], result['terminal_stamp_ns']
    transform = evaluation['initial_fixed_SE3_evaluation_only']
    R = np.asarray(transform['rotation_world_from_slam']); T = np.asarray(transform['translation'])
    attitude = []
    for pose in result['poses']:
        if not start <= pose['stamp_ns'] <= end:
            continue
        actual = C.bounded_pair(pose['stamp_ns'], result['truth'])
        if actual is None:
            continue
        est = Rotation.from_matrix(R)*Rotation.from_quat(pose['q'])
        attitude.append({'stamp_ns': pose['stamp_ns'], 'error_rad': float((Rotation.from_matrix(actual[1]).inv()*est).magnitude())})
    raw = [x for x in result['raw_imu'] if start <= x['stamp_ns'] <= end]
    raw_gaps = [b['stamp_ns']-a['stamp_ns'] for a,b in zip(raw,raw[1:])]
    crossing = {}
    for level in (.30,.50):
        item = next((x for x in raw if H.metric(x['quaternion'])[0] >= level), None)
        if item is not None:
            actual = C.bounded_pair(item['stamp_ns'], result['truth'])
            metric, roll, pitch = H.metric(item['quaternion'])
            item = dict(item, raw_Euler_metric_rad=metric, raw_roll_pitch_rad=[roll,pitch], same_stamp_GT_Euler_metric_rad=None if actual is None else H.metric(Rotation.from_matrix(actual[1]).as_quat())[0])
        crossing[str(level)] = item
    previous = result.get('eighth_segment_previous_receipt_stamp_ns')
    completed = [x['receipt']['stamp_ns'] for x in evaluation['regions'] if x.get('passed') and x.get('receipt')]
    interval_start = previous if previous is not None else max(completed, default=start)
    print(json.dumps({'case':name, 'original_passed':result['passed'], 'ordered_region_passed':[x['passed'] for x in evaluation['regions']], 'precision':evaluation['precision'], 'coverage':evaluation['coverage'], 'first_raw030':None if crossing['0.3'] is None else crossing['0.3']['stamp_ns'], 'CDR_interval_ns':[interval_start,end]}),flush=True)
    cdr = None if defer_cdr else H.audit_cdr(run, interval_start, end)
    firsthold_cdr = None if defer_cdr or crossing['0.3'] is None else H.audit_cdr(run, max(start,crossing['0.3']['stamp_ns']-1_000_000_000), min(end,crossing['0.3']['stamp_ns']+1_000_000_000))
    native = H.audit_native(run, result)
    active_generic_bad = [x for x in native['generic_slice_bad'] if start <= round(float(x['camera'])*1e9) <= end]
    readback = H.load(run/'control_parameter_readback.json')
    manifest = H.load(run/'cm_manifest.json')
    cleanup = H.load(run/'process_cleanup.json')
    maps = H.load(run/'actual_gz_process.json')
    selected = [v for x in maps for v in x['libraries'] if Path(v['path']).name in ('libjoint_trajectory_controller.so','libcontroller_manager.so','libcontrol_toolbox.so','libcontroller_interface.so','libhardware_interface.so')]
    gate_values = {}
    for service,value in readback['services'].items():
        row = value['attempts'][-1]
        gate_values[service] = {n:v for n,v in zip(row['request_names'],row['actual_response']['values'])}
    source_checks = {}
    for filename,entry in manifest['files'].items():
        source_checks[filename] = H.sha(run/'staging'/filename) == entry['sha256']
    sources = ['first_eight_result.json','control_parameter_readback.json','process_cleanup.json','cm_manifest.json','actuator/observer_result.json','fastlivo_debug/imu.txt','stack.log']
    if not defer_cdr:
        sources.append('actuator/actuator_suffix.cdrlog')
    report = {
        'verdict':'Original physical result retained. Independent original-window verification and input coverage only; no original failure is repaired.',
        'case':name, 'original_passed':result['passed'], 'original_failure':result['failure'], 'original_missing_acceptance':result['missing_acceptance'],
        'independent_original_regions':evaluation,
        'active_raw_IMU':{'rows':len(raw), 'span_ns':[raw[0]['stamp_ns'],raw[-1]['stamp_ns']], 'unexpected_1ms_gaps':sorted(set(x for x in raw_gaps if x != 1_000_000)), 'max_original_Euler_rad':max(H.metric(x['quaternion'])[0] for x in raw), 'first_crossings':crossing},
        'single_initial_SE3_attitude':{'rmse_rad':float(np.sqrt(np.mean([x['error_rad']**2 for x in attitude]))), 'max_rad':max(x['error_rad'] for x in attitude)},
        'eighth_segment_entered':previous is not None, 'eighth_or_active_CDR':cdr, 'first_raw030_two_second_CDR':firsthold_cdr,
        'CDR_decoding_deferred':defer_cdr, 'final_active_segment_interval_ns':[interval_start,end],
        'native_FAST_IMU':native, 'active_generic_bad':active_generic_bad,
        'actual_startup_dual_service_readback':readback, 'actual_name_mapped_parameter_values':gate_values,
        'actual_loaded_control_libraries':selected, 'actual_archived_execution_source_SHA_checks':source_checks,
        'cleanup':cleanup, 'manifest':manifest, 'original_data_immutable_SHA256':{n:H.sha(run/n) for n in sources},
        'limits':[
            'Same initial SE3 used once; every declared original receipt window is evaluated at identical SLAM/GT timestamps. Missing coverage fails and a later window cannot rescue an original failed one.',
            'Raw Euler crossings are independent references; actual Bridge execution timestamps and its fixed-body transform remain separately recorded.',
            'JTC output.effort reads the effort command interface. With this installed set_limited_value path and CM enforcement true, it can already be command-limited. It is not an observation of the hardware write or applied torque; actual output limits are reported by case.',
            'Actual JointState effort zero or legacy NaN is unavailable physical-force evidence, never proof of zero applied torque.',
            'Observed state header continuity is not native ControllerInterface trigger-update period proof.',
            'All original missing IMU/header timestamps, sparse foot-contact gaps, middleware-lost callbacks and suffix limits remain explicit. No artificial samples are inserted.',
            'Fresh A/B share this private-profile/gate and recording setup; previous failed JTCfalse A has common diagnostic differences and is not an only-CMflag comparator.',
            'Native memory hardware-write limit proof does not substitute for actual closed-loop motion or the complete three-stage demo.'
        ]
    }
    dest = HERE/('oct2_'+name+('_prefix_independent.json' if defer_cdr else '_actual_independent.json'))
    dest.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'report':str(dest), 'SHA256':H.sha(dest), 'original_passed':result['passed'], 'independent_regions':evaluation['passed'], 'CDR_fixed_complete':None if cdr is None else cdr['all_fixed_period_sensor_JTC_headers_complete'], 'CDR_payload_errors':None if cdr is None else len(cdr['full_original_payload_errors']), 'native_missing':native['missing_native_1ms'], 'active_generic_bad':active_generic_bad}),flush=True)
    return report

if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('case'); parser.add_argument('--defer-cdr',action='store_true')
    args=parser.parse_args(); audit(args.case,args.defer_cdr)
