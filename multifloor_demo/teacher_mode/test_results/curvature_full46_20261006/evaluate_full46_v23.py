#!/usr/bin/env python3
"""Version adapter for the frozen full46 reader; no runtime/control mutation.

All original checks remain, including the two explicit unverified full replay
checks. V20 changes only the source-module selection and effective spatial
heading reader. Extra checks bind SCAN selection, region deadlines and phase
proof order. No evidence is written into a run.
"""
import argparse
import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
TOP = HERE.parents[1]
V20 = TOP/'navigation/corridor_tracking_v23_curvature_full46'
FROZEN_VERSION_ADAPTER = TOP/'test_results/corridor_tracking_v20_continue_20261006/evaluate_full46_v20.py'
FROZEN_VERSION_ADAPTER_SHA = 'a4e529b882e31f2fa1d80bfdeff2c86e9243d1a09ae6ad5e97dba7998ccac133'
OLD = TOP/'test_results/pipeline_v19_20261006/evaluation/evaluate_mission46.py'
OLD_SHA = '9a16295984d60862727c92ab6df0a81e0254a8f04b7d77709cd4cc9c958498d2'
CONTROL_PINS = {
    'cascade_core.py':'3c1a3cb00993fe213b622b0df2bdaf511cb3fcbbd2728f0fc998a87c155f4e82',
    'controller.py':'2af343e23e60a3c9a2e9757ea1e60c8bdbb342b792f91bc5c5dcb315fb2a6a5d',
    'spatial_reference.py':'ba19a2f40b31d391810073361ed5c5821345ce0c5aef49ad34b8e542539a8cb7'}
UNIMPLEMENTED_FULL_REPLAYS = {'full_200Hz_native_actuator_trace_replay',
    'full_publication_PI_and_all_SCAN_geometry_replay'}

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as stream:
        for raw in iter(lambda:stream.read(1<<20),b''):h.update(raw)
    return h.hexdigest()

def angle_error(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))

def load_adapter():
    if sha(FROZEN_VERSION_ADAPTER)!=FROZEN_VERSION_ADAPTER_SHA:
        raise ValueError('Reviewed prior version adapter changed')
    if sha(OLD)!=OLD_SHA:
        raise ValueError('Frozen full46 evaluator changed; adapter review required')
    # The old evaluator imports by these names. Bind the reviewed V20 modules
    # first and verify their resolved locations, rather than changing old files.
    sys.path.insert(0,str(V20))
    for name in ('mission46_profile','mission46','mission46_runtime_evidence'):
        module=importlib.import_module(name)
        if Path(module.__file__).resolve()!=(V20/(name+'.py')).resolve():
            raise ValueError('Foreign cached reader module: '+name)
    spec=importlib.util.spec_from_file_location('frozen_full46_reader_for_v20',OLD)
    audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    audit.V19=V20
    sys.path.insert(0,str(V20));sys.path.insert(0,str(V20.parent))
    original_closure=audit.source_closure
    def source_closure(run,scope):
        out=original_closure(run,scope)
        reused=[V20/'mission46.py',TOP.parent/'simulation/obstacle_trigger.py',
            TOP.parent/'navigation/control_core.py',V20.parent/'guard_audit.py']
        for path in reused:
            if sha(path)!=sha(audit.snapshot(run,path.name)):
                raise ValueError('V20 reused reader dependency differs: '+str(path))
        out.update(version_adapter='V23 source selection; original closure gates retained',
            additional_current_modules_match_archives=[str(p)for p in reused])
        return out
    audit.source_closure=source_closure
    original_phases=audit.phases
    def phases(run,profile,anchor,poses,laststatus):
        request_check,checks=original_phases(run,profile,anchor,poses,laststatus)
        for phase,check in checks.items():
            rid=check.get('request_id');status=laststatus.get(rid)
            if not status:continue
            deadlines=[]
            for raw,receipt in zip(status.get('goals_definitions',[]),status.get('region_arrivals',[])):
                goal=audit.parse_goal(raw);start=receipt.get('goal_activated_ros_clock_ns');end=receipt.get('stamp_ns')
                ok=(type(start)is int and type(end)is int and 0<=end-start<=round(goal.timeout_sim_s*1e9))
                deadlines.append(dict(goal_id=goal.goal_id,passed=ok,activation_ns=start,arrival_ns=end,
                    original_timeout_s=goal.timeout_sim_s))
            ok=len(deadlines)==audit.ROUTE_COUNTS[phase] and all(x['passed']for x in deadlines)
            check['independent_original_deadlines']=deadlines
            if not ok:check.update(status='failed',passed=False)
        return request_check,checks
    audit.phases=phases
    audit.heading_reference_consistency=lambda run,profile:heading_reference(audit,run,profile)
    original_rgb=audit.rgb
    def rgb(run):
        out=original_rgb(run)
        receipt,proof=audit.bound_proof(run,'mission46_rgb_save_receipt.json',
            'teacher_mission46_rgb_save/v1',audit.evidence.MAP_CHECKS)
        meta=proof['metadata']
        if (meta.get('frame_id')!='camera_init' or meta.get('save',{}).get('complete')is not True or
                not all(meta.get('counts',{}).get(k,0)>0 for k in ('camera','lidar','imu','odom','colored_cloud'))):
            raise ValueError('Original RGB frame/completeness/per-sensor-count gates differ')
        out['original_frame_complete_sensor_counts_verified']=True
        return out
    audit.rgb=rgb
    return audit

def heading_reference(audit,run,profile):
    for name,pin in CONTROL_PINS.items():audit.snapshot(run,name,pin)
    errors=[];turns=other=0;maximum=0.;modes={}
    for row in audit.rows(run/'navigation_pid_history.jsonl'):
        c=row.get('cascade')or{};gate=row.get('heading_gate_reference')or{}
        if c.get('controller_updated')is not True:continue
        mode=c.get('mode')
        if mode not in ('drive','turn','capture','active_hold'):continue
        modes[mode]=modes.get(mode,0)+1
        try:
            spatial=c['spatial_reference'];tangent=spatial['tangent_xy'];normal=spatial['normal_xy']
            if (len(tangent)!=2 or len(normal)!=2 or
                    not all(math.isfinite(float(v))for v in tangent+normal) or
                    abs(math.hypot(*tangent)-1)>1e-9 or
                    max(abs(normal[0]+tangent[1]),abs(normal[1]-tangent[0]))>1e-9 or
                    c['control_horizontal_tangent']!=tangent or c['control_horizontal_normal']!=normal or
                    c['control_error_cross_m']!=spatial['cross_ref_m'] or
                    spatial['path_id']!=c['path_id']or spatial['path_sha256']!=c['path_sha256']):
                raise ValueError('Spatial tangent/normal/cross/path binding differs')
            ungated=math.atan2(tangent[1],tangent[0])
            if c.get('endpoint_is_fixed_goal')and c['remaining_horizontal_arc_m']<=.15:
                ungated=c['fixed_goal']['heading_rad']
                if spatial.get('heading_source')!='fixed_goal_tail':
                    raise ValueError('Fixed goal tail provenance differs')
            if not math.isfinite(spatial['heading_rad'])or angle_error(spatial['heading_rad'],ungated)>1e-9:
                raise ValueError('Spatial heading does not match its tangent/fixed tail')
            if mode in ('capture','active_hold'):
                expected=c['fixed_goal']['heading_rad']
            elif mode=='turn'and gate.get('phase')=='align':
                turns+=1;locked=gate.get('locked_heading')
                if type(locked)not in(int,float)or not math.isfinite(locked):
                    raise ValueError('Missing actual locked heading')
                expected=locked
                if (c.get('reference_yaw_source')!='external_heading_gate_locked_reference' or
                        c.get('external_turn_heading_rad')!=locked or
                        angle_error(c.get('ungated_reference_yaw_rad',float('nan')),ungated)>1e-9 or
                        not math.isfinite(c.get('ungated_reference_yaw_rad',float('nan')))):
                    raise ValueError('Actual lock/spatial provenance differs')
            else:
                expected=ungated;other+=1
            actual=c['reference_yaw_rad']
            if not math.isfinite(actual):raise ValueError('Nonfinite effective reference')
            diff=angle_error(actual,expected);maximum=max(maximum,diff)
            if diff>1e-9:raise ValueError('Effective yaw reference differs')
        except (KeyError,TypeError,ValueError,IndexError)as error:
            errors.append([row.get('sequence'),str(error)])
    return audit.ck(None if not turns and not errors else not errors,
        V20_control_source_pins=CONTROL_PINS,actual_align_turn_updates=turns,
        other_reference_updates=other,mode_counts=modes,maximum_effective_reference_error_rad=maximum,
        errors=errors[:30],runtime_Controller_called=False,full_PI_math_not_asserted=True,
        full_spatial_chord_geometry_recomputed=False,
        scope='Recorded V20 spatial tangent/normal/cross/path/heading joins and locked-heading/tail behavior; not a complete trajectory replay')

def scan_scope(audit,run,profile):
    runtime=audit.read(run/'runtime_manifest.json');loaded=audit.read(run/'scan_loaded_binary.json')
    contract=runtime['scan_workspace_contract'];expected=runtime['scan_binary_contract']
    if (profile.get('scan_workspace_selector')!='protected_baseline' or
            profile.get('corridor',{}).get('mode')!='shadow' or
            profile.get('corridor',{}).get('retain_valid_plan')is not False or
            contract['data'].get('candidate')!='protected_baseline' or
            loaded.get('verified')is not True or loaded.get('run')!=str(run) or
            loaded.get('actual_executable_path')!=expected['path'] or
            loaded.get('actual_executable_sha256')!=expected['sha256'] or
            runtime.get('scan_loaded_binary_receipt_sha256')!=sha(run/'scan_loaded_binary.json') or
            runtime.get('scan_loaded_binary_verified')is not True):
        raise ValueError('Expected protected baseline SCAN/shadow binding differs')
    certificates=run/'corridor_certificates.jsonl'
    count=0
    if certificates.is_file():
        for row in audit.rows(certificates):
            count+=1
            if row.get('shadow_only')is not True or row.get('control_authority')is not False:
                raise ValueError('Unexpected corridor control authority')
    if count:raise ValueError('Unexpected corridor certificate producer in no-exporter full profile')
    return dict(scan_workspace_selector='protected_baseline',actual_executable_SHA=expected['sha256'],
        loaded_binary_receipt_SHA=sha(run/'scan_loaded_binary.json'),corridor_mode='shadow',
        exporter_enabled=False,corridor_certificate_count=count,corridor_control_authority=False,
        corridor_geometric_certification='UNAVAILABLE_NO_EXPORTER_BY_PROFILE',
        prefix9_comparison_limitation='Validated V22 prefix9 and this V23 full46 both use protected_baseline SCAN; actual paths and mission scope differ. No strict causal A/B claim.')

def phase_proof_order(audit,run):
    final=audit.read(run/'mission46_status.json')
    if final.get('completed_stages')!=['exploring','returning','saving_map','navigating']:
        raise ValueError('Original completed-stage order differs')
    requests=[audit.read(run/'mission46_requests'/name)for name in
        ('exploration_1.json','return_origin_2.json','navigation_f1_f3_3.json')]
    ids=[r['request_id']for r in requests]
    actions=list(audit.rows(run/'mission46_actions.jsonl'))
    if any(b['sim_ns']<a['sim_ns']for a,b in zip(actions,actions[1:])):
        raise ValueError('Mission action source clock moved backward')
    selected=[r for r in actions if r['action']['kind']in ('route','save_map','final_active_hold')]
    if [r['action']['kind']for r in selected]!=['route','route','save_map','route','final_active_hold']:
        raise ValueError('Original route/save/final-hold action sequence differs')
    if [r['action']['request_id']for r in selected if r['action']['kind']=='route']!=ids:
        raise ValueError('Original route action IDs differ')
    rgb=audit.read(run/'mission46_rgb_save_receipt.json')
    dynamic=audit.read(run/'mission46_dynamic_receipt.json')
    parking=audit.read(run/'mission46_final_parking_receipt.json')
    if rgb['request_id']!=ids[1]or dynamic['request_id']!=ids[2]or parking['request_id']!=ids[2]:
        raise ValueError('RGB/dynamic/parking receipt phase joins differ')
    first=selected[-1]['action']
    if first['request_id']!=ids[2]or first['start_stamp_ns']!=parking['start_stamp_ns']:
        raise ValueError('Parking interval moved from first declared final hold')
    if parking['end_stamp_ns']-parking['start_stamp_ns']!=5_000_000_000:
        raise ValueError('Final first-fixed-five-second window changed')
    return dict(original_completed_stages=final['completed_stages'],request_ids=ids,
        action_sequence=[r['action']['kind']for r in selected],first_final_hold_start_ns=parking['start_stamp_ns'],
        action_file_sha256=sha(run/'mission46_actions.jsonl'))

def evaluate(run):
    audit=load_adapter();run=Path(run).resolve()
    if not(run/'run_result.json').is_file()or not(run/'runtime_manifest.json').is_file():
        raise ValueError('Run writers have not completed')
    report=audit.evaluate(run)
    profile=audit.read(run/'navigation_profile.json')
    report['checks']['actual_baseline_SCAN_and_no_corridor_authority']=audit.wrapped(lambda:scan_scope(audit,run,profile))
    report['checks']['original_phase_receipt_and_first_hold_order']=audit.wrapped(lambda:phase_proof_order(audit,run))
    states=[v['status']for v in report['checks'].values()]
    report['status']='failed'if'failed'in states else'unverified'if'unverified'in states else'passed'
    report['passed']=report['status']=='passed'
    # Explicitly scoped result; the overall formal result above retains ALL
    # original checks and remains unverified while either full replay is absent.
    functional={k:v['passed']for k,v in report['checks'].items()if k not in UNIMPLEMENTED_FULL_REPLAYS}
    report['functional46_limited_pass']=bool(functional)and all(v is True for v in functional.values())
    report['functional46_limited_scope']='Original 18+14+14 regions, measured dwell/deadlines, initialization, terrain, RGB, dynamic ACK/guard recovery, first5s hold, 50Hz recorded native safety, source/binary and pipeline closure. Excludes complete 200Hz/PI/SCAN geometry replay.'
    report['version_adapter']=dict(file=str(Path(__file__).resolve()),sha256=sha(__file__),
        frozen_base_evaluator=str(OLD),frozen_base_evaluator_sha256=OLD_SHA,
        source_modules=str(V20),old_checks_removed=[],runtime_Controller_called=False)
    report['limitations'][0]='Full200Hz, all PI/publication/SCAN geometry replay remain unverified. V23 recorded effective spatial-reference joins replace the V19 closest-segment heading assumption.'
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--check-adapter',action='store_true')
    args=parser.parse_args()
    if args.check_adapter:
        load_adapter()
        for name,pin in CONTROL_PINS.items():
            if sha(V20/name)!=pin:raise ValueError('Frozen V20 source changed: '+name)
        print(json.dumps(dict(adapter_imported=True,no_run_read=True,old_evaluator_sha256=OLD_SHA,
            V20_source_pins=CONTROL_PINS,runtime_acceptance=False)));return
    if args.run is None or args.output is None:parser.error('--run and --output are required')
    if args.output.exists()or args.output.resolve().is_relative_to(args.run.resolve()):
        raise ValueError('Output must be new and outside immutable run')
    os.sched_setaffinity(0,{max(os.sched_getaffinity(0))});os.nice(15)
    out=evaluate(args.run)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x')as stream:json.dump(out,stream,indent=2,ensure_ascii=False,allow_nan=False);stream.write('\n')
    print(json.dumps(dict(output=str(args.output),status=out['status'],functional46_limited_pass=out['functional46_limited_pass'],
        checks={k:v['status']for k,v in out['checks'].items()}),ensure_ascii=False))

if __name__=='__main__':main()
