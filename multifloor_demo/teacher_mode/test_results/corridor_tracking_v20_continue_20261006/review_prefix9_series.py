#!/usr/bin/env python3
"""Run the frozen prefix9 evaluator once; collect plot data in the same reads.

The original evaluator, old results, and run are never changed. This wrapper
does not weaken the evaluator gates. Its generator observes the exact parsed
PID/pose rows that the evaluator already reads, without another log scan.
"""
import argparse
import gzip
import importlib.util
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'corridor_tracking_v20_20261006/evaluation/evaluate_prefix9.py'
FROZEN_EVALUATOR_SHA='812946d475141e2bd2c71fb925cadb75b6b1086027167fb53d8a318b25c3481b'
spec=importlib.util.spec_from_file_location('frozen_prefix9_evaluator',OLD)
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)

PID_COLUMNS=['clock_s','pose_stamp_s','goal_index','mode','heading_phase','trajectory_id',
    'inner_heading_error_rad','outer_heading_error_rad','geometric_cross_m','control_cross_m',
    'cmd_vx_mps','cmd_vy_mps','cmd_wz_radps','slam_origin_vx_mps','slam_origin_vy_mps',
    'slam_origin_vz_mps','measured_euler_yawrate_radps','body_gyro_z_radps',
    'remaining_arc_m','goal_xy_distance_m','pose_x_m','pose_y_m','pose_z_m',
    'goal_x_m','goal_y_m','goal_z_m','controller_updated','source_line']
POSE_COLUMNS=['pose_stamp_s','x_m','y_m','z_m','qx','qy','qz','qw',
    'origin_body_vx_mps','origin_body_vy_mps','origin_body_vz_mps','source_line']


def finite(value):
    return value if type(value) in (int,float) and math.isfinite(value) else None
def seconds(value):
    value=finite(value)
    return value/1e9 if value is not None else None
def vector(row,key,count):
    value=row.get(key)
    if not isinstance(value,(list,tuple)) or len(value)!=count:return [None]*count
    return [finite(v) for v in value]


class Collector:
    def __init__(self):self.runs={}
    def observe(self,path,row,ref):
        if path.name not in ('navigation_pid_history.jsonl','navigation_slam_poses.jsonl','navigation_status.jsonl'):return
        entry=self.runs.setdefault(str(path.parent.resolve()),dict(pid=[],poses=[],goals=None))
        if path.name=='navigation_status.jsonl':
            if row.get('goals_definitions') and entry['goals'] is None:
                entry['goals']=row['goals_definitions']
            return
        if path.name=='navigation_slam_poses.jsonl':
            entry['poses'].append([seconds(row.get('stamp_ns')),*vector(row,'position',3),
                *vector(row,'quaternion',4),*vector(row,'body_velocity',3),ref['line']])
            return
        cascade=row.get('cascade') or {}; gate=row.get('heading_gate_reference') or {}
        steer=gate.get('steering') or {}; feedback=row.get('feedback') or {}
        gyro=vector(cascade,'measured_body_omega',3)
        entry['pid'].append([seconds(row.get('control_stamp_ns')),seconds(feedback.get('stamp_ns')),
            row.get('waypoint_index'),cascade.get('mode'),gate.get('phase'),row.get('trajectory_id'),
            finite(cascade.get('error_yaw_rad')),finite(steer.get('heading_error')),
            finite(cascade.get('error_cross_m')),finite(cascade.get('control_error_cross_m')),
            *vector(row,'command_after_slew',3),*vector(feedback,'origin_velocity_body',3),
            finite(cascade.get('measured_Euler_yawrate_radps')),gyro[2],
            finite(cascade.get('remaining_horizontal_arc_m')),finite(cascade.get('goal_distance_xy_m')),
            *vector(row,'control_pose',3),*vector(row,'goal',3),cascade.get('controller_updated'),ref['line']])

    def ninth_progress(self,run,diagnostic,activation=None):
        entry=self.runs[str(Path(run).resolve())]
        exposed=diagnostic['PID_rows']>0
        base=dict(region9_exposed=exposed,comparable=exposed,
            original_region_arrived=diagnostic['outcome']=='original_region_arrived',
            drive_to_pre_turn_count=diagnostic['drive_to_pre_turn_count'],
            reset_with_new_path_count=diagnostic['new_path_reset_count'],
            observed_elapsed_until_arrival_or_failure_s=diagnostic['observed_elapsed_until_arrival_or_failure_s'],
            source='Only the same raw PID/pose/status read used by the independent evaluator; no interpolation.')
        if not exposed:return {**base,'reason':'No ninth-region PID exposure; zero resets cannot establish improvement.'}
        start=(activation['data']['control_stamp_ns']/1e9) if activation else diagnostic['activation_sim_s']
        end=diagnostic['terminal_sim_s']
        if end is None:
            end=max((r[0] for r in entry['pid'] if r[2]==8 and r[0] is not None),default=None)
        poses=[p for p in entry['poses'] if start is not None and end is not None and p[0] is not None and start<=p[0]<=end and all(v is not None for v in p[1:4])]
        if not poses:return {**base,'reason':'No original SLAM poses cover the observed ninth-region interval.'}
        import numpy as np
        initial=np.array(poses[0][1:4]); final=np.array(poses[-1][1:4]);delta=final-initial
        length=0.;bad_gaps=0
        for a,b in zip(poses,poses[1:]):
            if 0<b[0]-a[0]<=.200000001:length+=float(np.linalg.norm(np.array(b[1:4])-a[1:4]))
            else:bad_gaps+=1
        base.update(observed_interval_s=[start,end],observed_slam_samples=len(poses),
            actual_pose_interval_s=[poses[0][0],poses[-1][0]],
            observed_duration_s=end-start,path_length_slam_3d_m=length,excluded_pose_gap_count=bad_gaps,
            net_displacement_3d_m=float(np.linalg.norm(delta)),net_displacement_xy_m=float(np.linalg.norm(delta[:2])),
            first_position_xyz_m=initial.tolist(),last_position_xyz_m=final.tolist())
        base['observed_elapsed_until_arrival_or_failure_s']=end-start
        base['activation_source']=activation
        base['activation_clock_scope']='Original segment activation event, not an index-transition status retaining the previous goal clock.' if activation else 'Uncorrected status activation; no segment event supplied.'
        selected=[r for r in entry['pid'] if r[2]==8 and r[0] is not None and start<=r[0]<=end]
        reset=[];phases={}
        for previous,current in zip(selected,selected[1:]):
            dt=current[0]-previous[0]
            if 0<dt<=.200000001:phases[str(previous[4])]=phases.get(str(previous[4]),0.)+dt
            if previous[4]=='drive' and current[4]=='pre_turn':
                reset.append(dict(clock_s=current[0],previous_source_line=previous[-1],source_line=current[-1],
                    new_path=previous[5]!=current[5]))
        base.update(drive_to_pre_turn_count=len(reset),reset_with_new_path_count=sum(r['new_path'] for r in reset),
            reset_scope='Only original segment activation through measured arrival, or failure when no arrival exists.',
            reset_events=reset,phase_durations_before_arrival_or_failure_s=phases,
            all_goal_index8_rows_drive_to_pre_turn_count=diagnostic['drive_to_pre_turn_count'],
            all_goal_index8_rows_scope='Includes post-arrival final capture/parking in prefix9; not directly comparable to nonterminal V19 region8.')
        goals=entry['goals']
        if goals is not None and len(goals)>8:
            prior=np.array(goals[7]['center']); goal=np.array(goals[8]['center']);direction=goal-prior
            norm=float(np.linalg.norm(direction))
            base.update(goal_center_xyz_m=goal.tolist(),
                initial_goal_distance_3d_m=float(np.linalg.norm(goal-initial)),
                final_goal_distance_3d_m=float(np.linalg.norm(goal-final)),
                goal_distance_reduction_3d_m=float(np.linalg.norm(goal-initial)-np.linalg.norm(goal-final)),
                net_progress_along_original_region7_to8_m=float(delta@(direction/norm)) if norm>0 else None,
                direction_definition='Original transformed region centers 7 to 8; not reset-local path progress.')
        return base


def read_ninth_activation(run):
    source=audit.Sources(); records=[]
    for row,ref in source.rows(Path(run)/'navigation_segment_activation.jsonl'):
        if row.get('waypoint_index')==8:records.append(dict(data=row,source=ref))
    if source.errors or len(records)!=1:raise ValueError('Ninth region requires one exact archived segment activation event')
    return records[0],list(source.bindings.values())


def run_once(run,baseline,output_dir):
    if audit.file_sha(OLD)!=FROZEN_EVALUATOR_SHA:raise RuntimeError('Frozen prefix9 evaluator changed; new review required')
    run=Path(run).resolve();baseline=Path(baseline).resolve();output_dir=Path(output_dir).resolve()
    if output_dir.is_relative_to(run) or output_dir.is_relative_to(baseline):raise ValueError('Outputs must be outside runtime archives')
    report_path=output_dir/(run.name+'_PREFIX9_ACTUAL_EVALUATION.json')
    series_path=output_dir/(run.name+'_COMPACT_SERIES.json.gz')
    metrics_path=output_dir/(run.name+'_REGION9_COMPARISON.json')
    if any(p.exists() for p in (report_path,series_path,metrics_path)):raise ValueError('Refusing to overwrite an existing independent result')
    collector=Collector();original=audit.Sources
    class ObservedSources(original):
        def rows(self,path):
            for row,ref in super().rows(path):
                collector.observe(Path(path),row,ref)
                yield row,ref
    audit.Sources=ObservedSources
    try:report=audit.audit(run,baseline)
    finally:audit.Sources=original
    candidate_activation,candidate_activation_bindings=read_ninth_activation(run)
    baseline_activation,baseline_activation_bindings=read_ninth_activation(baseline)
    candidate=collector.ninth_progress(run,report['ninth_region_diagnostics'],candidate_activation)
    comparison=collector.ninth_progress(baseline,report['r3_comparison']['baseline'],baseline_activation)
    metrics=dict(schema='prefix9_same_pass_region9_comparison/v1',candidate=candidate,baseline=comparison,
        valid_region9_exposure_in_both=candidate['region9_exposed'] and comparison['region9_exposed'],
        terminal_policy_difference=report['r3_comparison']['terminal_policy_difference'],
        activation_source_bindings=candidate_activation_bindings+baseline_activation_bindings,
        full46_pass=False,whole_200hz_physical_acceptance=False)
    relevant=[b for b in report['source_bindings'] if Path(b['file']).name in
        ('navigation_pid_history.jsonl','navigation_slam_poses.jsonl','navigation_status.jsonl')]
    payload=dict(schema='prefix9_same_pass_compact_plot_series/v1',pid_columns=PID_COLUMNS,pose_columns=POSE_COLUMNS,
        runs=collector.runs,source_bindings=relevant,
        missing='null is unavailable; values are never filled forward or inferred from adjacent samples',
        body_velocity_source='Original SLAM body-origin twist in the actual feedback row',
        command_source='Actual published command_after_slew in PID journal',
        cross_scope='geometric_cross is original raw projection; control_cross uses finite-arc normal. V19 has no control_cross field.',
        clocks='PID uses controller ROS compute clock; poses use original SLAM header clock.',
        one_log_read_per_file=True,navigation_ground_truth_used=False)
    output_dir.mkdir(parents=True,exist_ok=True)
    with gzip.open(series_path,'wt',encoding='utf-8',compresslevel=3) as stream:
        json.dump(payload,stream,separators=(',',':'),ensure_ascii=False,allow_nan=False)
    wrapper_binding=dict(file=str(Path(__file__).resolve()),sha256=audit.file_sha(__file__))
    report['same_pass_plot_series']=dict(file=str(series_path),sha256=audit.file_sha(series_path),
        wrapper=wrapper_binding,one_log_read_per_file=True)
    metrics.update(wrapper=wrapper_binding,plot_series=report['same_pass_plot_series'],source_bindings=relevant)
    report_path.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    metrics['full_prefix_report']=dict(file=str(report_path),sha256=audit.file_sha(report_path))
    metrics_path.write_text(json.dumps(metrics,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    return dict(report=str(report_path),series=str(series_path),region9_comparison=str(metrics_path),
        prefix9_limited_pass=report['prefix9_limited_pass'],checks=report['checks'],candidate_region9=candidate)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--baseline',type=Path,default=audit.DEFAULT_BASELINE)
    parser.add_argument('--output-dir',type=Path,default=HERE)
    args=parser.parse_args()
    print(json.dumps(run_once(args.run,args.baseline,args.output_dir),indent=2,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':main()
