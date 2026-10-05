#!/usr/bin/env python3
"""Read-only actual clock/source throughput and unlogged zero evidence.

Diagnostic phase wall times are bracketed by recorded bridge ROS clocks.
Receive clocks are never substituted for publisher clocks in acceptance replay.
Only small kind10/12/13 binary records are sought; no full binary scan/hash.
"""
from __future__ import annotations
import argparse,bisect,csv,hashlib,json,struct
from collections import defaultdict
from pathlib import Path
import numpy as np

HEADER=struct.Struct('<7Q')
def rows(p):return [json.loads(l)for l in p.open()if l.strip()]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dist(v):
    a=np.asarray(v,float);a=a[np.isfinite(a)]
    return dict(count=len(a),min=float(a.min()),p50=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))if len(a)else dict(count=0)
def evaluate(run):
    run=run.resolve();commands=rows(run/'navigation_command_history.jsonl')
    clocks=sorted({d['monotonic_wall']:d['stamp']['ros_sim_time_ns']for d in commands}.items())
    walls=[p[0]for p in clocks];phase=defaultdict(list);selected=hashlib.sha256()
    callbacks=defaultdict(dict);phase_start={};phase_propagation={};local_timing=defaultdict(list)
    with (run/'fastlivo_diagnostics/records.bin').open('rb')as f,(run/'fastlivo_diagnostics/index.csv').open()as ix:
        for row in csv.DictReader(ix):
            kind=int(row['kind'])
            if kind not in (2,3,10,11,12,13):continue
            f.seek(int(row['offset']));head=f.read(56);h=HEADER.unpack(head);body=f.read(int(row['n_values'])*8)
            if h[0]!=kind or h[2]!=int(row['stamp_ns'])or h[-1]!=int(row['n_values']):raise ValueError('binary/index differs')
            a=np.frombuffer(body,dtype='<f8');selected.update(head);selected.update(body)
            if kind in(2,3):
                source=h[2]+round(float(a[2])*1e9)
                callbacks['LiDAR'if kind==2 else'RGB'][source]=dict(received_wall=float(a[0]),original_header_ns=h[2],offset_s=float(a[2]))
                continue
            if kind==11:
                phase_propagation[h[1]]=float(a[0])
                if h[1]in phase_start:local_timing['Process2_plus_gravityAlignment_wall_s'].append([h[2]*1e-9,float(a[0])-phase_start[h[1]]])
                continue
            if kind in(12,13)and not a[0]:continue
            label='LIO_begin'if kind==10 and h[3]==2 else'VIO_begin'if kind==10 else'LIO_solve_end'if kind==13 else'VIO_solve_end'
            wall=float(a[0]if kind==10 else a[1]);stamp=h[2];j=bisect.bisect_left(walls,wall)
            if kind==10:
                phase_start[h[1]]=wall
                if h[3]==2:
                    frame_begin=round(float(a[1])*1e9)
                    original=next((callbacks['LiDAR'][candidate]for candidate in range(frame_begin-5,frame_begin+6)if candidate in callbacks['LiDAR']),None)
                    if original is not None:local_timing['accepted_LiDAR_callback_entry_to_LIO_Process2_start_wall_s'].append([stamp*1e-9,wall-original['received_wall']])
            elif h[1]in phase_propagation:
                local_timing[('LIO'if kind==13 else'VIO')+'_post_Process2_to_solve_end_wall_s'].append([stamp*1e-9,wall-phase_propagation[h[1]]])
            bracket=clocks[max(0,j-1):min(len(clocks),j+1)]
            if len(bracket)!=2 or not(bracket[0][0]<=wall<=bracket[1][0]):
                low=high=width=None
            else:
                low=(bracket[0][1]-stamp)*1e-9;high=(bracket[1][1]-stamp)*1e-9;width=bracket[1][0]-bracket[0][0]
            phase[label].append(dict(source_s=stamp*1e-9,wall=wall,age_lower_s=low,age_upper_s=high,clock_bracket_wall_width_s=width))
    stages={}
    for label,items in phase.items():
        bins={}
        for start in range(0,int(max(d['source_s']for d in items))+1,10):
            ds=[d for d in items if start<=d['source_s']<start+10];w=[d['wall']for d in ds]
            bins[str(start)+'_'+str(start+10)]=dict(records=len(ds),wall_progress_hz=(len(w)-1)/(w[-1]-w[0])if len(w)>1 else None,
                source_age_lower_s=dist([d['age_lower_s']for d in ds if d['age_lower_s']is not None]),
                source_age_upper_s=dist([d['age_upper_s']for d in ds if d['age_upper_s']is not None]),
                bracket_wall_width_s=dist([d['clock_bracket_wall_width_s']for d in ds if d['clock_bracket_wall_width_s']is not None]),
                outside_recorded_execution_clock_count=sum(d['age_lower_s']is None for d in ds))
        stages[label]=dict(ten_second_source_header_bins=bins)
    pid=rows(run/'navigation_pid_history.jsonl');hp=run/'navigation_control_clock_hold.jsonl';holds=rows(hp)if hp.exists()else[]
    events=sorted([(x['compute_monotonic_wall'],'pid',x)for x in pid]+[(x['compute_monotonic_wall'],'hold',x)for x in holds])
    receives={d['command_received_monotonic_wall']:d for d in commands if d.get('command_received_monotonic_wall')is not None}
    zero_receives=sorted((w,d)for w,d in receives.items()if d['requested']==[0.,0.,0.])
    prev=np.zeros(3);pt=None;pw=None;max_error=0.;mismatches=[]
    for wall,kind,d in events:
        if kind=='hold':prev=np.zeros(3);pt=d['publish_ros_clock_ns'];pw=wall;continue
        desired=np.asarray(d['desired_body_command']);obs=np.asarray(d['prepared_after_slew_command'])
        if d['cascade']['mode']in('protect','recovering','pre_turn','settle','path_end_hold'):expected=np.zeros(3)
        elif pt is None:expected=obs
        else:
            dt=(d['compute_ros_clock_ns']-pt)*1e-9;steps=np.array([.6,.6,.8])*max(0,min(dt,.1))
            expected=prev+np.clip(desired-prev,-steps,steps)
            if np.linalg.norm(desired[:2])<1e-9:expected[:2]=0.
        error=float(np.max(abs(expected-obs)));max_error=max(max_error,error)
        if error>1e-8:
            zeros=[dict(receive_wall=w,receive_ros_clock_ns=x['command_received_ros_sim_time_ns'],original_bridge_sequence=x['sequence'])for w,x in zero_receives if pw is not None and pw<w<wall]
            mismatches.append(dict(pid_sequence=d['sequence'],compute_ros_clock_ns=d['compute_ros_clock_ns'],compute_wall=wall,
                previous_observed_pid_or_hold_command=prev.tolist(),previous_observed_pid_or_hold_publish_clock_ns=pt,
                observed_prepared=obs.tolist(),replay_from_PID_hold_only=expected.tolist(),absolute_error=error,
                actual_requested_zero_receive_evidence_between_previous_and_this_compute=zeros))
        prev=np.asarray(d['command_after_slew']);pt=d['publish_ros_clock_ns'];pw=wall
    source_poses=rows(run/'navigation_slam_poses.jsonl');gaps=[]
    for a,b in zip(source_poses,source_poses[1:]):
        if b['stamp_ns']-a['stamp_ns']>300_000_000:gaps.append(dict(previous_source_ns=a['stamp_ns'],next_source_ns=b['stamp_ns'],
            source_gap_s=(b['stamp_ns']-a['stamp_ns'])*1e-9,previous_receive_wall=a['received_monotonic_wall'],next_receive_wall=b['received_monotonic_wall']))
    timing_summary={}
    for name,points in local_timing.items():
        timing_summary[name]={str(start)+'_'+str(start+10):dist([v for t,v in points if start<=t<start+10])for start in range(0,int(max(t for t,v in points))+1,10)}
    return dict(schema='read_only_actual_pipeline_clock_and_zero_evidence/v1',run=str(run),acceptance_status_not_changed=True,
        selected_phase_record_sha256=selected.hexdigest(),phase_clock_age=stages,
        actual_recorded_wall_segments=timing_summary,
        actual_recorded_wall_segment_scope={
            'accepted_LiDAR_callback_entry_to_LIO_Process2_start_wall_s':'Exact original accepted callback header+recorded LiDAR offset joined to phase lidar_frame_beg_time (only documented double-seconds rounding tolerance5ns). Includes preprocessing, synchronization wait, queueing and preceding pipeline work. Does not isolate communication.',
            'Process2_plus_gravityAlignment_wall_s':'kind10 before Process2 to kind11 after Process2/gravityAlignment. Includes diagnostic wall work between boundaries; wall time, not CPU time.',
            'LIO_post_Process2_to_solve_end_wall_s':'kind11 to actual processedkind13 immediately after state estimation. Includes downsampling/transform/map initialization when applicable, state solve and surrounding diagnostics, excludes later UpdateVoxelMap/publish work.',
            'VIO_post_Process2_to_solve_end_wall_s':'kind11 to actual processedkind12 after processFrame, includes visual preprocessing/retrieval/optimization; excludes later colored cloud/image output.',
            'map_update_and_individual_publication_wall_seconds':'N/A: original logger has no complete begin/end boundaries. Remaining cycle time must not be relabelled communication.',
            'source_acquisition_to_callback':'N/A for exact transport-only wall split. Original source sim header can be bracketed by actual /clock observations; includes simulation rendering/bridge/ROS queue and callback costs unless separately instrumented.'},
        clock_age_scope='Phase monotonic wall bracketed by original bridge ROS /clock observations. Ages are intervals; missing/post-cleanup clocks are N/A. Solve end excludes map updates and later image/cloud/path publications.',
        actual_accepted_navigation_source_gaps_over_300ms=gaps,
        slew_diagnostic=dict(PID_hold_only_mismatch_count=len(mismatches),maximum_absolute_error=max_error,
            rows_with_actual_received_zero_evidence=sum(bool(d['actual_requested_zero_receive_evidence_between_previous_and_this_compute'])for d in mismatches),rows=mismatches,
            missing_evidence='An observed topic receive proves a requested zero occurred, but receive ROS clock is not the publisher ROS clock. Receive evidence cannot be used to fake exact publication slew replay. Old formal receipt remains unchanged.'),
        input_sha256={n:sha(run/n)for n in ['navigation_command_history.jsonl','navigation_pid_history.jsonl','navigation_slam_poses.jsonl','fastlivo_diagnostics/index.csv']},reader_sha256=sha(__file__))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    d=evaluate(a.run);a.output.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(a.output),'mismatch_count':d['slew_diagnostic']['PID_hold_only_mismatch_count'],
        'rows_with_received_zero':d['slew_diagnostic']['rows_with_actual_received_zero_evidence']},indent=2))
if __name__=='__main__':main()
