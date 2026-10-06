#!/usr/bin/env python3
"""Read-only sampled-sim-band comparison; no old-run writes."""
from pathlib import Path
import importlib.util,json,hashlib,numpy as np
BASE=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode')
OUT=Path(__file__).resolve().parent
reader=BASE/'test_results/lidar_density_rate_20261005/evaluation/performance_proc_v12.py'
s=importlib.util.spec_from_file_location('old_proc_reader',reader);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
runs={'V12':Path('/var/tmp/go2_teacher_simulation_20261005/20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d'),'V18':Path('/var/tmp/go2_teacher_parallel_20261005/20261005_232930_closed_loop_cascade_clock_hold_combined_v18_prefix_r1_ac02')}
def stats(a):
 a=np.asarray(a,dtype=np.float64)
 return {'count':len(a),'mean':float(a.mean()),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(a.max())}if len(a)else {'count':0}
results={}
for tag,run in runs.items():
 common=m.read(run/'summary_closed_loop_cascade_independent.json');cutoff=common['checks']['execution_phase_status_and_cleanup_boundary']['last_actual_Actor_read_wall']
 samples=[r for r in m.rows(run/'external_owned_cpu_profile.jsonl')if r['monotonic_wall']<=cutoff and m.state_time(r)is not None and 30<=m.state_time(r)<=209.4]
 assert len(samples)>2
 proc=m.cpu_window(samples[0],samples[-1]);start,end=proc['same_physical_wall_window_s'];timing=m.summarize_boundaries(m.boundaries(run),start,end)
 feedback=[r for r in m.rows(run/'navigation_feedback_history.jsonl')if start<=r['received_wall_ns']/1e9<=end]
 unique={r['stamp_ns']:r for r in feedback};ordered=[unique[k]for k in sorted(unique)];source_dts=np.diff([r['stamp_ns']/1e9 for r in ordered]);wall_dts=np.diff([r['received_wall_ns']/1e9 for r in ordered]);
 stages=timing['boundaries'];elapsed_complete=timing['total_window_wall_seconds'];simspan=proc['sampled_sim_time_bounds_s'][1]-proc['sampled_sim_time_bounds_s'][0]
 rates={x['label']:{'completed_calls':x['completed_calls'],'per_complete_boundary_wall_second':x['completed_calls']/elapsed_complete,'per_sampled_sim_second_approx':x['completed_calls']/simspan}for x in stages if x['boundary']in(9,10,11,14,15)}
 fast=[x for x in proc['owned_processes']if x['comm']=='fastlivo_mappin'];assert len(fast)==1
 f=fast[0];main=[x for x in f['per_thread_CPU']if x['is_process_main_thread']];assert len(main)==1
 results[tag]={'run':str(run),'observed_sample_sim_s':proc['sampled_sim_time_bounds_s'],'proc_same_wall':proc,'same_wall_boundary_timing':timing,'FASTLIVO':{'process_average_CPU_cores':f['average_CPU_cores'],'main_average_CPU_cores':main[0]['average_CPU_cores'],'other_thread_CPU_cores_sum':sum(x['average_CPU_cores']for x in f['per_thread_CPU']if not x['is_process_main_thread']),'main_nonvoluntary_context_switches':main[0]['nonvoluntary_context_switches'],'main_nonvoluntary_per_wall_s':main[0]['nonvoluntary_context_switches']/proc['wall_seconds'],'thread_role_limit':f['thread_role_limit']},'completion_rates':rates,'actual_SLAM_feedback':{'original_unique_headers':len(unique),'source_header_dt_s':stats(source_dts),'receiver_wall_dt_s':stats(wall_dts),'source_Hz_span':(len(unique)-1)/sum(source_dts),'receiver_wall_Hz_span':(len(unique)-1)/sum(wall_dts),'age_at_receipt_sim_s':stats([(r['callback_ros_clock_ns']-r['stamp_ns'])/1e9 for r in ordered]),'position_min_xyz':np.array([r['position_world_xyz']for r in ordered]).min(axis=0).tolist(),'position_max_xyz':np.array([r['position_world_xyz']for r in ordered]).max(axis=0).tolist()},'timing_source_bindings':m.read(OUT/(tag+'_prefix_performance.json'))['verified_timing_source_bindings']}
a={x['boundary']:x for x in results['V12']['same_wall_boundary_timing']['boundaries']};b={x['boundary']:x for x in results['V18']['same_wall_boundary_timing']['boundaries']};deltas=[]
for k,x in a.items():
 y=b[k]
 deltas.append({'boundary':k,'label':x['label'],'V12_mean_wall_ms':x['mean_wall_ms'],'V18_mean_wall_ms':y['mean_wall_ms'],'V12_mean_caller_CPU_ms':x['mean_caller_CPU_ms'],'V18_mean_caller_CPU_ms':y['mean_caller_CPU_ms'],'mean_wall_change_pct':(y['mean_wall_ms']/x['mean_wall_ms']-1)*100 if x['mean_wall_ms']and y['mean_wall_ms']is not None else None,'mean_caller_CPU_change_pct':(y['mean_caller_CPU_ms']/x['mean_caller_CPU_ms']-1)*100 if x['mean_caller_CPU_ms']and y['mean_caller_CPU_ms']is not None else None,'nested_ids':x['nested_boundary_ids']})
report={'schema':'V12_V18_actual_prefix_common_sim_band/v1','requested_common_sim_band_s':[30,209.4],'reader_path':str(reader),'reader_sha256':m.READER_SHA,'runs':results,'stage_mean_comparison':deltas,'causal_limits':['Separate physical realizations have different actual robot pose and constraints/map history; no same-input historical replay. Stage differences cannot identify a single-kernel cause.','kind300 nested scopes overlap; residual query is inside StateEstimation, inside handleLIO, so costs cannot be summed.','CLOCK_THREAD CPU is measured calling-thread usage; CLOCK_PROCESS CPU includes concurrent workers/ROS/logger and is not exclusive stage cost.','Publication covers conversion/enqueue before return, not subsequent DDS/network transport. Wall minus caller CPU includes descheduling, worker overlap and waiting, not a direct communication-time measurement.','V18 initial /proc sample preceded SLAM creation; old whole-window process intersection only included runner. Comparison instead uses valid actual sim>=30 sample bounds where FASTLIVO exists at both ends.','Sampled begin/end sim clocks and complete kind300 windows differ slightly between runs; rates labelled approximate per sampled sim second.'], 'physical_navigation_acceptance':'Owned by independent evaluator, not duplicated here.'}
(OUT/'performance.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'CPU':{k:v['FASTLIVO']for k,v in results.items()},'feedback':{k:v['actual_SLAM_feedback']for k,v in results.items()},'stages':[x for x in deltas if x['boundary']in(0,3,4,5,6,7,8,12,14,15)]},indent=2))
