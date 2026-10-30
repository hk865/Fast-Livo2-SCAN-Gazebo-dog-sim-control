"""Offline source-clock and actual actor-input evidence for explicit LiDAR gap tests."""
from pathlib import Path
import argparse,json
import numpy as np

def records(path):
    with path.open()as stream:return[json.loads(line)for line in stream if line.strip()]
def score(run):
    setup=json.loads((run/'experiment_setup.json').read_text())
    sensor=json.loads((run/'lidar_filter_status.json').read_text())
    start,duration=setup.get('validation_lidar_gap_s')or sensor['validation_lidar_gap_s'];end=start+duration
    poses=records(run/'slam_poses.jsonl');history=records(run/'navigation_history.jsonl');acks=records(run/'actual_command_ack.jsonl')
    times=np.array([r['stamp_ns']for r in poses],dtype=np.int64)
    candidates=[i for i in range(1,len(times))if(start-.2)*1e9<=times[i-1]<end*1e9 and times[i]<=(end+1)*1e9]
    if not candidates:raise ValueError('no bracketing SLAM output around configured gap')
    i=max(candidates,key=lambda i:times[i]-times[i-1]);before=int(times[i-1]);after=int(times[i]);deadline=before+300000000
    stale=[r for r in history if r['clock_s']*1e9>=deadline and r['sim_s']*1e9<=before and r['clock_s']<end+1]
    first_zero=next((r for r in acks if deadline<=r['stamp_ns']<=after and r['command']==[0.,0.,0.]),None)
    held=[]if first_zero is None else[r for r in acks if first_zero['stamp_ns']<=r['stamp_ns']<after]
    resumed=next((r for r in acks if r['stamp_ns']>=after and any(r['command'])),None)
    nav_resumed=next((r for r in history if r['sim_s']*1e9>=after and r['clock_s']>=start),None)
    raw_before=np.asarray(poses[i-1]['raw_T_odom_body']);raw_after=np.asarray(poses[i]['raw_T_odom_body'])
    motion_rows=[r for r in history if start-1<=r['clock_s']<start]
    displacement_before_gap_m=float(np.linalg.norm(np.asarray(motion_rows[-1]['pose_world'])-motion_rows[0]['pose_world']))if len(motion_rows)>1 else 0.
    movement_before_gap=displacement_before_gap_m>.05 and any(start-1<=r['stamp_ns']/1e9<start and any(r['command'])for r in acks)
    result=dict(movement_before_gap=movement_before_gap,measured_SLAM_displacement_in_second_before_gap_m=displacement_before_gap_m,trigger_evidence=sensor.get('validation_gap_trigger'),scope='Actual sensor-source drop and actor command evidence; no GT is read, this is not arbitrary lost-pose or kidnapped recovery',configured_gap_s=[start,end],dropped_messages=sensor['validation_dropped'],first_dropped_ns=sensor['first_dropped_ns'],last_dropped_ns=sensor['last_dropped_ns'],last_output_source_before_gap_ns=before,first_output_source_after_gap_ns=after,source_gap_ms=(after-before)/1e6,original_freshness_threshold_ms=300,threshold_clock_ns=deadline,first_exact_actor_zero=first_zero,first_zero_after_threshold_ms=None if first_zero is None else(first_zero['stamp_ns']-deadline)/1e6,first_exact_zero_source_age_ms=None if first_zero is None else(first_zero['stamp_ns']-before)/1e6,stale_navigation_samples=len(stale),all_stale_navigation_samples_requested_zero=bool(stale)and all(r['command']==[0.,0.,0.]for r in stale),all_actor_samples_zero_until_new_source=bool(held)and all(r['command']==[0.,0.,0.]for r in held),actor_held_zero_samples=len(held),first_resumed_actor_input=resumed,nav_resume_clock_s=None if nav_resumed is None else nav_resumed['clock_s'],resume_after_sensor_window_ms=None if nav_resumed is None else(nav_resumed['clock_s']-end)*1000,raw_odometry_motion_across_gap_m=float(np.linalg.norm(raw_after[:3,3]-raw_before[:3,3])),sampling_note='Original navigation tick100Hz and actor50Hz quantize zero application. Actual delay and source age are reported explicitly; no lowered threshold or rewritten clock.')
    result['source_loss_stop_and_resume_observed']=bool(movement_before_gap and sensor['validation_dropped']and first_zero and stale and held and result['all_stale_navigation_samples_requested_zero']and result['all_actor_samples_zero_until_new_source']and resumed and nav_resumed)
    p=run/'lidar_gap_score.json';p.write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);args=parser.parse_args();print(json.dumps(score(args.run),indent=2))
