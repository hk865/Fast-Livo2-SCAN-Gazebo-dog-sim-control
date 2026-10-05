#!/usr/bin/env python3
"""Compile the production IMU gate and replay recorded physical IMU samples.

This launches no ROS or simulation processes. Quaternions and truth are never
passed to the tested class. Recorded physical motion must delay initialization;
the known stationary recordings must supply an actual consecutive 300 samples.
"""
import json
import math
import subprocess
import tempfile
from pathlib import Path

SLAM = Path(__file__).resolve().parents[1]
DEMO = SLAM.parent


def main():
    reports = {}
    with tempfile.TemporaryDirectory(prefix='demo_imu_init_replay_') as folder:
        executable = Path(folder)/'stationary_init_test'
        include = SLAM/'ros2_ws/src/fast_livo2_core/include/fast_livo2_core/core'
        subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',f'-I{include}',
                        str(SLAM/'tests/test_stationary_imu_initialization.cpp'),'-o',str(executable)],check=True)
        subprocess.run([str(executable)],check=True)
        inputs = {}
        for name in ('turn_raw_relaxed','turn_filtered_relaxed_v2'):
            path = SLAM/'test_results'/name/'sensor_audit.jsonl'
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            inputs[name+'_startup'] = (path,[r for r in rows if r['source']=='imu' and r['stamp']<=16.])
            # Start during actual turning; initialization must wait for the later
            # measured settle interval, rather than average the moving samples.
            inputs[name+'_turn_recovery'] = (path,[r for r in rows if r['source']=='imu' and r['stamp']>=19.])
        path = DEMO/'runs/20260930_185651_b1d6cb/fastlivo_imu.txt'
        rows=[]
        for line in path.read_text().splitlines():
            v=list(map(float,line.split()))
            rows.append({'stamp':v[0]+10.8,'omega':v[1:4],'acc':v[4:7]})
        inputs['run10_post_recovery_stationary'] = (path,rows)
        for name,(path,rows) in inputs.items():
            stream=''.join(' '.join(map(str,[r['stamp'],*r['acc'],*r['omega']]))+'\n' for r in rows)
            result=subprocess.run([str(executable),'--replay'],input=stream,text=True,capture_output=True,check=True)
            report=json.loads(result.stdout); report['input']=str(path.relative_to(DEMO))
            # B ends with only 293 consecutive still observations, so the
            # correct result is to keep waiting. No seven observations are
            # fabricated or borrowed from before the last actual motion.
            expected_ready = name != 'turn_filtered_relaxed_v2_turn_recovery'
            report['expected_ready'] = expected_ready
            assert report['ready'] == expected_ready,name
            if expected_ready:
                assert report['count']>=300 and report['span_s']>=2.9,name
            else:
                assert report['count']<300,name
            a=report['mean_acc'];report['estimated_gravity_tilt_deg']=math.degrees(math.atan2(math.hypot(a[0],a[1]),abs(a[2])))
            if name.endswith('_turn_recovery'):
                assert report['resets']>0 and report['first_stamp']>19.,name
            reports[name]=report
        # The successful original A/B input window remains eligible at launch
        # time. The test above also proves startup and motion rejection when the
        # same physical recordings are presented from their first IMU sample.
        for name in ('turn_raw_relaxed','turn_filtered_relaxed_v2'):
            path=SLAM/'test_results'/name/'sensor_audit.jsonl'
            rows=[json.loads(l) for l in path.read_text().splitlines()]
            rows=[r for r in rows if r['source']=='imu' and 10.<=r['stamp']<=16.]
            stream=''.join(' '.join(map(str,[r['stamp'],*r['acc'],*r['omega']]))+'\n' for r in rows)
            report=json.loads(subprocess.run([str(executable),'--replay'],input=stream,text=True,capture_output=True,check=True).stdout)
            assert report['ready'] and report['last_stamp']<16.,name
            reports[name+'_original_launch_window']=report
    result={'passed':True,'scope':'production C++ class contract and physical-recording replay; no new physics run',
            'limits':{'gyro_norm_rad_s':.03,'acceleration_norm_error_m_s2':1.2,'acceleration_axis_std_m_s2':.40,
                      'max_sample_gap_s':.03,'minimum_span_s':2.9,'consecutive_samples':300},'recordings':reports}
    output=SLAM/'tests/stationary_initialization_replay_result.json'
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
