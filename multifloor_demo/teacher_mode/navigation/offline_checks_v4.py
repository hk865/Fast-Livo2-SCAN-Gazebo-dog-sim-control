#!/usr/bin/env python3
"""Offline navigation fault/replay checks; creates no ROS node or simulation."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import tempfile
import threading
import time

from bridge import CommandGate
from request import StartupDeadline
from runtime_io import LatestCommandWriter,BackgroundCheck,EvidenceWriter
from sensor_gate import ImageInfoPairs,WarmupRecovery
from teacher_transition import TeacherHeadingGate

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();profile=json.loads((HERE/'flat_relative_roundtrip.json').read_text());checks={}
    def check(name,condition,evidence=None):
        checks[name]={'passed':bool(condition),'evidence':evidence}
        if not condition:raise AssertionError(name)

    warm=WarmupRecovery(requirements={'body_odom':{'min_samples':3,'min_span_sim_s':2.},
                                    'cloud':{'min_samples':3,'min_span_sim_s':2.}})
    for stamp in (0,1_000_000_000,2_000_000_000):
        for name in ('body_odom','cloud'):warm.record(name,stamp)
        warm.observe(False,stamp,stamp)  # Every intermittent gap used to reset initialization.
    check('cumulative_distinct_source_history_survives_current_stale',warm.complete)
    check('initialized_but_current_stale_never_ready',not warm.observe(False,2_100_000_000,2_100_000_000))
    check('first_current_fresh_pose_not_enough',not warm.observe(True,2_200_000_000,2_200_000_000))
    check('same_pose_repeated_does_not_recover',not warm.observe(True,2_350_000_000,2_200_000_000))
    check('two_new_poses_release_with_100ms_source_span',warm.observe(True,2_400_000_000,2_400_000_000))
    warm.observe(False,2_500_000_000,2_500_000_000)
    check('current_stale_resets_recovery_but_not_initialization',warm.complete and warm.samples==0)

    pairs=ImageInfoPairs();pairs.observe('image',1_000_000_000,'vehicle',640,480,10.)
    pairs.observe('camera_info',1_000_000_000,'vehicle',640,480,10.)
    pairs.observe('camera_info',1_100_000_000,'vehicle',640,480,10.1)
    check('new_unpaired_info_keeps_still_fresh_actual_pair',pairs.latest_fresh(1_100_000_000,10.1)is not None)
    check('actual_pair_exact_300ms_expires',pairs.latest_fresh(1_300_000_000,10.3)is None)

    gate=CommandGate();gate.clock(25_175_000_000,10.)
    gate.graph={'slam':True,'command':True,'details':{}};gate.graph_wall=10.
    gate.pose(25_100_000_000,[0,0,0],[0,0,0,1],'camera_init','demo_slam_body',10.)
    gate.pose(25_175_000_000,[0,0,0],[0,0,0,1],'camera_init','demo_slam_body',10.)
    gate.command([.2,0,0],[0,0,0],10.)
    check('command_stamp_uses_same_latest_real_ros_clock',gate.snapshot(10.01)['healthy'] and gate.command_sim_ns==25_175_000_000)
    gate.clock(25_175_000_000,10.2)
    check('unchanged_clock_not_refreshed_by_timer',gate.clock_wall==10.)
    check('paused_clock_300ms_parks',not gate.snapshot(10.301)['healthy'])
    gate.clock(25_170_000_000,10.31)
    check('real_clock_backwards_latches_failure',bool(gate.failed))
    gate=CommandGate(profile);gate.failed=None;gate.clock(1_000_000_000,10.)
    gate.position=[1.5,0,0];gate.anchor={'origin':[0,0,0],'yaw':0}
    gate.snapshot(10.)
    check('slam_relative_fence_still_latches',bool(gate.failed))
    gate=CommandGate();gate.command([.2,0,0],[0,1,0],10.)
    check('unsupported_axes_still_latch',bool(gate.failed))

    deadline=StartupDeadline(profile,0.);deadline.observe(1_000_000_000,10.)
    check('half_speed_not_stopped_at_100wall_seconds',deadline.observe(51_000_000_000,110.)is None)
    check('startup_simulation_90s_deadline_fails',bool(deadline.observe(91_000_000_000,190.)))
    check('startup_failure_is_latched',bool(deadline.observe(92_000_000_000,191.)))
    check('no_clock_finite_600wall_cap',bool(StartupDeadline(profile,0.).observe(0,600.)))

    transition=TeacherHeadingGate(profile)
    check('timer_without_measurements_never_drives',not transition.update(100.,0.,0.,100.)[0])
    for t in (1.,1.1,1.2,1.3):transition.observe(t,t,[.001,0,0],.001,[0,0,.001],t,True)
    check('actual_three_tenths_second_standstill_aligned_start',transition.update(1.3,0.,0.,1.3)[0])
    transition.reset()
    check('communication_hold_does_not_erase_actual_stop',transition.stopped(1.3,1.3))
    allowed,wz=transition.update(1.3,0.,math.pi,1.3)
    check('return_turn_translation_zero_and_yaw_limited',not allowed and abs(wz)<=.12 and wz>0)
    transition.observe(1.4,1.4,[0,0,0],.1,[0,0,.1],1.4,False)
    check('turning_or_nonzero_command_erases_standstill',not transition.stopped(1.4,1.4))
    check('turn_completion_requires_new_measured_stop',not transition.update(1.4,math.pi,math.pi,1.4)[0])
    for t in (1.5,1.6,1.7,1.8):transition.observe(t,t,[0,0,0],0,[0,0,.001],t,True)
    check('after_turn_actual_stop_allows_drive',transition.update(1.8,math.pi,math.pi,1.8)[0])
    check('large_new_heading_stops_translation_immediately',not transition.update(1.8,math.pi,math.pi+.3,1.8)[0])
    transition.observe(1.9,1.9,[.04,0,0],0,[0,0,0],1.9,True)
    check('measured_moving_body_blocks_stop_confirmation',not transition.stopped(1.9,1.9))
    for t in (2.,2.1,2.2,2.3):transition.observe(t,t,[0,0,0],0,[0,0,0],t-.2,True)
    check('unpaired_gyro_cannot_prove_stop',not transition.stopped(2.3,2.3))

    with tempfile.TemporaryDirectory(prefix='teacher-nav-offline-')as directory:
        writes=[];entered=threading.Event();release=threading.Event()
        class SlowDisk:
            def write(self,value):
                if value['sequence']==1:entered.set();release.wait(2.)
                writes.append(value)
        transport=LatestCommandWriter(SlowDisk(),Path(directory)/'written.jsonl')
        transport.submit({'sequence':1,'monotonic_wall':time.monotonic(),'command':[.2,0,0]});check('slow_write_started',entered.wait(1.))
        begin=time.perf_counter()
        for i in range(2,102):transport.submit({'sequence':i,'monotonic_wall':time.monotonic(),'command':[.2,0,0]})
        transport.submit({'sequence':102,'monotonic_wall':time.monotonic(),'command':[0,0,0]})
        duration=time.perf_counter()-begin;release.set();transport.close()
        check('blocked_disk_never_blocks_sensor_executor',duration<.05,{'101_submissions_wall_s':duration})
        check('stop_supersedes_motion_backlog_and_no_fifo_replay',[d['sequence']for d in writes]==[1,102],
            {'actual_written_sequences':[d['sequence']for d in writes],'superseded':transport.superseded})
        check('delayed_envelope_original_stamp_not_rejuvenated',writes[0]['monotonic_wall']<writes[1]['monotonic_wall'])
        records=[json.loads(x)for x in(Path(directory)/'written.jsonl').read_text().splitlines()]
        check('written_history_exact_actual_delivery',len(records)==len(writes)and records[-1]['command']==[0,0,0])
        class BrokenDisk:
            def write(self,value):raise OSError('injected failed disk')
        broken=LatestCommandWriter(BrokenDisk());broken.submit({'sequence':1,'monotonic_wall':time.monotonic()})
        for _ in range(100):
            if broken.error:break
            time.sleep(.001)
        check('transport_failure_visible_to_gate',bool(broken.error))
        try:broken.close()
        except RuntimeError:pass
        checker=BackgroundCheck(lambda:time.sleep(.08)or{'valid':True},.1)
        for _ in range(100):
            if checker.result:break
            time.sleep(.002)
        check('slow_integrity_check_not_freshened_on_completion',checker.result['completed_wall']-checker.result['started_wall']>=.075)
        checker.close()
        archive=EvidenceWriter();archive.append(Path(directory)/'evidence.jsonl',{'stamp':123});archive.close()
        check('async_evidence_drain_preserves_records',json.loads((Path(directory)/'evidence.jsonl').read_text())=={'stamp':123})

    # Replay actual recorded SLAM body twist with independently received raw
    # IMU, strictly causal pairing. This is a counterfactual gate audit, never
    # a simulated route or a claim that the old robot moved.
    run=ROOT/'runs/20261004_080143_navigation_resource_idle_v2_baseline_r1_137e'
    gyro=sorted((json.loads(x)for x in(run/'sensor_shadow/sensor_samples.jsonl').open()
                 if '"source": "imu"'in x),key=lambda d:d['stamp_ns'])
    if not gyro:
        gyro=sorted((d for d in map(json.loads,(run/'sensor_shadow/sensor_samples.jsonl').open())if d['source']=='imu'),key=lambda d:d['stamp_ns'])
    stamps=[d['stamp_ns']for d in gyro];gate=TeacherHeadingGate(profile);qualified=0;total=0
    for pose in map(json.loads,(run/'navigation_slam_poses.jsonl').open()):
        t=pose['stamp_ns']/1e9
        if not 25<=t<=105:continue
        i=bisect_right(stamps,pose['stamp_ns'])-1
        if i<0:continue
        g=gyro[i];gate.observe(t,t,pose['body_velocity'],pose['body_angular_velocity'][2],g['gyro_body'],g['stamp_ns']/1e9,True)
        total+=1;qualified+=gate.stopped(t,t)
    check('actual_idle_v2_slam_and_imu_can_establish_standstill',qualified>total*.8,
        {'actual_run':run.name,'paired_pose_samples':total,'measured_stop_qualified_samples':qualified,
         'claim':'offline measured stop eligibility only; old navigation remains failed',
         'sources_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in(run/'navigation_slam_poses.jsonl',run/'sensor_shadow/sensor_samples.jsonl')}})
    receipt={'schema':1,'status':'passed','runtime_validation':'unverified','starts_ros_nodes':False,
        'starts_simulation':False,'checks':checks,'passed_checks':len(checks),'source_sha256':
        {str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in sorted(HERE.glob('*.py'))+[HERE/'flat_relative_roundtrip.json']},
        'scope':'Meaningful offline clock/freshness/initialization/measured-transition/fault/blocked-I/O checks; no navigation pass'}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':receipt['status'],'checks':len(checks),'output':str(args.output.resolve())}))


if __name__=='__main__':main()
