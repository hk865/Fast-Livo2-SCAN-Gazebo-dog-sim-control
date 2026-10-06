#!/usr/bin/env python3
"""Deterministic fixture boundary checks, no ROS nodes or simulation."""
import argparse,copy,json,math,sys,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from dynamic_obstacle import Trigger,Program,Tail,fresh
from prepare import patch_world,basis,sha


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rule=json.loads((HERE/'protocol.json').read_text());checks={}
    def check(n,c):
        checks[n]={'passed':bool(c)}
        if not c:raise AssertionError(n)
    def samples(trigger,mutate=None):
        result=[]
        for i in range(4):
            sim=10.+i*.1;wall=100.+i*.1
            pose={'stamp_ns':round(sim*1e9),'frame_id':'camera_init','child_frame_id':'demo_slam_body',
                'received_monotonic_wall':wall,'body_velocity':[.1,0,0],'position':[.12+i*.01,0,0]}
            telemetry={'world_sim_time':sim,'requested':[.2,0,0],
                'navigation_envelope':{'read_monotonic_wall':wall,'read_status':'accepted','healthy':True}}
            nav={'state':'running','waypoint_index':0,'monotonic_wall':wall}
            anchor={'origin':[0,0,0],'yaw':0.,'source':'/demo/slam/body_odom','ground_truth_navigation_used':False}
            if mutate:mutate(pose,telemetry,nav,anchor)
            result.append(trigger.observe(pose,telemetry,nav,anchor,sim,wall))
        return result
    check('requires_four_new_real_slam_samples_and_300ms',samples(Trigger(rule))==[False,False,False,True])
    for name,change in [
        ('truth_localization',lambda p,t,n,a:a.update(source='/ground_truth/odom')),
        ('truth_anchor_flag',lambda p,t,n,a:a.update(ground_truth_navigation_used=True)),
        ('unaccepted_teacher',lambda p,t,n,a:t['navigation_envelope'].update(read_status='valid_but_unhealthy_or_stale')),
        ('no_actual_slam_motion',lambda p,t,n,a:p.update(body_velocity=[0,0,0])),
        ('no_forward_command',lambda p,t,n,a:t.update(requested=[0,0,0])),
        ('stale_slam_wall',lambda p,t,n,a:p.update(received_monotonic_wall=p['received_monotonic_wall']-.31)),
        ('wrong_slam_frame',lambda p,t,n,a:p.update(frame_id='world')),
        ('return_leg',lambda p,t,n,a:n.update(waypoint_index=1)),
        ('repeated_pose_stamps',lambda p,t,n,a:p.update(stamp_ns=10_000_000_000)),
        ('late_near_goal_entry',lambda p,t,n,a:p.update(position=[.31,0,0]))]:
        tr=Trigger(rule);check('rejects_'+name,not any(samples(tr,change)))
        if name=='late_near_goal_entry':check('late_entry_latches_missed_window',tr.missed_window)
    check('exact_300ms_age_expires',not fresh(10.,100.,10.3,100.3))
    check('old_source_not_refreshed_by_current_timer',not fresh(10.,100.,10.31,100.31))
    def actual(sim,pos):return {'status':'actual_observed','model':'moving_obstacle','navigation_input':False,
        'stamp_ns':round(sim*1e9),'received_monotonic_wall':100.+sim,'position':list(pos)}
    pr=Program(rule);initial=rule['initial_position_world'];blocked=rule['blocked_position_world']
    check('no_motion_until_real_trigger',pr.step(1.,False,None,101.)==initial and pr.phase=='waiting')
    pr.step(2.,True,actual(2.,initial),102.);check('starts_only_after_actual_initial_model_confirmation',pr.phase=='entering')
    span=abs(blocked[1]-initial[1])/rule['motion_speed_mps']
    pr.step(2.+span,False,actual(2.+span,initial),102.+span)
    check('service_target_is_not_actual_pose_evidence',pr.phase=='entering'and pr.hold_start is None)
    pr.step(4.,False,actual(4.,blocked),104.)
    check('blocking_clock_starts_at_actual_model_confirmation',pr.phase=='blocking'and pr.hold_start==4.)
    pr.step(13.99,False,actual(13.99,blocked),113.99);check('preserves_full_ten_sim_second_block',pr.phase=='blocking')
    pr.step(14.,False,actual(14.,blocked),114.);check('withdraws_after_ten_actual_sim_seconds',pr.phase=='leaving')
    pr.step(16.,False,actual(16.,initial),116.);check('clear_requires_actual_withdrawn_model_pose',pr.phase=='clear'and pr.clear_start==16.)
    pr=Program(rule);pr.step(2.,True,None,102.);check('missing_actual_initial_pose_fails_closed',pr.phase=='failed'and pr.target==initial)
    pr=Program(rule);pr.step(2.,True,actual(2.,initial),102.);pr.step(4.,False,actual(4.,blocked),104.)
    pr.step(4.31,False,actual(4.,blocked),104.31);check('stale_actual_block_pose_fails_closed',pr.phase=='failed')
    pr=Program(rule);pr.step(2.,False,None,102.);pr.step(1.,False,None,103.);check('clock_backwards_latches_failure',pr.phase=='failed')
    pr=Program(rule);pr.step(2.,True,actual(2.,initial),102.);pr.step(6.,False,actual(6.,initial),106.)
    check('entry_ack_without_actual_motion_times_out',pr.phase=='failed')
    with tempfile.TemporaryDirectory()as d:
        f=Path(d)/'test.jsonl';f.write_text('{"v":1}\n{"v":');tail=Tail(f)
        check('partial_json_line_never_used',tail.poll()=={'v':1})
        with f.open('a')as stream:stream.write('2}\n')
        check('completed_partial_line_read_once',tail.poll()=={'v':2})
        f.write_text('');caught=False
        try:tail.poll()
        except RuntimeError:caught=True
        check('source_file_truncation_rejected',caught)
    raw='<sdf version="1.9"><world name="teacher_demo"><gravity>0 0 -9.81</gravity><model name="go2"><pose>6 -.7 .4 0 0 0</pose></model><model name="moving_obstacle"><static>true</static><pose>1 4 .6 0 0 0</pose><link><collision><geometry><box><size>.5 .5 1.2</size></box></geometry></collision></link></model></world></sdf>'
    updated=patch_world(raw,rule)
    check('asset_patch_keeps_robot_pose_and_gravity_exact', '<pose>6 -.7 .4 0 0 0</pose>'in updated and '<gravity>0 0 -9.81</gravity>'in updated)
    records,campaign=basis();check('uses_exact_three_actual_finite_passes',len(records)==3 and all(x['finite_flat_navigation']=='passed'for x in records))
    result={'schema':1,'status':'passed','checks':checks,'starts_ros':False,'starts_simulation':False,
        'scope':'Pure offline trigger/program/asset fault checks; dynamic navigation remains unverified',
        'source_hashes':{str(f):sha(f)for f in [HERE/'dynamic_obstacle.py',HERE/'prepare.py',HERE/'protocol.json',Path(__file__)]}}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':'passed','checks':len(checks),'output':str(a.output)}))


if __name__=='__main__':main()
