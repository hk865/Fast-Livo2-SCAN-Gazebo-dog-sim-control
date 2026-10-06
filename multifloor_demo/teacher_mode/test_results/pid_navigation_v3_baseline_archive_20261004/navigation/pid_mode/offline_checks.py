#!/usr/bin/env python3
"""Low-cost actual PID/scope/launch checks; starts no ROS node or simulation."""
import argparse,hashlib,importlib.util,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from pid_core import HeaderPID,slew
from route import build_request
from pid_scope import HERE,ROOT,profile_for,verify_scope


def run_checks(run,output):
    checks=[]
    def check(name,condition):
        checks.append({'name':name,'passed':bool(condition)})
        if not condition:raise AssertionError(name)
    def rejected(name,fn):
        try:fn()
        except (ValueError,RuntimeError,KeyError):check(name,True)
        else:check(name,False)
    profile=profile_for('flat_short');pid=HeaderPID(profile);zero=np.zeros(3)
    def update(t,pose=zero,velocity=zero,wz=0.,target=None,heading=0.,mode='drive',key=None,clock=None):
        target=np.array([1.,.01,0.])if target is None else np.asarray(target)
        return pid.update(t,np.asarray(pose),0.,np.asarray(velocity),wz,target,heading,mode,key or('route',0,mode),t if clock is None else clock)
    cmd,d=update(1_000_000_000);cmd,d=update(1_100_000_000)
    check('forward saturated but small lateral integral progresses',pid.integral[0]==0 and pid.integral[1]>0 and d['antiwindup_blocked_axes']==[True,False,False])
    check('body axes and planar norm bounded',np.linalg.norm(cmd[:2])<=.2+1e-12 and abs(cmd[1])<=.1 and abs(cmd[2])<=.2)
    before=pid.integral.copy();lastcmd=cmd.copy();cmd,d=update(1_100_000_000)
    check('duplicate source header does not integrate',np.array_equal(pid.integral,before)and np.array_equal(cmd,lastcmd)and not d['updated'])
    cmd,d=update(1_100_000_000,clock=1_400_000_000)
    check('exact300ms source age immediately zero',np.array_equal(cmd,zero)and np.array_equal(pid.integral,zero))
    rejected('backward source clock failclosed',lambda:update(1_000_000_000))
    cmd,d=update(1_600_000_000)
    check('source gap clears integral',np.array_equal(pid.integral,zero)and d['header_dt_s']==0)
    cmd,d=update(1_700_000_000,velocity=[.1,.02,0.],wz=.03)
    check('derivative is negative measured velocity only',d['D_world_xy_yaw'][0]<0 and d['D_world_xy_yaw'][2]<0)
    import copy
    twin=copy.deepcopy(pid);_,reference=twin.update(1_800_000_000,zero,0.,np.array([.1,.02,0]),.03,np.array([1.,.01,0]),0.,'drive',('route',0,'drive'),1_800_000_000)
    cmd,d=update(1_800_000_000,velocity=[.1,.02,0.],wz=.03,target=[.5,.01,0.])
    check('setpoint change has no derivative kick',np.allclose(reference['D_world_xy_yaw'],d['D_world_xy_yaw']))
    cmd,d=update(1_900_000_000,mode='turn',heading=1.)
    check('alignment turn cannot translate',np.array_equal(cmd[:2],zero[:2])and 0<cmd[2]<=.2)
    cmd,d=update(2_000_000_000,mode='hold')
    check('hold clears integrals and commands',np.array_equal(cmd,zero)and np.array_equal(pid.integral,zero))
    cmd,d=update(2_000_000_000,mode='drive')
    check('protected duplicate cannot restore motion',np.array_equal(cmd,zero))
    rejected('nonfinite measurement refuses',lambda:update(2_100_000_000,velocity=[float('nan'),0,0]))
    pid=HeaderPID(profile)
    for i in range(50):
        cmd,d=update(3_000_000_000+i*100_000_000,target=[.02,.01,0.],heading=.01)
    check('integrator bounded each axis',np.all(abs(pid.integral)<=np.array([.4,.4,.3])))
    cmd,d=update(8_000_000_000,key=('newroute',1,'drive'))
    check('new goal clears old integral',np.array_equal(pid.integral,zero))
    previous=np.array([.15,-.03,.07]);desired=np.array([.18,.09,.13]);actual=slew(previous,desired,.05,profile['pid']['slew_acceleration'])
    check('independent slew preserves lateral command',actual[1]>previous[1]and np.all(abs(actual-previous)<=np.array([.15,.15,.25])*.05+1e-12))
    check('zero translation stop overrides slew',np.array_equal(slew(previous,zero,.05,[.15,.15,.25])[:2],zero[:2]))
    anchor={'source':'/demo/slam/body_odom','ground_truth_navigation_used':False,'origin':[2,3,.3],'yaw':math.pi/2}
    route=build_request('test',anchor,profile,{})
    check('route anchor uses actual SLAM yaw',np.allclose(route['goals'][0]['center'],[2,4,.3])and route['goals'][1]['center']==anchor['origin'])
    rejected('truth route source rejected',lambda:build_request('test',{**anchor,'source':'/demo/ground_truth/odom'},profile,{}))
    # Imports construct message types and class code only: no Node or rclpy.init.
    sys.path.insert(0,str(HERE));spec=importlib.util.spec_from_file_location('pid_controller_checks',HERE/'controller.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);C=module.PIDNavigation
    publications=[];records=[];wall=time.monotonic();ns=10_000_000_000;yaw=.2
    rotation=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    fake=SimpleNamespace(run=run,profile=profile,pid=HeaderPID(profile),pid_velocity=np.zeros(3),gyro_body=[0.,0.,0.],gyro_wall=wall,
        gyro_stamp=10.,pid_pose_stamp=ns,pose_stamp=ns,rotation=rotation,pose=np.zeros(3),pose_quaternion=[0,0,math.sin(yaw/2),math.cos(yaw/2)],
        steering={'heading':.25,'direction':[1.,0.],'exhausted':False},heading_gate=SimpleNamespace(phase='drive',heading=.25),
        obstacle_hold=False,request_id='test',waypoint_index=0,waypoints=np.array([[1,.2,0]]),command=[.05,.02,.01],last_command_time=9.95,
        pid_records=0,pid_row=None,active_trajectory_id=7,trajectory_archive_reference='checked.npz',
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ns)),state='running',alignment_hold=False,
        cmd_pub=SimpleNamespace(publish=lambda m:publications.append([m.linear.x,m.linear.y,m.angular.z])),freeze_pub=SimpleNamespace(publish=lambda m:None),
        counts={'commands':0},evidence=SimpleNamespace(error=None,append=lambda p,row:records.append(dict(row))))
    v,w=C.select_pid_velocity(fake,np.array([.1,0]),.01,np.array([1,.2,0]),True)
    expected=rotation[:2,:2]@fake.pid_prepared_output[:2]
    check('native guard direction is after-slew output',np.allclose(fake.steering['direction'],expected)and np.allclose(v,fake.pid_prepared_output[:2]))
    fake.record_native_guard=lambda *a:setattr(fake,'pending_guard_row',{'compute_ros_clock_ns':ns+5_000_000})or(False,None,0)
    C.evaluate_motion_guard(fake,[],[],[],[],[])
    check('exact PID-to-guard sequence association',fake.pending_guard_row['pid_sequence']==fake.pid_row['sequence']and fake.pending_guard_row['pid_control_pose_stamp_ns']==ns)
    C.publish_command(fake,v,w)
    check('publish does not change guarded velocity',np.allclose(publications[-1],fake.pid_prepared_output)and records[-1]['command_after_slew']==publications[-1])
    rejected('post-guard changed output refused',lambda:C.publish_command(fake,np.array([.2,.1]),.2))
    C.publish_command(fake)
    check('actual zero publisher resets PID',publications[-1]==[0,0,0]and np.array_equal(fake.pid.integral,zero)and fake.pid.last_stamp==ns)
    fake.command=[.1,.03,.1];fake.gyro_wall=wall-1
    v,w=C.select_pid_velocity(fake,np.array([.1,0]),.01,np.array([1,.2,0]),True)
    check('missing gyro source immediate all-axis zero before slew',np.array_equal(np.r_[v,w],zero))
    receipt=verify_scope(run/'navigation_scope.json');check('new explicit Teacher scope accepted',receipt['controller_kind']=='teacher'and not receipt['navigation_is_verified'])
    rejected('legacy scope schema cannot authorize PID',lambda:verify_scope(run/'navigation_scope.json',json.dumps({'schema':'teacher_finite_flat_navigation_scope/v1'}).encode()))
    raw=json.loads((run/'navigation_scope.json').read_text());bad={**raw,'controller_kind':'champ'}
    rejected('Teacher receipt cannot authorize CHAMP',lambda:verify_scope(run/'navigation_scope.json',json.dumps(bad).encode()))
    check('run provider exact frozen bytes',(run/'terrain_target_manifest.json').read_bytes()==(ROOT/profile['terrain_target_manifest']).read_bytes())
    check('executed-source snapshots present',all(Path(row['snapshot']).is_file()and hashlib.sha256(Path(row['snapshot']).read_bytes()).hexdigest()==row['sha256']for row in json.loads((run/'navigation_pid_source_manifest.json').read_text())['sources'].values()))
    bridge_spec=importlib.util.spec_from_file_location('old_bridge_pid_consumer_checks',HERE.parent/'bridge.py');oldbridge=importlib.util.module_from_spec(bridge_spec);bridge_spec.loader.exec_module(oldbridge)
    check('unchanged consumer new-schema acceptance',oldbridge.check_acceptance(run/'navigation_scope.json')['experiment']==profile['experiment'])
    from launch import LaunchContext
    s=importlib.util.spec_from_file_location('pid_launch_checks',HERE/'stack.launch.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
    description=m.generate_launch_description();context=LaunchContext();context.launch_configurations['run_dir']=str(run)
    actions=description.entities[1].execute(context)
    check('launch description constructs without execution',len(actions)==11 and all(a is not None for a in actions))
    report={'schema':'pid_offline_checks/v1','status':'passed','actual_physics_started':False,'ros_nodes_started':False,
        'checks':checks,'check_count':len(checks),'run':str(run),'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in HERE.glob('*.py')}}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':'passed','checks':len(checks),'report':str(output)}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run_checks(a.run.resolve(),a.output.resolve())
