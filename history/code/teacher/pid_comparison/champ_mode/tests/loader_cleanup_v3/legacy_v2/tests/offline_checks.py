#!/usr/bin/env python3
"""No ROS initialization, simulator, live signals or training execution."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
import numpy as np

HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('champ_offline_reader',HERE/'command_file_reader.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
read,ast_sha=m.consumer();checks={}
with tempfile.TemporaryDirectory(prefix='champ_reader_offline_')as temporary:
    path=Path(temporary)/'command.json';now=time.monotonic();base={'schema_version':1,'source':'scan_slam','mode':'scan_slam','acceptance':{'sha256':'test'},'sequence':1,
        'monotonic_wall':now,'sim_time':10.,'command':[.3,.1,-.2],'stop_requested':False,'healthy':True}
    def evaluate(data,t=10.,sequence=None):
        path.write_text(json.dumps(data));return read(path,t,'test',sequence)
    cmd,expired,_=evaluate(base);checks['fresh_same_command']=not expired and np.array_equal(cmd,[.3,.1,-.2])
    cmd,expired,_=evaluate({**base,'monotonic_wall':now-.31});checks['wall_300ms_reject']=expired and not cmd.any()
    cmd,expired,_=evaluate(base,t=10.301);checks['sim_300ms_reject']=expired and not cmd.any()
    cmd,expired,_=evaluate({**base,'sim_time':10.051});checks['future_50ms_reject']=expired and not cmd.any()
    cmd,expired,_=evaluate({**base,'healthy':False});checks['unhealthy_zero']=expired and not cmd.any()
    cmd,expired,_=evaluate({**base,'stop_requested':True});checks['controlled_stop_is_velocity_zero']=not expired and not cmd.any()
    sequence={};evaluate(base,sequence=sequence);original_stamp=sequence['envelope_sim_time'];evaluate(base,t=10.32,sequence=sequence)
    checks['repeated_read_no_timestamp_refresh']=sequence['envelope_sim_time']==original_stamp and sequence['sim_age_s']>.3
    cmd,expired,_=evaluate({**base,'command':[.31,0.,0.]});checks['command_bounds_unchanged']=expired and not cmd.any()
    cmd,expired,_=evaluate({**base,'command':[float('nan'),0.,0.]});checks['nonfinite_zero']=expired and not cmd.any()
    cmd,expired,_=evaluate({**base,'sequence':True});checks['boolean_sequence_rejected']=expired and not cmd.any()
    sequence={};evaluate(base,sequence=sequence);cmd,expired,_=evaluate({**base,'command':[0.,0.,0.]},sequence=sequence)
    checks['same_sequence_changed_payload_rejected']=expired and not cmd.any()
    checks['file_stamp_bytes_untouched_by_reader']=path.read_text()==json.dumps({**base,'command':[0.,0.,0.]})
run=HERE/'tests/prepared_offline';contract=json.loads((run/'champ_contract.json').read_text())
checks['all_14_original_protected_hashes_unchanged']=all(m.sha(p)==h for p,h in contract['protected_before'].items())
a=ET.parse(run/'champ/input_teacher_world.sdf').getroot();b=ET.parse(run/'world.sdf').getroot()
for tree in (a,b):
    model=tree.find("world/model[@name='go2']")
    for p in list(model.findall('plugin')):
        if p.get('name')in ('teacher_sim::TeacherActuator','gz_ros2_control::GazeboSimROS2ControlPlugin','champ_compare::NativeObserver'):model.remove(p)
checks['all_scene_geometry_physics_sensors_pose_exact_XML']=ET.tostring(a)==ET.tostring(b)
tree=ET.parse(run/'world.sdf').getroot();plugins=tree.findall("world/model[@name='go2']/plugin")
checks['sole_ros2_control_no_teacher_plugin']=sum('GazeboSimROS2ControlPlugin'in p.get('name','')for p in plugins)==1 and all('TeacherActuator'not in p.get('name','')for p in plugins)
cpp=(HERE/'native/champ_native_observer.cpp').read_text()
checks['native_no_force_position_velocity_reset_api']=all(api not in cpp for api in ('.SetForce(','.ResetPosition(','.ResetVelocity(','.SetLinearVelocity(','.SetAngularVelocity(','.AddWorldWrench(','.SetVelocityLimits(','.SetEffortLimits('))
checks['native_state_phase_postupdate_zero_offset']='ISystemPostUpdate'in cpp and contract['state_time_offset_s']==0
checks['controller_PID_native_and_no_body_servo']=contract['pid']['p']==220.982919 and contract['body_servo']is False and contract['body_stabilizer']is False
launchspec=importlib.util.spec_from_file_location('champ_offline_launch',HERE/'baseline.launch.py');launchmod=importlib.util.module_from_spec(launchspec);launchspec.loader.exec_module(launchmod)
from launch import LaunchContext
from launch.actions import OpaqueFunction
context=LaunchContext();context.launch_configurations['run_dir']=str(run)
ld=launchmod.generate_launch_description();opaque=next(x for x in ld.entities if isinstance(x,OpaqueFunction));actions=opaque.execute(context)
checks['launch_constructs_without_ROS_Gazebo_start']=len(actions)==8
reader=(HERE/'command_file_reader.py').read_text()
checks['no_model_load_or_training_import']='import torch'not in reader and 'torch.load'not in reader
checks['native_qtarget_not_fabricated']='\\\"qtarget\\\":null'in cpp
report={'schema':1,'scope':'offline prepare/consumer/launch construction only; no actual CHAMP simulation pass','checks':checks,'passed':all(checks.values()),
    'passed_count':sum(checks.values()),'total':len(checks),'consumer_function_ast_sha256':ast_sha,
    'source_sha256':{str(p):m.sha(p)for p in [HERE/'prepare.py',HERE/'command_file_reader.py',HERE/'baseline.launch.py',HERE/'native/champ_native_observer.cpp',HERE/'native/CMakeLists.txt',Path(__file__)]},
    'native_so_sha256':m.sha(HERE/'native/build/libchamp_native_observer.so'),'ROS_started':False,'simulator_started':False,'training_started':False}
(HERE/'tests/offline_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));raise SystemExit(0 if report['passed']else 1)
