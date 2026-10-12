#!/usr/bin/env python3
"""Convert a newly prepared Teacher run into an independent CHAMP baseline.

No ROS process, simulator, controller or training process is started here.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[1]
DEMO=TEACHER.parent
SIM=DEMO/'simulation'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(element):return ET.tostring(element,encoding='unicode')
def save_xml(path,element):ET.ElementTree(element).write(path,encoding='utf-8',xml_declaration=True)

def prepare(run,duration=120.):
    run=Path(run).resolve()
    if not 5<=duration<=600:raise ValueError('Duration must be 5..600 seconds')
    forbidden=['worker_ready','telemetry.jsonl','actuator.jsonl','champ_contract.json']
    if any((run/name).exists()for name in forbidden):raise RuntimeError('Refuse reuse or live/prepared CHAMP run')
    if not (run/'sensor_contract.json').is_file():raise RuntimeError('Run Teacher prepare with --sensors first')
    asset=json.loads((run/'asset_manifest.json').read_text())
    if asset.get('exclusive_writer')!='teacher_sim::TeacherActuator':raise ValueError('Unexpected input actuation contract')
    stage=run/'champ';stage.mkdir(exist_ok=False)
    originals=[SIM/'generated/go2_measured.urdf',SIM/'generated/go2_converted.sdf',SIM/'generated/three_floors.sdf',SIM/'config/gait.yaml',SIM/'config/ros_control.yaml',SIM/'joint_reference_adapter.py',SIM/'joint_stop_core.py',SIM/'execution_safety.py',SIM/'gait_selection.py',SIM/'gait_runtime.py',SIM/'controller_runtime.py',TEACHER/'runs/acceptance.json',DEMO/'camera_mode/simulation/generated/camera_rig.sdf',DEMO/'camera_mode/simulation/generated/three_floors_camera.sdf']
    protected={str(p):sha(p)for p in originals}
    (stage/'protected_before.json').write_text(json.dumps(protected,indent=2)+'\n')
    shutil.copy2(run/'world.sdf',stage/'input_teacher_world.sdf')
    shutil.copy2(run/'frames.urdf',stage/'input_teacher_frames.urdf')
    shutil.copy2(run/'asset_manifest.json',stage/'input_teacher_asset_manifest.json')
    world=ET.parse(run/'world.sdf').getroot();model=world.find("world/model[@name='go2']")
    if model is None:raise ValueError('Prepared world must contain go2')
    initial=canonical(world)
    plugins=model.findall('plugin');writers=[p for p in plugins if 'TeacherActuator'in p.get('name','')]
    if len(writers)!=1:raise ValueError('Input must contain exactly one Teacher actuator')
    for p in plugins:
        if p is writers[0]:continue
        name=(p.get('name','')+' '+p.get('filename','')).lower()
        if any(s in name for s in ('control','velocity','servo','actuator')):raise ValueError('Other writer plugin in input')
    model.remove(writers[0])
    native=HERE/'native/build/libchamp_native_observer.so'
    if not native.is_file():raise RuntimeError('Build isolated native observer before prepare')
    shutil.copy2(native,stage/native.name)
    sys.path.insert(0,str(SIM))
    from gait_selection import verified_deployment,WORKSPACE
    from controller_runtime import controller_manager_library
    deployment=verified_deployment();selected={}
    for key in ('executable','library'):
        source=WORKSPACE/deployment[key]['relative_path'];target=stage/('bin'if key=='executable'else'lib')/source.name
        target.parent.mkdir(exist_ok=True);shutil.copy2(source,target)
        selected[key]={'path':str(target),'sha256':sha(target)}
    shutil.copy2(SIM/'config/gait.yaml',stage/'gait.yaml');shutil.copy2(SIM/'config/ros_control.yaml',stage/'ros_control.yaml')
    share=DEMO.parent/'go2_sim_control/install/unitree_go2_sim/share/unitree_go2_sim/config'
    for part in ('joints','links'):shutil.copy2(share/part/(part+'.yaml'),stage/(part+'.yaml'))
    for name in ('joint_reference_adapter.py','joint_stop_core.py','execution_safety.py'):
        shutil.copy2(SIM/name,stage/name)
    # Geometry/frames remain the same as the Teacher run, including its added sensor frames.
    urdf=ET.parse(run/'frames.urdf').getroot();original_urdf=ET.parse(SIM/'generated/go2_measured.urdf').getroot()
    for c in original_urdf.findall('ros2_control'):urdf.append(copy.deepcopy(c))
    names=[]
    for joint in model.findall('joint'):
        limit=joint.find('axis/limit');u=urdf.find(f"joint[@name='{joint.get('name')}']/limit")
        if limit is None or u is None:continue
        names.append(joint.get('name'))
        for field in ('lower','upper','effort','velocity'):
            if limit.find(field)is not None:u.set(field,limit.findtext(field))
    if len(names)!=12:raise ValueError('Physical joint contract is not 12 joints')
    save_xml(stage/'champ.urdf',urdf)
    # The RSP used by the SLAM launch, if any, must be disabled in CHAMP mode.
    save_xml(run/'frames.urdf',urdf)
    p=ET.SubElement(model,'plugin',{'filename':'gz_ros2_control-system','name':'gz_ros2_control::GazeboSimROS2ControlPlugin'})
    ET.SubElement(p,'parameters').text=str(stage/'ros_control.yaml')
    ET.SubElement(p,'robot_param_node').text='robot_state_publisher'
    o=ET.SubElement(model,'plugin',{'filename':str(stage/native.name),'name':'champ_compare::NativeObserver'})
    ET.SubElement(o,'duration_s').text=str(duration+8)
    ET.SubElement(o,'done_file').text=str(stage/'monitor_done.json')
    ET.SubElement(o,'state_file').text=str(stage/'native_state.json')
    save_xml(run/'world.sdf',world)
    # Remove only actuation plugins for exact XML identity verification of physical scene.
    a=ET.fromstring(initial);b=ET.fromstring(canonical(world))
    for tree in (a,b):
        m=tree.find("world/model[@name='go2']")
        for p in list(m.findall('plugin')):
            text=p.get('name','')
            if text in ('teacher_sim::TeacherActuator','gz_ros2_control::GazeboSimROS2ControlPlugin','champ_compare::NativeObserver'):m.remove(p)
    if canonical(a)!=canonical(b):raise RuntimeError('CHAMP conversion changed the physical scene')
    contract={'schema':'champ_execution_contract/v1','controller_kind':'champ','run':str(run),'physical_scene_identical_except_actuation_plugins':True,
        'exclusive_effort_writer':'gz_ros2_control::GazeboSimROS2ControlPlugin','observer':'champ_compare::NativeObserver','observer_writes_joint_force':False,
        'observer_state_phase':'PostUpdate','state_time_offset_s':0,'duration_s':duration,'execution_duration_s':duration+8,'terminal_parking_after_navigation_duration_s':8,'input_world_sha256':sha(stage/'input_teacher_world.sdf'),'world_sha256':sha(run/'world.sdf'),
        'physical_model_mass_kg':asset['model_mass_kg'],'physics_step_s':asset['physics_step_s'],'joint_names_training_order':asset['joint_names'],
        'initialization':'Original ros2_control URDF initial positions (0,.9,-1.8 per leg), native hold_joints Pg100 until trajectory controller; CHAMP adapter nominal calibration .5 sim seconds and >=50 actual raw samples; zero body cmd before3s',
        'body_pose_resets':0,'body_servo':False,'body_stabilizer':False,'auxiliary':['original bounded nominal joint-stop adapter'],
        'pid':{'p':220.982919,'i':.2,'d':1.,'i_clamp':2.5,'configured_controller_manager_hz':250,'actual_period_requires_runtime_evidence':True},
        'command_boundary':{'file':'navigation_command.json','source':'scan_slam','mode':'scan_slam','dual_ttl_s':.3,'future_tolerance_s':.05,'limits':[.3,.2,.3],'slew_per_s':[.6,.6,.8]},
        'health':{'topic':'/demo/champ/execution_health','schema':1,'mode':'champ','source':'scan_slam','states':['ready','hold','failed'],'requires':['actual controller_state feedback fresh','actual Adapter nominal calibration','unique command/target publishers']},
        'selected_gait':selected,'qualified_gait_deployment':{'path':str(WORKSPACE/'deployment_manifest.json'),'sha256':sha(WORKSPACE/'deployment_manifest.json')},
        'controller_manager_library':{'path':str(controller_manager_library()),'sha256':sha(controller_manager_library())},
        'gz_ros2_control_plugin':{'path':'/opt/ros/jazzy/lib/libgz_ros2_control-system.so','sha256':sha('/opt/ros/jazzy/lib/libgz_ros2_control-system.so')},
        'executor_source_sha256':{str(p):sha(p)for p in [HERE/'prepare.py',HERE/'baseline.launch.py',HERE/'command_file_reader.py',HERE/'native/champ_native_observer.cpp',HERE/'native/CMakeLists.txt',TEACHER/'policy/worker.py',TEACHER/'policy/observation.py']},
        'artifacts':{str(p):sha(p)for p in stage.rglob('*')if p.is_file()},'protected_before':protected,
        'teacher_actor_started':False,'actual_runtime_verified':False,'historical_camera_demo_used_as_motion_evidence':False}
    if any(sha(p)!=h for p,h in protected.items()):raise RuntimeError('Protected original file changed during preparation')
    (run/'champ_contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    asset.update(exclusive_writer=contract['exclusive_effort_writer'],controller_kind='champ',disabled=['Teacher actor','TeacherActuator','body_stabilizer','body_servo'],world_sha256=sha(run/'world.sdf'),champ_contract_sha256=sha(run/'champ_contract.json'))
    (run/'asset_manifest.json').write_text(json.dumps(asset,indent=2)+'\n')
    return contract

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--duration',type=float,default=120.);a=p.parse_args()
    c=prepare(a.run,a.duration);print(json.dumps({'run':c['run'],'world':str(a.run/'world.sdf'),'contract':str(a.run/'champ_contract.json'),'kind':'champ','runtime_verified':False}))
if __name__=='__main__':main()
