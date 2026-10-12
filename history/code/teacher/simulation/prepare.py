#!/usr/bin/env python3
"""Generate isolated Go2 assets. Never edit the camera or CHAMP assets."""
import argparse, copy, hashlib, json, math
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT.parent
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def xml(p, e):
    E.indent(e); p.write_text(E.tostring(e, encoding='unicode')+'\n')
def main():
    a=argparse.ArgumentParser(); a.add_argument('--output',type=Path,default=ROOT/'simulation/generated')
    a.add_argument('--terrain',choices=['flat','ramp_up','ramp_down','step05','step10'],default='flat')
    a.add_argument('--sensors',action='store_true'); a.add_argument('--camera-only',action='store_true');args=a.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    source=DEMO/'simulation/generated/go2_converted.sdf'
    model=copy.deepcopy(E.parse(source).getroot().find('model'))
    # The only torque writer is the new plugin. Pose/odometry systems are read-only.
    for p in list(model.findall('plugin')): model.remove(p)
    qdefault=[-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5]
    mapping=[f'{leg}_{part}_joint' for leg in ['rf','lf','rh','lh'] for part in ['hip','upper_leg','lower_leg']]
    lower=[-1.0472,-1.5708,-2.7227]*2+[-1.0472,-.5236,-2.7227]*2
    upper=[1.0472,3.4907,-.83776]*2+[1.0472,4.5379,-.83776]*2
    for index,name in enumerate(mapping):
        j=model.find(f"joint[@name='{name}']")
        if j is None: raise ValueError(name)
        j.find('axis/limit/effort').text='23.5';j.find('axis/limit/velocity').text='30'
        j.find('axis/limit/lower').text=str(lower[index]);j.find('axis/limit/upper').text=str(upper[index])
        dynamics=j.find('axis/dynamics')
        if dynamics is None:dynamics=E.SubElement(j.find('axis'),'dynamics')
        for field in ['damping','friction','spring_stiffness']:
            e=dynamics.find(field)
            if e is None:e=E.SubElement(dynamics,field)
            e.text='0'
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type')!='contact' and not args.sensors and not (args.camera_only and sensor.get('type')=='camera'): link.remove(sensor)
            elif sensor.get('type')=='imu':sensor.find('update_rate').text='200'
            elif sensor.get('type')=='camera':
                info=sensor.find('camera/camera_info_topic')
                if info is None:info=E.SubElement(sensor.find('camera'),'camera_info_topic')
                info.text='/demo/camera_info'
                if args.camera_only:sensor.find('update_rate').text='2'
        for contact in link.findall("sensor[@type='contact']"):
            contact.find('update_rate').text='200'
    # The original URDF contact friction was .2. Archive uses random .5..1.0;
    # deterministic matched baseline uses .7, never conceal this controlled change.
    for col in model.findall('.//collision'):
        for mu in col.findall('surface/friction/ode/mu')+col.findall('surface/friction/ode/mu2'):mu.text='.7'
    # Flat tests are away from upper-deck overhead and scene walls. The
    # original (0,0) spawn has a third-floor deck above its height scan.
    x,y,z,yaw=6,-.7,.40,0
    if args.terrain=='ramp_up': x,y,z=4,2,.60
    if args.terrain=='ramp_down': x,y,z,yaw=8,2,1.,math.pi
    pose=model.find('pose')
    if pose is None:pose=E.SubElement(model,'pose')
    pose.text=f'{x} {y} {z} 0 0 {yaw}'
    p=E.SubElement(model,'plugin',{'filename':'libteacher_actuator.so','name':'teacher_sim::TeacherActuator'})
    # Plugin source uses these native defaults too; persisted for review.
    for key,val in [('kp','25'),('kd','.5'),('effort_limit','23.5'),('velocity_limit','30')]:E.SubElement(p,key).text=val
    world=E.parse(DEMO/'simulation/generated/three_floors.sdf').getroot()
    w=world.find('world');w.set('name','teacher_demo')
    w.find('physics/max_step_size').text='.005'
    if not args.sensors and not args.camera_only:
        for wp in list(w.findall('plugin')):
            if any(n in wp.get('name','') for n in ['Sensors','Imu']):w.remove(wp)
    if args.terrain.startswith('step'):
        height=.05 if args.terrain=='step05' else .1
        step=E.fromstring(f'<model name="teacher_low_step"><static>true</static><pose>8 -.7 {height/2} 0 0 0</pose><link name="link"><collision name="collision"><geometry><box><size>2 1.4 {height}</size></box></geometry></collision><visual name="visual"><geometry><box><size>2 1.4 {height}</size></box></geometry><material><ambient>.8 .5 .2 1</ambient><diffuse>.8 .5 .2 1</diffuse></material></visual></link></model>')
        w.append(step)
    w.append(model);xml(args.output/'world.sdf',world)
    # Read-only ROS frame/joint tree, stripped of every actuation element.
    urdf=E.parse(DEMO/'simulation/generated/go2_measured.urdf').getroot()
    for tag in ['ros2_control','gazebo','transmission']:
        for e in list(urdf.findall(tag)):urdf.remove(e)
    xml(args.output/'frames.urdf',urdf)
    meta={'terrain':args.terrain,'sensors':args.sensors,'camera_only':args.camera_only,'spawn':[x,y,z,yaw],
          'physics_step_s':.005,'decimation':4,'policy_hz':50,'exclusive_writer':'teacher_sim::TeacherActuator',
          'disabled':['CHAMP','body_stabilizer','ros2_control','JointTrajectoryController'],
          'joint_mapping':dict(zip(['FR','FL','RR','RL'],['rf','lf','rh','lh'])),
          'joint_names':mapping,'default_q':qdefault,'hard_lower':lower,'hard_upper':upper,'friction_baseline':.7,
          'model_mass_kg':sum(float(v.text)for v in model.findall('link/inertial/mass')),
          'base_inertial_pose':model.find("link[@name='base_link']/inertial/pose").text,
          'source_hashes':{str(s):sha(s)for s in [source,DEMO/'simulation/generated/three_floors.sdf']},
          'world_sha256':sha(args.output/'world.sdf'),'true_stairs':False}
    (args.output/'asset_manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta))
if __name__=='__main__':main()
