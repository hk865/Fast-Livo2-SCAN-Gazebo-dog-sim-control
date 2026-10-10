#!/usr/bin/env python3
"""Generate only local assets; original Go2 packages remain untouched."""
import json, math, pathlib, subprocess, sys, xml.etree.ElementTree as E

ROOT=pathlib.Path(__file__).resolve().parent
PROJECT=ROOT.parent.parent
sys.path.insert(0, str(ROOT.parent))
from mission.route_regions import add_route_regions
OUT=ROOT/'generated'
OUT.mkdir(exist_ok=True)
IMU_RATE_HZ=1000  # Matches the 1 ms physics step; actual slope A/B is archived.

def box(name, xyz, size, color, rpy=(0,0,0), static=True):
    pose=' '.join(map(str,(*xyz,*rpy))); dims=' '.join(map(str,size)); rgba=' '.join(map(str,(*color,1)))
    return f'''<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose><link name="link"><inertial><mass>50</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name="collision"><geometry><box><size>{dims}</size></box></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision><visual name="visual"><geometry><box><size>{dims}</size></box></geometry><material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>'''

world=['''<?xml version="1.0"?><sdf version="1.9"><world name="multifloor_demo"><physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/><scene><ambient>0.65 0.65 0.65 1</ambient><background>0.14 0.18 0.24 1</background><shadows>false</shadows></scene><light name="sun" type="directional"><pose>0 0 12 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 -0.3 -1</direction></light>''']
world.append(box('floor_1',(7.5,3.5,-.1),(23,15,.2),(.55,.55,.55)))
world.append(box('floor_2',(16,4.5,1.1),(4,9,.2),(.3,.6,.55)))
world.append(box('floor_3',(-.5,4.5,2.3),(5,9,.2),(.4,.5,.8)))
slope=math.atan(.1); length=math.hypot(12,1.2)
# Top surface meets landing heights at x=2 and x=14; overlap 2 cm avoids seams.
for name,y,z,pitch,col in [('ramp_12',2,.6,-slope,(.55,.55,.35)),('ramp_23',7,1.8,slope,(.45,.55,.7))]:
    world.append(box(name,(8,y,z-.05*math.cos(slope)),(length+.04,2,.1),col,(0,pitch,0)))
for side,y in [('south',-3.5),('north',10.5)]:
    world.append(box('wall_'+side,(7.5,y,2),(23,.2,4),(.75,.75,.75)))
for x in [-3.5,18.5]: world.append(box(f'wall_{x}',(x,3.5,2),(.2,14,4),(.7,.7,.72)))
# Colored protruding wall panels produce real camera features and 3D returns.
palette=[(.8,.2,.15),(.15,.55,.8),(.9,.65,.12),(.3,.75,.25),(.7,.25,.7)]
for i in range(28):
    x=-2.5+(i%14)*1.5; y=-3.32 if i<14 else 10.32
    world.append(box(f'panel_{i}',(x,y,1+(i%3)*.65),(.65,.15,.5),palette[i%5]))
for i,xyz in enumerate([(-2,-1,.5),(4,-2,.45),(8,-2,.65),(12,-2,.5),(17.2,4.5,1.7),(-2,5,2.9)]):
    world.append(box(f'landmark_{i}',xyz,(.5,.5,1),palette[i%5]))
# This obstacle is moved only by the explicit scenario service; robot is never teleported.
world.append(box('moving_obstacle',(1,4,0.6),(.5,.5,1.2),(.9,.1,.1)))
world.append('</world></sdf>'); (OUT/'three_floors.sdf').write_text('\n'.join(world))

desc=subprocess.check_output(['ros2','pkg','prefix','unitree_go2_description'],text=True).strip()
base=subprocess.check_output(['xacro',str(pathlib.Path(desc)/'share/unitree_go2_description/urdf/unitree_go2_robot.xacro'),f'robot_controllers:={ROOT}/config/ros_control.yaml'],text=True)
robot=E.fromstring(base)
for gz in robot.findall('gazebo'):
    for sensor in list(gz.findall('sensor')):
        if sensor.get('type') not in ('imu','gpu_lidar') or sensor.get('name')!='velodyne-VLP16' and sensor.get('type')!='imu': gz.remove(sensor)
        elif sensor.get('type')=='imu':
            for plugin in sensor.findall('plugin'):sensor.remove(plugin)
            sensor.find('update_rate').text=str(IMU_RATE_HZ)
            # Explicit static heading reference; empty/default parent means the
            # sensor's initial local orientation in Gazebo, not world axes.
            imu=sensor.find('imu')
            if imu is None:imu=E.SubElement(sensor,'imu')
            old_reference=imu.find('orientation_reference_frame')
            if old_reference is not None:imu.remove(old_reference)
            reference=E.SubElement(imu,'orientation_reference_frame')
            E.SubElement(reference,'localization').text='CUSTOM'
            E.SubElement(reference,'custom_rpy',{'parent_frame':'world'}).text='0 0 0'
        elif sensor.get('type')=='gpu_lidar':
            sensor.find('lidar/scan/horizontal/samples').text='480'
            sensor.find('lidar/scan/vertical/samples').text='32'
            sensor.find('lidar/range/max').text='35'
    for p in gz.findall('plugin'):
        if 'OdometryPublisher' in p.get('name',''):
            p.find('odom_topic').text='/demo/ground_truth'; p.find('odom_frame').text='world'
            E.SubElement(p,'dimensions').text='3'
for joint in robot.findall('ros2_control/joint'):
    value=0 if '_hip_' in joint.get('name') else .9 if '_upper_' in joint.get('name') else -1.8
    pos=joint.find("state_interface[@name='position']")
    E.SubElement(pos,'param',{'name':'initial_value'}).text=str(value)
for link,xyz,rpy,parent in [('demo_camera_link','.28 0 .1','0 0 0','base_link'),('demo_camera_optical_frame','0 0 0','-1.5707963267948966 0 -1.5707963267948966','demo_camera_link')]:
    E.SubElement(robot,'link',{'name':link})
    j=E.SubElement(robot,'joint',{'name':link+'_joint','type':'fixed'})
    E.SubElement(j,'parent',{'link':parent}); E.SubElement(j,'child',{'link':link}); E.SubElement(j,'origin',{'xyz':xyz,'rpy':rpy})
gz=E.SubElement(robot,'gazebo',{'reference':'demo_camera_link'})
gz.append(E.fromstring('''<sensor name="demo_rgb" type="camera"><topic>/demo/camera</topic><gz_frame_id>demo_camera_optical_frame</gz_frame_id><update_rate>10</update_rate><always_on>true</always_on><camera><horizontal_fov>1.3962634015954636</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>.05</near><far>35</far></clip></camera></sensor>'''))
# Contact evidence is separate: trunk contact is invalid, foot/ground contact is expected.
contacts=[('body','base_link','base_link_fixed_joint_lump__trunk_collision','/demo/body_contacts')]
for leg in ['lf','rf','lh','rh']:
    contacts.append((leg+'_foot',leg+'_lower_leg_link',leg+'_lower_leg_link_fixed_joint_lump__'+leg+'_foot_link_collision_1','/demo/contacts/'+leg+'_foot'))
for name,link,collision,topic in contacts:
    gz=E.SubElement(robot,'gazebo',{'reference':link})
    gz.append(E.fromstring(f'<sensor name="demo_{name}_contacts" type="contact"><always_on>true</always_on><update_rate>50</update_rate><topic>{topic}</topic><contact><collision>{collision}</collision></contact></sensor>'))
(OUT/'go2.urdf').write_text(E.tostring(robot,encoding='unicode'))
scenario={'world':'multifloor_demo','geometry':'three physical platforms connected by 10% ramps; not stair climbing','floor_elevations':[0,1.2,2.4],'spawn':{'x':0,'y':0,'z':.30,'yaw':0},'nominal_standing_body_height':.30,'frame':'camera_init','coordinates':'relative world-axis displacements from spawn; mission freezes sensor-derived heading and rotates these vectors into camera_init','exploration':[[0,-1,0],[1,-1,0],[1,0,0]],'return_origin':[[0,0,0]],'navigation_f1_f3':[[0,2,0],[2,2,0],[5,2,.3],[8,2,.6],[11,2,.9],[14,2,1.2],[16,2,1.2],[16,7,1.2],[14,7,1.2],[11,7,1.5],[8,7,1.8],[5,7,2.1],[2,7,2.4],[0,7,2.4]],'dynamic_obstacle':{'entity':'moving_obstacle','initial':[1,4,.6],'crossing_plane_x':1,'route_y':2,'surface_z':0,'size':[.5,.5,1.2]},'sensors':{'imu':{'frame':'imu_link','body_xyz':[0,0,0],'body_rpy':[0,0,0],'rate':100,'topic':'/livox/imu'},'lidar':{'frame':'velodyne','body_xyz':[.2,0,.1177],'body_rpy':[0,0,0],'rate':10,'topic':'/livox/lidar','samples':[480,32]},'camera':{'frame':'demo_camera_optical_frame','body_xyz':[.28,0,.1],'body_rpy':[-math.pi/2,0,-math.pi/2],'rate':10,'topic':'/camera/image_color','width':640,'height':480,'hfov':math.radians(80),'fx':320/math.tan(math.radians(40)),'fy':320/math.tan(math.radians(40)),'cx':320,'cy':240}}}
scenario.update(frame_id='camera_init', floor_heights=scenario['floor_elevations'], body_clearance=.30, origin=[0,0,0], slam_origin_in_world=[0,0,.30], waypoint_semantics='world-axis displacement anchored at first healthy SLAM body pose after fixed heading conversion; z is body elevation change, not absolute ground', ground_waypoints=scenario['navigation_f1_f3'])
scenario['sensors']['imu']['rate']=IMU_RATE_HZ
scenario['dynamic_obstacle']['activation_navigation_waypoint_index']=1
scenario['dynamic_obstacle']['activation_condition']='current matching NAV request is running at index 1, after the northward approach actually reached its first waypoint'
scenario['sensors']['imu']['orientation_reference']={'description':'Gazebo IMU CUSTOM/world identity','world_quaternion':[0,0,0,1],'body_imu_quaternion':[0,0,0,1]}
scenario.update(waypoint_reference='relative_world_axes',simulation={'description':'Gazebo Go2 四足物理 · 三层平台/10%缓坡（非楼梯）','kind':'gazebo_physics','terrain':'ramps'})
(ROOT/'scenario_smoke.json').write_text(json.dumps(scenario,indent=2)+'\n')
scenario['exploration']=scenario['exploration']+[[0,0,0]]+scenario['navigation_f1_f3']
scenario['return_origin']=list(reversed(scenario['navigation_f1_f3'][:-1]))+[[0,0,0]]
scenario=add_route_regions(scenario)
(ROOT/'scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
# Qualified measured-joint asset: append the exact read-only sensor used by the
# component runs. The existing ROS control parameter path already points to
# the standard config/ros_control.yaml, whose bytes are the tested lowD profile.
model_text=(OUT/'go2.urdf').read_text()
joint_names=[j.attrib['name'] for j in E.fromstring(model_text).findall('joint')
             if j.attrib['type']=='revolute']
if len(joint_names)!=12 or len(set(joint_names))!=12 or model_text.count('</robot>')!=1:
    raise ValueError('Expected the qualified twelve-joint robot model')
measured_plugin='\n  <gazebo>\n    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">\n      <topic>/demo/physical_joint_states</topic>\n'
measured_plugin+=''.join(f'      <joint_name>{name}</joint_name>\n' for name in joint_names)
measured_plugin+='    </plugin>\n  </gazebo>\n'
(OUT/'go2_measured.urdf').write_text(model_text.replace('</robot>',measured_plugin+'</robot>'))
(OUT/'measured_joint_bridge.yaml').write_text('- gz_topic_name: /demo/physical_joint_states\n  ros_topic_name: /demo/control/measured_joint_states\n  gz_type_name: gz.msgs.Model\n  ros_type_name: sensor_msgs/msg/JointState\n  direction: GZ_TO_ROS\n  qos_profile: SENSOR_DATA\n  publisher_queue: 5\n  subscriber_queue: 5\n  lazy: false\n')
print(json.dumps({'world':str(OUT/'three_floors.sdf'),'robot':str(OUT/'go2.urdf'),'scenario':str(ROOT/'scenario.json')}))
