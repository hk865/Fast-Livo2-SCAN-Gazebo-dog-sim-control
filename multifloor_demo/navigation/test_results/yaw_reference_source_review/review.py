"""Finite source and four-leg algebra review, no ROS or physical computation."""
from pathlib import Path
import hashlib,json,math,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[3]
SIM=ROOT/'simulation'
RUN=SIM/'test_results/20261002_downhill_b_enabled_v4'
OUT=Path(__file__).resolve().parent
manifest=json.loads((RUN/'runtime_manifest.json').read_text())
selected=manifest['selected_gait'];source=Path(selected['source_root'])
scenario=json.loads((SIM/'scenario.json').read_text())
urdf=SIM/'generated/go2_measured.urdf';robot=ET.parse(urdf).getroot()
joints={j.attrib['name']:j for j in robot.findall('joint')}
def origin(name):return tuple(map(float,joints[name].find('origin').attrib['xyz'].split()))
feet={}
for leg in ('lf','rf','lh','rh'):
 h,u=origin(leg+'_hip_joint'),origin(leg+'_upper_leg_joint')
 feet[leg]=(h[0]+u[0],h[1]+u[1])
T=.35;omega=.12;theta=2*math.sin(T*omega/4)
examples=[]
for leg,(x,y) in feet.items():
 delta=(math.cos(theta)*x-math.sin(theta)*y-x,math.sin(theta)*x+math.cos(theta)*y-y)
 expected=(omega*y,-omega*x) # -omega cross r: velocity of a world-fixed foot in body axes.
 stance=(-2*delta[0]/T,-2*delta[1]/T)
 examples.append(dict(leg=leg,nominal_xy_m=[x,y],half_step_delta_m=list(delta),
  stance_reference_velocity_m_s=list(stance),expected_small_angle_fixed_foot_velocity_m_s=list(expected),
  sign_agreement=all(a*b>0 for a,b in zip(stance,expected))))
def sum_vector(legs,field):
 vals=[e[field] for e in examples if e['leg'] in legs]
 return [sum(v[i] for v in vals) for i in (0,1)]
checks={
 'selected_private_source36_all_sha_match':len(selected['source_sha256'])==36 and all(hashlib.sha256((source/f).read_bytes()).hexdigest()==h for f,h in selected['source_sha256'].items()),
 'model_matches_physics_prepared_sha':hashlib.sha256(urdf.read_bytes()).hexdigest()==json.loads((RUN/'fixture_manifest.json').read_text())['physical_model_sha256'],
 'imu_mount_fixed_identity_in_actual_URDF':joints['imu_joint'].attrib['type']=='fixed' and joints['imu_joint'].find('parent').attrib['link']=='trunk' and joints['imu_joint'].find('origin').attrib=={'rpy':'0 0 0','xyz':'0 0 0'},
 'declared_IMU_body_and_world_reference_identity':scenario['sensors']['imu']['body_rpy']==[0,0,0] and scenario['sensors']['imu']['orientation_reference']['body_imu_quaternion']==[0,0,0,1] and scenario['sensors']['imu']['orientation_reference']['world_quaternion']==[0,0,0,1],
 'all_four_stance_signs_match_minus_omega_cross_r':all(e['sign_agreement'] for e in examples),
 'pure_yaw_four_leg_reference_sum_zero':max(map(abs,sum_vector(('lf','rf','lh','rh'),'half_step_delta_m')))<1e-14,
 'each_trot_diagonal_reference_sum_zero':all(max(map(abs,sum_vector(pair,'half_step_delta_m')))<1e-14 for pair in (('lf','rh'),('rf','lh'))),
}
paths={
 'leg_velocity_transform':source/'champ/include/champ/leg_controller/leg_controller.h',
 'stance_swing_projection':source/'champ/include/champ/leg_controller/trajectory_planner.h',
 'right_handed_rotation':source/'champ/include/champ/geometry/geometry.h',
 'nominal_leg_frame':source/'champ/include/champ/quadruped_base/quadruped_leg.h',
 'leg_order':source/'champ/include/champ/quadruped_base/quadruped_base.h',
 'twist_callback':source/'champ_base/src/quadruped_controller.cpp',
 'body_pose_inverse_rotation':source/'champ/include/champ/body_controller/body_controller.h',
 'SLAM_body_frame':ROOT/'slam/geometry.py',
 'sensor_scene_heading':ROOT/'slam/heading_alignment.py',
 'SLAM_heading_error':ROOT/'navigation/control_core.py',
 'scenario':SIM/'scenario.json','physical_model':urdf,
}
r=dict(passed=all(checks.values()),checks=checks,scope='Finite local-source/algebra review, not a controller/ROS/IK/contact or dynamic plant test.',
 selected_gait_executable=selected['executable'],selected_gait_library=selected['library'],
 sources={k:dict(path=str(v),sha256=hashlib.sha256(v.read_bytes()).hexdigest()) for k,v in paths.items()},
 parameters=dict(stance_duration_s=T,positive_body_yaw_rate_rad_s=omega,rotation_half_step_rad=theta,com_x_translation_m=0),four_leg_examples=examples,
 formulas={'positive_rotate_z':'Rz(theta)[x,y] = [cos(theta)x - sin(theta)y, sin(theta)x + cos(theta)y]',
 'step_theta':'2 sin(Tstance * omega / 4); small angle Tstance * omega / 2',
 'half_step':'delta = (Rz(theta)-I) r',
 'stance':'x(s) = (L/2)(1-2s), hence foot horizontal reference rate = -2 delta / Tstance',
 'contact_kinematic_expectation':'A world-fixed contact foot in body axes has velocity -omega cross r = [omega*y,-omega*x]. This is a sign expectation, not a no-slip control input.',
 'SLAM_frame':'R_camera_init_body = R_camera_init_IMU R_body_IMU.T; yaw = atan2(R[1,0],R[0,0]).',
 'scene_rotation':'R_camera_init_world is proper Rz(alpha), det +1; yaw_camera = yaw_world + alpha, so target minus body heading and yaw sign are preserved.'},
 source_lines={'cmdVel angular copied unchanged':'champ_base/src/quadruped_controller.cpp:116-121',
 'positive step direction':'champ/include/champ/leg_controller/leg_controller.h:67-109',
 'stance reduces scalar vs swing recovers scalar':'champ/include/champ/leg_controller/trajectory_planner.h:128-153',
 'positive RotateZ':'champ/include/champ/geometry/geometry.h:292-303',
 'body pose yaw compensation inverse':'champ/include/champ/body_controller/body_controller.h:79-91',
 'IMU/body transform':'slam/geometry.py:12-22','heading transform':'slam/heading_alignment.py:32-43',
 'positive heading error':'navigation/control_core.py:219-220,266-273'},
 conclusion='No definite sign inversion was found in this limited source chain. Pure-yaw stance foot directions agree with positive ROS right-handed body yaw; symmetric pure-yaw references contain no common longitudinal translation. This does not guarantee actual yaw or zero translation under contacts.',
 limitations=['No physical rerun, no ROS nodes, no suite, no benchmark, no source edits.',
 'Current input axes match trunk/IMU identity mounting; this does not audit unrelated optical camera frames.',
 'Finite algebra uses source formulas and true URDF offsets, not native live phase timestamps or an exact float32 header harness.',
 'Body tilt, joint tracking/clamping, contact slip/asymmetry, terrain, phase transients and actuator timing can still create actual retreat or opposite yaw. None is assigned a unique cause here.',
 'Selected binary/source SHAs are from the run manifest; actual runtime mappings are owned by the existing root capture.'])
(OUT/'result.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({'passed':r['passed'],'checks':checks,'theta':theta,'four_leg_examples':examples,'result_sha256':hashlib.sha256((OUT/'result.json').read_bytes()).hexdigest()},indent=2))
