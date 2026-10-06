"""Pure preview audit using frozen 55f6 raw path/SLAM; no ROS/Teacher."""
import ast,copy,hashlib,importlib.util,json,math,pathlib,sys,types
import numpy as np
BASE=pathlib.Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo')
HERE=BASE/'teacher_mode/navigation/closed_loop_multifloor_v3'
RUN=BASE/'teacher_mode/runs/20261005_144028_closed_loop_cascade_registered_ramp12_turn06_r2_55f6'
sys.path.insert(0,str(HERE))
from cascade_core import Controller,rotation
CORE_SHA='a195c5c2c0d5494d72184c988cd3fea3daa7c5eebd0ba9ac4b9e4186e11e9415'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(l)for l in p.read_text().splitlines()if l.strip()]
assert sha(HERE/'cascade_core.py')==CORE_SHA
for file in HERE.glob('*.py'):compile(file.read_text(),str(file),'exec',dont_inherit=True)
tree=ast.parse((HERE/'controller.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='follow_checked_trajectory')
result_holder={}
def follow(*args,**kwargs):return copy.deepcopy(result_holder['result'])
ns={'copy':copy,'np':np,'math':math,'follow_trajectory':follow};exec(compile(ast.Module(body=[method],type_ignores=[]),'extracted frozen preview','exec'),ns);preview_method=ns['follow_checked_trajectory']
class Evidence:
 def __init__(self):self.rows=[]
 def append(self,path,row):self.rows.append(copy.deepcopy(row))
profile=json.loads((RUN/'navigation_profile.json').read_text());pid=rows(RUN/'navigation_pid_history.jsonl');status=rows(RUN/'navigation_status.jsonl');stats=[x for x in status if x.get('steering')];arrays={};c=None;key=None;path_id=None;differences=[];reproductions=[];fresh_count=0
for row in pid:
 d=row['cascade'];r=row.get('path_receipt')
 if not r or not row.get('feedback') or not d.get('fixed_goal'):continue
 nextkey=(row['request_id'],row['waypoint_index']);g=d['fixed_goal']
 if key!=nextkey:
  key=nextkey;c=Controller(profile['cascade'],g['position_world_xyz'],g['heading_rad'],g['goal_id']);path_id=None
 if r['path_id']!=path_id:
  file=RUN/r['array_file'];samples=arrays.setdefault(str(file),np.load(file)['samples']);points=samples[r['source_sample_indices']]
  assert hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()==r['points_float64_sha256']
  c.set_path(points,r['path_id'],r['stamp_ns'],r['received_wall_ns'],r['frame_id']);path_id=r['path_id']
 if not d.get('controller_updated') or d['mode'] not in ('drive','turn'):continue
 f=row['feedback'];pos=np.asarray(f['position_world_xyz']);R=rotation(f['quaternion_wxyz']);actual_yaw=math.atan2(R[1,0],R[0,0]);clock=row['control_stamp_ns']
 matching=min(stats,key=lambda s:abs(s['ros_sim_time']-clock/1e9));old=matching['steering']
 original=(np.array([.01,.02]),.123,np.asarray(old['target']),copy.deepcopy(old));result_holder['result']=original
 fake=types.SimpleNamespace(cascade=c,pose=pos,rotation=R,pose_stamp=f['stamp_ns'],feedback=f,request_id=row['request_id'],waypoint_index=row['waypoint_index'],run=RUN,evidence=Evidence(),ensure_cascade=lambda:None,get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=clock)))
 before=(c.progress,c.new_path,c.velocity_integral.copy(),c.hold_integral.copy());result=preview_method(fake,pos,R,np.zeros((2,3)),np.zeros(3),return_steering=True)
 assert (c.progress,c.new_path)==before[:2];np.testing.assert_array_equal(c.velocity_integral,before[2]);np.testing.assert_array_equal(c.hold_integral,before[3]);np.testing.assert_array_equal(result[0],original[0]);assert result[1]==original[1];np.testing.assert_array_equal(result[2],original[2]);assert result[3]['direction']==old['direction']
 result2=preview_method(fake,pos,R,np.zeros((2,3)),np.zeros(3),return_steering=True);assert result2[3]==result[3] and c.progress==before[0]
 preview_yaw=result[3]['heading'];diff=abs(math.atan2(math.sin(preview_yaw-d['reference_yaw_rad']),math.cos(preview_yaw-d['reference_yaw_rad'])));differences.append(diff);assert diff<1e-12,(clock,diff)
 committed=c._project(pos);assert np.linalg.norm(committed[1]-d['nearest_projection_xyz'])<1e-10;assert abs(c.progress-d['progress_m'])<1e-10;fresh_count+=1
 if 54<=clock/1e9<=65:
  reproductions.append(dict(control_time_s=clock/1e9,source_stamp_s=f['stamp_ns']/1e9,path_id=path_id,old_lookahead_heading_rad=old['heading'],old_lookahead_error_rad=old['error'],actual_SLAM_yaw_rad=actual_yaw,new_preview_heading_rad=preview_yaw,new_preview_error_rad=result[3]['error'],original_core_heading_rad=d['reference_yaw_rad'],original_core_mode=d['mode'],original_core_actual_command=d['command_body'],source_pose_used_exactly=True,original_checked_target_and_direction_unchanged=True,progress_committed_by_preview=False))
assert fresh_count>100 and len(reproductions)>50
from transition_gate import TeacherHeadingGate
for sign in (1.,-1.):
 sample=reproductions[len(reproductions)//2];yaw=sample['actual_SLAM_yaw_rad']*sign;heading=sample['new_preview_heading_rad']*sign;old_heading=sample['old_lookahead_heading_rad']*sign
 gate=TeacherHeadingGate(profile);gate.phase='drive';allowed,rate=gate.update(60.,yaw,heading,100.)
 assert allowed and abs(rate)<.01
 oldgate=TeacherHeadingGate(profile);oldgate.phase='drive';oldallowed,_=oldgate.update(60.,yaw,old_heading,100.)
 assert not oldallowed
# Endpoint goal branch is exactly the frozen original core branch.
end=Controller(profile['cascade'],[1.,0.,0.],.8,'endpoint');end.set_path([[0.,0.,0.],[1.,0.,0.]],'end',0,0);end.progress=.95;end.new_path=False
fake.cascade=end;fake.pose=np.array([.98,0.,0.]);fake.rotation=np.eye(3);result_holder['result']=(np.zeros(2),0.,np.array([1.,0.,0.]),{'heading':0.,'error':0.,'direction':[1.,0.], 'target':[1.,0.,0.]});result=preview_method(fake,fake.pose,np.eye(3),np.zeros((2,3)),np.zeros(3),return_steering=True);assert result[3]['heading']==.8 and end.progress==.95
result=dict(schema='actual_SCANNearProjection_heading_preview_source_audit/v1',status='passed_offline_only',checks=['core SHA exact frozen original','all local source compile no pyc','all fresh 55f6 drive/turn preview heading equals original core','preview repeated calls no progress/newpath/I commit','checked target/velocity/rate/direction untouched','positive and mirrored-negative original lookahead enters turn but projection stays aligned','endpoint fixed goal yaw threshold matches'],fresh_rows_rebuilt=fresh_count,conflict_rows_54_to65=len(reproductions),maximum_heading_difference_vs_original_core_rad=max(differences),conflict_reproduction=reproductions,source_sha256={str(p):sha(p)for p in [HERE/'controller.py',HERE/'cascade_core.py',HERE/'transition_gate.py',RUN/'navigation_pid_history.jsonl',RUN/'navigation_status.jsonl',RUN/'navigation_profile.json']},scope='Offline original failed-run geometry diagnostics only; altered gate behavior does not simulate altered actual trajectories',navigation_actual_pass_assigned=False,Actor_loaded=False,ROS_started=False,physics_launched=False)
print(json.dumps(result,ensure_ascii=False,indent=2))
