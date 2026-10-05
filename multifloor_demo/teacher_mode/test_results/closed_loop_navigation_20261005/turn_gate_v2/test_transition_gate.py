"""CPU-only source/differential tests; never Actor, ROS or physics."""
import ast,copy,hashlib,importlib.util,json,math,pathlib
BASE=pathlib.Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode')
OLD=BASE/'navigation/teacher_transition.py'
NEW=BASE/'navigation/closed_loop_multifloor_v1/transition_gate.py'
PROFILE=BASE/'navigation/closed_loop_cascade/profiles/flat_short.json'
def load(path,name):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m.TeacherHeadingGate
Old=load(OLD,'original_gate');New=load(NEW,'experiment_gate');profile=json.loads(PROFILE.read_text());checks=[]
def check(name,fn):
 fn();checks.append(dict(name=name,status='passed'))
def eq(a,b):assert a==b,(a,b)
def state(g):return dict(phase=g.phase,heading=g.heading,settle_until=g.settle_until,outside_since=g.outside_since,last_stamp=g.last_stamp,history=list(g.history))
def constructor_rejection():
 for cap in [math.nan,math.inf,-math.inf,-.1,0.,.600000001,1.]:
  p=copy.deepcopy(profile);p['max_yaw_rate_radps']=cap
  try:New(p)
  except ValueError:pass
  else:raise AssertionError(cap)
 for cap in [.1,.3,.6]:
  p=copy.deepcopy(profile);p['max_yaw_rate_radps']=cap;assert New(p).max_turn_rate==cap
check('constructor finite positive cap <= .6',constructor_rejection)
def unmodified_methods():
 def methods(path):
  tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef));return {n.name:ast.dump(n,include_attributes=False) for n in cls.body if isinstance(n,ast.FunctionDef)}
 a,b=methods(OLD),methods(NEW);eq(set(a),set(b))
 for name in a:
  if name!='__init__':eq(a[name],b[name])
check('all transition/observe/stopped/reset/evidence methods AST identical',unmodified_methods)
def differential_sequence():
 a,b=Old(copy.deepcopy(profile)),New(copy.deepcopy(profile));seq=[]
 def call(name,*args):
  ra=getattr(a,name)(*args);rb=getattr(b,name)(*args);eq(ra,rb);eq(state(a),state(b));seq.append([name,ra])
 call('update',1.,0.,1.,10.)
 for i in range(4):
  stamp=1.+i*.1;wall=10.+i*.1;call('observe',stamp,wall,[0.,0.,0.],0.,[0.,0.,0.],stamp-.005,True)
 call('stopped',1.3,10.3);call('update',1.3,0.,1.,10.3)
 call('observe',1.4,10.4,[0.,0.,0.],.3,[0.,0.,.3],1.395,False)
 call('update',1.4,.95,1.,10.4)
 for i in range(4):
  stamp=1.5+i*.1;call('observe',stamp,10.5+i*.1,[0.,0.,0.],0.,[0.,0.,0.],stamp-.005,True)
 call('update',1.8,1.,1.,10.8);call('update',1.85,1.,1.1,10.85)
 call('observe',1.9,10.9,[.2,0.,0.],0.,[0.,0.,0.],1.895,False)
 call('update',1.9,1.,2.,10.9)
 call('observe',1.9,10.9,[0.,0.,0.],0.,[0.,0.,0.],1.895,True)
 call('observe',2.,11.,[math.nan,0.,0.],0.,[0.,0.,0.],1.995,True)
 call('reset');call('resume_after_stop',1.,2.,100.)
 call('evidence',2.,11.)
 assert any(x[0]=='update' and x[1][0] is True for x in seq)
 return seq
check('cap .3 full mixed-phase calling sequence behavior identical',differential_sequence)
def turn_clips():
 p=copy.deepcopy(profile);p['max_yaw_rate_radps']=.6
 for heading,expected in [(2.,.6),(-2.,-.6),(.2,.26),(-.2,-.26)]:
  g=New(copy.deepcopy(p))
  for i in range(4):g.observe(1.+i*.1,10.+i*.1,[0.,0.,0.],0.,[0.,0.,0.],1.+i*.1-.005,True)
  canwalk,rate=g.update(1.3,0.,heading,10.3);assert not canwalk;assert abs(rate-expected)<1e-12,(rate,expected)
check('cap .6 signed clip and interior yaw-PD unchanged',turn_clips)
def raw_stop_rejections():
 defaults=dict(stamp_s=1.,wall_s=10.,velocity=[0.,0.,0.],slam_wz=0.,body_gyro=[0.,0.,0.],gyro_stamp_s=.995,zero_command=True)
 bad=[dict(velocity=[.030001,0.,0.]),dict(slam_wz=.050001),dict(body_gyro=[.04,.04,0.]),dict(zero_command=False),dict(gyro_stamp_s=.849999),dict(gyro_stamp_s=1.150001),dict(body_gyro=[math.nan,0.,0.]),dict(velocity=[0.,0.]),dict(body_gyro=[0.,0.])]
 for case in bad:
  kw={**defaults,**case};a,b=Old(copy.deepcopy(profile)),New(copy.deepcopy(profile));ra=a.observe(**kw);rb=b.observe(**kw);eq(ra,rb);assert not rb;eq(state(a),state(b));eq(state(a)['history'],[])
check('raw speed gyro norm zero-command pair/finite checks unchanged',raw_stop_rejections)
def freshness_and_gap():
 for cls in [Old,New]:
  g=cls(copy.deepcopy(profile))
  for i in range(4):assert g.observe(1.+i*.1,10.+i*.1,[0.,0.,0.],0.,[0.,0.,0.],1.+i*.1-.005,True)
  assert g.stopped(1.3,10.3)
  assert not g.stopped(1.3,10.601)
  assert not g.stopped(1.601,10.3)
  assert not g.stopped(1.249,10.3)
  before=state(g);assert not g.observe(1.3,10.4,[0.,0.,0.],0.,[0.,0.,0.],1.295,True);eq(before,state(g))
  assert g.observe(1.501,10.501,[0.,0.,0.],0.,[0.,0.,0.],1.496,True)
  assert len(g.history)==1 and not g.stopped(1.501,10.501)
check('300ms wall/sim freshness increasing-stamp max-gap preserved',freshness_and_gap)
def pair_contract_original():
 for delta in [-.005,.01]:
  a,b=Old(copy.deepcopy(profile)),New(copy.deepcopy(profile));eq(a.observe(1.,10.,[0.,0.,0.],0.,[0.,0.,0.],1.+delta,True),b.observe(1.,10.,[0.,0.,0.],0.,[0.,0.,0.],1.+delta,True))
  assert b.history
check('original absolute gyro-pose pair gate unchanged (caller enforces gyro<=pose)',pair_contract_original)
def walk_cap_unchanged():
 for cls in [Old,New]:
  p=copy.deepcopy(profile);p['max_yaw_rate_radps']=.3 if cls is Old else .6;g=cls(p);g.phase='drive';eq(g.update(1.,0.,.2,10.),(True,.08));eq(g.update(1.,0.,-.2,10.),(True,-.08))
check('drive correction .08 and drive error threshold unchanged',walk_cap_unchanged)
print(json.dumps(dict(schema='teacher_turn_cap_source_cpu_checks/v1',checks=checks,count=len(checks),status='passed',original_source_sha256=hashlib.sha256(OLD.read_bytes()).hexdigest(),new_source_sha256=hashlib.sha256(NEW.read_bytes()).hexdigest(),fixture_profile_sha256=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),simulation_experimental=True,NAV_point6_actual_verified=False,Actor_loaded=False,physics_launched=False,causality_note='The original gate uses absolute pose-gyro interval; actual caller must separately enforce gyro_stamp<=pose_stamp. This clone leaves that check unchanged.'),ensure_ascii=False,indent=2))
