from pathlib import Path
import xml.etree.ElementTree as E
import socket,struct,threading,subprocess,time,os,signal,json,math,hashlib
ROOT=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo')
OUT=ROOT/'teacher_mode/test_results/native_actuator_audit'
OUT.mkdir(parents=True,exist_ok=True)
MODEL=E.parse(ROOT/'simulation/generated/go2_converted.sdf').getroot().find('model')
DEFAULT=[-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5]
LIB=ROOT/'teacher_mode/simulation/build/libteacher_actuator.so'

def run(name):
 d=OUT/name;d.mkdir(exist_ok=True)
 model=E.fromstring(E.tostring(MODEL));model.set('name','go2')
 for p in list(model.findall('plugin')):model.remove(p)
 for link in model.findall('link'):
  for s in list(link.findall('sensor')):link.remove(s)
 E.SubElement(model,'pose').text='0 0 .35 0 0 0'
 plugin=E.SubElement(model,'plugin',{'filename':str(LIB),'name':'teacher_sim::TeacherActuator'})
 E.SubElement(plugin,'initial_q').text=' '.join(map(str,DEFAULT))
 if name=='duplicate':
  p=E.SubElement(model,'plugin',{'filename':'gz-sim-joint-position-controller-system','name':'gz::sim::systems::JointPositionController'})
  E.SubElement(p,'joint_name').text='rf_hip_joint'
 sdf=E.Element('sdf',{'version':'1.9'});w=E.SubElement(sdf,'world',{'name':'native_teacher_audit'})
 phys=E.SubElement(w,'physics',{'name':'physics','type':'ignored'});E.SubElement(phys,'max_step_size').text='.005';E.SubElement(phys,'real_time_factor').text='1'
 for file,alias in [('physics','Physics'),('user-commands','UserCommands'),('contact','Contact')]:
  E.SubElement(w,'plugin',{'filename':f'gz-sim-{file}-system','name':f'gz::sim::systems::{alias}'})
 ground=E.SubElement(w,'model',{'name':'floor'});E.SubElement(ground,'static').text='true';E.SubElement(ground,'pose').text='0 0 -.1 0 0 0'
 link=E.SubElement(ground,'link',{'name':'link'});col=E.SubElement(link,'collision',{'name':'collision'});geo=E.SubElement(col,'geometry');box=E.SubElement(geo,'box');E.SubElement(box,'size').text='20 20 .2'
 w.append(model);E.ElementTree(sdf).write(d/'world.sdf')
 sockpath=f'/tmp/teacher_native_audit_{name}_{os.getpid()}.sock'
 requests=[];errors=[];ready=threading.Event()
 def server():
  try:
   with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
    s.bind(sockpath);s.listen(1);s.settimeout(15);ready.set()
    with s.accept()[0] as c:
     c.settimeout(5)
     while True:
      raw=b''
      while len(raw)<512:
       b=c.recv(512-len(raw))
       if not b:return
       raw+=b
      r=struct.unpack('64d',raw);requests.append(r)
      if name=='timeout' and len(requests)==3:
       time.sleep(.35);return
      res=DEFAULT+[0.,float(r[0]>=1.),0.,0.]
      if name=='nan' and len(requests)==3:res[0]=float('nan')
      c.sendall(struct.pack('16d',*res))
      if res[13]:return
  except Exception as exc:errors.append(repr(exc))
 thread=threading.Thread(target=server,daemon=True);thread.start();ready.wait(2)
 env=os.environ.copy();env.update(TEACHER_SOCKET=sockpath,TEACHER_ACTUATOR_LOG=str(d/'actuator.jsonl'),GZ_PARTITION=f'teacher_native_audit_{name}',GZ_IP='127.0.0.1',LIBGL_ALWAYS_SOFTWARE='1',QT_QPA_PLATFORM='offscreen')
 with (d/'gazebo.log').open('w') as log:
  p=subprocess.Popen(['gz','sim','-s','-r',str(d/'world.sdf')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:p.wait(timeout=5 if name in ('timeout','nan') else 10)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 try:os.unlink(sockpath)
 except FileNotFoundError:pass
 rows=[json.loads(l) for l in (d/'actuator.jsonl').read_text().splitlines()] if (d/'actuator.jsonl').exists() else []
 steps=[r for r in rows if r['kind']=='physics_step'];faults=[r for r in rows if r['kind']=='fault'];ownership=[r for r in rows if r['kind']=='model_plugin_ownership']
 pd_error=max((abs(t-max(lo,min(hi,raw))) for r in steps for t,lo,hi,raw in zip(r['tau'],r['tau_lower'],r['tau_upper'],r['pd_raw'])),default=None)
 result=dict(case=name,requests=len(requests),steps=len(steps),faults=faults,ownership=ownership,errors=errors,pd_clamp_error=pd_error,terminated=bool(steps and steps[-1]['terminating']),max_contact_counts=[max((r['contacts'][i] for r in steps),default=0) for i in range(5)],last_t=steps[-1]['t'] if steps else None,log=str(d/'actuator.jsonl'))
 if requests:
  result['policy_dt_max_error']=max((abs(b[0]-a[0]-.02) for a,b in zip(requests,requests[1:])),default=0.)
 if name=='normal':result['passed']=bool(steps and not faults and result['terminated'] and pd_error<1e-10 and result['policy_dt_max_error']<1e-10 and min(result['max_contact_counts'][1:])>0)
 elif name=='duplicate':result['passed']=bool(faults and faults[0]['code']==8 and not requests)
 else:result['passed']=bool(faults and faults[0]['code']==(3 if name=='timeout' else 4) and any(r['mode']==1 for r in steps))
 (d/'request_frames.json').write_text(json.dumps(requests));(d/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
 return result
results=[run(x) for x in ('normal','timeout','nan','duplicate')]
(OUT/'audit.json').write_text(json.dumps({'results':results,'source_sha256':hashlib.sha256((ROOT/'teacher_mode/simulation/teacher_actuator.cpp').read_bytes()).hexdigest(),'library_sha256':hashlib.sha256(LIB.read_bytes()).hexdigest(),'scope':'Actual Gazebo native actuator contract with mock target server; not Teacher motion acceptance','passed':all(r['passed'] for r in results)},indent=2))
