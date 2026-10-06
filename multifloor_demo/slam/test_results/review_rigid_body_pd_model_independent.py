"""Offline independent kinematics/FD mass and held-PD arithmetic review. No ROS."""
from pathlib import Path
import importlib.util,json,hashlib,math,xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
SRC=ROOT/'navigation/test_results/rigid_body_pd_model/model.py'
OUT=Path(__file__).with_name('oct2_rigid_body_PD_model_independent.json')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def vec(e,key='xyz',default='0 0 0'):return np.array([float(v) for v in (e.get(key,default) if e is not None else default).split()])
def skew(a):return np.array([[0.,-a[2],a[1]],[a[2],0.,-a[0]],[-a[1],a[0],0.]])
def rot(a,t):
 a=a/np.linalg.norm(a);s=skew(a);return np.eye(3)+math.sin(t)*s+(1-math.cos(t))*s@s
def org(e):
 v=vec(e,'rpy');t=np.eye(4);t[:3,:3]=rot(np.array([0.,0.,1.]),v[2])@rot(np.array([0.,1.,0.]),v[1])@rot(np.array([1.,0.,0.]),v[0]);t[:3,3]=vec(e);return t
before=sha(SRC);spec=importlib.util.spec_from_file_location('nav_rigid_model_readonly',SRC);nav=importlib.util.module_from_spec(spec);spec.loader.exec_module(nav)
result=json.loads(SRC.with_name('result.json').read_text());data=np.load(SRC.with_name('nominal_mass_matrices.npz'));q=data['q'];model=nav.Model(nav.URDF);mass,components=model.mass(q);names=nav.NAMES
xml=ET.parse(nav.URDF).getroot();links={e.get('name'):e for e in xml.findall('link')};joints=list(xml.findall('joint'));children={}
for e in joints:children.setdefault(e.find('parent').get('link'),[]).append(e)
root=(set(links)-{e.find('child').get('link') for e in joints}).pop()
def fk2(q,base=None):
 frames={root:np.eye(4) if base is None else base}
 def walk(parent):
  for e in children.get(parent,[]):
   t=frames[parent]@org(e.find('origin'));child=e.find('child').get('link')
   if e.get('type')!='fixed':
    assert e.get('type') in ('revolute','continuous');r=np.eye(4);r[:3,:3]=rot(vec(e.find('axis'),default='1 0 0'),q[names.index(e.get('name'))]);t=t@r
   frames[child]=t;walk(child)
 walk(root);return {n:t@org(links[n].find('inertial/origin')) for n,t in frames.items() if links[n].find('inertial') is not None}
frames=fk2(q);eps=1e-6;J={n:(np.zeros((3,18)),np.zeros((3,18))) for n in frames}
for k in range(18):
 qp=q.copy();qm=q.copy();bp=np.eye(4);bm=np.eye(4)
 if k<3:bp[k,3]=eps;bm[k,3]=-eps
 elif k<6:
  axis=np.eye(3)[k-3];bp[:3,:3]=rot(axis,eps);bm[:3,:3]=rot(axis,-eps)
 else:qp[k-6]+=eps;qm[k-6]-=eps
 plus,minus=fk2(qp,bp),fk2(qm,bm)
 for n,f in frames.items():
  jv,jw=J[n];jv[:,k]=(plus[n][:3,3]-minus[n][:3,3])/(2*eps);W=((plus[n][:3,:3]-minus[n][:3,:3])/(2*eps))@f[:3,:3].T;jw[:,k]=[W[2,1],W[0,2],W[1,0]]
max_jv=max(np.max(np.abs(J[c['link']][0]-c['Jv'])) for c in components);max_jw=max(np.max(np.abs(J[c['link']][1]-c['Jw'])) for c in components)
fdmass=np.zeros((18,18));inertial_checks=[];total=0.
for n,f in frames.items():
 e=links[n].find('inertial');m=float(e.find('mass').get('value'));i=e.find('inertia');I=np.array([[float(i.get('ixx')),float(i.get('ixy')),float(i.get('ixz'))],[float(i.get('ixy')),float(i.get('iyy')),float(i.get('iyz'))],[float(i.get('ixz')),float(i.get('iyz')),float(i.get('izz'))]]);R=f[:3,:3];v,w=J[n];fdmass+=m*v.T@v+w.T@(R@I@R.T)@w;total+=m;inertial_checks.append(m>0 and np.min(np.linalg.eigvalsh(I))>0)
fixed=mass[6:,6:];floating=fixed-mass[6:,:6]@np.linalg.solve(mass[:6,:6],mass[:6,6:]);held=[]
for label,M in [('fixed12',fixed),('floating_Schur12',floating)]:
 for passive in [0.,.01]:
  for p,d in [(100.,1.),(220.982919,3.360637),(220.982919,1.)]:
   F=np.zeros((24,24))
   for k in range(24):
    state=np.eye(24)[:,k];x,v=state[:12].copy(),state[12:].copy();u=-p*x-d*v
    for _ in range(4):v+=.001*np.linalg.solve(M,u-passive*v);x+=.001*v
    F[:,k]=np.r_[x,v]
   f=nav.held_pd_matrix(M,p,d,passive);eigs=np.linalg.eigvals(F);rho=float(max(abs(eigs)));pole=eigs[np.argmax(abs(eigs))]
   t=.004;zoh=np.block([[np.eye(12)-.5*t*t*p*np.linalg.inv(M),t*np.eye(12)-.5*t*t*d*np.linalg.inv(M)],[-t*p*np.linalg.inv(M),np.eye(12)-t*d*np.linalg.inv(M)]])
   held.append(dict(model=label,passive=passive,P=p,D=d,step_simulation_vs_closed_form_error=float(np.max(np.abs(F-f))),spectral_radius=rho,extreme_pole=[float(pole.real),float(pole.imag)],stable_model=rho<1,exact_ZOH_no_passive_spectral_radius=float(max(abs(np.linalg.eigvals(zoh))))))
nom=None;line_no=None
with (nav.RUN/'joint_stop_adapter.jsonl').open() as f:
 for n,line in enumerate(f,1):
  r=json.loads(line)
  if r['kind']=='nominal_frozen':nom=r;line_no=n;break
actual_names=json.loads((ROOT/'slam/test_results/oct2_classic_PD_startup_failure_mechanical_independent_final.json').read_text())['original_message_names']['jtc']
checks={'source_same_after_review':sha(SRC)==before,'actual_URDF_SHA_c1d454':sha(nav.URDF)=='c1d454f479246a095e9f90597213674c4abc28a7661c0573dca78e43bd0e7c7f','nominal_matches_actual_adapter_first_frozen_record':line_no==result['nominal_source']['line'] and nom==result['nominal_source']['record'] and np.array_equal(q,nom['positions']),'canonical_joint_names_match_actual_JTC':names==actual_names,'all_massive_link_inertias_positive':all(inertial_checks),'independent_FK_translation_Jacobians_FD':max_jv<1e-8,'independent_FK_rotation_Jacobians_FD':max_jw<1e-8,'independent_FD_mass_matches_analytic':np.max(np.abs(fdmass-mass))<1e-7,'full18mass_symmetric_positive':np.max(abs(mass-mass.T))<1e-12 and np.min(np.linalg.eigvalsh(mass))>0,'actual_mass_translation_block':np.max(abs(mass[:3,:3]-total*np.eye(3)))<1e-12,'saved_mass_matrices_match_recomputed':np.array_equal(data['full18'],mass) and np.array_equal(data['base_fixed12'],fixed) and np.array_equal(data['floating_Schur12'],floating),'held4_actual_step_vs_closed_form':all(x['step_simulation_vs_closed_form_error']<1e-10 for x in held)}
# Final published model: independently verify actual original 100s pose as well.
actual_q=np.array(result['actual_100s_pose']['mapped_q']);actualM,actualC=model.mass(actual_q);actualframes=fk2(actual_q);actualJ={n:(np.zeros((3,18)),np.zeros((3,18))) for n in actualframes}
for k in range(18):
 qp=actual_q.copy();qm=actual_q.copy();bp=np.eye(4);bm=np.eye(4)
 if k<3:bp[k,3]=eps;bm[k,3]=-eps
 elif k<6:axis=np.eye(3)[k-3];bp[:3,:3]=rot(axis,eps);bm[:3,:3]=rot(axis,-eps)
 else:qp[k-6]+=eps;qm[k-6]-=eps
 plus,minus=fk2(qp,bp),fk2(qm,bm)
 for n,f in actualframes.items():
  v,w=actualJ[n];v[:,k]=(plus[n][:3,3]-minus[n][:3,3])/(2*eps);W=((plus[n][:3,:3]-minus[n][:3,:3])/(2*eps))@f[:3,:3].T;w[:,k]=[W[2,1],W[0,2],W[1,0]]
actualFDmass=np.zeros((18,18))
for c in actualC:
 v,w=actualJ[c['link']];actualFDmass+=c['mass']*v.T@v+w.T@c['world_I']@w
actualJerr=max(max(np.max(abs(actualJ[c['link']][0]-c['Jv'])),np.max(abs(actualJ[c['link']][1]-c['Jw']))) for c in actualC)
actualdata=np.load(SRC.with_name('actual_100s_mass_matrices.npz'));actual_fixed=actualM[6:,6:];actual_floating=actual_fixed-actualM[6:,:6]@np.linalg.solve(actualM[:6,:6],actualM[:6,6:]);actual_d1=[]
for label,M in [('fixed_base',actual_fixed),('floating_base_Schur',actual_floating)]:
 F=np.zeros((24,24));p,d=220.982919,1.
 for k in range(24):
  state=np.eye(24)[:,k];x,v=state[:12].copy(),state[12:].copy();u=-p*x-d*v
  for _ in range(4):v+=.001*np.linalg.solve(M,u-.01*v);x+=.001*v
  F[:,k]=np.r_[x,v]
 rho=float(max(abs(np.linalg.eigvals(F))));claimed=result['actual_100s_pose'][label]['explicit_comparison_profiles']['only_lower_D_221_D1']['models']['semi_implicit_passive_0.01']['spectral_radius']
 actual_d1.append({'model':label,'P':p,'D':d,'passive':.01,'explicit_step_spectral_radius':rho,'reported_radius_absolute_difference':abs(rho-claimed)})
verifiedshort=json.loads((ROOT/'simulation/test_results/classic_pd_startup_audit/native_100s_short_frames.json').read_text());original=next(r for r in verifiedshort if r['kind']=='jtc' and r['header']['header_stamp_ns']==100000000000);mapping=dict(zip(original['joint_names'],original['values']['feedback.positions']))
checks.update(final_model_source_exact=before=='7e40cfa6a573d0de4f88f1be23c603ccf1fa7ca697b259c8e20e574a2a2117ce',published_result_exact=sha(SRC.with_name('result.json'))=='f96b26d66826feea2e287626db15fda3debc793d5f49bc68602545177ba0b8c9',actual100s_q_uses_actual_named_feedback=np.array_equal(actual_q,[mapping[n] for n in names]),actual100s_independent_FD_J=actualJerr<1e-8,actual100s_independent_FD_M=np.max(abs(actualFDmass-actualM))<1e-7,actual100s_saved_matrix_exact=np.array_equal(actualdata['full18'],actualM),actual100s_explicit_fourstep_radius_matches_claim=all(r['reported_radius_absolute_difference']<1e-12 for r in actual_d1))
exact_zoh=[]
for pose,Ms in [('nominal',{'fixed_base':fixed,'floating_base_Schur':floating}),('actual100s',{'fixed_base':actual_fixed,'floating_base_Schur':actual_floating})]:
 for label,M in Ms.items():
  for profile,P,D in [('original_100_D1',100.,1.),('failed_221_D3p36',220.982919,3.360637),('only_lower_D_221_D1',220.982919,1.)]:
   F=np.zeros((24,24));dt=.004
   for k in range(24):
    state=np.eye(24)[:,k];x,v=state[:12],state[12:];u=-P*x-D*v;a=np.linalg.solve(M,u);F[:,k]=np.r_[x+dt*v+.5*dt*dt*a,v+dt*a]
   rho=float(max(abs(np.linalg.eigvals(F))));rr=result[label] if pose=='nominal' else result['actual_100s_pose'][label];claimed=rr['explicit_comparison_profiles'][profile]['models']['exact_inertial_ZOH_no_passive']['spectral_radius']
   exact_zoh.append(dict(pose=pose,model=label,P=P,D=D,explicit_constant_force_ZOH_spectral_radius=rho,reported_radius_absolute_difference=abs(rho-claimed)))
checks['independent_constant_force_4ms_ZOH_all12_radius_matches']=all(r['reported_radius_absolute_difference']<1e-12 for r in exact_zoh)
checks={k:bool(v) for k,v in checks.items()}
report={'scope':__doc__,'source_model_sha256':before,'source_result_sha256':sha(SRC.with_name('result.json')),'checks':checks,'all_checks_true':all(checks.values()),'nominal_actual_q':q.tolist(),'joint_names':names,'total_URDF_mass_kg':total,'massive_links':len(frames),'FD_step':eps,'FD_Jv_max_error':float(max_jv),'FD_Jw_max_error':float(max_jw),'FD_mass_max_error':float(np.max(abs(fdmass-mass))),'floating_joint_min_eigen_inertia':float(np.min(np.linalg.eigvalsh(floating))),'held_models':held,'actual100s_FD_J_max_error':float(actualJerr),'actual100s_FD_mass_max_error':float(np.max(abs(actualFDmass-actualM))),'actual100s_D1_explicit_steps':actual_d1,'constant_force_exact_ZOH_comparisons':exact_zoh,'limitations':['Base fixed and force-free floating Schur are alternative no-contact assumptions; neither is the actual foot-contact plant.','Semiimplicit 1ms physics integrator with 4ms held force is an explicit model, not proof of Gazebo/DART update internals.','No contact/friction/impact/velocity or effort clipping/gravity linearization/I term/target interpolation is included.','Model spectral radii are arithmetic evidence, not unique explanation of classic startup failure or validation/adoption of a physical gain.','Prepared gain source is not an actual live RPC readback: the original classic startup did not reach that gate.']}
OUT.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'checks':checks,'sha256':sha(OUT),'FD_Jv':max_jv,'FD_Jw':max_jw,'FD_mass':float(np.max(abs(fdmass-mass))),'rho':[{k:x[k] for k in ('model','passive','P','D','spectral_radius')} for x in held]},indent=2));assert all(checks.values())
