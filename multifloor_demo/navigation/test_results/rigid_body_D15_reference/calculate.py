"""Only two frozen poses and D=1/1.5 in the existing 4x1ms held-PD model."""
from pathlib import Path
import hashlib,importlib.util,json,time
import numpy as np
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
old=ROOT/'navigation/test_results/rigid_body_pd_model'
spec=importlib.util.spec_from_file_location('existing_rigid_body_model',old/'model.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
start=time.monotonic()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
summary_path=ROOT/'simulation/test_results/full19_mechanical_audit/mechanical_summary.json'
summary=json.loads(summary_path.read_text())
snapshot=min(summary['snapshots'],key=lambda s:abs(s['requested_time_s']-856.137))
jtc=snapshot['jtc'];mapped=dict(zip(jtc['joint_names'],jtc['feedback']['positions']))
assert set(mapped)==set(m.NAMES)
poses={'nominal_original_frozen_q':np.load(old/'nominal_mass_matrices.npz')['q'],
       'Full19_feedback_at_failure_nearest_prior_JTC':np.array([mapped[n] for n in m.NAMES])}
urdf=ROOT/'simulation/generated/go2_measured.urdf';model=m.Model(urdf)
P=220.982919;result={}
for name,q in poses.items():
 full,_=model.mass(q)
 fixed=full[6:,6:]
 free=fixed-full[6:,:6]@np.linalg.solve(full[:6,:6],full[:6,6:])
 limits={}
 for bound,M in (('base_fixed',fixed),('force_free_floating_Schur',free)):
  inertia=np.linalg.eigvalsh(M);lower,upper=m.scalar_bounds(float(inertia[0]),P,.01)
  values={}
  for d in (1.,1.5):
   vals=np.linalg.eigvals(m.held_pd_matrix(M,P,d,passive=.01));rho=float(max(abs(vals)))
   values[str(d)]=dict(spectral_radius=rho,unit_circle_margin=1-rho,stable_in_this_frozen_model=rho<1.,
       damping_upper_margin=upper-d,damping_lower_margin=d-lower)
  limits[bound]=dict(minimum_eigen_inertia_kg_m2=float(inertia[0]),PD_sampled_D_bounds=[lower,upper],profiles=values)
 result[name]=dict(mapped_joint_names=m.NAMES,q=q.tolist(),models=limits)
r=dict(scope=__doc__,passed=True,controller_P=P,controller_period_s=.004,physics_substep_s=.001,held_substeps=4,passive_joint_damping=.01,
 poses=result,sources={str(p):sha(p) for p in (old/'model.py',old/'nominal_mass_matrices.npz',summary_path,urdf)},
 actual_bad_pose=dict(original_run='20261002_084329_14d9cc',requested_time_s=snapshot['requested_time_s'],
 jtc_header_ns=jtc['header_ns'],imu_header_ns=snapshot['imu']['header_ns'],feedback_positions_used=True,
 raw_rp=snapshot['imu']['roll_pitch_yaw'][:2],initial_nominal_frame='trunk axes; body orientation rigid-rotation invariant, no gravity/contact model'),
 limitations=['Only the existing frozen rigid-body 18DOF mass/force-free Schur and 4x1ms held explicit-PD model are used.',
 'No foot contact/impact, Coulomb friction, saturation/clamp, controller interpolation, sensor delay, integral action or actual Gazebo integrator guarantee.',
 'Pose range consists of two specific snapshots, not a workspace robust guarantee.',
 'D=1.5 is a numerical reference only; no gains, ROS nodes, physical runs or production source were changed.'],
 production_modified=False,elapsed_math_seconds=time.monotonic()-start)
(OUT/'result.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({'poses':result,'elapsed_math_seconds':r['elapsed_math_seconds'],'receipt_sha256':sha(OUT/'result.json')},indent=2))
