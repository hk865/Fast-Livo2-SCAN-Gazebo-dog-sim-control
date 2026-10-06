#!/usr/bin/env python3
"""Read-only first-point A-v2/full14 motion and actual stance-FK diagnostics.

No ROS node, control or GT feedback. Contact velocity assumes no stance-foot
slip and is evaluated against independently measured GT; it is not body truth.
"""
import ast,hashlib,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
A=ROOT/'simulation/test_results/20261001_balance_first8_a_disabled_v2'
FULL=ROOT/'runs/20261001_174341_6cd895'
OUT=ROOT/'navigation/test_results/first8_disabled_a_v2_navigation_review.json'
sys.path.insert(0,str(ROOT/'simulation'))
from joint_stop_core import JointGeometry

def rows(path):
    for line in path.open():
        try:yield json.loads(line)
        except json.JSONDecodeError:pass

physics_source=ROOT/'simulation/test_results/audit_full14_physics.py'
physics_ast=ast.parse(physics_source.read_text())
physical_functions=[n for n in physics_ast.body if isinstance(n,ast.FunctionDef) and n.name in ('pose','category','window')]

def profile(folder,truth,start,end,navrows):
    truth=sorted({r['stamp']:r for r in truth}.values(),key=lambda r:r['stamp'])
    times=np.array([r['stamp'] for r in truth]);positions=np.array([r['p'] for r in truth])
    yaw=np.unwrap([math.atan2(Rotation.from_quat(r['q']).as_matrix()[1,0],Rotation.from_quat(r['q']).as_matrix()[0,0]) for r in truth])
    commands={};raw=[]
    for r in rows(folder/'joint_stop_adapter.jsonl'):
        if r['kind']=='actual_champ_command':commands[r['sim']]=r['value']
        elif r['kind']=='raw_target' and start<=r['sim']<=end:raw.append(r)
    ct=np.array(sorted(commands));cv=np.array([commands[t] for t in ct])
    scope=dict(np=np,times=times,poses=np.column_stack([positions,yaw]),ct=ct,cv=cv)
    exec(compile(ast.Module(body=physical_functions,type_ignores=[]),str(physics_source),'exec'),scope)
    physical=scope['window'](start,end)
    rt=np.array([r['sim'] for r in raw]);rw=np.array([r['wall'] for r in raw])
    cadence=dict(observation='Adapter receipt times of actual raw CHAMP joint targets, not exact timer invocation timestamps',count=len(raw),
        sim_interval=[float(rt[0]),float(rt[-1])],wall_interval=[float(rw[0]),float(rw[-1])],
        observed_real_time_factor=float((rt[-1]-rt[0])/(rw[-1]-rw[0])),
        observed_wall_message_rate_hz=float((len(raw)-1)/(rw[-1]-rw[0])),
        wall_step_p50_p95_p99_s=np.quantile(np.diff(rw),[.5,.95,.99]).tolist(),
        sim_step_p50_p95_p99_s=np.quantile(np.diff(rt),[.5,.95,.99]).tolist(),
        repeated_sim_stamps=int(np.count_nonzero(np.diff(rt)==0)))
    phase=[];last=None
    for r in navrows:
        n=r['status']
        if n.get('waypoint_index')!=0 or not start<=r['sim']<=end:continue
        s=n.get('steering');key=(n['alignment_phase'],s.get('trajectory_id') if s else None,n['tilt_hold'])
        if key!=last:
            phase.append(dict(sim=r['sim'],phase=n['alignment_phase'],command=n['command'],pose=n['pose'],
                steering=s,raw_tilt=n['raw_imu_tilt_rad'],tilt_stops=n['tilt_stops']))
            last=key
    return dict(physical=physical,cadence=cadence,actual_NAV_phase_edges=phase),ct,cv,times,positions,yaw

def main():
    result=json.loads((A/'first_eight_result.json').read_text())
    a_start=next(r['sim'] for r in result['statuses'] if r['status'].get('request_id')==result['statuses'][-1]['status'].get('request_id')
        and r['status'].get('state')=='running' and r['status'].get('waypoint_index')==0)
    a_end=result['nav_confirmations'][0]['received_sim']
    a,ct,cv,times,positions,yaw=profile(A,result['truth'],a_start,a_end,result['statuses'])
    feedback=list(rows(FULL/'feedback_navigation.jsonl'))
    full_truth=[r for r in rows(FULL/'pose_audit.jsonl') if r.get('source')=='truth']
    navrows=[dict(sim=r['sim_time'],status=r['navigation']) for r in feedback if r.get('navigation') and r['navigation'].get('request_id')]
    f_start=navrows[0]['sim'];f_end=next(r['sim'] for r in navrows if r['status']['waypoint_index']==1)
    f,*_=profile(FULL,full_truth,f_start,f_end,navrows)
    # Compile only the actual staged FK/Jacobian class, with no ROS node imports.
    source=A/'staging/body_stabilizer_node.py';tree=ast.parse(source.read_text())
    klass=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Kinematics')
    scope=dict(JointGeometry=JointGeometry,np=np,Rotation=Rotation)
    exec(compile(ast.Module(body=[klass],type_ignores=[]),str(source),'exec'),scope)
    geometry=scope['Kinematics']((ROOT/'simulation/generated/go2.urdf').read_text(),.30,0.)
    imu=[];joints=[];contacts={leg:[] for leg in ('lf','rf','lh','rh')}
    for r in rows(A/'balance_inputs.jsonl'):
        ns=r['stamp_ns']
        if ns is None:continue
        t=ns/1e9
        if t>a_end+.20:break
        if t<a_start-.1:continue
        if r['kind']=='imu':imu.append((t,r['data']))
        elif r['kind']=='joints':joints.append((t,r['data']))
        elif r['kind']=='contact':contacts[r['data']['leg']].append((t,r['data']))
    imu=sorted({t:d for t,d in imu}.items());it=np.array([t for t,d in imu])
    candidates=[];truth_velocities=[];counts=[];gaps=[];scatter=[];sample_modes=[];plane_valid=0
    contact_times={l:np.array([t for t,d in contacts[l]]) for l in contacts}
    for t,j in joints:
        if not a_start<=t<=a_end:continue
        ii=int(np.argmin(abs(it-t)))
        if abs(it[ii]-t)>.005:continue
        mask=[]
        for leg in contacts:
            k=int(np.searchsorted(contact_times[leg],t,side='right'))-1
            mask.append(k>=0 and 0<=t-contact_times[leg][k]<=.040001 and contacts[leg][k][1]['nonempty'])
        ids=np.flatnonzero(mask);counts.append(len(ids))
        if len(ids)<2:continue
        ji=[j['names'].index(n) for n in geometry.names]
        feet,vf=geometry.measured(np.array(j['positions'])[ji],np.array(j['velocities'])[ji])
        im=imu[ii][1];omega=np.array(im['gyro']);R=Rotation.from_quat(im['q']).as_matrix()
        velocities=-(vf[ids]+np.cross(omega,feet[ids]));estimate=np.median(velocities,axis=0)
        k=int(np.searchsorted(times,t))-1
        if k<0 or k+1>=len(times) or times[k+1]-times[k]>.15:continue
        truth_velocity=R.T@((positions[k+1]-positions[k])/(times[k+1]-times[k]))
        if len(ids)>=3:
            _,s,vh=np.linalg.svd(feet[ids]-feet[ids].mean(axis=0),full_matrices=False)
            if len(s)>=2 and s[1]>=.025 and abs((feet[ids]-feet[ids].mean(axis=0))@vh[-1]).max()<=.008:plane_valid+=1
        candidates.append(estimate);truth_velocities.append(truth_velocity);gaps.append(abs(it[ii]-t))
        scatter.append(float(np.max(np.linalg.norm(velocities-estimate,axis=1))))
        ci=np.searchsorted(ct,t,side='right')-1
        sample_modes.append('walk' if abs(cv[ci,0])>1e-7 else 'turn' if abs(cv[ci,5])>1e-7 else 'zero')
    c=np.array(candidates);v=np.array(truth_velocities);m=np.array(sample_modes)
    diagnostics=dict(scope='Only fresh measured contact feet; no-slip median FK+gyro is a hypothesis evaluated by GT, never a control source',
        actual_joint_samples=len(joints),actual_contact_samples={l:len(x) for l,x in contacts.items()},
        contact_count_histogram={str(i):int(np.count_nonzero(np.array(counts)==i)) for i in range(5)},
        actual_stance_velocity_pairs=len(c),max_joint_IMU_stamp_gap_s=max(gaps,default=None),
        geometric_plane_valid_samples=plane_valid,candidate_scatter_p50_p95_m_s=np.quantile(scatter,[.5,.95]).tolist(),
        estimate_minus_GT_component_RMSE_m_s=np.sqrt(np.mean((c-v)**2,axis=0)).tolist(),
        by_actual_CHAMP_mode={kind:dict(samples=int(sum(m==kind)),
            estimate_forward_mean_m_s=float(c[m==kind,0].mean()),
            independent_GT_forward_mean_m_s=float(v[m==kind,0].mean()),
            forward_error_rmse_m_s=float(np.sqrt(np.mean((c[m==kind,0]-v[m==kind,0])**2)))) for kind in ('walk','turn','zero') if any(m==kind)})
    report=dict(scope=__doc__,component_original_passed=result['passed'],component_original_failures=result['missing_acceptance'],
        actual_eight_NAV_arrivals=result['nav_confirmations'],active_max_raw_tilt=result['active_max_imu_tilt'],
        active_bridge_holds=result['active_bridge_holds'],lifetime_bridge_initial_hold_is_not_active_failure=True,
        original_component_result_SHA256=hashlib.sha256((A/'first_eight_result.json').read_bytes()).hexdigest(),
        first_point_A=a,first_point_full14=f,actual_contact_velocity_diagnostic=diagnostics,
        fixed_sensor_calibration=result['heading_alignment'],CHAMP_source_time_contract=dict(wall_timer_rate_hz=200,
            phase_time='clock_.now(), actual ROS simulation time',joint_reference_horizon_sim_s=1/60,
            source_SHA256=hashlib.sha256((ROOT.parent/'go2_sim_control/src/unitree_go2_ros2/champ_base/src/quadruped_controller.cpp').read_bytes()).hexdigest()),
        limitations=['Completed disabled component is still FAIL because of six actual truth-bracket gaps and one active IMU tilt hold.',
            'This independent run has a different measured frozen scene yaw and initial physical state; elapsed differences cannot be attributed to enablement before matched A/B.',
            'Reported raw-target cadence measures adapter receipt times; it does not prove exact timer scheduling or drop counts.',
            'Contact-FK velocity depends on stance-foot no-slip, and measured contact/gyro/Jacobian evidence must not be relabeled ground truth.'])
    OUT.write_text(json.dumps(report,indent=2)+'\n')
    print('saved',OUT)
    for name,data in [('A',a),('full14',f)]:print(name,'seconds',data['physical']['seconds'],'totals',data['physical']['totals'],'cadence',data['cadence'])
    print('contact',diagnostics)

if __name__=='__main__':main()
