#!/usr/bin/env python3
"""Read-only frozen-run diagnosis. No navigation/control or receipt rewrites."""
import argparse,collections,hashlib,importlib.util,json,math,os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
RUNS=ROOT/'runs'
NAMES=('20261005_125808_closed_loop_cascade_actual_ramp12_up_r1_b318',
       '20261005_144028_closed_loop_cascade_registered_ramp12_turn06_r2_55f6')

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def wrap(value):return np.arctan2(np.sin(value),np.cos(value))
def rotation(q):
    w,x,y,z=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def records(path,hashes):
    h=hashlib.sha256()
    with path.open('rb')as source:
        for line in source:
            h.update(line)
            if line.strip():yield json.loads(line)
    hashes[str(path)]=h.hexdigest()
def finite(value):return value is not None and math.isfinite(float(value))
def stats(values):
    v=np.array(values,dtype=float);v=v[np.isfinite(v)]
    return None if not len(v)else dict(count=len(v),min=float(v.min()),median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(v.max()))

def analyze(run):
    # Final analysis never freezes a partial observation while a writer is live.
    if not (run/'runtime_manifest.json').exists()or not(run/'worker_result.json').exists():
        raise RuntimeError('Wait for root completion notification and final manifests: '+str(run))
    hashes={};small={}
    for name in ('navigation_scope.json','navigation_anchor.json','navigation_request.json','navigation_status.json',
                 'worker_result.json','runtime_manifest.json','source_manifest.json','navigation_source_snapshots.json',
                 'asset_manifest.json','sensor_contract.json','navigation_scene_axis_registration.json',
                 'navigation_registration_writer_receipt.json'):
        path=run/name
        if path.exists():hashes[str(path)]=sha(path);small[name]=json.loads(path.read_text())
    for name in ('world.sdf','terrain_target_manifest.json'):
        hashes[str(run/name)]=sha(run/name)
    archive_mismatch=[]
    for source,row in small['navigation_source_snapshots.json'].items():
        path=Path(row['snapshot'])
        if not path.is_relative_to(run/'sources')or not path.exists()or sha(path)!=row['sha256']:
            archive_mismatch.append(source)
    telemetry=[];native=[];poses=[];status=[];pid=[];paths=[];contract=None;graph=None
    native_counts=collections.Counter();max_tau=max_qd=0.;first_body=None;first_fault=None
    for row in records(run/'actuator.jsonl',hashes):
        if row.get('kind')=='actuator_contract':contract=row;continue
        if row.get('kind')=='controller_graph':graph=row;continue
        if row.get('kind')!='physics_step':continue
        q=row['quaternion_wxyz'];r=rotation(q);p=np.array(row['position']);offset=np.array(contract['base_com_offset_body_m'])
        com=p+r@offset;v=np.array(row['body_lin_vel_com']);omega=np.array(row['body_ang_vel'])
        roll=math.atan2(r[2,1],r[2,2]);pitch=math.asin(float(np.clip(-r[2,0],-1,1)));yaw=math.atan2(r[1,0],r[0,0])
        t=float(row['t'])-float(row['dt']);contacts=row['contacts']
        native.append([t,*p,*com,*v,*omega,roll,pitch,yaw,float(contacts[0])])
        max_tau=max(max_tau,max(abs(float(x))for x in row['tau']));max_qd=max(max_qd,max(abs(float(x))for x in row['qd']))
        if t>=.1 and contacts[0]and first_body is None:first_body=dict(t=t,position=p.tolist(),pairs=row['contact_pairs'])
        if row['fault'] and first_fault is None:first_fault=dict(t=t,fault=row['fault'],terminating=row.get('terminating'))
        for pair in row.get('contact_pairs',[]):
            text=canonical(pair)
            if 'foot'not in text:continue
            for terrain in ('ramp_12','floor_1','floor_2'):
                if terrain in text:native_counts[terrain]+=1
    for row in records(run/'telemetry.jsonl',hashes):
        env=row.get('closed_loop_read_evidence',{}).get('actual_original_envelope')or{}
        source=env.get('controller_source')or{}
        telemetry.append([row['world_sim_time'],*row['command'],*row['requested'],*row['body_lin_vel'],*row['body_ang_vel'],
            row['body_clearance'],float(row['command_expired']),row['inference_ms'],source.get('cascade_sequence',-1)])
    for row in records(run/'navigation_slam_poses.jsonl',hashes):
        q=row['quaternion'];r=rotation([q[3],*q[:3]])
        poses.append([row['stamp_ns']/1e9,*row['position'],math.atan2(r[1,0],r[0,0])])
    for row in records(run/'navigation_status.jsonl',hashes):
        c=row.get('cascade_parking')or{};steer=row.get('steering')or{};gate=row.get('teacher_transition')or{}
        status.append(dict(t=row['ros_sim_time'],state=row['state'],waypoint=row['waypoint_index'],
            gate_phase=gate.get('phase'),gate_heading=row.get('locked_heading'),SCAN_lookahead_heading=steer.get('heading'),
            CORE_mode=c.get('mode'),CORE_reference_yaw=c.get('reference_yaw_rad'),CORE_error_yaw=c.get('error_yaw_rad'),
            command=row['command'],pose=row.get('pose'),CORE_progress=c.get('progress_m'),
            path_id=c.get('path_id'),message=row.get('message'),navigation_ground_truth_used=row['navigation_ground_truth_used']))
    for row in records(run/'navigation_pid_history.jsonl',hashes):
        c=row['cascade']
        pid.append(dict(t=row['compute_ros_clock_ns']/1e9,pose_t=row['source_pose_stamp_ns']/1e9,
            sequence=row['sequence'],updated=c.get('controller_updated',False),waypoint=row['waypoint_index'],
            CORE_mode=c['mode'],CORE_reference_yaw=c.get('reference_yaw_rad'),CORE_error_yaw=c.get('error_yaw_rad'),
            desired=row['desired_body_command'],command=row['command_after_slew'],
            CORE_filtered_actual=c.get('velocity_PI',{}).get('filtered_actual_body'),
            CORE_rate_I=c.get('velocity_PI',{}).get('I',[None]*3)[2],path_id=c.get('path_id'),
            control_pose=row['control_pose'],fixed_goal=c.get('fixed_goal'),path_sha=c.get('path_sha256')))
    for row in records(run/'navigation_cascade_paths.jsonl',hashes):
        points=np.asarray(row['points_xyz'],dtype=float)
        expected=row['points_float64_sha256'];actual=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()
        paths.append(dict(path_id=row['path_id'],trajectory_id=row['trajectory_id'],stamp_ns=row['stamp_ns'],
            points=points.tolist(),actual_points_sha256=actual,expected_points_sha256=expected,sha_matches=actual==expected))
    native=np.array(native);telemetry=np.array(telemetry);poses=np.array(poses)
    aligned=[row for row in status if row['state']=='running'and row['gate_phase']=='align'and finite(row['gate_heading'])and finite(row['CORE_reference_yaw'])]
    differences=[float(abs(wrap(row['gate_heading']-row['CORE_reference_yaw'])))for row in aligned]
    opposite=[row for row in aligned if finite(row['CORE_error_yaw'])and abs(row['CORE_error_yaw'])>.001
        and np.sign(wrap(row['gate_heading']-(row['CORE_reference_yaw']-row['CORE_error_yaw'])))!=np.sign(row['CORE_error_yaw'])]
    first_failed=next((row for row in status if row['state']=='failed'),None)
    summary=dict(run=str(run),navigation_ground_truth_used=False,native_truth_use='offline execution and registration error diagnosis only',
        status='diagnostic_only',source_archive_mismatch=archive_mismatch,input_hashes=hashes,
        original_navigation_state=small['navigation_status.json']['state'],original_worker_fault=small['worker_result.json']['fault'],
        original_worker_samples=small['worker_result.json']['samples'],original_runtime_manifest=small['runtime_manifest.json'],
        original_arrivals=small['navigation_status.json']['region_arrivals'],request=small['navigation_request.json'],
        anchor={k:v for k,v in small['navigation_anchor.json'].items()if k!='scene_axis_registration'},
        native=dict(samples=len(native),first_effective_time=float(native[0,0]),last_effective_time=float(native[-1,0]),
            max_COM_x=float(native[:,4].max()),max_origin_x=float(native[:,1].max()),
            max_lateral_abs_from_physical_ramp_center_m=float(abs(native[:,2]-2).max()),
            min_clearance=float(telemetry[:,13].min()),max_roll_pitch=float(abs(native[:,13:15]).max()),
            maximum_force_command_Nm=max_tau,maximum_qd_radps=max_qd,first_body_contact=first_body,first_native_fault=first_fault,
            raw_named_foot_contact_pair_counts=dict(native_counts),
            complete_12m_seam_crossing='failed'if native[:,4].max()<14.25 else'unverified_pending_full_contact_receipt',
            torque_is_command_feed_to_physics_not_measured_motor_torque=True),
        command=dict(max_abs_actor_vx=float(abs(telemetry[:,1]).max()),max_abs_actor_vy=float(abs(telemetry[:,2]).max()),
            max_abs_actor_wz=float(abs(telemetry[:,3]).max()),expired_samples=int(telemetry[:,14].sum()),
            CPU_inference_ms=stats(telemetry[:,15])),
        heading_conflict=dict(align_status_samples=len(aligned),locked_gate_vs_CORE_reference_rad=stats(differences),
            larger_than_original_drive_gate_point2rad_count=sum(x>.2 for x in differences),
            opposite_remaining_yaw_errors_count=len(opposite),
            diagnostic_definition='parent locked desired minus actual SLAM yaw vs CORE local-tangent desired minus same yaw; not a new acceptance gate'),
        first_navigation_failed=first_failed,paths=[{k:v for k,v in row.items()if k!='points'}for row in paths],
        original_receipts_unchanged=True,formal_acceptance_not_recomputed=True)
    if 'navigation_scene_axis_registration.json'in small:
        registration=small['navigation_scene_axis_registration.json'];heading=registration['heading_receipt']
        snapshots=small['navigation_source_snapshots.json'];entry=next(row for source,row in snapshots.items()if Path(source).name=='heading_alignment.py')
        spec=importlib.util.spec_from_file_location('read_only_registered_heading_replay',entry['snapshot']);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        replay=mod.calibrate_scene_heading(registration['exact_paired_sample_records'],
            imu_reference_world_quaternion=heading['imu_reference_world_quaternion'],body_imu_quaternion=heading['body_imu_quaternion'],
            reference_description=heading['imu_reference_description'])
        anchor=small['navigation_anchor.json'];profile=small['navigation_scope.json']['profile'];origin=np.array(anchor['origin']);c,s=np.cos(heading['yaw_camera_init_from_world']),np.sin(heading['yaw_camera_init_from_world'])
        mapped=[]
        for row in profile['route_world_points']:
            p=np.array(row['xyz'])-np.array([*profile['spawn'][:2],profile['scene_initial_support_z']]);mapped.append(origin+np.array([[c,-s,0],[s,c,0],[0,0,1]])@p)
        difference=float(np.max(abs(np.array(mapped)-np.array([goal['center']for goal in small['navigation_request.json']['goals']]))))
        pairs=run/'nav_registration_pairs.jsonl';hashes[str(pairs)]=sha(pairs)
        t=anchor['pose_stamp_ns']/1e9;i=np.searchsorted(native[:,0],t,side='right')-1
        body_q=registration['exact_paired_sample_records'][-1]['slam_body_quaternion'];rs=rotation([body_q[3],*body_q[:3]])
        # Independent one-time alignment is used only to display measurement error.
        actual_axis=float(wrap(math.atan2(rs[1,0],rs[0,0])-native[i,15]))
        summary['registration']=dict(replayed_heading=replay,heading_receipt_exact_matches=canonical(replay)==canonical(heading),
            route_mapping_max_error_m=difference,pair_archive_SHA_matches=sha(pairs)==registration['paired_sources_expected_file_sha256'],
            pair_count=registration['paired_sources_expected_records'],known_spawn_prior_uncertainty_m=.15,
            offline_initial_native_effective_stamp_s=float(native[i,0]),offline_causal_gap_s=float(t-native[i,0]),
            offline_actual_camera_init_world_yaw=actual_axis,frozen_sensor_registered_yaw=heading['yaw_camera_init_from_world'],
            offline_axis_error_rad=float(wrap(heading['yaw_camera_init_from_world']-actual_axis)),
            frame_height_definition='SLAM initial body origin + support elevation difference; no absolute Gazebo Z in goals',
            absolute_translation_only_known_spawn_prior=True)
    return summary,dict(native=native,telemetry=telemetry,poses=poses,status=status,pid=pid,paths=paths)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'final');a=p.parse_args()
    if a.output.exists():raise RuntimeError('Refuse overwriting any previous diagnosis')
    results=[analyze(RUNS/name)for name in NAMES]
    a.output.mkdir(parents=True)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.grid':True,'figure.dpi':130})
    summary={'schema':'registered_ramp12_read_only_diagnosis/v1','status':'diagnostic_only',
        'source_script_sha256':sha(__file__),'scope':'Actual SLAM/SCAN run comparison; native truth is offline only',
        'runs':[row[0]for row in results],'no_simulation_or_inference_started':True,'old_receipts_never_written':True}
    fig,ax=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for (record,data),color in zip(results,('tab:orange','tab:blue')):
        label=Path(record['run']).name[-4:];n=data['native'];s=data['poses']
        ax[0,0].plot(n[:,1],n[:,2],color=color,label=label+' native offline')
        ax[0,1].plot(s[:,1],s[:,2],color=color,label=label+' raw SLAM')
        goals=np.array([goal['center']for goal in record['request']['goals']]);route=np.vstack((record['anchor']['origin'],goals))
        ax[0,1].plot(route[:,0],route[:,1],':',color=color,label=label+' requested prior')
        ax[1,0].plot(n[:,0],n[:,1],color=color,label=label+' base origin x')
        ax[1,1].plot(n[:,0],n[:,3],color=color,label=label+' base origin z')
    ax[0,0].plot([2,14,14,2,2],[1,1,3,3,1],'k:',label='original ramp footprint')
    for axes in ax.flat:axes.legend(fontsize=8)
    for axes in ax[0]:axes.set_aspect('equal',adjustable='datalim');axes.set_xlabel('x (m)');axes.set_ylabel('y (m)')
    ax[0,0].set_title('Physical scene: independent execution only');ax[0,1].set_title('Actual navigation frame: camera_init')
    ax[1,0].set_xlabel('effective physics time (s)');ax[1,0].set_ylabel('world x (m)');ax[1,0].axhline(14,color='k',ls=':')
    ax[1,1].set_xlabel('effective physics time (s)');ax[1,1].set_ylabel('world body origin z (m)')
    fig.savefig(a.output/'route_and_progress.png');plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(13,9),constrained_layout=True)
    for j,(record,data)in enumerate(results):
        t=data['telemetry'];n=data['native'];label=Path(record['run']).name[-4:]
        for k,col in enumerate((1,2,3)):
            ax[k,j].plot(t[:,0],t[:,col],label='Teacher actual input',lw=1)
            ax[k,j].plot(n[:,0],n[:,7+k if k<2 else 12],label='native COM/body response',lw=.6,alpha=.8)
            ax[k,j].set_ylabel(('body vx (m/s)','body vy (m/s)','body wz (rad/s)')[k]);ax[k,j].legend(fontsize=8)
        ax[0,j].set_title(label+' actual command and response');ax[2,j].set_xlabel('simulation time (s), original 5ms state phase retained')
    fig.savefig(a.output/'commands_and_response.png');plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(13,9),constrained_layout=True)
    for j,(record,data)in enumerate(results):
        rows=[row for row in data['status']if finite(row['CORE_reference_yaw'])and finite(row['gate_heading'])]
        ts=np.array([row['t']for row in rows]);core=np.array([row['CORE_reference_yaw']for row in rows]);gate=np.array([row['gate_heading']for row in rows]);body=core-np.array([row['CORE_error_yaw']if finite(row['CORE_error_yaw'])else np.nan for row in rows])
        ax[0,j].plot(ts,gate,label='parent locked SCAN lookahead');ax[0,j].plot(ts,core,label='CORE local tangent');ax[0,j].plot(ts,body,label='actual SLAM body yaw');ax[0,j].set_ylabel('camera_init yaw (rad)');ax[0,j].legend(fontsize=8)
        ax[1,j].plot(ts,wrap(gate-core));ax[1,j].set_ylabel('parent minus CORE reference (rad)')
        ax[2,j].plot(ts,[row['command'][2]for row in rows]);ax[2,j].set_ylabel('actual controller wz input (rad/s)');ax[2,j].set_xlabel('actual ROS simulation clock (s)')
        ax[0,j].set_title(Path(record['run']).name[-4:]+' heading reference conflict')
    fig.savefig(a.output/'heading_reference_conflict.png');plt.close(fig)
    for record,data in results:
        tag=Path(record['run']).name[-4:];np.savez_compressed(a.output/(tag+'_extracted_original_arrays.npz'),native=data['native'],telemetry=data['telemetry'],SLAM=data['poses'])
        with (a.output/(tag+'_control_diagnostics.json')).open('x')as out:
            out.write(json.dumps({'status':data['status'],'PID':data['pid'],'paths':data['paths']},ensure_ascii=False,allow_nan=False)+'\n')
    with (a.output/'aggregate.json').open('x')as out:out.write(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    lines=['# 注册坡道第二轮：独立只读诊断','',
        '这份报告不修改任何原验收，也不以仿真真值导航。原生状态仅用于离线执行、坡段范围和坐标注册误差核对。',
        '两轮改变了注册、分段目标和角速上限，不能作为单一变量因果试验。','']
    for record,data in results:
        n=record['native'];h=record['heading_conflict'];r=record.get('registration')
        lines += [f"- {Path(record['run']).name[-4:]}：导航原状态 `{record['original_navigation_state']}`；原始SLAM到达 {len(record['original_arrivals'])} 个；COM最远世界x={n['max_COM_x']:.6f}m；完整12m跨越={n['complete_12m_seam_crossing']}。",
          f"  原生机身接触首项={n['first_body_contact']}；最大roll/pitch={n['max_roll_pitch']:.6f}rad；最小clearance={n['min_clearance']:.6f}m。",
          f"  父层align期间参考差={h['locked_gate_vs_CORE_reference_rad']}；大于原.2rad参考门的样本={h['larger_than_original_drive_gate_point2rad_count']}，剩余转向误差反向样本={h['opposite_remaining_yaw_errors_count']}。"]
        if r:lines += [f"  实际10配对注册重放相同={r['heading_receipt_exact_matches']}；目标映射最大差={r['route_mapping_max_error_m']:.3g}m；原pair文件SHA匹配={r['pair_archive_SHA_matches']}；离线初始轴误差={r['offline_axis_error_rad']:.6g}rad。"]
    lines += ['', '结论边界：参考不一致能解释父层禁止前进但内层已接近自身角度的停滞；这不是Teacher已经跌倒、扭矩不足或GPU根因的证据。调整应先统一唯一转向参考，保留真实SCAN guard及原到达/停车门。未完整跨越并到达出口时，平台停车仍未验证。',
        '接触统计为原命名碰撞对的数量，仅辅助诊断；不代替正式顶面XYZ/normal/FK/连续支撑验收，不称量测各足载荷。']
    with (a.output/'REPORT.md').open('x')as out:out.write('\n'.join(lines)+'\n')
    inventory={path.name:sha(path)for path in a.output.iterdir()if path.is_file()}
    with (a.output/'artifact_hashes.json').open('x')as out:out.write(json.dumps(inventory,indent=2)+'\n')
    print(json.dumps({'output':str(a.output),'aggregate_sha256':sha(a.output/'aggregate.json'),
        'summary':[dict(run=Path(x[0]['run']).name[-4:],state=x[0]['original_navigation_state'],native=x[0]['native'],heading=x[0]['heading_conflict'])for x in results]},ensure_ascii=False))

if __name__=='__main__':main()
