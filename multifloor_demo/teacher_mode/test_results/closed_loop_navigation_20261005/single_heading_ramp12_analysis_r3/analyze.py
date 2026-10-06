#!/usr/bin/env python3
"""Original-run-only fence/path/cloud diagnosis; no regrading or control."""
import hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
RUN=ROOT/'runs/20261005_144730_closed_loop_cascade_single_heading_ramp12_r3_2d63'

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()
def write(path,value):
    with path.open('x')as stream:stream.write(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def distance_xy(points,line):
    points=np.array(points);best=np.full(len(points),np.inf)
    for a,b in zip(line[:-1],line[1:]):
        d=b[:2]-a[:2];f=np.clip((points[:,:2]-a[:2])@d/(d@d),0,1)
        best=np.minimum(best,np.linalg.norm(points[:,:2]-a[:2]-f[:,None]*d,axis=1))
    return best

def main():
    output=HERE/'final'
    if output.exists():raise RuntimeError('Never overwrite a previous analysis')
    original=HERE.parent/'registered_ramp12_analysis_r2/analyze.py'
    spec=importlib.util.spec_from_file_location('read_only_registered_ramp_diagnostic',original)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    summary,data=module.analyze(RUN)
    hashes=summary['input_hashes'];first=None
    for row in module.records(RUN/'navigation_command_history.jsonl',hashes):
        if first is None and row.get('state')=='failed'and (row.get('registered_route_fence')or{}).get('inside')is False:first=row
    if first is None:raise RuntimeError('No original failed finite-fence envelope')
    fence_time=float(first['sim_time']);pose_stamp=first['slam_stamp_ns'];pose_row=None;core_row=None;guard=None
    for row in module.records(RUN/'navigation_slam_poses.jsonl',hashes):
        if row['stamp_ns']==pose_stamp:pose_row=row
    for row in module.records(RUN/'navigation_pid_history.jsonl',hashes):
        if row['compute_ros_clock_ns']/1e9<=fence_time and row['waypoint_index']==1:core_row=row
    for row in module.records(RUN/'navigation_guard_history.jsonl',hashes):
        if row['compute_ros_clock_ns']/1e9<=fence_time:guard=row
    if pose_row is None or core_row is None or guard is None:raise RuntimeError('Original exact fence/control/guard source missing')
    cloud_index=None
    for row in module.records(RUN/'navigation_cloud_xyz.jsonl',hashes):
        if row['stamp_ns']==guard['cloud_header_stamp_ns']:cloud_index=row
    if cloud_index is None:raise RuntimeError('Original cloud SHA/stamp binding missing')
    cloud_file=RUN/cloud_index['array_file'];raw_file=RUN/cloud_index['raw_payload_file']
    cloud=np.load(cloud_file,allow_pickle=False)
    file_sha=sha(cloud_file);raw_sha=sha(raw_file);decoded_sha=hashlib.sha256(np.asarray(cloud,dtype='<f8').tobytes()).hexdigest()
    if (file_sha!=cloud_index['array_sha256']or raw_sha!=cloud_index['raw_payload_sha256']or
        decoded_sha!=cloud_index['filtered_xyz_float64_sha256']or decoded_sha!=guard['filtered_xyz_float64_sha256']):
        raise RuntimeError('Actual selected cloud/guard hashes differ')
    hashes[str(cloud_file)]=file_sha;hashes[str(raw_file)]=raw_sha
    anchor=summary['anchor'];route=np.array(anchor['registered_route_camera_init_xyz']);native=data['native'];telemetry=data['telemetry'];poses=data['poses']
    active_paths=[row for row in data['paths']if row['stamp_ns']/1e9<=fence_time and row['path_id'].split(':')[-3]=='1']
    active=next((row for row in active_paths if row['path_id']==core_row['cascade']['path_id']),None)
    if active is None:raise RuntimeError('Original CORE path not in archived accepted SCAN list')
    path=np.array(active['points']);planned_dist=distance_xy(path,route)
    c=core_row['cascade'];idx=np.searchsorted(native[:,0],fence_time,side='right')-1
    statuses=[row for row in data['status']if row['t']<=fence_time and row['state']=='running']
    heading_deltas=[float(abs(module.wrap(row['gate_heading']-row['CORE_reference_yaw'])))for row in statuses
                    if row['gate_phase']=='align'and module.finite(row['gate_heading'])and module.finite(row['CORE_reference_yaw'])]
    corridor=guard['union_result'];roi=(cloud[:,0]>=3)&(cloud[:,0]<=9)&(cloud[:,1]>=-2)&(cloud[:,1]<=1)&(cloud[:,2]>=-1)&(cloud[:,2]<=2)
    output.mkdir()
    receipt=dict(schema='single_heading_ramp12_read_only_diagnosis/v1',status='diagnostic_only',
        full_original_run_summary=summary,source_script_sha256=sha(__file__),helper_script_sha256=sha(original),
        first_failed_fence_original_envelope=first,original_fence_pose=pose_row,
        first_fence_time_world_s=fence_time,offline_native_before_fence={'effective_time_s':float(native[idx,0]),
            'position':native[idx,1:4].tolist(),'COM':native[idx,4:7].tolist(),'rpy':native[idx,13:16].tolist(),
            'body_COM_velocity':native[idx,7:10].tolist()},
        original_CORE_at_failure=dict(sequence=core_row['sequence'],source_pose_stamp_ns=core_row['source_pose_stamp_ns'],
            mode=c['mode'],reference_yaw_rad=c['reference_yaw_rad'],error_yaw_rad=c['error_yaw_rad'],
            path_cross_error_m=c['error_cross_m'],path_projection=c['nearest_projection_xyz'],
            local_tangent=c['local_horizontal_tangent'],actual_command=core_row['command_after_slew'],
            path_id=c['path_id'],path_sha256=c['path_sha256']),
        original_SCAN_selected_path=active,selected_SCAN_path_max_distance_from_fixed_registered_prior_m=float(planned_dist.max()),
        selected_SCAN_path_RMS_distance_from_fixed_registered_prior_m=float(np.sqrt(np.mean(planned_dist**2))),
        parent_CORE_alignment_delta_before_failure=module.stats(heading_deltas),
        original_guard_at_failure={'sequence':guard['sequence'],'compute_ros_clock_ns':guard['compute_ros_clock_ns'],
            'control_pose_stamp_ns':guard['control_pose_stamp_ns'],'cloud_header_stamp_ns':guard['cloud_header_stamp_ns'],
            'goal_corridor_result':guard['goal_corridor_result'],'motion_corridor_result':guard['motion_corridor_result'],
            'union_result':corridor,'obstacle_hold_after':guard['obstacle_hold_after']},
        selected_original_cloud=dict(index=cloud_index,verified_decoded_and_raw_SHA=True,
            actual_points=len(cloud),display_ROI_points=int(roi.sum()),display_ROI_camera_init_bounds=[[3,9],[-2,1],[-1,2]],
            XYZ_is_filtered_registered_cloud_not_planner_occupancy=True),
        planner_occupancy_state_archived=False,
        planner_internal_cost_causal_conclusion='unverified: exact occupancy/inflation/optimizer state was not archived; raw cloud alone cannot prove which cost caused lateral detour',
        body_height_cause_claimed=False,navigation_ground_truth_used=False,native_truth_only_offline=True,
        original_run_and_receipts_unchanged=True)
    write(output/'aggregate.json',receipt)
    np.savez_compressed(output/'extracted_original_arrays.npz',native=native,telemetry=telemetry,SLAM=poses,cloud=cloud,selected_SCAN_path=path,registered_route=route)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.grid':True,'figure.dpi':130})
    fig,ax=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for row in active_paths:
        points=np.array(row['points']);ax[0,0].plot(points[:,0],points[:,1],color='.7',alpha=.6,lw=.8)
    ax[0,0].plot(route[:,0],route[:,1],'k--',label='registered scene centerline prior')
    ax[0,0].plot(path[:,0],path[:,1],label='actual selected SCAN path',color='tab:purple')
    ax[0,0].plot(poses[:,1],poses[:,2],label='actual raw SLAM motion',color='tab:blue')
    ax[0,0].scatter(*pose_row['position'][:2],color='r',label='first fence failure',zorder=6)
    ax[0,0].set_xlim(0,14);ax[0,0].set_ylim(-1.5,.6);ax[0,0].set_title('Navigation frame: camera_init')
    ax[0,1].plot([2,14,14,2,2],[1,1,3,3,1],'k:',label='original ramp footprint')
    ax[0,1].plot(native[:,1],native[:,2],label='native execution (offline only)')
    ax[0,1].scatter(native[idx,1],native[idx,2],color='r',label='first fence time')
    ax[0,1].set_xlim(1,15);ax[0,1].set_ylim(.5,3.5);ax[0,1].set_title('Physical scene: independent execution')
    for axes in ax[0]:axes.set_xlabel('x (m)');axes.set_ylabel('y (m)');axes.legend(fontsize=8)
    xs=[];route_errors=[];core_errors=[];yaw_errors=[]
    for row in data['pid']:
        if row['t']>fence_time or row['waypoint']!=1:continue
        xs.append(row['t']);route_errors.append(float(distance_xy([row['control_pose']],route)[0]));yaw_errors.append(row['CORE_error_yaw'])
    for row in module.records(RUN/'navigation_pid_history.jsonl',{}):
        if row['compute_ros_clock_ns']/1e9>fence_time or row['waypoint_index']!=1:continue
        core_errors.append(abs(row['cascade'].get('error_cross_m',np.nan)))
    ax[1,0].plot(xs,route_errors,label='distance to fixed registered route')
    ax[1,0].plot(xs,core_errors,label='CORE cross error to accepted SCAN path')
    ax[1,0].axhline(.45,color='r',ls='--',label='unchanged fence .45 m');ax[1,0].set_ylabel('distance (m)');ax[1,0].legend(fontsize=8)
    ax[1,1].plot(xs,yaw_errors,label='CORE actual SLAM yaw error to SCAN tangent')
    ax[1,1].axhline(.2,color='r',ls=':',label='original drive heading threshold');ax[1,1].axhline(-.2,color='r',ls=':')
    ax[1,1].set_ylabel('yaw error (rad)');ax[1,1].legend(fontsize=8)
    for axes in ax[1]:axes.axvline(fence_time,color='r',ls=':');axes.set_xlabel('actual simulation clock (s)')
    fig.savefig(output/'route_fence_and_tracking.png');plt.close(fig)
    fig,ax=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    displayed=cloud[roi];stride=max(1,int(np.ceil(len(displayed)/20000)));shown=displayed[::stride]
    im=ax[0,0].scatter(shown[:,0],shown[:,1],c=shown[:,2],s=2,cmap='viridis',alpha=.6)
    fig.colorbar(im,ax=ax[0,0],label='actual registered point z (m)')
    for axes in (ax[0,0],ax[0,1]):
        axes.plot(route[:,0],route[:,1]if axes is ax[0,0]else route[:,2],'k--',label='fixed requested body-origin route')
        axes.plot(path[:,0],path[:,1]if axes is ax[0,0]else path[:,2],color='tab:purple',label='actual SCAN body-origin path')
    ax[0,1].scatter(shown[:,0],shown[:,2],s=2,c=shown[:,1],cmap='coolwarm',alpha=.5)
    ax[0,0].set_xlim(3,9);ax[0,0].set_ylim(-2,1);ax[0,0].set_ylabel('camera_init y (m)')
    ax[0,1].set_xlim(3,9);ax[0,1].set_ylim(-1,2);ax[0,1].set_ylabel('camera_init z (m)')
    for axes in ax[0]:axes.set_xlabel('camera_init x (m)');axes.legend(fontsize=8)
    ax[0,0].set_title('Original cloud at '+str(guard['cloud_header_stamp_ns']/1e9)+' s, not occupancy map')
    mt=telemetry[:,0]<fence_time+3;mn=native[:,0]<fence_time+3
    ax[1,0].plot(telemetry[mt,0],telemetry[mt,1],label='actual Teacher body vx input')
    ax[1,0].plot(native[mn,0],native[mn,7],label='actual native COM body vx',lw=.8);ax[1,0].set_ylabel('vx (m/s)')
    ax[1,1].plot(telemetry[mt,0],telemetry[mt,3],label='actual Teacher body wz input')
    ax[1,1].plot(native[mn,0],native[mn,12],label='actual native body wz',lw=.8);ax[1,1].set_ylabel('wz (rad/s)')
    for axes in ax[1]:axes.axvline(fence_time,color='r',ls=':');axes.set_xlabel('simulation time (s), native effective phase t-.005');axes.legend(fontsize=8)
    fig.savefig(output/'original_cloud_and_motion.png');plt.close(fig)
    rc={row['role']:row['returncode']for row in summary['original_runtime_manifest']['owned_processes']}
    text=f'''# 单一转向参考坡道第三轮：只读诊断\n\n实际原件：{RUN.name}。新父层与CORE均使用实际SCAN最近3D投影切线；原20Hz/10Hz真实源、Teacher执行链没有被本报告修改。\n\n首次围栏失败在{fence_time:.3f}s，原SLAM位置{pose_row['position']}；原生前一时相机身位置{native[idx,1:4].tolist()}。固定地图先验route error={first['registered_route_fence']['horizontal_error_m']:.6f}m，超过原0.45m门；CORE对其当前SCAN路径cross error={c['error_cross_m']:.6f}m，yaw error={c['error_yaw_rad']:.6f}rad。接受的SCAN路径相对固定注册中心线最大偏离{planned_dist.max():.6f}m。这证明跟踪的局部路径已经偏离地图中心线，而不是仅控制器离开自己的参考。\n\n首失败前原guard union={corridor}。所选原点云header={guard['cloud_header_stamp_ns']}、{len(cloud)}点；NPY文件/rawpayload/decoded float64/原guard SHA逐项相同。图中点云只是实际registered XYZ，**不是内部占据图**。当前run未归档planner occupancy/inflation或优化代价状态，不能据这些点云直接断言具体障碍体素或代价权重造成绕行。body_height在该链用于目标Z的加/减转换，不能未经证据称它影响坡面分类。\n\n首失败后停车保护由真实SLAM围栏触发。完整坡道和出口停车仍未验证。原运行收尾role返回值={rc}；bridge=-6是另一项真实收尾失败，保留原runtime error，不伪称完整退出通过。\n\n后续可采用更密的已注册坡中心线区域目标来约束局部规划跨度，同时保留原真实点云union guard、0.45m围栏和到达门；仍需另版实测。若需要解释规划器绕行的确切原因，应新增只读内部occupied/inflated voxel与对应优化路径/cost记录，不能用模拟真值补规划器观测。\n'''
    with (output/'REPORT.md').open('x')as stream:stream.write(text)
    write(output/'artifact_hashes.json',{p.name:sha(p)for p in output.iterdir()if p.is_file()})
    print(json.dumps({'output':str(output),'aggregate_sha256':sha(output/'aggregate.json'),
        'first_fence':fence_time,'CORE_cross_error':c['error_cross_m'],'CORE_yaw_error':c['error_yaw_rad'],
        'planned_path_max_route_deviation':float(planned_dist.max()),'guard':corridor,'own_returns':rc}))

if __name__=='__main__':main()
