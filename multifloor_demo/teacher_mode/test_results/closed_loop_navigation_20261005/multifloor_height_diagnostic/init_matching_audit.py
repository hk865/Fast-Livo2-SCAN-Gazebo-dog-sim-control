import os
for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
import json,re,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
BASE=HERE.parents[2]
PROJECT=BASE.parents[1]
CORE=BASE.parent/'slam/ros2_ws/src/fast_livo2_core'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def segments(t,w,limit):
    rows=[]; start=None; end=None; prev=None
    for tm,wi in zip(t,w):
        if wi<=limit and (prev is None or 0<tm-prev<=.030000001):
            if start is None:start=tm
            end=tm
        else:
            if start is not None:rows.append([float(start),float(end),float(end-start)])
            start=tm if wi<=limit else None;end=tm if wi<=limit else None
        prev=tm
    if start is not None:rows.append([float(start),float(end),float(end-start)])
    return sorted(rows,key=lambda a:a[2],reverse=True)
data={}
for name in ['20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e','20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182']:
    p=BASE/'runs'/name;log=(p/'navigation_stack.log').read_text()
    init=[s for s in log.splitlines() if '[DEMO_IMU_INIT]' in s][-1]
    actual=[]; hh=hashlib.sha256();nb=0;nc=0
    with(p/'navigation_imu_history.jsonl').open('rb')as f:
        for b in f:
            a=json.loads(b);tm=a['stamp_ns']/1e9
            if tm>6.7:break
            actual.append([tm,*a['angular_velocity_body']]);hh.update(b);nb+=len(b);nc+=1
    actual=np.array(actual);t=actual[:,0];w=np.linalg.norm(actual[:,1:],axis=1)
    ws={}
    for lo,hi in [(0,3.6),(.58,3.6),(3.6,6.7)]:
        mask=(t>=lo)&(t<=hi);ws[f'{lo}..{hi}']={'samples':int(mask.sum()),'gyro_norm_max_radps':float(w[mask].max()),'gyro_norm_mean_radps':float(w[mask].mean()),'gyro_axis_mean_radps':actual[mask,1:].mean(axis=0).tolist(),'sample_gap_max_s':float(np.diff(t[mask]).max())}
    strict={str(cap):{'longest_windows_start_end_span_s':segments(t,w,cap)[:5],'has_continuous_3s_window':any(a[2]>=3 for a in segments(t,w,cap))} for cap in [.01,.005,.02,.022,.025]}
    nv=[];h=hashlib.sha256();length=0
    with(p/'actuator.jsonl').open('rb')as f:
        for b in f:
            a=json.loads(b)
            if a.get('kind')!='physics_step':continue
            tm=a['t']-.005
            if tm>3.6:break
            h.update(b);length+=len(b)
            if tm>=.58:nv.append([tm,*a['position'],*a['quaternion_wxyz'],*a['body_ang_vel']])
    nv=np.array(nv);r=Rotation.from_quat(nv[:,[5,6,7,4]]).as_euler('xyz',degrees=True)
    matrix=np.loadtxt(p/'fastlivo_debug/imu.txt');mid=matrix[:,0]+.0125
    un,ix=np.unique(mid,return_index=True);mask=(un>=3.6)&(un<=6.7);av=matrix[ix][mask,4:7];norm=np.linalg.norm(av,axis=1)
    acc={'scope':'logged predictor midpoint acceleration = average actual IMU head/tail, not original individual accelerometer samples',
         'time_rule':'relative head timestamp + first_lidar .01 + midpoint .0025 s',
         'first_last_s':un[mask][[0,-1]].tolist(),'samples':len(av),'axis_mean_mps2':av.mean(axis=0).tolist(),'axis_std_mps2':av.std(axis=0).tolist(),
         'norm_min_mean_max_mps2':[float(norm.min()),float(norm.mean()),float(norm.max())]}
    final=json.loads((p/'navigation_status.json').read_text());ev=final['region_arrival_evidence'];err=np.array(ev['raw_position'])-final['current_goal']['center']
    first_failed=None;hstat=hashlib.sha256();nstat=0
    with(p/'navigation_status.jsonl').open('rb')as f:
        for b in f:
            a=json.loads(b);tm=a.get('ros_sim_time')
            if tm is not None and tm>230:break
            hstat.update(b);nstat+=len(b)
            if a.get('state')=='failed':first_failed={'ros_sim_time':tm,'message':a['message'],'waypoint_index':a['waypoint_index'],'current_goal':a['current_goal'],'pose':a['pose'],'region_arrival_evidence':a['region_arrival_evidence']};break
    data[name]={'initialization_accepted_log':init,'actual_gyro_windows':ws,'stricter_gate_windows':strict,
       'acceleration_after_initialization':acc,'native_initial_motion_offline_only':{'sample_count':len(nv),'xy_displacement_norm_m':float(np.linalg.norm(nv[-1,1:3]-nv[0,1:3])),
        'z_minmax_m':[float(nv[:,3].min()),float(nv[:,3].max())],'rpy_first_last_deg':r[[0,-1]].tolist(),'max_body_angular_velocity_radps':float(np.linalg.norm(nv[:,8:11],axis=1).max())},
       'runtime_result':json.loads((p/'run_result.json').read_text()),'first_archived_navigation_failure_before_230s':first_failed,
       'final_original_arrival':ev,'arrival_error_xyz_m':err.tolist(),'actual_config':json.loads((p/'navigation_fastlivo.yaml').read_text())['/**']['ros__parameters'],
       'provenance':{'actual_imu_prefix':{'path':str(p/'navigation_imu_history.jsonl'),'sha256':hh.hexdigest(),'bytes':nb,'rows':nc,'last_stamp_limit_s':6.7},
        'native_prefix_offline_only':{'path':str(p/'actuator.jsonl'),'sha256':h.hexdigest(),'bytes':length,'cutoff_state_time_s':3.6},
        'navigation_status_through_first_failed':{'path':str(p/'navigation_status.jsonl'),'sha256':hstat.hexdigest(),'bytes':nstat},
        'frozen_file_hashes':{k:sha(p/k) for k in ['navigation_stack.log','navigation_fastlivo.yaml','navigation_status.json','run_result.json','fastlivo_debug/imu.txt']}}}
source_files=['src/IMU_Processing.cpp','include/fast_livo2_core/core/stationary_imu_initialization.h','include/fast_livo2_core/core/IMU_Processing.h','src/LIVMapper.cpp','src/vio.cpp','src/voxel_map.cpp']
result={'schema':'offline_actual_imu_init_and_local_matching_audit/v1','scope':'read-only local code and original sensor logs; native motion is offline comparison only; no new PASS',
    'runs':data,'source_code_hashes':{str(CORE/k):sha(CORE/k) for k in source_files},
    'active_vio_information_weight':{'actual_img_point_cov':100000000,'source_default':100,'relative_information_per_pixel':1e-6,'equation':'K1=(H.T@H + (P/R)^(-1))^(-1), equivalent information H.T@H/R + P^(-1)'},
    'actual_call_graph':{'lio':'LIVMapper::handleLIO -> VoxelMapManager::StateEstimation -> BuildResidualListOMP -> weighted point-to-plane iterative EKF -> UpdateVoxelMap',
        'vio':'processFrame -> retrieveFromVisualSparseMap -> computeJacobianAndUpdateEKF -> generateVisualMapPoints -> updateVisualMapPoints',
        'reference_selection':'retrieveFromVisualSparseMap selects reference by observation photometric consistency or getCloseViewObs; updateVisualMapPoints adds observations >.5m/>.3rad/>40px and limits to30',
        'updateReferencePatch':'Defined but not called in current processFrame; do not credit as executed mechanism',
        'loop_closure':'No place-recognition/pose-graph/global loop-closure path found in this core; loop back log is timestamp regression handling'}}
(HERE/'init_matching_metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
text='''# 初始化、匹配和在线估计补充审计

本附录是新文件；没有覆盖 d42e 原失败、DIAGNOSTIC.md 或原 run 来源。当前接的是 FAST-LIVO2 的局部激光/视觉/惯性估计，不是 ORB-SLAM3。静止门有实际作用，但它验证的是低运动窗口，没有验证“机身完全不动”。

两轮都实际接受 0.58–3.60 s 共 605 样本、3.020 s；此前 reset=111、末次 reset 原因为 angular_motion。d42e max gyro .028850 rad/s、acc std [.023828,.027940,.040921] m/s²；b182 max gyro .028919、acc std [.024599,.026968,.041752]。原门为 gyro≤.03、|acc norm−9.81|≤1.2、axis std≤.4、sample gap≤.03、span≥2.9 s、样本≥600。大幅初始运动已被拒绝，随后缓慢转动仍通过。仅用于离线说明的物理初始窗口两轮相同：XY 位移 9.626 mm、yaw +2.115°、Z 范围 3.690 mm。

在原实际 IMU 0–6.7 s 来源中，**两轮都没有 gyro≤.01 或≤.005 rad/s 连续3 s窗口**。≤.01 最长 d42e .815–1.065 s共 .250 s，b182 .820–1.065 s共 .245 s；≤.005 两轮均只有 .020–.105 s共 .085 s，该段还处在下落初始化早期，不足以判断重力。3.6–6.7 s 尚未导航期 gyro 均值 .018925/.018898、最大 .020935/.020968 rad/s。直接仅收紧到 .01 并等待3 s，在这份实测前缀中不会完成初始化。更宽 .022/.025 的最长窗口数值保留在 JSON，能完成受理不等于完全静止。

原导航 IMU 归档记录 gyro 和 orientation，未保存原始单个加速度样本。受理前加速度只能依靠初始化器实际记录的完整窗口均值/std；受理后 debug/imu.txt 记录的是传播器相邻 head/tail 的平均加速度，而非单个原传感器样本。后者3.6–6.7 s 的加速度 norm/min/mean/max、axis std 已按此来源明确记录在 JSON，不能把这个平均信号冒充完整0–6.7 s原始加速度样本。

StationaryImuInitialization 收集 mean_gyro，但 IMU_Processing.cpp139 把 bias_g 明确设为零，没有把当前真实缓慢转动当作传感器零偏。不能直接用这段 mean_gyro 初始化 bias，否则会把真实 yaw motion 抹掉。该受理窗口的重力均值可能带来小角度倾斜；d42e 离线测得初始frame roll约 .28°，仍不足以解释后续突然20cm偏置。改善初始化需要能实际维持静止/姿态稳定的仿真初始化阶段，或采用明确允许移动的初始化方法；更严门本身不保证有合格数据。

实际视觉配置 `img_point_cov=100000000`，源码参数默认100。vio.cpp1516/1680 使用 `(HᵀH + (P/R)⁻¹)⁻¹`，等价于图像信息 `HᵀH/R + P⁻¹`：在同一H下实际每像素信息权重比默认弱 **100万倍**。这不是视觉功能被关闭，但视觉修正本来就会很弱，与 d42e 该区间 VIO位移修正近零一致。应先审查这一配置为何保留及图像标定/残差，再做独立逐级权重对照；不能因为已写了VIO接口就宣称融合有效，也不能凭默认100贸然覆盖已通过camera_mode。

当前实际匹配机制：LIVMapper.cpp529 调用 StateEstimation；voxel_map.cpp388迭代点到平面残差，462以平面/点方差加权，467构造位姿H，482–495 ESIKF更新并经状态协方差影响速度等状态。VIO processFrame1806 的实际调用是 retrieveFromVisualSparseMap、computeJacobianAndUpdateEKF、generateVisualMapPoints、updateVisualMapPoints；retrieve内参考观测按光度一致性/近视角选择，774/782做可选NCC和patch残差拒绝。updateVisualMapPoints928以位移>.5m、旋转>.3rad、像素移动>40选择新观测，观测达到30删除弱参考。**已有局部帧/参考管理**，不等于无帧匹配，也不等于全局回环。updateReferencePatch988仅定义，当前processFrame没有实际调用，不列入已执行能力。

本core源码没有发现 place recognition、pose graph 或全局loop closure的调用链；“lidar/imu/image loop back”日志只是时间戳倒退时清理缓存。可检查局部匹配/有效平面/视觉信息权重，再判断是否需要外部回环或图优化。接入 ORB-SLAM3 会涉及新的尺度、坐标、时间与融合合同，不能作为自动修复此短时高度状态偏差的结论。

V5 b182 仅开启在线 gravity/bias估计仍实际在connector_y原高度门失败：220.3 s原位置 [14.984697,4.394268,.983030]，原目标 [14.986784,4.414264,1.205630]，Z差−222.600 mm。其后约429 s BrokenPipe使整轮runtime失败，未完成600 s。因此可说“开启这两flags未消除这一次已观测的heightfail”；不能称完整有效长程对照、仅凭本轮认定唯一根因，或把后续断管当作先前heightfail原因。

复现：`PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 -B init_matching_audit.py`。原actual IMU prefix hash、消费字节数、配置及源码hash均在 init_matching_metrics.json。未启动Actor/ROS/physics，未改变任何原来源、controller或验收。
'''
(HERE/'INIT_MATCHING_APPENDIX.md').write_text(text)
files=['init_matching_audit.py','init_matching_metrics.json','INIT_MATCHING_APPENDIX.md']
(HERE/'appendix_manifest.json').write_text(json.dumps({'schema':'derived_init_matching_appendix/v1','original_sources_modified':False,'files_sha256':{k:sha(HERE/k) for k in files}},indent=2)+'\n')
print(json.dumps({'strict_windows':{n:v['stricter_gate_windows'] for n,v in data.items()},'appendix_sha256':sha(HERE/'INIT_MATCHING_APPENDIX.md')},indent=2))
