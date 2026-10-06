"""Create a small report/curve from completed independent receipts only."""
import copy,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_failed_region import lines,digest
from evaluate_mission46 import Mission,V19,TOP,snapshot,sha

HERE=Path(__file__).resolve().parent
RUN=TOP/'runs/20261006_024447_closed_loop_cascade_clock_hold_v19_original46_r3_heading_9779'
def load(name):return json.loads((HERE/name).read_text())
def create(name,data):
 with(HERE/name).open('x')as f:json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

facts=load('V19_9779_BOUNDED_RUN_FACTS.json');evaluation=load('V19_9779_INDEPENDENT_MISSION_EVALUATION.json');obs=load('V19_9779_FAILED_REGION_OBSERVATION.json');audit=load('V19_9779_HEADING_RESET_PROJECTION_AUDIT.json')
request_path=RUN/'mission46_requests/exploration_1.json';request=json.loads(request_path.read_text());profile=json.loads((RUN/'navigation_profile.json').read_text());anchor=json.loads((RUN/'navigation_anchor.json').read_text());m=Mission();m.scenario=profile['original_scenario'];m.run_id=RUN.name;m.origin=anchor['origin'];m.heading_alignment=anchor['scene_axis_registration']['heading_receipt'];m.request_route('exploration',0)
receipts=facts['physical_window']['region_receipts'];prefix=copy.deepcopy(m);prefix.current_goals=prefix.current_goals[:len(receipts)];prefix.goal_region=prefix.current_goals[-1];view=dict(request_id=m.current_request,goals_definition_sha256=m.current_goals_sha256,goals_definitions=prefix.current_goals,total=len(receipts),waypoint_index=len(receipts),region_arrivals=receipts);valid=prefix.region_completion_valid(view,receipts[-1]['raw_position'])
create('V19_9779_PARTIAL_ORIGINAL_REGION_RECEIPTS.json',dict(schema='independent_original46_partial_region_receipts/v1',run_id=RUN.name,run=str(RUN.resolve()),phase='exploration',full_phase_pass=False,full46_pass=False,actual_original_prefix_receipts_valid=valid,completed_regions=len(receipts),required_phase_regions=18,use_of_prefix='The original region_completion_valid gates are reused on the observed prefix exactly as TeacherMission46._valid_prefix; no full-phase completion inferred.',original_full_goals_definition_sha256=m.current_goals_sha256,exact_original_receipts=receipts,request_source=str(request_path),request_sha256=digest(request_path),source_bindings={str(HERE/'V19_9779_BOUNDED_RUN_FACTS.json'):digest(HERE/'V19_9779_BOUNDED_RUN_FACTS.json'),str(HERE/'V19_9779_INDEPENDENT_MISSION_EVALUATION.json'):digest(HERE/'V19_9779_INDEPENDENT_MISSION_EVALUATION.json'),str(snapshot(RUN,'state_machine.py')):sha(snapshot(RUN,'state_machine.py'))},max_actual_SLAM_dwell_gap_ns=evaluation['checks']['exploration_original_regions_and_actual_dwell']['max_source_gap_ns']))
first=None
for row,source in lines(RUN/'navigation_command_publications.jsonl'):
 if row['state_after']=='failed':first=dict(record={k:row.get(k)for k in('sequence','trigger','state_before','state_after','publish_ros_clock_ns','publish_started_monotonic_wall_ns','monotonic_wall_ns','command_before','command_after')},source=source);break
shared=snapshot(RUN,'shared_controller.py');wrapper=snapshot(RUN,'teacher_wrapper.py')
if not first:raise ValueError('Missing actual first failed publication')
elapsed=first['record']['publish_ros_clock_ns']/1e9-obs['activation_sim_s']
create('V19_9779_FIRST_FAILURE_CAUSAL_WINDOW.json',dict(schema='teacher_original46_first_failure_causal_window/v1',run_id=RUN.name,run=str(RUN.resolve()),first_observed_failed_publication=first,first_sampled_failed_nav_status=obs['first_failed_navigation_status'],goal_activated_sim_s=obs['activation_sim_s'],first_failed_publication_elapsed_sim_s=elapsed,declared_goal_deadline_sim_s=90.,timeout_condition_met=elapsed>90,original_timeout_precedes_stale_and_tilt_checks_in_control=True,failure_message_overwritten_by_pre_turn_or_settle=True,source_bindings={str(shared):digest(shared),str(wrapper):digest(wrapper)},physical_cleanup_boundary=facts['physical_window']['last_native_command_read_monotonic_wall'],last_cleanup_status=facts['physical_window']['last_cleanup_status'],navigation_first_failure_reason='Original per-goal 90s deadline exceeded; failed state is preserved while its message is overwritten by wrapper waiting-stop text.',limits=['First observed failed command publication is not claimed to be an exact timestamp of the earlier Python state assignment.','300ms protections and the original 90s deadline are unchanged.']))

# Visualize all original PID rows in the failed region. Plot data is projected,
# bounded in memory, and contains no substitute safety verdict.
pid=[]
for row,_ in lines(RUN/'navigation_pid_history.jsonl'):
 if row.get('waypoint_index')!=8:continue
 c=row['cascade'];g=row['heading_gate_reference'];s=g.get('steering')or{};q=row['feedback']['quaternion_wxyz'];w,x,y,z=q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));pid.append([row['compute_ros_clock_ns']/1e9,s.get('heading',math.nan),yaw,row['feedback']['origin_velocity_body'][0],row['command_after_slew'][0],row['control_pose'][0],row['control_pose'][1],dict(pre_turn=0,align=1,settle=2,drive=3).get(g['phase'],-1)])
a=np.asarray(pid);start=obs['activation_sim_s'];goal=request['goals'][8]['center'];fig,axs=plt.subplots(4,1,figsize=(11,10),layout='constrained');fig.suptitle('V19 original46 r3: exploration:8 timeout (actual SLAM / SCAN)')
axs[0].plot(a[:,0],a[:,5],label='Actual SLAM x');axs[0].axhline(goal[0],color='k',ls='--',label='Original goal x');axs[0].set_ylabel('x (m)');axs[0].legend(loc='best')
axs[1].plot(a[:,0],a[:,1],'.',ms=1.6,alpha=.55,label='Actual nearest SCAN tangent');axs[1].plot(a[:,0],a[:,2],lw=1.2,label='Actual SLAM yaw');axs[1].set_ylabel('Heading (rad)');axs[1].legend(loc='best')
axs[2].plot(a[:,0],a[:,4],lw=.8,label='Actually published vx command');axs[2].plot(a[:,0],a[:,3],lw=.8,label='Actual SLAM body vx');axs[2].set_ylabel('vx (m/s)');axs[2].legend(loc='best')
axs[3].step(a[:,0],a[:,7],where='post',lw=.8);axs[3].set_yticks([0,1,2,3],['pre_turn','align','settle','drive']);axs[3].set_ylabel('Heading gate');axs[3].set_xlabel('Simulator clock (s)')
for ax in axs:ax.axvline(first['record']['publish_ros_clock_ns']/1e9,color='r',ls='--',lw=1);ax.set_xlim(start,223.7);ax.grid(alpha=.2)
fig.savefig(HERE/'V19_9779_failed_region_curves.png',dpi=130);plt.close(fig)

timing=TOP/'test_results/pipeline_v19_20261006/performance/ACTUAL_CLOCK_DOMAINS.json';timing_ref=f'[{timing.name}](../performance/{timing.name})'
report='''# 原46区域 V19 r3 独立失败报告

原46任务未通过。本轮原点初始化和8个原探索区域的真实SLAM到达证据通过，但第9区 `exploration:8` 在原90s期限内未到达。返航14区、当次RGB保存、F1→F3导航14区、3次地形切换、动态障碍停车恢复和最终5s停车均未执行，不记通过。本次只做仿真。

朝向参考 revision03 已实测一致：938次 align 转向更新使用外部门真实锁定朝向，2549次其它有效参考更新也一致，最大参考差0。旧“外门锁旧路径、内环追新路径”的故障已解除，原增益、0.2rad行走朝向门、0.1rad转向完成门、300ms时效和90s期限均未改变。

失败窗口和直接机制：

- 目标在133.450s激活。首个已失败的实际零速度发布为223.475s（elapsed90.025s）；首个导航状态采样为223.615s。原控制源码在保护检查前执行该期限分支，随后 wrapper 把消息覆写成“等待停车”，因此不能把这句状态文字当根因。
- 第9区的104次 `drive→pre_turn` 全部伴随新SCAN路径，独立读取104个原NPZ、样本索引和哈希，并调用冻结core的纯局部投影后，切线与运行记录完全一致；全部超出原0.2rad门。未调用Controller.update，也未宣称完整PI重放通过。
- 最近起始小段方向与整条路线差别很大：163.340s path390最近段只有67.17微米，切线2.25080rad，但起点到终点方向0.17443rad；其规划起始 used_velocity 约[-0.000980,0.000570,0.001260]m/s。162.665s path372最近0.393mm段切线−0.63840rad，整条路径方向0.17477rad。
- 90s内PID记录覆盖的阶段约 pre_turn44.50s、align13.11s、settle5.355s、drive26.98s。约x10.4m以后频繁短行走脉冲被重新停车打断。独立事实支持“接近停稳的微小速度与样条起始局部方向，经单位化成为大朝向变化，并触发门复位”这一机制；为什么SCAN持续产生这种起始段、速度估计与Teacher动作各占多少影响，还未单独隔离，不把GPU或策略作为已证明的唯一原因。

来源、安全与流水线：

- 原8区域的身份、顺序、0.4s连续dwell和内控制区域均通过；实际SLAM最大dwell观测间隔35.000001ms。
- 第9区运行期max age为SLAM140ms、点云136ms、IMU91ms，均小于300ms；未出现obstacle_hold/tilt_hold。退出后的IMU超时单列为清理观测，不是首运行期失败。
- 11534帧原50Hz native快照：fault0、机身接触0、最大roll0.03555rad/pitch0.10441rad、最大关节速度13.9383rad/s/力矩15.6797Nm，最低机身间隙0.25811m。严格200Hz执行器trace和完整发布/PI/所有SCAN几何重放仍未验证。
- 冻结Teacher SHA、CPU单线程、唯一TeacherActuator、实际加载f06 SLAM库、181个归档源和真实SLAM来源检查通过。Actor仍用232维特权输入；导航未使用Gazebo真值。
- 流水线139102 accepted=delivered=committed，0canceled、0capacity reject、0closed reject，实际ROS context有效时完成排空。完整46任务因此失败，不能由流水线排空通过改称导航通过。

全运行量化事实：实际SLAM header30.302897Hz、独立数学有效更新3487次/212.950s≈16.37004Hz（行走/转向/停车门导致未每帧更新，不能当作输入帧率）；路线cross max0.092499m/RMS0.016445m，速度PI误差MAE[x,y,w]=[0.081256,0.029181,0.125212]。这些不是完整路线通过指标。源header频率和wall频率必须分开：同6879pose窗口wall22.338302Hz、仿真/墙钟比例0.737167，见 TIMING_REFERENCE；不能据此独断算法或系统通信耗时。

![实际失败窗口](V19_9779_failed_region_curves.png)

证据：

- [独立任务验收](V19_9779_INDEPENDENT_MISSION_EVALUATION.json)：总体FAILED，未执行项和未完整重放项保持UNVERIFIED。
- [8个原区域收据](V19_9779_PARTIAL_ORIGINAL_REGION_RECEIPTS.json)
- [首失败时钟/源码因果窗口](V19_9779_FIRST_FAILURE_CAUSAL_WINDOW.json)
- [104次切线与门复位审计](V19_9779_HEADING_RESET_PROJECTION_AUDIT.json)
- [流式原记录事实](V19_9779_BOUNDED_RUN_FACTS.json)
- [原目标窗口与选定原行字节引用](V19_9779_FAILED_REGION_OBSERVATION.json)

原始运行目录保持只读，所有新增报告与收据只写 evaluation/。图只用于观察，不替代完整安全、来源或数学验收。本轮没有重新训练、操作真机或修改通过的camera Demo。
'''.replace('TIMING_REFERENCE',timing_ref)
(HERE/'V19_9779_FAILURE_REPORT.md').write_text(report)
names=['V19_9779_INDEPENDENT_MISSION_EVALUATION.json','V19_9779_PARTIAL_ORIGINAL_REGION_RECEIPTS.json','V19_9779_FIRST_FAILURE_CAUSAL_WINDOW.json','V19_9779_HEADING_RESET_PROJECTION_AUDIT.json','V19_9779_BOUNDED_RUN_FACTS.json','V19_9779_FAILED_REGION_OBSERVATION.json','V19_9779_failed_region_curves.png','V19_9779_FAILURE_REPORT.md']
create('V19_9779_REPORT_FILES_MANIFEST.json',dict(schema='teacher_original46_report_files/v1',run=str(RUN.resolve()),run_id=RUN.name,files={n:dict(sha256=digest(HERE/n),bytes=(HERE/n).stat().st_size)for n in names},writers_stopped=True))
print(json.dumps(dict(partial_regions_valid=valid,first_failed_publication_sim_s=first['record']['publish_ros_clock_ns']/1e9,report=str(HERE/'V19_9779_FAILURE_REPORT.md'),manifest_SHA=digest(HERE/'V19_9779_REPORT_FILES_MANIFEST.json'))))
