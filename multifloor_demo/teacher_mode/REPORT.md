# Go2 Teacher 仿真验证报告

整体状态：**failed**。此报告读取原始逐轮收据，不运行仿真或重新评估，不挑选通过轮次。

| 层级 | 整体结果 |
|---|---|
| interface | passed |
| motion | failed |
| sim2sim | failed |
| navigation | unverified |
| real_robot | unverified |

motion_v2 从 `20261003_193541` 起，共要求14类×3次=42独立轮。当前基准轮 42，完成 42，失败 3；独立进程=True，配置一致=True。

| 正式测试 | 完成 / 要求 | 通过 | 失败 | 结果 |
|---|---:|---:|---:|---|
| stand | 3 / 3 | 3 | 0 | passed |
| forward | 3 / 3 | 3 | 0 | passed |
| backward | 3 / 3 | 3 | 0 | passed |
| left | 3 / 3 | 3 | 0 | passed |
| right | 3 / 3 | 3 | 0 | passed |
| turn_positive | 3 / 3 | 3 | 0 | passed |
| turn_negative | 3 / 3 | 3 | 0 | passed |
| walk_stop | 3 / 3 | 3 | 0 | passed |
| command_timeout | 3 / 3 | 3 | 0 | passed |
| switch | 3 / 3 | 3 | 0 | passed |
| ramp_up | 3 / 3 | 3 | 0 | passed |
| ramp_down | 3 / 3 | 3 | 0 | passed |
| step05 | 3 / 3 | 0 | 3 | failed |
| step10 | 3 / 3 | 3 | 0 | passed |

原起点额外场景门：**failed**，该场景导航不获准。它不能因被排除出远离overhead的基准出生点而被忽略。

平地九类相同命令对照子门：**passed**；地形对照：**failed**。
稳态共同窗口为5–10秒（switch按三段），停车使用两环境原生记录窗口交集；比较真实command profile、三轴均值、RMSE及停车RMS，差值限制0.1 m/s、0.12 rad/s。不能用全程平均覆盖稳态与停车失败。

实际记录247输入→冻结actor：**passed**，8305 个Teacher帧（排除45个PD初始化帧），最大动作误差 1.6093254089355469e-06，预设容差 2e-05。
最新物理快照重建247/actor的严格parity：**failed**，247最大误差 0.015912631526589394，重建输入动作最大误差 0.03318805992603302。这不表示实际记录输入的actor计算失败。重置gravity缓存和79个switch扫描帧的一周期延迟已量化，strict失败原记录保留；具体时序解释为来源与观测推断，未用替换输入掩盖失败。

已知动力学差异：Gazebo模型16.512 kg，训练参考16.087 kg；机器人摩擦0.7与1.0，地面1.0；基准关闭传感器渲染，相机/全传感器诊断另列。参考是CPU PhysX；此为受控差异，不代表两环境完全相同。

历史共 74 轮，失败 22 轮。分类可重叠：`{'assisted_stop': 6, 'insufficient_duration': 10, 'physical_fall_contact': 5, 'tracking_parking': 11, 'render_capture': 6, 'initialization': 8, 'terrain_crossing': 15, 'process_signal_unknown': 9, 'transport_execution': 10, 'interface_provenance': 9}`。所有历史失败及诊断保留。

| 运行 | 测试 | interface / motion | 纳入正式 | 原因 / 排除说明 |
|---|---|---|---|---|
| 20261003_191147_stand_r1_e589 | stand | unverified / failed | False | before_motion_v2_campaign; Invalid stand coverage: commissioning support hold replaced the Teacher; Missing complete named iface_checks receipt |
| 20261003_191440_stand_r1_4c92 | stand | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191458_forward_r1_dcae | forward | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191520_backward_r1_bb16 | backward | unverified / failed | False | before_motion_v2_campaign; Test ended before required duration; fallen_or_low_clearance; Roll/pitch exceeds criterion; Stand/stop translation drift; Stand/stop residual velocity; Missing complete named iface_checks receipt |
| 20261003_191541_left_r1_5e87 | left | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191602_right_r1_8a9d | right | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191624_turn_positive_r1_d76e | turn_positive | unverified / failed | False | before_motion_v2_campaign; Stand/stop translation drift; Stand/stop residual velocity; Missing complete named iface_checks receipt |
| 20261003_191645_turn_negative_r1_a512 | turn_negative | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191707_walk_stop_r1_ef29 | walk_stop | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191728_command_timeout_r1_e5b5 | command_timeout | unverified / failed | False | before_motion_v2_campaign; Test ended before required duration; fallen_or_low_clearance; Roll/pitch exceeds criterion; Stand/stop translation drift; Stand/stop residual velocity; Missing complete named iface_checks receipt |
| 20261003_191746_switch_r1_1f6f | switch | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191815_ramp_up_r1_2ae8 | ramp_up | unverified / failed | False | before_motion_v2_campaign; nonbaseline_spawn; Test ended before required duration; fallen_or_low_clearance; No active motion sample; Incomplete tracking window 5-10s; Incomplete stand/stop window; Insufficient uphill progress; Missing complete named iface_checks receipt |
| 20261003_191821_ramp_down_r1_2757 | ramp_down | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_191843_step05_r1_32f1 | step05 | unverified / failed | False | before_motion_v2_campaign; Low step was not crossed with measured height gain; Missing complete named iface_checks receipt |
| 20261003_191906_step10_r1_31ee | step10 | unverified / failed | False | before_motion_v2_campaign; Low step was not crossed with measured height gain; Missing complete named iface_checks receipt |
| 20261003_191946_backward_r1_ec55 | backward | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy time is nonincreasing, missing its origin, or has gaps above 30ms; Invalid native evidence: No native PD steps or policy exchange; EOFError: Gazebo actuator disconnected; Native termination/damping acknowledgement missing; Policy or Gazebo did not exit normally; Operational interface checks failed: policy_50hz, cpu_inference_samples, native_timing_and_transport, actual_pd_dcmotor_curve, actual_teacher_target_execution, actual_run_end; Test ended before required duration; No active motion sample; Incomplete tracking window 5-10s; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'maximum_gap_s': None, 'maximum_period_error_s': None}; Interface check failed: {'passed': False, 'samples': 1, 'worker_samples': 1, 'p95_ms': 0.438245}; Interface check failed: {'passed': False, 'error': 'No native PD steps or policy exchange'}; Interface check failed: {'passed': False}; Interface check failed: {'passed': False, 'samples': 0, 'target_max_error_rad': None, 'excluded_assisted_phases': ['support_capture', 'support_hold']}; Interface check failed: {'passed': False, 'native_terminate': False, 'terminal_time_alignment': False, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_191955_turn_positive_r1_aebf | turn_positive | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy time is nonincreasing, missing its origin, or has gaps above 30ms; Invalid native evidence: No native PD steps or policy exchange; EOFError: Gazebo actuator disconnected; Native termination/damping acknowledgement missing; Policy or Gazebo did not exit normally; Operational interface checks failed: policy_50hz, cpu_inference_samples, native_timing_and_transport, actual_pd_dcmotor_curve, actual_teacher_target_execution, actual_run_end; Test ended before required duration; No active motion sample; Incomplete tracking window 5-10s; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'maximum_gap_s': None, 'maximum_period_error_s': None}; Interface check failed: {'passed': False, 'samples': 1, 'worker_samples': 1, 'p95_ms': 0.463472}; Interface check failed: {'passed': False, 'error': 'No native PD steps or policy exchange'}; Interface check failed: {'passed': False}; Interface check failed: {'passed': False, 'samples': 0, 'target_max_error_rad': None, 'excluded_assisted_phases': ['support_capture', 'support_hold']}; Interface check failed: {'passed': False, 'native_terminate': False, 'terminal_time_alignment': False, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_192407_stand_r1_2370 | stand | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy time is nonincreasing, missing its origin, or has gaps above 30ms; Invalid native evidence: No native PD steps or policy exchange; EOFError: Gazebo actuator disconnected; Native termination/damping acknowledgement missing; Policy or Gazebo did not exit normally; Operational interface checks failed: policy_50hz, cpu_inference_samples, native_timing_and_transport, actual_pd_dcmotor_curve, actual_teacher_target_execution, actual_run_end; Test ended before required duration; No active motion sample; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'maximum_gap_s': None, 'maximum_period_error_s': None}; Interface check failed: {'passed': False, 'samples': 1, 'worker_samples': 1, 'p95_ms': 0.423115}; Interface check failed: {'passed': False, 'error': 'No native PD steps or policy exchange'}; Interface check failed: {'passed': False}; Interface check failed: {'passed': False, 'samples': 0, 'target_max_error_rad': None, 'excluded_assisted_phases': ['support_capture', 'support_hold']}; Interface check failed: {'passed': False, 'native_terminate': False, 'terminal_time_alignment': False, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_192412_forward_r1_c35f | forward | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy time is nonincreasing, missing its origin, or has gaps above 30ms; Invalid native evidence: No native PD steps or policy exchange; EOFError: Gazebo actuator disconnected; Native termination/damping acknowledgement missing; Policy or Gazebo did not exit normally; Operational interface checks failed: policy_50hz, cpu_inference_samples, native_timing_and_transport, actual_pd_dcmotor_curve, actual_teacher_target_execution, actual_run_end; Test ended before required duration; No active motion sample; Incomplete tracking window 5-10s; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'maximum_gap_s': None, 'maximum_period_error_s': None}; Interface check failed: {'passed': False, 'samples': 1, 'worker_samples': 1, 'p95_ms': 0.387728}; Interface check failed: {'passed': False, 'error': 'No native PD steps or policy exchange'}; Interface check failed: {'passed': False}; Interface check failed: {'passed': False, 'samples': 0, 'target_max_error_rad': None, 'excluded_assisted_phases': ['support_capture', 'support_hold']}; Interface check failed: {'passed': False, 'native_terminate': False, 'terminal_time_alignment': False, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_192418_ramp_up_r1_d76e | ramp_up | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy time is nonincreasing, missing its origin, or has gaps above 30ms; Invalid native evidence: No native PD steps or policy exchange; EOFError: Gazebo actuator disconnected; Native termination/damping acknowledgement missing; Policy or Gazebo did not exit normally; Operational interface checks failed: policy_50hz, cpu_inference_samples, native_timing_and_transport, actual_pd_dcmotor_curve, actual_teacher_target_execution, actual_run_end; Test ended before required duration; No active motion sample; Incomplete tracking window 5-10s; Incomplete continuous Teacher stand/stop window; Terrain evidence invalid: Missing settled terrain start/end window; Interface check failed: {'passed': False, 'maximum_gap_s': None, 'maximum_period_error_s': None}; Interface check failed: {'passed': False, 'samples': 1, 'worker_samples': 1, 'p95_ms': 0.554382}; Interface check failed: {'passed': False, 'error': 'No native PD steps or policy exchange'}; Interface check failed: {'passed': False}; Interface check failed: {'passed': False, 'samples': 0, 'target_max_error_rad': None, 'excluded_assisted_phases': ['support_capture', 'support_hold']}; Interface check failed: {'passed': False, 'native_terminate': False, 'terminal_time_alignment': False, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_192611_stand_r1_4b29 | stand | failed / failed | False | before_motion_v2_campaign; camera_only_diagnostic; Policy or Gazebo did not exit normally; Operational interface checks failed: actual_run_end; Interface check failed: {'passed': False, 'native_terminate': True, 'terminal_time_alignment': True, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_192612_backward_r1_dc6a | backward | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_192633_turn_positive_r1_b04b | turn_positive | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_192655_ramp_up_r1_227d | ramp_up | unverified / passed | False | before_motion_v2_campaign; Missing complete named iface_checks receipt |
| 20261003_192716_step05_r1_eb06 | step05 | unverified / failed | False | before_motion_v2_campaign; Low step was not crossed with measured height gain; Missing complete named iface_checks receipt |
| 20261003_192740_step10_r1_3479 | step10 | unverified / failed | False | before_motion_v2_campaign; Low step was not crossed with measured height gain; Missing complete named iface_checks receipt |
| 20261003_193024_stand_r1_de2f | stand | unverified / unverified | False | before_motion_v2_campaign |
| 20261003_193541_stand_r1_bc7f | stand | passed / passed | True |  |
| 20261003_193559_forward_r1_ef2b | forward | passed / passed | True |  |
| 20261003_193621_backward_r1_f1eb | backward | passed / passed | True |  |
| 20261003_193643_left_r1_b14c | left | passed / passed | True |  |
| 20261003_193705_right_r1_bb3d | right | passed / passed | True |  |
| 20261003_193726_turn_positive_r1_63a1 | turn_positive | passed / passed | True |  |
| 20261003_193748_turn_negative_r1_8b75 | turn_negative | passed / passed | True |  |
| 20261003_193810_walk_stop_r1_bc73 | walk_stop | passed / passed | True |  |
| 20261003_193831_command_timeout_r1_04e9 | command_timeout | passed / passed | True |  |
| 20261003_193853_switch_r1_538d | switch | passed / passed | True |  |
| 20261003_193923_ramp_up_r1_a1ec | ramp_up | passed / passed | True |  |
| 20261003_193944_ramp_down_r1_08d7 | ramp_down | passed / passed | True |  |
| 20261003_194006_step05_r1_7b46 | step05 | passed / failed | True | ; Low step tread was not reached with settled measured height gain |
| 20261003_194022_stand_original_origin_r1_dc76 | stand | failed / failed | False | spawn_override_diagnostic; camera_only_diagnostic; nonbaseline_spawn; fallen_or_low_clearance; Native actuator fault: inference_requested_fault_damping; Native physics steps contain a latched fault; Policy or Gazebo did not exit normally; Operational interface checks failed: actual_run_end; Test ended before required duration; No active motion sample; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'native_terminate': True, 'terminal_time_alignment': True, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_194035_step10_r1_b04c | step10 | passed / passed | True |  |
| 20261003_194104_stand_r2_126f | stand | passed / passed | True |  |
| 20261003_194122_forward_r2_65b6 | forward | passed / passed | True |  |
| 20261003_194144_backward_r2_45de | backward | passed / passed | True |  |
| 20261003_194206_left_r2_3273 | left | passed / passed | True |  |
| 20261003_194227_right_r2_3bbb | right | passed / passed | True |  |
| 20261003_194249_turn_positive_r2_8d67 | turn_positive | passed / passed | True |  |
| 20261003_194311_turn_negative_r2_12d6 | turn_negative | passed / passed | True |  |
| 20261003_194332_walk_stop_r2_ba65 | walk_stop | passed / passed | True |  |
| 20261003_194354_command_timeout_r2_1969 | command_timeout | passed / passed | True |  |
| 20261003_194416_switch_r2_0e60 | switch | passed / passed | True |  |
| 20261003_194445_ramp_up_r2_32bf | ramp_up | passed / passed | True |  |
| 20261003_194507_ramp_down_r2_73a0 | ramp_down | passed / passed | True |  |
| 20261003_194529_step05_r2_6f0b | step05 | passed / failed | True | ; Low step tread was not reached with settled measured height gain |
| 20261003_194558_step10_r2_8974 | step10 | passed / passed | True |  |
| 20261003_194626_stand_r3_aeff | stand | passed / passed | True |  |
| 20261003_194645_forward_r3_e3c2 | forward | passed / passed | True |  |
| 20261003_194707_backward_r3_a216 | backward | passed / passed | True |  |
| 20261003_194728_left_r3_13bd | left | passed / passed | True |  |
| 20261003_194750_right_r3_baa5 | right | passed / passed | True |  |
| 20261003_194812_turn_positive_r3_6c25 | turn_positive | passed / passed | True |  |
| 20261003_194833_turn_negative_r3_1b49 | turn_negative | passed / passed | True |  |
| 20261003_194855_walk_stop_r3_6a7f | walk_stop | passed / passed | True |  |
| 20261003_194917_command_timeout_r3_e733 | command_timeout | passed / passed | True |  |
| 20261003_194939_switch_r3_18f5 | switch | passed / passed | True |  |
| 20261003_195008_ramp_up_r3_a3b7 | ramp_up | passed / passed | True |  |
| 20261003_195030_ramp_down_r3_5222 | ramp_down | passed / passed | True |  |
| 20261003_195052_step05_r3_b4ba | step05 | passed / failed | True | ; Low step tread was not reached with settled measured height gain |
| 20261003_195120_step10_r3_18d2 | step10 | passed / passed | True |  |
| 20261003_200031_forward_sensor_shadow_r1_c28c | forward | failed / failed | False | full_sensor_diagnostic; Policy or Gazebo did not exit normally; Operational interface checks failed: actual_run_end; Interface check failed: {'passed': False, 'native_terminate': True, 'terminal_time_alignment': True, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0, 0] |
| 20261003_200329_stand_original_origin_boundary_fix_r1_23a2 | stand | passed / failed | False | spawn_override_diagnostic; nonbaseline_spawn; fallen_or_low_clearance; Native actuator fault: inference_requested_fault_damping; Native physics steps contain a latched fault; Test ended before required duration; No active motion sample; Incomplete continuous Teacher stand/stop window |
| 20261003_200820_forward_overview_boundary_fix_r1_1db0 | forward | failed / failed | False | spawn_override_diagnostic; camera_only_diagnostic; Policy or Gazebo did not exit normally; Operational interface checks failed: actual_run_end; Interface check failed: {'passed': False, 'native_terminate': True, 'terminal_time_alignment': True, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0] |
| 20261003_201443_forward_hardware_renderer_check_r1_40c6 | forward | passed / passed | False | spawn_override_diagnostic; full_sensor_diagnostic |

## 地形与原起点的实际对照

| 测试 | CPU参考原数值门 | 测量 / 失败原因 | Gazebo正式结果 |
|---|---|---|---|
| ramp_up | True | progress=1.9831771850585938 / 1.5 m;  | passed |
| ramp_down | False | progress=1.4728107452392578 / 1.5 m; Ramp settled progress below unchanged1.5m requirement | passed |
| step05 | False | x=8.558756828308105 / 8.4 m; height_gain=0.024344131350517273 / 0.03 m; Step settled endpoint or body height gain below unchanged criterion | failed |
| step10 | True | x=8.540679931640625 / 8.4 m; height_gain=0.06531096994876862 / 0.06 m;  | passed |

Gazebo原起点 `20261003_194022_stand_original_origin_r1_dc76`：2.02 s，位移 [0.1439674231180907, -0.5506854441715338, -0.29676957245448987] m，yaw变化 0.9365287391612119 rad；fallen_or_low_clearance; Native actuator fault: inference_requested_fault_damping; Native physics steps contain a latched fault; Policy or Gazebo did not exit normally; Operational interface checks failed: actual_run_end; Test ended before required duration; No active motion sample; Incomplete continuous Teacher stand/stop window; Interface check failed: {'passed': False, 'native_terminate': True, 'terminal_time_alignment': True, 'main_process_returncodes': [0, -11], 'runner_error': None}; process returncodes [0, -11, 0, 0]。安全停车/原生故障锁存及提前结束使该场景interface失败，不能用正式基准42轮interface通过覆盖它。

Gazebo原起点 `20261003_200329_stand_original_origin_boundary_fix_r1_23a2`：2.02 s，位移 [0.1439674231180907, -0.5506854441715338, -0.29676957245448987] m，yaw变化 0.9365287391612119 rad；fallen_or_low_clearance; Native actuator fault: inference_requested_fault_damping; Native physics steps contain a latched fault; Test ended before required duration; No active motion sample; Incomplete continuous Teacher stand/stop window。安全停车/原生故障锁存及提前结束使该场景interface失败，不能用正式基准42轮interface通过覆盖它。

CPU原起点：61帧 / 1.22 s，实际失稳=True，下面支持面间隙 0.1475372314453125 m，roll -0.7216825485229492 rad，位置 [-0.5016859769821167, 0.2176840901374817, 0.1475372314453125]。初始187scan真实记录范围 [-2.5000014305114746, -0.09999808669090271]，overhead命中 102。
CPU原起点单项关闭了native scan-height termination以观察实际物理跌倒；actor保持+20m射线含上层遮挡。默认native高度终止的另存AST反事实重放不是实际rollout。扫描几何误差、接触缺失等原始字段完整保留在验收JSON，不能用地形参考的小误差覆盖原起点几何差异。

## 版本与失败归因

全部源文件SHA按轮次分组存入JSON。较晚worker版本只增强未执行的导航命令校验，传感器/渲染分支影响诊断轮；基准运动协议、物理、247构造与actor没有变化，此解释来自root协调记录，版本哈希不隐藏。
分类是收据文本标签且可重叠。进程signal -11本身归为未知进程信号，不能独自推断渲染根因；未记录图像为unverified_missing。历史辅助停车、初始化和时间不足失败继续保留，后续通过不抹去旧失败。

导航未执行、未验证；必须使用SLAM位置和实测点云，保持原46区域门槛。真机部署未验证，本任务不操作实体机器狗。坡道和低台阶结果不能写成真实楼梯通过。

部署射线XY边界1e-9扩张已修正；42轮40692实际记录状态的247构造回放差0，旧构造与旧记录也差0，回归收据保存在ray_boundary_correction。修正后原起点实际复测仍失稳，不能放行导航。

真实IMU/关节/LiDAR/RGB旁路在sensor_shadow_diagnostics记录，SLAM与注册点云缺失不补真值。软件Ogre1退出崩溃历史保留；最后NVIDIA/Ogre2真实带传感器前进停车运行全部退出0，Actor继续CPU，图形显存增量约156MiB。只证明该传感器运行，不证明策略观测替换或导航。

完整逐轮哈希、对照窗口与数据差异见同目录验收JSON。此报告如标preview仅用于审阅，不能被启动器当正式放行收据。
