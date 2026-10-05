# Go2 Frozen Teacher：Gazebo 适配与 Sim2Sim 验证

这是独立仿真控制模式。冻结Teacher通过唯一关节力矩执行器实际控制Gazebo Go2；CPU单线程、策略50Hz、关节PD200Hz。没有重新训练、操作实体机器人、停止另一任务训练/评估或修改已通过的camera_mode。模型与逐轮执行源码、失败、配置、物理日志和相机均归档。

## 最新：队列与算法并行已经实现并实际测试

V18独立组合版（LIO Jacobian行4线程＋VIO块重组1线程）重新通过有限数值回归后，实际完成**32/32真实SLAM区域、两条12m上行坡道和首固定5s停车**：264.60s、13231次CPU推理，停车XY漂移2.14mm。原common/v2/ramp收据保留UNVERIFIED；新增metadata-v2联合审计只修正已证明的归档字段关联，并闭合所有原门与源字节。导航不使用真值。

局部算子有收益，但**实际整条SLAM未证明显著加速**：210s对照中LIO墙钟均值9.127→9.293ms、VIO整帧2.555→2.545ms，源年龄p95均40ms；VIO四线程尾延迟门失败，最终选一线程。不能把行算子约72%或视觉有限算子24–30%的改善称作整链同幅度改善。

独立V17接收/解码FIFO已实际运行60s。已提交项的顺序、原时间戳、线程所有权和容量通过，但**退出时31615项接收仅31614项提交、1项取消，按预定零取消门失败**。该项类型未被记录，不能豁免；队列版仍为实验候选，未合入默认导航。IMU→LIO→VIO仍由唯一状态线程依次提交，后台接收/解码与其重叠。

[本轮报告、失败与操作命令](test_results/parallel_pipeline_20261005/README.md)、[具体瓶颈与实现](test_results/parallel_pipeline_20261005/IMPLEMENTATION_AND_BOTTLENECKS.md)、[实际性能对照](test_results/parallel_pipeline_20261005/performance_actual/REPORT.md)、[实际画面与路线](http://127.0.0.1:8768/?run=20261005_234102_closed_loop_cascade_clock_hold_combined_v18_full_r1_ab53)。本轮新payload位于`/var/tmp/go2_teacher_parallel_20261005/`，独立外部清单与项目PACKAGE_MANIFEST一起核对；旧外部库不改写。Actor仍232维特权＋15维已知输入，指定静态路线单次通过不覆盖原46区、当前版动态障碍、全感知Actor、新律Isaac闭环Sim2Sim或真机。三层由坡道连接。

## 历史：64线30 Hz、V12逐点并行与实际多层路线

独立V12已实现LIO残差逐点4线程、独立byte标志和移除共享锁，保留原数学表达式、原点序汇总与顺序LIO/VIO状态校正。CPU单线程Teacher保持50Hz。实际完整run `20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb` 在275.18s完成 **32/32真实SLAM区域、两条12m上行坡道和首固定5s主动停车**；停车XY漂移5.38mm。SLAM实测30.2998Hz，源年龄最大60ms，控制器实际数学更新17.6836Hz；20Hz墙钟ticker单列。没有放宽300ms期限或原路线/姿态/速度/接触门。

新只读metadata v2审计闭合provider来源关联、全部必需门和48份源字节，支持本轮指定静态路线联合通过。原common/v2/ramp收据中的UNVERIFIED以及所有旧失败原字节保留；这是事后读取器修正，未把旧结果改成PASS。三层由坡道连接。Actor仍232维特权状态/高度与15维已知命令/上一动作，导航位置/速度/朝向与到达只取实际SLAM/IMU/点云/SCAN；真值只供离线验收。本轮单次通过不覆盖原46区域全任务、当前配置动态障碍、全感知Actor、新律Isaac闭环Sim2Sim或真机。

[实际画面、路线与独立结果](http://127.0.0.1:8768/?run=20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb)、[完整路线报告](test_results/lidar_density_rate_20261005/evaluation/FULL_REPORT.md)、[采样/线程/CPU/GPU测量](docs/LIDAR_DENSITY_RATE_REPORT_20261005.md)、[队列流水线审计](test_results/lidar_density_rate_20261005/pipeline_audit/QUEUE_ENGINEERING_AUDIT.md)、[算法并行审计](test_results/lidar_density_rate_20261005/pipeline_audit/ALGORITHM_PARALLEL_AUDIT.md)与[续接命令](docs/RESUME_LIDAR_PERFORMANCE_20261005.md)。截至本历史段归档时，新增队列、Jacobian/VIO并行和IMU区间扫描仅有设计审计；当时已运行版本仅V12上述逐点优化。后续V15–V18实现与实测见上方最新章节。4/8线程和本任务分核均已实测，8线程未优于4线程，分核未见吞吐收益；不改其他系统/训练任务。

新日志从创建起位于 `/var/tmp/go2_teacher_simulation_20261005/`，项目runs为同名alias；外部65,469文件/55,004,007,938 bytes由[独立库存](test_results/lidar_density_rate_20261005/EXTERNAL_STORAGE_MANIFEST.json)绑定。复制研究包应同时复制外部payload和核对PACKAGE_MANIFEST，不能只复制alias。当前无本任务运动控制器运行，8768只读回放保留。下面按原时间/合同保留历史结论，不以本轮通过回填。

## 2026-10-05 傍晚：高度失效已定位到具体错误墙面约束

独立V7诊断模式实际重跑210秒，原controller、路线和300mm/300ms保护保持。首次高度保护在133.13秒，仍仅16/32区域，整体FAILED。131–134秒估计内部高度下降336.5mm，而native机身只下降3.7mm、没有跌倒；早期单扫描环的真实竖直东墙被拟合成近水平面，约6秒后停止更新，当前墙点仍被旧法向接受。限定单迭代重算确认其向下速度修正贡献，尚未实施在线修复或取得新的多层通过。

[详细故障报告、曲线及复现命令](docs/SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)、[独立诊断源码/构建入口](navigation/closed_loop_multifloor_v7/README.md)、[真实Gazebo双相机与SLAM/SCAN失败回放](http://127.0.0.1:8768/?run=20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf)。日志159757条、0丢失，原core/ros和V1–V6未改；Teacher仍CPU单线程，导航不接真值，Actor232维特权来源仍明示。下一轮优先处理平面二维支撑、法向可信度和成熟/冻结机制，neighbor量纲bug另立对照；提高PID频率或加回环不能替代当前帧误约束的修正。

## 2026-10-05 下午：实际 SLAM / SCAN 与坡道

路径/朝向 PD＋速度 PI 已接入实际 SLAM/IMU 和 SCAN。单次完整 **12 m ramp12、14/14 区域与首 5 秒停车正式通过**，实际停车 XY 漂移 4.55 mm。平地三轮为 2 通过、1 停车失败；连续三层 V4/V5 仅 16/32 区域，二层 SLAM 高度误差约 22 cm，仍失败。V5 开在线重力/bias、V6 恢复视觉权重都未消除高度失效；不能因此称完整多层导航或 Sim2Sim 通过。

最新入口：[实际闭环多层报告](docs/CLOSED_LOOP_SCAN_MULTIFLOOR_REPORT_20261005.md)、[初始化/匹配/彩色云/回环审计](docs/SLAM_INITIALIZATION_MATCHING_AUDIT_20261005.md)。[单坡道正式结果](http://127.0.0.1:8768/?run=20261005_145244_closed_loop_cascade_segmented_ramp12_r4_4791)与[最新视觉权重对照](http://127.0.0.1:8768/?run=20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6)显示实际两路 Gazebo 图像、原 D/K 和实际 SLAM/SCAN 路线。当前 Actor 232 维特权输入仍明示，真值不进入导航；30 维传感器替换尚未与本批导航联测。

V6 已实际完成原 600 秒生命周期、30001 次 CPU 推理，131.7 秒的 SLAM 高度管道误差 306.505 mm 触发原 300 mm 保护，后续持续站立正常；运行完成不替代导航通过，正式收据独立追加。三层连接是坡道，真机未验证。下面历史章节按原时间/合同保留，以各 run 的冻结输入和独立收据区分作用域。

## 2026-10-05 09:08 阶段结论（历史）

|范围|实际结果|边界|
|---|---|---|
|接口|通过|247维观测、CPU推理、显式关节映射、唯一执行器、异常/300ms保护|
|真值PID选择|A2/B/C共27次通过|路径/朝向PD＋COM速度PI；25Hz是有限平地候选中选定值，非全局最优|
|冻结后Gazebo运动|D/D2共35次通过|完整ramp12/23上下坡各3次、5/10cm持续登台各3次、平地/折线/往返/0.5–1.0m/s；真值反馈|
|真实SLAM迁移|6次独立通过|实际SLAM/IMU反馈约10Hz，6m单程及6m＋6m往返各3次；初始机身相对固定路线，非旧地图注册|
|转弯包络|36次通过|0.3/0.5/1.0输入速度×左右转，最小已测边界各3次；独立60×60m平面|
|连续圆圈/S转向|行驶14/14，含停车13/14通过|R0.6左右各3次、R1.2左右各1次、两种S各10/25/50Hz；tightS25停车yaw 0.114522rad超0.1门|
|新tightS25主动停车|先导＋三次新确认4/4通过|固定终点位置/朝向PD＋COM速度PI；25Hz真值反馈；首5s XY漂移4.45mm、yaw0.00241rad；靠近14.12s仍偏慢|
|完整Sim2Sim|未通过|旧开环/原起点/缓存时序失败保留；新PID和曲线律未做对应Isaac闭环对照|
|完整导航|未验证|历史有限SLAM/SCAN/动障碍另有通过收据；新PID/曲线尚未合并SCAN、旧地图注册和多楼层|
|真机|未验证|本任务仅仿真|

最新报告：[固定终点主动停车](docs/ACTIVE_PARKING_REPORT_20261005.md)、[真值PID标定](docs/TRUTH_PID_CALIBRATION_REPORT_20261005.md)、[真实SLAM迁移](docs/SLAM_CONTROLLER_TRANSFER_REPORT_20261005.md)、[实测转弯半径/圆圈/S](docs/CURVATURE_ENVELOPE_REPORT_20261005.md)。接续入口为[下一轮交接](docs/RESUME_ACTIVE_PARKING_20261005.md)。曲线的实际200Hz轨迹、速度、朝向及失败见[14轮图表报告](test_results/curvature_campaign_20261005/curve_analysis/REPORT.md)与[新停车对照曲线](test_results/heartbeat_resume_20261005_0800/analysis/analysis_v4r2_confirm3/REPORT.md)。新控制器Actor仍使用232维原生特权状态/高度及15维命令/上一动作；实际IMU/关节30维替换的独立通过未与这些路线联测。

当前已测最小稳定实际半径：Teacher输入0.3m/s左/右约0.355/0.366m；0.5m/s约0.603/0.618m；1.0m/s约1.285/1.297m。它们是|wz|≤0.8的有限已测边界，非物理最小值，也不是准确实测速度固定下的极限。0.3m/s规划先留1.5倍余量，采用0.6m圆圈已实际验证；更高速度的曲线未测试。S是解析连续反向转弯/侧移返回路径，几何公式及弧区频率对照详见新报告。

紧圆的入/出弧误差与曲率前馈跳变、最终wz变化率限制同时出现，曲率渐变连接A/B尚未实施。原v1停车时外/内PI绕过输出零，Teacher继续零速度推理；tightS25的原失败不能解释为25Hz停车闭环效果差。新增V4先有界靠近固定终点、再弱位置/朝向保持，允许小幅非零Teacher命令，首个固定5秒窗独立验收4/4通过；原零命令合同与失败不回填。V3弱捕获180秒失败、V4缺可选观察器归档的三条未验证保留，V4r2仅修归档后另跑三次确认。本轮共8次真实物理运行及1次未启动物理的失败，不能称8次通过。当前新保持只测25Hz真值，实际SLAM曲线/SCAN及多层未验证。

新[相机先导/主动停车/实际曲线](http://127.0.0.1:8769/?run=20261005_084153_truth_pid_S_tight_active_hold_capture_p2_curvature_cascade_pi_active_hold_25hz_4788)在43秒有实际两路Gazebo RGB、原始stamp及D/K；页面独立显示新收据与原common FAILED，不改旧结论。08点单次任务已确认PAUSED；资源空闲时V3仍失效，不预设GPU是唯一原因。

真实归档浏览器：[gentle S中段/曲线](http://127.0.0.1:8769/?run=20261005_023202_truth_pid_S_gentle_curvature_cascade_pi_25hz_5c8b)、[tightS25失败](http://127.0.0.1:8769/?run=20261005_023511_truth_pid_S_tight_curvature_cascade_pi_25hz_0b78)、[实际SLAM往返](http://127.0.0.1:8770/?run=20261005_014940_SLAM_fixed_route_v3_roundtrip6m_mapped10_readtime_v5_r3_2e3f)。新平面固定旁观相机在12–20s可见机器狗，S末段走出视野的空帧保留；两路实际D全零，不生成替代影像。页面均只读回放；原8768旧SLAM/SCAN面板及8767相机Demo保留独立。

复现新测试会新建run；已保存的收据不会覆盖。以下从teacher_mode目录执行：

```bash
# 新固定终点主动停车；另建run，使用与V4相同控制律的归档修正版
python3 -B navigation/curvature_tracking_v4r2/run.py --profile test_results/heartbeat_resume_20261005_0800/profiles_v4r2/00_confirmation_tightS25_r1.json --camera

# 新run独立验收；旧共同零命令合同与新主动保持分开记录
python3 -B runs/新run目录/sources/truth/active_hold_receipt.py --run runs/新run目录

# 实际33s固定输入转弯边界；无--prepare-only才运行仿真
python3 -B navigation/curvature_tuning/run.py --plan test_results/curvature_campaign_20261005/boundary_plans_v3/v0.3_wp0.8_r2.json

# 实际gentle S；Actor始终CPU，保存两路Gazebo RGB
python3 -B navigation/curvature_tracking/run.py --profile test_results/curvature_campaign_20261005/curve_profiles_v1/04_S_gentle_25hz_r1.json --camera

# 新run完成后使用它的归档分析器追加独立收据，已有收据拒绝覆盖
python3 -B runs/新run目录/sources/truth/curve_receipt.py --run runs/新run目录

# 只读页，与实际SLAM页8770区分
python3 -B navigation/truth_tuning_design/dashboard.py --port 8769
```

机器可读最新结论为current_status.json；包清单PACKAGE_MANIFEST.json逐文件记录当前与执行归档哈希，并保存旧清单。原runs/acceptance.json保持原字节，不因新真值运动通过而提升历史总体验收。

## 2026-10-04历史接续结果（原失败保留）

旧开环完整12米ramp23失败、早期V4/V5六米PID路线失败，均保留原始收据。新冻结真值PID完整上下坡通过是另立试验，未完成真实SLAM多层导航或新律Isaac对照。旧V4短往返Teacher3/3、CHAMP0/3满足全项，仅属于该同条件短场景，不能据此宣称普遍优于CHAMP。原有限SLAM/SCAN往返四次和V4.5a动态障碍240秒19项通过，未与新连续曲线controller合并。

以下保留此前功能复测、接口、操作命令和历史原件说明；提到“完整坡道失败”的旧段落均指上述旧开环/旧控制版，其验收不回填。

## 低台阶功能复测

按用户要求直接持续给机身前进命令0.3 m/s，以真实顶面接触位置和法向、四脚分别连续支撑、随后继续前进及停车判定。机身世界高度增量只作诊断，不再以其占台阶高度的比例判断是否登台。冻结协议在 `tests/step_functional_protocol.json`，逐轮结果、实际录像及版本纠正收据见 `docs/STEP_RETEST_REPORT.md`。

| 台阶 | 独立实测 | 四脚顶面支撑确认 | 登台后继续前进 | 继续段实际世界前向速度 | 停车4 s最大漂移 |
|---|---|---|---|---|---|
| 5 cm | 3/3通过 | 9.055 s | 2.90666 m | 0.23385 m/s | 0.000514 m |
| 10 cm | 3/3通过 | 8.440 s | 3.08585 m | 0.23751 m/s | 0.003651 m |

这六轮都是固定名义参数和相同初态的独立进程重复。台阶前缘仍为x=7 m，顶面长度由旧测试的2 m延长为6 m，以便验证登台后继续运动；不是穿越整块障碍、连续楼梯或导航。Teacher在停车期继续推理零速度命令，未使用action=0停车、保持关节目标或机身助力。旧记录和旧数值判据失败原样保留，不能再把旧5 cm机身升高比例失败表述为“登不上”。

首六轮观察相机被场景柱体遮住登台瞬间；随后只调整无碰撞观察相机，额外5/10 cm各跑一轮，均通过并拍清楚跨沿、继续和停车。两轮的247观测及12动作与对应主测试逐样本差值为0，影像补充未替换或筛掉任何主测试。全部八轮及原文件哈希保存在新增报告。

2026-10-04实际相机参数核验：最初overview约103°、vehicle80°；新增有限导航两路均80°，两路Gazebo CameraInfo的D均为五个0，尺寸、焦距与图像匹配；独立Teacher模式漏桥接CameraInfo已修复。宽视野透视不等于漏做镜头去畸变，详见 `docs/CAMERA_AUDIT.md`。原相机Demo未修改。

## 2026-10-04接续实测

- 相机实际D全零，车载K匹配原Demo；两路CameraInfo、车载RGB与旁观RGB已独立存档，见 `docs/CAMERA_AUDIT.md`。
- 完整ramp23上行22.9秒、下行25.48秒失败，仅推进约4.8米；显式lower12地形provider上行也在36.025秒、推进8.006米后因载荷碰坡失败。均未到出口平台，出口停车未验证。CPU Isaac匹配30秒前缀已完成，见 `docs/FULL_RAMP_REPORT.md` 与 `docs/RAMP23_PREFIX_COMPARISON.md`。该场景是坡道，不是楼梯。
- 显式地形目标集合A/B显示：原点all_static扫描失稳，下层floor1/ramp12/floor2扫描站立15秒通过；训练只扫描ground terrain mesh，全场景碰撞物并非训练强制目标。这是新的特权provider诊断，未证明SLAM分层或完整场景等价，见 `docs/TERRAIN_TARGET_TIMING_AUDIT_20261004.md`。
- 实际IMU/关节30维已切入Teacher；其余217维未替换，其中190维COM/高度仍特权、27维是控制器已知状态。优化broker原前进3轮中2通过，1在18秒末帧因消息墙龄拒绝。8点后三轮清洁资源空闲复测均通过真实来源、247维重建、CPU回放、18秒运动和停车；另一次runner清理中断不计完整通过。保持原25ms仿真/300ms墙钟阈值及无真值fallback。详细来源、坐标、因果相位与失败见 `docs/OBSERVATION_REPLACEMENT.md` 与 `docs/SENSOR_REPLACEMENT_REPORT.md`。
- 真实FAST-LIVO2实际车载RGB/IMU/LiDAR链已运行。旧导航三轮及8点资源空闲V2对照仍全零，保留失败。V4修复回调积压、单一ROS时钟、连续预热和Teacher实测停车/heading后，三个180秒独立运行均有实际SCAN/Teacher非零执行、两目标区域真实SLAM连续0.6秒到达及到达后停车，见 `docs/NAVIGATION_REPORT.md`。目标中心相距1米、控制半径0.17米；实际最大位移约0.84米，不能称为精确走满1米。

各run独立收据与原始失败均保留；原42轮验收未被功能复测或有限实验覆盖。当前浏览器另有真正SLAM/SCAN路线面板，Gazebo真值轨迹继续只作独立诊断。

## 原42次协议结果（历史记录，未覆盖）

| 层级 | 结论 | 范围 |
|---|---|---|
| 接口 | 通过 | 正式42次均有完整247观测、12动作、CPUActor、真实关节/力矩、200/50 Hz、唯一写入者及归档校验；异常保护另存证据 |
| 运动控制 | 整体验收失败 | 正式42次39通过；5 cm低台阶3次未达到冻结机身升高判据；原Demo起点额外失稳 |
| Sim2Sim | 未通过 | 已有真实 Isaac CPU匹配命令、坡道与低台阶对照；原起点在两环境失稳；严格即时快照观测对齐仍存在传感器缓存时序偏差；旧5 cm升高比例只是历史数值判据失败 |
| SLAM/SCAN导航集成 | 当时未验证 | 历史正式门禁拒绝；之后新增有限平地实际实验另列于上方，不能改写旧记录 |
| 真机部署 | 未验证 | 本任务仿真限定 |

正式测试是固定名义参数、同种子、同初态的三次独立进程重复，不代表随机化、复杂地形或真机鲁棒性。10 cm登台虽三次通过，停车漂移0.131 m、偏航0.191 rad，接近0.15 m/0.20 rad门槛。上下坡只测10%坡道中的约2 m片段；低台阶只测登上平台，不是完整穿越或真实楼梯。

完整结论见 `docs/VALIDATION_REPORT.md` 和机器可读 `runs/acceptance.json`；失败记录、对照表与图在 `runs/`、`plots/`、`test_results/`。浏览器在 [Teacher 验证页](http://127.0.0.1:8768/)，实际RGB来自 Gazebo 相机，经 ROS桥采集；Gazebo轨迹明确标为仿真真值诊断；另一独立面板只显示真实SLAM与已核对原始数据哈希的SCAN路线。原相机 Demo 使用8767，保持独立。

实际带IMU/LiDAR/车载及第三人称相机的前进停车诊断 `20261003_201443_forward_hardware_renderer_check_r1_40c6` 已通过，全部进程退出0。CPU Actor保持单线程；Ogre2使用NVIDIA渲染，峰值图形显存增量约156 MiB。软件Ogre1在退出清理时崩溃的旧记录保留，默认采用已通过的硬件渲染。

## 冻结模型与执行契约

Checkpoint：

```text
/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt
SHA256 bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34
```

`actor_state_dict/mlp.*` 构建 `247→512→256→128→12` ELU网络，直接使用Gaussian均值、无观测归一化、无动作预裁剪。推理CPU、单Torch线程，物理5 ms、策略20 ms。调用本地已存在的 Isaac Python 来使用Torch，不占用训练GPU。

训练关节FR、FL、RR、RL各hip/thigh/calf，逐名映射到`rf/lf/rh/lh`各`hip/upper_leg/lower_leg`。默认姿态为 `[-.1,.8,-1.5, .1,.8,-1.5, -.1,1,-1.5, .1,1,-1.5]`，目标 `q_default+0.25*raw_action`。PD为25/0.5，DCMotor以23.5 Nm、30 rad/s执行速度相关力矩限幅。物理关节硬限位来自本次训练归档，目标不偷偷裁剪到通用部署契约。

247维顺序：COM线速度、角速度×0.2、机身重力、实际速度命令、q−default、qd×0.05、上次已限幅应用力矩×0.01、上次raw action、187高度。前60维先可选噪声、clip±100、再scale；默认清洁评估，`--noisy`才启用归档均匀噪声。高度为yaw-only的17×11网格，y外x内，10 cm分辨率；射线从base_z+20向下，`clip(base_z-hit_z-0.5,-1,1)`。+20不加到sensor/base高度上。初版all_static provider在原点有102列击中上层楼板；其集合比训练ground mesh更宽。新增显式terrain-target manifest保持网格、+20、yaw与clip并记录选中/排除表面；不能静默改成脚下高度或宣称几何已严格训练等价。

当前策略输入是 **Gazebo特权状态 + 静态SDF碰撞几何射线**，不是已经完成SLAM/点云感知部署。动障碍与自体不纳入训练mesh式射线，缺射线按训练clip语义为−1并记录缺失数。详见 `policy/contract.json`、`docs/OBSERVATION_REPLACEMENT.md`。默认旁路不替换；显式 `--sensor-feedback` 从3秒起替换实际IMU gyro/gravity及关节q/qd共30维，保持严格因果/新鲜度，缺测锁定阻尼。

原Demo模型仍含传感器载荷，与训练模型质量和惯性不同：总质量16.512 vs16.087 kg，base质量8.336 vs7.279 kg，base COM也不同。速度读取实际Gazebo COM，不能沿用训练COM修正。报告保留碰撞形状、摩擦混合和PhysX/DART等物理差异。基准模型摩擦0.7、地板1.0，是受控测试参数；训练随机化范围只归档，未声称本轮覆盖。

## 控制权、初始化与停车

复制原SDF后仅加入 `teacher_sim::TeacherActuator` 力矩写入者，删除CHAMP、ros2_control、JTC和旧body stabilizer。只在启动时设置一次关节初态，之后不重置基座、不施加基座伺服。只读关节状态/传感器发布器不拥有执行权。

初始化默认姿态PD 0.1 s，然后Teacher零速度持续到3 s；运动命令有限幅与加速度限制。命令停止或TTL超时后，速度命令平滑归零，**Teacher仍连续推理与控制**，以实际低速度/四脚支撑确认停车。不是action=0，也不冻结最后关节姿态。早期冻结停车目标导致后退/转向失稳，代码和失败保存在 `test_results/stop_capture_v1`。

IPC每策略周期同步，200 ms墙钟无有效回复、NaN/异常目标、其他控制器、断联或机身跌倒都会锁定故障，转入关节阻尼，并记录故障码。故障阻尼仅是仿真异常处置，不能声称已平稳站立。正式42轮命令超时测试是worker内模拟生产者停发。新增真实ROS生产者中断：精确暂停唯一执行桥10sim秒，保留原stamp和300ms双龄TTL，实测停车及恢复后往返通过；专项原始收据见 `runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/summary_ttl_dropout_independent.json`。普通短暂过期事件因恢复较快，完整物理停车总项仍未验证，不能以单次注入外推所有故障。

## 本地操作命令

本机已有ROS Jazzy、Gazebo Harmonic、CMake和Isaac Python。运行器自动设置ROS域79、单线程CPU、每轮独立Gazebo分区，生成新目录并保存实际源码快照。只管理自身子进程组；不要使用按名称批量杀进程。

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_

# 编译独立执行器；不改原模型或相机Demo
bash multifloor_demo/teacher_mode/scripts/run.sh build

# 正式42次测试；所有失败保留，不能只选通过的轮次
bash multifloor_demo/teacher_mode/scripts/run.sh test --repeat 3 \
  stand forward backward left right turn_positive turn_negative \
  walk_stop command_timeout switch ramp_up ramp_down step05 step10

# 真Gazebo相机，只读显示；默认已验证的NVIDIA/Ogre2，Actor仍CPU
bash multifloor_demo/teacher_mode/scripts/run.sh test --camera-only forward

# 新功能复测：直接登台并继续走，5/10 cm各3轮；独立于旧高度比例判据
bash multifloor_demo/teacher_mode/scripts/run.sh test --camera-only --repeat 3 \
  --label functional_retest step05_continue step10_continue

# 清晰观察登台过程：只改变观察相机，不改变控制或碰撞
bash multifloor_demo/teacher_mode/scripts/run.sh test --camera-only \
  --overview-pose 8.3 .9 1.8 0 .7 -1.5707963267948966 \
  --label visible_retest step05_continue step10_continue

# 生成独立功能报告及真实相机存档回放，保留所有轮次
python3 multifloor_demo/teacher_mode/scripts/step_retest_report.py --write

# 实际传感器旁路；不启动SLAM、不替换Actor输入
bash multifloor_demo/teacher_mode/scripts/run.sh test --sensors --shadow \
  --renderer hardware --render-engine ogre2 \
  --spawn 6 -.7 .4 0 --label sensor_validation forward

# 原Demo起点适用性诊断：本轮已实际失稳
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --camera-only --spawn 0 0 .4 0 --label original_origin stand

# 浏览已有记录，不启动机器人控制
bash multifloor_demo/teacher_mode/scripts/run.sh serve --port 8768

# 对某一真实run重新评估；会更新它的summary，原始物理日志不改
bash multifloor_demo/teacher_mode/scripts/run.sh evaluate \
  multifloor_demo/teacher_mode/runs/20261003_194006_step05_r1_7b46

# 只读汇总预览；--final才写入正式acceptance，不自动解锁导航
python3 multifloor_demo/teacher_mode/scripts/aggregate.py
python3 multifloor_demo/teacher_mode/scripts/plots.py --render

# 当前仅准备导航边界，不导入ROS、不启动导航、不写速度命令
python3 multifloor_demo/teacher_mode/navigation/bridge.py --prepare
```

每个run至少包含：`world.sdf`、`frames.urdf`、`asset_manifest.json`、`policy_manifest.json`、`source_manifest.json`与`sources/`、`runtime_manifest.json`、资源前后快照、`telemetry.jsonl`、200 Hz`actuator.jsonl`、`observations_actions.npz`、`worker_result.json`、`summary.json`、控制台日志；相机运行另含真实`frames/`、`frame.jpg`、`frame_source.json`。时序同时记录相对test time与world_sim_time；对ROS配对必须用后者。应用力矩是仿真ForceCmd而非硬件力矩测量。

Native输入是PreUpdate读取上次Physics结果，ROS传感器为PostUpdate；比较实际物理采样需用 `state_physics_world_time=world_sim_time−.005`，不能把相同stamp直接当相同状态。实际900组此相位匹配q/qd误差为0；随后实际30维切入策略已单列A/B运行与失败；不能据一次相位误差0宣称所有感知部署已通过。几何边界1e−9扩张问题已修复，全部42轮40692帧247回放差0、修正后原起点仍实际失败；补丁与失败记录保存在 `test_results/raycast_boundary_v1`。

## 有限导航与当前剩余工作

正式入口仍要求全局motion/Sim2Sim与原场景门禁通过。新的 `--navigation-stack` 仅生成有限平地实验收据：固定已验证spawn、三次平地能力证据、实际publisher、模型/来源/源码hash、SLAM不可变anchor和围栏，所有全局失败保持。它授权实际测试，不是验收通过开关。

```bash
# 实际SLAM，Teacher仅站立，无导航命令
bash multifloor_demo/teacher_mode/scripts/run.sh test --slam-only-stack stand

# 实际IMU/关节30维替换，COM/高度仍特权；命令仍为冻结运动测试
bash multifloor_demo/teacher_mode/scripts/run.sh test --sensor-feedback forward

# 有限真实SLAM/SCAN一米往返；未完成时保留failed/unverified
bash multifloor_demo/teacher_mode/scripts/run.sh test --navigation-stack \
  --navigation-duration 180 --overview-pose 6.5 3 2.2 0 .5 -1.5707963267948966 \
  --overview-fov 1.3962634015954636 --label slam_scan_roundtrip_v4 navigation

# 有限平地实际物理障碍：进入、保持10秒、撤离；真实点云停车与新SCAN恢复
# V4.5a：沿用V4.4到达/清空控制，仅补原点云预录与完整收尾
# 原0.12与turn20 profile保留，原到达/超时/停车判据不变
# 与旧相机Demo独立，不修改全局验收或坡道结果；结果必须实际验收
bash multifloor_demo/teacher_mode/scripts/run.sh test --navigation-dynamic-dwell \
  --navigation-duration 240 --overview-pose 6.5 3 2.2 0 .5 -1.5707963267948966 \
  --overview-fov 1.3962634015954636 --label slam_scan_dynamic_flat_v45a_preroll navigation

# 完整12米原ramp23物理运动协议；实际首轮失败
bash multifloor_demo/teacher_mode/scripts/run.sh test --camera-only \
  --schedule multifloor_demo/teacher_mode/tests/ramp_full/ramp23_up_full.json ramp_up

# 原起点明确下层特权terrain provider A/B；不是SLAM分层
bash multifloor_demo/teacher_mode/scripts/run.sh test --camera-only --spawn 0 0 .4 0 \
  --terrain-target-manifest multifloor_demo/teacher_mode/tests/terrain_targets/lower12.json stand
```

本轮最新入口为 `docs/CONTINUATION_REPORT_20261004.md`；08:00原交接 `docs/RESUME_20261004_0800.md` 保留。V4三轮与V4.1一轮有限路线已由实际SLAM/SCAN、非零Teacher控制、区域到达和停车独立验收。新增真实单次10sim秒命令生产者中断已通过停车、恢复和两区域到达，普通短暂恢复事件完整停车仍未验证。V4.1取消瞬时JSON逐条fsync，保留flush/close/atomic replace及原stamp/300ms双龄保护，写入等待显著下降；源龄合格但SLAM不健康的拒绝帧仍存在，不能统称TTL过期。

动态V4.1 `6614` 的实际障碍停车、清空后新SCAN复走、两区域通过，但navstack退出-15，完整运行失败。V4.2 `10c7` 修复直接ROS服务桥后全部自有/launch子进程清洁退出；返回90秒截止失败，清空1秒连续证据也不足，完整验收失败。V4.3 `2f70` 完整运行清洁退出，但去程边界停车反复重置到达计时；预定停车窗最大速度.033278m/s超.030；旧清空计时跨过.55s实际检测缺口而提前采用新SCAN。冻结独立验收失败全部保留。V4.4 `1536` 实际240秒的停车、真实clear/新SCAN恢复、两区域与终点停车、全进程退出均通过；严格完整记录只因进入前一帧原XYZ缺档而失败。最终V4.5a仅补observer有界原点云预录及异常收尾，沿用V4.4控制/profile/物理/全部门槛实际重跑；dc6f完整验收19/19通过，旧记录不回填。详见 `docs/DYNAMIC_OBSTACLE_REPORT.md`。

导航Actor使用220维原生特权状态/高度及27维控制器已知状态。独立IMU/关节30维替换前进三轮通过，使用190维特权COM/高度及27维控制器状态；尚未与导航联测。完整坡道运动仍失败，多楼层坡道导航和真机未验证。三层连接是坡道，不能称真实楼梯通过；局部可见点云不能直接保证训练187点mesh射线同义。

额外长时站立诊断：1536在3秒正式终点停车通过后，87.105..240.005s仍持续Actor零速度，但实际平面速度曾达.06681m/s、偏航速度.16460rad/s，位置最多偏离.05491m；无fault/机身接触。原3秒/5秒停车窗通过不代表全240秒始终无漂移。详见run内 `post_arrival_long_standing_diagnostic.json`。


## 最终动态平地复测与复现

实际运行 `20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f` 为240秒/12001策略帧，19项冻结独立判据全通过，6个自有程序及15个launch子进程正常退出。实际移动箱保持阻挡10秒；固定21.16..26.16秒停车窗最大平面速度0.002245m/s、漂移3.758mm、偏航0.014919rad。真实点云连续清空31.725..32.775秒；同一实际guard回调确认后解除停车，采用新SCAN，32.89秒恢复运动、实际推进0.666881米。两处原始SLAM区域50.6..51.2与90.9..91.5秒各7样本连续0.6秒到达；94.69..97.69秒终点停车漂移0.846mm、偏航0.013697rad。实际最大位移0.846669米，仍是目标中心相距1米的区域往返。

只读预录保存7份原始点云数组，障碍全过程guard所需原XYZ缺失0。519条实际guard中110条有原XYZ精确几何重放，其余409条位于记录窗外，不能据其hash称全240秒几何重放通过。CPU Teacher仍使用特权状态/高度；真实SLAM/IMU/点云决定导航、速度来源与区域到达，仿真真值只用于离线物理验收。完整坡道、感知观测替换与导航联测、多楼层以及真机结论没有提升。

最终原始收据：该run内 `summary_dynamic_obstacle_independent.json`，SHA256 `c1e26edf0bd71b284ed48012cda6faebba2615743d103609493b3387f02f9865`；执行前34源码冻结为 `test_results/navigation_v45a_observer_cleanup_freeze.json`，SHA256 `7d54a8ee0fd0a1763e0ed15c57b1a67ee33d3d5658f0b65b02ac58b07336bae9`。模型、配置、当前/逐轮源码、图像、失败、报告与日志逐文件哈希见根 `PACKAGE_MANIFEST.json`；旧包清单保存在其引用的history文件，原 `runs/acceptance.json` 原字节保留。当前分范围结论见 `current_status.json`。

```bash
# 按同一已冻结版本只读核对最终动态收据；不启动控制器
python3 multifloor_demo/teacher_mode/scripts/analyze_dynamic_obstacle_v45a.py --read-only \
  multifloor_demo/teacher_mode/runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f

# 已有运行/报告全部结束、current_status.finalized=true后更新包清单
python3 multifloor_demo/teacher_mode/scripts/update_package_manifest.py --write
```

浏览器可直接查看最终实测画面、车载/旁观CameraInfo、原始SLAM/SCAN路线与逐项验收：
[最终动态实测](http://127.0.0.1:8768/?run=20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f)。页面明确标记存档回放，不宣称仿真结束后仍实时运行。

最终dc6f额外长时零速诊断：97.69..240.005秒请求与Actor速度输入均为0；最大实际平面速度0.000443m/s、yaw率0.003962rad/s、最大XY偏离2.434mm、偏航偏离0.043790rad，无fault或机身接触。本轮后段稳定，前轮1536的自然脉冲失败诊断仍保留；该追加诊断没有加入或改变事前验收门槛。


## 新增PID路线纠偏与CHAMP同场景对照（实际验收另立）

入口 `navigation/pid_mode/` 根据实际raw SLAM的位置/偏航与实际IMU角速度，对已检查SCAN路线生成有界vx/vy/wz。PID位于速度命令层；冻结Teacher关节action/PD不改，不使用真值定位或导航。首版位置P/I/D=.6/.08/.12、偏航1.1/.04/.15，速度上限.2/.1/.2；按原SLAM header事件dt更新，微分对实际测量低通，停车/来源失联/换目标清积分。轴向限幅保留横向积分，障碍保护检查最终限速后方向，受保护时立即零速度并保留Teacher连续姿态反馈。

```bash
# 初期安全一米区域往返；独立新scope，不提升旧整体验收
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_short --label teacher_pid_v1_safety navigation

# 相同SLAM/SCAN/PID/profile/物理场景，单独启动CHAMP底层
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_short --navigation-champ --label champ_pid_v1_safety navigation

# 短程通过后逐步6m/原宽度12m坡道；各选择需要自己的实际收据
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_straight --label teacher_pid_v1_long navigation
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid ramp23_up --label teacher_pid_v1_ramp23_up navigation
```

事前协议 `tests/pid_navigation/protocol.json`，逐轮冻结为run内 `pid_navigation_freeze.json`，独立结果文件为 `summary_pid_navigation_independent.json`。新增V2 Teacher实际180秒一米往返：两原始SLAM区域到达、正式停车、最大横向误差0.11349m及完整退出通过；实际前进滚窗比例0.86876低于事前0.9，整轮失败。CHAMP旧启动依赖失败保留；修复后的V3实际188秒去程到达，64.93秒回程SLAM横向越过0.45m边界触发停车，整轮失败；旧PostUpdate ForceCmd已被Physics清空，力矩限幅未验证。独立分析勘误不覆盖原收据，分别另存 `.corrected_v1.json`/`.corrected_v2.json` 并核对显示哈希。PID V3将XY加速由0.15改0.3m/s²，补实际点云回调日志，31+67项离线检查通过；这仍是新实际复测的准备证据，不能追认旧失败或宣称优于CHAMP。逐轮证据见 `docs/PID_NAVIGATION_REPORT.md`。

Teacher Actor仍保留220维原生特权状态/高度与27维控制器已知状态，导航只用真实SLAM/IMU/点云。坡道12使用明确冻结下层terrain provider，坡道23使用原all-static provider；选择训练扫描的可碰撞层属于特权观测配置，不是SLAM导航定位。CHAMP不加载Actor，使用原qualified gait/IK/effortPID及有限关节停车适配，当前基线未启机身辅助。两者初始化与底层电机控制不同，均需核对200Hz原生ForceCmd<=23.5Nm、实际关节速度<=30rad/s；不能称硬件测量或相同电机模型。CHAMP在相同D秒路线命令后额外保留8秒零速度收尾，比较仅使用同路线期限和固定实际到达后的停车窗，额外生命周期单列。

原最终V4.5a包清单、README、状态与修改前入口字节存于 `test_results/pid_navigation_baseline_20261004/`，旧验收、失败、通过的camera_mode和另一任务评估/训练保持。三层连接仍为坡道；多层PID导航与真机须实际验证后单独结论。

PID V3真实复测 `20261004_132148_navigation_teacher_pid_v3_slew_r1_cddc` 已完成180秒：前进滚窗0.91924通过，但首目标90秒超时、原场景直线航向0.37974rad超过0.35，未返回，目的停车未验证。实际控制器379次完整点云回调，与同源1278个SLAM pose数量明显不同，保护停车占路线时窗约72.8%；不能据更低横向RMS宣称到达或优于CHAMP。新日志仅1条因无邻近SLAM拒绝，收到的包源龄均小于300ms；主要是交付间隙。已冻结独立只读QoS/传输诊断准备，未改历史相机Demo或全局ROS/训练设置。


## 速度PID实测结论与当前复现入口（2026-10-04）

速度命令层采用实际SLAM位置/速度与实际IMU角速度闭环，不修改冻结Teacher的12维action或PD。CPU单线程、唯一执行器、300ms来源保护、实际停车确认和原验收数值保持。

| 同版V4短往返 | Teacher | CHAMP |
|---|---|---|
| 完整独立通过 | 3/3，每轮25/25 | 0/3；三轮均真实到达两区域，但各有失败/未验证项 |
| 距预定有限路线最大偏离 | 0.0713 / 0.0921 / 0.1001 m | 0.6502 / 0.2183 / 0.3062 m |
| drive机身航向峰值 | 0.1751 / 0.1085 / 0.1323 rad | 0.5199 / 0.4491 / 0.4246 rad |
| 固定5秒终点停车 | 三轮全部通过 | 第二轮通过；第一、三轮reader窗口覆盖不足，原未验证保留 |

这是当前两条执行链配置的比较：CHAMP保留原步态/IK/关节PID，Teacher用训练PD/DC曲线；不把相同外层PID称为底层模型、启动姿态或电机曲线完全等价。目标中心间距1 m，实际位移约0.85–0.90 m，由0.17 m区域到达判定；不是精确往返各1 m。原CHAMP启动姿态失败及真实force/接触记录均保留。

6 m V4在实际到达/停车后，24/25门通过但drive航向0.42455 rad超过0.35，整项失败。V5只新增`route_leg`：由当前航段激活时的原始SLAM位置与冻结SLAM请求目标决定机身朝向，XY仍采用经过检查的SCAN目标。56新离线检查、31原默认检查及各1000组旧follow/PID输出完全一致，实际903条PID引用来源7/7审计通过。V5单轮180秒/9001策略帧仍24/25失败：航向0.09760 rad合格，预定场地路线最大偏离0.50488 m、drive RMS 0.22044 m超限。

V5实际SLAM请求目标离线单次固定坐标注册后，本来就相对预定场地终点偏了0.49087 m；原SLAM/native位置配对误差最大8.4 mm，不支持把该偏差统称SLAM定位漂移。另一方面，相对实际下发路线，中途仍有0.44738 m偏离。两种偏差都要解决，不能通过更换事后参考线追认原失败。该坐标注册只做离线诊断，没有把Gazebo真值送入导航。

V4首次通过轮在5秒正式停车窗之后，持续零速度仍出现约0.368 rad偏航；另外两轮后段较稳定。新V5固定停车窗通过不证明长期姿态保持已解决。完整12 m坡道仍有原始跌倒/载荷碰坡失败，坡道不是楼梯。

DDS独立实验保存了512KiB/64MiB两个新本地配置的实际A/B；64MiB配对观察者563可靠/563尽力云戳与563原SLAM戳一致。随后真实PID控制链使用该独立传输scope并实际运行；不能用旁路delivery结果替代执行桥证据，也不能把与历史LOCALHOST_ONLY1的差异全归因共享内存容量。详见`docs/CLOUD_TRANSPORT_REPORT_20261004.md`。

```bash
# 当前默认仍是原SCAN航向分支；以下均创建新实验目录，不提前继承历史通过结果
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_short --cloud-transport-profile shm_64m \
  --label teacher_pid_short_new navigation
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_short --navigation-champ --cloud-transport-profile shm_64m \
  --label champ_pid_short_new navigation

# V5仅允许Teacher的三个平地profile；本次6m整项失败，不能据此解锁坡道导航
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --navigation-pid flat_long --pid-heading-reference route_leg \
  --cloud-transport-profile shm_64m --label teacher_pid_v5_routeleg_6m_new navigation

# 完成运行后独立验收；保持旧摘要，以下命令用于新RUN
python3 multifloor_demo/teacher_mode/scripts/analyze_pid_navigation_v3.py "$NEW_RUN"
python3 multifloor_demo/teacher_mode/scripts/audit_pid_route_leg.py "$NEW_RUN"
```

现有V4三对三图与全部收据在`test_results/pid_matched_short_v4_20261004/`；V4总冻结72份来源为`test_results/pid_navigation_runtime_v4_transport_freeze_20261004/v4.json`，V5总冻结76份为`test_results/pid_navigation_runtime_v5_routeleg_freeze_20261004/v5.json`。不把V5单轮与V4三重复合并。

坡道采图使用独立`--slam-registration-only`入口：显式坡面初始位置、90秒零速度、lower12特权Actor扫描；实际SLAM/IMU/full-cloud只读观察者不发布速度、关节、导航目标或服务。stand2071共4501策略帧、90秒站立通过，旁路60秒完整记录原消息，冻结拟合输出`unverified/full_route=null`。采图/几何候选与完整运动/导航验收分开。操作及环境继承命令见`navigation/ramp_registration/README.md`；首轮9b70在旧平地配置限制处准备失败，未启动任何物理或ROS进程，原记录保留。

导航Actor仍含220维特权状态/高度加27维控制器已知状态，真实导航反馈来自SLAM/IMU/点云。30维实际传感器替换是另一个独立运动实验，未与导航联测。当前接口与基础运动、有限平地导航/动障停车有通过证据；完整场景Sim2Sim、多楼层坡道导航及真机部署没有通过证据。

PID频率核验：20Hz墙钟定时器、约10Hz原始SLAM新header才更新P/I/D；执行桥50Hz墙钟心跳，Teacher50Hz/关节PD200Hz为仿真频率，原IMU200Hz。V5六米实际903次调用中433次新反馈更新、470次duplicate保持；反馈dt中位100ms。更快重复旧SLAM数据不等于更快闭环。来源与实测在`test_results/pid_frequency_actual_20261004.json`。

真实坡道采图复现需两个终端，观察者必须在生产者仍运行时启动。新实验仍使用现有冻结CPU Teacher；不启动导航：

```bash
# 终端A：精确采图场景；创建独立run，约90仿真秒后自行结束
bash multifloor_demo/teacher_mode/scripts/run.sh test \
  --slam-registration-only --spawn 3.2 2 .52 0 --camera-rate 10 \
  --schedule multifloor_demo/teacher_mode/tests/ramp_registration_zero90.json \
  --terrain-target-manifest multifloor_demo/teacher_mode/tests/terrain_targets/lower12.json \
  --cloud-transport-profile shm_64m --label ramp_registration_new stand

# 终端B：确认latest确是终端A的新run且SLAM配置已生成；显式继承该run的已审计环境
source /opt/ros/jazzy/setup.bash
NEW_RUN=$(readlink -f multifloor_demo/teacher_mode/runs/latest)
python3 multifloor_demo/teacher_mode/test_results/cloud_transport_actual_20261004/launch_ramp_capture.py \
  "$NEW_RUN" /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/ramp_capture_new_unique
# 输出目录必须不存在。两进程都自然结束后离线分析（新输出目录必须不存在）
OPENBLAS_NUM_THREADS=1 python3 multifloor_demo/teacher_mode/navigation/ramp_registration/analyze.py \
  --capture multifloor_demo/teacher_mode/test_results/ramp_capture_new_unique/capture \
  --output multifloor_demo/teacher_mode/test_results/ramp_registration_new_unique
```

真实采集审计与首版坡道识别限制见`docs/RAMP_REGISTRATION_REPORT_20261004.md`；局部5.503°坡面诊断没有替换冻结注册失败或产生完整路线。
