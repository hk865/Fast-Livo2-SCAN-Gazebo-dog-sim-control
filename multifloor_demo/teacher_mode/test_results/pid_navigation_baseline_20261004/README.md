# Go2 Frozen Teacher：Gazebo 适配与 Sim2Sim 验证

这是独立仿真控制模式。冻结 Teacher 已实际通过关节力矩控制 Gazebo Go2，包含运动、停车、命令切换和地形测试。**最新功能复测：5 cm、10 cm各三次均实际登上顶面并继续行走；完整12米坡道失败，全场景Sim2Sim未通过；真实SLAM/SCAN有限平地往返四次均已通过独立闭环验收；最终V4.5a动态障碍240秒实际复测的19项独立判据全部通过。** 没有重新训练，没有操作实体机器人，没有覆盖已通过的 `camera_mode`。

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
