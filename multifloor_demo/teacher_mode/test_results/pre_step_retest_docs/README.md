# Go2 Frozen Teacher：Gazebo 适配与 Sim2Sim 验证

这是独立仿真控制模式。冻结 Teacher 已实际通过关节力矩控制 Gazebo Go2，包含运动、停车、命令切换和地形测试。**本轮整体运动验收失败，尚未达到 SLAM/SCAN 导航运行门槛。** 没有重新训练，没有操作实体机器人，没有覆盖已通过的 `camera_mode`。

## 2026-10-03 结果

| 层级 | 结论 | 范围 |
|---|---|---|
| 接口 | 通过 | 正式42次均有完整247观测、12动作、CPUActor、真实关节/力矩、200/50 Hz、唯一写入者及归档校验；异常保护另存证据 |
| 运动控制 | 整体验收失败 | 正式42次39通过；5 cm低台阶3次未达到冻结机身升高判据；原Demo起点额外失稳 |
| Sim2Sim | 未通过 | 已有真实 Isaac CPU匹配命令、坡道与低台阶对照；5 cm升高不足和原起点失稳在两环境出现；严格即时快照观测对齐仍存在传感器缓存时序偏差 |
| SLAM/SCAN导航集成 | 未验证 | 门禁拒绝执行；桥与命令消费边界已准备，Teacher导航启动器/路线/动态障碍闭环尚未完成 |
| 真机部署 | 未验证 | 本任务仿真限定 |

正式测试是固定名义参数、同种子、同初态的三次独立进程重复，不代表随机化、复杂地形或真机鲁棒性。10 cm登台虽三次通过，停车漂移0.131 m、偏航0.191 rad，接近0.15 m/0.20 rad门槛。上下坡只测10%坡道中的约2 m片段；低台阶只测登上平台，不是完整穿越或真实楼梯。

完整结论见 `docs/VALIDATION_REPORT.md` 和机器可读 `runs/acceptance.json`；失败记录、对照表与图在 `runs/`、`plots/`、`test_results/`。浏览器在 [Teacher 验证页](http://127.0.0.1:8768/)，实际RGB来自 Gazebo 相机，经 ROS桥采集；轨迹明确标为仿真真值诊断，不能称为 SLAM路线。原相机 Demo 使用8767，保持独立。

实际带IMU/LiDAR/车载及第三人称相机的前进停车诊断 `20261003_201443_forward_hardware_renderer_check_r1_40c6` 已通过，全部进程退出0。CPU Actor保持单线程；Ogre2使用NVIDIA渲染，峰值图形显存增量约156 MiB。软件Ogre1在退出清理时崩溃的旧记录保留，默认采用已通过的硬件渲染。

## 冻结模型与执行契约

Checkpoint：

```text
/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt
SHA256 bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34
```

`actor_state_dict/mlp.*` 构建 `247→512→256→128→12` ELU网络，直接使用Gaussian均值、无观测归一化、无动作预裁剪。推理CPU、单Torch线程，物理5 ms、策略20 ms。调用本地已存在的 Isaac Python 来使用Torch，不占用训练GPU。

训练关节FR、FL、RR、RL各hip/thigh/calf，逐名映射到`rf/lf/rh/lh`各`hip/upper_leg/lower_leg`。默认姿态为 `[-.1,.8,-1.5, .1,.8,-1.5, -.1,1,-1.5, .1,1,-1.5]`，目标 `q_default+0.25*raw_action`。PD为25/0.5，DCMotor以23.5 Nm、30 rad/s执行速度相关力矩限幅。物理关节硬限位来自本次训练归档，目标不偷偷裁剪到通用部署契约。

247维顺序：COM线速度、角速度×0.2、机身重力、实际速度命令、q−default、qd×0.05、上次已限幅应用力矩×0.01、上次raw action、187高度。前60维先可选噪声、clip±100、再scale；默认清洁评估，`--noisy`才启用归档均匀噪声。高度为yaw-only的17×11网格，y外x内，10 cm分辨率；射线从base_z+20向下，`clip(base_z-hit_z-0.5,-1,1)`。+20不加到sensor/base高度上。本场景的上层楼板会被射线击中，原点初始102列为上方击中；不能静默换成脚下高度。

当前策略输入是 **Gazebo特权状态 + 静态SDF碰撞几何射线**，不是已经完成SLAM/点云感知部署。动障碍与自体不纳入训练mesh式射线，缺射线按训练clip语义为−1并记录缺失数。详见 `policy/contract.json`、`docs/OBSERVATION_REPLACEMENT.md`；旁路IMU/关节测试不会替换Teacher输入。

原Demo模型仍含传感器载荷，与训练模型质量和惯性不同：总质量16.512 vs16.087 kg，base质量8.336 vs7.279 kg，base COM也不同。速度读取实际Gazebo COM，不能沿用训练COM修正。报告保留碰撞形状、摩擦混合和PhysX/DART等物理差异。基准模型摩擦0.7、地板1.0，是受控测试参数；训练随机化范围只归档，未声称本轮覆盖。

## 控制权、初始化与停车

复制原SDF后仅加入 `teacher_sim::TeacherActuator` 力矩写入者，删除CHAMP、ros2_control、JTC和旧body stabilizer。只在启动时设置一次关节初态，之后不重置基座、不施加基座伺服。只读关节状态/传感器发布器不拥有执行权。

初始化默认姿态PD 0.1 s，然后Teacher零速度持续到3 s；运动命令有限幅与加速度限制。命令停止或TTL超时后，速度命令平滑归零，**Teacher仍连续推理与控制**，以实际低速度/四脚支撑确认停车。不是action=0，也不冻结最后关节姿态。早期冻结停车目标导致后退/转向失稳，代码和失败保存在 `test_results/stop_capture_v1`。

IPC每策略周期同步，200 ms墙钟无有效回复、NaN/异常目标、其他控制器、断联或机身跌倒都会锁定故障，转入关节阻尼，并记录故障码。故障阻尼仅是仿真异常处置，不能声称已平稳站立。正式命令超时测试是worker内模拟生产者停发，300 ms仿真TTL；实际ROS导航超时消费还未运动验证。

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

Native输入是PreUpdate读取上次Physics结果，ROS传感器为PostUpdate；比较实际物理采样需用 `state_physics_world_time=world_sim_time−.005`，不能把相同stamp直接当相同状态。实际900组此相位匹配q/qd误差为0；仍未将传感器输入接入策略。几何边界1e−9扩张问题已修复，全部42轮40692帧247回放差0、修正后原起点仍实际失败；补丁与失败记录保存在 `test_results/raycast_boundary_v1`。

## 后续集成门槛

`navigation/bridge.py` 和worker的navigation入口均要求正式汇总`motion=passed`与`sim2sim=passed`。当前文件如实失败，所以必须拒绝；没有绕过开关。未来速度来源固定为现有SLAM/SCAN `/demo/cmd_vel`，位置为 `/demo/slam/body_odom`，且验证唯一真实发布节点、坐标与时效。原46个到达区域几何保留，真值仅离线验收。完整Teacher导航启动器尚未交付为可运行成功Demo，不能运行旧Go2整栈同时夺取关节执行权。

下一步应先解决上层扫描场景适用性、低台阶姿态和严格传感器缓存时序差异，再做真实SLAM/IMU/局部点云输入旁路及A/B闭环。局部可见点云不能保证训练+20 m垂直射线的最高表面，缺失/遮挡/动态物体语义仍未解决。本任务没有通过改输入或放宽判据掩盖这些限制。路线、多楼层全坡、动态障碍停车恢复与真机均保持未验证。
