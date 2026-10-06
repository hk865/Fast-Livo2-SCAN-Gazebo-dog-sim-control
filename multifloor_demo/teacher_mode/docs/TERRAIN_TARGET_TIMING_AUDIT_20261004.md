# Teacher 地形目标、策略相位与真实传感器审计

本审计只读训练归档、Isaac Lab 源码和既有真实日志；未启动 Kit/训练，未停止或改动训练进程，未改运行源。2026-10-04。冻结模型 SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。

## 地形目标结论及此前解释的更正

训练的187射线**不面向全场景碰撞物**。冻结 `params/env.yaml:241–247` 的 TerrainImporter 为 generator，prim 为 `/World/ground`；`435–469` 的 RayCaster 只声明 `/World/ground`，+20 m、yaw、0.1 m网格及世界负Z方向。`TerrainGenerator:167` 将生成的各地形块与边框合成一个 mesh；`TerrainImporter:92,241–252` 以 `terrain` 导入；`terrains/utils.py:85–99` 创建 `/World/ground/terrain/mesh`。归档的接触过滤也明确指向这个 mesh（`env.yaml:527`）。

`BaseRayCaster:159–195` 只允许一个目标 prim，先选择其首个 Plane，否则首个 Mesh，读取并转换该 mesh 的世界顶点；`303` 只向这个 Warp mesh 求交。初始化时缓存该静态 mesh，不逐帧自动吸收其他物体或新几何。visualizer、机器人、场景外的 props 和其他 Mesh 不因为存在碰撞就进入此扫描。

冻结生成器（`env.yaml:260–403`）是低随机高度场、正负坡、凸起/凹坑、两种地面网格和两种台阶，拼接在地面 XY 地形块上，不是叠置三层楼板。首命中依然可能是训练地形内的凸起/台阶；“训练只选地形”不等于“每格强制脚下平地”。也不能以 `get_first_matching_child_prim` 推断只选某一地形块，因为此时所有块已在一个合并 mesh 中。

此前将“+20 m 竖直射线”解释为必须扫描全部 Gazebo 静态物，并认为改变扫描集合必然违反训练语义，过度扩大了源码结论。准确结论是：**射线算法正确，初版部署的目标集合比训练定义更宽。** 原起点102条射线看到楼上是真实的初版 provider 结果，但不是训练 `/World/ground` 必然含三层楼板的证据。Isaac 原起点诊断 h 同样将完整 SDF 静态几何合并成了它的 `/World/ground/terrain`，所以两环境共同失稳只说明在这一共同输入/场景条件下失败，不能单凭此认定策略唯一根因，也不能称原生训练几何对照。

`kernels.py:91–118` 将 base源prim实际姿态和物理frame偏移组合，只用yaw旋转局部ray starts，方向仍世界负Z；cfg的+20已加在局部starts（`BaseRayCaster:202–213`），不加到 `sensor.data.pos_w`。高度公式 `observations.py:297–305` 仍为 `base_z-hit_z-.5`，裁剪[-1,1]。保持这些算法后，可显式改变地形目标集合做部署 provider A/B；这是正当的可审查适配，仍须标明与冻结训练几何严格不同。

## 固定目标集合及可执行验证

物理场景三层平台、两坡道、墙和定位物全部保留；安全clearance继续从base高度向全部静态碰撞求脚下支撑。仅policy scanner读取 manifest 的固定include模型列表。推荐：

| ID | include_models | 范围 |
|---|---|---|
| lower12 | floor_1, ramp_12, floor_2 | 原起点与一至二层走廊特权扫描诊断 |
| upper23 | floor_2, ramp_23, floor_3 | 二至三层走廊特权扫描诊断 |
| walkable | floor_1, floor_2, floor_3, ramp_12, ramp_23 | 排除非地形props的独立对照；仍有叠层首命中 |
| origin_floor1 | floor_1 | 可选的原起点最小平地隔离诊断，不作为完整路线provider |

这些集合不由当前机器人真值自动切换，也不为导航提供位置。manifest应保存选择理由、实际碰撞名称、排除名单、manifest/world/model/config/checkpoint哈希；缺模型/非法名称/不支持几何应失败。跨层切换未来若使用SLAM区域或局部点云，需要作为新的感知provider明确记录和实测。

2026-10-04只读数值核验使用 corrected `policy/observation.py` 和原 `three_floors.sdf`（哈希见下表），没有运行actor或模拟器：

| 点/名义base z | 初版all-static scan | 固定走廊scan |
|---|---|---|
| origin (0,0,.4), yaw0 | 85 floor_1 +102 floor_3；102 overhead；raw[-2.5,-.1] | lower12：187 floor_1；0 overhead；raw全-.1 |
| ramp12低端 (1.2,2,.4) | 187 floor_3；全部overhead和clip=-1 | lower12：176 floor_1 +11 ramp_12；raw[-.1004975,-.1] |
| ramp12中点 (8,2,1) | 187 ramp_12；0 overhead | 同值，raw[-.1804975,-.0204975] |
| ramp12高端 (14.8,2,1.6) | 176 floor_2 +11 ramp_12；0 overhead | 同值 |
| ramp23低端 (14.8,7,1.6), yawπ | 176 floor_2 +11 ramp_23；0 overhead | upper23同值 |
| ramp23高端 (1.2,7,2.8) | 176 floor_3 +11 ramp_23；0 overhead | upper23同值 |

沿名义中心线x=1.2..14.8、每0.1 m取support+.4 m：ramp12的all-static overhead从187条降到x2.8的11条，x2.9起为0；ramp23的137个样本均无overhead。该计算只是几何/输入核验，不是坡道物理通过证据。

**实际轨迹限定补充：名义中心线等价不代表整条轨迹等价。** gazebo_audit 对真实完整 ramp23 上/下行姿态回放（`test_results/full_ramp_audit_20261004/height_scan_target_shadow.json`）：侧漂使网格越过坡道y=6..8后，all-static会看到下层 `floor_1`，而upper23排除了它，所以成为nohit=-1；实际all-static扫描重建误差0，但upper23的假设扫描最大clip差2。上行19.12 s起190帧/4023点不同，下行21.28 s起211帧/3025点不同，各最多51/187缺点。upper23是有输入缺测风险的新provider，不能因0overhead就预设其语义等价或运动必改善。后续可对照含floor_1的walkable集合，或明确冻结未知区域停车规则，不隐藏真实低层落差。

有效闭环A/B：同一当前源、物理SDF、spawn `[0,0,.4,0]`、初始化PD .1 s、Teacher零命令15 s，A显式all-static集合，B显式lower12；唯一预期实验变量为扫描集合。保存真正收到的247维、raw/hit/mask/name、action、q_target、限幅后tau和200 Hz原生姿态/接触/脚下clearance。先比较bootstrap完全相同，再定位第一个Teacher action差异；轨迹分叉后的不同状态不能用逐帧action差值冒充同输入差值。可在同一真实状态、cmd和上一rawaction下仅替换scan，CPU前向得到反事实动作灵敏度，必须标记为离线诊断，不当成执行后的运动成功。至少重复A/B各3次，再拓展运动、坡道和台阶；单个origin站立成功不能宣称全Sim2Sim/导航或真机通过。

实际一对已完成并独立复核：A `20261004_010504_stand_origin_all_static_current_ab_r1_8ffb`、B `20261004_010357_stand_origin_lower12_target_r1_3ebc`，见 `test_results/terrain_target_origin_ab_20261004_v2.json` 与 `scripts/analyze_terrain_target_ab.py`。两个world字节、物理asset字段、native actuator契约、8项核心source hash和控制条件完全一致，均CPU冻结actor与真实零cmd。初始化position/quaternion/q/qd/q_target/tau逐元素相同。两组全部247维按真实state及实际动作历史重建误差0；CPU actual247→action重放差分别 `1.0967e-5/1.0729e-6`。

首Teacher时刻 `.1 s`，两组前60维完全相同，高度最大差`.843347`，actual action最大差`2.921686`，q_target最大差`.730421 rad`。只将A首帧height换为B的CPU反事实action与B首actual action误差0。因此该输入目标集合在这一受控起点确实改变策略动作和闭环结局，但后续状态分叉，不作同输入动作差或唯一全局病因结论。

| 实际结果 | A all-static | B lower12 |
|---|---|---|
| 持续时间/冻结站立判据 | 1.52 s触发低clearance fault，damping后2.02 s结束/failed | 15 s/passed，无fault |
| 200 Hz最小脚下clearance，自PD结束起 | .102341 m | .275679 m |
| 200 Hz最大绝对roll/pitch | 2.331738 rad | .113963 rad |
| 200 Hz body contact样本 | 88 | 0 |
| 5–14 s停车漂移/偏航 | 未完成 | .010761 m/.042922 rad |

表中的200 Hz安全统计覆盖PD结束至实际run终止，包括A最后约.5 s的故障damping；A首fault日志为1.52 s、roll2.230234 rad、clearance.141401 m。这些数值不冒充故障后仍在正常Teacher执行的表现。

此结果仍仅一对特权目标集合诊断；其新provider、Sim2Sim全面通过、SLAM导航与真机状态须分开。分析v1误把初始化期间仅用于日志的actor action推进到上一动作，导致人工重建差；旧failed receipt与相同哈希的旧分析源码保留，并在 `test_results/terrain_target_ab_analysis_correction_20261004.json` 说明。v2按worker实际init history=0重建，未改任何源日志、policy或物理。

## 严格快照差异的实际作用

matched CPU reference c实际8350帧，其中45帧PD初始化、8305帧Teacher。日志中的真实247维重放actor最大差 `1.6093e-6`，而即时状态独立重建的strict parity仍失败，旧失败保留：

- 9个reset首帧gravity最大差 `.01591263`，其reset quaternion已重置但gravity缓存尚未按新sim timestamp失效。`articulation.py:489–501` invalidates姿态/机体/Jacobian等缓存，不包含 `_projected_gravity_b`；`articulation_data.py:995–1008` 仅时间推进才更新它。所有这些首帧都在PD init，未参与本轮Teacher动作执行，不能拿它解释ongoing失稳。
- 非reset gravity差最大 `8.94e-7`，其他proprioception/command/action差≤`2.98e-8`。
- switch79帧scan相对fresh pose差最大`.00221783`；实测sensor z与上一pose z在ray浮点精度内一致。`sensors/kernels.py:10–30` float32累计5 ms，以20 ms及1e-6容差判断到期，支持周期采样滑拍解释，但当时未记录sensor内部timestamp，所以具体时钟成因仍是源码与实测相位支持的推断。recorded raw height裁剪与actual actor scan严格一致。

接口验收应分开“实际actor收到的247→action”“逐项构造/坐标/点序”“被采样的物理相位/源龄”和“严格瞬时snapshot parity”。训练step在decimation之后更新scene并构造下次obs（`manager_based_rl_env.py:201–229`），不是任意ROS同戳所有原始量都必须与即时truth一致。合理修正是记录有效状态时间、每项采样时间与policy序号，按实际policy相位因果配对；不能篡改时间戳、用未来数据、悄悄回填真值，或把strict failed改写成passed。

## 真实输入替换矩阵与A/B

| 区间 | 候选来源/变换 | 已有真实证据和下一步 |
|---|---|---|
| 0:3 COM速度 | SLAM `/demo/slam/body_odom` body原点速度+body gyro×本轮COM杆臂 | 实际shadow SLAM缺失。现有adapter约扫描频率差分平均速度，需IMU传播/估计、source age与误差审计；不能把10 Hz hold当50 Hz瞬时速度 |
| 3:6 角速度 | `/livox/imu` gyro，经body←IMU固定旋转；clip后×.2 | 单轮已实际200 Hz；按物理相位gyro RMSE约2e-4 rad/s。先单组闭环替换并查方向/偏置/丢包 |
| 6:9 gravity | SLAM重力对齐姿态或真实姿态估计 | Gazebo CUSTOM/world IMU orientation旁路误差0，只证明仿真接口。真实SLAM重力对齐/姿态估计未验证；不能归一化运动加速度当gravity |
| 9:12 command | 同一控制器限幅/超时/slew后的实际cmd | 无需真值；导航cmd及实际policy cmd均记录 |
| 12:24 q | actual JointState逐名FR/FL/RR/RL映射，减冻结default | 单轮900相位配对误差0；单独闭环替换、缺名/重复/迟到停车反例 |
| 24:36 qd | 同一actual JointState velocity，clip后×.05 | 单轮900相位配对误差0；缺velocity不能无标注差分替代 |
| 36:48 torque | 唯一执行器软件last applied clipped DC命令，clip后×.01 | 训练本身是applied command语义，控制器可保存；Gazebo JointState effort零/接触wrench不是等价输入，也不是实体力矩测量 |
| 48:60 last_action | 控制器上一policy tick raw action | 无需真值；故障/停车过渡保持准确历史，不把当前action或0冒充历史 |
| 60:247 height | SLAM局部注册点云/地图与重力对齐姿态，固定17×11 yaw-grid | 实际shadow注册cloud缺失。最高可见点不是完整地形ray hit；新support-layer provider需coverage/age/unknown/自体/动态/叠层规则与闭环A/B |

真实 hardware/Ogre2 旁路轮 `20261003_201443_forward_hardware_renderer_check_r1_40c6` 已收到3602 IMU、3601 joint、181原始LiDAR。没有SLAM/cloud发布者；未切策略。native `PreUpdate(world_t)` 读前一步physics，PostUpdate ROS样本header为当前world_t。同戳qd最大差13.7292 rad/s；实际前一步 `world_t-.005` 因果样本配对后q/qd/gravity误差0（900联合帧，首joint样本缺失仍未配对），gyro RMSE `[.00019628,.00020279,.00019868]`。同戳失败与缺源记录保留。离线能配对不等于live采样在policy deadline之前已经收到，仍需接收时刻/source age日志。

按组A/B先gyro，再q/qd，再gravity/COMvelocity，再height，A/B保持相同冻结actor、PD/DC包络、模型、物理、停车和命令；各3 fresh spawn重复。先旁路247和CPUaction灵敏度，再真实B闭环。额外“相同已测延迟的特权源”和“真实传感源”对照可分离延迟与坐标/噪声影响，但延迟特权源只是诊断；B不能偷偷fallback truth。预先冻结源龄、丢包/乱序/unknown gates，加入失联和遮挡停车恢复；SLAM-only导航位置仍是独立门禁。

## 核心源码与哈希

本地IsaacLab HEAD为 `51b4fb54184fa7c0a406643129953a8b26136230`，下列RayCaster/terrain文件无本地git差异。行号以这些哈希为准。

| 文件 | SHA256 |
|---|---|
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/base_ray_caster.py` | `6aa7b375bc54716215407bbb885f31624aea8a107e3d45d99ba0ae11bf61dd2a` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/terrains/terrain_importer.py` | `f55af93107d608172d776995ebbdab54f282955759f92166ec7e65954ba47963` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/terrains/terrain_generator.py` | `a364d85812a7713b21a94a450f40e8ef5d448bbe652bcb2ee36184b6300c409b` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/terrains/utils.py` | `48a19cba47c62a520541b350258a525a2da4069759b9fb2fd7202b5c863a7c5a` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/kernels.py` | `13e72f44e28215d9eda1f092284576edc829834e8713567fe371f35faa53b408` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/envs/mdp/observations.py` | `3a86305373ae26f0eb96b349fd84e026654282d54d12ee09920c59eb76fe2211` |
| `/home/hyh001/IsaacLab/source/isaaclab/isaaclab/sensors/ray_caster/patterns/patterns.py` | `3315d8e13b8e5687f821d253b3c6c1dedd907bf9c2739d6da82d8930769fe94e` |
| `/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/simulation/generated/three_floors.sdf` | `1f781af572ba8ce70ff926e9789cc1a9db326e61822660fcb66a623eb3765161` |
| `/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/policy/observation.py`（本次数值扫描） | `b0c8c6b6ef8cd51f9b150c6a387631dc20cc1cc214706e2d70fb2683cfba810b` |

归档为 `/home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/params/env.yaml`，SHA `b48ccebc1ef9ac7aa4d31451c720d38af040997fbc0c84c096cdd12b281e5d79`。完整日志、strict failed、old h边界2.4000038 m差及零epsilon诊断均保持原样；本报告只更正解释与给出实验边界。
