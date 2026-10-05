# 247 维 Teacher 特权观测的传感器替换审计

当前已完成有限范围的真实ROS IMU/关节输入替换闭环：t≥3 s替换30维，站立单轮通过；优化消息桥首组三次前进为2通过/1末帧墙龄失败，08:00资源空闲另有**3个清洁前进/停车重复全部通过**。中途runner清理受SIGTERM打断的一轮不计完整运行通过，原墙龄失败保留。其余190维COM/height仍来自Gazebo特权物理/SDF，27维为控制器已知状态，不能宣称完整感知替换或所有资源条件可靠性通过。详见[独立传感器报告](SENSOR_REPLACEMENT_REPORT.md)。`scripts/sensor_shadow.py`仍只做旁路；它早期单轮SLAM body odom与注册点云缺失的记录仅描述该轮，不能扩称后来导航运行都缺失。2026-10-04的地形目标集合A/B仍使用特权SDF，是独立来源诊断，不是SLAM地图输入验证。

所有语义依据来自本地冻结训练归档、`policy/contract.json`、`policy/observation.py` 及现有 SLAM 源码，无网页或通用模型契约替代。冻结 checkpoint SHA256 为 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。训练归档 `params/env.yaml` SHA256 为 `b48ccebc1ef9ac7aa4d31451c720d38af040997fbc0c84c096cdd12b281e5d79`，`params/agent.yaml` 为 `a5dbae28edf797729166044b2d3fda0afcc8812726295e28dfb580a39e862ec3`。

## 字段顺序、来源与当前可替换程度

区间为 Python 的左闭右开索引。训练与适配器都使用机身 +X 前、+Y 左、+Z 上及右手角速度。处理顺序是语义值 → 可选训练噪声 → 裁剪 → 缩放 → 拼接；冻结评估默认关闭训练噪声。不能因有传感器就再加入训练噪声，或改变裁剪与缩放顺序。

| 字段 / 维度 / 区间 | 冻结语义与处理 | 候选实际来源 | 当前判断 |
|---|---|---|---|
| 机身线速度 / 3 / 0:3 | base rigid-body **COM点**的瞬时速度，表达在机身轴；裁剪 ±100，缩放 1 | `/demo/slam/body_odom` 的机身原点差分速度，加 `ω×r_COM` | 可做旁路比较；10 Hz左右的有限差分、延迟和COM偏移需要验证，不能视为已等价 |
| 角速度 / 3 / 3:6 | base角速度，裁剪 ±100 后乘 0.2 | `/livox/imu` 实际gyro，经body←IMU固定旋转 | 已切入真实30维闭环；有效帧来源/旋转重建误差0，资源空闲三清洁前进/停车通过，原2/3与墙龄失败保留 |
| 重力方向 / 3 / 6:9 | `R_world_body.T @ [0,0,-1]`，裁剪 ±100，缩放 1 | 实际Gazebo IMU orientation；未来SLAM/实体姿态估计 | 已切入上述30维闭环；本轮CUSTOM/world来源有明确外参，SLAM姿态与实机估计未验证，不能直接把运动加速度归一化当重力 |
| 速度命令 / 3 / 9:12 | 策略本次实际接收的vx/vy/wz，裁剪 ±100 | Teacher命令入口；未来经已门禁的SLAM/SCAN桥 | 本来无需特权；必须记录限幅、超时与停车后的实际送入值，不能用导航原始请求代替 |
| 关节相对位置 / 12 / 12:24 | `q-q_default`，裁剪 ±100，缩放 1 | `/demo/control/measured_joint_states` 完整实际position | 已切入30维闭环；逐名映射真实callback与actor输入误差0；末帧墙钟过期真实触发阻尼 |
| 关节相对速度 / 12 / 24:36 | 默认速度为0，裁剪 ±100 后乘 0.05 | 同一实际JointState的velocity | 已切入30维闭环、重建误差0；缺少velocity仍不得差分后无标注冒充原传感器值 |
| 执行器力矩 / 12 / 36:48 | 上次DCMotor**执行命令**，经过速度相关限幅，裁剪 ±100 后乘 0.01 | 实际执行器的软件命令状态，而非接触力或未经限幅的PD | 软件可保存等价命令；当前 `applied_torque` 不是实体力矩测量，真实电机/关节测量来源尚未建立 |
| 上一动作 / 12 / 48:60 | 上个policy tick的raw action；裁剪 ±100，缩放1；无action预裁剪 | 同一Teacher控制器内部状态 | 本来无需特权；当前tick的日志action不能直接当上一动作。停车中仍保持与执行状态一致，不能每次置0 |
| 局部高度 / 187 / 60:247 | yaw-only网格；`clip(base_z-hit_z-0.5,-1,1)` | `/cloud_registered_full` 加SLAM姿态的观测候选 | 最难等价，遮挡、缺点、上层平台与动态物体语义未解决；暂不允许直接替换 |

训练关节顺序是FR、FL、RR、RL，各hip/thigh/calf。候选ROS名称依次为 `rf_hip_joint/rf_upper_leg_joint/rf_lower_leg_joint`、`lf_*`、`rh_*`、`lh_*`，以 `contract.json` 的完整数组逐名索引；不得依赖JointState数组当前顺序。`q_default` 是训练冻结的 `[-.1,.8,-1.5, .1,.8,-1.5, -.1,1,-1.5, .1,1,-1.5]`，不能用旧Demo `0/.9/-1.8` 替代。

## 时间、坐标与COM修正

`slam/odom_adapter.py` 从 `/aft_mapped_to_init` 经传感器外参转换，发布 `/demo/slam/body_odom`：父坐标 `camera_init`、子坐标 `demo_slam_body`。`slam/geometry.py::transform_state` 先将IMU状态转换到目标原点，再由 `body_twist` 计算差分速度。线速度是当前机身轴表达的**机身原点**速度，不是COM速度；角速度是姿态差分。首条位姿无有效速度，adapter以 `twist.covariance` 的巨大数值标记，不能当真实静止速度。

应在相同时刻构造：

```text
omega_body = R_body_imu × omega_imu
v_COM_body = v_origin_body + omega_body × r_origin_to_COM_body
gravity_body = R_reference_body.T × [0,0,-1]
```

角速度本身在刚体各点相同，没有额外COM角速度修正；**线速度**才需要 `ω×r`。如果来源仍是IMU点的线速度，则偏移须用 `r_IMU_to_COM`；当前SLAM adapter已转换到机身原点，不能再次减去IMU杆臂。`r_COM` 必须来自本轮实际模型惯性坐标，旁路工具读取 `asset_manifest.json.base_inertial_pose`；训练名义COM `[0.020153064,0,-0.0051089553]` 不能直接套用到惯性属性不同的Gazebo模型。

当前scenario声明IMU固定外参为同轴且原点重合，Gazebo orientation参考为CUSTOM/world identity。旁路工具仍按 `body_imu_quaternion` 和 `world_quaternion` 显式旋转。SLAM的 `camera_init` 只有在实际静止初始化与 `fastlivo.yaml` 的 `gravity_align_en=true` 生效后才可把该参考的负Z当重力；不能用独立真值对齐或修正导航姿态。

训练physics步长5 ms、decimation4、策略50 Hz。当前Teacher `prepare.py --sensors` 将实际IMU设为200 Hz，旧Go2正式scenario文档的1000 Hz不代表本轮真的1000 Hz。SLAM/cloud典型10 Hz；相邻位置的有限差分约含半个扫描间隔的平均延迟，再加处理、队列和传输。50 Hz取同一旧SLAM速度五次不等于50 Hz瞬时速度。IMU传播、速度估计或外推需要另行实现与验证；旁路工具不会通过未来样本插值来改善误差。

每个候选应保留源生成仿真戳、接收monotonic墙钟、策略比较戳、源龄、IMU/关节/SLAM/cloud各自配对间隙、掉帧、重复与倒退。wall时间决定来源已失联，sim时间决定跨模态语义是否同时；仿真暂停时wall看门狗仍要运行。`sensor_shadow.py`只选不晚于参考policy tick的因果样本，sim和wall均最多300 ms，这是宽松旁路窗。**实际输入替换更严格：header≤native有效物理world_t−.005，sim龄≤25 ms、wall龄≤300 ms**；t≥3 s缺失/无效/过期立即阻尼、停止actor推理，没有特权fallback。工具不能从没有发布源戳的接口测出精确端到端生成延迟。

`sensor_feedback.py`只订阅IMU与JointState，不发布执行或导航命令。v2内部80条、raw完整JSONL保持，live最近16条以200 Hz墙钟原子导出，SHA256 `de84a4242279d824e94c5fd0c47978dbbca2c4228009d6ead5cae15c40e1c549`；实际成功帧sim龄p95从旧20 ms降到0。优化前进三轮仍有一轮末帧约427 ms真实callback接收间隙导致joint墙龄拒绝，所以没有放宽TTL或宣布延迟问题全解决。

08:00资源空闲三个清洁18 s前进/停车复测分别751帧真实替换、247重建误差0，actor误差≤1.55e−6，sim龄p95=0/max≤5 ms、wall龄max≤11.8 ms；完整五个自有进程均正常退出。原模型、物理、PD、观测/执行器、TTL、运动/停车判据没有变，worker仅失败日志增强。较低GPU负载与接收间隙降低、三轮通过相关，不能视为唯一因果或对所有资源条件的保证；另一轮清理SIGTERM且runtime缺失不计通过。

## 187点高度扫描的射线与地形目标集合

冻结网格x为 `[-.8,.8]` 共17点，y为 `[-.5,.5]` 共11点，分辨率0.1 m。使用 `meshgrid(indexing='xy')` 后C展开：**y外层、x内层**，形状11×17。网格只跟随机身yaw，射线始终沿参考世界竖直负Z；不能让roll/pitch旋转扫描平面，或按17×11的相反点序喂入。

训练scanner的+20 m配置抬高的是ray starts，不是观测参考位置。射线从 `base_z+20` 向下首先命中**显式选中的地形 mesh**；高度值仍是 `base_z-hit_z-0.5`，不是 `base_z+20-hit_z-0.5`。平地数值随机身高度变化，不能固定187点为1或5。缺失ray hit为+inf，原始高度为-inf，训练裁剪后为-1；适配必须另记原始缺失与有效mask，不能把未知当0高度或默认为平地。

冻结 `env.yaml:241–247,435–469` 声明 generator 地形 `/World/ground`，RayCaster 的 `mesh_prim_paths` 只有该路径。`TerrainGenerator:167` 合并各地形块；`TerrainImporter:92,241–252` 导入为 `terrain`；`terrains/utils.py:85–99` 创建 `/World/ground/terrain/mesh`。`BaseRayCaster:159–195` 只选择路径下首个 Plane，否则首个 Mesh，`303` 只向该 mesh 投射，并不扫描全场景碰撞物。归档的高度场、地面网格与台阶没有三层叠置楼板这一结构。相关本地源码与 SHA 见 [地形目标及时间审计](TERRAIN_TARGET_TIMING_AUDIT_20261004.md)。

**更正此前解释：+20 m 射线起点不要求把全部 Gazebo 静态碰撞物当作训练地形。** 初版 SDF provider 的全静态碰撞集合比训练地形目标更宽，包含楼板、墙、视觉定位物等。原起点 `[0,0,.4,0]` 的102条射线命中三层平台顶面是真实的初版 provider 结果，不能据此说冻结训练语义强制在下层看楼上，也不能把共同使用该集合的 Isaac 诊断失败当作训练原生目标集合等价证明。早期数据和失败记录保留，修正来源解释。

可将扫描目标用固定 manifest 显式限定为地形，物理碰撞和安全判据继续使用完整场景。`lower12` 选 `floor_1/ramp_12/floor_2`；`upper23` 选 `floor_2/ramp_23/floor_3`；全可走表面对照选择三层平台与两坡道。前两者是新的部署观测 provider，具有清楚的特权来源和实验范围；它们保持+20、网格、yaw与clip，但不能宣称与冻结训练几何严格相同。完整可走集合仍会在原起点命中上层板，单纯去掉 landmark 不会解决该输入差异。若未来由 SLAM 局部地图选择支撑层，必须再记录地图来源、分层规则、未知区域与闭环 A/B，不能用仿真真值做导航或隐式切层。

`/cloud_registered_full` 是可见面点云，未观测区域、机身遮挡、足下盲区、侧壁稀疏点与顶面遮挡都需要记录。在每个yaw-grid柱内取最高可见点，只能得到“最高已观测候选”，不能证明它就是+20 m射线的最高交点；看到上层板底面也不等于知道其板厚和顶面。时间积累的局部地图可能保留旧障碍、跨楼层混合、误差扩散，不能无标注使用已过期点补洞。

训练height mesh不包含机器人；当前特权SDF适配还排除移动障碍。实测点云含自体与动态物体，需要只由实际外参、姿态、观测和检测确定过滤来源，不能用仿真模型真值在“传感器候选”里清除障碍。真实动态障碍仍应进入SCAN避障。两种用途的地图不能因为想匹配Teacher就一起删掉动态障碍。

旁路工具因此只输出最高已观测柱、valid_mask、187列覆盖、观察到的overhead数，以及同刻特权SDF参考扫描的诊断差。每个候选固定 `eligible_for_policy_replacement=false`；缺列保持null。SDF仅作独立误差参考，绝不进入候选点云、策略切换或导航位置。

## 力矩、命令与上一动作的来源

`simulation/teacher_actuator.cpp` 明确将已按DCMotor速度/力矩包络裁剪的数值写入原生 `JointForceCmd`，并以“passed to physics, not hardware torque measurement”标记。policy telemetry中的 `applied_torque` 是该**执行命令状态**，不能称为实体关节力矩传感器读数。冻结训练 `joint_effort` 本身也是执行器applied torque语义；保留正确软件命令可避免重新引入特权状态，但必须保证控制器唯一、限幅时刻与上个physics/policy tick一致。若未来以电流估计或关节力矩测量替代，需要另外验证命令与测量的系统差异。

`/demo/control/measured_joint_states` 来自实际Gazebo关节状态桥；其effort字段的实体测量语义尚未建立，旁路工具不拿它假充真实力矩。实际 hardware/Ogre2 单轮旁路已收到200 Hz IMU/关节、10 Hz原始LiDAR及图像，并记录真实 `ros_gz_bridge` 发布者。该轮 SLAM 和 `/cloud_registered_full` 发布者为空，策略未替换。生成SDF包含传感器、ROS实际收到数据、感知闭环通过是三个独立验收状态。

该轮 native Teacher `PreUpdate(world_t)` 读取上一物理结果，而 IMU/JointState 的 `PostUpdate` 样本使用当前 world stamp。同戳 qd 比对最大误差13.73 rad/s；按源码确定的物理相位选择 `world_t-.005` 的实际历史样本后，900帧 q/qd误差0、gravity误差0、gyro RMSE约2e-4 rad/s。原同戳失败保留。这是因果配对诊断，不能通过提前使用未来样本或重写传感器戳解决闭环延迟；真正切入策略前仍须记录当时已经收到哪些样本和source age。

命令与上一动作属于控制器内部可访问状态，无需真值。命令取策略本次实际收到的值，上一动作取前一个20 ms policy tick的raw action。动作scale为0.25，并无统一action clip；不能改为未经记录的裁剪动作。软件停车与异常处理改变了执行参考时必须记录该过渡，不得通过动作置0假称已停车。旁路工具区分当前日志action与上一tick action；首条或有跳帧时上一动作保持未验证。

## 旁路采集与后续A/B验收

立即可执行的只读工具不会加载actor、不发布关节或速度、不切换策略、不初始化导航。它只订阅源码声明的四个实际话题并读取指定运行的 `state.json`、新增 `telemetry.jsonl`；默认ROS域79。每次使用新输出目录：

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 \
python3 multifloor_demo/teacher_mode/scripts/sensor_shadow.py \
  --run multifloor_demo/teacher_mode/runs/latest \
  --output multifloor_demo/teacher_mode/test_results/sensor_shadow_current_20261003 \
  --duration 30
```

纯准备检查不导入ROS、不创建输出目录：

```bash
python3 multifloor_demo/teacher_mode/scripts/sensor_shadow.py --prepare
```

`--run` 在启动时冻结到该目录，不混合新一轮；输出 `shadow.jsonl`、`summary.json` 保存topic实际收到/拒绝数、因果配对源龄、缩放后的gyro/COM速度/gravity/joint误差、缺失源、候选点云扫描、参考几何和源码/config哈希。当前camera-only轮的源缺失应如实显示 `unverified_missing`，工具整体始终标 `shadow_unverified`。没有实际数据时不计算数值误差，不以“脚本退出0”宣布输入替换通过。

下列剩余分级验收不重新训练Teacher；有限范围30维替换诊断已经实跑，不代表全部基础运动或Sim2Sim门禁已通过：

1. **采集质量**：启用本轮实际传感器与相应GZ→ROS桥，确认topic、发布者、帧、IMU rate、JointState完整顺序、静止SLAM启动窗口与源哈希。记录站立、六向运动、转向、停止、命令切换、坡道和低台阶三次独立重复，旁路候选全程不执行。
2. **语义与误差门槛**：在A/B前预登记最大源龄、掉帧比例、方向符号、各维误差及扫描覆盖门槛。可将训练噪声范围作为候选误差设计参考，但不能把“在训练噪声内”当性能已保证。建议原始量起始目标：COM速度RMSE≤0.05 m/s、gyro≤0.05 rad/s、gravity方向角≤0.02 rad、q≤0.01 rad、qd≤0.5 rad/s；这些是待冻结的审阅目标，不是本轮已测通过。高度另要求可比有效列误差、overhead/缺点身份与点序检查，覆盖比例单独达标不足以证明完整扫描同义。
3. **一次替换一组**：A为当前特权观测，B依次替换gyro、关节q/qd、gravity、COM线速度，再考虑height候选。命令、上一raw action和软件执行命令在两组保持同样语义。使用同模型/配置/PD/命令序列、同测试场景与fresh spawn，固定随机种子及三次重复；执行器、限幅、停车和安全门不变。
4. **延迟与缺测反例**：B加入已测的源龄、丢帧/乱序/时钟倒退、姿态covariance未知、JointState漏名、足下遮挡、上层平台、动态障碍和扫描空洞。异常必须触发已有受控停车；不能从真值回填以掩盖失效。检查20 ms policy deadline、CPU占用及真实速度、漂移、姿态、接触、力矩命令与目标。
5. **闭环与导航**：全部运动判据以及相同训练环境命令对照通过后，才在整体验收记录感知运动替换通过；导航位置仍只用SLAM，并继续原46区域、坡道、动态障碍停等恢复验收。height语义若因support选择发生改变，单列为新候选及未解决限制，不能写成训练扫描等价。实体部署保持未验证。

## 本地依据

- [冻结适配契约](../policy/contract.json)：九项247维顺序、clip/scale/noise、COM、DCMotor及height引用与归档SHA。
- [实际特权观测实现](../policy/observation.py)：yaw-only网格、+20 starts、base参考、静态SDF射线与缺失/overhead诊断。
- [Teacher执行器](../simulation/teacher_actuator.cpp)、[传感器生成器](../simulation/prepare.py)、[实际测试启动器](../scripts/run_test.py)：力矩命令来源、实际COM惯性、200 Hz传感器选择与各轮真实桥配置。
- [实际消息broker](../scripts/sensor_feedback.py)、[独立30维审计](../scripts/audit_sensor_replacement.py)、[实际替换报告](SENSOR_REPLACEMENT_REPORT.md)：因果源龄、raw保留、精确维度、失败阻尼、CPU actor重放及2/3前进结果。
- [SLAM坐标与差分](../../slam/geometry.py)、[里程计适配](../../slam/odom_adapter.py)、[SLAM配置](../../slam/fastlivo.yaml)：IMU到机身变换、body原点twist、gravity-align与静止初始化。
- [实际传感器桥](../../simulation/config/bridge.yaml)、[实际关节桥](../../simulation/generated/measured_joint_bridge.yaml)、[导航消费端](../../navigation/controller.py)：实际topics、camera_init点云、原始IMU与SLAM输入。

本轮结论：命令、上一动作和限幅执行器命令可由唯一控制器保存；实际IMU与完整关节已完成30维策略替换、单轮站立与资源空闲三清洁前进/停车重复通过，历史来源墙钟失败及清理中断保留。SLAM COM速度、SLAM姿态和187点真实局部点云扫描仍未完成替换闭环；有限30维结果不扩称整体感知、完整Sim2Sim、导航或实机通过。
