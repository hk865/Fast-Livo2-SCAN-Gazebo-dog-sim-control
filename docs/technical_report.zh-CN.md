# 四足机器人仿真导航技术报告

> 算法与历史验证基线为原工程`b29d3eef`；本次独立整改已按职责拆分源码并新增干净构建入口。本文保留原理和历史PID证据，源码路径更新为本包；本轮构建、组件与导航结果单列[验证状态](验证状态.md)，不得用第26轮给新构建代证。

## 阅读导引

本文沿一条实际的数据依赖展开：机器人如何知道自己在哪里，地图如何约束这种判断，规划器如何给出可执行参考，速度控制和学习策略如何把参考变成关节力矩，最后如何判定停车。源码目录及消息索引另见[源码结构与数据流](source_architecture.zh-CN.md)。日常使用从[根 README](../README.md)开始。

全文区分四层证据：**上游原理**解释设计；**上游默认**只解释原项目；**本地实现**由当前源码决定；**当前启用**由 `start.py`、运行配置及生成器共同决定。已有实验只能支持其实际测量过的范围，参数文件中遗留的 `verified`、实验名或历史 hash 不能单独证明今天的功能状态。

## 1. 系统目标：在多平台和坡道上完成一个闭环任务

当前工程将 Go2 模型放进 Gazebo Harmonic 的三个平台及两段约10%坡道场景。默认任务从 F1 前往 F3，导航目标来自配置中的相对位移及既有路线几何。它不是未知环境自主探索，也不是台阶攀爬。真实仿真传感器驱动 FAST-LIVO2；已知静态场景约束局部里程计的漂移；SCAN 生成局部可行轨迹；导航控制器输出机体速度；CPU 上的 Student45/GRU 策略输出十二个关节目标；原生执行器通过关节 PD 力矩推进物理仿真。反馈重新进入传感器与控制闭环。

运行入口只有`start.py`，转交`bringup.main.main()`。它读取模块配置、检查必要资产、准备本次run并取得三把互斥锁，再启动唯一总launch。总launch包含定位、规划、控制、仿真四个子launch；仿真模块等待Actor真实加载并bind/listen后才启动Gazebo。SCAN监督器独占原生planner子进程。模块不是彼此完全独立的融合器，launch列表顺序也不等于传感器就绪保证。

```text
Gazebo物理、IMU、LiDAR、相机
  → ROS桥/relay/自体过滤
  → FAST-LIVO2：IMU传播 → LiDAR更新
      ├─ post-LIO导航提交（在本轮VIO更新前）
      │   → raw里程计/完整扫描 → body/LiDAR外参适配
      │   → 已知场景匹配与导航纠偏 → 同源位姿/点云/坐标代次
      │   → SCAN局部地图与B样条 + 全局路线参考
      │   → 空间跟踪/速度控制 → 命令门控 → Student命令修正
      │   → GRU Actor → 关节目标 → 关节PD → Gazebo
      │   ↘ 到达捕获、实际零输入确认、固定5秒停车观察
      └─ 随后VIO更新同一滤波状态 → 供后续循环使用
          ↘ post-VIO彩云支路 → raw坐标RGB档案
```

这里的箭头表示数据传递和先后依赖，不表示所有模块都在同一线程，也不表示各估计器互相独立。Gazebo 的机器人真值位置不负责导航、路线推进或到达判定；但 Actor 输入的关节、姿态代理与 Student 内环 COM 速度仍来自原生仿真状态。因此当前属于有明确特权反馈边界的 **Gazebo仿真部署与sim2sim适配范围**；完整跨仿真等价未验证，不能写成已完成真机传感器部署。

## 2. 共同语言：坐标、时间与地图

### 2.1 变换的方向比 frame 名称更重要

全文用 `T_A_B` 表示把 B 系中的点变到 A 系：`p_A = R_A_B p_B + t_A_B`。旋转在左，平移以米计，角度以弧度计。机体 body 原点、IMU、LiDAR、相机光学坐标以及质心 COM 是不同物理参考点。比较高度或速度前必须先说明是哪一个参考点。

| 量/坐标 | 语义 | 主要来源与消费者 |
| --- | --- | --- |
| Gazebo `world` | 场景的物理坐标；平台高度0/1.2/2.4m | SDF、传感器与原生执行器；机器人真值位置不直接交给导航 |
| 原始 `camera_init` | FAST-LIVO2 初始化的里程计坐标 | `/aft_mapped_to_init`，经 `odom_adapter`生成 raw body/LiDAR 位姿 |
| 导航 `camera_init` | 对原始里程计应用导航变换后的坐标 | `/demo/slam/body_odom`、纠偏后的 `/cloud_registered_full`、SCAN和到达判定 |
| `demo_slam_body` | 导航所用机体原点 | `src/localization/odom_adapter.py` 的 body 外参适配；与原生 COM 有安装偏移 |
| `C_world_odom` | 里程计到已知场景 world 的纠偏 | matcher 提议，`CorrectionState`受停车事务约束地采用 |
| `T_navigation_world` | 场景 world 到导航坐标 | 当前入口按 `slam_origin_in_world`构造；路线也必须使用相同约定 |

原始与纠偏后的消息可以保留相同 frame 字符串；**字符串相同并不证明属于同一数值坐标代次**。`localization_bundle` 中的 `producer_epoch`、整数源时间、`generation`、纠偏矩阵及 planner epoch 共同保护位姿、点云与路径的一致性。纠偏时旧路径不能继续用于新坐标。

当前导航位姿的组成是：

```text
T_navigation_body(t) = T_navigation_world · C_world_odom(t) · T_odom_body(t)
```

这也解释了为什么目标 Z 误差不等于定位误差：目标 Z 是初始化机体估计加楼层增量；它不是同时刻同一机体参考点的物理真值。C 来自传感器与场景拟合，也不是从目标 Z 反求的“真值修正”。

### 2.2 三种时间服务于三种问题

ROS `/clock` 和消息 header 使用仿真时间；控制的积分、导数及因果配对使用实际源时间差；`time.monotonic()`/steady clock 使用墙钟，处理通信失联、等待和响应截止。仿真暂停时墙钟仍前进，因此源码同时检查源时间与收据年龄，不能把“刚收到旧消息”当成“传感器刚测量”。

控制时钟不前进时先输出精确零并冻结短时积分/驻留；持续停顿进入保护复位；时钟倒退锁存失败。IMU 配对保留真实 FIFO，使用不晚于目标源时刻的记录，最大配对间隔20ms；关键 SLAM/云/IMU 源新鲜度为300ms。它们是当前输入保护，不是过去实验成功文件的授权检查。

| 环节 | 配置/生成的名义频率 | 消费或触发方式 | 已有实测边界 |
| --- | --- | --- | --- |
| Gazebo物理 | 0.005s，即200Hz | 原生 `PreUpdate` | 不等于已测墙钟实时率 |
| IMU | 200Hz | ROS桥→relay→IMU队列 | `src/simulation/world/prepare.py`有效值；旧scenario的1000Hz不是当前值 |
| LiDAR | 30Hz，64×480采样 | 自体过滤→前端测量分组 | 配置率；无本报告新测吞吐量 |
| RGB相机 | 30Hz | relay→前端图像队列 | prepare最初10Hz会被sampling覆盖 |
| FAST-LIVO2 | 事件驱动测量分组 | 依IMU/云/图像可用性，不保证每条都输出 | 不能用传感器率代替定位实际输出率 |
| 已知场景matcher | 每1s仿真时间尝试提交 | 后台单worker，单个待处理槽由新任务覆盖 | 单次预算1.6s墙钟，非保证1Hz完成 |
| 定位/SCAN监督 | 50ms steady timer | 发布状态、代次切换与新鲜bundle | 定时器周期不是端到端延迟 |
| SCAN FSM/地图安全 | 10ms/50ms wall timer | 原生单线程executor | 规划求解占用会影响实际调度 |
| 导航主控制/bridge | 50ms/20ms wall timer | 真实源header决定计算dt | 当前控制不是固定20Hz源采样 |
| Student策略 | 0.02s仿真时间，50Hz | 原生执行器每4物理步请求一次 | IPC墙钟响应截止200ms |
| 固定停车观察 | 0.1s仿真网格，持续5s | 因果选择各格以前的不同header | 第26轮51个样本，源龄最大100ms |

实现依据：`bringup.prepare.prepare()`、`localization.sensors.sampling.override_sensors()`、`src/simulation/world/prepare.py`、`bringup.main.environment()`、`control_clock_contract`与`src/common/integration_limits.py`。本报告没有将名义率包装成新的性能测量。

### 2.3 “地图”至少有七种表示

| 阶段 | 实际表示、尺度与坐标 | 更新/传递 | 静态与动态对象 |
| --- | --- | --- | --- |
| 物理世界 | SDF中的有尺寸、有位姿collision/visual；机器人外观DAE/STL网格 | `assets/world`与生成world；Gazebo物理/传感器读取 | 动态障碍是真实可移动模型；mesh不等于占据地图 |
| 几何资产 | `reference.json`的43个解析盒体，有限面、法向及world变换；历史采样元信息记录≤0.05m网格 | 路线支撑检验读取解析几何；元信息本身不含完整点云数组 | 静态参考排除robot/moving obstacle |
| FAST-LIVO2激光地图 | 0.5m顶层voxel + 自适应分层八叉树，保存点统计、平面与不确定性 | 随LIO更新；当前滑动开关关闭 | 没有语义动态分割；动态点可污染匹配/地图 |
| FAST-LIVO2视觉地图 | 带3D位置/法向的视觉点及多帧patch/参考观测 | 维护视觉点及观测；参考patch更新启用边界见3.4节 | 不是语义纹理mesh，也不是独立全局图优化 |
| 彩色地图档案 | `map_archive`对实际RGB投影点作0.08m稀疏体素聚合 | 走raw位姿/云支路，供展示和存档 | 未应用导航C；不作为全局matcher或SCAN的静态底图 |
| 全局定位参考 | 本次生成world中的静态解析box map | `localization.known_scene_reference_builder.build()`→`SceneReference`，最近面和射线首回波查询 | 不从机器人当前真值位姿求解；也不是独立传感器实测地图 |
| SCAN局部地图 | 0.08m滚动3D log-odds占据、膨胀层和计数/射线临时标签，范围10×10×5m | 纠偏后dense云、LiDAR射线原点/body参考→局部更新 | 当前云保留动态障碍，随观测更新；有额外实际动态清障释放保护 |
| 路线/局部轨迹 | `body_points/ground_points`有序折线、surface IDs、support connector；局部三次B样条 | 路线弧长参考→SCAN全局引导→局部path→空间参考 | 路线表达意图与支撑，不替代当前碰撞检查 |

历史 `metadata.json`记录679,434个float32采样点、258个面；当前几何目录没有对应完整XYZ/face_ids数组，日常入口也不读取它们。不能仅凭元信息说导航在加载一个现成静态大点云。局部地图、RGB档案、解析参考、全局路线之间共享几何含义，但其格式、坐标、更新时机和用途不同。

## 3. FAST-LIVO2：把惯性预测变成可用的局部里程计

上一章规定了传感器和坐标，接下来要解决短时间连续运动估计。FAST-LIVO2 的核心是激光、视觉、惯性紧耦合迭代误差状态滤波。[上游论文与代码](https://github.com/hku-mars/FAST-LIVO2)解释总体设计；当前行为由本地 `src/localization/frontend/fast_livo2_core`及ROS2封装决定。

### 3.1 工程入口、状态与耦合

`fast_livo2_ros/src/main.cpp`经 `fast_livo2_core::run_mapping()`进入 `LIVMapper`；`LIVMapper::run()`读取ROS回调缓存，`sync_packages()`形成测量组，`processImu()`传播/去畸变，随后 `handleLIO()`调用 `VoxelMapManager::StateEstimation()`，`handleVIO()`调用 `VIOManager::processFrame()`。同一个 `StatesGroup`顺序接受LIO与VIO更新，图像和激光的缓存/切分允许按采集时刻处理。导航odometry和dense扫描在post-LIO同一状态、同一源时刻提交；后续VIO状态影响下一轮，彩色显示云可在post-VIO生成，不能混成同一次提交。

它不是为一组历史位姿建立滑窗因子图；图像patch保留多个观测也不等于多状态滑窗优化。它也不是两个各自独立、结果再平均的LIO/VIO。当前 `FASTLIVO_PIPELINE_MODE=serial`；LiDAR雅可比行可用四线程算子并行。原配置曾把VIO patch意图记为1，但实际线程选择读取编译宏`V16_VIO_PATCH_THREADS`。本轮只读核对原成功二进制发现其条件团队为1或4，新构建明确对齐编译上限4与64点条件；环境字符串不能替代编译选项。局部算术并行不改变滤波先后依赖。

误差状态按当前源码排列为：

```text
δx = [δθ(3), δp(3), δτ(1), δv(3), δbg(3), δba(3), δg(3)] ∈ R¹⁹
R ← R · Exp(δθ)
```

R为IMU到里程计世界的姿态，p/v为该状态原点的位置/速度，bg/ba为陀螺/加计偏置，g为重力，τ为逆曝光尺度。`StatesGroup::operator+/-`负责流形增量；旋转不能直接按欧拉角逐元素相加。相机/激光外参来自传感器合同，是模型标定参数，不在此19维状态中一并在线自由估计。

实现落点：`common_lib.h::StatesGroup`、`LIVMapper.cpp::run/sync_packages/handleLIO/handleVIO`。ROS2版本还包含时间、instantaneous type0点云、初始化、运行队列和发布端的工程改动；不能用上游README证明这些改动已不存在。当前`require_complete_imu=false`，不应声称每次都等到IMU完全覆盖图像时刻才处理。

### 3.2 IMU初始化、传播与点云去畸变

启动时要求连续静止数据段。有效配置以200Hz IMU构造600样本、至少2.9s跨度的初始化，检查角速度范数≤0.03rad/s、加速度模长误差≤1.2m/s²、各轴标准差≤0.40m/s²、样本间隔≤0.03s。运动或缺样会丢弃累积均值。当前生成器覆盖base YAML，开启 `gravity_est_en`与`ba_bg_est_en`；只看base文件中false会得出错误结论。

对相邻IMU样本取均值，扣除偏置；加计按初始化平均重力模长校准尺度。离散传播可概括为：

```text
ω = mean(ωm) − bg
a = mean(am) · 9.81 / ||mean_acc_init|| − ba
Rₖ₊₁ = Rₖ Exp(ω Δt)
aW = Rₖ₊₁ a + g
pₖ₊₁ = pₖ + vₖ Δt + ½aW Δt²
vₖ₊₁ = vₖ + aW Δt
Pₖ₊₁ = F Pₖ Fᵀ + Q
```

这里Δt来自源时间，F为误差状态传播雅可比，Q含IMU测量噪声和偏置随机游走。公式是源码离散更新的概括，不是另一个外置IMU姿态滤波器。偏置和重力通过后续观测与协方差耦合得到修正。

旋转扫描中的各点并不同时采样。`UndistortPcl()`使用IMU传播得到的姿态/位置段，把点从其采样时刻补偿到本组切分时刻；点时间偏移由`curvature/1000`转换成秒，随后再经过LiDAR/IMU外参。当前Generic type0仿真输入的逐点偏移为0，表示瞬时扫描，有专门5ns边界容差及首点单次处理修复。它没有从仿真云补出一圈机械扫描的真实逐点时间。初始化重力来自加速度均值；源码初始`bias_g=0`，也不能笼统说初始化直接用陀螺均值完成零偏标定。

### 3.3 LiDAR：不确定性感知的点到平面更新

去畸变点先降采样，变到状态坐标，查询自适应voxel地图中的局部平面。设激光点pL，外参 `(R_IL,t_IL)`，平面上一点q和单位法向n，则一个几何残差为：

```text
pI = R_IL pL + t_IL
rL = nᵀ(R pI + p − q)
wL = 1 / (0.001 + Jnq Σplane Jnqᵀ + nᵀΣpoint n)
```

`Σplane`描述估计平面参数的不确定性，`Σpoint`包含点测量/投影的不确定性，`Jnq`把平面参数误差投到残差上。较可靠的面和点获得更高权重；法向/平面关联错误并不会因为权重公式存在就自动消失。

误差状态迭代融合先验与当前残差，概念上求：

```text
δx* = argmin [ ||(x ⊞ δx) ⊟ x_prior||²_(P_prior⁻¹)
              + Σᵢ wLᵢ rLᵢ(x ⊞ δx)² ]
```

`⊞/⊟`指状态增量/差分，旋转遵循前述SO(3)约定。源码在状态空间用信息式小矩阵求解，更新包括先验差向量，不应简写成无先验的普通点云ICP。`BuildJacobianRows()`构建残差与权重，`StateEstimation()`最多5次迭代并更新P；`BuildResidualListOMP()`及地图节点负责关联。

顶层体素0.5m，最大分层2，平面成立使用最小特征值门限0.005；单节点点数受50点上限等参数约束。它是面向匹配的统计/平面地图，不是导航的0.08m占据网格。当前 `map_sliding_en=false`，因此不能声称已启用按机器人移动滑出的全局定长地图。源码有额外surface-validity分支，日常入口显式设置 `FASTLIVO_SURFACE_VALIDITY=0`，不能把该分支实验当成当前功能。

### 3.4 视觉：用局部图像块修正同一状态

LiDAR几何为视觉点提供3D位置和表面法向。当前图像按预测位姿投影这些点，选择可见/有效参考patch，依据局部平面和视角构造仿射warp，在图像金字塔上做直接光度对齐。默认patch为8×8，金字塔4层，每层最多5次迭代。

设参考patch像素通过warp落到当前图像的u，τ为逆曝光，则光度残差可写成：

```text
rV(u) = τ_current I_current(u) − τ_reference I_reference_warped(u)
∂rV/∂δx 只直接占用旋转、位置和曝光对应的7列
```

位姿雅可比沿图像梯度→投影→几何变换展开，曝光列取决于开关。VIO把这些观测继续更新到LIO后的同一状态与协方差。其余速度/偏置等状态可以通过P的交叉项间接受影响；不是给视觉另建一套独立导航位姿。

当前 `normal_en=true`、`raycast_en=false`、`inverse_composition_en=false`、`exposure_estimate_en=false`：启用法向相关warp，走forward compositional路径，曝光状态仍在结构中但该估计列关闭。base `img_point_cov=1e8`会被模块配置合并得到的`slam_visual_measurement_covariance=1000`覆盖；这个值是像素残差权重参数，不是以米计的定位误差。不能用base的极弱视觉权重描述当前运行，也不能宣称视觉和激光等权。

视觉点保存在独立0.5m视觉hash及`VisualPoint/Feature`中，并共享LIO平面几何，不是只存在一个C++地图容器。当前`processFrame()`会生成和维护视觉观测，但没有调用已定义的`updateReferencePatch()`；不能把论文中的全部参考patch/法向细化机制都计为当前启用。关键实现：`vio.cpp::retrieveFromVisualSparseMap/updateState/processFrame`及core头文件。其输出是同一滤波器的新状态与视觉地图，导航post-LIO提交边界如3.1节所述。后续导航还需要把状态原点变为机体原点，并解决已知场景中的全局一致性问题。

## 4. 全局定位与纠偏：将局部里程计约束到已知场景

局部滤波并不自动保证跨平台长距离的全局位置。当前工程用给定静态场景作导航级纠偏；这是一种已知场景地图辅助定位。它不执行全局建图、回环图优化或全局bundle adjustment，也不修改FAST-LIVO2内部速度与偏置。

### 4.1 查询数据与地图从哪里来

`localization.known_scene_reference_builder.build()`从本次 `world.sdf`提取静态盒体。`SceneReference`保存盒体刚体变换和半尺寸，支持解析最近面与射线相交，排除机器人和动态障碍。使用场景几何是有先验的仿真地图定位；它不等于使用实时机器人真值位姿，也不等于已建成独立实测传感器地图。

匹配查询的`SensorSnapshots`要求raw body和post-LIO dense cloud完全同源、同frame；定位发布端进一步要求body、LiDAR位姿和dense cloud三者整数源时间相同。当前查询用同源body位姿逆变换注册云得到body坐标点，然后与已知场景匹配。这里不能再重复加LiDAR安装平移。保留报告指出历史版本曾从raw/adapted点云加安装平移组装查询，因此“matcher参数相同”不证明历史与当前输入数组相同。

### 4.2 迭代拟合、多假设与质量门

初值的XY和姿态来自当前预测；高度假设由场景中floor/ramp的上下表面及名义机体离地0.30m产生。保留上下表面可以暴露类似楼层/面的歧义。它没有枚举所有XY和yaw，不能宣称任意初始位置下的全局重定位完备性。

每个候选求body到world刚体变换T，近似目标为：

```text
min_T Σᵢ ρₕ( nᵢᵀ(T pBᵢ − qᵢ) )
```

ρₕ为阈值0.06m的Huber损失，pB为同源body点，q/n来自当前候选下的最近解析表面。`_fit()`迭代重建对应关系并求6维位姿步长；单步平移≤0.2m、旋转≤0.05rad，最多40次，总计算预算1.6s。拟合最多900点，评分最多4000点，最少400个静态支撑点。

低残差只说明“贴到某些面上”，不保证贴对楼层，也不保证姿态可观测。因此当前 `static_support_nonforeground/v1`还检查：

| 检查 | 当前门限/含义 |
| --- | --- |
| 静态支撑 | 静态capped RMS≤0.06m；非前景surface P95≤0.12m；静态距离<0.1m点/全部评分点≥90% |
| 首回波可见性 | 非前景射线命中≥90%；behind比例≤12%（观测比首交点后方>0.1m）；绝对误差中位数≤0.045m，P95≤0.35m |
| 几何退化 | 几何信息矩阵特征值比 λmin/λmax≥1e-5；不是滤波P给出的置信度 |
| 多候选歧义 | 位移>0.10m或角度>0.10rad的不同候选，若评分差<0.012m则拒绝 |
| 相对当前C的跃变 | ≤0.65m、≤0.20rad |
| 跨帧确认 | 至少两次，源时差0.15～10s，C相差≤0.15m/0.08rad |

前景判定结合相对静态最近面距离与负首回波残差；目的是允许真实前景遮挡而不把它当静态面。当前quality model不使用旧版5%前景硬上限，不能把旧参数旁注当现行语义。它仍要求足够全体点中的静态支撑。所有门限都是工程质量限制，不是经统计校准的后验正确概率。

### 4.3 从匹配结果到受控坐标更新

匹配给出 `T_world_body_match`，原始同源位姿为 `T_odom_body`，所以：

```text
C_proposed = T_world_body_match · inverse(T_odom_body)
```

初始化只在导航idle且未发布bundle时提交。运行中按同一捕获body比较旧/新C对机体位置的作用；平移>0.04m或旋转>0.025rad才触发新事务。单看C原点平移会把旋转在远离原点处的实际影响算错，因此源码的 `proposal_body_effect()`锁定worker捕获的body，不使用任意最新pose替代。

新事务只在导航状态`running`时启动。匹配在到达后仍可继续，不能据此说到达后仍不断采用新纠偏。事务设`control_allowed=false`，要求精确零请求/命令和实际测得停止；停止条件含机体线速度范数<0.08m/s、yaw角速度<0.1rad/s、至少100ms源时长。C按0.04m/s和0.02rad/s逐步靠近冻结目标。释放需冻结body处残差<0.005m、原点残差<0.005m、旋转<0.003rad，并继续满足停止。

纠偏会改变位姿/云，不对C差分制造物理速度，也不回写前端v/bg/ba。`corrected_odom()`保留child/body twist，旋转pose协方差；这不表示地图匹配的全部不确定性已被完整标定并融合进协方差。

`scan_epoch_supervisor`清理旧坐标下的规划状态、拥有并重启其planner子进程，要求当前epoch下的新云/odom/消费确认，再接纳新路径。这里的request ID、代次、序号和ACK是当前协议的一部分，不能与已移除的历史版本批准/hash运行合同混同。全局定位最终交给规划器的是**同源、同代次的机体位姿和注册云**，而不是一句“地图匹配成功”。

## 5. SCAN：从地图与路线到可跟踪的局部轨迹

定位解决了“在哪里”，但不直接回答“接下来如何安全移动”。本地SCAN规划器结合全局路线方向、实际局部点云、机体状态与几何约束，产生局部三次B样条。它不是DWA式速度窗口枚举，也不是直接输出十二关节动作的策略。

### 5.1 输入及局部地图

`scan_epoch_supervisor`监督同代输入与原生child消费确认、在切换时重建child；实际位姿/云由localization及shared_controller发布，并非主管代为转发。planner的配置默认文件随后被 `planning.scan_runtime_parameters.parameters()`生成的覆盖值替换。当前重要值为：`camera_init`、0.08m地图分辨率、10×10×5m滚动范围、0.65m局部规划horizon、最大速度0.5m/s、最大加速度0.15m/s²。不能引用上游 `planner.yaml`的0.05m、7.5m、0.75m/s作为当前参数。

`GridMap`从实际纠偏云、LiDAR原点位姿（射线起点）和body参考维护log-odds占据证据、raycast与膨胀，不需要先生成全局ESDF。射线真实终点加hit，穿越体素加miss，按`l_new=clip(l_old+log(p/(1-p)),l_min,l_max)`累加；默认hit=.85、miss=.30、occ=.80，最长射线5m。滑动环形buffer保留重叠区域并清理新窗口，真实hit优先，截断边界不能伪造障碍终点。局部地图保留当前动态回波；路线及解析静态地图不替代它。机器人用前后两个圆柱近似包络：半径0.25m、前后偏置0.18m，Z膨胀±0.12m。`body_height=.4m`还用于参考消息的高度传输约定，不宜直接当成整机真实高度。

### 5.2 从种子轨迹到rebound优化

全局路线是337点有序折线及surface/support元信息；当前自动参考链直接提交有序路线点，换代后从实际位姿取有序后缀，避免跨越折返或不相邻平台；`planning.reference.global_route_reference.SegmentGuides`只属于fallback参考分支。当前每次启动读取保留路线资产，没有重新执行生成此PCT路线的完整全局搜索。`SCANPlannerManager::reboundReplan()`从当前实测位置、速度、零起始加速度及局部目标构造初始多项式或复用轨迹，再参数化为B样条。`applyLinearZReference()`按XY累计弧长插值起终点Z；投影A*用XY八邻域、按投影比例给Z，不是自由26邻域三维搜索。碰撞段生成离障基点和方向，rebound优化迭代改变内部控制点。

若Qᵢ为控制点、Δt为结点时间间隔，三次均匀B样条为：

```text
p(t) = Σᵢ Bᵢ,₃(t) Qᵢ
v控制点 = (Qᵢ₊₁ − Qᵢ)/Δt
a控制点 = (Qᵢ₊₂ − 2Qᵢ₊₁ + Qᵢ)/Δt²
```

导数也有B样条表示，所以速度、加速度可通过相邻控制点差分惩罚。当前rebound目标准确写为`J=λs Js+λc Jc+λf Jf`：平滑、碰撞、可行性三项。`Js=Σ||Qᵢ₊₃−3Qᵢ₊₂+3Qᵢ₊₁−Qᵢ||²`，不能无说明地叫作按物理时间归一的jerk积分；碰撞项是沿rebound方向的投影距离不足惩罚；可行性项惩罚各轴速度/加速度超限。refine用fitness项替换碰撞项。两套梯度都把Z行置零，保留初始化高度参考，局部避障主要改变XY。这不是自由空间全3D轨迹优化，也没有完整足接触规划。

L-BFGS优化内部控制点，首末各3点固定，memory16、最多200次迭代；可行性检查失败时可延长时间、重新参数化/refine或返回失败。额外动态检查按0.01～0.05s采样，实际门限包含默认各1.0的速度/加速度tolerance；配置`max_vel=.5/max_acc=.15`不是数学上零容差硬界。读取`max_jerk`也不代表最终检查已硬约束jerk。成功求解还需要外层路径准入。

上游[SCAN论文](https://arxiv.org/html/2606.19555v1)及[作者代码](https://github.com/wuyi2121/SCAN-Planner)提供方向相关双圆柱、投影A*、高度保持与滚动地图思想；本工程的代次监督、源配对、实测起始状态、短接收、路径交接及当前小horizon是集成层扩展。源码中保留的演示open/closed-loop controller不负责当前主Student控制链。上游楼梯和真机结果不作为本工程验证。

### 5.3 输出、交接与动态停车

原生planner发布B样条、元数据及显示Marker；Python导航采样B样条并发布`/demo/navigation/path`，从有效path构造空间参考，接纳时核对请求、源、frame和坐标代次。`path_admission`、`pending_path_commit`、`path_handoff`等模块把几何检查和状态提交分开，防止求解中途用旧代次结果覆盖当前参考。`planning.admission.static_geometry.certify_physical_curve()`将三次段变为Bézier，通过凸包、有限支撑面并集、OBB分离与递归细分作静态连续空间检查；最多4096patch、深度18、墙钟2s，未能证明/预算耗尽返回`UNKNOWN_REJECTED`，不等于已证碰撞。该计算由single-flight独立worker进程执行。它不证明动态安全、足接触或实际步态稳定；`corridor`保留为实验shadow观察，普通入口已关闭。

资产中的body-ground高度差约0.32m；Path传输先减`.4`，C++再加`body_height=.4`，是一对编码/解码约定，不可把二者混成真实地形高度。SCAN接收的body twist经Python显式旋转为世界速度，child名`demo_scan_body_world_twist`；不能按标准child-frame twist直接理解它。

实际障碍保护在当前点云和机体状态上工作：前方0.30～0.95m、横向±0.42m，按路线body高度筛选body_z−.12到body_z+.60m范围内至少3个回波；转向时对路线方向与实际转向方向走廊取保守并集。没有按Gazebo实体名识别动态物体、追踪速度或预测未来轨迹。发现障碍/过倾/源失效可发精确零；动态障碍清除后，必须至少十次真实clear评估、跨度至少1s、最大间隔300ms，再释放并观测恢复。配置定时器运行多次不等于真实clear样本已经满足条件。第26轮测得一次动态停走，采用十二次clear评估、1.000s跨度与0.93804m后续恢复，详见第9章。

SCAN的产物是带有效性的局部运动参考。把参考变成稳健的速度，再变成足部运动，分别属于下一章的导航控制和学习控制。

## 6. RL：能够复核的部署，以及不能补写的训练历史

### 6.1 先回答“机器人如何训练出来”能回答到哪一步

导航包提供CPU部署、观测构造、Student适配器、接口配置和JIT权重。初次文档核对只覆盖导航树，因此没有把它当成训练源码。后续在独立本地训练工程中找到了016训练、017导出、formal配置/审计和原始checkpoint；下面据这些新材料收紧原先的缺口，不从部署.pt倒推训练。

| 问题 | 已核实材料与结论 | 边界 |
| --- | --- | --- |
| 训练入口 | `scripts/train_go2_deployable_016.py`选择RSL-RL DistillationRunner或OnPolicyRunner；016配置是两个实验臂 | 不能把对照Asymmetric PPO的critic/损失写进部署Student |
| Student任务/优化 | 45维可部署观测，冻结Teacher247动作监督；fresh Student及optimizer，Huber/Adam，LR5e-4、2epochs、BPTT24、gradnorm1、5000updates、4096env | 环境有奖励，不等于Student优化器直接使用环境reward |
| 网络 | agent配置为45→单层GRU256→256→128→12，ELU，无观测归一化 | 部署确定性动作与隐状态接口另做导出核验 |
| 初始化 | initialization_audit记录Teacher tensor exact、requires_grad=false、Student新初始化 | 015 Teacher由Rough002父权重warm-start，更早谱系未逐级核完 |
| 完成记录 | model_4999现存；training_audit为passed/5000updates、标量有限、Teacher冻结检查 | completion早期文件写audit_pending，不能只看单一文件猜审计结果 |
| 导出关系 | 017导出审计与4999/agent哈希吻合；部署25个ZIP成员与原导出对应成员逐字节相同，只移除10个debug成员 | 这不单独证明Gazebo关节映射、动作缩放或泛化 |
| 课程/域随机化 | 016配置和formal env.yaml存在 | 需继续逐项对照实际task注册/奖励/终止实现；本文不把未核完的配置写成已验证行为 |

Student动作蒸馏优化的是Student与冻结Teacher动作的差。现存配置指定Huber损失；本次核对没有为其补写未逐项确认的delta/奖励系数。Asymmetric PPO对照臂使用45维actor与训练时247维critic，是另一条训练路径。当前导航使用016的4999导出，而非017后续继续训练的9999。

独立训练仓库配置远程为`hk865/My_RL_for_unitree_learned`，本次连接读取404，不能据此认定已经公开。016脚本、017导出审计脚本和016配置当前未被该仓库跟踪；仅给其HEAD不足以复现，需收集实际源码与内容身份。完整训练环境、所有奖励/课程/随机化和更早Teacher父链尚未在本包闭环，因此导航源码包不承诺一条命令重训练。

`src/control/policy/native/contract.json`和归档Teacher循环记录旧Teacher为247→512→256→128→12、ELU、Gaussian mean，无观测归一化。247维含COM线速度3、角速度3、重力3、命令3、q12、qd12、上一已施加力矩12、上一raw action12与高度187。高度为17×11、0.1m格、yaw-only对齐，从base_z+20m向下射线，按clip(base_z−hit_z−.5,−1,1)编码；缺击中不能随意填零。当前Student输入是45维；原生高度与247有限性路径仍保留作为既有仿真安全边界，不因去历史入口而删掉。

Teacher归档字段记录摩擦、质量、COM/PD/reset随机化及速度扰动、命令分布；这些字段不能证明Student继承了完全相同的随机化。权重所有权与再分发声明也未因找到源码而自动闭合，详见[资产与来源](资产与来源.md)。

### 6.2 当前Student观测、记忆和动作

`src/control/policy/student.py::exact_student45()`构造严格45维观测：

```text
obs45 = [0.2ωB, R_WBᵀ(0,0,−1), command3, q−q0, 0.05qdot, raw_action_previous]
q0 = [−.1,.8,−1.5, .1,.8,−1.5, −.1,1.,−1.5, .1,1.,−1.5] rad
```

依次为角速度3、投影重力3、实际策略命令3、相对默认关节位置12、关节速度12、上一帧原始动作12。没有额外height、linear velocity、全局pose、effort或contact直接输入Student。Teacher247仍作为诊断/原有provider路径存在，不能把它说成Student实际输入。腿顺序FR、FL、RR、RL，每腿hip/thigh/calf，逐名映射Gazebo的rf/lf/rh/lh关节。

观测中的原生gyro/quaternion/q/qd仍是仿真代理；投影重力由姿态变换得到。Student命令还受使用原生COM速度的内环PID修正，因此“45维没有线速度”并不代表整条闭环完全没有特权线速度。

模型以CPU执行，隐状态形状`[1,1,256]`，每0.02s仿真时间更新一次。隐状态仅在run/robot启动复位，不在零命令、换路、waypoint、PID复位时清空。否则每次停车都重置步态记忆会改变已验证策略的动力学行为。

```text
(actionₖ, hiddenₖ₊₁) = Student(obs45ₖ, hiddenₖ)
q_target = q_default + 0.25 · actionₖ
```

动作没有在Student适配器额外clip；执行器仍检查目标有限值/绝对界并施加力矩/速度/关节约束。关节顺序由`src/control/policy/native/contract.json`与原生joint names共同定义，不能仅按URDF遍历顺序接线。上一帧动作保留原始策略输出，不能把施加的PD力矩或受限后的速度放进该字段。

### 6.3 从导出模型到物理执行

`src/control/policy/worker.py`使用`control.policy.native.worker`提供原生IPC循环，显式注入Student加载、观测/命令适配与帧完成回调。旧Teacher加载和脚本命令表归入实验档案；当前正常链固定Student，不热切换策略。原生请求为64个little-endian float64，即512字节，响应16个float64；包中携带仿真时钟、原生状态、目标及协议字段。该socket是原始float64帧，源码没有CRC字段；其有限值、时钟与响应截止检查仍有效。其他SCAN协议/观测通道的序号与CRC必须单独理解，不能搬到此socket包上。

下一次原生请求到达才证明上一策略响应完成，worker据此绑定实际已知命令ACK。写入命令文件只说明“请求已发布”，不能用它直接证明Actor已经收到零命令。Actor经过修正、限幅、单一slew和ordinary zero适配后的命令才进入obs45；该因果ACK又服务于控制积分与停车资格。

物理步长0.005s，策略decimation=4。`TeacherActuator::PreUpdate()`是唯一关节力写入者，关节PD近似：

```text
τ_joint = saturate[ Kp_joint(q_target − q) − Kd_joint qdot ]
```

饱和上下界随实测qd变化，不能只理解为常数clip。默认Kp=25N·m/rad、Kd=0.5N·m·s/rad，力矩/饱和力矩23.5N·m，速度界30rad/s；实际SDF会传入参数，应以生成文件为准。故障阻尼另有分支。机器人不是通过base pose伺服或额外机体推力“沿路线拖动”；仅启动前做一次关节位置初始化，随后由关节力矩及接触物理推进。

## 7. PID与执行：三层反馈，各有职责

上一章的策略只知道速度命令。路径跟踪、消除速度偏差和关节执行因此分成不同层，不能把所有控制都叫同一个PID。

### 7.1 当前导航外环

当前 `PIDNavigation`采用 `control.tracking.cascade_core.Controller`；`ResetBookkeeping`保留复位统计但不执行legacy数学。顶层 `config.pid`及`control.tracking.pid_core.HeaderPID`属于旧链，不能作为当前主外环的有效参数表。

当前空间参考沿SCAN path按弧长取点，计算横向误差、切线方向及曲率。外环组合横向PD、yaw PD/曲率前馈与速度PI，使用实际SLAM机体源及因果IMU角速度。设误差单位分别为米、米每秒或弧度/弧度每秒：

```text
vXY_ref = vs t + clip(−0.8 e_cross − 0.2 vXY_filteredᵀ n, ±0.2) n
yaw_ff = κ vs
yawrate_ref = 1.3 e_yaw + 0.15(yaw_ff − yawdot_filtered) + yaw_ff
u_raw = v_ref + Kp_v(v_ref − v_measured_filtered) + Ki_v I
```

其中t/n为路径XY单位切/法向，κ为曲率，vs为已受剩余弧长及曲率预算限制的参考速度。上述yaw式对应曲率前馈开启的跟踪分支；捕获/转向分支另有更紧控制界。速度/角速度来自实际传感器或SLAM twist，不能对含C跳变的纠偏位置差分生成物理速度。原点速度转换到COM使用`v_COM^B=v_origin^B+ω_B×r_COM^B`，当前COM偏移为`[.0552309203,−.0018690019,.0060952992]m`。Euler yawdot也不等于body wz：`yawdot=(sin(roll)ωy+cos(roll)ωz)/cos(pitch)`。

| 当前外环参数 | 值 | 作用/量纲说明 |
| --- | --- | --- |
| cross Kp / Kd | 0.8s⁻¹ / 0.2 | 横向位置PD，横向修正界±0.2m/s |
| yaw Kp / Kd | 1.3s⁻¹ / 0.15 | 航向误差与实测角速度反馈 |
| vx、vy速度PI Kp | 0.6、0.4 | 速度误差到速度修正，比例无量纲 |
| vx、vy速度PI Ki | 0.5、0.3s⁻¹ | 积分单位m，输出m/s |
| yaw rate PI Kp / Ki | 0.3 / 0.2s⁻¹ | rad/s误差到角速度修正 |
| 速度PI积分界 | 各轴±0.5 | XY单位m、yaw单位rad；不是命令界 |
| 外/内层测量低通τ | 0.2s / 0.1s | 原点世界vxy/Euler yawdot；COM body vxy/body wz |
| 期望巡航速度 | 0.5m/s | 可受曲率、目标/保护状态限制 |
| 命令界vx、vy、wz | ±0.6m/s、±0.2m/s、±0.6rad/s | profile cascade层；后续Student/bridge还可能更严格 |
| 命令slew | 0.6、0.6m/s²；0.8rad/s² | 不能将限速率理解成真实机体加速度保证 |
| 空间参考弧长/导数步长 | 0.20m / 0.05m | 弦偏差≤0.03m，最小弦比0.5，曲率前馈与曲率限速开启 |

积分按实际源dt累计；zero/protection、路径/运动状态切换、饱和与ACK反馈决定冻结/清零/反饱和处理。没有实际执行确认时，不把请求速度当已实现速度继续积累。`src/control/controller.py::ensure_cascade()`、`src/control/tracking/cascade_core.py`及`src/control/policy/lower_loop_ack.py`是阅读主线。

### 7.2 Student速度修正PID

Student之前还有一层固定 `CorrectionOnlyPID`，以原生COM机体系vx/vy和ωz对命令作小幅修正；它不接管全局位置和到达判定。设每轴期望与测量之差为e，滤波导数为d，则：

```text
I_candidate = clip(I + eΔt, ±Imax)
I_next = 同向饱和时保留I，否则采用I_candidate
α = Δt/(0.08+Δt)
d ← d_old + α(−(y−y_old)/Δt − d_old)
Δu = clip(Kp e + Ki I_next + Kd d, ±Δumax)
u_pid = clip(request + Δu, ±command_bounds)
```

这里y为实测速度，微分对测量，避免命令跳变直接产生D kick。固定native状态dt需约20ms。精确饱和反馈与复位次序见`control.policy.student.CorrectionOnlyPID`。随后仍只调用原有worker的一次slew，不再叠加第二个slew缓存。

| 轴 | Kp | Ki(s⁻¹) | Kd(s) | 积分界 | 修正界 |
| --- | --- | --- | --- | --- | --- |
| vx | 0.5 | 0.2 | 0.02 | ±0.15m | ±0.08m/s |
| vy | 0.5 | 0.2 | 0.02 | ±0.4m | ±0.1m/s |
| wz | 0.5 | 0.15 | 0.015 | ±0.2rad | ±0.08rad/s |

导数时间常数0.08s。运行启动、已接纳request/waypoint运动epoch变化、原有精确零请求会触发相应复位；不因每次命令数值变化或trajectory ID变化就复位，不默认按方向翻转复位。零请求不叠加PID修正。当前0.5m/s speed tier将Student最终界设为`[.6,.2,.3]`，即vx/vy为m/s、wz为rad/s；cascade层yaw上界.6还会被下层收紧为.3，实际ACK残差参与抗饱和。旧base `command_bounds=[0.3,0.2,0.3]`不能直接当最终有效纵向上限。唯一worker slew每20ms最大增量为`[.012,.012,.016]`。

### 7.3 当初如何调参：保留的事实与缺失的历史

历史过程有四份保留报告支持，见本章末H1～H4来源表。它不是一次系统辨识完成后直接算出PID，而是先发现整链误差，再控制变量比较有限候选，最后把选定结构迁移到SLAM。

**早期直接位置PID。** 旧HeaderPID位置P/I/D为`.6/.08/.12`，yaw为`1.1/.04/.15`。V2短往返虽能到达/停车，前进速度滚窗比例`.868757<.9`，整体失败；V3把XY slew从`.15`提升到`.30m/s²`后比例`.919240`合格，但90s目标未达、航向`.379738>.35rad`仍失败。V4同版本短往返Teacher3/3、CHAMP0/3只说明当时两个整链的有限比较；延长至6m后V4航向失败。V5引入route-leg方向后航向合格，但路线最大偏差`.50488m`、RMS`.22044m`仍超限。这些结果不能因局部指标变好而改写成整体通过。

**用真值分离控制响应。** 2026-10-05的标定先暂时使用真值反馈，把SLAM误差从控制响应比较中分开。A2阶段为三种控制结构×10/25/50Hz，共9次；B阶段单因素增益比较9次；C阶段前三候选各重复3次，共9次。27次均满足当时冻结检查，有限矩阵选择了路径/heading PD＋COM速度PI及25Hz。0.3m/s命令下，旧位置/路径PD实速约0.24m/s，cascade PI约0.296m/s；速度积分改善了稳态欠速。三种频率的均速仅差0.000115m/s，25Hz评分51.061优于10Hz的38.242与50Hz的39.051，主要来自停车漂移。因此结论是有限样本下的折中，不是“越快越好”或通用频率定律。

历史还有35次留出运动验证，涵盖直线、折线、6m往返、ramp12/23上下及5/10cm持续登台。它们属于旧Teacher、真值反馈、特权高度场景；固定初态/seed重复不能当随机扰动鲁棒性，也不能拿来宣称当前Student、真机或完整SCAN多层导航都通过。新控制律与Isaac完全同条件的闭环对照没有完成。

**显式迁移到SLAM。** 随后冻结cascade结构和增益，跑了6m单程3次、6+6m往返3次，保留报告记132/132门通过。该时期真实不同SLAM header约10Hz，控制仅在新源到达时更新，明确接受从25Hz真值标定降率；50Hz Actor或wall ticker不是数学更新率。TTL仍300ms。这些是当时固定平地相对路线的结果，不覆盖今天的全局注册/多层SCAN/Student。当前30Hz高密传感配置是后续演进，不能倒写到旧试验。

**停车失败推动capture设计。** 旧tightS25首次5s yaw漂移`.114521697>.1rad`；该段已经绕过PI，不能简单归咎25Hz过慢。弱capture V3跑满180s仍距目标约66mm，命令接近`.06m/s`上限而原点实速近零，失败；V4同时提高靠近P与速度包络，未逐项分离贡献。一个有效先导加V4r2三次新确认共4/4，首次窗口XY约4.45mm/yaw`.002409rad`，但旧规则允许非零速度主动保持，反馈为25Hz真值、独立平面。它与今天的actual-zero pose-only停车是不同合同；另外三轮缺可选源档只能称未验证，不能合计成八轮都通过。

现存后期profile/helper还保留转向减速、积分卸载、yaw PD fade、measurement P retention、heading taper、支撑捕获、XY stop takeover等候选，当前默认全部关闭。候选文件存在不证明实际运行或获胜。当前Student修正PID首轮完整搜索/选参记录、训练loss和可复现导出实现仍**无法从现存材料核实**；已存在的有限候选标定也没有完成传递函数/频响拟合、稳定裕度分析或全局最优证明。

| 历史来源 | 可支持的事实 | 不能替代的当前证据 |
| --- | --- | --- |
| H1《PID_NAVIGATION_REPORT》 | V2～V5整链失败与旧HeaderPID演进 | 当前cascade有效参数 |
| H2《TRUTH_PID_CALIBRATION_REPORT_20261005》 | A2/B/C共27次、候选选择、35次留出 | Student训练或当前导航可靠率 |
| H3《SLAM_CONTROLLER_TRANSFER_REPORT_20261005》 | 冻结结构向实际SLAM迁移、6次/132门、约10Hz新源 | 当前多层SCAN/全局纠偏精度 |
| H4《ACTIVE_PARKING_REPORT_20261005》 | 旧主动保持失败和capture改进 | 当前精确零输入固定5s判据 |

H1～H4是本次定向读取的本地保留技术报告，未将其私有宿主路径或原始审计数据复制到公开文档，也未重新回放旧数据。第19/25轮observer错误及第26轮当前结果另由正式`phase_summary.md`记录；历史控制标定、现行控制参数与当前整链验证各自保留边界。

### 7.4 从到达捕获到固定5秒停车

位置接近目标、低速度、输入为零以及稳定停车是不同事件。当前 `ordinary_terminal_parking_contract`先保留普通到达窗口，再进行终端捕获：XY≤0.025m、yaw≤0.035rad、捕获驻留0.6s，并要求原有精确零请求被实际下环ACK。**第一次**满足条件的时刻锁定为T0，锚点随之固定，不允许后来滑动重选一个更好看的窗口。

随后以100ms网格观察不可变5s区间：每格选择不晚于该格的、不同header的最近实际机体pose；不插值，源龄≤300ms。相对T0最大XY漂移≤0.05m、yaw漂移≤0.1rad。控制状态只检查实际收到于窗口内的零hold，不把T0前状态搬进来；边界与内部覆盖间隔≤300ms。

这里的规则是XY/yaw pose-only。高频速度可诊断摇晃/来源/控制过程，不是附加停车否决项；Z仍可报告，但不是该规则的第三个停车阈值。这不等于“Z无需关心”，而是避免把另一种验收模式偷换到已经定义的停车判据。动态障碍停车、纠偏停车、源失效停车与终点停车共享零输入和来源保护，触发与释放条件各有不同，不能互相代证。

## 8. 通信、线程与工程修复

算法闭环还依赖消息及时送达。当前系统同时使用ROS2/DDS消息、普通原子JSON状态文件以及Actor的Unix socket；这些通道不能混为一个DDS瓶颈。

### 8.1 进程、队列与计算所有权

传感器relay保持原消息payload与header，不新增应用队列；它只为IMU/RGB两个writer选async发布。FAST-LIVO2在serial主owner中优先服务IMU，再接收普通回调并执行估计；两个SingleThreadedExecutor的1ms intake预算不能抢占正在运行的callback。入站队列限512项/64MiB，通用点云另限8帧、每帧≤65536点；不要把某个DDS subscription深度当成所有缓存规模。LIO雅可比算子使用4线程；VIO线程配置与构建元数据差异见3.1节。`SensorSnapshots`和定位发布侧各最多12个同源槽；`BackgroundMatcher`是单worker、一个可覆盖待处理job槽和一个结果槽，不建立无界任务队列。

导航控制用两线程 `MultiThreadedExecutor`及当前positive-yield包装，另有路径物理可行性后台工作/待提交队列。原生SCAN单线程spin，FSM、地图、安全wall timer共享该executor。worker主循环与Gazebo执行器按请求/响应同步推进策略步，deadline处理墙钟超时。已有异步记录采用有界诊断队列，不能把“日志已入队”当物理动作已经完成。

这些安排的目标是避免重计算和大日志占用关键回调。它们不构成硬实时保证；Linux调度、Python GIL、DDS内部线程、规划求解和物理渲染仍可引入延迟。生产消费应按源header、回调收据、求解耗时、publish返回、DDS take和实际ACK分别测量。

### 8.2 SHM健康检查为何会拖慢无关发布

Fast DDS共享内存端口检查依赖listener heartbeat。watchdog刚检查过一个健康闲置listener时，counter与last_verified_counter相同；随后opener可能等待listener下一次周期刷新。在1000ms健康超时配置下，等待步长约为timeout/3，即333ms；当open路径持有相关锁时，延迟可波及其他发布。上游[问题#6535](https://github.com/eProsima/Fast-DDS/issues/6535)和[修复PR#6559](https://github.com/eProsima/Fast-DDS/pull/6559)描述了这一机制。

本地2.14.6窄范围回移植修改 `SharedMemGlobal.hpp`两处：checker通知等待listener并在1ms后重试；listener即使被无数据唤醒也刷新heartbeat。只改其中一端不足以避免原等待。数据可用/close快路径、总健康超时、可靠QoS以及SHM+loopback UDP拓扑保持原语义。补丁不能被描述成“关闭可靠性”“绕过死锁检测”或“只升级Python发布模式”。

`third_party/fastdds/`保留生产patch、正向/负向回归fixture和修正watchdog时间的test-only patch；`common.communication.dds_environment.configure()`把项目私有库目录置于运行库路径前端，两端都必须实际加载该构建。源码来源与build hash用于可复现构建，不重新成为日常启动的历史批准合同。本轮已重新编译并执行5项SHM回归，运行时也读取通信两端的实际库映射，均指向本包install/dds；这不替代完整导航验收。

### 8.3 不同故障必须分别归因

DDS修复解释一种SHM健康检查等待。实际工程还出现过控制热路径成本、队列服务/接收压力、缺失因果IMU缓存、旧route SHA消费、已退役archive接口、停车source/helper耦合、observer加入错误速度否决或T0前状态等。它们不是同一原因。调试应先确定停在哪里，再把source age、CPU时间、墙钟、DDS take、命令ACK和物理运动分开。

日常入口不读取旧成功记录、冻结source/model/binary清单、审计journal或PASS文件。独立工具 `tools/validation/run_navigation_check.py`才会启动本轮测量、移动`moving_obstacle`、观察行为并结束它自己拥有的入口；普通启动不导入或调用它。保留实数有限值、frame/代次一致、IMU配对、新鲜度、障碍/倾斜保护、命令界和截止检查，不等于恢复旧的版本批准机制。

## 9. 已验证的范围与不能外推的结论

本轮模块化源树的新构建已在原场景完成一次F1→F3、动态停走和固定5秒停车：XY最大漂移2.407cm、yaw 0.01543rad，51格因果pose；entry/observer和15个launch常驻组件均退出0，几何worker及owned SCAN child也正常退出。所有原安全与停车阈值保持，具体资源、失败修复与证据边界见[本轮验证](验证状态.md)。最终Dockerfile额外全新镜像构建单列，不与这轮源树运行混为一次测量。

第26轮记录在 `docs/validation_summary.json`与`phase_summary.md`。被测可执行源码为`baa5d067`；后续本地提交修正文档中的Z解释。该表只记录原运行；本次新增验证单列[验证状态](验证状态.md)。

| 项目 | 第26轮记录 |
| --- | --- |
| 入口与路线 | 默认`start.py`，F1→F3，entry退出码0 |
| 实际SLAM路线推进 | 34.882590m |
| 动态停走 | 一次实际stop/clear/recovery通过；12次clear、1.000s；恢复0.938043m |
| 固定停车窗口 | T0=441.645s，T1=446.645s，51个因果10Hz样本 |
| 最大XY/yaw漂移 | 0.03052665m，即3.052665cm；0.01201929rad |
| 数据覆盖 | pose源龄最大100ms；窗口内44个实际零hold状态，最大gap165ms |
| Z观测 | 锚点最大变化0.027125m；目标Z最大差0.014661m；不作为ATE |
| 辅助检查 | 保留记录十五项focused tests通过，十八owned进程实际装载项目依赖 |

一次完整运行支持“该次默认入口完成该路线、动态停走与既定停车规则”。它不能自动证明重复运行可靠率、任意地形、真机、台阶、全46区任务或毫米级全局定位精度。3.0527cm是固定锚点后的停车XY漂移，不是整条路线的绝对定位误差。

第21轮曾见目标Z差0.115630m、锚点Z差0.085899m。后续只读复核发现其5秒内C保持相同，generation138/planner epoch9/transaction8不变；目标本身是初始化相对目标，同窗口同物理机体参考点的真值轨迹未对齐。因而不能把它定性成新增定位故障，也不能反过来证明估计精确。前端传播、机体真实运动、参考点偏移与坐标处理的贡献尚未分离，没有计算同源Z绝对轨迹误差。

原工程部分native FAST-LIVO2关停曾在ROS context已无效时出现非零退出；外层entry退出0、进程消失及锁释放应分别表述。本轮修正已有normal-stop/drain与context关闭顺序，完整场景的SLAM及其他组件均退出0。原两次CPU资源保护中止的历史根因仍未闭合，不能用这次成功反写。原迁移当时复用了本地二进制；本轮公共环境的干净构建与新运行已通过，不再查找原.local，范围见本轮状态。完整本地包包含Student与L1，公开再分发许可仍按具体资产核验。

## 10. 实现索引、来源与后续可复核工作

| 本文主题 | 首先阅读的源码/报告 | 核心符号 |
| --- | --- | --- |
| 生命周期与配置 | `src/bringup/main.py`、`src/bringup/prepare.py`、`src/bringup/launch/demo.launch.py` | `main`、`prepare`、`generate_launch_description` |
| 误差状态/IMU | `src/localization/frontend/fast_livo2_core/include/fast_livo2_core/core/common_lib.h`、同core下`src/IMU_Processing.cpp` | `StatesGroup`、`ImuProcess::Process2/UndistortPcl` |
| 激光/视觉 | 同core下`src/voxel_map.cpp`、`src/vio.cpp`、`src/LIVMapper.cpp` | `StateEstimation`、`updateState`、`handleLIO/handleVIO` |
| 地图定位 | `src/localization/known_scene_matcher.py`、`src/localization/sensor_snapshot.py`、`src/localization/localization_runtime.py` | `KnownSceneMatcher`、`SceneReference`、`proposal_body_effect` |
| SCAN | `src/planning/scan/plan_manage`、`plan_env`、`bspline_opt`、`path_searching` | `SCANPlannerManager`、`GridMap`、`BsplineOptimizer`、`AStar` |
| 实际控制 | `src/control/tracking/cascade_core.py`、`src/control/controller.py`、`src/control/policy/student.py` | `Controller`、`PIDNavigation`、`CorrectionOnlyPID` |
| 策略/物理 | `src/control/policy/native`、`src/control/policy/worker.py`、`src/simulation/actuators/teacher_actuator.cpp` | `build_observation`、`StudentRuntime`、`TeacherActuator` |
| 停车/独立观测 | `src/control/parking/ordinary_parking.py`、`src/control/parking/parking_pose10hz.py`、`tools/validation` | 固定锚点、因果源采样、received-status窗口 |
| 已有验证与修复 | `docs/phase_summary.md`、`validation_summary.json`、`fix_registry.md` | 第26轮与各自限制 |

正式的外部原理来源如下；本地实现偏离上游时，以本文的启用说明和源码为准：

1. Zheng等，**FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry**：[论文固定版本](https://arxiv.org/abs/2408.14035v2)、[官方源码](https://github.com/hku-mars/FAST-LIVO2)。用于滤波耦合、激光/视觉直接更新原理；不为本地ROS2接线和运行结果背书。
2. Zheng等，**SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation**：[论文固定版本](https://arxiv.org/abs/2606.19555v1)、[作者代码](https://github.com/wuyi2121/SCAN-Planner)。局部rebound/B样条实现应同时看本地源码，不以算法名字代替核对。
3. Zhou等，**EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors**：[官方源码](https://github.com/ZJU-FAST-Lab/ego-planner)。用于理解rebound/B样条算法谱系；SCAN已加入方向相关机身碰撞和高度保持，不能说成纯原版EGO-Planner。
4. [ROS2 Jazzy executor官方说明](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html)、[时钟设计](https://design.ros2.org/articles/clock_and_time.html)：用于解释executor与时钟域；不证明本工程实际吞吐量。
5. eProsima [Fast DDS SHM问题#6535](https://github.com/eProsima/Fast-DDS/issues/6535)、[PR#6559](https://github.com/eProsima/Fast-DDS/pull/6559)：用于heartbeat等待机制；本地补丁及fixture证明具体回移植范围。上游状态可能后续变化。

本次已补上公共环境干净构建、模块化源码和一轮原场景验证。后续仍需补齐完整训练复现材料、同物理参考点的时间对齐真值、跨机器移植和多轮统计验证；这些分别回答训练来源、绝对精度、环境适配和可靠性问题，不能用一次停车PASS相互替代。
