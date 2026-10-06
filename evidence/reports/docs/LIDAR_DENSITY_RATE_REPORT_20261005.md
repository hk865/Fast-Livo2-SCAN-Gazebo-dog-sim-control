# 增加 LiDAR 线数、30 Hz 与 Teacher CPU/GPU 实测

本轮按用户要求实际运行 Gazebo，对比64线和30 Hz。冻结 Teacher 未变，导航仍取实际 SLAM/IMU/点云及 SCAN；仿真真值只用于离线验收。没有操作实体机器人、重新训练、修改另一任务训练或已通过的 camera_mode。

## 试验设计

保留原32线×480、LiDAR10 Hz、车载RGB10 Hz的V7诊断基线。新V8只在新生成世界中改传感器采样：64×480、LiDAR10/30 Hz、车载RGB10/30 Hz。相机D/K、视场、噪声、外参、物理5 ms、Teacher50 Hz、关节PD200 Hz、路线、速度/朝向控制律、300 ms双期限、300 mm高度管道、到达判据和唯一执行器均保持。每组实际运行210 s，属于有限前缀；单次对照不能证明随机化鲁棒性。

当前FAST-LIVO2的LIVO帧由相机驱动。LiDAR30/RGB10可为约10 Hz的前端提供更多点云，但不能称为30 Hz SLAM反馈。另测RGB30以检验实际前端提频；30 Hz在5 ms物理步长上以30/35 ms原始header间隔实现，统计平均约30.30 Hz。控制器墙钟ticker仍20 Hz，不因传感器设置变成30 Hz控制器。

独立源码为 `navigation/lidar_sampling_v8/`，复用V7冻结的诊断core/ROS二进制，不改变匹配算法。RGB30静止注册缓存容量由10改为26，以保留原不少于0.8 s样本跨度，实际注册26对、0.825 s、IMU配对最大10 ms。没有缩短初始化或保护窗口。

## 已完成的采样对照

|采样与版本|实际run|区域|整体结论与近因|
|---|---|---:|---|
|32线、LiDAR10、RGB10，V7|`20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf`|16/32|失败：133.13 s高度管道保护；错误墙面法向约束|
|64线、LiDAR30、RGB10，V8|`20261005_192400_closed_loop_cascade_lidar64_30hz_rgb10_r1_f728`|22/32|失败：190.365 s避障guard误把相同计算ROS时刻当永久故障|
|64线、LiDAR30、RGB30，V8|`20261005_192802_closed_loop_cascade_lidar64_30hz_rgb30_r1_cb5c`|3/32|失败：61.285 s连续输入不可用超过8 s；算法消费旧数据积压|
|64线、LiDAR10、RGB10，V8|`20261005_193528_closed_loop_cascade_lidar64_10hz_rgb10_density_r1_22ca`|17/32|210 s前缀未完成32区域，整体失败；重复时钟报错仅发生在210.01 s清理，物理期无failed|

原失败、共同验收和清理状态均保留。worker完成210 s、10501次推理不等于导航通过。

密度only组在131–134 s相对高度误差max16.14 mm，native+2.996 mm、SLAM+1.390 mm；全有效期max181.0 mm，仍有后续高度漂移。物理期918条状态均running，执行与清理边界独立检查通过；不能把清理阶段的同clock报错解释为途中停驶。

在原131–134 s窗口，64线/LiDAR30/RGB10的native机身高度增量为+2.594 mm、SLAM为+1.313 mm，离线相对高度误差最大10.866 mm；旧基线同窗口为292.453 mm。新试验整个6.7–209.9 s对齐时段最大误差45.876 mm，没有机身接触/跌倒。这个相同时间窗并非严格相同空间姿态，实际轨迹和指令均保存，不能单靠该窗口归因全部改善。

132 s实际墙面约束847→1123，法向与真实墙面相差>60°的比例26.0%→9.8%，仍有110条坏约束。历史墙面二维支撑mid/max中位比0.182→0.552。增加采样改善了这一轮几何支撑，但没有修正原来的最小特征值判据和成熟平面冻结机制，不能宣称根因已从算法中消除。

RGB30组传感器继续产生数据到209.98/210.01 s，前端原始IMU只消费到196.07 s，LIO/VIO处理到196.09/196.055 s，约14 s队列落后。控制器最后接受位姿52.475 s，在ROS52.770 s时已达295 ms；随后超300 ms位姿被拒绝，61.285 s按原连续8 s失联规则停车。进程继续处理到清理且正常退出，不能把“处理完旧帧”当实时30 Hz有效反馈。具体同步警告与LIO/VIO/日志成本的分解见本轮只读审计。

## 同计算时钟失效

这里的`native guard`名称指原 `control_core` 点云避障几何函数，不是执行器物理时钟。V8第一组190.27 s策略socket往返耗时105.544 ms，同帧Teacher CPU推理仅0.308 ms。native physics_step和policy_exchange各自header严格递增；controller墙钟ticker在短锁步停顿内多次读到190.27 s计算时钟。

同时实际新云190.2 s合法到达，旧guard使用190.1 s云，因此必须重新检查新云。原`ns<=previous`却提前失败。不能简单忽略所有相同计算时刻的新云，不能放宽300 ms保护，也不能把这次停顿归因于GPU推理不足。调度、序列化、写盘与IPC余耗尚未完全分解。

另立V9保守修复：短同计算时刻发布精确零速度，冻结PI积分、到达/清障计时，不复用旧云执行非零动作；实际时钟推进后完整重验新云。真时钟倒退、源时间/墙龄双300 ms、倾斜、连续失联8 s和唯一执行器保护保留。该修复属于控制器时钟处理变化，必须单列实际新run，不能提升V8原失败结论。

## 20 Hz、多核和纯诊断减负

用户随后要求实际检验20 Hz及不改算法的多核加速。核心已有Release `-O3 -funroll-loops -fopenmp`；单纯再加O3没有意义。当前20逻辑核整体load p95约9.9、GPU p95约11%，不能将单线程前端吞吐不足解释为整机所有硬件已耗尽。

V9 `20261005_195524_closed_loop_cascade_clock_hold_lidar64_20hz_rgb20_r1_2a1a` 已实际完成210 s，25/32区域、物理期无failed。真实输入、SLAM、LIO阶段约20 Hz，VIO约19.98 Hz；接受SLAM4128条到210 s、无>300 ms header间隙，全有效期相对高度误差max71.26 mm。共同收据只因未完成全部32区域失败，完整双坡与终点停车未验证。新的事前冻结hold合同单列数学时序回放：3194次PI更新最大误差1.69e-15、含零命令的slew最大误差1.73e-14，全部新增保护检查通过。19次hold中15次短duplicate，4次stalled均在清理阶段。第一次reader错用cloud callback进入时间而非真实接受时间，其源码和错误收据保留，修正读取既有`accepted_cloud_received_monotonic_wall`后另追加新收据，没有修改物理数据或数值门槛。

V10只在独立CMake对`voxel_map.cpp`启用既有`MP_EN;MP_PROC_NUM=2`，全部C++原算法字节保持。残差按索引独立计算、原mutex保护位写、按原索引串行收集；VIO浮点误差归约保持串行。729点合成fixture四份state25/cov361逐字节一致，GOMP观察实际team=2；这不是完整真实地图重放。实际 `20261005_200025_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lio2_r1_70fc` 运行210 s，4/32、82.26 s出现物理期failed，不能认定2线程已使30 Hz可用。实际加载新库SHA `5c45dc9d9dc98214bd8e8a85397af468803b19937dd8f24ac5c42e8c02a9902a`，详细因果/吞吐收据单列。

同一墙钟窗口内，旧30 Hz串行组20–40 s输入30.03 Hz而完整循环29.70 Hz，图像到LIO开始等待中位232.7 ms；115–165 s输入29.60而循环24.00 Hz，等待中位9.287 s。大详单窗口会放大积压，但首次失联61.285 s早于115 s，不能把日志认定为唯一原因。[同步/性能审计](../test_results/lidar_density_rate_20261005/sync_audit/README.md)与[两线程构建/数值证据](../test_results/lidar_density_rate_20261005/lio_multicore/README.md)保存实际编译与来源。

V11另立纯诊断减负：不在详单窗口外构造无人需要的逐点查询/矩阵诊断，不伪造不存在的kind100统计；全时段原IMU/传感器header、LIO/VIO最终state/cov、导航/SCAN/原始guard云、物理与执行日志保留。算法公式、输入、点筛选、权重、PD/PI与保护不变，详细采集实际限定115–118 s。原V9/V10的“detail3s”profile尚未实跑，发现旧runner实际环境仍硬编码115–165 s；不能把它们声明为已实现的3 s A/B。V11将声明与实际环境统一，并对raw kind4以其修正sourceheader选择窗口。新编译及运动结果必须独立追加。

现有失败日志已占用大量磁盘，后续完整路线使用有界诊断版本保留必要验收来源，不删除或压缩改变历史绑定文件。完整32区域的最终新结果在下文追加，前缀25/32不提前称完整导航通过。

## 增加线程的有限计算基准

在实际运动全部退出后，对同一 V11 安装库、24,843 点三平面合成地图，分别测 LIO 残差 team 1/2/4/8；只在测试进程包装该 OpenMP region，实际部署不使用包装库。P核组残差耗时中位为4.99/6.92/7.77/8.16 ms，E核组为5.20/5.17/5.85/8.64 ms，当前实现4/8线程没有加快。每种实际team/TID/affinity都记录；不能以这个合成地图代表完整真实匹配复杂度，也没有无绑核性能对照来证明绑核收益。

原因候选已定位到代码：`BuildResidualListOMP` 的 `vector<bool>` 是压缩位容器，不同索引写入仍可能共享存储，因此原代码用全局mutex串行保护每个成功/失败flag及成功残差赋值。增加线程会增加锁竞争。V12随后将flag换为逐点独立字节，保留原逐点浮点计算、只读地图查询和按原索引串行收集；构建、有限数值与真实前缀验证已完成，见后文。

P核同组全state/cov与有序残差字节一致，E核同组也一致；P/E之间37个state/cov double有最大2.22e-18差，有序残差字节相同，不能宣称跨核类型逐位一致。原fixture外部map计数为0是by-value复制后的元数据误报，不是没有地图，内部真实匹配接受24,843点，修正及原结果均保存。

## 计算复用、流水线与预积分边界

用户进一步要求类似预积分的预计算和并行优化。当前进程已分开，CPU调度尚共享；独立分核可以检验本任务内部抢占，不能隔离其他系统任务、共享内存带宽或GPU，也不能等同搬到另一台机器。只操作本轮自有进程的继承affinity，不设置全机隔离、不改训练。

源码当前已保存IMUpose供去畸变复用，LIO每帧已缓存body covariance/cross matrix，VIO已有inverse-compositional相关缓存，不能把这些已存在机制包装为新优化。可进一步验证同一IMU区间、完全相同dt的去畸变刚体变换缓存；保留每点原表达式与真实rolling不同dt重算。这是相同时间变换复用；V13仅准备源码，尚未构建/验证，也不是Forster预积分。

FAST-LIVO2在同一ESIKF状态/协方差上顺序更新LiDAR与视觉，不能直接把两个更新同时写同一状态；地图查询可在固定只读地图上分点并行，但地图写入/删除与查询重叠需要生命周期与快照设计，不能只套线程。论文明确其顺序更新与统一体素地图：[FAST-LIVO2原论文](https://arxiv.org/abs/2408.14035)。局部相对预积分Delta在同测量区间、同bias线性化条件下可与绝对起始p/R/v分离复用；改变绝对起始pose不必丢弃局部Delta，但映射到绝对状态、gravity及cov应用必须重新处理。这与当前依赖起始state的绝对IMUpose缓存不同。Forster预积分将关键帧间高频IMU压缩成相对运动约束并处理偏置校正，主要用于因子图重复优化；引入当前ESIKF不是线程参数开关，也不能消除点云匹配开销：[预积分原论文](https://arxiv.org/abs/1512.02363)。

同作者资源受限版本采用退化感知的视觉帧选择与紧凑局部地图，是更贴近现有框架的算法级参考，但它改变视觉测量使用和地图管理，需要独立精度/导航实验，其数据集收益不能直接套到本场景：[FAST-LIVO2资源受限版本](https://arxiv.org/pdf/2501.13876)。本轮先验证数值等价的计算复用和并行改动，当前不会直接用降低视觉约束掩盖此前错误墙面法向问题。

## V12 实际30 Hz与计算消耗

已实际运行 `20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d`（外部真实目录见下文）。冻结core SHA `8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed`；V12不改变匹配公式、输入筛选、浮点表达式及原索引归约，LIO独立4线程、VIO显式循环串行。18个LIO/VIO有限过程与3个计时开关对照数值一致，41个控制/保护测试通过。完整实际结果另核：210s/10501推理无fault、25/32区域、物理期running。真实输入6364帧、30.304Hz，LIO6254/VIO6253约30.303Hz，post-anchor SLAM源年龄最大60ms、无>300ms来源间隙或首个guard失败。数学controller3544次、有效17.435Hz；20Hz墙钟ticker不能混称20Hz有效数学反馈。

独立v2联合验收26项23通过、1失败（未到32区域）、2未验证（终点5s停车、完整非平地）。七个追加账本/数学/保护检查全部通过；4301实际发布等于writer expected及原commands counter，4041个PID关联、260个PID之外（含15次hold），不再用接收时刻猜生产发布顺序。正式坡道判据中，下层第一条12m ramp_12 已完整通过：COM出入口、四足top support、全12m bins、landing及实际SLAM正确楼层区域；第二条尚在行进。整体不是完整多层通过。[独立前缀与计时报告](../test_results/lidar_density_rate_20261005/evaluation/V12_PREFIX_AND_MEASURED_PIPELINE_COST.md)。

离线与仿真真值比较的tube max0.161572m/RMS0.039867m、drive heading max0.166472rad、speed MAE0.038978m/s均满足原门。全有效期相对高度误差max90.75mm；131–134s窗43.92mm，nativeΔz−3.5mm、SLAM+6.4mm，无旧20cm突降。真值仅用于离线这些验收，不参与导航。

kind300有214行、invalid0，均为只读计时。完整边界累计wall/caller-CPU：StateEstimation50.71/49.05s，其中residual query29.33/27.71s；VIO14.07/14.05s；UpdateMap3.35/3.34s；同步publication/publish调用9.07/9.05s；spin_some32.07/23.01s（含callback/preprocess）。StateEstimation包含query，不能相加。process-CPU统计包含并发OpenMP/DDS/writer，也不是该函数独占CPU。publication含point transform/彩色投影/转换以及同步publish/RMW调用；调用内部可能含序列化、锁或同步发送，返回后的DDS异步活动没有单独归因，不能把它当纯通信。spin_some约9.06s wall与caller-CPU之差仍混有等待/锁/调度，不能据此指定网卡或共享内存传输时间。

/proc物理采样从24.78s起，早段CPU N/A；186wall s记录FASTLIVO155.71CPU s≈0.837核，main121.34s≈0.652核，三个较重worker合22.82s（仅依据TID/comm，不冒充已证明OMP角色）。main非自愿切换17,897次，提示分核有检验价值。当前主要可见工作集中于LIO匹配/状态估计、VIO与callback数据处理；IMU传播/去畸变占比小，不能承诺单独换预积分解决吞吐。[计算图、缓存失效与并行边界](../test_results/lidar_density_rate_20261005/thread_scaling/COMPUTE_DEPENDENCY_AUDIT.md)包含源码依赖、矩阵inverse候选及buffer复用风险。

## 独立分核和外部证据存储

V14显式复用同一冻结V12 core/lib，分配Gazebo P0–3、SLAM P4–7、Actor E18、controller E19，其余本任务bridge/capture/SCAN E8–17。这仅让本任务的进程间CPU mask分开，不对全机CPU、GPU、内存或磁盘作独占保证，也不修改其他任务。5个新建测试子进程的mask已验证，实际6eb0的210s已完成、25/32、无fault。独立104条实际采样、25,219个线程样本证明5组mask符合声明；v2全部7个附加检查通过，共同门仅全32区域失败、终点停车/完整非平地未验证。SLAM6252条约30.294Hz，最大header gap65ms，post-anchor source age p50/p95/max为30/40/65ms，无>300ms缺口。数学controller有效17.640Hz。第一条12m坡道正式COM/四足/landing/SLAM区域门全通过，第二条未完成。

共同sim60–110、130–170、170–210s窗口的主线程非自愿切换率，V12→V14依次为120.79→174.24、56.18→138.71、120.50→154.92次/s；StateEst mean wall分别10.512→11.615、6.713→7.638、9.361→10.765ms，query亦未改善。进程平均CPU核占用小幅下降不能等同加速。两次实际位置、迭代数不同且只有单次，不能严格归因绑核变慢，但当前没有支持绑核收益的证据。完整600s验证采用V12原调度，保留V14。详见[实际分核对照](../test_results/lidar_density_rate_20261005/evaluation/V12_V14_AFFINITY_COMPARISON.md)。

新增日志的真实目录从创建起固定为 `/var/tmp/go2_teacher_simulation_20261005/`（root盘空间充足、当前UID、0700、无root symlink）。项目 `runs/<run_name>` 是同名alias，所有源快照/PointCloud数组/收据均在真实run内部按原规则验证。home仅约26GiB空闲，而30Hz210s的导航云原始归档约12GiB；不删除、搬迁或改写任何历史日志。外部payload并不嵌入项目目录，最终另生成外部全文件SHA manifest并由项目PACKAGE_MANIFEST记录该清单与alias；复制包时也要带上外部payload。[外部库存脚本](../test_results/lidar_density_rate_20261005/external_storage_inventory.py)必须在所有writer结束后执行。

V13同dt去畸变变换缓存仅源码准备、尚未构建/数值/实际验证，prepare强制拒绝。它是有限相同dt刚体变换复用，不能称已引入Forster预积分。当前不改变ESIKF顺序校正、不同时写同一状态或可变地图，也没有实现完整IMU线程流水线。

## 完整32区域实际运行与线程CPU

V12原调度 `20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb` 使用相同冻结core/profile和原32目标/数值门，kind300 off。600s为上限，实际native到275.185s、worker275.18s/13,760推理，无fault/机身接触；32/32真实SLAM区域succeeded，终点保持自动完成后runner退出0。Teacher CPU p50/p95/max0.3103/0.3683/1.5856ms。SLAM源8,138 post-anchor样本年龄max60ms，无guard物理失败；相对高度误差全程max108.35mm/终点−4.72mm。131–134s nativeΔz+7.61mm/SLAM+11.57mm，无旧突降；时间窗并非严格相同空间状态，不作单因果比较。

原common/v2已独立确认32/32、SCAN/来源、运动/速度/安全/PI发布链、前declared active-hold固定268.145–273.145s五秒通过。native漂移5.38mm/偏航0.000596rad。生产诊断198,244/198,244、drop0；5,626条实际publication等于expected和原counter。原common与v2仍有严格非平地UNVERIFIED；原ramp追加收据中两条12m坡道所有物理门与终点落地支持通过，但provider状态因果关联UNVERIFIED。源码比对证实history比status多monotonic_wall/ros_sim_time两个归档字段，原reader对整个JSON比较导致关联拒绝。新增只读metadata v2审计已通过：精确核对冻结producer、完整74字段payload唯一匹配、因果时间、原187射线双provider重放、必需19/26/25项与7项附加门、48份来源字节。9组负例通过。旧三份收据与v1追加收据原字节均保留；新v2是明确的事后读取器修正，没有修改路线、来源、数值门、停车或TTL。

新v2收据SHA `b07b0a4b4b5b80cbd1a48201e745d0241596fdba8c56b93d1cb6da5f85919d43`，支持本次单轮32区域、两条完整12m上行坡道及首固定5s停车的联合通过结论。不是新物理复测或全场景/随机鲁棒性证明。完整验收、轨迹/速度/CPU和原收据关系见[完整路线报告](../test_results/lidar_density_rate_20261005/evaluation/FULL_REPORT.md)。

CPU采样从runner startup启动，143条、自身2.462CPU s/最大一次18.4ms。FASTLIVO实际出现后280wall s记录224.76CPU s≈0.803核，其中用户态218.37s/内核6.39s；main175.81s≈0.628核。同一物理采样范围内，SCAN真实PID387057有184.36CPU/280wall≈0.658核；runtime自有Gazebo PID387147（comm ruby）172.89≈0.617核；Teacher worker PID386994有93.79CPU/282wall≈0.333核（包含观测/IO，不能当策略forward CPU）；bridge/capture约0.159/0.122核。未记录实际argv的其他python3不凭名称归属通信或具体功能。实际名称为dds.*的线程合5.57CPU s（shared-memory4.69、event0.85、UDP0.03）；命名是线程观察而非完整通信归因，主线程仍可做序列化/RMW，同步等待不在CPU量中，因此不能用5.57/224.76宣称通信只占2.5%。kind300此轮明确N/A，不将未记录段写0。独立summary按每个PID/TID的实际首次与最后物理样本累计，后启动线程及未知早段明示。外层sampler工具exit143而completion完整，原因未验证，另存external_cpu_sampler_tool_outcome.json；不改写为clean0或当成物理失败。

## CPU / GPU

同一冻结247→512→256→128→12 ELU策略，真实V7观测抽取1000条、FP32、batch=1、CPU/interop各一线程，分别测CPU和CUDA。GPU端到端包括观测H2D、forward、12动作D2H及同步。

|50 Hz实际调用节奏，150次/设备|p50 ms|p95 ms|p99 ms|
|---|---:|---:|---:|
|CPU|0.3358|0.5239|0.6430|
|CUDA|0.6443|0.9568|1.2602|

两者均无20 ms超期，最大动作差7.15e-7，CPU重放与原动作逐位一致。仅GPU Event的0.0312 ms不含完整主机链，不能据此声称GPU更快。当前保留CPU；切换Teacher设备不会消除错误平面约束或FAST-LIVO旧数据队列。基准未与64线渲染或训练饱和并发，其他负载下尾延迟未验证。完整基准、4300条计时、资源、模型/输入/脚本哈希见 [GPU实测](../test_results/lidar_density_rate_20261005/gpu_benchmark/README.md)。

## 证据和复现

- [采样与独立启动说明](../navigation/lidar_sampling_v8/README.md)。运行会创建新run，既有收据拒绝覆盖。
- [原高度失效归因](SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)。
- `test_results/lidar_density_rate_20261005/evaluation/`：只读分析器、真实header频率、native/SLAM高度轨迹、云/平面几何及正式收据执行记录。
- 每个新run包含 `sensor_sampling_contract.json`、改动前/后的世界SDF、配置/模型/源码scope哈希、原SCAN数组、双相机、指令/速度/关节目标/力矩/接触及二进制诊断。f728诊断162547条、14.9 GB；cb5c259333条、46.5 GB，均0drop、0writer错误。
- [第一组实际Gazebo与SLAM/SCAN](http://127.0.0.1:8768/?run=20261005_192400_closed_loop_cascade_lidar64_30hz_rgb10_r1_f728)、[RGB30失联组](http://127.0.0.1:8768/?run=20261005_192802_closed_loop_cascade_lidar64_30hz_rgb30_r1_cb5c)。

外部全文件库存已封存：65,469个文件、55,004,007,938 bytes，清单[EXTERNAL_STORAGE_MANIFEST.json](../test_results/lidar_density_rate_20261005/EXTERNAL_STORAGE_MANIFEST.json) SHA `9c5847719284843d55919b79f1933012837f2bb6c5aa27125400ac201df23af7`。各run指向创建时的真实目录，复制研究包必须同时保存外部payload；未删除或重写历史失败。

针对用户引用的“解释SLAM前端上限”，新增[队列/工程流水线审计](../test_results/lidar_density_rate_20261005/pipeline_audit/QUEUE_ENGINEERING_AUDIT.md)和[算法并行审计](../test_results/lidar_density_rate_20261005/pipeline_audit/ALGORITHM_PARALLEL_AUDIT.md)。优先独立接收与图像/点云解码、单同步assembler、单估计器、不可变输出队列；同轮LIO Jacobian按行与VIO patch按索引并行后原顺序汇总。分块正规方程会改变浮点累加次序，IMU区间Delta/FQ前缀扫描涉及偏置/协方差/校正epoch，需要独立验证。上述新增方案未构建或实际运行；已部署的V12仅点残差并行与相关锁移除。

基础接口、既有有限运动和本次指定32区域静态多层路线通过；原46区域全任务、当前版动态障碍、完整Sim2Sim新律对照及真机未验证，历史失败保留。Actor仍232维特权状态/高度+15维已知命令/上一动作，真实30维替换尚未与本路线组合。三层由坡道连接，未验证真实楼梯。
