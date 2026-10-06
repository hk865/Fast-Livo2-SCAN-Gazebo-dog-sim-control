# Frozen Go2 Teacher、FAST-LIVO2 与 SCAN 仿真实验报告

报告日期：2026-10-06。实验发生于2026-10-03至05；全部限本地仿真，没有操作实体机器人、重新训练Teacher、停止其他任务训练，或覆盖已通过的相机Demo。

**当前已实现冻结Teacher的Gazebo执行与有限运动闭环，并以实际SLAM/IMU/点云和SCAN完成两轮指定静态32区域三层坡道路线。严格跨引擎Sim2Sim、全传感器Actor和真机部署仍未通过或未验证。** 本仓库发布源码、配置、验收器和保留摘要；原始实验数据按用户授权清理后，本文是当时独立验收的历史记录，不能仅凭摘要或SHA重新独立审核原实验。见[证据保留边界](EVIDENCE_RETENTION.md)、[精简证据索引](../evidence/INDEX.md)与[最终证据库存](../evidence/EVIDENCE_MANIFEST.json)。

## 2026-10-06追加：V19流水线与原46区域

上文及下方1–6节是清理前历史报告，不回填旧通过与失败。清理后新增V19多线程流水线、100次有限数学检查及实际ROS正常排空验证；同包1x四轮和1.5x单对照均已完成，有限容量探测表明积压下降。之后完成60秒实际前缀，并尝试原46任务：1次启动失败、2次实际运动均8/46失败。最终旧朝向参考冲突已修复，剩余SCAN微小起始段切线104次触发转向门、第9区90秒超时，不能记原46通过。

[追加最终报告与所有边界](../multifloor_demo/teacher_mode/test_results/pipeline_v19_20261006/README.md)、[故障报告](../multifloor_demo/teacher_mode/test_results/pipeline_v19_20261006/evaluation/V19_9779_FAILURE_REPORT.md)。新raw仍在原工作区；本仓库精简V19清单另列，不属于旧EVIDENCE_MANIFEST或旧351.5 GiB删除范围。静态32区域历史通过保留，严格Sim2Sim、全感知Actor、当前完整46动态任务及真机均没有获得新通过。

## 1. 模型、控制链与来源

| 项目 | 本次实际契约 |
|---|---|
| Teacher | 冻结`model_1000.pt`，SHA256 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34` |
| 网络 | `247→512→256→128→12`、ELU、无观测归一化；兼容checkpoint的`actor_state_dict/mlp.*` |
| 观测 | body系COM线速度、角速度、重力、命令、关节相对位置/速度、已限幅施力命令、上一raw action、187点高度扫描，按训练归档裁剪/缩放/拼接 |
| 动作 | `q_target=q_default+0.25×action`；默认姿态及名称映射取本次训练归档 |
| 关节映射 | 训练FR/FL/RR/RL的hip/thigh/calf，显式映射Demo RF/LF/RH/LH的hip/upper/lower，不依赖数组碰巧同序 |
| 执行 | Teacher CPU单线程50Hz；物理步长5ms、关节PD200Hz，stiffness25/damping0.5，DCMotor速度相关限矩23.5Nm、名义速度限30rad/s |
| 安全 | 唯一`TeacherActuator`写关节施力；双时钟300ms来源保护，初始化、无效输出、重复/倒退时钟、受控停车与故障锁存分别记录 |
| 停车 | 持续Teacher速度控制或固定终点主动hold；没有把action=0、冻结关节目标当作停车 |

Actor输入仍有**232/247维仿真特权状态与高度扫描**；另15维是速度命令3和上一动作12。高层导航只使用实际传感器SLAM/IMU/点云、SCAN路径与实际SLAM区域到达。Gazebo真值用于Actor特权观测和独立离线运动/几何验收，未用于导航定位、目标生成或到达判定。施力字段是送入仿真物理的限幅命令，不是实体电机力矩测量。

训练原始噪声/随机化范围已核查归档，但多数正式测试使用固定名义初态、种子和无噪声；确定性重复不代表随机扰动鲁棒性。详细源码契约见[policy/contract.json](../multifloor_demo/teacher_mode/policy/contract.json)、[适配审计](../multifloor_demo/teacher_mode/docs/ADAPTATION_AUDIT.md)。旧CHAMP相机模式保留，没有完成与Teacher同路线同条件的直接胜负比较。

## 2. 运动与控制器标定

| 阶段 | 当时结果 | 实际范围及限制 |
|---|---|---|
| 首轮接口/基本运动 | 42轮正式基准接口通过；运动原协议39/42 | 站立、前后/左右移动、正负转向、命令切换、行走停车、超时、短坡；旧5cm项仅高度比例门失败 |
| 5/10cm功能复测 | 两高度各3/3主测通过 | 四脚顶面实际支持、继续前行与停车；各继续2.907/3.086m。台面延长为6m新fixture，非完整障碍跨越或楼梯 |
| 原开环完整坡道 | 失败 | ramp23上/下行仅推进4.820/4.807m，横漂约0.71m、离支撑面高度低于安全门；另ramp12推进8.006m后附体碰坡 |
| 真值串级标定 | 27次比较、35次留出运动通过 | 直线、折线、往返、0.5/0.7/1.0m/s、完整两坡上下行、5/10cm；仅真值反馈标定 |
| 转弯包络 | 36/36通过 | 输入vx0.3/0.5/1.0m/s、正负wz至0.8rad/s；是最小已测合格半径，不是物理极限 |
| 圆圈/S曲线 | 行驶14/14；含停车13/14 | tight S25零命令停车yaw漂移0.114522rad超原0.1rad门，失败保留 |
| tight S主动停车 | 先导+3次确认4/4通过 | 固定5s窗XY漂移4.45mm；旧弱捕获180s未进入hold及旧归档缺失未验证保留 |
| 实际SLAM平地迁移 | 6/6、132/132门通过 | 6m单程及6m+6m往返各3次，0.3m/s；body-relative固定路线，不是旧全局地图注册 |

旧5cm高度增量为26.0mm，小于当时30mm门，但四脚已经登台。后来功能协议以实际足部支持和持续运动验收，不能把旧数字失败解释为“登不上”。10cm功能确实通过。原旧判据与新fixture分开记录，未回改旧失败。[台阶报告](../multifloor_demo/teacher_mode/docs/STEP_RETEST_REPORT.md)、[原完整坡道失败](../multifloor_demo/teacher_mode/docs/FULL_RAMP_REPORT.md)。

所选真值控制器为路径/朝向PD外环和COM速度PI内环：横向PD=0.8/0.2、朝向PD=1.3/0.15；vx PI=0.6/0.5、vy PI=0.4/0.3、wz PI=0.3/0.2，外/内滤波0.2/0.1s。横向误差取固定有向空间路径的带符号法向投影，朝向误差取路径切线与yaw之差；沿程剩余距离减速、终点距离判断到达，没有按时间追逐一个理论位置点。原点速度与COM速度用实际杆臂和`ω×r`转换。

比较10/25/50Hz后，25Hz在有限矩阵中综合较好；0.3m/s参考的串级实际速度约0.296m/s，旧位置控制约0.24m/s。这不是全球最优，也没有证明PID必须比Teacher50Hz更快。实际10Hz SLAM迁移只在新header上积分，以源dt更新；30Hz路线部署墙钟ticker20Hz，实际数学更新约18Hz，不能混称50Hz模型或30HzSLAM就是控制环频率。[真值标定](../multifloor_demo/teacher_mode/docs/TRUTH_PID_CALIBRATION_REPORT_20261005.md)、[曲率包络](../multifloor_demo/teacher_mode/docs/CURVATURE_ENVELOPE_REPORT_20261005.md)、[主动停车](../multifloor_demo/teacher_mode/docs/ACTIVE_PARKING_REPORT_20261005.md)、[真实SLAM迁移](../multifloor_demo/teacher_mode/docs/SLAM_CONTROLLER_TRANSFER_REPORT_20261005.md)。

## 3. SLAM高度失效与采样提频

原V7诊断在133.13s触发高度误差303.483mm保护，只到16/32区域。131–134s内，真实机身Z仅变化−3.668mm、最大roll/pitch约0.92°/1.79°、机身接触和执行fault均0；发布SLAM Z却下降330.971mm。故该次“突然降低20cm”主要是估计高度失效，不能据画面认定狗实际下沉或剧烈倾覆。

实际源码与约束重建支持故障链：远墙早期单扫描环形成近线簇，平面最小特征值门接受不可靠近水平法向；模型达到点数上限后停止更新，近墙新竖墙点仍使用旧法向。LIO向下修正速度，经位置—速度协方差耦合和已有负vz积分持续降低高度，VIO约束变弱但仍向上修正。窗口内部状态ΔZ为已有vz积分−323.481mm、当步加速度项−0.0247mm、LIO−79.444mm、VIO+66.405mm。这不排除更早初始化/bias等影响，也不是“所有IMU无问题”的证明。低负载可复现，GPU竞争不是必要条件。[高度失效诊断](../multifloor_demo/teacher_mode/docs/SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)。

| 独立配置 | 210s当时结果 | 限定解释 |
|---|---:|---|
| V7：32线、LiDAR10/RGB10 | 16/32，失败 | 上述错误几何导致高度保护 |
| V8：64线、LiDAR30/RGB10 | 22/32，失败 | 高度改善，190.365s重复计算时钟误判；RGB10仍驱动约10Hz LIVO |
| V8：64线、LiDAR30/RGB30 | 3/32，失败 | 算法消费旧帧积压，61.285s连续失联；末端约14s落后 |
| V9：64线、LiDAR20/RGB20 | 25/32，前缀失败 | 约20Hz源跟上，无物理期失败；未完成32区域，不提升为完整通过 |
| V10：30Hz、LIO2线程 | 4/32，失败 | 增线程本身未使整链可用 |
| V12：30Hz、LIO4锁移除+诊断减负 | 25/32，前缀未完成 | 新鲜SLAM约30Hz、第一条完整坡道通过；随后另跑完整路线 |

64线增加真实二维支撑，132s坏墙面约束比例从26.0%降至9.8%，仍有坏约束。没有改掉原平面成熟/冻结算法，不能宣称密度从根本上消除了几何错误。完整记录见[密度、频率与CPU/GPU报告](../multifloor_demo/teacher_mode/docs/LIDAR_DENSITY_RATE_REPORT_20261005.md)。

## 4. 多核、计算复用与流水线实验

V12将残差成功flag从压缩`vector<bool>`改为独立字节槽，去掉逐点全局mutex，保持原浮点公式、只读地图查询和原索引顺序收集；同时缩短详细诊断到115–118s并避免窗外无用payload构造。V15进一步按索引并行构造LIO Jacobian行；V16重组VIO patch独立槽及原序归约。V18独立合并LIO4+VIO1，并重新做729点状态/协方差和41种视觉边界的逐字节数值回归后才运行物理。

V15每场景每变体32独立进程批次，共384；真实约6k行有限kernel中位改善约72%。V16同定义768进程，大patch单线程变体改善24–30%，四线程尾延迟门失败，最终选T1。不能把内部重复当独立批次、把T1收益归多线程、或把算子收益乘成整链收益。历史真实HTH重建存在约3–4e−8差异，精确历史回放失败保留；fresh同输入数值相等不代表完整历史`processFrame`回放。[组件独立报告](../multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/evaluation/COMPONENT_BENCHMARK_REPORT.md)。

实际210s V12→V18对照**未证明整链显著加速**：LIO墙钟均值9.127→9.293ms，调用线程CPU8.833→8.568ms；VIO整帧2.555→2.545ms，两者均跟上约30Hz输入。实际轨迹、地图历史和约束不同，不能作单算子强因果结论，也没有测更高输入率的饱和点。残差query嵌套在StateEstimation中，不能相加；wall−threadCPU不是纯通信，进程CPU含RX/DDS/logger等并发线程。固定P/E分核V14也未证明收益。现有IMUpose、协方差和inverse-compositional缓存已存在；pose/bias/map/时间锚变化有不同失效条件，不能把SO3/SE3矩阵套用通用“预积分加速”。[实际整链对照](../multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/performance_actual/REPORT.md)、[计算依赖审计](../multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/thread_scaling/COMPUTE_DEPENDENCY_AUDIT.md)。

Teacher小网络batch1实测50Hz节奏端到端CPU p50/p95=0.3358/0.5239ms，CUDA=0.6443/0.9568ms（含传输和同步）。GPU Event的约0.031ms不是完整主机闭环；本次继续CPU，没有通过GPU推理解决前端队列或错误几何。

V17后台单RX接收/解码，经512项/64MiB有界FIFO交给唯一状态线程，60s实际smoke严格失败：accepted31615、delivered＝committed31614、canceled1、rejected0。已提交FIFO、原整数header/receipt、不同RX与owner、因果、容量和writer12项通过；原16项前缀运动/来源/安全及7项发布保护另行通过，但不抵消零取消门。缺失事件未记录类型或准确时间，不能猜为timer并作退出豁免。运行期LIO/VIO约30Hz，LiDAR/RGB/IMU接收→提交p95=0.738/4.109/15.585ms；这是60s测量，不是匹配A/B加速证明。V17没有合入默认路线候选。[严格队列审计](../multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/pipeline_runtime_audit/actual_efde/README.md)、[smoke原保护覆盖](../multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/evaluation/actual_v17_smoke/SMOKE_REPORT.md)。

## 5. 两次完整静态三层导航

两轮600s均为事前上限，实际在目标和停车完成后提前自然结束。参考均速约0.19m/s；不是1m/s三层导航。原路线32个三维区域、两条各12m上行坡道、四足有序支持、各1m分箱、落平台和首个声明的固定5s主动hold均按原门验收。三层由坡道连接，**没有验证真实楼梯**。

| 指标 | V12完整`fbcb` | V18完整`ab53` |
|---|---:|---:|
| 实际Teacher时长/样本 | 275.18s / 13760 | 264.60s / 13231 |
| 真实SLAM区域 | 32/32 | 32/32 |
| 源header平均频率 | 30.2998Hz | 30.29583Hz |
| 最大源header间隔 | 65ms | 65ms |
| 离线路线最大/RMS距离 | 0.139573/0.038518m | 0.138906/0.037279m |
| 行驶朝向最大误差 | 0.147896rad | 0.166577rad |
| 速度MAE | 0.039399m/s | 0.038278m/s |
| 首固定5s停车XY/yaw漂移 | 5.38mm / 0.000596rad | 2.14mm / 0.004682rad |
| 接触/执行fault | 无机身接触或fault | 无机身接触或fault |

路线误差为一次实际来源锚点的body-relative离线SE3几何验收；绝对旧场景中心线注册仍UNVERIFIED，不能称绝对全局定位精度。V18实际控制行位姿sim龄p50/p95/max=45/65/80ms，receipt墙龄18.30/36.98/136.09ms；控制数学4634次、17.974Hz，独立于20Hz墙钟ticker、50HzActor和200Hz关节PD。300ms双时效、真实来源、SCAN、发布变化率与唯一执行权均保留。[V12完整报告](../multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/evaluation/FULL_REPORT.md)、[V18完整报告](../multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/evaluation/actual_v18_full/FULL_REPORT.md)。

两轮原common/publication/ramp收据均曾为UNVERIFIED：物理门通过，但terrain provider来源关联比较了整条history，而history额外有`monotonic_wall`、`ros_sim_time`两个归档字段。新增只读metadata-v2审计以冻结producer证明剥离仅这两字段，原74字段payload唯一精确匹配、原因果、187ray、全部mandatory19/26/25门和7个发布门及输入字节重新闭合；负例拒绝缺账本、外国run、修改payload、重复关联、非因果和其他失败。新联合结论通过，**原三份收据未改字节、未回填PASS**。它是事后读取器修正，不是新物理复测或门限放宽。

| 历史闭合收据 | SHA256 |
|---|---|
| V12 metadata-v2 | `b07b0a4b4b5b80cbd1a48201e745d0241596fdba8c56b93d1cb6da5f85919d43` |
| V18 metadata-v2 | `1398f53ec33310c510944ba5100d62d67d205c7596250b972a89332bb57e5280` |
| V17严格pipeline | `80efc11ed7c5537c7ef70493f58527db5b8f28ef20cdcfe3bcc2e2dd5df4af20` |

以上SHA是历史字节身份，不包含被删除的原始输入。保留收据不意味着清理后仍能重新计算其中所有门。精简证据按`V12_fbcb`、`V18_prefix_ac02`、`V18_full_ab53`、`V17_efde`、`height_failure_6fdf`分组；0.2s派生曲线仅供可视化，不能代替200Hz安全接触、每次发布或完整ROS原始输入验收，具体保留/抽样范围以evidence索引为准。

## 6. 分层最终结论与剩余工作

| 层级 | 结论 |
|---|---|
| 接口 | 已验配置通过：冻结模型、247/12、映射、时序、PD/DC限幅和唯一执行器；历史渲染/归档/来源失败保留 |
| 运动控制 | 已测有限范围通过，包括六向运动、切换、停车、台阶及闭环完整坡道；旧开环失稳、曲线停车和弱捕获失败保留 |
| Sim2Sim | **严格整体未通过**：平地九组匹配命令子门通过，但即时缓存/观测快照一致性、原起点共同失稳和地形差异未闭合；新控制律未做Isaac同路线对照 |
| 导航集成 | **指定静态32区域双坡道路线限定通过**：V12/V18各一轮；非原46区域全任务，不证明随机鲁棒性或所有路线 |
| 动态障碍 | 旧V4.5a限定平地单轮19/19通过；当前V12/V18三层控制器未联合复测动态停车恢复 |
| Actor真实输入 | IMU/关节30维有限站立及前进停车替换通过；来源过龄失败保留。其余特权输入未全部替换，也未与最新三层路线联合通过 |
| 全局一致性/后端 | 回环/位姿图方案已调研；未完成后端融合或绝对旧地图注册。后端不替代错误前端约束的修复 |
| 真机 | 未验证，无实体操作；不能据仿真参数直接部署 |

旧V12局部报告曾把双坡Gazebo部署运动门称“本冻结配置Sim2Sim通过”；本报告采用严格Gazebo↔Isaac对照含义，保持整体未通过，不把单引擎导航通过升级为跨引擎认证。固定1m/s仅在真值平直线实测，不外推到曲线、多层或SLAM导航。[原严格验证](../multifloor_demo/teacher_mode/docs/VALIDATION_REPORT.md)、[30维替换](../multifloor_demo/teacher_mode/docs/SENSOR_REPLACEMENT_REPORT.md)、[旧平地动态范围](../multifloor_demo/teacher_mode/docs/DYNAMIC_OBSTACLE_REPORT.md)。

两路Gazebo实际相机640×480、D五项为0、R单位阵，内参/原帧时戳已匹配；宽FOV透视与遮挡不能解释为漏去畸变。原相机模式未修改，实体镜头标定未验证。[相机核验](../multifloor_demo/teacher_mode/docs/CAMERA_AUDIT.md)。浏览器曾展示真实归档画面和SLAM/SCAN路线；删除原帧后不再承诺历史录像可播放，不合成替代画面。

后续应在新独立run继续：V17停止接收/在途解码/owner排空的收尾屏障；地图二维支撑与可靠新视角重估；完全真实Actor观测，特别是局部点云与训练最高表面扫描的语义；当前控制器动态障碍；新控制律Isaac对照、重复扰动和其他路线。新源码复现需要按[仓库说明](../README.md)重建依赖、绑定本机路径/实际库/模型/资产并重新冻结来源；构建或模型加载成功不能继承本文PASS。
