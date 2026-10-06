# V19流水线比较与原46区域任务：事前验收协议

日期：2026-10-06。当前状态是**源码审查与协议准备，没有启动V19仿真，没有通过V19性能或46区域任务**。冻结Teacher不重新训练，CPU单推理线程50Hz，物理关节PD200Hz，命令与来源双时钟300ms保护，唯一Teacher执行器；不启动CHAMP/JTC，不操作实体机器人或改变另一任务训练。旧完整raw已经按用户要求删除，V12/V18/V17结论仅作为历史摘要背景，不能对已删除的输入重新验收或声称精确回放。

本协议分开记录：有限解码/顺序数值通过、流水线吞吐改善、运行期新鲜度/安全通过、原46区域完整任务通过。没有必要门缺失或未验证时才能给该层PASS。代码构建、模型加载、60/210秒前缀或32区域成功都不能替代46区域结论。

## 一、原46区域任务到底是什么

权威定义是[原场景](../../../simulation/scenario.json)、[区域生成与校验](../../../mission/route_regions.py)、[区域几何与原始时间戳dwell](../../../navigation/goal_regions.py)及[任务状态机](../../../mission/state_machine.py)。声明采用`three_platform_body_arrival_v2`、`relative_world_axes`，在一次健康初始化的实际SLAM机身原点上冻结一次传感器IMU/SLAM朝向转换。

| 顺序 | 原请求名 | 区域数 | 参考中心折线长 | 阶段义务 |
|---|---|---:|---:|---|
|1|exploration|18|43.1197m|F1初始小回路，再经两条坡道探索到F3|
|2|return_origin|14|39.1197m|两条坡道下行，返回同一初始化原点|
|3|saving_map|无新增区域|—|返回后实际保存本轮RGB地图，不加载参考点云冒充本轮观测|
|4|navigation_f1_f3|14|39.1197m|由F1再到F3，同时完成真实动态障碍停车、清障、重新规划与恢复|

三段合计**46个原区域、121.3591m参考中心折线**。该长度是场景中心折线诊断，不是要求实际狗精确走到每个中心或固定运动时长。三层为0/1.2/2.4m平台、10%坡道，合计两坡各上行两次、下行一次；不能称真实楼梯。

动态障碍**属于完整46任务**。[任务map_saved](../../../mission/state_machine.py)在保存地图成功后发布enable并进入第三段；[原触发器](../../../simulation/obstacle_trigger.py)要求当前匹配的第三段请求`running`、`waypoint_index==1`，实际SLAM已完成北向第一点，才在一次冻结的场景坐标`[1,2,0]`附近触发。[实际障碍节点](../../../simulation/obstacle_controller.py)通过Gazebo实体服务移动`moving_obstacle`，运动进入、阻挡20s、离开；原场景箱体0.5×0.5×1.2m。不得仅发布状态、停住静态箱或关闭动态义务后称原完整任务通过。允许另跑静态46诊断，但必须标`STATIC_46_DIAGNOSTIC`，不升级原完整任务。

原46区域的控制/验收几何必须逐字段与原scenario相等，不可用32的圆柱覆盖：

| 区域 | 实际SLAM内侧控制范围 | 离线真值外侧验收范围 |
|---|---|---|
|平层|水平半径0.25m，高度±0.10m|半径0.35m，高度±0.10m|
|坡道及接缝|沿坡/横向/法向±0.25/0.20/0.07m|±0.35/0.30/0.10m，原坡向正交坐标轴|
|返航最终原点|0.17m球域|0.22m球域|

每点原dwell是0.4仿真秒，原观测最大间隔0.2s、每目标期限90仿真秒。三段的18/14/14个`goal_id`、原顺序、中心、区域、axes、control_band、dwell和timeout全部在执行前冻结。独立验收使用实际NAV回执的同一个原始SLAM时间窗，窗内所有raw SLAM在内侧、同时间离线native在外侧；初始单次SE3只用于离线真值比较，不进入导航。禁止后验挑另一个窗口、插值捏造足够dwell、跳点、重排、重置地图/原点或每段重新对齐漂移。

## 二、为什么不能直接把32改成46

[V18路线生成](../../navigation/combined_compute_v18/route.py)从`route_world_points`创建一个单请求，32个1m分段点、起点`[1.2,2,0.4,0]`，只走lower12上坡、连接廊、upper23上坡；终点`[1.2,7,2.4]`。其区域是统一disc_prism外半径0.22/内0.17m、高度±0.10m、dwell0.6s。这是不同的有限双上坡任务，未探索小回路、返航、保存地图或执行第三段动态障碍。32门的成功不可补成46的通过。

需要新增的独立Teacher适配工作，不是修改原相机Demo或原46几何：

1. 接入原Mission三次不同request_id及全部完成证据、保存地图服务/结果、全阶段有界终止；一次初始anchor跨三段保持。新请求要正确丢弃前阶段final_bundle、路径、积分和停车状态，且不把第一段succeeded当全任务结束。
2. 接入动态障碍真实mover、实际pose observer和原trigger。V18的`stack.launch.py`中`DYNAMIC_EXPERIMENTS=set()`，当前没有原完整任务协调器，不能仅开一个profile旗标假称已接入。Teacher世界名/服务要按新world实际绑定；不能直接调用旧`/world/multifloor_demo`去操纵另一个仿真。
3. 当前[Actor地形provider](../../navigation/combined_compute_v18/terrain_provider.py)是`connector_mid`触发的**一次lower12→upper23**切换，显式拒绝其他goal_id，不能支撑原任务的upper23→lower12返航及再次上行。新版本必须按实际SLAM完成区域驱动阶段切换，绑定当前request/goal-definition/arrival原SHA；每次保留原187射线、双方在共同landing上的相等验证、因果时钟/来源及原独立全静态安全地图。建议审查原`exploration:11`、`return_origin:5`、`navigation_f1_f3:7`在F2廊道的候选切换，**它们不是已验证触发器**；必须先按真实几何与187ray证明可用。native可检验Actor射线相等，不能用native位姿自动选导航楼层。
4. 原初始spawn为`[0,0,0.3,0]`；Teacher此前在`[0,0,0.4,0]`零命令实测2.02s失稳，102/187条全静态扫描先击中F3楼板。旧[验证报告](../../docs/VALIDATION_REPORT.md)保留该失败。新46必须先测原点健康初始化/扫描provider；不得搬到成功的`[1.2,2]`起点却继续称原路线原场景等价，也不能把ray输入场景过滤修改隐藏成训练扫描无差异。
5. 原[完整启动器](../../../scripts/stack.launch.py)启动CHAMP/JTC、对应PID参数和gait运行库；原[evaluate_run.py](../../../scripts/evaluate_run.py)还包含这些执行器来源门。**不允许直接运行旧完整栈**或伪造CHAMP库通过。新Teacher启动器可复用纯Mission/区域/地图/传感器/动态验收算法，并用冻结Teacher唯一执行器、实际加载库、PD/DC和所有publication证据替换执行器身份项。所有替换及新schema必须事前逐项列明。

## 三、V19比较阶段：吞吐、等待与计算分别验

基准背景为V18 LIO4/VIO1、64×480 LiDAR30Hz、RGB30Hz，Actor CPU1、原控制ticker20Hz与按SLAM新header更新数学。V17严格全生命周期`canceled=1`失败保留；V19首先需要修复和证明停止接收/在途解码/owner排空顺序。旧raw删除后不可用旧指标冒充这次新A/B输入。

### A. 无物理的语义与同输入性能

先用新事前冻结的有限输入fixture检查真实生产解码及commit边界：PointCloud2字段/过滤/空云、Image encoding/step/空数据、IMU整数header、重复/倒退/太近时间戳拒绝、HILTI非法配置、有效及被拒header伴随解码异常。比较V18原同步入口和V19入口的**同输入**decoded浮点字节、原接受/拒绝/异常位置以及全部顺序与owner状态；这些只能证明有限组件，不能称完整processFrame/地图轨迹重放。

用同一个V19编译库的pipeline off/on作性能A/B，固定算法公式、OMP LIO4/VIO1、编译FP选项、同P核类型、总可用CPU mask、输入数据/全局序号、原逻辑时间与诊断配置；另用fresh V18作为有限数值参考。所有ROS可变状态、地图、sync/IMU/LIO/VIO与timer commit保留单owner，后台仅接收或解码；多decoder完成顺序不能改变原全局commit顺序。源receipt在接收入口立即记录，worker完成不得刷新源TTL。

至少每数据规模每变体32个独立进程批次，交替ABBA/BAAB，12次预热，每批至少30内部有效样本；内部样本不能冒充独立批次。分别报告每批median及其p95、内部尾延迟、吞吐、总process CPU秒、owner/worker thread CPU、实际team/TID/CPU核、负载与CPU频率观察。小/中/大输入全部列出，保留失败规模，不只选最有利的点数。

事前有限性能门：大输入饱和负载下，完整接收→解码→按序commit的有效吞吐median至少改善10%，端到端延迟p95不退化；小输入使用原串行fallback或端到端p95不退化。同时要求same-input数值/接受与拒绝语义全部通过，计入所有线程成本。若只有owner阻塞时间降低而完整吞吐或尾延迟未通过，标`OWNER_BLOCKING_IMPROVED_ONLY`，不能称整条SLAM链加速。30Hz未饱和时输出仍30Hz不等于毫无模块收益，也不等于吞吐提升：另报固定提供速率下队列等待和源年龄。

### B. 实际210秒匹配前缀

在完成A以后，按同冻结32区域prefix及相同传感器/模型/控制/地图参数实际跑一对新A/B；独立run、固定初态/seed、相同完整记录策略，A pipeline off、B on。原轨迹不同会造成地图/匹配工作量变化，因此不能称真实运行是逐字节同输入，只将它作为有限运行性能与无退化证据。若结果差异与单轮噪声同量级，追加对称顺序复测，而不凭单轮小差异宣布因果收益。

前缀沿用V18原19共同门、7附加publication/保护门及原ramp适用门，原数值不可更改；32未完成、终点停车和未完成坡段仍FAILED/UNVERIFIED。保留全部来源绑定与正式原收据，不能手工写PASS。原metadata-v2的精确字段关联修正只能在该run冻结producer确实只附加那两metadata键时适用，必须重验全部源及mandatory集合；不能拿旧V18收据解锁新V19。

## 四、时序指标与完整性账本

每个接受事件记录原global seq、topic/type、原整数source header、接收入口wall、decode开始/结束wall及thread CPU、ready、owner pop、commit结束、队列大小/bytes水位及RX/decoder/owner TID。每段计时命名和时钟域在事前schema列明；timer也是序列中的原owner事件。退出契约要求accepted=committed、delivered=committed、missing/duplicated/reordered/canceled=0；有意拒绝须在accept之前有原reason并单列计数，不能在停止时将已接受事件重新称rejected。生命周期和运行期完整性分别报告，正常shutdown也不自动豁免尾包。

| 指标 | 原时间/含义 | 不能偷换成 |
|---|---|---|
|sensor header周期|实际发布传感器采集时间；IMU/LiDAR/RGB分别计数|订阅次数或配置名义30Hz|
|RX入口年龄|接收时ROS clock减原header；入口wall receipt|decode后新的receipt年龄|
|等待|RX→decode start、decode end→owner pop、同步配对等待|所有剩余wall都算纯通信|
|计算|decode、IMU传播/deskew、LIO状态估计/query、VIO、地图update、同步publish各自身wall/thread CPU|嵌套段累加、process CPU当函数独占CPU|
|有效输出率|每个成功LIO/VIO、发布SLAM新header、控制数学新header更新、Actor帧数|spin/ticker频率或重复发布次数|
|闭环可用年龄|真正控制/guard消费时对应原pose/cloud/IMU的双龄|平均帧率代替最大延迟/300ms保护|

同步publish包含转换/序列化/RMW同步工作；DDS返回后的异步通信未单独计时的部分明确N/A。进程外CPU采样只读取自有PID及线程，从launch起每2s记录ticks/affinity/context switches和wall窗口；不把生命周期ps均值或未证实角色的python进程算成“通信”。日志窗口之外缺失kind100/patch细节标N/A，不写0。

固定前缀比较窗为sim60–110、130–170、170–210s，同时给全活动期；报告每窗输入、接受/消费/输出率、p50/p95/p99/max源龄、header间隙、queue水位、decode/commit时延、源超时/clock hold/零命令次数、CPU和wall秒。Actor保持CPU1，控制ticker20Hz不变，数学只在真实新header更新，duplicate clock保护及全部publication chronology依原合同。GPU训练负载只观察，不停止、不调整。

## 五、原46任务运行与独立验收

顺序：原点静止初始化→同新provider的单上下坡/阶段切换→V19匹配prefix→真实动态停车恢复组件→原三阶段46。各阶段失败保留；只有前置安全与来源可用后才尝试完整长任务，不靠加长TTL、删除故障门或放宽姿态/区域“跑完”。完整新scope可沿用0.2m/s参考和64×480 LiDAR30/RGB30等已测设置，但场景传感器rate变更须显式另冻，不能冒充原10Hz硬件配置没变；不在本次流水线实验夹带1m/s或PID重新调参。

原46义务逐项通过，不能拿32/46分子或固定仿真时长当成功：

- 原18/14/14区域全在顺序的同一NAV raw SLAM dwell窗通过；每request定义SHA相同、回执唯一、累计46，没有跨楼层误到达；一次初始坐标锚点跨返航/保存/再导航保持。
- 实际状态转移`waiting_sensors→exploring→returning→saving_map→navigating→completed`，返回原点后再保存；地图是当次`/cloud_registered`与实际RGB投影，binary/PCD点数一致、≥500点、≥8种颜色、capacity_rejections=0；保存时原六类来源龄<2s且SLAM/相机健康。地图保存2s历史门不放宽运动的300ms门。
- 动态箱有成功的真实Gazebo位置更新和零失败更新、原navigation阶段active history、≥1m实际位置变化、blocking后leaving/clear；实测cloud guard先停车，固定5s窗口按既有严格动态协议检查实际速度/角速度，原raw cloud涵盖进入前至少300ms的pre-roll及所有guard使用帧，连续clear≥1s，guard相邻间隔≤300ms，实测停稳后丢弃旧轨迹并接收新SCAN reference才恢复。不以脚本“箱已离开”的真值替代点云清障。
- 两条12m坡道分别两次上/一次下全部完成COM入口/出口、四脚实际顶面接触/法向、有序全12个1m bins和landing支持；事件须按stage/request/方向唯一绑定，不能只验一次上行然后按次数复制。平台和倾角/接触、PD/DC、安全与关节门仍依冻结Teacher契约及既有原common/ramp门；原旧任务<0.75rad上限不能覆盖Teacher更严格保护。
- 原Teacher physical/source/publisher/control math与原强制账本门通过：200Hz native完整、50Hz Actor、body-contact/fault=0、clearance≥0.18m、roll/pitch≤0.65rad、torque≤23.50001Nm、joint speed≤30.001rad/s、无全足失支撑>0.3s、q_default+0.25action及PD/DC逐样本可重放、唯一执行器、300ms双龄与原slew/PI全部复核。运动保护仍原0.30rad hold/0.50rad fail、清除<0.18rad连续0.8s，不用离线0.65门解除运行保护。
- 行驶tube max≤0.20m/RMS≤0.08m、heading≤0.20rad、速度MAE≤max(0.05m/s,0.25×reference均速)作为原Teacher运动门，参考每阶段原冻结SCAN路径；不能用后验最近点滑动消除错误。最终到F3后固定首5s active hold，XY drift≤0.05m、yaw≤0.1rad、原body-origin速度peak≤0.08m/s及Euler yawdot/body-wz≤0.1rad/s；未进入hold或缺窗口标未验证。
- actual loaded库/模型/配置/world/profile/源快照SHA完整，自有worker/bridge/nav/SCAN/Gazebo/observer/writer退出与全生命周期账本一致，无掉记录/写入错误。nav真值使用必须显式false；Actor232特权+15命令/上一动作单独说明。地图/物理真值只用于独立离线验收。

必须新建46适用的验收schema：原46区域几何与0.4s门取原Mission定义，原32 common的统一0.17/0.6s区域函数并不能直接复用。替换的是**不同任务的声明与计数**，不是事后放宽门；其余Teacher来源/物理/控制/停车全部数值照旧。事前冻结完整必需check key集合，缺一个必要门、输入缺失或foreign-run hash均不得给总PASS。负例至少包括46→32删点、stage重排、中心/axes/height/dwell变更、跨request复制回执、phase重新注册、关闭动态、错用GT导航、缺map彩色payload、缺publication、queue尾包取消、缺raw guard pre-roll及其他原门failed。

目标全局上限可另预登记1500仿真秒作为一次有界尝试；600秒只是旧32任务上限，121m/0.2m/s已约607秒，不宜机械复制600让旧46必超时。每点90s、原阶段1500s与地图保存30s原义务仍保持，若实际实现需要新的总wall上限须事前明示；期限更长不能把缺区域/动态/停车视为通过。

## 六、raw预算与结果保存

旧full每帧30k左右原cloud .bin约1.4MB、filtered XYZ .npy约0.70MB，30Hz约63MB/s。210秒匹配prefix每轮约17GB完整payload；完整46估计900–1500s，仅这两路导航云就约57–95GB，再加telemetry/native、实际RGB帧、地图和诊断建议每完整run预算120GB，开跑盘可用≥150GB且保留至少30GB安全余量。先离线同输入比较、再一对prefix、然后一轮46，失败后根据结果决定复测，避免所有候选同时全量积累。

只保3s详单窗口（115–118s）和轻量全程kind300/最终state/raw header等既有必需来源；任何新的记录减负/压缩/去重必须事前冻schema与逻辑字节SHA、完整读取回放测试，A/B记录策略相同。不能运行中删required clouds、用0.2s摘要或hash替代原200Hz安全/全部guard/每次publication；不能为省空间丢掉退出尾包证据。存储达到预算应安全停车、完整落盘并记录未完成，不临时放宽验收。

每run保存manifest/源快照、original正式失败/未验证/通过收据、性能表及曲线、raw记录、失败原因和两个真实相机画面；完成独立验收、writer关闭、封存/远程精简证据后才按用户已授权的清理规则删除大raw。清理后再次显著标明历史无法完整重审。导航/运动/Sim2Sim/性能/接口/真机分层结论始终分别报告。

## 审查收据

本次只读源身份与原46/32结构核对保存在`evaluation/SOURCE_AUDIT.json`。本协议本身不是执行授权收据、生产预检或PASS来源；V19新源/实际库/队列契约/46adapter与新验收器完成后，还须在首次实际运行前生成绑定这些生产文件的prospective manifest。已有root用户授权允许在独立仿真任务内推进，但不能继承旧已封存PASS来证明新库。
