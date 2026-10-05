> 当前任务已切换至 [相机传感器直接移动模式](../camera_mode/README.md)。四足控制、步态和强化学习试验按用户要求暂停；本文保留 Go2 历史说明。v9实际为0/2区域、90秒超时失败，试验已停止。

# 开发计划、模块分工与验收状态

更新日期：2026-10-02。所有模块已接通，多项组件实际验证完成，**完整闭环尚未通过**。最新Full19探索18/18与首个返航区域通过，随后三楼下坡倾角失败，未返航保存正式地图。329项生产源码保持冻结，主动姿态反馈仍关闭。后续下坡v8启动包络仅完成1/2后在117.389sim触发0.50rad失败；同源开启姿态反馈的v9只改变该开关，保持原区域、期限、D1及停车归位。既往失败与缺样完整保留，不以数学或组件来源通过替代完整运行通过。

最新阶段：CHAMP 首摆两行相位修正、低D闭环关节控制、CM总命令限幅、SLAM ALIGN纵向补偿及关闭姿态控制的必要状态转发节点已正式接入网页入口。原四区域4/4、原八区域8/8、动态两点2/2均通过原组件合同与独立审查；八区域raw峰0.236467rad／0hold，动态raw峰0.216827rad／0hold，均无躯干接触。第19轮329项源码与运行时前后一致，完整任务实际FAIL；见[原官方验收](../runs/20261002_084329_14d9cc/acceptance.json)及[有限机械审计](../simulation/test_results/full19_mechanical_audit/final_receipt.json)。见[正式资产11项等价核验](../simulation/test_results/full19_standard_equivalence.json)、[八区域独立验收](../slam/test_results/oct2_CHAMP_phase_first8_actual_independent_verified.json)及[动态独立验收](../slam/test_results/oct2_CHAMP_phase_dynamic_actual_independent_verified.json)。原失败、活动RH丢样及各记录器/FAST消费缺口均保留，不以组件成功回填历史。

## 交付范围

目标流程：网页启动真实Go2仿真 → 按规定路线观测三层彩色点云 → 返回初始化原点 → 保存本轮地图 → 再次从F1到F3 → 感知实际移动障碍、停车、清空后重规划恢复。

当前可重复场景采用高度0/1.2/2.4m的三层实体平台与10%缓坡，明确非楼梯。使用真实Gazebo接触/关节动力学、LiDAR、IMU和RGB；没有用XYZ样条回放代替机身物理，也没有加载完整参考PCD冒充建图。当前动态避障范围为停等恢复，主动绕行尚未交付。

默认ROS域72、Gazebo分区 `go2_multifloor_demo`、[网页8767](http://127.0.0.1:8767/)，与旧域55/66分开。实现细节见[代码结构](CODE_STRUCTURE.md)，操作命令见[Demo README](../README.md)。

## 分工和已交付内容

| 模块 | 责任与写入范围 | 已实现/已分测 | 仍需完整运行验证 |
|---|---|---|---|
| 场景、Go2、传感器 | simulation子agent，`simulation/` | 独立三层SDF/Go2 URDF；CHAMP/effort控制；唯一IMU/RGB/LiDAR；独立真值；躯干和足端接触；实体动态障碍；平地、上下缓坡、左右90/180纯转、转后正向前进和contact正负例通过；停止归位的平地/短坡A/B与真实四转完成，默认接入并合并健康/故障 | 完整探索/返航/再上楼过程的步态、控制切换、净间距和接触 |
| SLAM、外参与地图 | slam子agent，`slam/` | FAST-LIVO2启动；参考点/有效速度适配；本轮实际RGB积累与保存；数值/存档/ROS接口通过 | 全程行走/跨层定位精度、返航误差和三层地图覆盖 |
| SCAN、反馈执行、障碍 | navigation子agent，`navigation/` | 仅接SCAN核心；真实SLAM反馈；自体回波/地面包络修正；侧移禁用、纯转对齐、斜率/倾角/断流保护；ROS与SCAN接口通过；0.08行走偏航修正的动态两点同步验收与严格单坡通过 | 长路线执行稳定性、全路线中实际移动障碍停等与恢复关联证据 |
| 调度、网页、总装验收 | root，`mission/`、`scripts/`、`web/`、`tests/`、`runs/` | 分阶段状态机、服务/进程生命周期、真实相机/地图/路线网页、失败门控、预声明区域/身份哈希/整数时间保持/同窗口双源独立验收 | computer-use操作后的所有阶段完成、保存文件和最终独立通过报告 |
| 文档 | root统筹，子agent更新各模块与结构/计划 | 根README、代码结构、分工、分测说明已落实 | 完成后按实际最终结果重写操作和验收报告 |

模块边界内维护实现；跨模块接口先协调，保留旧实验与失败证据。真实物理测试域73、SLAM接口域74、导航接口域75只用于隔离分测，不能污染域72整合结果。

## 已完成的里程碑

1. **基线与契约**：建立README、目录分工、独立域/分区与本轮存档；保留旧SLAM5和参考SCAN演示。
2. **模块装配**：真实Go2传感器接入FAST-LIVO2，状态/时序/速度适配后接SCAN；网页可实际启动和观察本轮RGB/路线/相机。
3. **必要分测**：早期单源100Hz IMU且时间戳无重复（后经真实采样率对照，当前生产1000Hz），雷达10Hz、RGB约10Hz；短程上下缓坡与正反纯转通过；实际躯干碰撞负例被正确识别，正常足端地面接触独立分类。
4. **集成修正**：修正自体回波、地面体素膨胀与机身参考点不匹配、与航点冲突的障碍停放位置；保留失败记录和现场点云。
5. **执行策略收紧**：第三轮完成前三个航点后复合运动倾覆；当前vx上限.12、vy=0、yaw上限.12，纯转后对齐/稳定，加入斜率和倾角保护。固定转向目标后，再增加转前精确零速1仿真秒，四段物理转向/前进均通过；同一步态最大IMU倾角由失败版.611rad降至.133rad。保留.07m步高，不引入未必要的步态变更；这些分测尚不替代完整SLAM导航验收。

## 集成进度与后续顺序

| 轮次 | 已观察事实 | 当前结论 |
|---|---|---|
| 第1轮 | SCAN起点被真实机身自体回波占据，首航点失败 | 已记录并加入已知实体包络过滤 |
| 第2轮 | 地面离散膨胀/参考点导致近目标规划异常，未完成全任务 | 已修正几何语义；同轮核查发现停放障碍占用探索航点并移出 |
| 第3轮 | 前三个航点完成，复合侧移/转弯时倾覆，真值保护触发失败 | 已收紧执行策略；未把倾覆后的地图当可用结果 |
| 第4轮 | 首段偏离目标并超时，完整流程再次失败 | 正在定位命令、步态切换与测量方向，不预判修复成功 |
| 第5轮 | 同步记录证明连续正vx方向正确，反复细调转向造成净漂移；SLAM倾角保护最终触发失败 | 已加入固定转向目标、转前/转后停稳；首版四段物理分测失败，新增转前停稳的单变量对照四段通过，待集成验证 |
| 第6轮 | 首点附近反复切换停车、对齐与行走，未满足稳定到达窗口 | 已平滑跟踪方向并加入持续偏差门控，严格到达阈值保持不变 |
| 第7轮 | 首点同时通过SLAM/真值验收；第2、3点只有SLAM确认，第4点SCAN优化失败、返回原地停车样条 | 独立验收失败；已拒绝退化样条，增加连续占据云存档；正在对比实体自回波过滤与IMU覆盖 |
| 第8轮 | 前三点联合验收通过，全程定位RMSE约0.050m；第四点受原点附近历史z=−.20m占据及竖向膨胀阻塞 | 完整验收失败；射线更新修复已通过实际地图类、ROS与几何回归 |
| 第9轮 | 首个负向纯转时倾覆，yaw≤.12、vx/vy=0；SLAM姿态滞后 | 完整验收失败；加入导航和执行器两层原始100Hz IMU门控，隔离接口与负向纯转/四转/实际SLAM+SCAN负载物理分测通过，尚不能证明全程稳定 |
| 第10轮 | 全程指令为零，启动恢复运动混入300条IMU初始化；重力倾角3.876°，3°坐标校验拒绝 | 初始化超时，完整验收失败；原始记录器序列化错误导致本轮原始记录缺失，错误已修复并通过真实ROS传输回归，正在增加连续静止初始化 |
| 第11轮 | 双层静止初始化实际通过、前四航点联合通过；最大物理倾角.1823rad，无机身接触；第五点目标方向多次变化后超时 | 完整验收失败，正在核查当前航点与SCAN规划的交接，保持原有阈值 |
| 第12轮 | 连续静止初始化、北向交接及前7航点联合通过；爬升0.62m，第8点最近SLAM/真值0.317/0.307m，超时 | 完整验收失败；定位瞬时跳动、反复转向与坡上漂移继续分离验证，保留严格到达和超时标准 |
| 第13轮 | 真正1000Hz IMU、无点时间雷达同步修复及前7航点联合通过；定位RMSE0.00475m/max0.02040m，爬升0.592m | 完整验收失败；第8点转向净后退0.881m，正在核对路线更新与真实偏航响应，保持到达/碰撞/时间门限 |
| 第14轮 | 默认停止归位与0.08行走修正真实运行，前7/18联合通过；固定初始SE3定位max0.01636m；第8段纯转净world X−1.938m、3次倾角暂停/max0.42794rad | 第8点超时，完整验收失败；45次归位均完成且身体接触0，执行接口成功不代表主动平衡；[独立物理分解](../simulation/test_results/20261001_full14_physics_audit.json) |
| 第15轮（新区域合同） | 18个NAV区域到达、实际跨两坡到三层；同初始SE(3)定位RMSE0.04365m/max0.13411m；独立有序联合前缀11，第12区域GT未满足同窗口边界；131次归位完成、躯干接触0、两次倾角hold/raw峰0.44077rad | 返航途中主动停止并清理，完整验收FAIL；实际RGB快照293253点，正式返原点后PCD保存及再次导航/动态障碍未完成；[独立验收](../slam/test_results/full15_final_independent_evidence.json)、[物理分解](../simulation/test_results/20261001_full15_physics_audit_official.json) |
| 第16轮（v2控制内带） | 实际10–13sim静止启动通过；导航前机身命令全零/关节目标恒定，16.938sim原始IMU达到0.50rad并锁存失败，关闭尾部峰1.0513rad | 未进入任何探索目标，官方完整FAIL；来源/二进制前后一致并清理；[独立启动审计](../simulation/test_results/20261001_full16_startup_physical_audit.json) |
| 第17轮（最终CM与退出修复） | 网页实际启动约20.4wall秒；适配器337.237毫秒回调记录空窗触发原300毫秒required状态期限，同期IMU连续，全部已记录机身命令为零/关节目标恒定；后续idle状态恢复不清除安全锁存 | 未进入路线、零SLAM地图，完整FAIL；具体阻塞函数尚未证实。生产保持冻结，准备仅诊断回调/发布/日志边界；[官方验收](../runs/20261001_232612_ad2317/acceptance.json)、[清理与来源](../runs/20261001_232612_ad2317/root_terminal_provenance.json)、[复现审计](../simulation/test_results/full17_startup_analysis/audit.json) |
| 第18轮（可选控制边界诊断） | 网页启动后前三个区域完成，第四目标纯转6.973sim伴真实水平漂移0.390m；姿态119.295sim越0.20，119.823sim越0.30并立即zero，120.104sim越0.50；末段适配器/桥状态新鲜、无长调用停顿证据 | 完整FAIL；失稳起于停止归位之前。原步态缺少主动姿态反馈，关节实际响应仍缺本轮JTC观测，不能宣布唯一机械根因；[物理审计](../simulation/test_results/full18_physical_audit/report.json)、[官方验收](../runs/20261002_001559_882ddb/acceptance.json)、[来源及清理](../runs/20261002_001559_882ddb/root_terminal_provenance.json) |
| 第19轮（正式相位/低D/ALIGN组合） | 原18探索区域及首个返航区域同窗通过；三楼下坡820.629/834.817/855.978sim触发倾角hold，856.137sim原IMU达到0.500293rad并FAIL | 329源及实际库保持、进程清理。有限窗8次zero的q0连续与ACK齐，前7归位完成，末次因失败中断；倾角先于stop，不能归为唯一reset原因。360万CDR后缀覆盖2177615条前缀、早期采样缺口原留；[独立原窗](../slam/test_results/full19_independent_final.json)、[机械收据](../simulation/test_results/full19_mechanical_audit/final_receipt.json) |

第17轮之后的40sim冷启动诊断没有重现337ms适配器停顿；节点本地边界完整，附加CDR观察器报告了667条IMU/clock等丢记录，不能称跨流记录完整。后续采用更窄的可选 `control_timing_trace.py` / `control_timing_runner.py`：每节点500000事件环形RAM，只在退出后保存最近后缀和覆盖计数，首个Bridge failure独立保留；没有worker、磁盘热路径、状态发布或控制动作。原Adapter/Bridge全文未改，默认组件关闭，网页子进程显式启用。真实ROS53项（多次覆盖、消息字段与对象、原异常、保存故障、原300ms锁存）和轻量导入4项通过，见[最终收据](../simulation/test_results/control_timing_ring/ready_receipt.json)。第18轮实际保留Adapter56.371–120.265sim、Bridge47.7–120.264sim后缀，均无诊断错误；末段命令/关节目标发布最长1.382/1.396毫秒，首次失败为实际姿态保护而非状态超时。该结果不证明第17轮停顿已修复，也不代替实际关节伺服周期或力矩测量。


第十四轮失败及清理证据保留。第十五轮已通过浏览器实际执行至探索结束与部分返航，未完成整条流程：三层探索18区域 → 原路返航14区域与RGB地图保存 → F1→F3的14区域 → 实体动态障碍 → 独立区域验收。本轮使用已采纳的生产执行层；主动姿态反馈和专用关节传感器候选仍隔离，不加入本轮。前置阶段失败立即停止后续任务；实际区域完成也不能代替地图保存、动态障碍或全流程验收。

新运行使用预先声明的 `three_platform_body_arrival_v2` 控制内带：平层R0.25m/H±0.10m、坡道沿坡/横向/法向±0.25/0.20/0.07m、原点球R0.17m。外部验收区域继续为平层R0.35m/H±0.10m、坡±0.35/0.30/0.10m、原点R0.22m；所有46中心/外轴、0.4s保持、0.2s间隙和90s期限不变。v1明确保留给旧run，控制收紧不允许改写第十五轮FAIL。

[独立v2几何审查](../simulation/test_results/20261001_region_v2_geometry_review.json)完成13458个内外包含/坐标变换样本，实际固定trunk及传感器collision（含L1网格）XY包络半径0.328892m。用保守0.40m圆盘检查外区域，静态障碍XY余量至少1.0m、坡横向余量0.30m。此结论不覆盖运动腿、roll/pitch、坡接缝和足端接触；平层高度仍与外区域相同，未增加法向误差余量。下一轮必须实际验证收紧后是否能持续停车，并由同窗口SLAM/GT验收，不能仅凭几何检查宣布成功。

## 10月1日组件验证与恢复分工

| 组件 | 实际结果 | 边界与证据 |
|---|---|---|
| 停止归位平地A/B | 最后一次停止相邻目标跳变0.1986→0.00423rad；JTC参考速度/实测速度/输出effort分别记录 | [实际报告](../simulation/test_results/20261001_stop_adapter_ab.json)；不能声称全部指标改善或全局实际关节速度≤3rad/s |
| 停止归位短坡A/B | 两组均完成；A52.971/B63.436仿真秒，B较慢且峰值倾角较高 | [原始组件报告](../simulation/test_results/20261001_slope_stop_ab.json)；历史driver真值插值未设最大覆盖间隙，不能当最新严格覆盖契约通过证据 |
| 生产HeadingGate四转 | 四个4秒前进段实际前向位移0.312/0.356/0.389/0.344m，8次归位完成；无身体接触 | [四转审计](../simulation/test_results/20261001_fourturn_stop_adapter/fourturn_audit.json)；真实SLAM/SCAN负载，原步态和门限 |
| 固定混合运动A/B | 同vx0.12，yaw−0.04/−0.08各15秒；两组安全完成，B负向响应更强 | [固定输入对照](../simulation/test_results/20261001_mixed_yaw_ab.json)；A未复现反号，不能据此单独证明闭环修复 |
| 动态两点0.04/0.08 | A第2段超时且出现倾角暂停；B两点联合到达、第2段75.469秒、峰值倾角0.169rad、无身体接触 | [同步验收](../slam/test_results/oct1_dynamic_strength_synchronized_acceptance.json)及[实际停车时序](../navigation/test_results/20261001_dynamic_strength_held_audit.json)确认停车零命令/实际新SCAN恢复；原driver的4Hz标签滞后FAIL与SHA不覆盖 |
| 0.08严格单坡 | 3m/0.3m目标45.42秒完成，12项通过；峰值倾角0.2038rad，固定初始SE3最大误差0.00606m | [实际单坡](../navigation/test_results/slope_strength_008_actual/slope_result.json)；真值覆盖间隙≤0.15秒，原90秒与0.22/0.30m门限 |
| 1000Hz审计缓冲 | 实际ROS500输入/80ms暂停旧depth5记录424，新depth2000记录500 | [传输对照](../test_results/20261001_sensor_audit_buffer/result.json)；仅改审计记录缓冲，安全桥仍最新样本depth1 |
| 支撑面姿态反馈 | 数学/ROS边界与真实无运动启动通过；同源专用关节前8点A全部完成但倾角0.4593rad/3次hold严格FAIL，B启动跨流接收停顿约334ms、无运动且安全FAIL | [A/B审计](../simulation/test_results/20261001_measured_first8_pair_audit.json)和[staging说明](../simulation/test_results/active_body_feedback_staging/README.md)；A关节385455个连续1ms样本/定位RMSE0.006m，不能抵销姿态FAIL；候选默认禁用，物理平衡效果未验证，不是WBC |
| 最终CM下姿态反馈前4区域对照 | 原Full18 v2定义/90秒期限/全部保护不变，only-enable 13项和完整映射库一致；A 4/4，B 0/4，B运动中49.653sim越0.20、50.114越0.30并zero、50.170越0.50 | [实际对照报告](../simulation/test_results/full18_feedback_first4_audit/REPORT.md)；B输入新鲜且实际JTC已记录，output.effort为命令而非实际电机力矩。候选不采纳；实际Kd .04→0消融完成4/4，但raw峰0.4646/2次hold，19/20检查通过、严格稳定性FAIL |
| 原始SLAM纯转漂移保护候选 | 保持姿态反馈关闭、原JTC/步态/保护不变，仅NAV新增0.15m/0.2sim/3观测中断；同源前4原区域20/20通过，4次中断、raw峰0.2182rad/0hold，原窗22→120.346sim | [实际结果](../simulation/test_results/20261002_nav_drift_first4_disabled/first_four_result.json)、[物理终审](../simulation/test_results/full18_nav_drift_first4_audit/physical_receipt.json)；16次真实停止零命令与q0连续性通过，871229条CDR无后缀覆盖、第四原窗逐流覆盖。仅隔离组件，不宣布全流程或普遍防倾成功 |
| 同候选扩展前8原区域 | 仅扩大任务输入，8/8原始联合窗完成、全部真值覆盖通过；原20项19项真，raw峰0.3619rad/1hold严格FAIL；第8坡道窗320.8–389.841sim到达 | [原始FAIL](../simulation/test_results/20261002_nav_drift_first8_disabled/first_eight_result.json)、[停止/后缀](../simulation/test_results/full18_nav_drift_extension_audit/first8_stop_suffix.json)。首次hold为193.283sim一楼第5段walk；CDR后缀始于约269.956sim，早期JTC/q/接触缺失不重建。源/运行库不变、域76清理；候选未采纳，动态仅准备 |
| JTC插值起点单参数对照 | 同NAV、关闭姿态反馈、同PID/模型/保护，实际回读false/true；A完成7/8、第八段90秒超时、raw峰0.2291/0hold；B完成0/8、第一段90秒超时、raw峰0.1719/0hold | [A原始FAIL](../simulation/test_results/20261002_jtc_desired_first8_a_false/first_eight_result.json)、[B原始FAIL](../simulation/test_results/20261002_jtc_desired_first8_b_true/first_eight_result.json)、[B机械审计](../simulation/test_results/jtc_desired_first8_audit/b_true_mechanics.json)。参数候选不采纳；两个新组全部2828297/860922条CDR保留，源/实际映射库一致、进程清理。相同360万条/2GiB预算不是覆盖保证/总RSS上限，缺样另列，旧900k组FAIL不改写 |
| CM末级总命令限幅候选（实际双FAIL） | 实际CM18b0→ResourceManager→内存硬件write：122原生检查及11来源检查通过；启禁两组12关节±100/±5/0输入，开启按URDF努力界限截断，合法neutral及4ms输入 | [原生范围报告](../simulation/test_results/classic_joint_profile_design/REPORT.md)。fresh A/B均保持JTC=false、原P100/I.2/D1；实际双服务启动gate的65项ROS78检查和37项纯合同通过。唯一对照变量为CM enforce开关；fresh A=false完成6/8，第七段90秒超时、raw峰0.2965／0hold；B=true完成5/8，第六段90秒超时、raw峰0.2520／0hold。两组完整记录、同源库及清理保留，[实际报告](../simulation/test_results/cm_limits_first8_audit/REPORT.md)；均FAIL、不采纳。总命令限幅不等于实际施加力矩或稳定性通过 |

当前分工：simulation负责执行层实际证据及隔离的IMU/关节/接触反馈候选；SLAM验收agent独立复核原始传感器、固定初始对齐和候选边界；navigation负责当前SCAN路径、障碍原生云事件与实际执行策略；root负责生产采纳、生命周期/故障传播、网页操作和完整全路线验收。所有实际分测进程均独立清理，不能与域72全栈混用发布者。

停止适配器现默认启用，采用 `/demo/control` 正式话题。零机身速度立即传给CHAMP；非零行走目标原样透传；归位按仿真时间连续回到启动实际标定的名义关节姿态，重复零不重启，新命令等待归位。启动等待标定与桥ready；适配器失败/已建立流失联使任务失败，必需Gazebo/CHAMP/桥/适配器/NAV进程退出关闭本轮。主动姿态反馈仍未进入此链路。

## 操作和复测入口

工作区根目录使用 `bash multifloor_demo/scripts/run.sh build` 编译Demo的SLAM两包和SCAN四包，`prepare` 生成独立资产，`serve` 启动8767服务后由网页开始。`unit` 执行项目常规测试；运行环境与完整逐项命令见[Demo README](../README.md)。不要在已有完整栈旁再运行物理分测或另起同域控制发布者。

主动反馈离线验证不启ROS：

```bash
python3 multifloor_demo/simulation/test_results/active_body_feedback_staging/test_feedback_core.py
python3 multifloor_demo/simulation/test_results/active_body_feedback_staging/test_node_contract.py
python3 multifloor_demo/simulation/test_results/active_body_feedback_staging/check_body_pose_fixture.py
```

这些命令仅验证数学/接口反例/几何。真实ROS握手、失联和一次40sim无运动启动已通过；最新最终CM下开启反馈组已运动失败，不能采纳。保持P .25、步态/PID和全部护栏，仅移除Kd .04支路的实际消融已完成4/4原区域，但0.4646rad/两次hold仍违反严格组件稳定性；所有来源及第四段逐流覆盖保留。后续先分析步态支撑、实际关节跟踪与控制层可用性，不翻转未经证明的符号，不放宽30ms关节/5ms配对或100ms墙钟新鲜度。旧传输停顿诊断作为独立问题保留。

区域合同复测（不启动仿真）：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s multifloor_demo/tests -p 'test_mission*.py' -v
DEMO_TEST_PRODUCTION_GOAL_CORE=1 python3 multifloor_demo/navigation/test_results/goal_region_staging/test_goal_regions.py
python3 multifloor_demo/navigation/test_results/goal_region_staging/test_controller_methods.py
python3 multifloor_demo/slam/tests/test_region_evaluator.py
```

当前证据为23项Mission逻辑、5项[真实ROS77](../test_results/mission_regions/actual_ros_result.json)、17项[NAV真实ROS75](../navigation/test_results/goal_region_staging/ros75_v5/result.json)、25项新区域及34项旧独立验收合同。接口合成输入不冒充真实物理；ROS重跑需无并行全栈、独立域、新结果目录，禁止覆盖历史收据。

## 接口和几何约定

- `/livox/lidar`、`/livox/imu`、`/camera/image_color`来自本轮实际仿真传感器；`/clock`是仿真时间来源。
- `/aft_mapped_to_init`是FAST-LIVO2 IMU原始状态，原始twist不能当有效机身速度；适配后发布 `/demo/slam/body_odom` 和 `/demo/slam/lidar_odom`。
- `/cloud_registered_full`用于完整几何；`/cloud_registered`的RGB来自相机观测，用于本轮彩色积累。参考PCD不注入本轮建图。
- `/demo/navigation/request/status`与 `/demo/mission/state`仅根据实际反馈确认任务；样条时间走完不等于机身到点。
- `/demo/cmd_vel`是机身平面速度请求。高度来自物理坡道，不写入机身XYZ。控制斜率和稳定窗口采用仿真时间，失联看门狗采用单调墙钟。
- `/demo/ground_truth`只用于独立验收及失败判定，不为SLAM/导航提供位置或纠偏。比较坐标系只用固定的初始SE(3)对齐。
- `/demo/body_contacts`只在实际接触时报告事件，不能把无碰撞无消息当断流；必须确认发布者/传感器可用。初始化前10s仅记录，之后身体碰撞判失败。足端地面/坡道支撑单独分类。
- `scenario.json`使用relative_world_axes；通过显式IMU CUSTOM/world静态参考与初始SLAM配对冻结heading，再把world方向位移转到camera_init并锚定初始机身。不能直接改frame字符串、用运行时GT纠偏或把SLAM高度当world楼面高度。

## 通过条件

- 所有必需传感器与SLAM持续有效；旧时间戳或重复消息不算恢复，失联必须停车。
- 实测彩色点云在探索过程中增长，保存为本轮实际RGB地图；无参考地图替代、容量截断或缺失存档。
- 新合同运行前冻结46个区域及定义哈希，按序完成探索18/返航14/导航14；平层圆柱半径0.35m、高度±0.10m，坡盒沿坡/横向/法向±0.35/0.30/0.10m，返航原点球半径0.22m。
- 每区需递增原始SLAM整数纳秒在预声明控制内带保持0.4s、观测gap≤0.2s、每段90sim期限；独立GT以同一次初始SE(3)在同一实际保持窗口进入原外验收区域。旧v1继续其原同区域保持；旧固定点运行保持原0.30m合同，历史FAIL不重评。
- 三层实际高度变化、连续路径、控制稳定性及接触证据齐全；无身体碰撞、无倾覆。完整评估原GT最大倾角<0.75rad；控制raw0.30/0.50rad分别触发hold/fail，不能把隔离first8组件的max0.30rad严格标准混作完整run评估门限。
- 实体移动障碍进入路线后，实际LiDAR触发停车；清空后重规划并恢复，不能只凭障碍动画或合成测试确认。
- 网页点击启动/停止、真实RGB画面、地图/路线更新、阶段迁移和失败状态经过computer-use检查。
- `scripts/evaluate_run.py`对完整run生成独立验收报告；缺少任何必需证据视为失败。

CHAMP没有IMU闭环平衡或地形落足规划；当前结果不延伸为楼梯、崎岖地形或实机Go2验收。最终报告须分别列明SLAM、RGB地图、物理执行、动态障碍与网页行为的通过范围。

## 接手后的可复现记录

每轮新增 `source_manifest.json` 和 `source_snapshot.tar.gz`，保存当时 Demo 源码与配置。运行结束由服务自动执行严格逐航点验收，浏览器根据 `acceptance.json` 显示结果。停止使用有界的本轮进程组清理；初始化失败会结束本轮全部模块，避免后台残留。

记录器作为本轮启动栈的受管子进程，在物理仿真启动前建立订阅；必需审计记录器异常退出将结束本轮，不把缺失证据补写成有效运行。

第十五轮RGB仅另存[离线诊断PCD](../simulation/test_results/20261001_full15_diagnostic_rgb/run15_final_live_rgb.pcd)及[来源证书](../simulation/test_results/20261001_full15_diagnostic_rgb/certificate.json)：293253点、14341种实际RGB、xyz和颜色读回完全一致，原run文件哈希未变。证书明确正式保存阶段未完成；不得将该离线导出作为mission保存成功的收据。

## 第十六轮后的执行器时间诊断

保持原模型、步态、PID、现代停止适配器、0.08偏航上限和保护阈值，一次域76默认启动诊断只增加被动观测。完整JTC参考/反馈/误差/输出、真实JointState和四足Contacts以原生CDR归档，保留整数头时间、接收时间、DDS序号及发布者QoS；65项真实ROS接口检查通过后实际执行，来源/运行库哈希与第十六轮相同，进程全部清理。

[实际诊断](../simulation/test_results/20261001_default_startup_actuator_v1/offline_actuator_analysis.json)未复现倾覆，但不是健康全通过：出现一次IMU接收失联hold；JTC的16.456秒头时间重复84条、持续331.8毫秒，真实反馈关节持续运动。输出在部分区间保持，时间恢复时参考速度31.724rad/s、实测速度20.060rad/s；8015条关节目标仍恒定。JTC output effort只是命令，四足记录只覆盖传感器所在链接的实际接触，不能排除全部未装传感器的自碰撞。捕获在40.016sim结束，落盘/关闭尾部官方IMU到40.990sim，实际总墙钟46.95秒；此尾部边界如实保留，不称精确40.000sim停止。

接续工作是核查实际控制器时间、周期、轨迹插值与PID输出之间的关系，再用隔离同源对照验证最小修复。不能将区域内带当作步态平衡修复，不能把SLAM位置/头向反馈误写成已经实现机身速度或足端闭环；测量速度观察器目前仍仅诊断、没有修改运动命令。主动姿态候选保持默认禁用。

隔离站立对照A3/B3已经补齐原生 `ControllerInterfaceBase::trigger_update` 的真实输入证据：A的同步JTC有149次零周期，两个连续零周期区间约260/328ms；B的JTC10003次及JSB10013次均为4ms，控制时间严格前进。[对照报告](../simulation/test_results/20261001_cm_time_a3_b3_comparison.json)的13项来源/时基/零命令证据全部通过，同阶段停止前全部非CM加载库路径与哈希相等，前后源码及运行库未变、子进程均正常退出。两组10sim后原始IMU倾角峰0.00257/0.01454rad，B的短暂关节速度与输出峰仍更大，不能称稳定性全面改善。A记录器缺3个IMU样本、B头时间连续，均明确保留。

时基修复已在`simulation/ros2_control_ws`重建为最终库，真实受控接口12项通过；生产仅Gazebo子进程选择该库（SHA `18b0ed04…`），PID/步态/阈值未变。接续验证须核实际加载身份，执行真实四次转向/停车、严格单坡和动态两点，再由网页运行完整区域任务；不因站立时基通过而跳过运动验收。首次A缺JSB参数回应及A2/B2早期库映射覆盖不一致均保留为失败证据，新有界只读参数重试和停止前路径快照没有改变运动或安全门限。

## 当前待验证的明确关节控制方案

CM限幅单参数对照两组均超时失败，后续测试采用明确的多参数方案：JTC插值使用期望状态、十二关节P=220.982919／D=3.360637、原I=0.2／i_clamp=2.5／velocity FF=0、CM总命令限幅开启。其依据是200Hz目标／250Hz原生JTC接口拟合，不是机器狗动力学模型，也不构成单一原因证明。保持原canonical前四个区域、原90秒期限及全部姿态／接触保护、关闭主动机身姿态反馈。

[准备收据](../simulation/test_results/classic_pd_first4_staging/prepared_receipt.json)记录私有YAML/模型参数路径、269生产源码和运行库不变、21纯合同与41实际ROS参数门控检查通过。新增益原生接口19项及独立审查完成后，实际仿真仍在150秒启动门限内持续角运动，静稳门以 `angular_motion` 超时。未触发后续64值实际回读，也未启动SLAM/NAV；不能表述为前四点导航的完成比例。931082条原记录全部保留、组已清理；原 `actual_control_profile_verified=false` 和退出前映射快采失败保留，候选不采纳；零速恒定目标下约125Hz交替限幅振荡的原始帧见[机械审计](../simulation/test_results/classic_pd_startup_audit/REPORT.md)。见[原失败](../simulation/test_results/20261002_classic_pd_first4_candidate/startup_gate.json)及[清理收据](../simulation/test_results/20261002_classic_pd_first4_candidate/process_cleanup.json)。

低D（D=1.0，其余上述声明不变）接续组件已实际完成原前四区域：20/20检查、4/4原始窗，18.000–134.320sim，raw峰0.282319rad／0hold，19次实际停止归位检查通过。实际64参数与启动前5库路径SHA先于原静稳门采集，终态全部来源／清理通过。静止10–13sim实际qdot峰1.10e−5rad/s，运动误差和转向退移仍存在；详见[机械及停止证据](../simulation/test_results/classic_lowD_first4_audit/REPORT.md)。同源前八区域实际测试现已终态：7/8，第八原355.8–445.945sim窗口90秒超时，raw峰0.27399971rad／0hold，原20项16项真、完整组件FAIL。66次真实停止归位通过，参数／映射库／来源和owned清理均通过；3245263条CDR全保留但DDS缺样仍单独记录。第八段粗命令分类显示walk约57.66s前进3.781m、纯转约12.70s退移1.063m，精确native命令分解另独立核验。详见[八点原始FAIL](../simulation/test_results/20261002_classic_lowD_first8_candidate/first_eight_result.json)和[物理与CHAMP几何审计](../simulation/test_results/classic_lowD_first8_audit/REPORT.md)。不因安全分项通过而采纳或自动进入动态／完整流程。

## 最终CM的运动分测完成状态

三段均使用最终CM `18b0ed04…`、原CHAMP步态/PID、生产0.12前进/0.08行走偏航/0.12纯转向限速、现代停止适配器；未启用主动姿态候选。实际Gazebo加载路径与哈希、前后源码/运行库及进程组清理均核对。

| 组件 | 实际结果 | 证据 |
|---|---|---|
| 四转/停车 | 8/8；−90/0/90/180°后各走4sim，独立前向位移0.344/0.269/0.378/0.387m；raw峰0.2031、无hold/body接触，8次归位完成 | [结果](../simulation/test_results/20261001_final_cm_fourturn_v2/motion_result.json) |
| 严格单坡 | 15/15；45.117sim完成原3m/0.3m目标，小于90sim；raw峰0.19475、无hold/body接触，GT全覆盖、固定SE3最大误差0.00526m | [结果](../simulation/test_results/20261001_final_cm_slope/motion_result.json) |
| 动态两点 | 22/22；实际终态117.013sim；61.742–87.256sim停等中1533条双命令全零，新当前目标SCAN参考执行后SLAM/GT均真实前进；raw峰0.19879、无hold/body接触 | [结果](../simulation/test_results/20261001_final_cm_dynamic/motion_result.json) |

[执行器离线汇总](../simulation/test_results/20261001_final_cm_motion_suite_summary.json)保留8/1/9次停止：入口q0与上一发布参考精确相等、原生零速确认/归位齐全，非walk状态实际机身非零指令为0。实际JTC参考速度仍可超过理想插值曲线3rad/s界，不能混称实际关节速度限位已满足；输出effort仍只是控制器命令。

动态组件SIGINT之后适配器`main finally`发生关闭竞争、子进程exit1，原记录保留。已只修该退出路径：上下文已关闭时忽略该RCLError，活跃上下文仍抛出，并确保日志和节点清理；运行中Adapter类方法保持不变。真实ROS79三例21项通过，见[修复收据](../simulation/test_results/adapter_finally_shutdown/result.json)。其后的完整网页第十七轮启动失败、第十八轮第四区域转向倾覆，原结果均保留；不能将这些组件结果改写成完整通过。


## ALIGN 补偿组件与下一输入扩展（10月2日）

在相同低D方案上，新增仅导航 ALIGN 阶段的实际 SLAM 纵向位置／速度补偿，保持原漂移中断、真实零速归位和新 SCAN 交接。隔离前四区域本次4/4、原20项全部通过，raw峰0.149693rad／0hold；12次停止连续归位、356条归位期实际命令全零。实际64参数、5库、来源及清理通过，但活动第三段88.411sim的IMU与三个足流各一次丢样通知原样保留，第四段固定频率原始数据完整。见[实际关节与停止报告](../simulation/test_results/classic_lowD_align_first4_audit/REPORT.md)和[独立原窗验收](../slam/test_results/oct2_classic_lowD_ALIGN_first4_actual_independent_final.json)。同候选前八区域随后实际完成6/8，在第七坡道目标达到0.50rad失败（活动峰0.500207rad、2holds），第八未进入；源码、实际参数／库与清理通过，2408478条记录全保留，少量活动期丢样原样保留。首0.20出现在普通DRIVE，首0.30才在ALIGN；保护两次均完成真实零速和连续归位。失稳前还观测到普通步态重新启动时两腿关节目标约0.42rad跳变，尚未证明唯一原因。见[八点临界关节与停止审计](../simulation/test_results/classic_lowD_align_first8_audit/REPORT.md)。生产未采纳，完整Demo仍未通过。

前八准备只扩展原 canonical 输入数量和任务编排；三份 NAV 候选源码、P220.982919/I0.2/D1.0、JTC=true、CM限幅、关闭机身姿态反馈、实际64RPC→已加载5库→原静稳门启动顺序、3.6M条/2GiB被动后缀预算均与本次四区域一致。第八段覆盖必须从第七个原native receipt结束时间逐流核验，不能用迟到status或条数估计代替。未重复相同字节的参数/原生JTC套件，实际执行需独立准备签收及root调度。


上述前八组已实际终止为6/8 FAIL。后续动态和全流程未执行；下一控制变更必须以首失稳阶段和真实关节目标连续性证据为依据，不重复失败组或放宽区域／期限／姿态门限。关闭尾部最大倾角与活动原0.50失败分开报告。
