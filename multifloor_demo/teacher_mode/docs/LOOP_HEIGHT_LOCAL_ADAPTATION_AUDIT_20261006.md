# 回环、局部点云匹配与高度误差：本地适配审计

日期：2026-10-06。范围：只读源码和已有小报告；没有新仿真、训练、原始流重扫或生产源码修改。本文是候选接线与验证顺序，不是后端已实现或高度问题已解决的报告。上游后端选择由根任务另行调研；此处不重复罗列未核验的第三方实现。

## 当前结论与证据边界

应并行准备两条能力：**局部匹配健康检查，阻止不可信测量继续污染状态/地图；关键帧与回环后端，校正长期地图和轨迹一致性。** 只增加回环节点，不能保证首次约219秒的高度偏离不会发生，也不能保证马上恢复当前SCAN地图。

本次V25 e8de中，独立固定初始SE3、同时间比较表明实体base-link在217–230秒约1.50m稳定，而SLAM在相同平台附近逐渐出现约−0.20m的高度偏差。第12区原90秒超时，11/46实际到达；34次退化样条、50连续规划失败与停车已保留。高度偏离和规划停车处于相近窗口，**缺少首个失败占据查询和当时局部地图，不能宣称高度误差已被证明直接造成该碰撞判定**。来源：[V25最终诊断](../test_results/curvature_full46_20261006/v25_diagnosis/E8DE_FAILURE_DIAGNOSIS_FINAL.md)、[SCAN源码审计](../test_results/curvature_full46_20261006/v25_diagnosis/SCAN_STOP_SOURCE_AUDIT.md)。

此前V7的另一轮详细诊断已有较强机制证据：近线竖墙点被历史近水平plane关联，LIO更新经完整19维协方差耦合改变vz并继续积分，实体没有相同下沉。该证据支持优先检查平面二维支撑和法向一致性；**不能把它直接当成e8de的已证根因**。[旧高度故障报告](SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)中的单迭代反事实，也不是修复后的全轨迹。旧初始化审计仍可复用，但其中“V6 pending”是当时状态，不是当前结论；后续报告已经记录V6仍失败。

初始注册小倾角不够解释本次高度误差。27bd第12区审计中，初始重力残差最大1.891mrad；将完整初始roll/pitch加入目标变换仅使目标z增加约22–29mm，反而增加负偏差。它不能排除后续重力/bias/地图误差。[高度/坐标系审计](../test_results/curvature_full46_20261006/v24_actual_review/HEIGHT_FRAME_AUDIT.md)。本次native只作为离线验收参考，不能成为导航位置或在线高度修正来源。

## 实际源码和分层边界

下表中的 `CORE` 指 `multifloor_demo/teacher_mode/navigation/pipeline_v19/slam_ws/src/fast_livo2_core`；`NAV` 指 `multifloor_demo/teacher_mode/navigation/corridor_tracking_v25_terrain_event`；行号是本次读取的冻结源码。主项目还存在 `slam5_navigation/ros2_ws/src/fast_livo2_core`，它与实际使用的V19不能混作同一版本。V25通过 `NAV/slam_workspace.py:8–68` 显式继承V19源/构建/私有库，并要求实际加载证明。

| 边界 | 当前实现与可接位置 |
|---|---|
| ROS可执行程序 | `pipeline_v19/slam_ws/src/fast_livo2_ros/src/main.cpp:8` 创建ROS节点、调用core、处理正常退出。主SLAM5的main也只负责薄启动。 |
| core入口 | `CORE/include/fast_livo2_core/runner.hpp:10` 隐藏含Eigen/PCL的LIVMapper布局，`CORE/src/runner.cpp:15` 在同一库中构造。不要在新ROS可执行程序跨ABI构造具体LIVMapper。 |
| 接收与估计owner | `CORE/src/LIVMapper.cpp:800–851` 接收/解码可并行，但 `sync_packages→processImu→stateEstimationAndMapping` 仍由一个owner顺序推进。后端线程只能回传候选约束/变换，不能同时改 `_state`、`voxelmap_manager->state_`、VIO历史或地图指针。 |
| IMU预测 | `LIVMapper.cpp:462–475` 去畸变/传播后保存 `state_propagat`，将同一先验送LIO；这里适合标记前后状态和测量源时间，不适合异步覆盖bias。 |
| LIO更新 | `LIVMapper.cpp:603–607` 调 `StateEstimation`、立即接收状态和关联点。可信度判定须在接受状态、发布和地图写入之前形成。 |
| 匹配与求解 | `voxel_map.cpp:487–675` 逐迭代关联、H/R/z、完整19维prior与状态/P更新；`BuildResidualListOMP:827` 为每点选择plane，`build_single_residual:947` 做半径/协方差门和概率择优。 |
| 地图写入 | `LIVMapper.cpp:646–674` 用更新后位姿生成点和协方差，`UpdateVoxelMap:790` 修改活动体素，随后滑窗。当前不是不可变关键帧图。后端不得直接持有这些裸指针跨owner更新/滑窗。 |
| VIO阶段 | `LIVMapper.cpp:532–542` 在另一同步stage调用视觉更新并刷新EKF传播锚。`vio.cpp:2094–2131` 灰度化、局部patch匹配、更新、增添观测；没有全局地点识别调用链。 |
| 位姿/点云输出 | `LIVMapper.cpp:643` 发布LIO后odometry；`:693–695` 完整导航云在同一LIO时间输出；`:1899–1928` 当前全部是 `camera_init`，直接发 `camera_init→aft_mapped`。VIO阶段不重新发相同body odometry。不要混用LIO后位姿与VIO后内部状态作为同一测量端点。 |
| 传输与外参适配 | `multifloor_demo/slam/odom_adapter.py:41–99` 将IMU原点转body/lidar并用连续位姿差分求child-frame twist。原message未提供协方差时`:72–84`填保守值，**这不是实际滤波器P或退化检测结果**。 |
| 控制消费 | `NAV/shared_controller.py:226–255`严格只收camera_init真实SLAM位姿，`:258–306`用附近实际SLAM位姿去自身点；`cascade_core.py:308–311`源仿真龄和本地墙龄都保持300ms。 |

core/ROS已有包边界适合增添独立后端，**但core内部仍直接依赖rclcpp、订阅/发布和ROS参数**（`LIVMapper.cpp:375–400`，core CMake依赖rclcpp等），不是现成的完全无ROS数学库。最小落地可先新增core内只读快照接口和独立后端ROS包，保留现入口；逐步提取消息无关的数据结构，避免为了装后端先大范围重构。

本地V19与主SLAM5未发现place recognition、pose graph、全局loop closure调用链。`lidar/imu/image loop back`表示时间戳倒退清缓存，不是回环。已有VIO参考管理（`vio.cpp:1068–1136`）不等于全局关键帧图；`updateReferencePatch:1140`有定义但当前processFrame没有调用。RGB点云是着色输出，不是已经执行color ICP。

代码还有 `/ground_truth/odometry` 的GPS快捷融合（`LIVMapper.cpp:384–387,1258–1338`），实际Teacher流水线构造检查拒绝 `use_gps`（`:69`）。**后端适配不能启用该通道或把仿真真值换名作为后端测量。**

## 为什么回环不能直接阻止首次高度偏离

1. 回环需要先有可信的旧关键帧，再成功识别重访及几何验证。首次走到新的平台、尚未重访时，未必有回环边；同一XY位置的不同楼层也不是合法回环。
2. 后端通常异步优化。当前LIO在 `voxel_map.cpp:593` 已写state，在 `LIVMapper.cpp:643` 发布，再在`:666`写地图。若错误约束先被接受，后端稍后发现不一致时，当前SCAN占据图、局部平面和视觉参考已可能被污染。
3. 闭环修正的是相对轨迹/地图一致性。没有额外可信绝对高度测量时，全局z原点仍是规约选择；回环不是可指定机身永远处于某个目标z的高度计。
4. 仅修正输出位置或平滑z，不同步处理速度、P、地图与观测版本，会掩盖问题并制造新的帧不一致。已有旧故障显示p-v交叉项能改变vz；不能关掉正常耦合或仅回填一个位置值。
5. 后端的正确全局变换也不自动恢复局部前端的错误模型。是否需要重定位、重建局部子图或重置滤波器epoch，应作为另一个有完整状态/地图合同的流程。

## 更早可做的局部匹配与退化检查

当前确实已有逐点点到plane匹配，不能以“再加ICP”替代查原关联。需要分开看**约束是否不足**和**约束很强但关联错误**。

- 每帧轻量质量摘要：有效约束数/比例、残差RMS/MAD和尾部、normal方向/空间覆盖、`M=HᵀR⁻¹H`的带尺度特征谱、z的条件信息、创新/Δp/Δv及完整prior耦合、plane age/frozen比例、VIO有效点与接受/回退。这些应来自实际采用的H/R和plane，而不是原始点数、发布云的拟合或adapter默认协方差。
- `voxel_map.cpp:156–164`只按最小特征值判断plane；第二切向支撑判断仍是注释。`:309–333`达到max_points后停止更新并释放点。候选可检查二维支撑、采样角覆盖、可靠新视角与历史法向的一致性，并允许有界重估/延迟成熟。旧单环墙面机制值得用实测数据验证；不能套用一个事后mid/max阈值，因为真实地面/坡道扫描弧也可能近线。
- 关联候选协方差门（`:971–988`附近）与求解权重（`:474`）不是同一个sigma。大的历史plane不确定性可以让错误面通过，而数千错误方向约束仍可给出小残差/满秩信息。仅inlier多、condition好或ICP收敛，不足以判健康。
- z在纯水平地面可观察，但XY/yaw可能退化；反过来，竖墙被误分类为水平面可能提供“强但错”的z信息。必须保留真实地面、坡道、墙/边缘、弱视角的正负样例，不以所有小特征值场景一律拒绝。
- 质量门先只记录，再单独版本评估拒绝/重匹配策略。若拒绝更新，必须一起处理trial state/P、关联点/法向、latest EKF传播锚和map insertion；不能只恢复pos。预测-only不得无限延长健康TTL，质量无效时保持原停车保护，并记录测量拒绝及恢复条件。

现有 `voxel_map.cpp:619–672` 的kind100/101/102以及地图事件已能记录这些分析所需的一部分。**当前full46 profile/run冻结详细窗口为[115,118]s**（`NAV/run.py:85,175–178`，logger `diagnostics.h:43–48`）；`StateEstimation:493–494`的aggregate也以detail为条件。219秒附近没有同级完整H/plane/query，不应称已有该窗口的完整匹配重放。下一个独立取证版本应预声明靠近故障的短窗口或有界触发环形缓冲，同时绑定first-stop查询的SCAN map版本/体素/表面来源，控制诊断量和实际时序。

## 关键帧局部子图与后端接点

最小共同接口为一个不可变 `KeyframeSnapshot`，由估计owner在明确的LIO后或VIO后边界生成。字段至少包括：run/estimator epoch、keyframe ID、parent ID、原始整数sensor header时间和本地receipt、明确阶段、body/IMU/lidar外参版本、`T_odom_body`、真实P/健康摘要、去畸变lidar-frame点及可用图像/相机标定ID。保留body-frame原点，避免保存只有已漂移world-frame云后无法重新配准。LIO后world云不能随意与后续VIO后状态配对。

短期子图匹配可使用最近若干**已接受且质量足够**的关键帧，不必等全局回环：先用IMU/LIO预测作为初值，与独立历史子图验证当前垂直位移和法向关联；采用完整3D变换，记录overlap、信息量、双向一致性及动态点影响。子图若由同一批错误模型构成仍会自洽错误，不能称独立绝对参考。多楼层同XY、重复走廊/坡道、低overlap应作为明确误匹配负例；颜色/图像只能作为额外验证，不能凭“彩色”推定免疫。

后端消费这些快照，输出包含keyframe双方ID、epoch、相对SE3、信息矩阵、匹配统计/拒绝原因的约束；全局优化输出 `CorrectionSnapshot`（图版本、锚keyframe、有效源时间、`T_map_odom`与校正关键帧poses）。既有活动体素裸指针不离开owner。若需要重新建图，在另一个不可变子图版本中重建后再由owner原子切换，禁止backend并行改活动map或partial更新部分plane。

提交接口需有有限队列和backpressure/失败记录。不能为吞吐省略IMU、刷新旧header或绕过V19正常drain；队列溢出应显式失败而不是悄悄丢关键帧后仍称全图优化。异步结果到达时再次核epoch/ID/source版本，过期约束拒绝。

## map→odom与连续局部控制合同

**候选新帧合同，不是当前已实现的TF。** 当前camera_init同时承担滤波器、SCAN、路线、到达参考；现有文件仅添加一个map→odom TF不会自动让这些消费者正确切换。

- 保留前端连续局部 `odom→body/lidar`；后端唯一发布 `map→odom`，满足 `T_map_body=T_map_odom T_odom_body`。后端全局校正不在局部位姿差分中生成假速度；body twist从同源局部状态/IMU求取。前端自身跳变仍需局部质量检查，双帧结构不能把它自动消除。
- 原路线由冻结SLAM/IMU场景注册生成（`multifloor_demo/slam/heading_alignment.py:27–102`）；这是地图轴/目标定义，不是在线定位。新global frame以首可信关键帧固定锚，路线与区域转换只做一次，有明确注册ID。不得随当前机器人姿态重新注册，也不能用Gazebo真值代替。
- 近程SCAN与Tracker最好在同一个局部odom epoch使用点云、body/lidar pose和局部目标；global map中的目标经同一已确认的correction version转入odom。全局展示/区域判定可在map中，但其位姿、区域和校正必须同版本。完整46区原3D盒、dwell、timeout仍保持原值。
- correction提交时，所有goal、spline、cloud、collision-map、路径投影和到达dwell都必须有版本。旧图与新pose不能混用；若不能原子重投影/重建，就先保护停车，再清旧reference，确认新地图/plan来源后恢复。不能假装回环瞬间的位姿跳跃跨过到达盒就构成原连续dwell。
- 原header/测量时间不因backend重新发布而更新；本机receipt单独记录。局部输入的300ms sim+wall门、IMU配对、ACK、唯一执行器与失联停车不放宽。backend健康不能替代实际fresh SLAM/cloud。
- `odom_adapter.py:79–82`当前用pose差分；若把跳变的map pose直接送进去会产生假linear/angular速度。`shared_controller.py:227,258`和`cascade_core.py:168,372`还会拒绝非camera_init；需要新候选明确接线与负例，不能仅改frame字符串。当前IMU高频预测topic还使用 `world` 字符串（`LIVMapper.cpp:916`），适配时要补实测frame/时间合同，不能直接当现成连续odom接口。

## 有限验证顺序与准入

| 步骤 | 可验证内容与必须保留的限制 |
|---|---|
| 1. 只读观测候选 | 原公式/排序/state/P逐字节基线；新质量摘要对非wall数值无影响。完整19维含bias/gravity/p-v交叉项。冻结诊断窗口、加载库、输入source与storage预算；不继承旧实际PASS。 |
| 2. 故障窗口取证 | 真实相同cloud/IMU/image和地图模型重放，关联ID/plane/H/R、state分担、SCAN失败voxel。旧115–118真实row有限fixture只证明局部Jacobian输入，不是219秒地图/轨迹重放；缺失数据如实标未验证。 |
| 3. 局部质量/子图候选 | 正常地面/坡道/墙、近线支持、不同楼层同XY、动态点、低overlap；验证错误constraint拒绝、有效坡道保留、trial rollback、map不污染与stale停车。阈值事前冻结，不能从单个故障事后挑选后直接生产。 |
| 4. 后端离线图 | 正确闭环、假闭环/错层拒绝、无闭环前缀、优化失败/过期epoch。验证Z和SE3残差及信息量；不把全局残差下降当真实高度精度。 |
| 5. 双帧纯接口 | TF唯一authority，整数header/receipt不刷新；pose/cloud/goal/path统一epoch；回环时无假twist、无假dwell、过期结果拒绝、正常drain/溢出失败。 |
| 6. 独立实际候选 | 先原第11→12区短验证（仍原区域/90秒/停车门），再完整46区、双向坡道/动态停止与恢复。真值仅离线固定SE3验收。确认局部保护与后端各自收益，不能将多项改动组合结果拆称单因素因果。 |

先实现可审阅的快照/质量观测接口，随后以实际故障窗口决定前端可信度规则，再接独立后端与双帧导航。当前接口/既有有限运动测试与此前有限路线通过，不构成本轮完整46区或后端融合通过；实体机器人部署仍未验证。
