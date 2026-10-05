> 当前任务已切换至 [相机传感器直接移动模式](../camera_mode/README.md)。四足控制、步态和强化学习试验按用户要求暂停；本文保留 Go2 历史说明。v9实际为0/2区域、90秒超时失败，试验已停止。

# 当前代码结构

核查日期：2026-10-02。传感器、FAST-LIVO2、三维SCAN、SLAM反馈跟踪、RGB地图、任务与网页通路已经接通；**探索→返航→保存→F1到F3→动态障碍的完整验收尚未通过**。最新完整网页Full19完成18个探索区域及首个返航区域，随后下坡倾角失败，未保存正式地图。

当前生产使用停止归位、行走偏航限幅、原始IMU保护、连续静止初始化、修正后的Gazebo控制时基，以及经组件验证的低D、ALIGN纵向SLAM补偿和CHAMP首摆两行相位修正。原四、八区域及真实动态两区域组件通过，完整流程仍失败，实际命令跟踪有偏差。当前控制是CHAMP与关节PID，未接入机器学习策略，主动机身姿态反馈关闭。新增0.60秒启动包络仅在私有构建中验证，v8下坡1/2后倾角失败，未替换生产；详情见[运动控制修正](MOTION_CONTROL.md)。

## 三个独立入口

| 目录 | ROS 域／网页 | 实际用途 |
|---|---|---|
| `slam5_navigation/` | 55／8765 | 已验证的四轮车 FAST-LIVO2、OctoMap 和二维 Nav2 |
| `scan_multifloor/` | 66／8766 | 官方四层参考点云、平面运动学与 F1→F2 理想样条示例 |
| `multifloor_demo/` | 72／8767 | 新 Go2 物理传感器 SLAM、真实 RGB 地图、任务调度与 SCAN 反馈导航 |

[新闭环网页](http://127.0.0.1:8767/)由本地任务服务提供；[操作命令](../README.md)由总装模块维护。旧 8766 示例的参考 PCD 和理想位姿不能作为新链路的建图或物理运动证据。

## 算法职责与替换边界

旧SLAM5的二维Nav2与新Go2 Demo是不同入口。新Demo的规定跨层路线由任务模块下发，SCAN在`plan_manage/src/planner_manager.cpp`生成三维全局多项式参考；局部规划通过三维占据图、A*辅助路径和B样条优化给出实际可行路径。它支持三维地图和三维规划，当前实体跨层依靠10%坡道，尚无楼梯攀爬验收。

全局路线生成、SCAN局部规划、SLAM路径跟随、CHAMP腿部控制可以分别替换；当前没有算法选择菜单。替换规划器需保持目标身份、参考戳、完整路径、碰撞与可执行性检查，替换运动控制需重新验证实际关节反馈、停车、失联和接触稳定性。机身平面速度与偏航请求不会直接写入Gazebo位置或高度。SLAM位置/航向用于导航纠偏，当前CHAMP步态加关节PID未接入机器学习策略；SLAM里程计反馈也不等同于高频腿部平衡控制。目标使用预声明区域及连续停稳窗口，GT只参与独立验收。

## 新闭环数据流

```text
浏览器 :8767 ↔ mission_server.py ↔ Mission 状态机
                        │ 创建本轮目录、管理进程、分阶段下发路线
                        ▼
Gazebo 三层实体平台/10%缓坡 + Go2 12关节动力学 + CHAMP
  ├─ LiDAR → 固定传感器外参的机身包络过滤 ┐
  ├─ IMU/RGB → 连续静止初始化门控 ────────────────┴→ 隔离 FAST-LIVO2
  │                               ├─ 原始状态 → 几何/速度适配 → SLAM机身与雷达位姿
  │                               ├─ 完整实测云 → 自体过滤 → SCAN局部地图/样条
  │                               │                              │
  │                               │             SLAM反馈/障碍停等/重规划
  │                               │                              │
  │                               │                 命令限制/IMU门控/看门狗 → 停止归位适配 → CHAMP
  │                               └─ 实测RGB云 → 本轮地图快照/保存PCD → 网页
  ├─ 实际仿真RGB相机 → JPEG → 浏览器
  ├─ 可碰撞移动障碍 ← 只在跨层导航阶段启用
  └─ 独立真值/躯干接触/足端接触 → 验收，不输入SLAM或导航
```

这条代码通路已接通，但完整任务尚未通过。场景为高度 **0／1.2／2.4 m** 的三层平台，以 **10%缓坡**连接，非楼梯攀爬。

## 模块与源码

| 模块 | 关键文件 | 当前职责 |
|---|---|---|
| `simulation/` | `prepare.py`、`simulation.launch.py`、`config/` | 从现有Go2 xacro生成独立URDF/SDF，启动CHAMP、effort关节控制、唯一IMU、LiDAR、RGB与接触传感器 |
| `simulation/` | `controller_runtime.py`、`ros2_control_ws/src/controller_manager/` | 独立CM 4.45.2两处仿真时基修正；仅Gazebo子进程选择该动态库，实际库SHA进入运行清单；PID、接口及系统安装独立保留 |
| `simulation/` | `control_bridge.py`、`obstacle_controller.py`、`obstacle_trigger.py` | 机身vx±0.12、vy=0、yaw±0.25与400ms看门狗、独立原始1000Hz IMU停车与失败锁定；障碍仅在同一导航请求的第二个航点启用；只移动障碍物实体，绝不设置机器人XYZ位姿 |
| `simulation/` | `joint_reference_adapter.py`、`joint_stop_core.py`、`execution_safety.py` | 默认启用的停止归位；零机身命令立即送达CHAMP，关节目标回真实静止标定的名义站姿；行走目标原样透传，重复零不重启，归位时新非零请求排队。桥合并初始化/故障/失联状态，正常归位不触发倾角hold |
| `simulation/test_results/active_body_feedback_staging/` | `feedback_core.py`、`body_stabilizer_node.py`、离线测试/回放 | 真实IMU/关节/接触的roll/pitch反馈候选，仅excluded staging、生产禁用；隔离ROS契约通过，首8点物理对照失败并保留证据，正在修复跨话题时间配对 |
| `simulation/` | `scenario.json`、`scenario_smoke.json` | 18点三层探索、14点返航、14点再次F1→F3；relative_world_axes方向由固定heading转成SLAM目标，短程探索单独保留 |
| `simulation/` | `probe_physics.py`、`probe_turns.py`、`probe_turn_then_drive.py`、`probe_heading_gate.py` | 平地、缓坡、正反旋转、转后前进和生产HeadingGate多段切换的独立物理测试 |
| `simulation/` | `probe_sensors.py`、`probe_contacts.py`、`audit_scenario_geometry.py`、`record_execution.py` | 传感器与RGB、接触正负例、名义路线几何、只读命令/实测速度记录 |
| `slam/` | `ros2_ws/src/fast_livo2_core`、`ros2_ws/src/fast_livo2_ros` | Demo 独立FAST-LIVO2源码；至少300条且跨度≥2.9仿真秒的连续稳定IMU重力初始化，可选IMU覆盖同步与诊断记录，旧SLAM5源码保持独立 |
| `slam/` | `launch.py`、`fastlivo.yaml`、`camera.yaml`、`self_echo_filter.py` | 校验隔离包路径；原始雷达通过固定外参移除已知机身包络，保留实际字段与时间，不依赖SLAM或真值 |
| `slam/` | `geometry.py`、`odom_adapter.py` | 正确转换IMU/机身/雷达参考点，用传感器时间计算child-frame速度，拒绝非法/倒退数据 |
| `slam/` | `map_archive.py`、`audit_live.py` | 只累积本轮相机RGB云，8cm体素去重、不可变快照、原子存档与保存服务；只读SLAM精度诊断 |
| `navigation/` | `scan.launch.py`、`controller.py`、`control_core.py`、`trajectory_contract.py` | 只启动SCAN核心；速度语义转换、自体回波过滤、当前航点与完整样条配对、真实样条反馈执行、到点/倾角/失联门控、障碍停等及重规划 |
| `navigation/` | `event_archive.py` | 对实际障碍门控所用的不可变点云和同刻姿态异步存档；先发零速再写文件，记录请求、目标、时间戳与文件SHA |
| `navigation/` | `ros2_ws/src/{plan_env,path_searching,bspline_opt,plan_manage}` | Demo独立SCAN四包；`plan_manage`对应包名`scan_planner`。`plan_env`修复连续射线遍历、线段终止与占据更新；规划初始位置保留实测值，速度采用限幅测量值/停车参考边界，未观测加速度置零；依赖重新链接 |
| `navigation/` | `test_control_core.py`、`test_ros_contract.py`、`test_scan_interface.py`、`inspect_live_cloud.py` | 数值与ROS契约、真实SCAN节点接口、实际点云诊断 |
| `navigation/` | `tests/test_ray_coverage.py`、`tests/test_grid_map_edges.py`、`ros2_ws/src/plan_env/test/raycast_regression.cpp` | 独立连续线段几何对照、实际GridMap共享端点/命中优先/截断/跨帧/滑动回归与稀疏实测云夹具 |
| `mission/` | `route_regions.py` | 启动前生成并校验v2到达区域及内部停稳范围，保留v1规则；保持SCAN中心不变，中心和区域轴用同一次传感器heading变换；返航原点仍为0.22m球域 |
| `navigation/` | `goal_regions.py` | 共享不可变区域几何/定义SHA、实际整数里程计时间的0.4s停留；重复、倒退、超过0.2s缺口、保护和出域重置 |
| `mission/` | `state_machine.py` | 按真实反馈推进等待→探索→返航→保存→导航；无到点/保存/动态响应证据则失败 |
| `mission/` | `artifacts.py`、`processes.py` | 保存源码/运行二进制清单；对本轮拥有的进程组进行有界停止清理 |
| `scripts/` | `mission_server.py`、`wait_sensors.py`、`stack.launch.py`、`run.sh` | 本地HTTP/ROS服务、相机、地图和进程；10s预热后等连续3s静止IMU和零指令执行桥，再装配SLAM/导航 |
| `scripts/` | `sensor_startup_gate.py`、`record_sensors.py` | 启动静稳策略及原始IMU/传感器时间证据；审计IMU缓冲depth=2000，独立安全IMU仍depth=1；必需记录器异常退出停止本轮 |
| `navigation/` | `record_feedback.py` | 从仿真启动前只读记录请求/安全速度、SLAM/真值、稀疏实测点云及同刻雷达原点 |
| `scripts/` | `evaluate_run.py` | 按运行前声明的点/区域合同与原始NAV收据逐目标验收；同一窗口SLAM与独立真值均须满足对应界限，同时检查地图、接触、倾角及障碍，缺证据失败 |
| `web/` | `index.html`、`architecture.html` | 本轮真实RGB点云、执行/规划路线、仿真相机、阶段、独立验收与架构；不加载参考PCD冒充本轮地图 |
| `tests/` | `test_mission.py`、`test_processes.py`、`test_sensor_audit_ros.py`、`test_heading_history.py`、`test_startup_gate*.py` | 状态迁移、失败门控、动态响应、进程清理、真实记录器传输与启动静稳测试 |

CHAMP 当前由速度和机身请求生成足端/关节轨迹，**没有IMU平衡反馈或复杂地形落足规划**。当前纯转和缓坡短测不代表任意复合运动稳定。导航纯转yaw上限0.12rad/s、行走修正上限0.08rad/s，采用先对齐后前进、侧移恒零、稳定窗口与倾角保护；这些策略的完整物理任务仍须验收。

`scripts/run.sh build` 先编译隔离CM一包 `controller_manager`，再编译隔离SLAM两包 `fast_livo2_core/fast_livo2_ros`、SCAN四包 `plan_env/path_searching/bspline_opt/scan_planner`。正常启动先加载本机ROS、Go2、旧SLAM5依赖与SCAN，再依次加载两个Demo的`install/local_setup.bash`。CM不全局加载overlay：`controller_runtime.py`只为Gazebo子进程前置其动态库目录，系统spawner和其他ROS节点仍沿原路径；运行清单直接记录所选CM文件。若未构建该文件，网页启动返回明确构建提示。当前SLAM配置`common.require_complete_imu=false`、`debug.sensor_sync_diagnostics=true`。职责分工和未完成项见[开发计划](DEVELOPMENT_PLAN.md)。

SCAN 修复让共享端点及重复穿越仅跳过本帧重复计数，继续完整射线；DDA采用真实连续方向，只有实际线段在体素中穿过正长度才记空闲，裁剪端点同样检查。真实端点命中优先，每帧重置临时标记避免char回绕；未观测/遮挡区域不会因时间流逝清空。没有引入真值、TTL或缩小障碍膨胀。离线测试通过只证明这些地图语义，不能替代完整仿真验收。

Gazebo、CHAMP、执行桥、默认停止适配器及导航是必需执行进程，退出会关闭整栈；适配器未完成名义站姿标定时启动门不放行，建立健康后失联或失败锁定停车，并使活动任务失败。默认话题在 `/demo/control`；历史 `DEMO_TEST_JOINT_STOP_ADAPTER=1` 对照仍使用 `/demo/test`，旧记录不改写。

网页停止会先停导航，再异步结束本轮拥有的进程组。`process_running=false` 且 `stopping=false` 才表示清理完成；网页服务继续保留，服务终端 Ctrl+C 结束服务。正常完成/失败会自动触发离线验收；阶段标签与独立验收结果分别保存。

## 接口和坐标

| 接口 | 语义 |
|---|---|
| `/livox/lidar`、`/livox/imu`、`/camera/image_color` | 本轮真实Gazebo传感器 |
| `/demo/slam/lidar_filtered`、`/demo/slam/lidar_filter_status` | 用固定sensor→body外参过滤机身包络的真实雷达输入与过滤诊断 |
| `/cloud_registered_full`／`/cloud_registered` | 完整去畸变世界云／相机视野内实测RGB云 |
| `/demo/slam/body_odom`、`/demo/slam/lidar_odom` | 正确参考点、时间与速度语义的SLAM状态 |
| `/demo/navigation/request`、`/demo/navigation/status` | 阶段路线请求与实测完成反馈 |
| `/demo/navigation/trajectory_metadata` | 实际规划输出的不可变参考戳、原始目标、完整样条载荷和起始状态；与B-spline双消息匹配后才执行 |
| `/demo/navigation/cloud`、`/demo/navigation/bspline` | 已知自体体积过滤后的SCAN输入／实际SCAN输出 |
| `/demo/cmd_vel` | 机身平面执行请求，Z始终不驱动机身位姿 |
| `/demo/control/safe_cmd_vel`、`/demo/control/safety` | 经过IMU/超时/适配器健康门控的请求和桥状态；这是适配器上游请求，并不等于实际CHAMP输入 |
| `/demo/control/actuator_cmd_vel` | 停止适配器实际送入CHAMP的机身命令；归位期间保持零 |
| `/demo/control/joint_reference/raw`、`/demo/control/joint_stop_safety` | CHAMP原始关节目标／适配器标定、归位与故障状态；经过适配器的目标再送实际JTC |
| `/demo/slam/map_status`、`/demo/slam/save_map` | 本轮地图状态／保存服务 |
| `/demo/mission/state` | 阶段、路线、失败原因和证据状态 |
| `/demo/obstacle/enable`、`/demo/obstacle/state` | 独立障碍启用和实际服务确认位置 |
| `/demo/ground_truth`、`/demo/body_contacts` | 只供独立验收，未作为SLAM或规划输入 |
| `/demo/contacts/{lf,rf,lh,rh}_foot` | 足端接触单独记录，地板/坡道支撑合法 |
| HTTP `/api/state`、`/api/acceptance`、`/api/camera.jpg`、`/api/map/…`、`/api/mission` | 网页/清理状态、独立验收、图像、本轮地图以及限定启动/停止动作 |

当前Go2机身与IMU同原点；雷达偏移 `[.2,0,.1177]` m，相机偏移 `[.28,0,.1]` m，光学RPY `[-π/2,0,-π/2]`。`camera_init` 为本轮SLAM初始化参考；航点为world方向的相对位移，通过静态IMU参考与初始SLAM配对冻结heading后转到camera_init，并锚定初始机身位置；不能直接把其z当作Gazebo楼面高度。生成源与新资产显式使用1000Hz IMU及CUSTOM/world identity，旧run快照不自动变化。新适配器不沿用旧四轮TF的5cm参考点语义差异。

## 证据与来源

`runs/<run_id>/` 保存 `mission.json`、`scenario.json`、`pose_audit.jsonl`、`sensor_audit.jsonl`、`feedback_navigation.jsonl`及稀疏云、`feedback_navigation_trajectories.jsonl`、本轮 `fastlivo_debug/` 连续MAT/IMU诊断、`startup_gate.json`、`map_metadata.json`、`colored_map.<revision>.bin`、`stack.log`。实际保存成功后才有 `colored_map.pcd`，评估后生成 `acceptance.json`。已完成模块分测，集成失败记录仍保留；最终以本轮完整验收为准。

新运行同时保存 `source_manifest.json`、`source_snapshot.tar.gz` 与 `runtime_manifest.json`：前两项记录Demo模块源码/配置（含CM源码、补丁来源、LICENSE）和逐文件SHA256，后一项记录FAST-LIVO2、SCAN、CHAMP二进制及CHAMP、所选CM、控制接口、control_toolbox PID、JTC、JSB和gz_ros2_control动态库，以及`plan_env/path_searching/bspline_opt`三个静态库的绝对路径与SHA256，并记录ROS域/Gazebo分区和声明的控制时基配置。实际进程映射和控制参数另行核查，不把声明当作实际观测。源码快照排除文档及test_results，不包含整个外部依赖或操作系统环境；旧run缺少新增字段时保持历史原貌。

`acceptance.json` 的 `ordered_waypoints` 记录探索、返航、再上楼的逐个到达证据。旧点目标运行保持原同刻SLAM与插值真值≤0.30m验收。新区域运行使用预声明几何和原始NAV到达收据，同一连续0.4仿真秒内SLAM须在预声明内部停稳范围，插值真值须在原外部区域（v1仍要求两者都在外部区域），记录时刻严格递增且缺口不超过0.2s；所有活动期SLAM必须有真值覆盖。全程固定同一个初始SE(3)，不通过阶段重对齐或事后扩大区域补造通过。`navigation_audit.jsonl`保存每次原始状态回调，`pose_audit.jsonl`保存原header整数纳秒，避免阶段立即切换漏掉收据。

- [仿真模块和分测](../simulation/README.md)：单源IMU、RGB、平地/缓坡、左右纯转、转后正向前进、四段转前停稳单变量对照、contact正负例。
- [SLAM模块和分测](../slam/README.md)：参考点、时序、RGB、存档和真实ROS接口；合成接口输入不代表SLAM精度通过。
- [导航模块和分测](../navigation/README.md)：几何策略、ROS契约、真实SCAN节点接口；当前避障实现为停等通过后重规划，不宣称任意动态障碍主动绕行。
- 旧稳定SLAM5源码：`slam5_navigation/ros2_ws/src/{tb3_fastlivo,fast_livo2_core,fast_livo2_ros,fast_livo2_nav}`。
- Demo SCAN核心：`navigation/ros2_ws/src/`，源自保留的`scan_multifloor/ros2_ws/src/planner/`；Go2/CHAMP来源：`go2_sim_control/src/unitree_go2_ros2/`。

`build/`、`install/`、`log/` 是生成物；上游原始副本和旧演示保持独立。

## 10月1日组件证据与未验证边界

平地停车、短坡和四转用相同传感器、原步态与实际SLAM/SCAN负载完成同源对照；[平地停车报告](../simulation/test_results/20261001_stop_adapter_ab.json)记录的是理想目标、实际JTC参考、实测关节速度和JTC输出effort各自的指标，不能把理想曲线3rad/s上限当作实际关节全局上限。JTC输出effort也不是独立测得的施加力矩。

[动态两点同步验收](../slam/test_results/oct1_dynamic_strength_synchronized_acceptance.json)确认0.08候选真实到达、真实障碍停车期间零命令与新SCAN恢复；原driver因4Hz状态滞后给出的FAIL仍保留原SHA，纠正只在独立同步审计中说明。[严格单坡](../navigation/test_results/slope_strength_008_actual/slope_result.json)45.42仿真秒完成，12项检查通过。这些是局部组件结果，不替代三层18点探索、14点返航和14点再次上楼。

[记录器缓冲反例](../test_results/20261001_sensor_audit_buffer/result.json)使用实际ROS的500条输入及80ms接收进程暂停：旧depth5收到424条，新depth2000收到500条。它证明本次传输/记录场景，不能承诺任何负载下不丢数据。

[姿态反馈候选](../simulation/test_results/active_body_feedback_staging/README.md)的19项数学、10项实际方法AST接口、2196个实际CHAMP头文件几何样本及隔离ROS消息契约通过。首8点真实物理对照已执行：关闭反馈的A实际到达8/8，但出现一次动作期倾斜保护和6条独立真值配对缺口，严格结果FAIL；开启反馈的B在启动期因传感器时间新鲜度判定锁定失败，未产生运动请求，不能评价平衡效果。原始结果及进程清理证据保留在[基线A](../simulation/test_results/20261001_balance_first8_a_disabled_v2/first_eight_result.json)和[候选B](../simulation/test_results/20261001_balance_first8_b_enabled/first_eight_result.json)。新被动记录器已采集同步IMU、关节与真实足端接触；离线首60秒可拟合支撑面，但不代表live depth1消费齐全，足速估计仍依赖无滑动假设，不能用作已验证速度反馈。当前生产仍无主动姿态平衡；该候选未接入生产且不是WBC。

[第十四轮实际执行分解](../simulation/test_results/20261001_full14_physics_audit.json)按真实adapter CHAMP命令零阶保持分类，用覆盖间隙不超过0.15秒的独立GT只作评估：第8点行走55.332秒/纯转14.847秒/零速20.072秒，world X贡献分别+2.551/−1.938/−0.016m。归位中非零实际CHAMP命令为0，故不能将归位接口成功当作坡上稳定控制成功。

## 新区域与实测关节验证进展

用户提出将目标设为区域后，完整场景新增`route_goals`：平地半径0.35m、高度±0.10m；坡道沿坡/横向/法向半尺寸0.35/0.30/0.10m；返航原点半径0.22m，连续停留0.4仿真秒，单目标超时90仿真秒。各次新运行冻结完整定义与SHA，46个区域的保守静态包络检查通过；[任务检查](../tests/test_mission_regions.py)包含错楼层、改变定义、乱序、保护与不足停留反例。网页绘制实际请求中的到达体积。此改变尚无整条物理路线通过结论。

当前Go2 Demo采用CHAMP步态与关节PID，未接入机器学习策略。姿态候选第二组A到达8/8但原始倾角触发两次保护，B因实际关节发布停顿启动失败。发布调用观测确定约330ms停顿位于原`rcl_publish`调用内，未确认DDS具体内部原因。新候选使用Gazebo实体关节传感器的实际q/qd及物理时间，经独立BEST_EFFORT桥送入反馈；不含世界位姿控制。其[40sim无运动检查](../simulation/test_results/20261001_balance_measured_startup_v1/measured_joint_audit.json)通过，10–39.9sim的29901条传感器header间隔均为1ms，3005个实际反馈配对可核对原始关节/IMU，原30ms/5ms门限不变。本轮未重现旧发布长停顿，因此不宣称已覆盖重复停顿；行走、坡道和动态障碍的反馈效果仍待验证。

当前v2保持上述外部验收区域不变，控制停车/停留使用更小范围：平地半径0.25m、高度±0.10m；坡道沿坡/横向/法向0.25/0.20/0.07m；原点半径0.17m。`arrival.control_band`与完整区域一起进入定义SHA，内部范围共用中心、朝向轴、0.4s停留和90s超时。第15轮原始失败及v1定义不修改；内范围仅对后续运行生效。

## 仿真控制时基修正与实际运动验证

第16轮 `20261001_212305_774068` 在启动期失败，所有已记录的机身速度指令为零，尚未建立SLAM原点或执行路线。随后被动诊断记录到：Gazebo物理更新继续进行时，控制器收到的ROS时间可能重复。实际ControllerManager同步回调测试确认，原4.45.2库在这一条件下传入零时长，候选使用Gazebo调用方的物理时间后保持4ms。关节PID在零时长时沿用旧输出，已由原始源码和实际反馈重放交叉验证；这还不能证明第16轮摔倒只有这一原因。

候选原始实验位于`navigation/test_results/cm_time_contract_staging/`，正式源码已复制至`simulation/ros2_control_ws/src/controller_manager/`，仅修改`use_sim_time=true`时的同步控制时间来源。最终路径重建的库为`18b0ed045399…`；[最终构建与原生接口收据](../navigation/test_results/cm_time_contract_staging/final_adoption_v2/receipt.json)确认12项实际检查及公开ABI保持。原步态、PID、轨迹插值、关节接口和非仿真分支不变；不支持运行中重置世界时间。被动原生控制周期观测器和站立对照工具仍留在excluded实验目录。

[A3/B3独立站立审查](../slam/test_results/oct1_CM_standing_A3_B3_independent_ready.json)确认两组均40sim结束、416个非CM动态库路径/SHA相同、进程全清。原JTC149次零周期、最大336ms，候选JTC+JSB所有20016次实际回调均4ms。A3的CDR记录器缺3条IMU观察保留原貌，另一官方传感器记录完整；B3的29901条10–39.9sim IMU全为连续1ms。候选短暂关节波动仍比基线大，不能宣称全面物理稳定改善或原摔倒唯一根因已解决。当前默认接入隔离修正。随后四转/停车8项、坡道15项、实际动态障碍两点22项及独立审查通过，见[运动汇总](../simulation/test_results/20261001_final_cm_motion_suite_summary.json)。这些结果不代替完整路线。

第17轮 `20261001_232612_ad2317` 已由真实网页启动，20.4墙钟秒时因适配器状态超时停止；未建立SLAM原点、无路线或地图保存。适配器实际日志有337.237ms回调空窗，原健康超时为300ms（导航指令看门狗的400ms是另一合同）；同期IMU连续，具体阻塞调用尚未唯一定位。原[完整验收FAIL](../runs/20261001_232612_ad2317/acceptance.json)和[来源/运行时未变、进程清理证据](../runs/20261001_232612_ad2317/root_terminal_provenance.json)保留。下一步以不改变控制、QoS与门限的有界调用边界观测定位停顿，然后重跑网页任务。只读原生观测器记录真实ControllerInterface入参和完整返回值；JTC输出effort表示命令力矩，不能当作独立测得的施加力矩。单个组件通过不代表三层探索、返航、地图保存与再次导航通过。

## 网页运行的可选被动调用历史

`simulation/control_timing_trace.py` 与 `control_timing_runner.py` 仅在启动子进程的 `DEMO_CONTROL_TIMING_AUDIT=1` 时包裹原Adapter/Bridge。组件默认关闭；网页协调器显式启用并在本轮runtime manifest保存声明。每个节点固定保留最多500000条最近callback/publish/log.write/executor-wait边界，累计数量、覆盖序号、环覆盖次数和首次失败另外记录；它不是全程采集，也不能把一次发布的墙钟耗时全部解释为DDS内部耗时。

捕获路径只写内存，无额外订阅或控制动作，退出后保存本轮`adapter_boundaries.*`/`bridge_boundaries.*`。诊断写入或保存失败只使诊断失效，不改变原控制的消息、返回或异常；实际Adapter与Bridge实现保持第17轮原SHA。网页入口接线的真实ROS组件检查见[结果](../test_results/full18_gateway_contract/result.json)，该检查截获子进程创建，没有运行Gazebo或路线。整套网页仍待正式验证。

## 第18轮实际网页结果

`20261002_001559_882ddb` 在第四目标前的持续纯转中真实侧倾并停止。前三个region按同一初始SE3、同一原NAV保持窗联合通过；定位最大误差8.45mm，实际RGB142199点，但未保存正式PCD。原控制0.30/0.50 Euler保护及时执行；Adapter状态新鲜，最近50万条环内没有相应长发布停顿。完整验收为[FAIL](../runs/20261002_001559_882ddb/acceptance.json)。

当前生产仍只有导航SLAM位置/航向反馈、关节PID和IMU安全保护；基于IMU/真实关节/足端接触的支撑面姿态PD候选留在excluded实验目录，尚未证明其运动效果，不能称为已接入机器学习或全身力矩控制。同源前四区域A/B单独验证该候选，导航锁向几何修正另行验证，不混合两个变量。

2026-10-02前四区域反馈A/B已实际执行：关闭反馈A四个原窗口联合通过（运动期原始Euler倾角峰0.19326rad、零倾角暂停）；开启反馈B在首目标行走期50.170sim触发0.50rad，未完成任何区域。两组保持当前生产269文件、最终CM和相同测量链，仅启用开关不同，进程已清理。B当时输入仍新鲜，因此不是此前启动失联的重复；该候选保持禁用。原始关节/JTC/接触与姿态输出进一步审计用于寻找实际控制原因，不用导航到点替代平衡验证。

纯转漂移纠偏候选位于`navigation/test_results/turn_drift_staging/`：`nav_drift_controller.py`继承原`Navigation`，`turn_drift.py`管理原始SLAM漂移检测和停车交接。纯转平移超过0.15m、至少3条递增观测持续0.2sim后发布精确零速；实际Adapter确认归位、保持1sim静止后请求新SCAN参考，原轨迹身份、完整样条和规划检查通过才恢复。原目标区域、单段90sim期限和IMU保护保持，GT不参与控制，不加入速度PI补偿。

该候选[实际前四区域](../simulation/test_results/20261002_nav_drift_first4_disabled/first_four_result.json)20项通过，4次漂移中断、倾角峰0.218219rad且无倾角暂停；[独立验收](../slam/test_results/oct2_NAV_turn_drift_first4_actual_independent.json)确认4个原始窗口及984/984条活动期真值覆盖，[独立交接](../slam/test_results/oct2_NAV_turn_drift_actual_handoff_independent.json)确认实际零命令、归位、静止和新完整SCAN路径。随后[原前八区域](../simulation/test_results/20261002_nav_drift_first8_disabled/first_eight_result.json)实际到达8/8，但倾角峰0.361907rad、一次保护暂停使严格稳定性FAIL；首次保护位于第5个平层目标的前进行走，非第8坡道或新增纯转交接。该时刻关节/JTC/接触不在CDR保留后缀内，不能补造因果证据。候选尚未进入生产入口；关节轨迹衔接、实际移动障碍交接及完整流程仍需验证，主动姿态反馈保持关闭。


## 关节参考衔接对照与下一项限幅验证

`simulation/test_results/jtc_desired_first8_staging/`为隔离入口，真实ROS参数服务在原连续静止门控后、SLAM/NAV之前读取JTC BOOL；同源A/B仅改变`interpolate_from_desired_state`，原P100/I0.2/D1和区域、90sim期限保持。A=false实际7/8，第8目标超时，倾角峰0.22909rad；B=true实际0/8，首目标超时，倾角峰0.17190rad。B机身在实际正前进命令下前向响应接近零，定位RMSE5.38mm不能证明运动执行正确。两轮均清理完成，true未采纳。[实际报告](../simulation/test_results/jtc_desired_first8_audit/REPORT.md)与[独立审查](../slam/test_results/oct2_JTC_desired_first8_B_true_actual_independent.json)保留失败与原消息缺口。

当前模型关节力矩限值hip/upper为23.7Nm、lower为35.55Nm；实际CM启动日志确认总命令限幅关闭。JTC的PID输出限幅只限制PID项，参考effort与速度前馈仍会叠加，不能用其代替总命令限幅。下一项只验证CM `enforce_command_limits=true`，保持JTC=false、原PID、f592/1019导航候选及原安全/区域合同。隔离ResourceManager原生合同需使用真实12关节URDF及合法站姿状态，检查PID+前馈后的最终命令接口在hardware write前是否限幅，并核对实际开关、控制周期及库SHA。此时尚无该候选实际物理结果，生产269文件继续冻结。


总命令限幅的[实际A/B](../simulation/test_results/cm_limits_first8_audit/REPORT.md)已经完成且均FAIL：A关闭6/8、第7目标90sim超时，B开启5/8、第6目标90sim超时；原始倾角峰0.29651/0.25198rad，无倾角暂停。实际参数、源码、运行库与进程清理全部核对，[独立证据](../slam/test_results/oct2_CM_limits_first8_A_B_actual_independent_final.json)确认原到达窗与活动期真值全覆盖。CM开启时安装版`LoanedCommandInterface.set_value`会经过限幅setter，JTC状态输出可以已受限幅；它仍不是独立测得的硬件施加力矩。原失败和审查文案修正前版本均保留。

新的控制profile已完成一次实际启动试验：JTC期望状态连续参考、P220.982919/D3.360637（来自真实5ms目标输入、4ms关节控制的受控原生响应测量），保留I0.2/i_clamp2.5/FF0及CM总命令限幅。目标是避免旧effort回灌并恢复受控测量中的反馈响应，不是已证明的Go2动力学等效；多变量profile试验不能据此声称唯一根因。实际启动时目标恒定，但LF小腿qdot和输出命令每4ms正反交替，约125Hz采样级振荡，原150秒静稳门超时；未进入SLAM/NAV或实际参数回读门。该profile不采纳。原区域、停稳、期限及保护保持。当前只读核对实际URDF惯量与离散控制模型，机械原帧见`simulation/test_results/classic_pd_startup_audit/REPORT.md`。
