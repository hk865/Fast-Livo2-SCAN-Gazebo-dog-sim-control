# SLAM 反馈导航与动态障碍停车恢复

2026-10-02 当前状态：标准执行链已整合，网页第 19 轮完成原 18 个探索区域和首个返航区域，但返航第二个区域因严重倾角保护失败，完整跨层 Demo 尚未通过。此前完整运行的失败记录保留。整合前的私有 CHAMP 首次摆腿相位修正、低 D 关节控制与 SLAM 对齐补偿组合，已通过原前 8 区域和两点动态障碍实际组件检查；这些组件结果不代替探索、返航、跨层导航、地图保存与再次导航的完整验收。

标准 stack 的导航入口为 `align_controller.py`。它保留实测 SLAM **位置和航向**路线闭环，并只在 `ALIGN` 阶段增加原始 SLAM 纵向 P+D 补偿；普通 `DRIVE` 的速度控制与上限保持不变。独立 body-frame 运动观察器仍仅作诊断。补偿请求并不等于机身实际速度，组件审计仍观察到对齐中的后退和命令响应差异。边界、标准入口与证据见 [DEVELOPMENT.md](DEVELOPMENT.md)；下列 `run.sh controller` 启动的是基础控制器，不能替代完整 stack 的执行链。

此模块不启动 SCAN 原有的理想 XYZ 控制器、平地运动学模拟器或参考 PCD 感知模拟器。`scan.launch.py` 仅启动官方 SCAN 规划核心，数据来自本次传感器 SLAM。

```sh
bash multifloor_demo/navigation/run.sh planner
bash multifloor_demo/navigation/run.sh controller
bash multifloor_demo/navigation/run.sh test
```

## 任务接口

发布 `/demo/navigation/request`，类型 `std_msgs/msg/String`，内容为 JSON：

```json
{"request_id":"explore-001","frame_id":"camera_init","waypoints":[[1.0,0.0,0.0],[2.0,0.0,0.0]]}
```

航点是 **机身 base 中心** 在 `camera_init` 下的实际目标，z 为相对初始化机身位置的高程。不能将 Gazebo world 路线仅改 frame 字符串后发送。相同 `request_id` 重发是幂等查询，运行中拒绝其他新请求；先停止再启动另一任务。单次最多 500 个航点。

| 方向 | 话题 / 类型 | 语义 |
|---|---|---|
| 输入 | `/demo/slam/body_odom` / Odometry | 真实 SLAM 机身位姿，twist 在 child frame |
| 输入 | `/demo/slam/lidar_odom` / Odometry | 与世界云对应的 SLAM 雷达位姿，直接给 SCAN |
| 输入 | `/livox/imu` / Imu | 1000 Hz 原始姿态，只做倾角安全保护；回调只消费最新有效帧 |
| 输入 | `/demo/control/safety` / String | 执行桥独立保护的 hold / ready / failed 状态 |
| 输入 | `/cloud_registered_full` / PointCloud2 | 全视场实测去畸变云，在 camera_init |
| 输入 | `/demo/navigation/request` / String | 上述任务请求 |
| 输入 | `/demo/navigation/stop` / Bool | true 立即停止 |
| 输出 | `/demo/cmd_vel` / Twist | 机身平面 vx/vy 和 yaw，20 Hz；z 总为零 |
| 输出 | `/demo/navigation/status` / String | JSON，4 Hz，transient-local |
| 输出 | `/demo/navigation/path` / Path | 实际 SCAN B 样条采样路径，用于可视化 |
| SCAN → 导航 | `/demo/navigation/trajectory_metadata` / String | 已成功参考请求的不可变时间戳、原始机身目标、完整样条及起始状态诊断 |

状态字段为 `request_id, state, waypoint_index, total, message, obstacle_hold, obstacle_stops, obstacle_resumes, min_obstacle_clearance, replans, reference_requests, pose_age, cloud_age, pose, command, counts`。状态为 `idle/running/succeeded/failed/stopped`。`waypoint_index` 为已经完成的航点数，因此成功时等于 `total`。`last_rejected` 给出最近一次无效或冲突请求原因，不覆盖正在运行任务。

## 几何语义和执行策略

- SCAN 上游读取 odom 的 twist 时假设世界速度；本模块显式把标准机身 twist 旋转后转发 `/demo/navigation/scan_body_odom`，保留原始 SLAM 输出不动。
- SCAN `REFERENCE_PATH` 会给输入路线 z 加 `grid_map.body_height`；适配器将该内部偏移减掉，确保请求仍表示机身目标，避免凭空增加 0.4 m。
- 每段航点必须得到实际 SCAN 轨迹才可运动。跟随器用测得的 SLAM 位姿投影到轨迹，再取前视点；不会以样条时间结束判断到达。旧 `waypoints` 请求保持三维距离 <0.22 m 并稳定 0.4 秒；新 `goals` 请求按下文预声明区域和内侧停车范围判断。
- SCAN 在规划失败时可能发送所有控制点相同的停车样条。目标未到达时，路径弧长不足 0.05 m 或几何跨度不足 0.025 m 的样条会立即清除此前运动并停车，状态给出 `degenerate_splines` 与 `last_spline_rejected` 原因；持续重新请求有效规划，不将停车轨迹解释成已到达。
- 前进指令最高 0.12 m/s，侧移恒为零；纯转最高 0.12 rad/s，行走偏航校正增益为 0.5、最高 0.08 rad/s。任何进入对齐前都先 `pre_turn` 精确零速停稳 1 秒，避免身体仍有行进动量时开始转。随后锁存有效 SCAN 前视航向，一次转到误差 <0.10 rad 后再次精确零速并固定停稳 1 秒，不追随中途新样条微调。结束时余转误差 ≤0.20 rad 才放行，否则重新进入制动/对齐流程。前视距离为 0.8 m。行走中的方向偏差超过 0.20 rad 并持续 0.5 个仿真秒才重新制动/对齐，达到 0.55 rad 则立即制动。仅选取前视位置和计算路线方向使用时间常数 0.4 个仿真秒的实测 SLAM 位置低通，以免近航点的步态/定位短暂侧摆不断打断起步；到达半径、障碍、姿态保护均继续使用未经低通的当前 SLAM 位姿。线加速度限制为 0.15 m/s²，角加速度为已在物理旋转测试验证的 0.25 rad/s²；障碍/失联停车立即置零。高度只是规划和到达条件，爬坡由接触物理、执行控制器决定，导航不写入机身 Z。
- Go2 躯干实际高度 0.114 m，SCAN 垂直占据膨胀上下均为 0.12 m；在 0.08 m 网格中会向上取整到 0.16 m。避免把原先 0.18 m 取整到 0.24 m 后，地面 −0.28 m 单元与机身接近 z=0 的 −0.04 m 单元错误连接。此处按躯干几何加余量，平面双圆柱不缩小。
- 位姿超过 0.8 秒或非空点云超过 1.0 秒没有新时间戳，立即发零速；连续 8 秒失联任务失败。传感器重复旧时间戳不算恢复。
- 加速度、对齐/姿态恢复/到点/障碍清空稳定窗口均使用 ROS 仿真时间，使低实时率时物理限制保持一致；只有传感器失联与接口重试间隔使用单调墙钟。
- 倾角保护同时使用 SLAM 姿态和 1000 Hz 原始 IMU。IMU 用 `simulation/scenario.json` 的固定参考旋转和机身到 IMU 外参转换为机身姿态，仅用于安全停车；不会修正 SLAM 位姿、路线或航向。任一路横滚/俯仰绝对值达到 0.30 rad 即停车，达到 0.50 rad 任务失败；两路都低于 0.18 rad 并稳定 0.8 个仿真秒后重规划恢复。IMU 回调直接发布精确零速，不等待 20 Hz 控制 tick。IMU 只接受有效 orientation 和严格递增的时间戳；墙钟收包年龄和 ROS 时间戳年龄均不得超过 0.25 秒，超时停车，旧/重复帧不算恢复。执行桥的独立 hold 阻止恢复，其 failed 锁存直接传递为导航任务失败，防止执行器已经停车而任务继续运行。状态报告 `tilt_source, raw_imu_age, raw_imu_stamp_age, raw_imu_tilt_rad, raw_imu_max_tilt_rad, raw_imu_last_rejected, execution_bridge_safety`。所有保护均不订阅真值，独立物理真值验收仍由总装完成。
- 实际传感器会看到 Go2 机身、髋部和腿。控制器按邻近时刻（差值不超过 150 ms）的 SLAM 姿态转为机身局部坐标，去掉机身/髋部 `|x|<0.30, |y|<0.23, -0.10<z<0.20` 与腿部 `|x|<0.45, |y|<0.28, -0.36<z≤-0.07` 两个已占体积的回波，再发布 `/demo/navigation/cloud` 给 SCAN。原始 SLAM 点云保持不变；没有使用大半径盲区删除外部障碍。
- 实测点云进入前方 0.95 m、宽 0.84 m 的机身通道即停车。参考路段高程用于排除坡道支撑地面；高于局部机身中心 −0.12 m、低于 +0.60 m 的点参与障碍检测。连续 1 秒清空后重新调用 SCAN，再继续。
- 当前避障交付是 **停等移动障碍通过后重规划恢复**；不宣称任意动态障碍主动绕行，也不从障碍物模型真值决定停车。

`test_control_core.py` 验证几何/接口策略。传感器断流、物理爬坡和动态障碍端到端证据必须来自集成运行，纯单元测试不能替代。

## 本模块已执行的验证

2026-09-22：16 项纯几何/状态测试全部通过；隔离 ROS 域 75 中，控制器接口/停车/恢复检查的实际结果为 `ros_contract_result.json`。合成输入只用于组件测试，没有用于声称物理导航或 SLAM 通过。

真实集成首次诊断发现机身自体回波使 SCAN 起点被占据；`live_cloud_diagnostic.json` 与 `.npy` 文件保存实际订阅证据。实际 15066 点中的 598 个自体回波经上述包络去除，包络外点保持不变，近机身高于地面 −0.20 m 的异常点降为 0。该修正之后重跑全部 12 项 ROS 控制器接口检查，仍全部通过。

第三轮真实运行通过前三个航点后，在组合侧移/转弯时倾覆，独立验收正确停止并判失败。因此执行层改为上述保守前进/纯转策略并加入倾角保护；这项修正仍须后续物理完整运行验收，不能据纯软件检查声称已解决全部步态问题。

第五轮进一步确认：正向前进段方向正确，净反向漂移主要发生在反复微调转向段。因此再加入锁存转向和固定停稳状态，代码及实测依据见 `champ_gait_audit.md`。`run.sh trace --output <file> --duration 90` 提供没有任何发布器的只读控制/SLAM/独立真值记录器，可审计后续真实闭环行为。

加入转前 `pre_turn` 停稳后，共用生产 `HeadingGate` 的四段独立真实 Go2 转向/前进物理检查全部通过：机身前向位移分别 0.430、0.543、0.467、0.552 m，最大 IMU 倾角 0.1334 rad，结果为 `../simulation/test_results/heading_gate_preturn_physics.json`。未加转前停稳的上一版在第 4 段倾角达到 0.611 rad 失败，两版对照保留。该证据验证执行层，不替代带 FAST-LIVO2、SCAN、跨层地图和动态障碍的完整集成验收。

真实 SCAN 节点接口检查输出 `scan_interface_result.json`：1 条高程样条、159 条占据地图消息、最大 1173 个占据点。输入机身目标 z=0.2 m 时，样条控制点高程范围约 0.000009–0.200088 m。初次检查发现测试订阅器 QoS 与上游不匹配，已修为 sensor-data QoS；静止夹具显式冻结执行时间，避免把未移动的测试位姿当作已执行的轨迹。

2026-09-30 接手审计第六轮 `runs/20260922_211651_8232a9`：首点最小未经低通的 SLAM 距离为 0.2183 m，但到达窗口最多 0.101 秒，未满足保持 0.4 秒。20 次前进窗中位仅 0.7035 秒；制动、对齐、停稳合计占记录样本约 76%，近终点反复微调打断步态。传感器持续新鲜，障碍暂停和倾角暂停次数均为零，原失败记录保留。上述路线方向低通、持续偏差去抖与温和行走偏航修正通过 22 项行为测试及隔离 ROS 域 75 的 15 项接口检查。证据在 `run6_recovery_audit.json` 与 `ros_contract_result.json`；完整物理闭环仍必须以随后集成运行验收，组件检查不等于完整 Demo 通过。

第七轮 `runs/20260930_173838_c5e7ff`：导航的未经低通 SLAM 到达检查确认前三点，但独立真值联合验收只确认首点；第二、第三点受转向时 SLAM 跳动影响，不能称物理到达通过。第四点因 SCAN 优化持续失败并输出停车样条而超时，记录在 `feedback_recovery.jsonl` 与该运行的 `stack.log`。因此增加上述退化样条拒绝和明确原因，累计 24 项行为测试与 17 项 ROS 组件检查全部通过。

只读 `trace` 默认每 3 个仿真秒保存当前 full/filtered/occupied/inflated 点云的稀疏快照到 `<output_stem>_clouds/`，每帧为压缩 `.npz` 及对应 `.json`。仅为诊断裁剪测得机身周围 XY 半径 3 m、高程 −0.7～+0.8 m 并按 0.08 m 采样；不修改实际导航点云。元数据记录原消息时间、坐标系、点数、最近 SLAM 机身位姿及时间差，不能把相隔过久的云/机身误当作同步。`--cloud-interval 0` 可关闭云档案。域 75 合成接口夹具的 recorder 检查确认 full/filtered 实际话题均入档、最近机身时间差为 0，证据在 `test_results/trace_contract_20260930*`；该夹具没有 SCAN 占据发布器，occupied/inflated 须在完整实测运行中确认。

重跑 ROS 组件检查时使用独立域 75，各组分别启动对应节点，完成后结束节点再运行下一组：

```sh
# 终端 A（第一组）
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh controller
# 终端 B（第一组）
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh test-ros

# 终端 A（结束第一组后，第二组）
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh planner
# 终端 B（第二组）
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh test-scan
```

`min_obstacle_clearance` 为机身中心沿行进方向到雷达回波的距离，不是机身外壳净间距。端到端验收仍须独立检测接触和模型几何距离。


2026-09-30 第八轮 `runs/20260930_180318_efeddd` 在返回原点时受历史抬高地面占据阻塞。SCAN 原本有 free-space ray clearing，但共享端点或已经访问的体素使整条射线提前停止，漏掉后续实际观测空间；原 DDA 又以整数体素差计算方向，可能访问真实线段未穿过的格。导航现使用隔离的 `navigation/ros2_ws`，保留旧 `scan_multifloor` 与原 `SCAN-Planner-main` 不动。新版使用连续方向 DDA 和正长度线段/体素相交门控，各测量射线完整遍历，每帧每格最多一次 hit/miss，实际 endpoint hit 优先，逐帧清理 scratch 标记避免 char 第 255 帧冲突。未经过新测量射线的遮挡或未观测障碍保持占据；不使用 TTL、缩小膨胀或真值修图。

`test_results/grid_map_regression.json` 记录真实生产 `GridMap::raycastProcess` 的 14 项测试全部通过，原版同夹具 5 项失败。使用第八轮 212.3 秒的 2237 个稀疏存档实测点与同时刻 SLAM 雷达位置，把当时 10 个阻塞占据单元设为最大 log odds 后重复重放，新版剩余占据为 0、原版剩余为 2。这是人工设置初始占据的组件对照，不是完整历史日志重演。独立连续几何 oracle 验证 2671 条射线，包含同份实测云；结果及点云来源复核保存在 `test_results/review_*`。

真实 ROS 域 75 的 `tests/test_ros_raycast.py` 启动实际新 SCAN 核心，先以合成输入形成这 10 格占据，再重放同份实测云。实际 occupancy 话题中 10 格从占据变为清空，未观测旧障碍及其 inflation 仍保持，实际测量表面有 2204 格占据，7 项检查全部通过，见 `test_results/ros_raycast_result.json`。该测试不启动 Gazebo、不发送行走指令，不替代随后完整物理闭环。

隔离导航编译和射线组件复测：

```sh
# 已先编译旧 workspace 依赖后，在 Demo 导航 workspace 内执行
source /opt/ros/jazzy/setup.bash
source scan_multifloor/ros2_ws/install/setup.bash
cd multifloor_demo/navigation/ros2_ws
colcon build --packages-select plan_env path_searching bspline_opt scan_planner --parallel-workers 2 --allow-overriding bspline_opt
cd ../../..
source multifloor_demo/navigation/ros2_ws/install/local_setup.bash
multifloor_demo/navigation/ros2_ws/install/plan_env/lib/plan_env/raycast_regression multifloor_demo/navigation/test_results/run8_ray_fixture.txt
python3 multifloor_demo/navigation/tests/test_ray_coverage.py

# 终端 A：真实 SCAN occupancy 组件节点
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh planner
# 终端 B：同一个依赖环境，完成后结束终端 A
ROS_DOMAIN_ID=75 python3 multifloor_demo/navigation/tests/test_ros_raycast.py
```

`scan.launch.py` 强制检查 `scan_planner` 来自本 Demo overlay，拒绝回落到旧核心。完整启动应最后加载 `navigation/ros2_ws/install/local_setup.bash`，避免 overlay 的 `setup.bash` 重新加载旧 SLAM underlay。


2026-09-30 第九轮首次纯转即被独立验收判为倾覆，未完成首航点，不能据此评价新 SCAN 第四点历史占据修复。原始失败保存在 `runs/20260930_183805_30e555`；`navigation_turn_failure_audit.json` 记录零平移、最大纯转 0.12 rad/s，以及独立倾角 0.30→0.50 rad 仅间隔 0.04 仿真秒。因此补充上述原始 IMU 回调快速安全停车，并传播执行桥失败锁存。累计 28 项行为/安全单元测试、24 项实际 ROS 控制器组件检查全部通过；`ros_contract_result.json` 保留原始 IMU在 SLAM 水平时停车、两源稳定恢复、缺失/重复时间戳停车、严重原始倾角失败、桥已失败时导航任务失败等证据。组件测试不代表 Go2 步态或完整 Demo 已通过，必须由随后整合运行验收。只读 recorder 已处理 ROS 结束异常，并先写临时云文件再原子替换，减少停止时不完整快照。

## 2026-09-30 第 11 轮交接修复与组件复验

真实运行 `runs/20260930_192104_5bf2b6` 的前 4 个航点通过了同一初始 SE3 下的独立联合到达检查；第 5 点仍失败，完整 Demo 尚未通过。第 3 次转向有约 0.542 m 的瞬态定位误差，不能把前 4 点通过解释为全程误差小于 0.30 m。

第 5 点的新北向参考在 175.1 仿真秒被 SCAN 接收，随后新生成的局部目标却为 `[2.81,-0.163,0.00448]`，导致向东转。此处不是已证实的旧消息回放：实际 SLAM 世界速度约 `[-0.01874,-0.00782,-0.01147]`，执行指令为零，而规划起始速度仍继承旧样条的 `[-0.157,0.048,0.000442]`，加速度也来自旧名义轨迹。独立生产 PolynomialTraj C++ 探针用反推加速度约 `[0.2434,-0.0571,0.00032]` 同时重现打印的局部目标和目标速度；加速度未在原日志记录，因此该值是推断，并非实测。

修复全部放在 Demo 的导航 overlay，旧 SCAN 示例保留：

- 所有当前轨迹重规划都使用原始 SLAM 起点，世界速度按真实 odom 时间戳做 0.4 秒低通并限范数 0.12 m/s，不再继承未实际执行的旧样条导数。接口没有可信实测加速度，起始加速度取零。停车/纯转的 `execution_frozen` 使用零速度作为新的规划参考边界；这不表示实测速度为零，纯转时仍可能有平面漂移，原始测量另行记录。
- 全局单段的首尾是同一个时间单元，原先连续乘两次 2 形成 ×4；改为单段只 ×2，多段保留首尾各 ×2。规划器及优化器速度/加速度与执行层一致，分别为 0.12 m/s、0.15 m/s²。
- SCAN 仅在全局参考规划成功后提交 `reference_stamp` 和原始 `body_goal`；占据目标裁剪另记 `adjusted_body_goal`。导航只接受当前航点当前参考的 metadata 与实际 Bspline 全 payload 精确对应的成对消息，两个到达顺序均支持，缓存每种最多 32 项。同一参考只接受递增 `traj_id`，切换航点/参考时清空匹配状态。可变的样条 `start_time` 仅用于 payload 匹配，不用于请求身份或新旧排序。该校验证明来源，不替代 SCAN 碰撞检查或原始 SLAM 到达判断。
- 状态记录 `planning_start_state`，含原始位置/测量速度、实际用于规划的速度/加速度与 `frozen_reference_boundary`；正式 stack 的既有记录器保存该状态，不需要另开记录器。

隔离 ROS 75 的实际 SCAN 测试先生成旧原点样条，再发送第 11 轮测量状态与北向目标。新实际 Bspline 的最大 x 为 0.30379 m、终点约 `[-0.05315,1.99847,-0.00136]`、峰值速度 0.08626 m/s、峰值加速度 0.00777 m/s²；完整 metadata 配对与未冻结的实测速度低通检查也通过，结果在 `test_results/planner_handoff.json`。这是合成接口的实际规划器测试，不是物理 Go2 或 SLAM 端到端验收。独立 measured-state 7 项、生产 polynomial 6 项、身份配对 12 项（含 24 个跨话题排列）均通过。

本次实际 ROS 测试还暴露一个真实 ray 边界问题：混合正负方向的终点恰在多个整数网格面上时，DDA 同时跨轴可能永远无法达到 `floor(endpoint)`。原进程出现约 96% 单核 CPU 与回调不再进展；附加调试器被系统 ptrace 限制拒绝，未取得进程栈。独立原源码有限步探针重现了不终止。修复在真实线段参数到达 1 时结束，最后一个正长度单元仍返回一次，AABB 正长度门控、真实 endpoint hit、hit 优先、遮挡/未知空间规则均保留。重跑 2719 条独立真实 C++ 射线（含 48 条新增混合边界、2237 条实测存档射线）、实际 GridMap 14 项与 ROS mapper 7 项均通过。之前的失败接口结果、停止的 ROS 日志与调试器拒绝信息保留在 `test_results/planner_handoff_before_terminal_fix.json`、`planner_handoff_ros75_before_terminal_fix.log`、`planner_handoff_gdb_attach.txt`。

组件复验命令如下；实际规划器测试需要先在另一个终端启动隔离域 75 的 planner，控制器契约测试则先结束该 planner，再启动隔离域 75 的 controller，避免两个测试互相发布数据：

```sh
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh planner
# 在已 source /opt/ros/jazzy/setup.bash、旧 SCAN underlay 和 Demo NAV local_setup 的终端：
ROS_DOMAIN_ID=75 python3 multifloor_demo/navigation/tests/test_planner_handoff.py
ROS_DOMAIN_ID=75 python3 multifloor_demo/navigation/tests/test_ros_raycast.py
python3 multifloor_demo/navigation/tests/test_ray_coverage.py
python3 multifloor_demo/navigation/tests/review_run11_polynomial.py
python3 multifloor_demo/navigation/tests/review_trajectory_association.py
python3 multifloor_demo/navigation/tests/test_trajectory_contract.py
# planner 停止后：
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh controller
# 另一个相同环境终端：
ROS_DOMAIN_ID=75 bash multifloor_demo/navigation/run.sh test-ros
```

到达半径 0.22 m / 稳定 0.4 秒、独立 0.30 m 验收、航点时间限制、原始 IMU 倾角保护、碰撞足迹与动态障碍停车规则均保留。下一轮必须通过完整物理路线，以上组件通过不作为完整 Demo 成功。

## 第 12 轮坡道进展与跟踪修复

`runs/20260930_200540_d9d7b3` 的前 7/18 个探索航点通过独立联合到达，第 8 点在原 90 秒时限内失败；最近时刻两源距离约为 SLAM 0.31725 m、独立实际 0.30737 m，均未达 0.30 m，因此完整 Demo 仍失败。第 5 点这次按新北向 SCAN 轨迹正确交接，上一轮继承旧样条导数造成的向东过冲未再发生。

独立物理审计显示，第 8 段纯转约 10.75 秒并倒退约 0.829 m；停车排除最初 0.4 秒落稳后，每段漂移不到 6.6 mm。SLAM 平移误差包含约 1 秒宽的 Y/Z 波动，短窗中值并未改善主要转向重启次数，故未加入中值滤波。接近终点时，旧目标向量只剩约 0.2～0.4 m，同样的横向波动会放大航向角。

跟随器现在保留实际 SCAN 最近点与 0.8 m 弧长前视点，使用二者的路径切向及横向误差形成 `0.8 × 单位切向 + 横向纠偏`。纠偏尺度不随终点距离缩小，速度仍按实际前视点和原始目标距离衰减。到达/云输入仍使用原始 SLAM；0.20 rad 持续 0.5 秒重转、0.55 rad 严重偏差即时停、真实大角度换向及 0.12 rad/s 转向上限均保留。越过已检查路径终点但尚未原始到达时停车并请求新 SCAN，不能沿虚拟延长线运动。

障碍清除后仍先停稳并取得新的有效 SCAN。只有实际连续停车至少 1 秒且当前真实 SLAM 航向与新路径方向误差不超过 0.20 rad 时，才恢复前进；否则执行完整转前停稳/对齐/转后停稳。点云保护同时检查原实际目标走廊和实际纠偏运动方向走廊，两者任一阻塞都停车，碰撞足迹不缩小。

控制状态的 `steering` 保存控制器实际使用的 target/projection/direction/heading/error、ROS/odom 时间和关联 traj_id。正式 stack 的同一个只读 recorder 将完整数值 Bspline metadata 与实际接受的 Path 保存到 `feedback_navigation_trajectories.jsonl`；原 `slam.lookahead` 是记录器用 raw pose 重算的诊断，不能当作控制器实际 target。

34 项行为单元、域 75 实际控制器 26 项 ROS 契约、独立 8 项路径安全挑战及只读 recorder 4 项文件/退出检查通过。尾段不可变 68 点真实 SCAN 与原始 SLAM 回放还精确重建了 5094 条 tracking pose，原逻辑触发一次重转，新逻辑未触发；最大误差角约 0.388→0.236 rad。完整早期 Bspline 历史未存档，回放只适用于 524.774 秒后的尾段且采样时钟不同，不等于整个坡段或物理闭环已通过。证据分别见 `test_results/run12_heading_replay.json`、`test_results/recorder_trajectory_smoke_result.json` 和 `test_results/run12_steering_freeze.json`。下一步使用同一冻结导航版本进行真实坡道对照。

```sh
python3 multifloor_demo/navigation/test_control_core.py
python3 multifloor_demo/navigation/tests/replay_run12_heading.py
# 已停止域 75 控制器/规划器后，仅测试只读归档：
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=75 python3 multifloor_demo/navigation/tests/test_record_trajectory_capture.py
```

## 2026-10-01 实际动态障碍与行走偏航强度

首次实际两点组件 `test_results/20261001_dynamic_two_goal_actual` 确认北向第一点，第二点在原 90 秒内失败；真实箱子停车、通过后新 SCAN 规划和双源实际恢复前进成立。恢复后同一条近直线轨迹持续约 42 秒，前三次重转并非频繁换轨迹。实际机身在行走中偏航增加，横向纠偏随着接近路径减小，负方向误差触发原 0.20 rad 持续保护；行走偏航指令已大量达到原 0.04 rad/s 上限。独立真值只做评估，没有反馈导航。完整曲线与实际 steering 的只读审计保存在该目录 `navigation_geometry_audit.json`。

实际固定输入 15 秒对照中，原 0.04 与候选 0.08 均未触发倾角或接触保护；两者实际偏航响应均为正确符号，不能将动态失败解释为通用符号错误。随后用同一完整 source/runtime、同一 wrapper、同一原步态和 test-only joint-stop adapter，只改变行走偏航 cap 做真实两点 A/B：

| 真实组件 | 结果与证据 |
|---|---|
| `dynamic_strength_a_004` | 第二点 90 秒超时，最大原始倾角 0.4111 rad、桥 hold 1；失败保留 |
| `dynamic_strength_b_008` | NAV 在 65.551 / 141.020 秒真实确认两点，第二点耗时 75.469 秒；同一初始 SE3 双源联合到达通过，最大原始倾角 0.1690 rad、桥 hold 0、机身接触 0 |
| `slope_strength_008_actual` | 单目标相对世界 `[3,0,.3]`，真实 NAV 45.420 秒完成，原 90 秒 / 0.22 米保持 0.4 秒合同保留；最大原始倾角 0.2038 rad、桥 hold 0、机身接触 0、四足只有支撑面接触；独立初始 SE3 max 0.00605 米，12 项严格检查通过 |

上述目录位于 `navigation/test_results/`。B 与严格坡道在启动前逐项比较完整 source_manifest、实际库/二进制及 wrapper 参数；这些组件没有混入新的步态、主动机身反馈、GT 导航或理想 XYZ 控制。

因此正式 `MAX_WALK_YAW_RATE` 采用 0.08 rad/s，增益仍为 0.5；纯转 0.12 rad/s、0.20 rad 持续 0.5 秒 / 0.55 rad 即停、转前及转后各 1 秒、到点、倾角、障碍与时间限制保持。状态的 `motion_limits` 从实际 `control_core` 全局值报告 `walk_yaw_rate_rad_s`、`pure_turn_rate_rad_s`，并报告实例实际 `forward_speed_m_s`。这些组件结果不代表完整多楼层路线已经通过。

### 原生停车云与测试状态时序

控制器在首次实际 obstacle_hold 边沿先发布精确零速，再把该次 guard 使用的 self-filtered readonly 云、匹配机身位姿/旋转及时间戳、tracking pose、实际 SCAN target/direction、route/goal、request/index、保护结果交给有界异步 archive。NPZ 与 JSON 临时写入后原子替换，JSON 最后作为 commit；写入异常不撤销停车。文件在本次 `DEMO_RUN_DIR/navigation_events`，无环境变量时使用独立临时目录，不写共享路径。新请求清空状态上下文，旧事件文件保持。状态报告 archive 提交/完成/丢弃计数；队列满或退出超时的事件必须按 commit 验证，不能把未完成文件当作成功记录。

两点 B 原始 driver JSON 因使用最近一次 4 Hz 状态给命令贴 hold 标签，仍保存为 FAIL。精确 native 边沿与已执行同目标参考复核显示，两个真正 held 区间为 `[71.900,95.592)`、`[95.642,96.742)`，共 1488 条 requested/safe 指令全为零。4 条误标签非零在清空后的 96.793～96.843 秒，已使用新 SCAN；原 JSON 没有覆盖。独立正确时序审计为 `test_results/20261001_dynamic_strength_held_audit.json`，保留原结果 SHA 和所有其他合同，A 仍失败、B 实际组件证据通过。

未来 test-only driver 按 immutable native zero 边沿、真实 resumes 计数、同目标成功 metadata 与实际 accepted steering 界定 held 区间；仅收到未执行新 reference、旧 goal 或旧 request 不能解除 held 判断，真实 held 内非零仍失败。11 个含快速再次阻挡的边沿负例通过。旧 4 Hz 标签仍作诊断保留，不作为精确控制状态。

采用后软件复验：

```sh
python3 multifloor_demo/navigation/test_control_core.py
python3 multifloor_demo/navigation/tests/test_trajectory_contract.py
python3 multifloor_demo/navigation/test_results/dynamic_strength_staging/test_held_command_contract.py
```

当前分别 34 / 10 / 11 项通过。staging wrapper 固定了物理测试前的生产 SHA，归档入口用于复核那次单变量测试；正式源采用后它会拒绝不匹配的源，不能直接用旧 wrapper 伪装成当前版本重测。完整 Demo 由正式 stack 和网页随后验证，组件证据不替代完整探索、返回原点及跨层导航验收。


## 2026-10-01 预声明区域目标

新请求使用 `schema_version: 2` 与 `goals`，旧 `waypoints` 请求继续兼容；同一请求 ID 的完整目标定义哈希不同会被拒绝，不会修改正在运行的目标。每个新目标明确 `goal_id`、`center`、`arrival` 和 `timeout_sim_s`。SCAN 始终规划中心点，区域只负责原始实际 SLAM 到达判定，不修改占据地图、碰撞走廊或运动保护。

```json
{
  "schema_version": 2,
  "request_id": "flat-region-example",
  "frame_id": "camera_init",
  "goals": [{
    "goal_id": "floor1-north",
    "center": [0.0, 2.0, 0.0],
    "arrival": {
      "type": "disc_prism",
      "radius_m": 0.35,
      "height_half_span_m": 0.10,
      "axes": [[1,0,0],[0,1,0],[0,0,1]],
      "dwell_sim_s": 0.4
    },
    "timeout_sim_s": 90
  }]
}
```

`camera_init` 中心和区域轴由任务层使用启动后冻结的一次 SLAM 原点及场景航向转换。`disc_prism` 的 axes 是区域沿向、横向、法向三个单位轴，按行排列；平层建议半径 0.35 m、高度半范围 0.10 m。坡道使用 `oriented_box` 和预配置坡面 axes，`half_extents_m: [0.35,0.30,0.10]` 分别约束沿坡、横向、法向，不能将楼层高度忽略。返航原点继续使用 `sphere`、`radius_m: 0.22`，开球边界保持严格小于。区域必须运行前声明；历史失败的固定点合同和结果不变。

新区域的 0.4 秒保持时间只由新接收、递增的原始 SLAM 里程计整数纳秒 stamp 推进。重复/逆序 stamp、超过 0.2 秒的观测间隙、区域外、传感器失效、执行保护和障碍 hold 都重置保持。区域 90 秒期限在实际 segment 开始后同时覆盖区域内保持重置与保护等待；不能因一直处于区域内但观测间隙过大而无限等待。旧点计时顺序保持。完成前点的 stamp 不能开始后点的保持；重叠目标也需要下一次真实里程计。状态发布 `goals_definition_sha256`、`goals_definitions`、`current_goal`、`region_arrival_evidence` 和按序 `region_arrivals`，收据包含 request/goal/index、精确起止 stamp、dwell、原始位置、中心距离和 arrival 定义。旧请求保留原 0.22 m 点到达与计时逻辑，不使用新区域收据。

区域目标在已障碍 hold 时不能完成。若已检查 SCAN 路径在区域内结束，控制器继续用同一真实点云 guard 检查障碍，命令保持精确零；实际清除持续 1 秒后丢弃旧路径并请求新 SCAN。障碍仍存在、未取得有效路径或航点超时均不能通过到达判定。非 hold 的路径耗尽仍停车，不能沿虚拟延长线行走。

共享数学核心位于 `goal_regions.py`。`parse_request` / `parse_goal` 校验并冻结定义，`definitions_sha256` 哈希包含所有缺省字段；`RigidTransform.goal` 同时转换中心和轴，`contains` 与 `ArrivalWindow` 分别提供几何和真实观测保持。独立 GT 只用于评估，同一个初始 SE3 将真实位置转换后检查同一预声明区域，没有 GT 控制输入。

验证结果：18 个几何/时间反例、15 个实际 controller 方法 AST 测试、34 个运动与保护回归、10 个轨迹身份回归通过；真实域 75 synthetic DDS 请求/状态/SCAN-reference/停止契约 17 项通过，全部 owned 节点退出 0。域 75 没有 Gazebo/实际 SLAM，故这些是接口与逻辑验证，不是多楼层物理通过。`test_results/goal_region_staging/ros75_v2` 保留 fixture 自体过滤导致点云为空的失败，最终修正输入后的结果为 `ros75_v5/result.json`。旧 held-endpoint 源在同一真实方法反例中有 2 项失败，保存在 `controller_methods_before_endpoint_fix.json`；正式修改后通过。

```sh
DEMO_TEST_PRODUCTION_GOAL_CORE=1 python3 multifloor_demo/navigation/test_results/goal_region_staging/test_goal_regions.py
python3 multifloor_demo/navigation/test_results/goal_region_staging/test_controller_methods.py
python3 multifloor_demo/navigation/test_control_core.py
python3 multifloor_demo/navigation/tests/test_trajectory_contract.py
```

ROS75 fixture 结果目录禁止覆盖；重新执行前须选择新的结果目录，并先确认域 75 无其他测试节点。正式端到端验证由网页与正式 stack 运行，独立任务评估同时检查 region 定义哈希、实际 NAV 收据和同刻 SLAM/真实位置，不能只凭 index 自行宣告成功。

## 预声明内侧停车范围与近终点重新规划

第 15 轮 `runs/20261001_203152_88725b` 实际完成 18 个探索区域的 NAV 收据并到达三楼，完整 Demo 仍为 **FAIL**：第 12 个区域收据窗口 606.2～606.6 秒内，原始 SLAM 五帧都在半径 0.35 m 内，同一初始 SE3 下的独立实际位置五帧半径为 0.3571 / 0.3569 / 0.3593 / 0.3655 / 0.3519 m，均在原外侧范围外。之后的接近不能补救这个收据窗口；旧结果和原验收文件保留。

下一次任务在启动前声明 `arrival.control_band`，让控制器在外侧验收区域的内侧停稳。它继承同一中心、轴、类型和 0.4 秒保持时间，不能独立修改中心或方向；每个尺寸必须为正且不大于外侧范围。缺少该字段的旧区域定义、哈希和行为完全保持。建议正式配置为：

| 区域 | 原外侧验收范围 | 预声明内侧停车范围 |
|---|---|---|
| 平层 `disc_prism` | 半径 0.35 m，高度半范围 0.10 m | 半径 0.25 m，高度半范围 0.10 m |
| 坡道 `oriented_box` | 沿坡/横向/法向半宽 0.35 / 0.30 / 0.10 m | 0.25 / 0.20 / 0.07 m |
| 返航 `sphere` | 严格开球半径 0.22 m | 严格开球半径 0.17 m |

例如平层 arrival 增加 `"control_band":{"radius_m":0.25,"height_half_span_m":0.10}`。规范定义会完整包含继承的类型、轴和保持时间，并纳入同一个 `goals_definition_sha256`。同一 request ID 仅改变内侧范围仍会拒绝，不会修改进行中的任务。

`contains` 始终检查原外侧区域，`contains_control` 检查内侧；`ArrivalWindow` 的整段原始 SLAM 新时间戳保持必须在内侧。外侧内、内侧外时继续实际经过碰撞检查的 SCAN 路径，不能提前停车或开始保持。收据新增 `control_region_inside` 和完整 `control_arrival_definition`，原 `region_inside`、外侧 `arrival_definition` 和精确原始时间戳继续保存。独立验收对每个同刻原始 SLAM 检查内侧、对独立真实位置检查原外侧，保持原来的单一初始 SE3；不扩大验收范围、不使用真值控制。

14 项生产共享内侧几何测试、18 项生产 controller 实际方法 AST 反例、18 项旧共享核心回归通过。ROS75 实际生产 controller 的 synthetic DDS 接口 21 项通过，验证外侧内不能完成、内侧完整保持、改内侧哈希冲突、错楼层、重复/受保护观测、旧请求、严格有序时间戳和 90 秒期限；所有 owned 节点退出 0。证据在 `test_results/goal_region_inner_staging/`。这些测试没有 Gazebo 或真实 SLAM，不是物理路线通过。

返航内侧 0.17 m 暴露 SCAN 原近点门槛：旧 `reboundReplan` 在目标距离 <0.20 m 时直接返回失败。ROS75 实际旧规划器从 `[0.19,0,0]` 请求原点时，连续 50 次拒绝后只发布原地停车样条，不能恢复尚未到达的内侧目标。仅 Demo 隔离 `planner_manager.cpp` 将过短拒绝门槛改为 <0.05 m；实际中心、测量起点、原碰撞优化/细化/可行性链和所有配置均保留，旧 SCAN 工程没有修改。

新实际规划器同输入产生长度约 0.18935 m 的完整样条，终点约 `[0.000329,0,0]`。真实 DDS 点云地图中加入阻挡墙后，占据/膨胀地图确认障碍，规划器未发布移动路线。正负例与实际参数查询共 23 项通过，两个 owned 规划器正常退出。详见 `test_results/goal_region_inner_staging/near_goal_planning/fixed_ros75_v2/result.json`；这是实际规划节点接收合成传感器消息的组件验证，无机器人执行。

该规划器继承的 `manager.feasibility_tolerance=0.5`，导数控制点分量边界约为 0.1801 m/s、0.2251 m/s²。短曲线采样峰值约 0.15057 m/s、0.22054 m/s²，高于名义 0.12 / 0.15，但符合原容差；独立复算控制点也符合。曲线时间导数没有直接作为执行命令，NAV 前进 0.12 m/s、行走偏航 0.08 rad/s 上限保持。首个新 fixture 错把 10% 当成已有容差，其 FAIL 原样保存在 `fixed_ros75_v1`；随后直接查询实际 ROS 参数按原契约复核，没有改变或放宽生产配置，不能据此宣称机械精确跟踪。

```sh
DEMO_TEST_PRODUCTION_INNER=1 python3 multifloor_demo/navigation/test_results/goal_region_inner_staging/test_inner_regions.py
DEMO_TEST_PRODUCTION_INNER=1 python3 multifloor_demo/navigation/test_results/goal_region_inner_staging/test_controller_inner.py
# 域 75 空闲，已加载 ROS / 旧 SCAN 依赖 / Demo NAV local_setup；该 fixture 自己启动并清理 planner：
ROS_DOMAIN_ID=75 python3 multifloor_demo/navigation/test_results/goal_region_inner_staging/near_goal_planning/test_ros75.py --output <全新结果目录名>
```

内侧范围是下一次运行前的新控制合同。第 15 轮及更早的 FAIL 不会用它重新解释；完整 46 段探索、返航和跨层导航仍须网页启动后的严格实际验收。

## 2026-10-02 实测运动偏差候选验证

现有 NAV 已使用真实 SLAM 位姿和航向反馈跟随经过检查的 SCAN 路线；命令速度与机器人实际速度仍有偏差。里程计速度观察器目前只作诊断，没有部署新的速度 PI 或使用 GT 修正运动。

`test_results/turn_drift_staging/` 保存一个未采用的控制候选：原地转向期间，原始 SLAM 累计非预期水平位移达到 0.15 m、持续 0.2 仿真秒且至少包含三个不同原始时间戳时，先发精确零速，再等待实际 Adapter 新归位确认与新鲜 idle 状态保持一秒。随后请求同一目标的新 SCAN 参考，验证完整样条身份、原始进展检查和原 heading gate，才恢复转向。障碍等待、倾角保护和重新规划都计入原来的每目标 90 秒，没有注入前进补偿或改变增益。Bridge safe 是 Adapter 输入；实际零输出另由原生 Adapter 发布日志核验。

该候选在 `simulation/test_results/20261002_nav_drift_first4_disabled` 完成四个原始区域窗口，四次中断交接和实际零输出均通过独立审计，最大原始倾角 0.2182 rad、无倾角 hold。扩展到前八区域的 `20261002_nav_drift_first8_disabled` 虽完成八个原窗口，严格组件仍为 **FAIL**：第 5 个目标行走期间，193.283 秒原始倾角触及 0.30 rad，最大值 0.3619 rad。14 次漂移交接满足原确认链，第 8 区域在前点收据后 69 秒完成；这些成功项不能替代倾角安全。首 hold 的 JTC/关节数据已不在该次录制后缀中，不能补造或据此确定关节原因。独立证据在 `test_results/nav_drift_first8_audit/ready_receipt.json`。

这个候选仍只用于排除目录中的实际测试。其旧 legacy 点到达分支存在等待中仍推进旧计时的反例，不能直接作为兼容旧请求的生产入口；现行完整任务及扩展动态测试均明确使用 schema2 区域收据。后续采用必须处理该边界。

针对关节参考的 `interpolate_from_desired_state` 单变量方案，`test_results/jtc_desired_native_contract/actual_20261002_024858/result.json` 保存 installed JTC 4.40.1 实际接口的 31 项验证：两组服务读回 BOOL、原 effort PID 闭环、每组 269 条真实新 DDS 目标与 293 次实际返回 4 ms 周期，停稳/重启的 producer 目标和测量输入逐值相同。候选改变插值起点以及参考 effort 前馈；测得的接口 commanded effort 不等同于实际施加力。该受控内存接口不是 Go2 物理通过，真实前八段对照仍需终态数据判定，生产源码与控制标准保持冻结。
