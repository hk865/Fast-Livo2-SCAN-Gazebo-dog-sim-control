# Go2 多层 Demo 的传感器 SLAM 与本轮彩色地图

该模块以稳定 `slam5_navigation` 的真实 FAST-LIVO2 源码为基础，在本目录 `ros2_ws` 构建专用于 Demo 的覆盖层。稳定 SLAM5 源码与库未改；覆盖层新增连续静止 IMU 初始化、仿真瞬时点云的时间边界修复、可选的完整 IMU 时刻等待和内部同步诊断。当前场景是三层物理平台与 10% 缓坡，完整跨层验收以最终运行报告为准。

## 源码

| 文件 | 职责 |
|---|---|
| `launch.py` | 启动相机参数节点、FAST-LIVO2、位姿适配器、地图存档；旧目录拒绝启动 |
| `fastlivo.yaml`、`camera.yaml` | 与 `../simulation/scenario.json` 一致的真实传感器外参、相机内参和 LIVO 参数 |
| `odom_adapter.py`、`geometry.py` | 将 IMU 状态变换成机身／雷达参考点，按传感器时间计算有效 child-frame 速度，发布独立 SLAM TF |
| `heading_alignment.py`、`HEADING_ALIGNMENT.md` | 在静止初始化窗口配对 IMU／SLAM 姿态，冻结场景 world 轴到 `camera_init` 的航向；不修正运行位姿 |
| `self_echo_filter.py` | 在 LIO 输入前按固定雷达安装外参删除已知机身／腿部实体体积内的实际回波，保留外部点原始字段与采集时间 |
| `ros2_ws/src/fast_livo2_core`、`fast_livo2_ros` | 隔离的 Demo 算法覆盖层；默认记录 LIO 处理时最新／实际使用 IMU 与相机时刻，严格等待选项默认关闭 |
| `map_archive.py` | 只累积本轮 `/cloud_registered` 实测 RGB，按 8 cm 体素去重，原子写网页二进制与 PCD |
| `audit_live.py` | 只读统计真实传感器时间、RGB 和 SLAM；独立真值仅在验收报告中做固定初始对齐误差比较 |
| `tests/test_slam_contract.py` | 参考点、旋转外参、速度、点云布局／字节序／RGB、原子存档回归测试 |
| `tests/test_ros_interfaces.py` | 独立域合成消息的 ROS 传输、保存服务、时序和断流测试；不代表 SLAM 精度验收 |

## 坐标、时间与数据来源

FAST-LIVO2 原始 `/aft_mapped_to_init` 表示 IMU 在 `camera_init` 中的状态，原始 twist 没有有效值。新适配器使用 `T_world_target = T_world_imu × inverse(T_body_imu) × T_body_target`，不把原始 IMU pose 直接重命名成其他参考点。当前 Go2 的 IMU 与机身同原点同轴；雷达在机身 `[0.2, 0, 0.1177]` m。旧四轮模型中的 5 cm 参考点差异没有沿用。

相机位于机身 `[0.28, 0, 0.1]` m，optical RPY `[-π/2, 0, -π/2]`，640×480、80°水平视场、fx=fy=381.3611496301472。参数 `Rcl` 与 `Pcl` 满足 `p_camera = Rcl p_lidar + Pcl`。RGB 来自 Gazebo 相机成像后由 FAST-LIVO2 投影，不是高度、强度或楼层伪彩色。

先由场景门控等待至少 10 s 仿真暖机时间，并确认真实 IMU 连续静止、传感器有效、执行桥 ready 且速度为零，再启动 FAST-LIVO2。算法内仍单独要求至少 300 条连续静止 IMU（时间跨度至少 2.9 s）才完成重力初始化；当前实际传感器为 1000 Hz，满足跨度通常需要约 2901 条或更多连续样本，不能收够 300 条就在 0.3 s 提前完成。算法加载后再次出现运动时会清空累计，不能混入启动恢复动作。空 IMU 集合不推动计数或冻结重力轴。

内部静止标准只用实际加速度、角速度和 stamp：角速度范数不超过 0.03 rad/s、加速度范数与 9.81 m/s² 差不超过 1.2 m/s²、累计窗口各轴加速度标准差不超过 0.40 m/s²、相邻样本间隔不超过 0.03 s。重复／倒退或非法样本同样清空累计。该门控只作用于第一次初始化，运动导航时正常 IMU 传播不受限制；不读取 IMU 姿态或真值来修正 SLAM。`[DEMO_IMU_INIT]` 日志记录实际计数、首末时刻、重置原因、均值与标准差。3° 航向重力轴一致性验收保持不变。

阈值依据已记录的真实静止输入：A/B 静止段角速度峰值分别 0.0063／0.0082 rad/s，B 的 z 加速度标准差约 0.298 m/s²；第十轮恢复后峰值 0.00321 rad/s、z 标准差 0.174 m/s²。第十轮旧初始化窗口 13.6–16.6 s 混入物理恢复动作，重力倾角达 3.876°，最终正确拒绝启动。原始传感器记录器崩溃使该轮初始化原始输入缺失，保留失败证据，不回填它。新算法类对已存物理录音回放：运动期间清空累计，恢复后的真实 300 条静止样本可完成；B 录音末只有 293 条时仍等待。结果是离线回放验证，不能宣称完整 Demo 已通过。

当前 `preprocess.lidar_type: 0` 使用瞬时标准 PointCloud2 分支：所有点的 `curvature` 为零，不将其它传感器的每点时间字段或单位套在 Gazebo 点云上。已链接实际生产库的 fixture 复现旧边界问题：相机与 LiDAR 同 stamp 时 `0 < 0` 为假，整帧推迟到下一次 LIO；下一帧的零偏移点又被 deskew 的严格 `>` 跳过。覆盖层只对 generic 0 接纳同 stamp 点（5 ns 容差仅处理约 2 ns 的浮点转换差）、补偿起始零偏移点并避免首点重复变换。初始化期间同步推进 IMU 传播与点偏移的测量时间原点；不会跨整个初始化窗口补偿首帧。有每点时间的传感器保持原分帧规则。`[DEMO_LIDAR_SLICE]` 记录处理相机／上次 LIO 时刻、实际点数、偏移最小／最大值与下一批点数。

`camera_init` 原点是本次静止初始化的 IMU／机身附近，地面 z 约 −0.316 m，场景航点以初始化机身的相对高程定义。绝不能把本地 SLAM z 当成 Gazebo 绝对楼面高度。初始化后记录的实际 SLAM 初值由任务模块用于路线偏移。

连续内部诊断 `mat_pre.txt`、`mat_out.txt`、`imu.txt` 写入 `DEMO_RUN_DIR/fastlivo_debug/`。Mapper 与 ImuProcess 在构造时捕获同一目录，启动器显式把 run_dir 传给算法进程；不会先打开共享源码 Log 再切换路径。未提供运行目录的独立 fixture 使用进程唯一的临时目录，回归运行器还显式提供一次性测试目录。IMU 日志固定九位小数，完整任务超过 1000 s 时仍能区分 1000 Hz 的 1 ms 样本。此修复只影响记录位置与格式，不改输入、传播、更新或任何验收条件。关闭的上游可选 Colmap／图像／PCD 导出路径保持原行为；本模块真实彩色地图由 `map_archive.py` 存档。

`/demo/slam/body_odom` 与 `/demo/slam/lidar_odom` 的 stamp 原样保留 LIO 更新时刻，与 `/cloud_registered_full` 同步。速度用相邻变换后的 SLAM 位姿差分，在各消息的 child frame 表达，包含旋转杆臂速度；首帧或 >1 s 间隔的速度标为高协方差。重复／倒退时间和非法姿态拒绝发布。仿真重置时必须重启 SLAM 并创建新 run。

独立 TF 为 `camera_init → demo_slam_body` 与 `camera_init → demo_slam_lidar`，避免与 CHAMP 的 `odom → base_link` 和 URDF 传感器树争用同一个 child。FAST-LIVO2 的旧 `publish_base_tf` 已关闭。raw `aft_mapped` TF 仍由算法自身发布，供调试。

## 运行与操作

先构建 Demo 覆盖层：在项目根目录运行 `bash multifloor_demo/scripts/run.sh build`。常规完整任务的 `serve`／`stack` 入口会在稳定工作区之后加载此覆盖层；单独启动下面的 SLAM 命令时也必须先加载它，否则启动器明确拒绝旧算法库。

先运行场景模块并让 Go2 站稳；在项目根目录的新终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source slam5_navigation/ros2_ws/install/setup.bash
source multifloor_demo/slam/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=72
export ROS_LOG_DIR="$PWD/multifloor_demo/test_results/ros_logs"
export DEMO_RUN_DIR="$PWD/multifloor_demo/runs/$(date +%Y%m%d_%H%M%S)"
ros2 launch "$PWD/multifloor_demo/slam/launch.py" run_dir:="$DEMO_RUN_DIR"
```

启动器不启动或控制 Go2；传感器须在同一 ROS 域发布 `/clock`、`/livox/lidar`、`/livox/imu` 和 `/camera/image_color`。地图不可复用旧 run 目录。任务调度器正常情况下负责这些环境和进程生命周期。

检查当前算法输出和请求保存：

```bash
ros2 topic echo /demo/slam/status --once
ros2 topic echo /demo/slam/map_status --once
ros2 service call /demo/slam/save_map std_srvs/srv/Trigger '{}'
```

## 网页存档契约

每个 `DEMO_RUN_DIR` 内：

- `map_metadata.json` 每 2 s 原子替换；`run_id`、`revision`、`binary_filename`、`point_count`、`frame_id` 为固定接口。包含 counts、接收 wall age、仿真 stamp、真实 RGB 来源、bounds、save 状态及容量错误。
- `colored_map.000001.bin` 等为不可变快照，16 字节／点：XYZ little-endian float32（偏移 0、4、8），R/G/B uint8（12、13、14），padding（15）。读取 metadata 指定的文件，保留至少最近 3 个 revision。
- `/demo/slam/map_status` 为 `std_msgs/String` JSON，与 metadata 一致。网页应根据 `updated_at` 加上自身经过时间判断断流，不能把 HTTP 获取时间当成传感器到达时间。
- `/demo/slam/save_map` 为 `std_srvs/Trigger`；没有实测 RGB 点、容量溢出、SLAM／雷达／IMU／完整点云／相机／彩色点云超过 2 s 未更新或有地图错误时返回失败。成功后返回绝对 PCD 路径，并在 `save.healthy_sensor_evidence` 保存当时的源健康、接收年龄和传感器 stamp。重复／倒退 stamp 不刷新源健康。PCD 的 RGB 按 PCL packed uint32 写入，网页 bin 为直接 RGB 字节，二者布局不同。
- `colored_map.pcd` 是请求保存时的本轮地图。后续导航可继续增长网页地图，但已存 PCD 保持请求时状态，直到再次明确保存。

新运行点云最多 8,000,000 体素，保持 8 cm 分辨率；这是稀疏字典的容量保护，上限不会预分配内存或制造点。第十一轮实际已有 629,714 个真实 RGB 体素，按三层视角与 1.5 倍返航／重复导航余量外推约 2,833,713 个体素，超过原 2M 预算，因此为未完成的长路线留出 8M 上限。此估计包含重叠和未知可见区域的不确定性，不能当成地图精度承诺或完整跨层验证。

实际全量 629,714 点重建测得每体素深层对象内存约 488.3 B。使用真实存档代码的临时合成容量压力 fixture 在 2M／4M／8M 的快照数组构造耗时分别 0.567／1.125／2.258 s；8M 内存高水位约 5.35 GB，二进制 128 MB，PCD 生成另耗 0.213 s。压力 fixture 不发布 ROS、不修改观测地图，也不代表 SLAM 产生了 8M 点。主机可用内存约 55 GB；当前预估 2.83M 时构造约 0.8 s，内存有余量。最大 8M 规模可能使现有单线程存档超过 2 s 周期，延迟数据处理和健康状态；目前保持现有设计，完整运行必须持续验证实际源健康，不能以增大容量代替通过验收。

已知场景 44 个盒体全部六面约 1,654 m²，理想单层表面约 258k 个 8 cm 格，但已观测地图占据 629k 个三维格，且 bbox z 为 −3.878 到 6.480 m。重复配准和姿态误差会使表面占据多层体素，不能仅按理想表面积预测容量。所有实际记录和异常范围都保留；只读场景几何比较没有使用真值重映射或筛图。空间分布、最近快照增量、实测内存和压力 fixture 报告在 `runs/20260930_192104_5bf2b6/capacity_audit/`，预算汇总在 `tests/map_memory_budget.json`。旧运行原 600k／2M 上限仍留在其原始 metadata 中。超容量计数和严格保存拒绝保持不变。

## 已执行的验证

```bash
source /opt/ros/jazzy/setup.bash
python3 multifloor_demo/slam/tests/test_slam_contract.py
python3 multifloor_demo/slam/tests/replay_stationary_initialization.py
python3 multifloor_demo/slam/tests/run_core_regression.py \
  multifloor_demo/slam/tests/test_instantaneous_lidar.cpp \
  --output multifloor_demo/slam/tests/instantaneous_lidar_regression_result.log
python3 multifloor_demo/slam/tests/run_core_regression.py \
  multifloor_demo/slam/tests/test_run_log_isolation.cpp \
  --output multifloor_demo/slam/tests/run_log_isolation_result.log
ROS_DOMAIN_ID=74 ROS_LOG_DIR=/tmp/go2_slam_test_logs \
  python3 multifloor_demo/slam/tests/test_ros_interfaces.py
```

当前：13 项数值／存档测试、8 项航向校准测试、3 项自体过滤几何／字节布局测试和 16 项真实 ROS 传输与服务检查通过。存档回归包含保存后导航点云继续增长：最新 live 点数增加，PCD、保存点数与当时的健康证据保持固定。接口测试使用明确标识的合成消息和临时目录，结果在 `tests/ros_interface_result.json`。真实传感器建图、返航和跨层精度应以主任务的端到端运行报告为准；不能从这些接口测试推断它们已通过。

初始化回放会临时编译生产代码实际使用的 `stationary_imu_initialization.h`，检查 299／300 样本边界、运动重置、缺口、重复、非法样本、加速度变化、最小时间跨度和倾斜安装；随后回放 A/B 真实启动与转向数据及第十轮恢复后的实际 IMU。摘要保存到 `tests/stationary_initialization_replay_result.json`，不启动仿真。`tests/imu_process_initialization_result.log` 是实际链接 Demo 核心库的 `ImuProcess::Process2` 状态回归。

`test_instantaneous_lidar.cpp` 调用实际 `LIVMapper::sync_packages` 与 `ImuProcess::UndistortPcl`，验证同 stamp、早／晚 10 ms、±2 ns、初始化后首帧、均匀零偏移点的 0.1 s 运动补偿，以及有每点时间类型的原行为。旧库复现记录在 `tests/instantaneous_lidar_legacy_evidence.log`，新库验证在 `tests/instantaneous_lidar_regression_result.log`。该 fixture 使用独立 ROS 域 79 的参数服务，不启动 Gazebo、不发布控制或替代定位。

`test_run_log_isolation.cpp` 实际构造两个 Mapper，切换环境目录后分别完成对应 ImuProcess 的 300 样本初始化，验证 A／B 的 MAT 与 IMU 文件互不改写；未给环境的实例写入另一个临时目录，源码树原三文件保持完全不变。另以实际 IMU 输入触发传播，验证日志中的 1000.123、1000.124、2000.001 s 均唯一且误差不超过 1 ns。结果在 `tests/run_log_isolation_result.log`。运行中的正式任务保持源码／库冻结，禁止同时执行链接同一旧库的 fixture。

真实仿真运行时可另开终端只读检查（同域，不发布速度）：

```bash
ROS_DOMAIN_ID=72 python3 multifloor_demo/slam/audit_live.py \
  --duration 20 --output /tmp/go2_livo_live_audit.json
```

比较报告中首个匹配时间的 SE(3) 对齐仅用于消除 `camera_init` 与验收真值 frame 的初始差异；之后全程使用同一变换，没有按后续真值修正 SLAM。静止窗口的误差不能代替行走或跨层误差。

完整任务结束后使用 `../scripts/evaluate_run.py <run_dir>` 写入 `acceptance.json`，失败返回非零。评估要求探索、返航和跨层导航的**每一个规定航点**在严格递增的实际记录时刻到达，且同一时刻的 SLAM 与独立真值距离均不超过 0.30 m。终点正确但跳过中间航点、顺序错误或两源在不同时间抵达均失败；报告在 `ordered_waypoints` 列出逐点实际证据。全程只用一个初始 SE(3) 对齐，不能按阶段或航点重新拟合。

`relative_world_axes` 路线必须具有本轮冻结的 `heading_alignment.attitude_pairs`，评估器从这些 IMU／SLAM 姿态重新计算航向并与摘要核对；所有配对须属于记录中的静止 `waiting_sensors` 窗口，姿态须与实际 SLAM audit 一致。还要求返航和 F3 终点正确、四个阶段全部完成、真实 RGB 二进制和 PCD 均至少 500 点、保存时传感器健康、没有容量截断、实际接触监测可用且无机身碰撞，并检查实际倾角。

动态障碍必须在 `mission.validation.obstacle_history` 中保留导航阶段的 active 状态、成功 Gazebo 更新计数、确认后的实际位置与仿真时间。相对首个确认位置至少移动 1 m，按时序出现 `blocking → leaving/clear`，且没有任何失败更新或计数／时间倒退；静止盒子的停止／恢复计数和缺少历史的旧记录均不能通过。`python3 multifloor_demo/scripts/evaluate_run.py --self-test` 的 34 项合成用例只验证评估器，不代表实际 Demo 已通过；结果在 `tests/evaluator_selftest_result.json`。


## 转向与自体回波的物理 A/B 诊断

新输入链为 `/livox/lidar → self_echo_filter → /demo/slam/lidar_filtered → FAST-LIVO2`。过滤只使用固定的 URDF 雷达到机身外参，不订阅定位或独立真值；在机身坐标内删除 trunk 与腿部已知实体体积。被保留点的时间、XYZ、intensity 和其它原始字段均不变。发布端用可靠传输（depth 10）匹配 FAST-LIVO2 的订阅；首次测试发现不兼容的 best-effort 设置已明确记录为失败，未当成 SLAM 结果。

在独立域 74 用真实 Go2 四足物理做两次 90° 原地转向和两次 12 s 前进：原地最大角速度 0.12 rad/s，前进 0.1 m/s；运动控制只依据原始 IMU 航向。独立真值只记录和离线评估，每次测试均只从初始化首对匹配姿态取得一个固定 SE(3)。

| 指标 | A：原始真实 LiDAR | B：实体自回波过滤 |
|---|---:|---:|
| 全程定位 RMSE | 0.04877 m | 0.06601 m |
| 最大定位误差 | 0.25931 m | 0.22326 m |
| 最终定位误差 | 0.04475 m | 0.00327 m |
| LIO 最大单次位置修正 | 0.14201 m | 0.12863 m |
| VIO 最大单次位置修正 | 0.02048 m | 0.02387 m |
| 实际 RGB 地图点 | 151,006 | 205,847 |

两次内部同步日志都没有超过 1 ms 的相机／最新 IMU 缺口；最大差约 2 ns，属于时间浮点转换误差。B 在 10.6 s 启动阶段出现一包空 IMU 集合，运动阶段没有缺失。每扫描有约 597/15,360（3.89%）点落在已知自体体积中，B 已实际收到 685 帧过滤云并产生 653 帧机身／RGB 输出。滤波降低了本次最大和最终误差，但 RMSE 更高；不能据此认定它是唯一根因或所有精度都改善。`common.require_complete_imu` 默认关闭，`debug.sensor_sync_diagnostics` 默认开启。该短测没有验证三层完整任务。

证据目录：`test_results/turn_raw_relaxed` 与 `test_results/turn_filtered_relaxed_v2`，保存实际物理位姿、传感器记录、内部 before/after 状态和 `turn_diagnosis.json`。

第十二轮 `runs/20260930_200540_d9d7b3` 只有前七个探索航点通过同刻 SLAM／独立真值 ≤0.30 m 验收，第八点超时；没有完成返航、保存或跨三层。`tests/audit_slope_inputs.py <run_dir>` 只读分离已存 raw IMU、LIO／VIO pre/out：525.9–526.9 s 的传播 z 增量累计 −0.354 m，LIO 与 VIO 分别纠正 +0.062／+0.024 m；实际 GT z 只变 +0.0055 m。原始 IMU 姿态旋到世界并减重力后，加速度积分给出 z 速度变化 −0.521 m/s，而真实 50 Hz 位置的中心差分速度只变 −0.041 m/s，说明输入本身存在显著积分差异；不是一次视觉更新把位置直接拉动 0.3 m。

该轮真实 IMU 100 Hz、物理步进 1000 Hz、最大 IMU 间隔 0.01 s；已用 Gazebo 源码核对采样读取瞬时物理加速度，没有在这条发布链中平均。原始 LiDAR 字节和逐平面残差没有保存，裁剪后的 registered 云也不能完整重建每一次激光约束。报告 `slam_slope_input_audit.json` 保留这些证据限制与 951 组真实传播／更新分离记录；真值只做离线比较，不替换 IMU、重映射点云或放宽验收。

随后在域 73 的真实短坡物理测试中，只改变传感器 IMU 的 100／1000 Hz 采样率，控制、步态、SCAN、SLAM 核心与几何的 hash 一致。两组均通过同时刻双源 ≤0.30 m 到达，0 执行桥 hold、0 机身接触。干净的 A2 100 Hz 与 B 1000 Hz 比较，原始世界 z 加速度的累计速度残差为 +4.9519／−0.003803 m/s，一秒窗口残差 RMS 为 0.2364／0.08398 m/s，支持瞬态加速度抽样误差的解释。两组使用同一固定初始姿态对齐的 SLAM RMSE 为 0.002665／0.004406 m，最大误差 0.007605／0.010179 m；B 不是所有定位指标都更好，耗时也从 70.281 增至 78.242 s。当前选择 1000 Hz 是为了真实输入与物理步进一致，并非承诺导航效率或所有精度均改善。

实际 `DEMO_SYNC` 逐帧消费计数与独立高深度 raw 相符：A2 7320／7320、B 81200／81200，没有缺样；内部记录的头尾加速度／角速度均值与实际 raw 逐条对应（A2 7327 对、B 81239 对，0 缺失，仅六位旧日志舍入差）。实际 PointCloud2 是 480×32、point_step 32 的 XYZ／intensity／ring，raw 和实体过滤后的字段均没有每点时间。完整记录与固定 hash 在 `../simulation/test_results/slope_a2_100hz/`、`slope_b_1000hz/`，汇总 `slope_imu_a2_b_analysis.json`；归档内部日志为各目录 `fastlivo_internal/`。首次 A 的共享 IMU 日志曾被另一个 linked fixture 截断，MAT 在下一次启动前未归档；旧 A 保留限制，A2 是全新独立测试，不向旧数据回填。日志隔离修复的新核心 hash 只改变诊断路径／格式，频率 A/B 对应的是先前冻结核心。短坡从坡上创建机器狗，不复现十二轮完整历史，也不等于三层任务已通过。
