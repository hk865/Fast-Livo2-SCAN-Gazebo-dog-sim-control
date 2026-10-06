> 2026-10-02 当前任务切换：已暂停 Go2 控制／步态／强化学习调试，优先使用无腿相机传感器载体直接移动。入口与操作见 [camera_mode/README.md](camera_mode/README.md)。下文 Go2 记录保留历史结果，不代表相机模式验收。

# Go2 多楼层探索与导航 Demo

浏览器启动 Gazebo Go2 → 沿指定路线用 FAST-LIVO2 建彩色点云 → 返回起点保存地图 → 从 F1 到 F3 → 遇到移动障碍停车、等待清空、重新规划后继续。

当前各模块已接通，完整物理流程仍在集成验证，**尚未通过完整验收**。已保留每次失败的原因和原始记录。场景采用三个平台和两段 10% 缓坡；本版本不代表台阶攀爬或实机能力。动态障碍响应采用停车等待，不宣称已实现侧向绕行。

当前正式执行组合为 CHAMP 步态与关节 PID，实际64项参数回读、Gazebo五库、步态进程映射、连续静稳门及退出清理均纳入本轮证据。SLAM 里程计用于位置、航向和转向漂移纠偏；目标已改为预声明区域停稳判定。当前没有加载机器学习运动策略。

最近完整网页运行 Full19 完成18/18探索区域与首个返航区域，随后下坡倾角达到0.50rad，完整结果 **FAIL**。观测彩色点云289173点，但没有返回原点保存正式PCD，也未执行后续F1→F3和完整动态障碍阶段。见[原完整验收](runs/20261002_084329_14d9cc/acceptance.json)与[独立审查](slam/test_results/full19_independent_final.json)。

后续隔离下坡 v8 的0.60秒步态启动包络完成数学、私有构建及来源检查，但实际仅完成1/2区域后再次触发倾角失败，**未采纳**。实际新节点/控制库与源码一致，所属进程已清理；见[v8原结果](simulation/test_results/20261002_downhill_disabled_envelope_v8/downhill_result.json)。开启姿态反馈的同源对照正在验证。低D、首摆相位及真实动态两区域组件通过，不替代完整演示通过；旧失败与缺样均保留。详细过程见[运动控制现状](docs/MOTION_CONTROL.md)和[历史验证记录](docs/VALIDATION_HISTORY_20261002.md)。

## 开始使用

本机依赖 ROS 2 Jazzy、Gazebo，以及已构建的 Go2 控制、旧 SLAM5 依赖和 SCAN 工作区。新 Demo 使用 `slam/ros2_ws/` 与 `navigation/ros2_ws/` 两个隔离工作区，保留旧 SLAM5 和参考 SCAN 示例。首次运行或修改这些源码后先构建；命令在项目根目录执行：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
bash multifloor_demo/scripts/run.sh build
bash multifloor_demo/scripts/run.sh prepare
bash multifloor_demo/scripts/run.sh serve
```

`build` 先构建独立 FAST-LIVO2 的 `fast_livo2_core`、`fast_livo2_ros`，再构建独立 SCAN 的 `plan_env`、`path_searching`、`bspline_opt`、`scan_planner`。启动脚本加载本机依赖后，依次加载这两个 Demo 的 `install/local_setup.bash`，避免后加载工作区的 underlay 链覆盖 Demo 包；SLAM 和 SCAN 启动器分别检查实际包路径。

SLAM 启动前要求10秒预热后真实 IMU 连续静止3秒、执行桥就绪且零指令。SLAM 核心再次要求至少300条、跨度不小于2.9仿真秒的连续稳定 IMU，加载过程中发生运动则重置初始化统计；重力只来自实际加速度，不用真值或姿态替换。

当前真实 IMU 使用1000Hz，与1ms物理步一致；短坡100/1000Hz单变量对照显示原始加速度积分偏差显著降低，两组均联合到点，但不宣称路径效率或最终定位精度全面提高。完整路线效果以本轮验收为准。

当前原始雷达先按固定传感器外参过滤已知机身包络，再送入 FAST-LIVO2；不使用 SLAM 位姿或真值过滤。隔离核心增加可选 IMU 覆盖同步与诊断记录，当前 `require_complete_imu=false`、同步诊断开启。短程 A/B 结果见 [SLAM 模块](slam/README.md)，不能代替完整路线验收。

彩色地图仍使用8cm体素与实际相机RGB，存储上限提升到800万体素并按观测分配；超限仍拒绝保存。以本机实测推算，800万规模约5.35GB峰值内存、单次快照约2.26秒；这是容量余量与极限处理成本，完整路线实际增长和数据健康仍须实测。

规划重新从原始SLAM位置、按传感器时间低通并限制到Go2能力的测量速度开始，不再继承未执行曲线的速度/加速度；无可靠加速度测量时使用零加速度边界。转向/停车冻结期间的零速度是规划参考边界，诊断仍保留实际测量速度。单段全局时间不再重复放大；输出完整样条与当前参考戳、原始航点配对后才允许执行。该修复已通过分测，完整路线仍须实测。

独立 SCAN 修复共享端点和重复穿越造成的射线提前终止，采用连续射线方向、实际线段与体素的相交检查、真实端点命中优先，以及每帧清空临时标记，以及按实际线段参数结束遍历，避免混合方向在终点边界无限越过。占据只能由实际观测更新；修复没有加入时间到期清空、真值输入或缩小膨胀范围。详见 [导航模块](navigation/README.md)。

打开 [演示页面](http://127.0.0.1:8767/)，点击“启动完整演示”。页面显示五个阶段、本次彩色点云、SLAM 实际轨迹、SCAN 规划轨迹和真实仿真 RGB 相机画面。每次运行使用新目录，避免混入上次地图。

点击“停止演示”后等待本轮进程组清理完成，再次启动按钮恢复可用后再运行。停止是异步过程；`/api/state` 中 `process_running=false` 且 `stopping=false` 才表示清理结束。网页服务会继续运行；在服务终端按 Ctrl+C 可结束服务并清理其拥有的运行进程。停止操作保留已有日志和失败证据。

服务默认只监听本机，使用端口 8767、ROS 域 72 和独立 Gazebo 分区。不要同时在域 72 启动另一套传感器、定位或速度发布器。完整物理路线需要持续运行，页面出现“任务未通过”即为失败，不能用点云数量代替验收结论。

## 代码与模块分工

- [代码结构](docs/CODE_STRUCTURE.md)：历史系统、新模块、数据来源与入口。
- [开发计划与分工](docs/DEVELOPMENT_PLAN.md)：仿真、SLAM、导航、任务编排、网页和验收的职责与接口。
- [仿真模块](simulation/README.md)：三层几何、Go2 物理接触、传感器和移动障碍。
- [SLAM 模块](slam/README.md)：实际数据融合、坐标适配、RGB 点云保存。
- [导航模块](navigation/README.md)：SCAN 三维路线、实际位姿跟踪、避障与失联停车。

主链路：`Gazebo LiDAR/IMU/RGB → 原始雷达自回波过滤 → FAST-LIVO2 → SLAM 位姿与实测点云 → SCAN → 路径跟踪 → 原始 IMU 安全门控与速度看门狗 → CHAMP → 12 个关节的物理控制`。过滤仅作用于雷达，IMU 与 RGB 直接输入 SLAM。任务编排根据到达和保存反馈推进阶段。Gazebo 真值只参与独立验收，不参与导航反馈。CHAMP 当前没有 IMU 平衡反馈或复杂地形落足规划。

## 测试命令

无需启动仿真的逻辑检查：

```bash
source /opt/ros/jazzy/setup.bash
python3 multifloor_demo/tests/test_mission.py
python3 multifloor_demo/tests/test_processes.py
python3 multifloor_demo/tests/test_heading_history.py
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 multifloor_demo/tests/test_sensor_audit_ros.py
python3 multifloor_demo/slam/tests/test_slam_contract.py
python3 multifloor_demo/slam/tests/test_heading_alignment.py
python3 multifloor_demo/slam/tests/test_self_echo_filter.py
python3 multifloor_demo/navigation/test_control_core.py
python3 multifloor_demo/navigation/tests/test_trajectory_contract.py
python3 -m unittest discover -s multifloor_demo/tests -p test_startup_gate.py -v
python3 multifloor_demo/scripts/evaluate_run.py --self-test
```

完成上述构建后，还可运行不启动 ROS 节点或 Gazebo 的射线/地图类检查：

```bash
source /opt/ros/jazzy/setup.bash
python3 multifloor_demo/navigation/tests/test_ray_coverage.py
python3 multifloor_demo/navigation/tests/test_grid_map_edges.py
multifloor_demo/navigation/ros2_ws/install/plan_env/lib/plan_env/raycast_regression multifloor_demo/navigation/test_results/run8_ray_fixture.txt
```

真实点云夹具使用第八轮的稀疏存档与同时间雷达原点，并为对照预置占据格；它验证地图更新，不是完整历史状态或物理任务重放。旧实现作为负对照保留，部分回归预期失败。

物理、传感器和实际 ROS 接口的独立检查见三个模块的 README；独立测试使用域 73、74、75；安全桥与审计记录器接口测试使用域 76、77，完整演示固定域 72。接口模拟测试通过不能替代完整物理运行。

每次网页启动后，运行编号显示在页尾，对应 `multifloor_demo/runs/<运行编号>/`。对已结束的运行执行：

```bash
python3 multifloor_demo/scripts/evaluate_run.py multifloor_demo/runs/<运行编号>
```

评估器写入该目录的 `acceptance.json`，任何必需证据缺失或验收失败均返回非零退出码。

## 验收与产物

探索、返航和再次 F1→F3 的**每一个规定区域**都必须按顺序到达。当前 `three_platform_body_arrival_v2` 使用平层圆柱、沿坡面定向盒域和返航原点球域：SLAM 内部范围分别为平层半径0.25m／高度±0.10m，坡道沿坡／横向／法向±0.25／0.20／0.07m，原点半径0.17m；独立真值外部范围分别为0.35m／±0.10m、±0.35／0.30／0.10m及半径0.22m。必须在同一原始时间窗口连续停稳0.4仿真秒，样本间隙不超过0.2秒，每区域期限90仿真秒。跳点、乱序、错楼层、两源分时到达或缺少终态均不能通过。整条轨迹只使用一次初始坐标对齐，不能每阶段重新拟合；历史运行保留各自原始规则和结果。详见[运动反馈与区域定义](docs/MOTION_CONTROL.md)。

还须返回起点、保存本次 RGB 地图、连续行走到 F3，并具备实体动态障碍实际运动、实测点云触发停车及恢复、无机身碰撞或倾覆的证据。任务阶段结束不等于独立验收通过；最终以本轮 `acceptance.json` 为准。正常完成或失败后服务会自动评估；手动停止的运行可用上述命令检查，缺少阶段时会返回失败。

运行目录保存：

| 文件 | 用途 |
|---|---|
| `scenario.json` | 本次实际使用的规定路线 |
| `source_manifest.json`、`source_snapshot.tar.gz` | 启动时 Demo 模块源码与配置快照，以及逐文件 SHA256 |
| `runtime_manifest.json` | 本次 FAST-LIVO2 可执行文件/core 动态库、SCAN/CHAMP 可执行文件、CHAMP、JTC和Gazebo关节控制动态库，以及 `plan_env`、`path_searching`、`bspline_opt` 静态库的绝对路径与 SHA256；含 ROS 域和 Gazebo 分区 |
| `mission.json` | 阶段、航点、事件、数据新鲜度、独立验证记录 |
| `map_metadata.json` | 本次点云数量、颜色来源、保存状态 |
| `colored_map.*.bin` | 网页实时使用的 XYZRGB 点云 |
| `colored_map.pcd` | 返回起点后保存的彩色地图 |
| `pose_audit.jsonl` | SLAM 与独立真值轨迹 |
| `control_audit.jsonl` | 速度指令、姿态、航点及控制状态 |
| `startup_gate.json` | 实际启动静止窗口、阈值和放行/失败原因 |
| `sensor_audit.jsonl` | 从启动开始记录原始 IMU 数值、姿态及全部传感器时间戳；仅只读 |
| `feedback_navigation.jsonl`、`feedback_navigation_clouds/` | 从启动开始记录 SLAM/独立真值、请求与执行器安全速度，以及稀疏实测点云 |
| `feedback_navigation_trajectories.jsonl` | 完整 SCAN 样条元数据和实际接受路径，供精确回放；实际跟踪方向在反馈状态中记录 |
| `navigation_events/` | 实际障碍停车首次触发所用的完整点云、姿态、请求与门控上下文；异步写入，缺失或写入失败另有状态记录 |
| `fastlivo_debug/mat_pre.txt`、`mat_out.txt`、`imu.txt` | 本轮连续 SLAM 内部诊断；IMU 时间戳保留9位小数，不与其它运行或核心测试共用文件 |
| `stack.log`、`ros_logs/` | 各模块日志 |
| `acceptance.json` | 完整离线验收结果 |

必需记录器由本轮启动栈管理，在仿真动作前开始记录；异常退出会停止本轮。原始 IMU 有效姿态倾角达到 0.30 rad 时停车，达到 0.50 rad 时锁定失败，降至 0.18 rad 以下持续 0.8 仿真秒才恢复；导航与实际执行器桥分别执行该保护。

这些快照用于核对本次代码与实际二进制，不包含完整操作系统、外部依赖或整个运行环境。新增清单仅出现在实现加入后的新运行中；历史失败记录不补写成新版本运行。

[集成尝试记录](test_results/integration_attempts.json) 保留未通过运行及修复原因。建图只保存相机观测得到的 RGB，不使用预制环境 PCD 或高度伪彩色；最终地图反映传感器实际观察范围。
