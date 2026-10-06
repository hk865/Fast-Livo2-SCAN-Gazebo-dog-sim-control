# Teacher 导航接入门禁：已准备，未验证

本轮基础运动尚未通过，不启动导航。`navigation/bridge.py` 是可审阅的接入边界；代码准备、门禁检查或模拟消息测试均不能写成导航集成通过。只有正式 `teacher_mode/runs/acceptance.json` 中 `levels.motion` 与 `levels.sim2sim` 都显式为 `passed`、原场景额外门通过、`navigation.allowed=true`、确认导航不用真值并匹配冻结模型哈希，桥才能初始化 ROS。preview和单轮 `summary.json`、接口 ONNX 成功或相机模式通过都不满足此条件。

本文件不提供绕过门禁的开关，也不启动旧 Go2 完整栈。旧 `simulation/simulation.launch.py` 会启动 CHAMP、JTC 与既有执行适配链，不能同 Teacher 执行器一起运行。相机模式及其通过记录保持独立。

## 数据来源与命令边界

源码审计确认：`slam/odom_adapter.py` 的 `demo_slam_odom_adapter` 从 FAST-LIVO2 `/aft_mapped_to_init` 生成 `/demo/slam/body_odom`，父坐标 `camera_init`、子坐标 `demo_slam_body`，速度为 SLAM 位姿差分后转入机身坐标。`navigation/controller.py` 的 `demo_navigation` 使用该位姿、`/cloud_registered_full`、原始 IMU 与 SCAN 样条，发布 `/demo/cmd_vel`。`navigation/align_controller.py` 继承该节点与接口。

桥只订阅 `/demo/slam/body_odom`、`/demo/cmd_vel` 与 `/clock`。它固定 SLAM/SCAN 话题，拒绝将输入重映射到其他话题；位姿必须有上述坐标系，并要求 ROS 图中唯一发布者为 `demo_slam_odom_adapter`，命令唯一发布者为 `demo_navigation`。它不订阅 `/demo/ground_truth`，不把仿真真值转换为导航里程计。

桥检查递增位姿时间戳、有限位置、单位四元数、至少两个连续有效位姿，以及墙钟和仿真时间的新鲜度。SLAM、命令、仿真时钟任一缺失或达到 300 ms 陈旧就输出零速度与停车请求。稳态墙钟定时器每 20 ms 检查，仿真暂停时看门狗继续运行。时钟倒退、非法命令、验收文件变化或丢失锁定失败，须重新审阅并开启新运行。

有效命令按机身坐标裁剪到 `|vx|≤0.3 m/s`、`|vy|≤0.2 m/s`、`|wz|≤0.3 rad/s`；z 运动与 roll/pitch 速度被拒绝。Twist 没有来源时间戳，桥记录实际接收墙钟与最近仿真时刻，不能将其冒充 SCAN 发布时刻。

桥以排他文件锁确认唯一写入者，原子替换 `TEACHER_COMMAND_FILE` 指定的 JSON 文件，默认 `teacher_mode/runs/nav_command.json`。记录包含 `mode/source=scan_slam`、`command`、原始 `requested`、序号、`monotonic_wall`、`sim_time`、完整 `stamp`、300 ms 有效期、停止与健康状态、观测来源及验收 SHA256。桥退出时尽力写入停车记录；文件写入失败时消费方必须依靠自己的新鲜度看门狗停车。

健康与停车另发布到 `/demo/teacher/navigation_bridge/status`、`/demo/teacher/navigation_bridge/stop`。此健康状态仅指 SLAM/SCAN 输入，不能代替 Teacher 物理执行器健康。不得把这个 `ready` 直接冒充现有 `/demo/control/safety` 的执行器已就绪；后续启动器须组合真实 Teacher 执行器状态、IMU保护及此输入门禁后再接入导航。

Teacher 消费端必须验证 `schema_version`、`source=scan_slam`、递增序号、同一机的 `monotonic_wall` 新鲜度、有限 command、healthy 与 stop_requested，然后执行自己的受控停车过渡。停车请求不等于 `action=0`，也不等于实际狗已经停稳。桥不计算关节目标、不切换控制器、不声明当前 worker 已经消费此文件。

当前准备的 Teacher worker 使用 Gazebo 特权机身速度、姿态、关节物理状态、实际 DC 限幅力矩和静态 SDF 局部高度扫描。桥文件对此明确提示，并要求消费端依据本轮 `policy_manifest.json` 审计。后续替换为 SLAM/IMU/局部点云必须逐字段记录来源、坐标、延迟、缺点处理及与训练扫描的偏差；不能将特权运动输入称为感知部署完成。导航位置始终使用 SLAM。

## 保持原 46 个区域的验收

权威声明为 `simulation/scenario.json`、`mission/route_regions.py` 与 `navigation/goal_regions.py`，共有探索 18、返航 14、F1→F3 14 个区域，按原顺序全部完成。不能跳点、重排、跨楼层误到达、用仿真计时结束代替实际区域回执，或在每段重新对齐来消除 SLAM 漂移。

| 区域 | SLAM 内部停车范围 | 独立真值外部验收范围 |
|---|---|---|
| 平层 | 水平半径 0.25 m，高度 ±0.10 m | 半径 0.35 m，高度 ±0.10 m |
| 坡道 | 沿坡 / 横向 / 法向 ±0.25 / 0.20 / 0.07 m | ±0.35 / 0.30 / 0.10 m |
| 返航原点 | 半径 0.17 m 球域 | 半径 0.22 m 球域 |

每区域使用原始传感器时间，必须在同一窗口连续停稳至少 0.4 仿真秒，样本间隙不超过 0.2 秒，单区域期限 90 仿真秒。SLAM 原始位置须在内部范围，独立真值同窗在外部范围；真值只进入离线验收。整条路线只冻结一次初始坐标对齐。路线和区域声明及其哈希在运动前归档。三层平台由 10% 坡道连接，坡道通过不等于真实楼梯通过；低台阶运动测试单独记录。

完整集成还要求返航后正式保存本轮 RGB 地图、连续到 F3、真实动态障碍运动、实测点云触发停车、障碍清空后重新规划并恢复、无机身碰撞与倾覆。相机 Demo 的原验收器含相机控制契约，旧 Go2 验收器含 CHAMP/JTC 运行库契约；Teacher 应保留原区域几何和传感器验收，同时另审自己的唯一执行器收据，不可伪造旧控制契约通过。

## 逐级运行顺序

1. 当前仅审阅代码和门禁。基础运动与 Sim2Sim 都失败或未验证时保留拒绝结果，不启动 SLAM/SCAN 运动闭环。
2. 每个基本命令、站立、停车、切换、超时/异常、上下坡及低台阶满足预登记阈值，并完成三次独立重复；与训练环境相同命令对照通过后，才在汇总验收写入 `motion=passed`、`sim2sim=passed`。写入必须来自实际证据。
3. 在独立 Teacher 传感器与执行器栈中先验证真静止初始化、SLAM/IMU时间、桥新鲜度、唯一发布者及实际消费端停车。不启动 CHAMP 或旧 JTC。先单个平层区域，再短路线；逐项保存 SLAM到达回执、独立真值、扫描路径、关节和力矩。
4. 平层路线完成后验证单坡上行和下行，然后原完整探索、返航保存与 F1→F3 46区域。动态障碍先做组件停车恢复，再纳入完整路线。任何阶段失败都保留日志，导航总层级继续为失败或未验证。
5. 在同一独立浏览器页显示实际 RGB、SLAM路线、SCAN路径和验收；诊断真值轨迹另标来源。当前只读仪表盘中的真值 x/y 曲线不是导航融合成功证据。真机部署始终为未验证，本任务不操作实体狗。

仅准备命令可现在运行，既不导入 ROS，也不写命令文件：

```bash
python3 multifloor_demo/teacher_mode/navigation/bridge.py --prepare
```

下面仅为门禁通过后的桥启动方式。需要另行完成并审阅独立 Teacher 导航启动器、同域真实传感器栈及消费端文件协议；它不构成现已可运行的完整导航演示：

```bash
source /opt/ros/jazzy/setup.bash
TEACHER_COMMAND_FILE=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/nav_command.json \
python3 multifloor_demo/teacher_mode/navigation/bridge.py \
  --acceptance /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/acceptance.json
```

桥只管理自身命令文件和ROS状态。关闭它不停止其他任务；不得停止或修改另一任务正在运行的训练。使用独立ROS域和Gazebo分区，每轮新目录归档模型、配置、代码、实际执行库及命令消费证据。当前发布状态：**导航边界准备完成，导航集成未验证，真机部署未验证**。
