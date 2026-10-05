# Teacher 的有限 SLAM/SCAN 导航实验

此入口实际接入车载 RGB、CameraInfo、IMU 和 LiDAR，复用已安装的 FAST-LIVO2 和 Go2 SCAN。首次实验是一米前进并返回两个区域。只允许已验证的平地起点 `[6,-0.7,0.4,0]`，不启动原 camera carrier、CHAMP、ros2_control 或任何第二关节执行器。Teacher 的 247 维运动观测仍是 Gazebo 特权输入；**导航定位与到达判定只使用传感器 SLAM**。这项实验不会改写 `runs/acceptance.json`，也不代表整体 Sim2Sim 或导航已通过。

## 先验证真实 SLAM

由独立编排器生成新 run 的 full sensors 资产并运行持续零速度的 Teacher。配置生成本身不启动 ROS 或 Gazebo：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo
python3 teacher_mode/navigation/scoped_profile.py --run "$RUN" --slam-only
source slam/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ros2 launch \
  /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/slam.launch.py \
  run_dir:="$RUN"
```

`--slam-only` 只准备配置，生成独立 `navigation_fastlivo.yaml`、`navigation_camera.yaml`、`navigation_scenario.json` 和源码/二进制哈希收据。此 launch 仅启动传感器 QoS relay、自回声过滤、FAST-LIVO2、SLAM body/lidar adapter 和本轮颜色点云归档；没有速度发布器。

实际 topic 链是 `/demo/camera` → 保留全部像素与原始时间戳的 reliable relay `/demo/teacher/slam/image`；`/livox/imu` → 同样原样 relay `/demo/teacher/slam/imu`；`/demo/teacher/raw_lidar` → 仅按固定物理外参删除 Go2 已知身体/腿部包络的 `/demo/slam/lidar_filtered`。这些 relay 必需，因为现 Teacher Gazebo 桥使用 best effort，而 FAST-LIVO2 图像/IMU订阅使用 reliable。过滤器不读取 SLAM 或真值，保留剩余点记录的原字节与原始采集时间戳。地图归档中的历史名称 `/camera/image_color` 经本 launch 明确 remap 到 `/demo/camera`，不是 overview 图。

车载相机实际 640×480、80°、`fx=fy=381.3611496301472`、`cx=320,cy=240`、D 全零。配置保持原始 K 和 `scale=0.5`，FAST-LIVO2 内部缩成 320×240、焦距 190.680574815；relay 不再缩放。每轮实际 CameraInfo 与车载图必须同 frame/尺寸并在 50 ms 内配对，overview 只显示。外参由该轮 `sensor_contract.json` 的实际 IMU、LiDAR、相机位置/方向计算，不能配错 overview 的内参或外参。

IMU 实际 200 Hz，因此初始化为 600 样本并仍要求连续静止时段至少 2.9 s，保留原 gyro/加速度/缺帧阈值。实际站立轮 `20261004_010925_stand_actual_slam_stationary_r1_5d59` 的 2 Hz 图像已产出 SLAM，但位姿 p50 周期 0.5 s，不能满足 300 ms 导航门。后续 `20261004_011355_stand_actual_slam_10hz_stationary_r1_ba11` 的实际 10 Hz 图像得到位姿 p50 0.1 s、p95 0.2 s，注册点云 p50 0.1 s、p95 0.3 s；最大缺帧仍会触发停车。因此有限导航必须新生成 **实际 `--camera-rate 10`**，不能插帧或放宽缺失判据。上述站立测试不是路线通过收据。

## 开始一米往返

现独立 `scripts/run_test.py --navigation-stack` 负责创建新 run、冻结所有来源、full sensors、Teacher CPU worker、Gazebo、桥和视频、只结束自己的子进程。它会调用以下配置准备和 stack；勿在同一 ROS domain 再启动另一份速度控制器：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo
python3 teacher_mode/scripts/run_test.py navigation --navigation-stack --label flat_slam_scan_roundtrip
```

编排器使用：

```bash
python3 teacher_mode/navigation/scoped_profile.py --run "$RUN"
# Teacher worker 的其他 socket/run 参数由编排器提供：
# --test navigation --command-file "$RUN/navigation_command.json" \
# --acceptance "$RUN/navigation_scope.json" --navigation-duration 120
source slam/ros2_ws/install/setup.bash
source navigation/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ros2 launch \
  /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/stack.launch.py \
  run_dir:="$RUN"
```

`navigation_scope.json` 是**允许有限实验的收据**，状态为 `experimental_unverified`。它要求原正式平地十类动作各三次全部实际 interface/motion 通过，并冻结本轮配置、world、sensor contract、CPU policy、native actuator、共享 SLAM/SCAN 源码/二进制和全部原始运动收据的哈希。它不会把整体 `motion=failed/sim2sim=failed/navigation=unverified` 改成 passed，也不会放行坡道、台阶、上层遮挡或原起点。文件变化拒绝运行或停车；新能力需新范围、新 run 和新收据。

传感器门检查实际 publisher 链、正常车载 K/D/image 配对和真实初始化历史。V4 要求实际 IMU 至少600个不同时间戳、跨度至少2.9秒，以及 SLAM body/注册云/raw LiDAR/image/CameraInfo 各至少10个不同时间戳、跨度至少2秒。间歇缺帧不会抹去已经实际收集的初始化历史；**当前**所有源仍必须满足300ms，并需两个新的SLAM位姿、至少0.1秒跨度才能恢复。仅此门就绪后，`request.py` 从 **SLAM 原始 body pose 和 SLAM body yaw** 一次冻结起点与一米前方区域，写 `navigation_anchor.json` / `navigation_request.json`。没有仿真 world→SLAM 真值对齐或实时真值导航；返回中心使用相同固定 SLAM 起点。场景起点是生成资产的已知配置，只用于约束实验地点，不进入导航目标/反馈计算。

每个区域为半径 0.22 m、高度 ±0.10 m 的 disc prism，控制停车半径 0.17 m，连续原始 SLAM 时间戳停留 0.6 s；单观测缺口 0.2 s 重新累计，单区域最长 90 s。先申请与目标戳/轨迹 ID 关联的真实 SCAN 轨迹，再执行；禁止没有已检查轨迹时直接向目标走。共享 Go2 `body_height=0.4` 的 SCAN reference z 偏移由同一控制器抵消，不改变请求的 body center。速度限 0.2 m/s、原地转向 0.12 rad/s；返回包括实际 SLAM 转向与新 SCAN 路径。

bridge 仍按真实 wall 和 sim time 检查 pose、clock、command、健康门在 300 ms 内，且 publisher 唯一。SLAM 原始 pose 超出相对路线围栏（沿程 −0.45…1.45 m，侧向 ±0.45 m，高度 ±0.15 m）锁定停车。异常/缺失输出给零**速度命令**并继续 Teacher 停车过渡，绝不 action=0 或冻结关节当停车。仅 native Teacher 插件有执行权。

V4 使用单一 rclpy ROS TimeSource 的实际 `/clock`（depth1），传感器接收只保留最新帧；文件持久化、轨迹压缩、日志与graph/hash检查移到有界后台线程。命令传输最多一帧写入中、一帧待写，停车替换待写运动；源戳不在写文件时刷新，延迟超过300ms仍被 worker 拒绝。实际写入历史记录 superseded 数量和写入延迟，不能把它称为每帧都按时送达。通信 hold 仍立即停车，但不会被当成真实倾角而反复清除路径。真实倾角0.30/0.50rad与清除0.18rad/0.8秒保护不变。

Teacher 专用起步/转向转换依据零速度命令下的**实际 SLAM body-frame 速度**（平面≤0.03m/s、yaw rate≤0.05rad/s）及**实际IMU body gyro**（norm≤0.05rad/s），至少3个位姿、0.3秒源戳窗口，位姿间隔≤0.2秒、gyro配对≤0.15秒，最新测量仍满足300ms。朝向只来自SLAM，转向≤0.12rad/s；转后重新实测停车，误差≤0.20rad才允许前进。IMU yaw不参与定位。真实云障碍保护、清空一秒和新SCAN路径要求保留。初始请求期限是实际sim90秒，另外wall600秒上界处理时钟缺失；不再半速运行到wall100秒就提前终止。

源版本修正与离线证据见 `V4_CORRECTION_20261004.md`。离线检查通过不是路线通过，需在全新run中实际运动与审计。

## 实际验收与后续范围

首次路线通过需要两个原始 SLAM 区域收据、实际 SCAN committed 轨迹关联、持续 Teacher 控制、全程源数据正常、到达后的实测停车和无机身碰撞/跌倒/越界。`navigation_status.jsonl`、`navigation_slam_poses.jsonl`、`navigation_sensor_gate.json`、`navigation_command.json`、原始 policy/native logs、真实视频和源码哈希均保留。脚本只发送请求和状态，**不自动写导航 passed**；最终 evaluator 应审查真实运行。即使局部路线过，也只证明此平地有限条件。

现有控制器已实现注册点云阻挡→停车→清空一秒→重新请求 SCAN→恢复；Teacher 包装额外要求 measured SLAM body 位移速度 <0.03 m/s 才恢复。没有实际障碍出现/清除和物理停车记录就保持动态障碍恢复未验证。首次实验不会主动生成障碍。

后续按新的独立范围逐级扩展：多区域平地路线、受控动态障碍、已验证坡道 fixture、三层连接与原 46 个区域。完整坡道与原点/上层扫描语义必须按当前真实数据单独判定，不能套旧短坡道 pass；5 cm 旧机身高度差门也不能代替“脚登台继续走”的已通过功能证据。坡道连接始终称坡道，不称真实楼梯。真机仍未验证。

本任务不操作训练进程、原 camera_mode 资产或真实机器人。
