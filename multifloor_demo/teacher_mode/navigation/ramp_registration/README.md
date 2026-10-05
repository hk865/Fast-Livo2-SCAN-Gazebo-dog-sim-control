# 真实点云坡道注册 Shadow

这是独立只读工具。它不发布速度、关节、导航目标或虚构点云，不接入现有 PID 控制链；目前只有离线合成检查，尚无实际坡道注册通过结果。本模块未修改既有 PID、CHAMP、Teacher、相机 Demo 和保护阈值；整个任务中的其他授权版本变更另有收据。

`observer.py` 在 ROS 79 读取真实完整注册点云、SLAM 机身/IMU姿态、实际加速度与角速度。回调只复制原消息字节和元数据；有界后台单写线程进行字段解码、哈希和存档。每帧保留完整原 payload（包括 padding/NaN）、全部结构化字段、有限 float64 XYZ 及其原点索引，支持字段任意顺序、FLOAT32/FLOAT64、大小端和 organized row padding。normal 字段保留但不作为有效几何法线使用。

重力向上轴来自静止时实际 `/livox/imu` 加速度，经源戳不晚于 IMU、已收到的 `/aft_mapped_to_init` SLAM IMU姿态旋入 `camera_init`，同时要求实际机身速度和角速度较低。轴累计校准一次后冻结，记录原始时间戳、配对姿态、变换后的样本和偏差；不使用 IMU orientation 字段、世界 Z、Gazebo姿态或场景朝向参考。云的机身自体过滤采用已收到的最近 SLAM pose（最大源戳差150ms），这属于墙钟因果近邻配对，不能称严格源时间因果配对。

实际运行仅由 root 编排。在已启动独立 SLAM、完成初始化并生成配置文件的 run 上，使用同一环境启动旁路 observer；它不会启动或停止任何生产者。输出目录必须是新目录。下面的 `RUN` 指实际采图 run，`OUT` 指新的独立证据目录：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=79
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/ramp_registration/observer.py \
  --source-run "$RUN" --output "$OUT" --duration-s 60 --wall-timeout-s 180 \
  --environment-manifest "$RUN/cloud_transport_manifest.json"
```

这条命令还必须继承实际 run 的 `cloud_transport_manifest.json.environment` 和移除后重新设置的传输变量，不能只执行上述 `export` 就声称配置一致。observer 会逐项校验实际环境、XML和来源哈希；没有显式 transport manifest 的 run 可省略该参数，但仍需 ROS79 和 source-run。source-run 必须已有 `sensor_contract.json`、`navigation_fastlivo.yaml`、`navigation_camera.yaml`。这些文件只作传感器/生产者配置审计，工具不读取 world.sdf、Actor扫描或场景几何。生产者是否确实执行这些文件还须由 root 的 runtime manifest/进程树独立确认，文件哈希本身不能证明执行。

`capture_manifest.json` 记录队列容量、原/处理计数、归档哈希、源拒绝原因、重力校准、环境和清理状态。队列/存储溢出、写入/收尾错误会尽量写 failed 收据并非零退出，不会静默截断。中断采集标为 interrupted；陈旧、重复、错误 frame、空包和布局错误保留原数据/拒绝理由，不能进入几何分析。

采集自然结束且 root 确认生产者清理后，离线分析：

```bash
OPENBLAS_NUM_THREADS=1 python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/ramp_registration/analyze.py \
  --capture "$OUT" --output "$NEW_REGISTRATION_OUT"
```

分析在重力方向分离保留多层支持面。当前支持层由实际 SLAM 机身与局部实测支持面的关系选择，允许机身位于坡面；不是最高/最低层选择，也不是实际脚接触测量。真实双侧边界须同时具有横向支持点覆盖、外侧实测下降和拟合竖直侧面；单边点云、点云凸包边缘或仅下层地面都不足。完整候选还要求观测到坡面两端实际相接的平台，且连接当前支持层唯一。

缺出口、单边、遮挡、支持不连续、楼层/路径选择歧义或采集不完整时输出 `partial/unverified`，`full_route` 必须为 null。合格时也只输出 `full_candidate` 和实际平台接缝间的测量中心线，表示完整观测的几何候选，不能称完整坡道运动、避障或导航通过。机器人从坡面中段开始采图也不能冒称走过了整条坡道。

实际浅角 LiDAR 可能看不清竖直侧面，平台可能遮挡下层或坡顶；首版因此可能长期只得到 partial。这是当前保守判据的限制，不能用 SDF 或预设宽度补证据。RANSAC 候选并不是通用地形语义识别，支持面精度和 SLAM 漂移尚需实际数据审计。图像彩色云不能满足完整云覆盖要求。

离线验证命令只产生合成测试证据，不初始化 ROS、不启动物理：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/ramp_registration/offline_checks.py --output "$NEW_OFFLINE_DIR"
```

下一步实际站立/局部运动采图，独立检查实测可观测范围；尚未接入 request 或完整路线执行接口，仅采图与几何候选验证。
