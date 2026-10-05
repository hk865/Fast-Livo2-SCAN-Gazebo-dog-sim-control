# DDS 完整点云稀疏来源审计（2026-10-04，只读）

完整点云在多个独立接收者处确实稀疏，问题并不限于 PID 控制回调；现有证据不能把原因唯一归为 DDS。此次只读取原日志及源码，未启动 ROS/Gazebo、建立 DDS participant、发送信号、修改训练或系统参数。原日志保持原样，输入 SHA 与重现脚本见 [dds_historical_source_audit.json](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/pid_dds_transport_audit_20261004/dds_historical_source_audit.json) 和 [make_readonly_audit.py](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/pid_dds_transport_audit_20261004/make_readonly_audit.py)。

| 原始记录 | b579，PID V2 | cddc，PID V3 |
|---|---:|---:|
| DEMO_LIDAR_SLICE（同步切片记录，并非发布收据） | 1596 | 1311 |
| 实际 SLAM 位姿行 | 1560 | 1278 |
| 控制器接受的完整云时间戳 | 952 | 378 |
| gate / shadow 非空整数云时间戳 | 953 / 946 | 382 / 411 |
| 三接收者云时间戳并集 / 交集 | 962 / 933 | 449 / 326 |
| 实际位姿时间戳在三接收者云并集中全缺 | 599 | 837 |
| map_archive：odom / full_cloud / colored_cloud | 1560 / 944 / 1560 | 1278 / 374 / 1282 |
| shadow 云间隔模拟秒 p50 / p95 / max | .100 / .480 / 1.100 | .300 / 1.100 / 2.600 |

集合明确排除 `None`，因此 gate 及并集比此前口述计数各少 1；位姿全缺数字不变。cddc 的实际回调为 379 次，其中 378 次接受，1 次在云 94.9s、最近位姿 94.599999999s 时因 300000001ns 间隔拒绝；回调耗时最大 5.015ms。单条拒绝不能解释大量云共同缺失。map_archive 同进程的较小 odom/colored_cloud 到达显著多于 full_cloud，也是独立旁证。

当前受保护 SLAM 库与执行文件在两轮不变，冻结 SHA 分别为 `a007493e…1478bb`、`ea95f2b0…c323`。当前源码 [LIVMapper.cpp:568](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp:568) 在 `handleLIO` 发布 odometry，同一函数的 [605–609 附近](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp:605) 无条件发布 full_cloud，沿用原 `last_lio_update_time`；完整云没有经过彩色云的发布抽样分支。但历史日志没有发布调用开始/结束或消息序列收据，不能把切片日志或 odom 数量直接等同于实际 full_cloud 发布数量。

当前点类型为 [PointXYZINormal](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/include/fast_livo2_core/utils/types.h:8)。安装的 PCL 1.14.0 转换保留结构 padding；离线实际编译确认 `point_step=48`。根据历史 input_points 推算 b579/cddc 完整云 data 为 0.699–2.103MB / 0.707–2.829MB，未计 CDR/RTPS 开销。**这是类型与转换推算，历史日志没有直接保存实际 point_step/data_bytes**；新探针会记录这些实际字段。

## 传输机制与不能归因事项

实际安装的是 FastDDS 2.14.6、rmw_fastrtps 8.4.4；库 SHA 已入冻结。版本对应的 [rmw participant 源码](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/pid_dds_transport_audit_20261004/sources/rmw_participant_8_4_4.cpp:162) 先加载 XML，再在 LOCALHOST 分支追加默认 SHM 与 UDP；SYSTEM_DEFAULT 分支保留 XML transport。因此，给原 `ROS_LOCALHOST_ONLY=1` 直接增加 64MiB XML，可能同时保留另一套默认 SHM，不能作为纯容量对照。[官方 8.4.4 源码](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp)

FastDDS 2.14.6 默认 SHM segment 为 512KiB。发送会分配共享缓冲，部分 overflow discard 路径仅以 INFO 记录且可返回成功；没有错误输出不能排除丢失。全局 `/dev/shm` 空余空间不能证明单 participant segment 足够。大云碎片、容量和并发负载因此值得控制对照；但历史没有实际 RMW 环境/进程 maps、传输选择、发布耗时或碎片计数，也未排除发布序列化、接收执行器调度与队列竞争，**不能宣称 DDS 是唯一已证根因**。[官方 2.14.6 发送实现](https://github.com/eProsima/Fast-DDS/blob/v2.14.6/src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp)、[SHM 配置说明](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html)

`asof_environment.json` 是审计进程当时的环境与只读 kernel snapshot，不能代替旧运行环境。读取到 rmem/wmem max 4MiB、default 212992B；没有修改任何 sysctl。

## 已准备的新独立对照

新 [prepare_transport.py](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/cloud_transport_probe_20261004/prepare_transport.py) 只在新 RUN 内复制 XML、保存环境与原始来源 SHA，不启动进程；拒绝覆盖或活跃 RUN。两 preset 使用同一显式回环 UDP+SHM、无 multicast、SYSTEM_DEFAULT、关闭 builtin transports；**彼此只改变 segment_size：512KiB 与 64MiB**。端点 QoS、原始 header、300ms TTL、PID/Actor/物理保持原源。它们与历史 localhost 配置比较时，还包含 discovery/transport 构造差异，必须单列。

安装的 XML 默认加载解析及安全/冻结离线检查 19/19 通过，未创建 participant。V4 freeze `052d3a51…c92ee` 的 72 项 SHA 全部匹配，路径全部 canonical，未发现源差异。该结果是配置接口/离线解析通过，**不是实际传输改善或导航通过**。原始控制与物理判据没有放宽。

新 paired-QoS 探针只记录原整数 header、收包墙时/clock、实际 cloud fields/data_bytes、可用 MessageInfo、endpoint QoS 及探针自身 RMW/maps。RELIABLE 探针可能改变发布者重传/背压，所以零命令诊断要单列，不能冒充历史运动复现。探针 maps 只证明它自身；既有探针环境字段未直接收 `SKIP_DEFAULT_XML`、`ROS_STATIC_PEERS`，RUN transport manifest 与 parent effective-environment 收据补充这两项。实际 SHM/UDP fallback、完整云来源周期和控制健康仍需 root 的事前冻结实际 A/B。

配置/helper/runtime 源已经停止写入；本审计仅新增此目录的报告。未改任何原日志、通过的 camera Demo 或冻结 CHAMP V4。
