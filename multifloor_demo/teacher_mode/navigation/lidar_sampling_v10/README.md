# V10：既有 LIO 残差并行段，两线程候选

独立软件加速候选，尚未实际运行。冻结 V7 core/ROS 全部 C++ 保留，只为 `voxel_map.cpp` 单独启用既有 `MP_EN;MP_PROC_NUM=2`，VIO 显式浮点归约段仍串行。冻结 V9 速度/姿态/路径控制与 clock hold 逻辑保持；同名 profile 的采样、路线、保护和日志窗口保持，模型仍 CPU 1 线程。详细证据与限制见[有限构建/数值/线程报告](../../test_results/lidar_density_rate_20261005/lio_multicore/README.md)。实际已安装 core SHA 与启动范围由[多核契约](LIO_MULTICORE_CONTRACT.json)、[本候选预检查](multicore_preflight.json)固定。

原 clock hold preflight、prospective criteria、scope amendment 及 diagnostic preflight 按历史原字节保留。它们证明复用执行逻辑，不表示已对新库完成真实轨迹重放。新合成 729 点 fixture 的串行/两线程、logger off/on state25+cov361 四份逐字节相同；非 wall 日志 payload 相同。测试专用 GOMP 观察确认两线程；实际 Gazebo 吞吐、位姿新鲜度、运动、导航仍待根任务执行。不要据此宣称 30 Hz 或完整多层路线已通过。

```bash
# 项目根目录；prepare-only 不启动 ROS/Gazebo
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_210 --label lidar64_30hz_rgb30_lio2_prepare --prepare-only
# 仅在旧仿真/控制器退出后运行
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_210 --label lidar64_30hz_rgb30_lio2_r1 --domain 88
# 比较日志开销必须按相同 detail window profile 配对
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v10/run.py --profile l64_r30_c30_detail3s_210 --label lidar64_30hz_rgb30_lio2_detail3s_r1 --domain 88
```

仅仿真，唯一 Teacher 执行器，原 300 ms 来源保护、200 Hz PD/50 Hz CPU Actor/20 Hz wall 控制 tick 保持。导航仍真实 SLAM/IMU/点云/SCAN，Actor 特权输入保留并披露。连接场景是坡道；不代表真实楼梯、完整 Sim2Sim 或真机已验证。
