# V7：二层平台高度故障的独立诊断

实际210秒已完成：首次保护133.13秒、16/32区域，独立导航FAILED；物理执行安全、完整退出和159757条诊断完整性通过。131–134秒内部估计高度下降336.5mm，native机身仅下降3.7mm。具体证据指向早期东墙单环簇的错误水平面法向及停止更新机制，限定单迭代移除实验支持实质贡献，尚未在线修复。详见[总报告](../../docs/SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)、[LIO报告](analysis/actual_r1_lio/LIO_MATCHING_FAULT_REPORT.md)、[独立实体/采样审计](analysis/actual_r1_plane_geometry_audit.md)、[规范VIO重建](analysis/actual_r1_vio_v2/summary.json)。旧VIO分析错误保留并纠正，不作为运行算法损坏的证据。V7实际执行源码/配置/schema/binaries现已冻结，后续修复另建V8。

仅仿真，冻结 CPU Teacher，不更改 camera_mode、V1–V6、原安装 SLAM 库或另一任务。导航仍使用实际 SLAM/IMU/点云和 SCAN；Gazebo native 只供离线对照。Actor 的 232 维特权输入沿用本轮 V6 契约，不声称已经全部传感器化。

这是与 V6 同场景、同路线、同串级控制器和原保护门的 **210 s 有限前缀**，不是完整 600 s / 32 区域验收。300 ms 来源有效期、300 mm 路径高度门、唯一关节执行器和持续 Teacher 停车推理保持。

`slam_ws/` 是独立源码及安装副本。新增记录原始 IMU 加速度/陀螺/整数时间戳，实际 IMU 传播，LIO/VIO 状态与完整协方差/信息矩阵，115–165 s 的真实匹配面、查询拒绝、地图新建/冻结、视觉 track 及预处理点云。日志异步有界写入，丢记录/IO 失败通过 writer_stats 和独立解析检查；不允许以不完整诊断宣称精确重建。原算法表达式、权重、对应选择和分支保持。

```bash
# 项目根目录；只构建独立两包。underlay 提供既有 vikit/livox 等依赖。
source /opt/ros/jazzy/setup.bash
source multifloor_demo/slam/ros2_ws/install/setup.bash
DIAG_WS="$PWD/multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v7/slam_ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon --log-base "$DIAG_WS/log" build --base-paths "$DIAG_WS/src" --build-base "$DIAG_WS/build" --install-base "$DIAG_WS/install" --packages-select fast_livo2_core fast_livo2_ros --executor sequential --allow-overriding fast_livo2_core fast_livo2_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 新建独立 run，不复用旧 run 或地图；runner 最后加载新 local_setup，验证 /proc/exe/maps。
python3 -B multifloor_demo/teacher_mode/navigation/closed_loop_multifloor_v7/run.py --profile multifloor_diagnostic210 --label height_diag_baseline_r1 --domain 87
```

运行数据在 `teacher_mode/runs/新run/fastlivo_diagnostics/`，二进制布局由三份 `*_diagnostic_schema.json` 描述，与执行源码共同冻结。`diagnostic_baseline.json` 固定72个原件 SHA；`analysis/` 中离线脚本可追加分析，不进入执行链。实际运行和诊断结论见本轮报告，模型加载与构建成功不等于运动、Sim2Sim 或导航通过。
