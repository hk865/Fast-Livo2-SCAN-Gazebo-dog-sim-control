# 新 PID / CHAMP 同场导航协议

本目录是新测试范围，不修改历史 acceptance、相机 Demo 或封存结果。`protocol.json` 是事前判据，`cases.json` 是事前路线及期限；首轮运行后不得修改这两个实际 runtime 引用。新增阶段或控制器修订应使用新的冻结版本和运行目录。

先做 `flat_short_1m_roundtrip`，再做 6 m 直行和往返、原宽四个单坡方向。连续两坡案例暂缓。各实际案例需三次同源通过；CHAMP 没有新实际通过证据时保持未验证。

每轮保存 `pid_navigation_protocol.json`、`pid_navigation_case.json`、`pid_navigation_freeze.json`、实际源快照、SLAM/SCAN/PID/guard/命令/200 Hz 物理证据。由根任务串行启动仿真，分析器不运行 ROS 或 Gazebo。

完成运行后独立验收：

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_pid_navigation.py /absolute/path/to/new/run
```

分析器只新建 `summary_pid_navigation_independent.json` 和 `pid_navigation_independent_arrays.npz`。`--no-write` 可只读复核，历史通用摘要保持原样。区域证据使用原始 SLAM；真值只作独立路线及安全分析。缺数据不通过；先加载模型或默认站立均不能算完成路线。

完整口径见 `docs/PID_NAVIGATION_REPORT.md`。本轮 Actor 仍使用已明示的仿真特权输入；导航定位来自真实传感器 SLAM。坡道不能称楼梯，仿真结果不能视为真机部署通过。
