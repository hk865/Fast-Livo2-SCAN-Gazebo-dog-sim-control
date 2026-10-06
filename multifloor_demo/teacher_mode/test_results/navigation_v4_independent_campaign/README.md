# 三轮真实有限导航全程证据

三轮独立有限平地区域往返 passed；全局Sim2Sim仍failed。目标中心相距1米，实际机身最大XY离初站偏移0.834–0.858米，区域到达不等于精确中心到达。TTL专门故障注入、动态障碍和多层导航仍未验证。

- `navigation_three_runs_commands_physical_180s.png`：实际50Hz requested/actor vx、wz与200Hz机身COM线速度/角速度，完整180模拟秒。细线是所有原生样本；额外20样本/100ms后向平均线仅辅助阅读，不进入验收。绿虚线标真实原SLAM区域到达，绿带为连续Teacher0命令下3秒停车窗。
- `navigation_three_runs_actual_slam_xy.png`：原始SLAM XY与一次冻结的区域中心/控制半径0.17m/外半径0.22m。实际原SLAM dwell点为各7条。灰线是真值经事后一次固定SE3对齐，仅独立误差诊断，没有返回导航。
- `navigation_three_runs_plot_arrays.npz`：从原JSONL/误差NPZ精确选取的绘图数组。机身COM vx不和world vx混用；原生时标为PreUpdate t−0.005s。
- `navigation_v4_independent_campaign.json`：三轮来源/收据哈希、实际运动/停车/到达、同版NAV与policy源码核验，以及全局保留状态。全部原始generic摘要字节未改。
- `plot_navigation_campaign.executed.py`：本轮绘图脚本原样副本。脚本只读实际日志，不运行Gazebo或ROS。

原始run：`20261004_082258_navigation_slam_scan_roundtrip_v4_r1_9c33`、`20261004_082715_navigation_slam_scan_roundtrip_v4_confirm_r1_1c57`、`20261004_083026_navigation_slam_scan_roundtrip_v4_confirm_r2_bd19`。各run的独立路线收据为 `summary_navigation_independent.json`；原 `summary.json`误套站立窗口仅历史诊断，不作为本导航验收。

图与输入SHA随JSON保存。只读重生成时将 `--output` 指向新目录，保留当前归档不覆盖。
