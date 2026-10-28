# 独立导航验证工具

本目录执行开发验收观察；与日常入口分离。`run_navigation_check.py`启动根`start.py`，观察SLAM、命令/执行及障碍，最后向自己拥有的入口发送SIGINT。它会移动实际Gazebo `moving_obstacle`，因此不是无副作用的只读查询。

## 使用

```bash
python3 -B tools/validation/run_navigation_check.py --output runs/validation_new --wall-timeout 1800
```

每次选新的输出目录；依赖与普通入口相同，需可用ROS/Gazebo/PyTorch及本地资产。本轮已用新构建产物执行完整原场景一次并通过，详细资源与边界见[验证状态](../../docs/验证状态.md)。单元回归可用根`tests/`，不需要启动完整路线。

## 分工与规则

| 文件 | 职责 |
| --- | --- |
| `run_navigation_check.py` | 独立运行生命周期和结果汇总 |
| `navigation_observer.py` | 当前SLAM/控制、路线推进及固定停车观察 |
| `obstacle_observer.py` | 测试障碍控制及真实障碍观测 |
| `parking_status.py` | 只检查固定窗口内实际收到status的覆盖/零命令 |

每个真实stop都检查零hold、实测停止、≥10次真实clear跨≥1s及实测恢复。停车锚定第一次actual-zero+几何资格T0，不重选；5s/10Hz、XY/yaw漂移≤.05m/.1rad，源龄/状态gap≤300ms。不得加入高频速度veto、插值或将T0前减速status搬进窗口。机器人真值不是导航/到达权威，障碍真值只用于独立观察其实际移动。

调试区分应用失败、数据覆盖失败和observer实现错误；原失败不因之后修正而重标PASS。第26轮既有结果见[phase_summary](../../docs/phase_summary.md)和[机器摘要](../../docs/validation_summary.json)。它只验证该次范围，不能外推毫米级全局精度、全46区或重复可靠率。
