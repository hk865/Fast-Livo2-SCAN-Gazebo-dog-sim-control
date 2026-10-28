# 有限安全与停车窗口回归

`test_navigation_safety.py`覆盖当前源关联/纠偏及安全相关小逻辑；`test_parking_status_window.py`覆盖窗口内收到状态、边界覆盖、精确零及代次等负例。测试不启动完整Gazebo路线。

从仓库根运行：

```bash
python3 -B -m unittest discover -s tests -v
```

单元通过不能替代真实传感器、物理执行、动态障碍或固定停车验证。完整观察在[tools/validation](../tools/validation/README.md)。本轮20项回归还覆盖重规划ACK、延迟观察端口和严格不同源戳的停车网格。旧十五项focused tests属于第26轮证据；本轮完整导航另见验证状态。
