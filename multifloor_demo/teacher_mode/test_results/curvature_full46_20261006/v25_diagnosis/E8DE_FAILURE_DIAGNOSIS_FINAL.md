# V25 e8de：第 12 区停车与高度偏离诊断

**原结果保持 FAILED：11/46。第 12 区尚差约 2.43 米水平距离，随后触发原 90 秒超时；没有任何地形层切换。** 正式 22 项检查仍为 10 通过、4 失败、8 未验证，未修改原报告或判据。

可以确定：SLAM 高度在约 218–221 秒相对实体位置出现约 0.2 米偏离，实体机身并未同量下降。SCAN 随后反复规划失败、发布紧停，控制器拒绝没有几何进展的停车样条并持续零命令。现有证据不能进一步证明高度偏离直接造成哪个占据体素碰撞，也不能把 SCAN 碰撞提示等同实际物理碰撞或保护失效。

| 同时间窗口 | SLAM camera_init z 中位数 | native base_link 世界 z 中位数 | 固定初始 SE3 映射后的 native z 中位数 |
|---|---:|---:|---:|
|217–218 s|1.222882 m|1.500858 m|1.201892 m|
|219–220 s|1.102386 m|1.499243 m|1.200675 m|
|225–230 s|1.007224 m|1.502314 m|1.203915 m|
|285–289.45 s|1.004763 m|1.501770 m|1.203368 m|

转换只用初始 5.745–6.570 秒的 26 对实测 SLAM/native 位姿求一次固定 SE3，之后不再拟合。初始位置残差 RMS 为 `[0.885, 0.558, 0.330] mm`；1366 个后续源位姿按真实 SLAM header 与 native `state_physics_world_time` 配对，native 插值间隔最多 20 ms，最近快照距离最多 10 ms，无外推。native position 是 **base_link 世界原点**，没有再减 COM 位置偏移；只有速度字段使用 COM 速度。另保留冻结路线 yaw 加初始平移的敏感性比较。这是离线诊断，不向导航输入真值。

220.10 秒的最后一次实际 PID 仍为 `drive`，命令约 `[0.27035, -0.04251, -0.00296]`，没有 failure latch，路径 236 尚余 2.02257 米。220.31 秒首次状态记录“原地停车样条”拒绝；之后至 289.39 秒的 457 个 running 状态均为严格零命令、accepted trajectory 为 None，`obstacle_hold` 全部为 false，累计拒绝 34 次。289.61 秒首次记录第 12 区超时失败。因此长期停车并非外层 LiDAR obstacle_hold 或曲率约束持续锁死，而是此后未再取得可接受的、具有几何进展的 SCAN 路径。

SCAN 原日志记录 25 次 `A-star failed; aborting optimization`、34 次 `Replan failed 50 times; emergency stop and wait for a new target`、594 次 `Obstacle discovered; emergency stop in 0.000s`。源码中相应紧停函数构造六个相同位置控制点；控制器进度检查拒绝这类未到目标的停车样条并清空路径。原日志 wallclock、行号、offset 和每行 SHA 均保留；附属双时钟近似配对将首次 Astar/fail50 放在约 219.790/220.099 秒，不能当成精确仿真时刻或体素因果证明。

最后三份**成功归档**的实际 SCAN NPZ 为 234、235、236，水平弧长分别 2.42672、2.46356、2.47145 米；全系数、结点和采样点都已保存到紧凑 JSON。三份原目标与 adjusted goal 相同。它们不是紧停样条：被进度检查拒绝的输入在成功归档之前已经 return，所以缺少实际停车样条的完整原系数，不能用这三份有效路径代替。控制器终点前的最后明确到达检查时间为 289.445 秒，SLAM 相对第 12 区中心误差约 `[-0.13234, -2.42984, -0.19920] m`。

没有足够证据把估计高度偏离进一步归因于 gravity/bias、地图残差或某个变换错误；也没有保存首个失败占据查询的体素/表面及当时局部地图，可证明其直接导致 Astar 失败。下一步最小取证应补这两端的对应输入，而不是先放宽高度、碰撞、90 秒或到达判据。此次 terrain provider 一直为 initial、failure=None、completed transitions=[]，因此 V25 地形记录修复尚未经过实际切换；后续阶段与停车均未验证。

![Fixed frame height comparison](E8DE_HEIGHT_FIXED_FRAME.png)

![Region 12 commands and repeated stop rejection](E8DE_REGION12_COMMAND_STOP.png)

来源：`E8DE_STREAM_RECEIPT.json` 记录每条原始流唯一完整扫描的 SHA/字节数与事前单行 schema 预览；`E8DE_FRAME_AND_STOP_DIAGNOSIS.json` 保存完整同时间转换、关键时间点与统计；`E8DE_LAST3_SCAN_GEOMETRY.json` 保存最后三份 NPZ 全部数组；`SCAN_STOP_SOURCE_AUDIT.*` 为独立源码复核；`SCAN_LOG_CLOCK_JOIN.json` 明示近似时钟配对限制。两张图同时提供 SVG，绘图源只读取 compact，SHA 见 `PLOT_PROVENANCE.json`。未重读大原始流、未读 200 Hz trace 或点云、未修改生产源码/运行文件/原验收结果。
