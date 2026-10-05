# 连续圆圈/S 弯独立追加验收

本模块尚未用于 actual 曲线运行；纯检查不是连续运动通过。只在 root 运行结束后读取原日志，不启动仿真、没有 ROS/关节/位姿写接口、不改旧共同收据。输出 `summary_curve_independent.json` 与 `curve_evidence_arrays.npz` 只创建新文件；任何同名原件存在即拒绝覆盖。

原共同件必须为 `summary_truth_pid.json`、schema `independent_truth_teacher_pid_benchmark/v2`，原 12 共同门、平地适用性及原 overall 全部 PASS。原安全、速度、轨迹、朝向、第一完整五秒停车门限保持不变。缺门、失败、缺样本、未完成或来源哈希错误都不能变成追加 PASS。

追加验收核对冻结的全部源/manifest/profile/path 参数/模型哈希、CPU 单线程、唯一 native 执行器；需要归档 `native/libteacher_actuator.so`，核对运行 binary SHA。Actor 输入 247 维，其中 232 维为本轮 privileged 原生物理/地形反馈，速度命令和上一动作共 15 维已知；不计真实 SLAM、IMU、点云导航替代。

使用原 200 Hz native cached state 时间 `t-dt`，用完整真实 `COM_velocity - omega×COM_offset` 得到原点速度，再按原四元数转为世界系；Euler yawdot 根据 roll/pitch/omega_y/body_wz 计算。全部 native iteration/dt 与原 50 Hz actor/control 覆盖保留。

只从 hash 固定的归档 `core.py` 导入 `StaticPath` 做解析几何，绝不创建 Controller。物理运动、时序、曲线有序进度及连续性独立重建；几何公式与冻结控制器共享，不能称完全独立几何推导。实际轨迹按空间局部投影重建弧长进度，并逐个核对固定 0.25 m 弧长检查点、入口、完整圈/完整 S、出口和终点；起点或圈起止重合不算完成。

弯曲段禁止 pre_turn/turn/settle。所有完整 1 s 原始滑窗的切向原点速度均需至少 `0.7×desired_speed`；沿用原 startup 1 s 排除，末端仅排除最后 0.8 m，并使用原控制记录的因果 progress 决定排除，不找最佳/安静窗口。带符号曲率段按固定几何段累计真实 unwrapped Euler yaw 核对方向，圆圈还需累计完成约一整圈；不因 gait 瞬时摆动反转而直接失败。

另存真实 XY 轨迹在固定 1 s 窗口的二次拟合运动曲率（每 0.5 s），与 body yawdot 区分。S 弯不要求局部圆拟合精确等于变化曲率，该量为诊断；主要门仍是实际完整有序路径、持续前进、正确转向、原位置/朝向/速度门限。原控制日志的 `kappa*v`、几何参数 SHA 和 0.64 rad/s 前馈预算逐帧核对，不能用正确前馈日志当实际转弯证据。

命令（root 需先把本三文件归档到 `RUN/sources/truth/` 并列入原 source manifest）：

```bash
python3 -B /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/curvature_tracking/test_curve_receipt.py
python3 -B RUN/sources/truth/curve_receipt.py --run RUN
```

独立追加收据 PASS 仅表示该次 isolated plane 真值连续曲线运动通过，不意味着当前 SLAM5 多层导航融合、全局最小半径、最佳频率或真机部署通过。所有原收据和失败保留。
