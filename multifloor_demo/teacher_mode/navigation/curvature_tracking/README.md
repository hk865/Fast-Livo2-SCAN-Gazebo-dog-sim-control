# Teacher 连续曲线跟踪（独立实验模块）

本目录为新实验，使用 Gazebo 真值作反馈，仅在仿真平地标定。未启动仿真、未声明圆弧/S 弯通过、未选定可行最小转弯半径，不计 SLAM 融合或真实机器人验收。原 v3 直线/折线控制器、原收据及通过的 camera_mode 均保持原样。

`core.py` 提供 `Controller(profile).update(native64_state, elapsed_s)`，输出有界 `[vx, vy, body_wz]` 和 mode，状态详情在 `controller.row`。不导入 ROS、不发布关节目标、不写仿真位姿、不创建执行器。实际接入仍需 root 的独立曲线 worker/runner 归档本目录源码；`teacher_sim::TeacherActuator` 必须保持唯一关节力矩执行者。

静态路线为平地 3D 曲线，参数及其 canonical SHA256 固定。圆弧为完整一圈，支持正负方向，带入口/出口切向直线，圈后航向与入口一致。S 弯以弧长 `u` 定义 `theta(u)=sign*A*sin(2*pi*u/L)^3`，解析曲率为 `dtheta/du`，位置是 `[cos(theta),sin(theta)]` 对弧长的确定积分。64 点 Gauss 数值计算该解析积分；切向和曲率直接使用解析公式，纯函数测试核对导数误差。入口/出口切向一致，S 端点曲率为零。

每次新鲜反馈在前次弧长附近做空间最近点投影，承认的弧长进度单调不退，局部窗口以米定义；不根据时间生成理论位置点，不全局跳到圆圈重合的后段。圆圈只有 `progress >= full_length-0.15 m` 且终点低速/朝向/距离满足条件才进入停车 dwell，防止起点或第一次路过相邻末端误判完成。

外环沿切向给速度，法向使用冻结的横向 PD，朝向使用冻结的 heading PD 并新增 `kappa*v` 角速度前馈。D 项跟踪误差导数，使用 `kappa*v_actual_tangent/(1-kappa*cross)-yawdot`，避免恒定转弯时把前馈抵消。连续曲线没有旧版 `|yaw_error|>0.2` 停车/原地转向门；不会把分段 stop-turn 计为连续曲线通过。

内环保持当前冻结默认 CAS25 的 COM 速度 PI、机身角速度 PI、0.2/0.1 s 测量低通、积分限幅与条件抗饱和。空间位置使用 link 原点，COM 速度使用 `omega×COM offset` 修正；Euler yawdot 与 body wz 按实际 roll/pitch/omega_y 转换。原点速度测量使用完整实际 body omega；构造 COM 速度参考时沿用期望 body omega_xy=0 的平地假设，同时 Euler yaw-rate 换算仍使用实测 omega_y，日志明示该假设。执行桥仍每 20 ms 限加速度 `[0.6,0.6,0.8]`，Controller 仅估算同一 slew 以阻止加剧饱和的积分；held 帧不积分，dwell/parking 积分冻结。停车持续给零速度命令运行 Teacher，不把 action 清零。

真实反馈支持 10/25/50 Hz，是 Teacher50 Hz 的整数降采样；物理/关节 PD200 Hz 不变。`native64[0]` 为 PreUpdate 世界时间，状态时戳为 `world_time-0.005`，初始 3 s 零速度。超过 300 ms 的反馈间隔或墙钟新鲜度锁定零速度；墙钟间隔在承认下一次 fresh 反馈前检查，迟到的新帧不能先刷新时刻而掩盖超时。异常状态/不递增时钟抛异常，由 worker 与原 native socket 安全保护停止。行走标称速度不超过 0.8 m/s；曲率处再约束 `v <= 0.8*0.8/abs(kappa)`。速度命令范围设为 `[0.8,0.35,0.8]`，比原训练边界保守。这个前馈预算是规划约束，不是实际可行性验收。

`profiles.py` 必须显式给圆圈半径/S 参数。未提供真实 plant 结果时只标为 pending。最终圆圈计划需同速度、同方向已通过的 actual 最小测试半径来源，计划半径至少为其 1.5 倍；“最小”仅是已测试通过的半径集合，不是全局物理极限。S 计划需同速度左/右两个实测来源，按较差方向保留 1.5 倍几何半径余量。构造器核对选中的 `summary_radius_independent.json` SHA、18 个当前 plant 必需门、全部原输入及归档源 SHA、已批准 analyzer/protocol SHA、原 PASS、实际拟合半径、原速度/符号及冻结模型 SHA；缺门不能形成可行性计划证据。最小候选/边界重复选择仍由 root 根据完整 plant 结果形成，不由单次收据推断。提供余量证据仍不自动标记跟踪通过。

纯 CPU 检查命令（不启动仿真）：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/curvature_tracking
python3 pure_tests.py
```

下一步实际流程：先完成 plant `[v,wz]` 连续转弯，获取每个速度/方向稳定实测半径；再固定路径参数与源码 hash，测试有余量半径的正负一圈（入口/完整圈/出口/停车）、低/高变化曲率 S。原始 200 Hz 接触/力矩/姿态与 50 Hz 轨迹/速度/命令全部保存，依据几何参考重新计算误差；任何连续性、命令饱和、越界、停车失败原样保留。真实 SLAM 接入在这些真值实验之后另行验收。
