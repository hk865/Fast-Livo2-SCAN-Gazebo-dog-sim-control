# Tight S25：固定终点主动捕获与原零命令停车实际对照

本报告只读已完成原件。1个新实际run；先导失败，三次确认未启动。
旧0b78停车FAILED原件保留；新合同允许有界非零Teacher速度命令，不追认旧零命令门。

|run / stage|原收据|capture→hold delay s|正式5s XY / yaw|capture+[.6,5.6] XY / yaw（诊断）|
|---|---|---:|---|---|
|0b78 / old_zero|failed|0.6199999999999974|0.004315m / 0.114522rad|0.004450m / 0.114791rad|
|7f55 / pilot|failed|None|未声明hold，未验证|0.015225m / 0.019639rad|

## 原始物理与指令事实

- 7f55 capture有效物理源戳 37.285000s；持续记录142.720s，首次hold声明：False。
- capture始/末终点误差 0.099740/0.066460m；末yaw误差 0.004771rad。
- 新鲜capture vx请求在+.06上限的比例 26.702%；实际Teacher after-slew capture vx均值 0.058686m/s。
- 最后20s仅作诊断：实际原点body vx均值 0.000000033m/s、median 0.000000008m/s；目标距离均值 0.066460m。
- 原common UNVERIFIED保持：declared_completion_and_termination, fixed_final_goal_parking。缺失receipt NPZ未补写回run，全部曲线由原native/telemetry重建。

这支持当前弱捕获速度包络存在请求非零但实际几乎不前进的响应问题，尚不能单独证明通用Teacher死区、GPU或积分唯一因果。姿态收敛不等于严格25mm目标捕获成功；capture诊断小漂移不等于停车通过。后续候选必须独立登记并实际检验，不能调低入/保持误差或挑安静窗口。

## 源链与相位

所有原收据、其input hashes、已有arrays、runtime、逐run source archives、模型和profile都验证。正式停车首个native state仍关联计算时间不晚于该state的旧缓存reference，保留约5ms相位；未向前绑定新的active_hold命令。
原目标/新目标允许已预登记的1e-12m绑定容差；不是事后重设。完整source/input SHA与每项原判据保存在aggregate.json。

## 实际科研图

![capture真实姿态与速度](01_capture_actual_pose_speed.png)

![Teacher命令与实际响应](02_actor_commands_native_response.png)

![固定阶段窗口](03_fixed_window_stage_metrics.png)

灰色窗口及灰柱是事前登记capture+[.6,5.6]诊断；从未替代新hold首次正式5s窗。保留全部200Hz运动脉动与50Hz命令原始时标，不插值、合成或执行模型。
