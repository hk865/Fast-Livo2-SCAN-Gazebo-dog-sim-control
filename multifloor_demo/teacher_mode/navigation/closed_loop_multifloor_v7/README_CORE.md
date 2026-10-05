# 实际 SLAM / SCAN 串级控制核心

这是仿真中拟接入的纯控制器；当前只有离线检查，尚未宣称真实 SLAM / SCAN 闭环或多层坡道通过。无 ROS、Gazebo、Actor、关节执行或训练接口。Teacher 继续由唯一执行桥 CPU 推理；本核心仅生成 `[vx_COM, vy_COM, omega_z_body]` 速度请求。

## 接口

`Controller(config, fixed_goal_xyz, fixed_goal_yaw_rad, goal_id)` 构造一个固定的 `camera_init` 目标，深复制并计算 SHA256。目标必须由包装器从真实 SLAM 坐标注册，不可用 Gazebo 真值，也不可在每次到达/漂移时用测量位置重设。一个新目标使用新的 Controller 实例，包装器先安全停车再切换。

`set_path(points_xyz, path_id, stamp_ns, received_wall_ns, frame_id='camera_init')` 只接受实际审核通过的 SCAN 路径。相同 ID 不可变；新的 ID 只清零几何进度，不重置 PI 或滤波。路径生成时间是来源记录，不是 300 ms 超时源；包装器必须持续用新鲜点云、SCAN 健康和运动保护确认该路径仍合法。Capture 以后固定端点不接受新路径。

`update(feedback, imu, ack, clock_ns, wall_ns, guard_reason=None, mode_override=None)` 返回 `(command_body: ndarray[3], row: dict)`。时间全部为原始整数纳秒，墙钟为原始 monotonic 纳秒；重复读取不能更新来源接收时间。

| 输入 | 必需字段 |
| --- | --- |
| feedback | `stamp_ns, received_wall_ns, frame_id='camera_init', position_world_xyz, quaternion_wxyz, origin_velocity_body` |
| imu | `stamp_ns, received_wall_ns, frame_id='body', angular_velocity_body` |
| ack | `stamp_ns, received_wall_ns, sequence, requested_command_body, applied_command_body` |

姿态四元数必须是 **wxyz**，速度为实际 SLAM body-origin 速度在 body 坐标中的分量。IMU 必须先按已审计安装变换转为 body gyro。Ack 是执行桥确认已用过的 Teacher 速度输入，不能是拟发送输入；同一 ack 帧中的 `requested` 和 `applied` 是同一次请求及实际限幅/变化率处理后的输入，不能跨帧配对。包装器从执行桥原始滚动记录中选择最新 **ack_stamp ≤ pose_stamp**，不读取仿真真值位置。

严格 `imu_stamp ≤ pose_stamp ≤ clock`，IMU 配对间隔默认 20 ms；`ack_stamp ≤ pose_stamp`。Pose、IMU、ack 都有 sim 与 wall 双 300 ms 有效期；来源不得来自未来。新鲜的 **不同 SLAM header** 才更新 PD、滤波、积分、进度和驻留。重复 header 只保持请求，仍核对有效期。先检查前一个墙钟反馈间隔再接受新帧，迟到新帧不能消除既有超时。

`mode_override` 支持 `None/drive/turn/hold/capture`。默认内部 stop-turn-drive：大于 .2 rad 先低速驻留 .3 s，再转向；小于 .08 rad 且低角速度后 settle .3 s。根包装器复用已经验证的 TeacherHeadingGate 时传 `config.external_heading_gate=True`，gate 的 pre_turn/settle 传 `hold`、align 传 `turn`、drive 传 `drive`，避免两套 gate。`turn` 输出零平移，冻结不可执行的 xy 积分；`hold` 或 `guard_reason` 立即请求零速度。最终停车通过真实 SLAM RegionArrival 后可传 `capture`，固定目标必须在 .3 m 内并符合楼层高度门，包装器对实际最终方向重新执行点云/碰撞保护。

## 误差、参考与串级结构

最近点来自静态/SCAN 折线路径的 **三维线段投影**，不是按时间的理论轨迹点。初次投影选择 xyz 距离最近线段，重合楼层用 z 区分；之后搜索单调水平弧长进度附近 `[s-.15,s+.8]`。Cross 是局部水平左法向上的有符号距离；along 记录剩余水平弧长。局部切向定义航向，坡度是 `dz / ds_xy`，前进参考含对应 z 速度。路径前端只决定方向与空间参考，不自漂移；局部路径结束而尚未覆盖固定目标时停车等待新合法规划。

沿用已验证参数：cross PD `.8/.2`，yaw PD `1.3/.15`；COM x PI `.6/.5`、y PI `.4/.3`、body wz PI `.3/.2`；外环 .2 s、内环 .1 s 一阶滤波，积分限幅 ±.5。测量 COM 速度为 `v_origin_body + omega_body × offset`；Euler yawdot 用 roll/pitch 与实际 IMU 转换，命令再转为 body wz。参考 COM 杠杆仍保留旧控制器的近水平假设：参考 omega_x/y=0；真实测量转换使用完整三轴 gyro，坡道上需实际核验该假设的影响。

按 profile 限幅、平面范数约束和 drive 非负 vx 进行条件积分。下游变化率/饱和防积分使用同一个实际 ack 的 `(requested - applied)` 残差，**不估算 Teacher 帧数，不在本核心进行 slew**。根执行桥是最后 `[.6,.6,.8] /s` slew 的唯一所有者；保护请求零速度仍经已有执行桥正常停车、持续 Teacher 推理，不能发送 action=0。

建议首次接口配置为 `desired_speed=.3, command_limits=[.6,.2,.3], external_heading_gate=True`，数值收益尚待实际闭环测试。配置上界保持训练命令 `[1,.4,1]`，不提高训练边界。

## 主动端点停车

普通驱动到固定终点后进入 capture，首次 capture 独立停车积分清零一次。Capture P/D `.8/.2`，世界 xy 参考限 .05 m/s，速度请求限 `[.15,.07,.1]`；active_hold P/D `.18/.2`，参考限 .025 m/s，请求限 `[.06,.04,.1]`。Yaw P/D `.65/.18`，Euler 参考限 .07 rad/s。沿用 3 mm / 5 mrad deadband。

鲜活源驻留 .6 s 要求：xy 进入 .025 m、保持 .03 m；yaw 进入 .035 rad、保持 .045 rad；实际 origin xy 速度 <.03 m/s、Euler yawdot <.06 rad/s。首次声明帧即用 hold 数学，capture→hold 不重置积分。间隔 >.2 s 中断连续驻留，但 300 ms 保护阈值不放宽。记录 `parking_capture_pose_stamp_ns`、`parking_hold_declared_pose_stamp_ns` 与相应 clock/wall；到达实测位置只是诊断，不改变目标。

停车验收是首次 hold 原始 header 对应的固定 5 s 窗：XY 漂移 ≤.05 m、yaw ≤.1 rad、速度 ≤.08 m/s、Euler 与 body 角速率 ≤.1 rad/s，均须独立从实际日志/离线物理记录核验。请求不必全零；这不是旧 zero-command 停车协议通过。任何保护中断保留首次窗口并标记 `parking_window_interrupted`，不可重新挑较安静的窗口。

源短超时及外部障碍保护可在两条新鲜不同 SLAM 测量后恢复；保留固定路径/目标，清零积分/滤波与未完成驻留。非有限反馈、时间倒退或相同 header 数据改变锁定失败，不自动恢复。真正物理停车/失败由包装器与唯一执行桥实施。

## 离线检查

在本目录执行 `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 pure_tests.py`。覆盖 header 更新频率、迟到 before-fresh TTL、因果 IMU/ack、NaN/倒退锁定、重规划不重置 PI、跨楼层投影、坡度/坐标转换、ack 防积分、turn、循环起终点退化及固定停车首窗。它们只核接口与数学安全，不替代真实运动/导航验收。
