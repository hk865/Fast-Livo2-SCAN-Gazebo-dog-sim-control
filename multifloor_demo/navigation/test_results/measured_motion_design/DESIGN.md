# 基于实际 SLAM 的运动响应观察与候选反馈

这是设计与只读证据，生产增益、速度上限、到达区域、保护和步态未修改。不能将设计当作物理通过；历史 A/B 结果保持原样。

## 当前信息能否使用

`slam/odom_adapter.py` 先由固定 IMU/body 外参变换出机身原点，再对实际 SLAM 位姿差分。`slam/geometry.py::body_twist` 使用当前机身旋转 `R1`：

- `v_body = R1.T @ (p1 - p0) / dt`。
- `omega_body = R1.T @ log(R1 @ R0.T) / dt`。

因此 `/demo/slam/body_odom` twist 是当前 child axes 中的实际位姿有限差分，并没有复制仿真真值或原 IMU 的速度。controller 给 SCAN 发布时又显式将线速度转回 world axes；NAV 当前路径与航向闭环已经使用 SLAM 位姿。新增需求是利用指令与实际运动响应的差值做低频反馈，不能把现有路径闭环说成未使用 SLAM。

现有适配器仅用 `0 < dt <= 1 s` 标志 twist valid；covariance 的 0.04/1e6 是固定可用性标记，并非标定的统计置信度。SLAM body odom 约 10 Hz，原始 IMU 1000 Hz 不等于 1000 Hz 独立位置观察。原始有限差分包含步态摆动和 LIO 更新，不能直接当高带宽速度传感器。

只读重放 `simulation/test_results/20261001_balance_pair2_a_disabled`：3320 条独立 body odom 中，3224 条 active、相邻 80–120 ms 的重构 body twist 与消息 twist 误差 p95 为线速度 3.7e-14 m/s、角速度 1.7e-13 rad/s，验证了坐标与计算口径一致。历史录制 stamp 是浮点秒；本次 round(ns) 只整理重放，不冒称保存了原始整数 header。新 observer 必须直接用 sec/nanosec 整数差值。

按实际 adapter 的六维 `actual_champ_command`（yaw 在下标 5）区分 walk/turn/zero，且排除模式转换前后 0.5 s：

| 实际模式 | 样本数 | SLAM body vx 均值 | 实际 vx 指令均值 | body vx 的 5–95% |
|---|---:|---:|---:|---:|
| walk | 1814 | +0.08153 m/s | +0.11919 m/s | -0.03621…+0.20205 |
| pure turn | 611 | -0.04364 m/s | 0 | -0.21754…+0.10232 |
| zero | 200 | +0.000008 m/s | 0 | -0.04141…+0.06554 |

walk 原始 body yaw-rate 的 5–95% 为 -0.69459…+0.72529 rad/s，实际 yaw 指令为 -0.07528…+0.06882。这是原始短间隔差分，不是稳态 plant gain；正负转向混合的均值也不能证明符号错误。证据 `actual_velocity_response.json` 不消费任何 GT。对应 A 的严格倾角 FAIL 保持不变。

## 建议只增加观察接口

候选 `MeasuredMotionObserver` 不产生指令，接收：

```
observe_pose(stamp_ns, position_camera_init, quaternion_camera_init_body,
             received_wall_ns, frame_id, child_frame_id)
observe_applied_command(stamp_ns, [vx, vy, yaw], execution_mode, adapter_state)
snapshot(now_ros_ns, now_wall_ns)
```

输出 `last_stamp_ns`、`window_start_ns`、`dt_ns`、`raw_body_twist`、`window_body_twist`、`heading_rate_world`、`actual_command_average`、`response_error`、`fit_residuals`、`sample_count`、`age`、`valid` 与明确 `invalid_reason`。world heading rate 与 body omega_z 分开，斜坡 roll/pitch 下不能将二者无条件互换。

采用有限 0.4 s 时间窗，在 camera_init 中拟合真实位置与解卷绕 heading，对速度再转当前 body axes；保留原始差分供诊断。窗长是未验证候选，需要用真实输入误差/延迟对照，而不是既定有效值。新的整数 stamp 必须递增、四元数/位置有限、frame 明确；重复不推进健康时间，clock reset 清空。实际采样间隙、wall age、最少样本和拟合残差联合标出不可用，不能仅凭固定 covariance 伪造“高置信”。不改变原始 SLAM 云、导航 raw 到达、独立评估或原安全 watchdog。

观察窗与实际执行阶段必须匹配。hold、归位握手、纯转、转后 settle、前進分别记录；转换后清空反馈修正，不用停前速度驱动停后恢复。无可靠窗口时禁止附加速度修正，实际 NAV 原有健康/保护仍决定是否可动。

## 最小反馈候选及限制

先观察，后单独试验低频、限幅 P 型 yaw-rate 反馈；不引入积分器，不增加 .08/.12 yaw 或 .12 forward cap，不添加负向速度，不改 .20/.55 heading brake。只能在已接受有效 SCAN、drive phase、所有保护 ready、实际 applied command 确认时产生候选修正。将路径期望 heading 的低频变化率、heading error 和实际 heading rate 清晰区分并记录，避免把路径横向纠偏角当速度。

前进指令多数已接近 .12 m/s。将 .11626 增至上限 .12 仅 +3.2%，不能修复过去真实 .048 m/s 的速度损失；单纯加 forward P 会饱和，不能据此宣称修好。可先用 SLAM 独立识别连续无进展/反向运动并安全停稳，或验证步态执行器的 actual response。纯转期间实测平移应作为执行异常诊断，不能在旋转时擅自注入前进抵消回退。足接触/关节/姿态反馈与 SLAM 响应可以协同，但不可同时更换两个控制变量再声称单因果改善。

## 先要覆盖的反例与实际隔离验证

1. body yaw=90°，实际向 world X 运动必须映射为 body -Y，不能按 no-slip 将其强解释为 forward。
2. yaw 从 +179° 到 -179° 应得到 +2°，不能差出 -358°；斜坡必须区分 world yaw-rate 与 body omega_z。
3. repeated/out-of-order/间隙/迟到消息不能产生新速度或刷新观察健康；首条消息和只有一个点不能有效。
4. LIO 位置修正导致瞬时大差分应标低可信，绝不能让大修正注入控制；raw arrival 与云仍保留原值。
5. 指令 .12 已饱和时，持续速度误差不能积累隐藏补偿；纯转 vx=0、hold 与归位过程中候选输出仍严格为零。
6. 新 SCAN 身份、drive→turn、stop→resume 必须清空旧 response 窗；仅收到 reference 未真实执行不能标恢复。
7. 同一 actual 曲线的坡上正负 yaw 与步态历史需覆盖；一个 fresh 15 s 对照不能代表先转后走的动态路线。

先纯数学及实际 ROS 接口验证观察者 frame/整数时间/模式/限幅，再在 sole73、同 source/runtime/真实 SLAM+SCAN+保护负载下做短的固定混合运动和“先转再走”两种历史，唯一变量是候选 yaw-rate feedback enabled。最后仍需新的完整八点/动态/多楼层路线。GT 只做固定初始 SE3 独立评估，不反馈 observer 或控制器；独立保护、原始到达与预声明区域合同不变。

## Excluded 观察器已落实的范围

`motion_observer.py` 现已实现上述整数时间、有界缓冲和状态窗口 API，但不挂 ROS、不产生 Twist。`test_motion_observer.py` 的 17 个反例通过。0.4 s 窗保留窗口左侧唯一真实支持点，必要时仅在已有双测量间做位置/SO(3) 插值；不外推，不改变导航 raw pose。输出明确 `measured_support_start_stamp_ns` 和 `window_start_interpolated`，避免实际 1 ns stamp 抖动意外丢掉 0.1 s 观测。拟合速度的有效时刻为窗口样本平均整数 stamp，通常约 end-0.2 s，不能当作零延迟当前速度。

`replay_motion_observer.py` 没有消费 GT，按历史 body pose 与实际 CHAMP 指令流重放，共 3236 个 active snapshots：1688 有效、926 拟合残差高、316 执行状态转换、295 观测间隙、7 支撑不足、4 指令覆盖不足。拟合门限 .02 m/.04 rad 是未验证候选，只是质量标签。不能把 52% 有效覆盖当成可用于完整闭环的成熟速度源。

在所有可拟合 walk 窗中，body omega_z 的 5–95% 从原始 -0.694…+0.725 降为 -0.364…+0.389 rad/s；依然远大于 ±0.08 控制量。这是测量摆动与真实运动混合，不证明 plant 增益或符号，也不支持直接加增益。历史 callback wall 是第一次 recorder 输出该 pose 的时刻，不能验证源消息传输延迟；historical full SCAN context 未拼接，使用 None 并标明，不伪造精确身份。新正式运行中的观测能力与反馈效果仍须独立验证。

独立只读审查提醒：`valid` 仅表示观测拟合可用，不能代表执行安全 ready。输出 `feedback_eligible` 固定为 false，另列 `observed_motion_ready`，返回中的 failed/returning 等原始状态保留作诊断；反向指令明确标记 reverse，不归到 zero。没有反向控制或恢复行为。
