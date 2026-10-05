# V5 b182 导航失败与 socket 故障时间附录

本附录复用已完成的原始记录审计，不重新运行模型或仿真。该试验正常初始化、实际通过 16/32 个原始 SLAM 三维区域，其中包括完整 ramp12 上行、二楼出口与 connector_mid；其后导航到达失败与执行器传输故障是两个先后发生的事件。三份正式收据均维持 FAILED。

| 时间与来源 | 实际证据 |
|---|---|
| /clock 130.38s | connector_y 航段激活，原合同每目标 90s，截止时间为 220.38s。 |
| 原 SLAM 头 220.3s；首 failed 状态历史观察 /clock 220.49s | connector_y 原目标 `[14.986783893381528, 4.414263510750097, 1.2056299220538802]`，原测量 `[14.984696739180977, 4.394267576258686, 0.9830298072292131]`。XY 误差 0.02010457m，但高度误差 −0.22260011m，超过未改变的 ±0.1m 门。原区域没有开始连续到达 dwell，90s 超时后停车。状态保留的上一 steering control 时间为 220.345s，并非此次失败事件的独立时间戳。 |
| native 有效物理时间 429.320s | 最后 fault=0 的原生样本：base origin `[16.0067856823, 6.9958352386, 1.5021038901]`，body contact=0，四脚 contact 均为 1。 |
| 原 fault 事件 `t=429.33s`；有效 native 相位 429.325s | code=3，`socket_timeout_disconnect_or_partial_frame`。原生插件锁存故障并进入速度阻尼；Teacher worker 最后完成的时间为 429.32s，21467 帧，随后记 `BrokenPipeError: [Errno 32] Broken pipe`。 |
| native 有效物理时间 429.720s | 首个 body contact，fault 仍为 3。429.33s 之前的原生记录中 body-contact 样本数为 0。 |
| native 日志 `t=439.225s`（有效 439.220s） | 阻尼阶段末样本 base origin z=1.28079935m，body/四脚均接触，fault=3。原全时段独立安全门正确失败：最低净空 0.08079456m、1520 个 body-contact 样本、1980 个 fault 样本。 |

原生日志的 `physics_step.t` 是下一物理边界；状态有效时间为 `t-dt`，这里 dt=0.005s。原故障事件按插件请求时间打印，故 429.33s 与首故障状态有效相位 429.325s 的 5ms 差异符合原协议。失败状态历史附加的观察时钟、原 SLAM 头与保留 steering 时间也分别记录，不以其中一个替换另一个。

传输保护独立于导航新鲜度：归档 native 源 `teacher_actuator.cpp:343–346` 设置 socket 收发超时 200ms，`Transfer` 对一次完整帧设置 200ms **墙钟**截止；`Exchange` 将发送/接收超时、断连或不足帧合并为 code 3 并关闭该连接。导航命令双 TTL 仍为原来的 300ms。不能因这次 IO 故障放宽任何一个时限。

已知因果顺序是“原 SLAM 高度不满足到达 → 导航超时停车 → 更晚的 native socket 故障 → 故障阻尼后机身接触”。现有合并故障码不能区分 send/recv、超时、EOF 或部分帧，也没有 wall-time stage 明细，故尚不能确定断管的唯一根因。CPU actor forward p50=0.26831ms、p95=0.34469ms、max=6.268834ms 只测网络前向，不能排除整帧观测、文件记录或调度延迟。故障前 body contact=0，以及故障之后的真实塌落，不应被改写为“Teacher 策略在坡道上跌倒”。全时段安全失败、未完成上坡/停车仍如实保留。

本次 online SLAM 变体仅改变归档配置的在线 gravity/ba/bg 估计开关；对结果的归因仍限于该单轮记录。Actor 仍为冻结 CPU Teacher，247 维中 232 维为仿真特权输入；导航不使用 live Gazebo 真值。坡道不是楼梯，完整多楼层导航没有通过。

原件及来源链：

- [正式 common FAILED](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/summary_closed_loop_cascade_independent.json)，SHA `aa9b50dcb74b9e6aa3daaebc18cf1ebf628b7f1f55d82b5049cea7f5e493e2e1`。
- [正式 ramp FAILED](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/summary_closed_loop_ramp_independent.json)，SHA `107e591f1ad39e1d7fd0ce3397efd5e6ca301f6446a900ad4ac836b86806ea2a`；下层完整 12m 七项专项通过，上层未完成。
- [正式 heading 来源 FAILED](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/summary_heading_source_independent.json)，SHA `eabc0ea40a06957584604617eec3391f9f14a5219310f2076eddd81048be683a`；两条 parent locked/reference 差 0.0006181474rad 超过原严格来源一致性容差，不能将其当作高度漂移或 IO 故障的唯一原因。
- [原 native 记录](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/actuator.jsonl)，原 common 已验证 SHA `d9ad5bcc646b8129f38c1f1efd0ecb1cabcb8a5f1bbd83ed2025ed566cad90d6`。
- [原状态历史](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/navigation_status.jsonl)，原 common 已验证 SHA `89897fab717c70a8b8073b00f09a52edb8b350bbbc4cd31eed7d0f97214fe178`。
- [原 worker 结果](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/worker_result.json)，原 common 已验证 SHA `2a0a52b8a4afc8bf1d9550961cd91d08800a162b2a227ffb4b35d914b8bebfe9`。
- [归档 native 源](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/sources/simulation/teacher_actuator.cpp:343)，SHA `5602c2051ee7e9f416e0cb0c94bd3efd404bdd7dea1281c891fcc38c936327ba`。

本附录没有覆盖任何原始日志、合同、收据、源文件或结果。进行中的 V6 尚未纳入本附录，也未在进行中执行重日志评估。
