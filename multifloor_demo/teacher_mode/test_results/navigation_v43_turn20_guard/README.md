# V4.3：动态路线转向预算和逐次真实障碍检查证据

本目录是**离线准备与回归证据**，没有启动 ROS、Gazebo、导航或训练，也不是动态导航通过报告。正式来源冻结文件为 `../navigation_v43_dynamic_turn20_exact_guard_freeze.json`（SHA256 `c75a38a4c7ce39362628be67d5aebc3772e52452286ca3bfa8aaf5dd7610077b`）。`runs/20261004_navigation_v43_offline_*` 三目录仅生成配置和构造 LaunchDescription；不得计入实际运行次数。

V4.2 的真实动态实验 `20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7` 已正常完成240秒，六个自有进程和15个 launch 子进程正常退出，但返回阶段超过原90秒期限。返回约78秒处于转向阶段，实际非零平移仅7.10秒；实际转向有92次命令过期后恢复，受零命令和加速度恢复影响。完整 `.12rad/s` 命令片段仍能被 Teacher 正确跟踪，因此不能归因为错误转向方向或仅硬件资源竞争。原40.9秒去程到达、131.1秒返回失败和清除证据不足全部保留。

新实验只把动态路线的对齐转向上限从 `.12` 改为 `.20rad/s`。正向18秒真实 Teacher 校验 `20261004_094018_turn_positive_yaw02_navigation_calibration_r1_789a` 已通过：5–10秒实际角速度均值 `.1897235909rad/s`，15–18秒停车漂移 `.0010337042m`、偏航漂移 `.0239196153rad`，四个自有进程退出0。它提供命令范围依据，不能代替动态导航实测。

三种选择器分别为：

| 选择器 | 导航 experiment | 对齐转向上限 |
|---|---|---:|
| 原普通平地 | `flat_relative_roundtrip_v1` | `.12rad/s` |
| 原动态 | `finite_flat_dynamic_stop_resume_v1` | `.12rad/s` |
| 新动态 | `finite_flat_dynamic_stop_resume_turn20_v1` | `.20rad/s` |

新 runner 参数为 `--navigation-dynamic-turn20`，隐含动态和完整导航栈；作用于一次新 run 的240秒实验。配置准备对应 `navigation/scoped_profile.py --dynamic --dynamic-turn20 --run RUN`。物理障碍 scope 的 experiment 仍为 `finite_flat_dynamic_stop_resume_v1`，新增 `navigation_experiment` 与 `.20` 上限必须匹配父导航收据。该 scope 只是允许实验，状态一直为 `experimental_unverified`。

```bash
python3 scripts/run_test.py navigation --navigation-dynamic-turn20 \
  --navigation-duration 240 --label slam_scan_dynamic_turn20_v43 --repeat 1
```

执行命令以 root 编排为准；本目录没有执行上述命令。90秒每目标期限、`.17m` 控制到达半径、双时钟300ms命令TTL、1秒连续真实点云清除、最大观测间隔300ms、5秒真实停车及原停车阈值均不变；actor、247维构造、训练关节映射、PD和物理步长没有变化。

逐次真实检查日志为 `navigation_guard_history.jsonl`。它使用原 `steering_obstacle_ahead` code object，通过私有函数命名空间记录原本两次 `obstacle_ahead` 返回值，不重新计算走廊，也不改返回值。完整字段定义冻结于 `navigation/dynamic/guard_trace_schema.json`。每条包含真实 SLAM 的整数时间戳、姿态、实际自体过滤输入、实际 immutable filtered XYZ 的 SHA256、路线、方向、目标和两次原走廊结果。`filtered_xyz_float64_sha256` 是实际过滤后 XYZ 的 little-endian float64 SHA，**不是原 PointCloud2 消息哈希**。独立复算必须使用 passive observer 保存的真实点云和当次过滤姿态，先匹配该 hash，再重算两走廊及 union；仅保存 hash 不足以证明几何清除。

日志经原 bounded asynchronous writer 落盘，无 callback fsync。严格要求 `navigation_guard_writer_receipt.json` 为 `drained`、无 queue_error、实际连续行序列完整且行数等于 expected_records。缺失原XYZ、来源时间过期、hash 不同或 queue 错误都不能宣称清除通过。模拟器 `moving_obstacle` 的真值姿态只用于独立确认障碍实体入场/撤回，不能成为导航输入。NAV 可在实际点云走廊清除且新 SCAN 检查通过时于 leaving 阶段恢复；mover 的 clear 状态只表示实体回到初始位姿，两者分开验收。

104项离线检查已重新执行：新28项（6000步原普通平地全部状态和输出相同、800例原障碍函数结果和类型完全相同、每次仅原两次几何计算、实际 controller hook 与异步 after-control 记录），原37项接口边界、9项实际 controller 方法故障检查、30项物理 mover/资产边界均通过。三选择器的配置、来源哈希与 LaunchDescription 构造均通过；未运行 LaunchService。先前15项 atomic transport 和8项 direct service bridge 清理检查通过证据继承，相关实现没有变化。

下一轮总体通过必须同时满足真实阻挡与停车、实际撤离/点云连续清除、新SCAN及复走、两个原SLAM区域在原期限内到达、终点实际停车、六个自有进程退出0和所有已启动子进程正常清理。旧6614的 `navigation_stack=-15`、10c7的返回期限失败/清除证据不足、全局原42项验收及完整坡道失败收据均不覆盖。`mock`/本目录中的 guard JSONL 是离线测试伪状态，绝不作为真实传感器日志。
