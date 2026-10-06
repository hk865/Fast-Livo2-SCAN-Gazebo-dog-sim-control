# 单次实际命令生产者中断夹具

准备阶段仅编译、纯mock自测和独立源码审查，没有启动ROS/仿真或发送真实信号。执行协议已事前冻结：fixture SHA256 `3741fdd1168da5098ee9e87b097e0b614698994850c8a66b0fbeca772c62dcce`；protocol SHA256 `b01ae8502bfb3a173291c99767c7be719d8a163f2bde04d84cf780cfe3c4b7ac`。

先在独立终端启动夹具，使用新的输出目录：

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/ttl_bridge_pause_v4.py \
  --label slam_scan_ttl_drop_v4 \
  --output /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/ttl_bridge_pause_v4_actual_20261004_r1
```

随后在另一终端沿用已通过的V4有限平地导航启动命令，将`--label`设为`slam_scan_ttl_drop_v4`、`--navigation-duration`设为180。夹具排除它启动前的全部run，仅接受之后新创建、该label的唯一navigation run。180秒墙钟内未出现真实移动触发便失败退出，不会拿已有任务补测。

触发依据为原始`navigation_slam_poses.jsonl`里的机身前向vx≥0.03 m/s，真实stamp连续至少0.3 s（间隙≤0.2 s），同时实际导航命令文件healthy且前向vx≥0.08 m/s。两者使用真实源时间与接收墙钟，导航姿态/命令来自SLAM/SCAN；不读取仿真真值位置、速度或Teacher telemetry来触发。

目标必须是唯一同用户`navigation/bridge.py`进程，argv中精确匹配本run的acceptance与command-file，环境ROS79、Gazebo partition、DEMO_RUN_DIR以及navstack祖先匹配。记录PID、内核starttime、argv，并用pidfd固定身份。仅该PID接收SIGSTOP；確認T态后重新读取实际sensor_gate的`/clock`戳，至少10仿真秒后SIGCONT。Gazebo、SLAM、SCAN控制器和Teacher不收信号。原命令文件字节、mtime和源时间戳保持原样，夹具不发布/写速度或关节。

运行后保存`events.jsonl`（实际SLAM速度与clock、原始完整envelope、暂停/恢复状态与信号）和`receipt.json`（目标身份、原envelope、真实暂停时长、恢复与失败原因），同时拷贝执行脚本和协议。退出或SIGINT/SIGTERM/SIGHUP时finally同身份SIGCONT；PID重用、节点已退出/Z态不能称恢复。时钟停滞60墙钟秒、输入消失、场景提前结束、进程身份不符等都失败并尝试恢复。不可恢复时如实记录，不向其他PID或进程组发信号。

`injection_completed`仅表示中断/恢复夹具完成；运动→TTL过期→Teacher物理停车、原SLAM连续、恢复门禁、路线继续与停车由独立原日志分析验收。

2026-10-04已实际执行一次：`runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/ttl_fixture_evidence/`保存原始协议、执行脚本、事件、收据和独立来源审计。实际唯一bridge PID1744350（starttime31801905）经确认T态后，在真实`/clock`7.43至17.43秒保持暂停10.000仿真秒并恢复；暂停期间命令文件字节、mtime和原始时间戳不变。原envelope自身sim_time为7.475秒，不能用7.43秒的gate相位替代其TTL起点。恢复后首个envelope仍为unhealthy零命令，随后重新取得实际有效来源。该来源/信号收据不单独证明物理停车或导航通过；对应运动验收须引用同run的独立分析。

离线自测可独立重复；它只向mock sender记录符号，不发真实信号：

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/ttl_bridge_pause_v4.py --self-test
```
