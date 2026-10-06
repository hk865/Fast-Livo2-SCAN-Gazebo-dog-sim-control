# V12：独立 Teacher / 真实 SLAM–SCAN / 无锁 LIO 四线程

本目录是仿真实验候选。冻结 Teacher 保持 CPU 单线程、50 Hz 推理；关节执行仍由唯一原生 TeacherActuator 持有。导航反馈来自实际 SLAM/IMU 和实际 SCAN 点云路线。Actor 的 232 维特权状态/高度输入仍已明示；Gazebo 真值只用于离线验收。三层连接为坡道，未验证真机和真实楼梯。

## 实际执行的软件

本目录的 `slam_ws` 是 V11 源码的普通独立副本。残差循环使用每点独立 byte 标志代替 packed vector<bool>，移除相应共享锁，并保留逐点浮点表达式、只读地图查询和按点索引串行合并。只 `voxel_map.cpp` 定义 `MP_EN;MP_PROC_NUM=4`；VIO 显式循环仍串行。原 O3/unroll 设置保留，未启用 fast-math。实际 core SHA256 为 `8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed`。

有限验证包括 -j2 独立构建、18 次 synthetic LIO/VIO 开关对照、3 次计时开关数值对照、41 项控制测试和大点数线程矩阵。相同核类型上的 state/cov 和有序残差一致；跨 P/E 的极小浮点差异及合成测试限制保留在 `../../test_results/lidar_density_rate_20261005/lockfree_residual/REPORT.md`。实际速度、SLAM 高度、路线、停车和时效是否通过，以每个实际 run 的独立报告为准。

## 日志与保护

详细 solver/query/VIO 复制只在仿真时间 [115,118] 秒内启用，窗口实际通过 profile 传给 logger。kind300 为单独可选的只读边界计时，最多每墙钟秒一行。各段 wall/thread CPU/process CPU 具有嵌套和并发关系，字段说明见 `boundary_timing_diagnostic_schema.json`；不要把段耗时相加当独占总量。

每次真实速度命令发布都记录 `navigation_command_publications.jsonl`，包括 PID 之外的保护零命令。300 ms 源时效、唯一执行器、异常停止和停车过渡保持原合同。不要手动另起速度发布器。

## 操作命令

在 `teacher_mode` 目录运行（只操作本次自有进程）：

```bash
/usr/bin/python3 -B navigation/lidar_sampling_v12/run.py --profile l64_r30_c30_lockfree4_timing_detail3s_210 --label v12_timing_preflight --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --prepare-only
/usr/bin/python3 -B navigation/lidar_sampling_v12/run.py --profile l64_r30_c30_lockfree4_timing_detail3s_210 --label v12_timing_actual --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --domain 86
/usr/bin/python3 -B navigation/lidar_sampling_v12/run.py --profile l64_r30_c30_600 --label v12_full600 --run-storage-root=/var/tmp/go2_teacher_simulation_20261005 --domain 86
```

私有 ROS domain 如已占用，启动器会拒绝。`--prepare-only` 只冻结生成资产、实际源码/二进制/模型/配置和 scope，不能当作实际运动通过。600 秒 profile 保留原 32 个目标和验收门，kind300 关闭；20 Hz 完整候选为 `l64_r20_c20_600`。V12 未采用 V14 的 CPU 分组。

显式外部存储从创建起就使用真实 `/var/tmp/go2_teacher_simulation_20261005/<run>`；根目录要求当前 UID、0700、无 symlink，并在项目 `runs/<name>` 建唯一 alias。不传参数则仍在项目 runs。所有数组均写在 canonical run 内，旧日志不移动或覆盖。

## 验收层级

接口/有限数值验证和实际闭环验收分开。完整路线要求全部 32 区域、真实 SCAN 路线、实际源时效、控制发布链、姿态/接触/关节限幅、实际停车与数据完整性同时满足冻结 criterion。一次前缀成功、构建成功、scope 通过或零 native fault 都不足以宣称完整多层导航通过。原失败与未通过项继续保留。
