# V18 同条件 210 s 前缀独立验收

运行：`/var/tmp/go2_teacher_parallel_20261005/20261005_232930_closed_loop_cascade_clock_hold_combined_v18_prefix_r1_ac02`。原 common、publication v2、ramp 总状态仍是 failed；有限前缀运动/来源/执行必需门通过，不能称完整路线通过。

实际 worker 运行到210.0s，共10501条、faultnull；runner与CPU sampler均退出0。真实SLAM三维区域25/32，原控制半径/驻留/来源年龄门未修改。没有物理期 first_failed_guard。

原 common 的物理/清理边界、冻结来源、实际SLAM/IMU因果更新、真实SCAN payload、固定目标与原有界路径投影、实际cloud字节、movementguard原几何、双时钟TTL与slew、执行数学、native200Hz、安全/力矩/支撑和唯一Teacher CPU/关节执行全部通过。publication v2原新增七项也全部通过。

第一条 ramp12 原完整12m、COM入坡到出坡、四足有序接触、12m各分箱、连续落地及原SLAM到达正确物理楼层通过。第二条 ramp23 在163.85s入坡，210s尚无出坡证据，完整第二坡未通过。坡道不是楼梯。

LiDAR和RGB回调各6364帧，平均30.304Hz；LIO处理6254帧、VIO处理6253帧，初始化后约30.303Hz。真实数学控制3599行、平均17.708Hz；20Hz ticker不等于每次都运行PI。optimizer详细记录仅115–118s各91帧、约30.252Hz，窗外不记为0。

未完成：全部32区域、固定5s停车、第二完整12m坡道。原provider status整行比较的已知metadata join问题仍保留UNVERIFIED；前缀不运行完整25门metadata合成来冒充通过。

CPU/boundary对照由独立performance_actual报告：原全窗首样本simNone且SLAM尚未启动，端点交集只有runner，不能把该值当SLAM或整机CPU。不同场景同一仿真秒不保证空间状态完全相同，无法隔离组合内两个模块的实际收益。

Actor仍有232维特权仿真输入和15维已知命令/上一动作，CPU单线程50Hz；导航反馈使用实际SLAM/IMU/SCAN，真值只作离线验收。

[原门覆盖收据](PREFIX_MANDATORY_COVERAGE.json)；原始所有收据均保留且未改阈值。
