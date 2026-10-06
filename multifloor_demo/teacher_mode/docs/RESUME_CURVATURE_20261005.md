# 后续接续：实测曲率控制（2026-10-05）

当前物理批次已结束。先读当前README、current_status.json及TRUTH_PID_CALIBRATION_REPORT_20261005、SLAM_CONTROLLER_TRANSFER_REPORT_20261005、CURVATURE_ENVELOPE_REPORT_20261005；08点旧交接及旧失败不覆盖。仅CPU单线程冻结Teacher，不重新训练、不停止另一任务训练/评估、不改通过的camera_mode、不操作真机。

已完成：真值A2/B/C27次、冻结后D/D2运动35次（含原ramp23完整上下坡各3次），实际SLAM V5六米单程/往返6次，plant转弯36次，连续曲线14次。连续曲线drive14/14通过；完整项13/14，tightS25固定5s停车yaw0.114522rad超0.1，保留原FAILED。原全局acceptance仍为历史失败，新控制律Isaac对照未做，不能升级通用Sim2Sim。

曲线冻结入口navigation/curvature_tracking/，curve_freeze_v1.json及每run sources/runtime均归档。真值标定选定cascade_pi25Hz与新curvature_cascade_pi是两个明确版本；后者加入解析空间曲率κv前馈、连续行走，不含原折线停转门。二者相同COM速度PI和路径/朝向PD增益，策略50Hz/nativePD200Hz不变。所有TTL300ms、唯一执行器、限幅/反积分饱和及原验收门保持。

优先调优方向：

1. 为完整圆圈的切线连接加入事前冻结的曲率渐变/变化率限速候选，与原R0.6左右各三轮对照；原前馈跳变与wz slew限幅同时出现，不预设提升频率能消除。
2. 停车另立主动速度、位置及朝向保持契约。当前goal_dwell/parking绕过PI输出零，Teacher仍持续零速度推理，不能称为主动pose hold。新的契约仍验证首个固定5s窗和到达来源，不偷偷改变旧全零命令验收或挑更安静窗口。
3. 在基础完整项通过后，将新连续curve core接入实际SLAM/IMU源并保留同源时间/坐标/TTL审计；现有SLAM V5只验证旧core、10Hz有效新pose及初始机身相对平直线，不包含曲线/SCAN/旧地图注册。
4. 原场景路线注册、局部点云替换187高度、SCAN、动态障碍及多层坡道分别推进。三层连接是坡道，不称楼梯。Actor232维原生特权观测仍需明确，独立30维传感替换没有与这些新路线联测。

现有只读浏览器8769展示真值曲线原始轨迹/相机/原门，8770展示实际SLAM六轮；8768保留旧有限SCAN/动态结果，8767仍为原相机Demo。gentle S固定overview中段12–20s可见真实机器狗，末段走出视野空帧保留。两路实际CameraInfo D全零，不是漏做镜头畸变校正。

所有失败、源链、模型/配置/逐轮源码/相机/物理日志在run与test_results中保留。当前报告全部完成后才更新PACKAGE_MANIFEST；脚本只做清单，原acceptance不改写、不自动PASS。目录名字含plant_radius_prepare的36轮有实际runtime，已改用自run源码/执行器哈希身份绑定分类，不能当作仅准备或按名字判PASS。
