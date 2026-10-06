# 实际两点动态障碍组件：FAIL，证据保留

本次真实组件采用固定版本 FAST-LIVO2、SCAN、原 Go2 步态及 test-only stop adapter；导航是唯一机身指令发布者。无真值反馈、机身瞬移、阈值或超时放宽。所属进程组 4011732 已清理。

第一点在 69.827 仿真秒由真实 NAV 确认。第二点在 159.844 秒因原 90 秒合同失败；最近原始 SLAM 距离约 1.594 米，未通过 0.22 米稳定 0.4 秒到达。独立初始 SE3 精度 RMSE 0.00350 / max 0.01383 米，不能把失败归为大定位误差。原始结果 17 项检查中 14 项通过，3 项航点完成检查失败。

真实箱子在 70 / 78 / 98 / 106 秒进入、阻挡、离开、清空；已成功设置的实际箱位置记录最大位移 4 米。记录的是 Gazebo 服务成功回执，不冒充独立实体位姿测量。两次原生障碍边沿完整云及同时刻控制输入已提交，所有观测的 held requested/safe 指令均为零。第二次清空后通过新 SCAN 参考恢复，SLAM / 独立实际前进分别 0.03635 / 0.03673 米，超过原 0.025 米恢复证据要求。

恢复后并未被 SCAN 停车轨迹或旧目标持续阻塞：degenerate=0、身份拒绝=0，所有 metadata 原始及调整目标一致。所有 10 条当前目标完整样条重建与实际接受 Path 的点数组最大差为 0。102.365 秒生成的 traj13 长 1.959 米、横弯约零，持续至 144.767 秒；随后的 traj14 长 2.318 米、横弯约 3 毫米。

四次前进后重新转向（116.155 / 126.717 / 136.413 / 147.647 秒）前三次使用同一条 traj13。各前进窗开始到触发时，实际机身航向增加约 0.327 / 0.223 / 0.180 / 0.225 rad，规划切向改变约 0 / 0.00002 / 0 / 0.00439 rad。沿线前进使横向纠偏需求减少约 0.126 / 0.095 / 0.077 / 0.040 rad，两项共同形成负方向误差并触发保留的 0.20 rad 持续保护。实际 SLAM 航向与独立真值在触发时最大差仅 0.000684 rad，不能用方向噪声解释这些实际换向。

独立物理审计显示第二点 walk 20.742 秒、turn 31.730 秒、zero 37.545 秒；行走贡献世界 X +1.304 米，纯转贡献 −0.981 米，净 X 仅 +0.429 米。恢复后的五个行走窗，实际 CHAMP 偏航指令积分均为负，但机身真值净偏航均为正。导航安全行走指令约 62% 时间已达到原 0.04 rad/s 上限，仅增加增益而保持同一上限不能改变这些饱和段。随后 fresh 固定输入 −0.04 物理检查却正确负向响应，说明此处有运动历史/接触依赖，不能宣称固定 CHAMP 符号错误或通用固定低增益。

可复核数据为 [几何审计](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/navigation/test_results/20261001_dynamic_two_goal_actual/navigation_geometry_audit.json)、[原始实际结果](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/navigation/test_results/20261001_dynamic_two_goal_actual/dynamic_result.json)、官方 full Bspline/Path、两个 `navigation_events` 以及 [独立物理审计](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/simulation/test_results/20261001_dynamic_physics_audit.json)。几何审计通过源码 SHA 对照确认 77 个导航生产文件未变；运行状态重算的实际方向与记录差为 0。终态后的 steering 保留最后控制时刻，而 pose 仍接收新传感器，因此不将停止后的不同时间状态作精确重放。

下一候选仅暂存于 `../dynamic_strength_staging`，以单变量行走上限 0.04 / 0.08 做真实两点 A/B；生产默认仍 0.04。固定输入安全通过不等于动态路线已解决，完整 Demo 也未通过。
