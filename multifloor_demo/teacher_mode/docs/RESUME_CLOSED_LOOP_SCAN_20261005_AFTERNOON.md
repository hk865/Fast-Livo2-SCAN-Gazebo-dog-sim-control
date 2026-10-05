# 当前闭环接续：2026-10-05 下午

最新人类问题是初始化慢动是否污染 IMU，以及是否借鉴 ORB-SLAM3 关键帧/回环。专门审计见 `SLAM_INITIALIZATION_MATCHING_AUDIT_20261005.md`，实际运动/多层验收见 `CLOSED_LOOP_SCAN_MULTIFLOOR_REPORT_20261005.md`。不要回到只加载模型、旧开环门或真值导航。

## 已执行且不能覆盖的结果

- 实际 SLAM/IMU+SCAN+串级控制平地 54a1、5011 通过，a9a2 首停车窗 60.75 mm 失败。
- V4 4791 单次完整12m ramp12、14区域、四脚接触与首5秒停车 **正式通过**，正式停车漂移4.5475 mm，不是早期source初筛3.6mm。
- V4 d42e full32仅16区域，connector_y高度误差216.49mm失败，下一坡道未完成。V5 b182开重力/bias仍222.60mm失败；429.33s传输故障之后的塌落不是前阶段Teacher策略跌倒，完整安全FAIL保留。
- V6 9cc6恢复视觉协方差1000，仍16区域失败：128.4→131.7 s raw SLAM Z下降315.858mm，offline native Z仅下降0.268mm。131.74 clock的原管道高度误差306.505mm>300mm触发保护；Actor继续站立，原600s/30001次CPU推理正常完成、无worker fault、五角色/11子进程全部清理。必须读取该run最终正式收据，不以runtime completed提升导航。
- 原5/10cm持续登台各3次功能通过；原高度比例失败已撤出登台判据、旧数值保留。真值标定/运动与本批实际导航来源分开。

执行源目录 V1～V6、每run源副本、共同/ramp/heading验收器都已冻结。后续任何变更另建新版本，不能改这些目录中的py、profile或旧收据。camera_mode与RL训练/评估不动；仅本任务自有仿真可清理。Actor仍CPU单线程，导航无Gazebo真值，232维Actor特权来源不能漏报。

## 已确定与仍待定位的原因

初始化605样本/3.02秒以前拒绝111次运动，受理窗口仍慢转2.1度；不能称完全静止，也不能用mean gyro扣掉真实转动作为bias。仅收紧到.01rad/s不能在原启动0～6.7秒形成3秒窗口。固定初始倾斜仅约6mm目标误差，不能解释20cm突变。

点云地面仍有约6000点、5.5mm拟合残差。点到平面匹配已有，视觉为灰度光度patch/局部参考观测；彩色发布不是color ICP。1e8视觉协方差严重弱化图像信息，恢复1000仍失败。当前缺少实际solver内部对应/拒绝/信息矩阵，不能从raw cloud平面proxy推断内部Hessian；未发现全局回环调用链，首次平台失效不能依赖尚未发生的回环修好。

## 下一项具体工作

在**独立复制源码及独立编译/运行版本**中加最小可观性诊断，插点、状态索引、数据相位、范围和误读陷阱见 `test_results/closed_loop_navigation_20261005/front_end_observability/PROPOSAL.md`。保留现有已通过camera_mode的库与二进制；新run必须记录实际加载的库/源/配置SHA，不能仅记录原库却通过LD_LIBRARY_PATH换库。

优先保存 actual LIO ptpl correspondence、map plane/normal/原始点、候选拒绝与gate sigma、solver R_inv、实际HᵀR⁻¹H/HTz、完整P19、solution19、Δpz/Δvz与地图写入；VIO保存实际tracks/reference age、各NCC/patch拒绝、每level/iteration信息及接受/回滚。限定115～165秒详细窗，阶段用真实integer header/ns，不以“点数非零”猜阶段。记录额外开销，不能放宽300ms导航或200ms socket门。

由实际匹配与协方差证据区分退化、错配/局部地图自洽漂移、初始化/重力偏置，才决定修前端或增加稳定关键帧/子地图、地图高程约束、重定位/回环。已有纯离线点云+已知层高观察器在 `ground_plane_fusion_design/`；它没有上线，若融合必须同时校正pose/cloud、明确已知地图先验、处理层/坡面歧义与不确定度，不能用goalZ盲补或选择native pose。

不要重复扫没有证据依据的PID/频率来补错误位置。现控制器在单坡道已有合格路线与停车，完整多层仍需要定位通过。后续全32区域与双坡道必须按原3D到达/90s、路线、物理接触、首停车窗和完整退出独立复核，再验证动态障碍恢复。三层是坡道，真实楼梯和真机未验证。
