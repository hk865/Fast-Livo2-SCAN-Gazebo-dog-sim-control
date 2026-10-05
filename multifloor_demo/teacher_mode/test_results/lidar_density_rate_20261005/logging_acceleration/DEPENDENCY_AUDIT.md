# 诊断复制的数据依赖审计与 V11 有限检查

V11 只缩短诊断复制/记录的执行范围，没有修改估计匹配、ICP/IEKF、权重、状态更新、IMU积分、点数/降采样、控制器或保护。独立两包已构建，通过有限合成检查；实际 30 Hz 吞吐和完整多层导航尚待根任务运行。新旧 run 与前一轮 `lio_multicore/` 证据均未修改。

## 依赖与安全条件

| 诊断结构/计算 | 原消费者 | V11 条件与保留内容 |
|---|---|---|
| 每个查询点60个double初始化、点坐标/9项cov复制、counterfactual邻居查询、候选历史/置信度副本 | kind102 detail 行；同时 kind100 全时段汇总 cols1–7、37–45、origin | **两者一起限 detail**。窗外不收集 kind100，不能用空行产生伪“0匹配”。真实递归匹配/候选概率/`pv.normal`/约束字段仍照原算法执行 |
| kind100 每迭代状态25、P361、K1/G/M6/HTz/solution | 离线诊断写入与重建 | 窗内保留完整原格式，窗外无该类记录。caller kind13 最终LIO状态/协方差仍每阶段记录 |
| VIO retrieval counters、全feat_map诊断计数遍历、frame cov/G、kind201迭代完整矩阵、回滚分支额外计算 diagnostic_M/HTz | kind200/201，离线重建；均在原 `FASTLIVO_DIAG`块内 | retrieval/updateState/updateStateInverse/frame-cov 诊断条件由 enabled改detail；原视觉残差/float error归约（串行）/接受与回滚/状态数学保留 |
| plane birth/update counters/timestamps | 窗内kind101/103面历史 | **仍 enabled 全时间维护**，避免进入3s窗时丢失面年龄/冻结前历史 |
| `diagnostic_frame_stamps_` | 窗内视觉关键帧参考年龄 | **仍 enabled 全帧维护**，防止引用窗前frame时年龄缺失；不影响特征管理/匹配 |
| raw kind4 每帧点云5项复制 | 原硬编码115–165s点云记录 | 改用同logger `detail_at(cur_head_time)`谓词，读取profile声明的115–118s；`cur_head_time`仍原offset-corrected源header时间，raw记录原整数header不变 |
| kind1/2/3、10/11、12/13、20 | 原始IMU/LiDAR/RGB回调及Process2、最终VIO/LIO、全部IMU传播 | 原 gating/格式/实际数据保留；kind14本来已detail-only；原VIO空图/少点异常skip小记录保留 |

`diagnostic_query_rows_`、`PointToPlane.diag_*`、VIO诊断计数/矩阵副本/历史map无估计算法消费者；唯一非detail日志消费者kind100被一并缩窗。所有正常约束字段（点、法向、plane covariance、dis_to_plane）、真实算法 `pv.normal` 始终构造，匹配索引合并仍原输入顺序。

四个 C++/header 差异仅为：voxel_map两处诊断 bool、VIO五处诊断 gating、LIVMapper kind4 gating、Logger统一谓词 helper。按逐条逆替换恢复 V10 **全部源字节**，其余37个源文件与 V10 完全相同；CMake保持原source-specific LIO2线程，VIO无MP宏。可见[实际逐行diff](voxel_map.cpp.v10_v11.diff)、[源准备proof](source_only_preflight.json)、[构建flags](build_receipt.json)。未更改历史实际数据的日志契约；旧记录可继续按旧schema解读，新 V11 窗外solver字段只能显示未采集/N/A。

## 已执行的有限检查

两包 `-j2` / sequential构建65s完成；新core SHA256：`f8431ddff43e23fe00c99112c19f00ef4530e9447f4ed213c08494141717b6ef`。同样无fast-math/march-native，只有voxel_map.cpp有 `MP_EN;MP_PROC_NUM=2`。

测试包含 LIO729点、VIO9个合成patch forward、VIO9patch inverse三个scene；各自链接真实安装V10/V11库，并分别logger off / inside / outside，18个独立进程。各scene六份状态/协方差/求解输出逐字节一致；inside-window日志按header/order及payload逐字节一致，仅LIO四个wall计时double除外。窗外旧库仍生成kind100/201，新库均无完整solver记录；LIO查询数组由729行变为0行。VIO两分支检查也涵盖继承LIO后主线程OMP默认2的情况。fixture未运行ROS/Gazebo/Teacher/设备。

独立Logger检查115/118包含边界、边界外1ns拒绝，以及raw source时间与当前estimator context不同的情况，统一谓词正确，未重写任何源时间。[18进程数值比较](finite_fixture_comparison.json)、[窗口测试](window_fixture.log)、[最终有限preflight](final_preflight.json)。首个**测试专用**camera ctor参数失配已修正，原失败编译日志保留于`baseline_V10/compile_initial_interface_failure.log`；生产构建无失败。

这些是有限合成检查及源码数据依赖证明，不能替代完整真实输入/地图重放或闭环数值一致性验收，也尚不能证明实时吞吐改善多少。

## 实际窗口修正与配置范围

V9/V10虽然有预制“3s”profile，但`run.py`仍硬编码环境115/165。该短profile尚未作为实际3s实验运行，不能引用它证明已减少采集。V11仅日志环境读取已冻结`diagnostic_scope.detail_window_sim_s`，固定允许115/118，并与kind4共享谓词。根任务应检查actualruntime plan、effective environment及finalwriter_stats均为该3s窗口。

V11提供三个profile：30Hz210s有限前缀、20Hz600s及30Hz600s完整路线。后两者是**传感器采样 + 既有LIO两线程 + 诊断复制缩窗**的组合配置，不能将前后差异单独归因于多核或日志减负。Actor仍CPU1线程，50Hz；控制器20Hz wall tick/原300ms来源期限/唯一执行器/200HzPD、路线/SLAM/SCAN源都保持。当前所有实际吞吐/闭环/导航结果未验证，由根任务继续。
