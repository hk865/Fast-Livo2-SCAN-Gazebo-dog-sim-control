# V12 30 Hz 单次 210 s 前缀与实测耗时

实际 run：`/var/tmp/go2_teacher_simulation_20261005/20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d`。项目 runs 中同名路径只是别名；收据绑定真实路径及原始归档。本报告仅离线验收，没有发布控制命令，没有改动训练或数值判据。

## 闭环结果

- 原 common：25/32 原始三维区域到达；物理期间 running，首次失败 guard 无。全 32 区域未完成，整体仍 FAIL；终点停车与完整非平地接触门为 UNVERIFIED。
- 新 publication v2：原 common 其余门保持；756ad4c3… 前瞻契约及实际源快照绑定通过，七个附加门全部通过。4301 条实际发布与 writer 和原命令计数一致；4041 条关联 PID、260 条 PID 外发布，其中15条短时重复时钟 hold。每次实际发布原 ROS 时钟用于 slew 回放，没有用接收时钟冒充生产者时钟。
- 3743 次实际几何 guard 独立回放通过。原双时效、唯一 CPU Teacher、50 Hz 执行、native 连续 200 Hz、身体安全门通过。
- 第一条 ramp_12：实际完整12 m COM出入口、四足有序坡面支持、全部12米分段、落地区连续支持、真实SLAM目标楼层区域均 PASS。第二条 ramp_23 尚在运行，未完成全坡接触与目标落地证据；总体多楼层导航不能称通过。这些连接是坡道。
- 离线 native 对真实路线（单次原始锚点配准）：最大距离0.161572 m、运动段RMS0.039867 m、drive最大航向误差0.166472 rad、速度MAE0.038978 m/s，均通过原门。Gazebo位姿仅用于上述离线验收，不进入导航控制。

## 实际频率和高度

LiDAR和RGB各6364帧，原始header跨度0.01–209.98 s，实际source平均30.304 Hz；离散物理时钟使帧间隔25–35 ms，不能将“30 Hz配置”理解成每帧严格33.333 ms。IMU42002帧、200 Hz，以上均无>300 ms源缺口。实际LIO/VIO processed handler分别6254/6253次，source平均30.303 Hz；优化器细节仅115–118 s收集91/91帧、30.252 Hz，窗口外优化器数学输出未采集，记N/A，不是0。

20 Hz墙钟ticker触发与实际新源PI更新分开：数学3544次、17.435 Hz，包含初始化、短时hold、朝向及停车阶段。实际真实SLAM post-anchor callback age最大60 ms；已接受SLAM源没有>300 ms缺口。cleanup后IMU停止与停车保护不算物理途中故障。

全post-anchor离线SLAM-native相对Z最大90.75 mm；原131–134 s窗43.92 mm，该窗native高度变化−3.52 mm而SLAM变化+6.44 mm，没有重现原20 cm高度突然降低。机器人空间状态与旧run同时间窗不同，不把这个窗口直接当成完全相同墙面和姿态的A/B。全程身体接触/物理fault为0。

## CPU与wall分解

CPU sampler从实际sim24.78 s起，原始/proc计数的物理窗口为sim24.78–209.38 s、185.999965 wall s。早于首条采样的CPU不可重建。FASTLIVO进程155.71 CPU s，平均0.837核；main TID121.34 CPU s、0.652核、17897次非自愿上下文切换。三个最重非main TID合计22.82 CPU s；相同comm不能独立证明其是OpenMP、ROS或writer。每个TID、真实mask和processor记录在原始sampler，不使用生命周期ps百分比。

kind300共214条、invalid clock calls=0。以下累计只选择完全包含在上述CPU窗口的kind300完成窗口；边界嵌套、有开始于窗口之前而在窗口内完成的scope，不能直接把各行相加。

| 边界 | 累计wall s | caller thread CPU s | 同区间并发process CPU s | 单次平均wall ms |
|---|---:|---:|---:|---:|
| Process2 whole | 5.551 | 5.544 | 5.926 | 0.501 |
| 后向点去畸变 | 4.353 | 4.348 | 4.558 | 0.786 |
| LIO residual query（所有迭代） | 29.333 | 27.715 | 51.140 | 1.613 |
| StateEstimation（包含query） | 50.710 | 49.049 | 72.987 | 9.160 |
| UpdateVoxelMap | 3.351 | 3.343 | 3.474 | 0.605 |
| VIO processFrame | 14.074 | 14.046 | 14.361 | 2.542 |
| 同步转换及publish/enqueue | 9.073 | 9.055 | 9.284 | 0.234 |
| spin_some（包含所有回调） | 32.068 | 23.013 | 29.358 | 0.144 |
| LiDAR回调（包含预处理） | 2.284 | 2.278 | 2.378 | 0.413 |
| sync_packages | 2.225 | 2.221 | 2.284 | 0.010 |
| handleLIO（包含匹配/建图/发布等） | 70.441 | 68.735 | 93.530 | 12.724 |
| handleVIO（包含视觉/发布等） | 19.593 | 19.549 | 20.087 | 3.539 |

StateEstimation是这里直接测量的最大计算段。VIO、地图更新、同步publish的caller CPU接近wall，说明这些同步阶段在该run中主要消耗CPU。spin_some累计wall高于caller CPU约9.06 s，此差包含调度、ROS同步、排队等，不能全部标成通信。process CPU包含同时运行的OMP/DDS/logger线程，不能当函数独占CPU，也不能跨嵌套边界相加。publication只测同步转换与enqueue，DDS返回后的通信没有被该边界单独测量。

现有kind2回调与kind10 Process2-start的精确header/offset连接可测“回调入口到LIO启动”，其中含预处理、同步等待及此前管线工作；采集到回调仅通过原始/clock观测给age区间，无法单独排除渲染/桥/队列而得到纯transport延迟。详见`comparison_v12_30_lockfree4_timing/pipeline_clock_and_zero_evidence.json`。该旧phase读者的map/pub N/A是指其自身kind10/11/12/13边界；本报告新增kind300已独立测到whole-map与同步pub，两个scope不能混用。

主线程未持续占满一个核，但有可见非自愿切换，因此CPU分核值得实际对照；仅此证据不能保证分核改善路线。V11与V12曾有不同机器人空间状态、不同并行/锁结构，不能把平均耗时比直接解释成只改线程数的因果收益。

## 证据与复现

- 原run：`summary_closed_loop_cascade_independent.json`、`summary_closed_loop_clock_hold_independent.json`、`summary_closed_loop_ramp_independent.json`，并列scoped收据，未覆盖历史。
- 本目录：`comparison_v12_30_lockfree4_timing/performance_proc_same_wall.json`；同子目录run名称中的`summary.json`、native与实际SLAM轨迹CSV、`height_commands_full.png`。
- 只读命令：`python3 -B performance_proc_v12.py --run <上述真实run> --output <新结果.json>`；`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B publication_ledger_v11/audit_publication_ledger.py --run <新完成run>`。
- 新CPU sampler源SHA256：`43c3c7bc4bf033d7cc62399a1a2f3815fdc748026dab5c2bfd9bb6926d6c7cd6`。sampler实际95条，1.520 CPU s、最大单次周期20.01 ms；不改调度或affinity，仅写读数。

结论范围：CPU Teacher接口与本前缀运动/安全通过，第一条完整坡道通过；完整32区域导航、第二条完整坡道和终点停车未通过/未验证，真机部署未验证。
