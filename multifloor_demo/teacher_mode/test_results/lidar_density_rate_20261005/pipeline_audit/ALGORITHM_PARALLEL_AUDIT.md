# 本地 FAST-LIVO2：算法并行审计

2026-10-05。对照用户引用的“解释SLAM前端上限”讨论、本地 V12 冻结源码和已完成实际测量。引用讨论只作分析材料，实际实现结论以本地源码为准。本文不部署新队列、不改相机 Demo、原估计器或训练；下列新增候选尚未构建或运行。

## 已实现与新增候选

V12 已完成逐点 residual query 的独立字节槽/移除共享锁、LIO 循环配置4线程及原点顺序合并。大点数有限基准4线程优于8线程；同核型数值结果一致。30/30Hz实际32区域与双上行坡道/固定5s停车已有新来源关联 v2 审计覆盖，详见 ../evaluation/FULL_REPORT.md；不等于本文各新增算法已部署。

当前17边界计时：StateEstimation累计50.71s包含query29.33s，VIO14.07s、MapUpdate3.35s。没有单独测Jacobian生成与normal-equation乘法，因此不能把剩余StateEst时间都算成矩阵求解或给出新优化的加速倍数。CPU/等待/传输的实际边界见 [测量报告](../../../docs/LIDAR_DENSITY_RATE_REPORT_20261005.md)。

| 计算 | 当前本地实现 | 新增并行入口 | 必须保持的依赖 |
|---|---|---|---|
| 本轮点坐标/方差 | TransformLidar及pv_list逐点写入 | 固定state/extrinsic后分连续点区间 | 原double→float舍入、括号与每点初始化；所有行完成后才匹配 |
| 地图匹配/残差 | V12独立点槽并行 | 已实现；后续按粒度实测而非只加线程 | 本轮地图只读，按原索引收集，成功/失败标志不得共享bit |
| LIO Jacobian与权重 | 串行构造Hsub/R_inv/measurement | 每线程写独立i行/列，最后原Eigen乘法 | 固定本轮state与state_propagat；保持plane variance权重和所有19维先验 |
| LIO局部正规方程 | 大H乘法得到6×6与6-vector | 分块约束摘要后归约 | 新浮点归约次序会改变舍入，必须另列数值及实际控制验收；不能称逐位等价 |
| VIO patch residual/J | 普通分支7列，inverse分支6列；显式MP关闭 | 同层同轮、独立patch区间写H/z/error slots | 图像、参考patch/normal、曝光和当前pose固定；网格/参考选择先串行完成 |
| VIO误差汇总 | float error按patch顺序累计，决定接受/rollback | 独立patch_error/count，原i顺序串行汇总 | 不直接启用既有float OpenMP reduction；error≤last_error分支必须对照 |
| 去畸变 | IMUpose确定后逆序interval/point游标 | 先建立相同interval索引，再按点区间并行 | 相同时间切分、边界clamp、插值、外参、参考时刻；不是逐点重新积分IMU |
| 地图更新 | 顺序插入、自适应八叉树、plane维护 | 不同voxel的独立工作或固定点集统计 | 原每voxel点顺序、拟合触发、点数阈值、plane id和树生命周期 |
| IMU轨迹/协方差 | 顺序名义传播，保存IMUpose | 固定bias区间摘要/前缀扫描可研究 | 同离散化/误差坐标/噪声、完整19维cov及校正epoch；不是简单换线程 |

## 观测方向的算法并行

固定当前线性化状态x_l和只读地图。每个观测可以独立计算r_i、J_i、W_i，产生局部贡献：

```text
Lambda_i = J_i^T W_i J_i
eta_i    = -J_i^T W_i r_i
```

合并观测贡献后只更新一次原状态。先验只加入一次；不得每个线程各解位姿后平均，也不得把完整19维状态缩为6维或7维。观测只直接涉及6/7维，其他变量仍通过完整先验交叉协方差更新。实际LIO measurement取负plane residual，因此实现符号必须沿用现有Hsub_T_R_inv/meas_vec，不套一个通用式替换原更新。

数学实数域的矩阵和/乘法具有结合性；IEEE浮点加法不严格结合。树形归约和分块6×6摘要会重排原Eigen累计，尤其可能影响退化条件、收敛阈值、视觉接受/回退。第一候选可只并行H行生成，保留原矩阵乘法及顺序误差汇总；下一候选才是局部normal-equation归约，并单列数学等价/浮点差异验收。

LIO原循环中的权重还依赖plane uncertainty和预测state；不是只算一个几何J外积。VIO普通分支包含曝光交叉项，不能把旋转、平移、曝光拆成独立估计器。每次迭代pose改变后，关联、投影与相关J仍需重算；这些摘要不是跨迭代永久有效的预积分因子。

## 时间方向的算法并行

给定同测量区间、bias参考和离散化，局部DeltaR/DeltaV/DeltaP可以与绝对起始pose分离；相邻区间按原时间次序复合。去畸变需要中间轨迹，只有最终摘要不够，需分块前缀扫描再恢复每个原始时间点的轨迹。

名义轨迹确定后，协方差传播可表示为P_out=F P_in F^T+Q。区间A后接B的摘要为：

```text
F_AB = F_B F_A
Q_AB = F_B Q_A F_B^T + Q_B
```

F/Q依赖的姿态线性化必须先与原名义轨迹一致；这里是保持时间顺序的结合，不是交换顺序。当前实现还包含gravity、bias、acc scaling、曝光随机游走及IMU切分/末端外推。跨校正epoch提前按旧bias计算，之后一阶bias修正不等于任意bias变化下精确重积分。

本轮IMU200Hz、融合30Hz，每周期平均仅约6.7个IMU输入；不能为了凑并行批次等未来输入而增加控制延迟。IMU传播/deskew实际占比较小，因此优先级低于每帧成千上万点/patch的并行与回调解耦。V13相同dt变换缓存仅准备源码、未验证，也不是已完成的区间预积分。

## 可落地的次序与验证

1. 保留已验证V12作为独立基线，新增候选先做LIO Jacobian按i并行和VIO patch按i并行+原顺序float error汇总；分别比较每行H/R/z、状态/cov、迭代数、接受/回退及源header。
2. 将工程接收/解码与单assembler/estimator解耦；完整接口见同目录QUEUE_ENGINEERING_AUDIT.md。先固定原FIFO、timestamp和camera分段，禁止靠丢IMU或刷新旧源receipt来提高表面频率。
3. 若矩阵构造/乘法占比经新计时证明显著，再试分块normal equations，明确浮点新合同和重复运动/导航验证；现有有限基准不能代替真实复杂地图。
4. MapUpdate或IMU区间扫描分别作为独立候选，不同时改变点筛选/迭代上限/视觉使用规则。并行读写同一map须快照/epoch，而两个完整LIO/VIO更新仍按统一ESIKF顺序提交。

每个候选先预冻结配置/源码/模型/来源合同，再用相同输入比较全过程，然后实际同路线复测。已完成V12 full单轮不是新队列、VIO并行或跨帧IMU流水线通过证明。

## 本地源码定位

源码根：navigation/lidar_sampling_v12/slam_ws/src/fast_livo2_core。

- src/voxel_map.cpp:406、454、485、534、544、548、790：原状态迭代、点var、H/J、原乘法、完整先验与匹配。
- src/vio.cpp:929、1498、1569、1747、1790、1867、1893、1907：金字塔/迭代、参考缓存、patch循环、误差归约/接受与完整state更新。
- include/fast_livo2_core/core/common_lib.h:33：DIM_STATE=19。
- src/LIVMapper.cpp:785、851、1319：串行spin/sync/估计、高频传播重锚与同步切分。

论文核对：[FAST-LIVO2](https://arxiv.org/abs/2408.14035)明确统一ESIKF顺序测量更新和统一地图；[Forster预积分](https://arxiv.org/abs/1512.02363)说明关键帧间相对约束与偏置处理。本文的具体并行入口来自本地源码审计，而非把ROS1上游泛化结论视为本分支已部署。
