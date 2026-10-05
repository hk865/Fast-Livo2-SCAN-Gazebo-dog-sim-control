# 本轮并行实现、瓶颈与收益边界

本说明使用已完成的离线验证与V18的210s实际前缀证据。V18完整路线和V17队列候选的实际结论由主任务后续补充；这里不预判通过。未修改原相机Demo、Frozen Teacher或既有控制器，Teacher保持CPU单线程，源新鲜度仍为300ms。

## 已实现的计算候选

|候选|实现和最终选择|验证与收益|明确限制|
|---|---|---|---|
|V15 LIO rows|按约束索引i独立构造Jacobian、权重和测量项；有效约束不少于256时4线程，小循环仍走串行分支。保持每行表达式、原索引汇总、原Eigen乘法及完整19维先验。|384个独立进程批次；729点有限StateEstimation及真实约6k约束行的新同输入输出逐字节一致。真实约6k行构造中位耗时约降72%，24843行约降75%。|这是约0.13ms级的小算子。24843点整个合成StateEstimation中位仅约降5.7%；small/medium整个State的p95退化保留。历史记录HTH重建未逐字节一致，原因仍未验证，不能声称完整历史前端回放通过。|
|V16 VIO patches|patch内部残差/Jacobian计算独立，误差与计数仍按原索引顺序汇总；最终选择编译threads=1、min_points=64。|41个边界场景覆盖正逆更新、曝光、cache、空输入、金字塔、阈值与rollback，输出和原非计时诊断逐字节一致。正式有限large算子基准中T1约降24–30%，p95门通过。|T1收益来自计算组织变化，不能称多核加速。T4性能门失败，原结果保留，不用于部署。有限patch测例不等于真实整帧processFrame同幅度收益。|
|V18 组合|独立复制V16 T1源码，合入V15的voxel_map.cpp/.h；新库的LIO行team4与VIOteam1已观察。|新跑27个LIO/视觉回归进程和41个视觉场景共82进程，状态、协方差和非计时诊断逐字节一致；控制器7个冻结文件与V12相同。|组合离线门只允许有限实际实验，不能复制两个组件PASS来宣称导航成功；组合实测也不能拆成某个单一算子的因果。|

V15继承V12已有的残差匹配4线程、独立uint8索引槽及原序收集。当前优化没有改变平面关联、权重公式、ICP/EKF更新、IMU传播或控制器参数。VIO没有启用旧的全局浮点OpenMP reduction。

## V18前缀实际没有明显整链加速

对照V12前缀0b8d与V18前缀ac02，采用两次都存在FASTLIVO进程的实际sim30–209.4s采样区间。两次均为名义64线LiDAR30Hz、相机30Hz；实际机器狗位置、地图历史与约束输入不同，以下是观测比较，不是同输入算法因果试验。

|实测量|V12|V18|
|---|---:|---:|
|LIO StateEstimation平均墙钟|9.127ms|9.293ms|
|LIO调用线程平均CPU|8.833ms|8.568ms|
|VIO processFrame平均墙钟|2.555ms|2.545ms|
|handleLIO平均墙钟|12.689ms|13.113ms|
|FASTLIVO进程平均CPU核数|0.837|0.862|
|FASTLIVO主线程平均CPU核数|0.652|0.646|
|主线程非自愿切换/墙秒|95.5|190.3|
|实际SLAM原header频率|30.304Hz|30.297Hz|
|SLAM接收墙钟频率|30.068Hz|29.591Hz|
|位姿接收时源年龄p95 / max|40 / 60ms|40 / 60ms|

LIO主调用线程CPU约下降3%，墙钟略升；视觉整帧基本相同。此次没有实证整链吞吐明显提高，也没有发现源年龄在该前缀持续积压。主线程被抢占的频率更高，但这里不能进一步认定具体外部负载或某个线程是唯一原因。

局部加速难以等比例传到整链：LIO行构造在约9ms的StateEstimation中占比很小，剩余求解、协方差更新、关联和地图工作仍存在；有限视觉测例强调patch算子，而真实整帧还包括特征选择、缓存管理、关联及其他处理。实际点数、约束数、视觉有效patch、迭代数和姿态会改变这些占比。线程启动、同步及调度也有成本，因此继续增加线程不保证更快；VIO4失败正是本轮需要保留的证据。

## 当前队列与状态依赖边界

本地核心仍由一个canonical state owner依次推进IMU传播/去畸变、LIO状态与协方差更新、地图更新、VIO更新，再产生发布快照。LIO和VIO共享同一状态与先验；VIO使用LIO提交后的位姿，下一组IMU传播又依赖更新后的bias、cov和时间锚。这些是跨阶段的数据依赖，不能把三段随意并行后再拼接结果。

V17候选把输入收集与主状态估计分开：同一个后台SingleThreadedExecutor负责接收和解码，并形成ready输入；主状态owner按原同步规则消费。接收与解码仍在同一后台线程，解码可能短暂推迟下一次接收；这个后台输入链与主状态owner重叠。当前没有另设独立decoder线程，也不是IMU、LIO、VIO跨帧三段自由流水。原IMU200Hz时间序列、源header与首次接收时间、同步组顺序、epoch/reset语义和300ms保护必须保留，队列入队或出队不能刷新旧数据年龄。V17是否在实际中改善吞吐及延迟仍待主任务的真实测试结论。

可继续独立拆分的方向是不可变输入解码、只读几何算子，以及估计提交后的不可变发布/记录快照。涉及state/cov、bias、deskew时间锚、地图写入和视觉缓存的操作需要明确所有权或版本化快照；直接换MultiThreadedExecutor会使现有同步队列与预处理scratch等共享访问暴露竞态，不能用线程数配置代替这一步工程设计。

## 计时解释

kind300记录的是完成边界，存在嵌套：残差query在StateEstimation内，StateEstimation又在handleLIO内，因此表中阶段不得相加。调用线程CPU不含其他worker；进程CPU包含同时运行的OpenMP、ROS/DDS和logger，不能当作某个阶段独占成本。publication边界只覆盖同步转换与enqueue，DDS后续通信未计入；wall减threadCPU混有抢占、等待和worker重叠，不能直接称为通信时间。

V18外部CPU采样第一次记录先于SLAM启动，旧reader的全窗进程交集只剩runner；本对照另选实际sim≥30s的共同有效区间，避免把runner占用误报为SLAM占用。

证据：`lio_v15/REPORT.md`、`evaluation/V15_CONFIRMATORY_INDEPENDENT.json`、`vio_v16/FINAL_OFFLINE_SUMMARY.json`、`evaluation/V16_CONFIRMATORY_INDEPENDENT_V2.json`、`combined_v18/REPORT.md`、`performance_actual/performance.json`与`performance_actual/REPORT.md`。完整路线、坡道与停车门仍由独立验收收据判定；三层场景连接是坡道，真机部署未验证。
