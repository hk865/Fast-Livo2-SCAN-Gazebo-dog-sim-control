# V12与V14实际分核对照

两组均为同一冻结V12库、CPU Teacher、64×480 LiDAR与RGB30 Hz、20 Hz控制ticker、210 s实际前缀、115–118 s细节日志。V14仅为新owned进程增加事前声明CPU mask：Gazebo0–3、SLAM4–7、Actor18、controller19、其他8–17；没有移动其他任务或保留CPU独占权。

真实目录均在`/var/tmp/go2_teacher_simulation_20261005/`，basename：

- V12：`20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d`
- V14：`20261005_210604_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_affinity_timing_r1_6eb0`

## 相同门下的结果

两组原common与v2收据除32区域未完成、终点停车和完整非平地接触未验证外全部通过；v2七个附加source/时钟hold/PI/实际publish-slew门均PASS。均25/32、物理期间running、无firstfailedguard；第一条完整12m坡道的COM、四足、全部分段、landing及真实SLAM正确楼层区域通过，第二条尚未完成。这里是坡道，不是楼梯。

| 实际指标 | V12 | V14 |
|---|---:|---:|
| 已接受SLAM源>300ms间隔 | 0 | 0 |
| post-anchor source age p50/p95/max s | .030/.040/.060 | .030/.040/.065 |
| 原路线离线tube最大/RMS m | .161572/.039867 | .172087/.046423 |
| drive最大heading rad | .166472 | .167727 |
| 速度MAE m/s | .038978 | .039908 |
| 实际controller新源数学Hz | 17.435 | 17.640 |
| 全post-anchor SLAM-native相对Z最大 m | .090751 | .113140 |

单次差异不能视为统计显著性。两组机器人姿态、位置和匹配迭代数略不同，不是同状态运算回放；仍须完整600 s路线实测，不能凭25/32前缀升级完整导航。

## mask与调度

独立重新解析原`cpu_affinity_witness.jsonl`真实`Cpus_allowed_list`，104条、25219个thread样本、5groups均观测，全部与事前合同一致。不能仅凭producer自身`verified:true`接受；reader独立展开真实CPU范围，并对比对应group的冻结预期。mask只限制本run线程去向，没有隔离GPU、磁盘、内存带宽，也没有排除其他任务在相同CPU执行。

CPU采样首帧V12 sim24.78、V14 sim45.38；早于首条没有外部/proc CPU证据。以下择共同请求的simulation-time段，实际/proc端点小幅不同，按实际wall秒计算。

| 请求sim段 s | V12→V14 main非自愿切换 /wall s | V12→V14 StateEst单次wall ms | V12→V14 query单次wall ms | V12→V14进程平均CPU核 |
|---|---:|---:|---:|---:|
| 60–110 | 120.792→174.240 | 10.512→11.615 | 1.848→2.181 | .879→.846 |
| 130–170 | 56.184→138.711 | 6.713→7.638 | 1.062→1.341 | .755→.702 |
| 170–210 | 120.500→154.925 | 9.361→10.765 | 1.655→1.893 | .847→.828 |

分核没有显示减少主线程抢占或匹配wall耗时的收益。较少CPU秒不等于较低延迟；进程CPU还含并发OMP/DDS/writer，不能当函数独占CPU。kind300各段相互嵌套，不能累加为总耗时。数据不足以断定分核必然变差，但不支持在这个单次对照后优先选V14。

建议完整600 s选择V12原调度；保留V14真实分核对照作为已验证mask、有限运动有效、性能未获改善的证据。所有原TTL、停车、朝向、region和安全判据不变。

机器可读对照：`comparison_v14_30_lockfree4_affinity_timing/V12_V14_performance_comparison.json`，绑定两组独立perf结果及实际mask witness SHA；两组`performance_proc_same_wall.json`保存每TID CPU、上下文切换与mask。原common、v2及ramp收据在各真实run内，并有独立suffix副本，历史未覆盖。
