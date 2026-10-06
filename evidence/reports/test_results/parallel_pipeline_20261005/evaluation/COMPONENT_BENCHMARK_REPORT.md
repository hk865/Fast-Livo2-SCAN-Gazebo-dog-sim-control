# 独立有限组件基准验收

V15 LIO 四线程行构造通过有限 kernel 门；V16 VIO 单线程变体通过有限 kernel 门。V16 四线程性能门失败。这里没有新的 Gazebo 或导航结果；组合 V18 必须用新库做 fresh 数值验证并实际运行。

V15 正式 384 个独立进程，6 场景、每变体每场景 32 批；各批 10 次预热、40 个嵌套测量。V16 正式 768 个独立进程，8 场景、3 变体、各 32 批；各批 12 次预热、30 个嵌套测量。嵌套样本不当作独立批次。对称 ABBA/BAAB、P 核、资源和频率记录已核对；CPU 频率未锁定，测试没有系统负载隔离。

独立读取原 samples 重算，并重新核对所有 384 和 768 canonical 输出及 27 和 123 有限数值输出的实际字节哈希。机械恢复 V15 原 cpp/header；V16 只有声明的 vio.cpp/CMakeLists 两处源码变化。浮点观测单独进行，不计入统计性能。

| LIO 场景 | 独立批次中位数改善 | 门 | 说明 |
|---|---:|---|---|
| small147 | -17.10% | 有限通过 | 冻结阈值下实际串行，行计算和 wholeState p95 退化保留 |
| medium2883 | 67.41% | 有限通过 | 有限行 kernel，不能直接代替完整吞吐；wholeState p95 退化 |
| large24843 | 75.29% | 有限通过 | 有限行 kernel，不能直接代替完整吞吐 |
| actual_first_6196_rows | 72.45% | 有限通过 | 有限行 kernel，不能直接代替完整吞吐 |
| actual_maximum_6339_rows | 72.61% | 有限通过 | 有限行 kernel，不能直接代替完整吞吐 |
| actual_minimum_6068_rows | 72.38% | 有限通过 | 有限行 kernel，不能直接代替完整吞吐 |

大规模 synthetic StateEstimation 中位数约 21.87 → 20.64 ms，约 5.65% 改善；行构造约 75% 改善不能称 SLAM 整链加速 75%。真实历史 HTH 聚合重建比原日志差约 2.98–4.47e−8，基线和候选均有此差异，仍为精确回放失败；fresh 同输入同 P 核基线/T1/T4 输出一致，未修改浮点容差。

| VIO 大场景 | 单线程批次中位数改善 | 四线程有限门 |
|---|---:|---|
| forward_n1024_level1_searchmixed_exposure | 28.48% | 通过 |
| forward_n512_level1_searchmixed_exposure | 29.53% | 失败 |
| inverse_n1024_level1_searchmixed_exposure | 23.75% | 失败 |
| inverse_n512_level1_searchmixed_exposure | 30.22% | 失败 |

单线程新源已比 V12 快，四线程相对同源单线程的收益必须另列；不能把新源的全部收益归因于多线程。报告保留批次中位数 p95、批内 p95 和汇总样本 p95，各种尾时延口径不混用。四线程 inverse1024 中位数改善仅约 8.10%，不满足原 10% 门；多个场景尾时延更差。

所有实际导航源、300 ms 双时钟 TTL、唯一执行器、CPU Teacher 50 Hz/单线程、32 个三维目标、两条完整 12 m 坡道和固定 5 s 停车门保持原值。Actor 232 维仍来自特权仿真输入，15 维为已知命令/上一动作；导航输入仍必须来自真实 SLAM/IMU/SCAN。坡道通过不能叫真实楼梯通过。

来源：[V15 独立收据](V15_CONFIRMATORY_INDEPENDENT.json)、[V16 最终独立收据](V16_CONFIRMATORY_INDEPENDENT_V2.json)、[V18 事前组合范围](PROSPECTIVE_V18_COMBINED_SCOPE.json)。所有既有原收据和失败均保留。
