# LIO 线程扩展与最小无锁收集测试

本次只做离线合成 LIO fixture，不启动 ROS/Gazebo、相机或实体，不触碰训练。生产没有使用测试 LD_PRELOAD。完整路线、实际30Hz、导航到达及停车以各自真实run的独立收据为准。

当前建议是独立 V12 生产4线程进入真实链验证，不继续追8线程。原V11增加线程没有改善；针对具体共享位锁的 V12 优化产生了有限组件收益。V12 10个case与同核型V11的 state25、cov361和完整有序残差逐字节一致。跨P/E核型仍有37个state/cov double差异≤2.22044e-18，残差逐字节同一；未证明产生微小差异的具体原因。

## 固定负载与实际team

合成3张独立平面（地面+两墙），每面91×91，共24,843点；manager内部实际288个根体素，24,843 accepted residuals。它并非实际三层地图，不包含实际地形、全图空间分布或完整传感器链。每component30次，StateEstimation预热5次，residual预热8次；输出IO在component计时外。

CPU是Core Ultra7 265KF，20逻辑/物理核，P PMU0–7、E PMU8–19。本测试P affinity0–7，E使用8–15子集。真实team由测试-only GOMP_parallel观察记录，每preloadedcase143个target region都等于requested1/2/4/8且CPU在指定集合内。wrapper仅按dladdr识别调用者BuildResidualListOMP，其余OpenMP调用原样通过，不改VIO分支或原始循环体。MP_PROC_NUM只决定omp_set_num_threads，没有按线程数分配的算法scratch array，override不越界。生产执行不得复制这种环境。

## 有限性能对照

单位ms，各列是30次中位数；StateEstimation包含缓存构造、全部迭代、矩阵求解等，不只是残差。

| 核组 | 实际team | V11 StateEstimation | V11残差 | V12 StateEstimation | V12残差 |
|---|---:|---:|---:|---:|---:|
| P | 1 | 29.203 | 4.988 | 22.732 | 3.646 |
| E | 1 | 29.032 | 5.198 | 26.530 | 4.555 |
| P | 2 | 37.521 | 6.920 | 20.629 | 3.351 |
| E | 2 | 28.639 | 5.175 | 22.874 | 3.624 |
| P | 4 | 41.309 | 7.770 | 20.057 | 3.051 |
| E | 4 | 29.660 | 5.853 | 21.975 | 3.078 |
| P | 8 | 42.396 | 8.159 | 21.669 | 3.243 |
| E | 8 | 37.896 | 8.638 | 21.974 | 2.678 |

V11无preload P2原生对照33.960/8.434ms，V12无preload P4原生20.644/3.972ms。同核组全部preload case分别与各自无preload对照逐字节一致。V11、V12非随机交错执行，背景负载、频率及cache可能变化，因此不把表中的差值全部归于单一因素；仍需真实完整链对照。没有未绑核的基线，不能声称固定P/E affinity能改善整链。

## 保留的失败与metadata纠正

原run_scaling.py把P/E全矩阵SHA强制相同，E矩阵结束后因跨核型SHA不同而assert失败，results_E.json未写。所有E原始输出保留，results.json明确global byteequal=false，同核型byteequal=true，未把原失败改称全局通过。

原fixture把外部传入unordered_map的size读作map_root_voxels=0；VoxelMapManager构造函数按值复制map，随后建立的是manager.voxel_map_。它并非“没有地图却有24,843约束”。保留lio_scaling_fixture.v1.cpp、fixture.v1、build_receipt.v1.json和全部旧timings。只纠正测试metadata，E_nopreload2测到manager内部288，canonical SHA与原E全组同一。

## 代码与数值绑定

V11 core SHA f8431ddff43e23fe00c99112c19f00ef4530e9447f4ed213c08494141717b6ef。
V12 core SHA8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed。
P全体canonical SHA cafaa07675c760480269e8faa5af7945080be0cb74b2d3210f4766261ff6dae0；E全体3f17e1bf1f11cec6eb147af9042c32dd63e0a98da1c21eaf37aae8d79a29842d。

V12只把vector<bool>位标志改成独立unsigned char槽，并去除相应共享锁，保持原索引串行collect和原浮点/地图查询表达式。计时observer在此矩阵FASTLIVO_BOUNDARY_TIMING=0；V12实际运行可单独打开只读kind300。测试结果不代表生产增加线程绝不改变任何场景结果，也不代表更改核型位级相同。

## 文件与复现范围

- results.json：原V11全部原始数值/性能与跨核型失败。
- candidate_V12/results.json：独立V12全部10case、30次原始样本与同核型bitexact。
- build_receipt.json / candidate_V12/build_receipt.json：实际编译argv、库/fixture/source绑定。
- */actual_teams.jsonl：原始GOMP真实team/CPU。
- resources_*：测试前后负载与拓扑；没有停止任何外部任务。
- [COMPUTE_DEPENDENCY_AUDIT.md](COMPUTE_DEPENDENCY_AUDIT.md)：SO3/SE3、现有IMUpose与patch缓存、失效条件、流水线风险、旧时序缺口及V12 kind300嵌套边界。

目录创建采用exist_ok=false，脚本拒绝覆盖已有case。若复测应复制测试harness至新的独立输出目录、重新绑定库与compile argv，在root确认没有实际仿真/构建竞争的窗口运行；不可在生产Gazebo命令里设置GO2_TEST_LIO_TEAM或本测试LD_PRELOAD。此报告不修改冻结的sync_audit/viewer_audit证据。
