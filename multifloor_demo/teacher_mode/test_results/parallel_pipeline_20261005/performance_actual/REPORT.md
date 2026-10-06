# V18与V12实际210秒前缀性能

只读原kind300和独占进程采样，统一选择sim30–209.4秒的实际采样区间。V18初次外部采样先于SLAM创建，旧reader全窗进程交集仅runner，不能据此判断SLAM CPU。本对照使用SLAM在两端均存在的区间。

|阶段|V12墙钟均值(ms)|V18墙钟均值(ms)|V12调用线程CPU(ms)|V18调用线程CPU(ms)|
|---|---:|---:|---:|---:|
|Process2 whole|0.502|0.513|0.501|0.509|
|LIO residual matching query|1.602|1.762|1.515|1.655|
|LIO StateEstimation whole|9.127|9.293|8.833|8.568|
|UpdateVoxelMap|0.605|0.665|0.603|0.658|
|VIO processFrame|2.555|2.545|2.550|2.519|
|publication conversion/enqueue|0.235|0.246|0.234|0.244|
|spin_some|0.144|0.150|0.103|0.108|
|lidar preprocessing|0.399|0.428|0.399|0.425|
|handleLIO whole|12.689|13.113|12.386|12.348|
|handleVIO whole|3.552|3.585|3.544|3.547|

这次组合实际测试未证明整链吞吐显著提高。LIO调用线程CPU约降3%，墙钟约增1.8%；视觉整帧墙钟基本相同。主线程非自愿切换由约95.5升至190.3次/墙秒，不能把这部分墙钟差直接称通信开销。

V12：FASTLIVO进程平均0.837核，主线程0.652核，其余线程合计0.185核；源header更新30.304Hz，接收墙频30.068Hz，位姿源年龄p95=40.0ms/max=60.0ms。
V18：FASTLIVO进程平均0.862核，主线程0.646核，其余线程合计0.216核；源header更新30.297Hz，接收墙频29.591Hz，位姿源年龄p95=40.0ms/max=60.0ms。

CPU进程时钟包含并发OpenMP、ROS和logger，不能作为阶段独占CPU。残差query在StateEstimation内，后者在handleLIO内；表中各阶段不得相加。publication只计同步转换/enqueue，DDS后续通信未测。来源到callback、callback到估计的纯通信分解仍无完整trace，不能用wall减threadCPU替代。

同名路线的实际机器狗位置、地图历史和约束输入不同，不能把某段差值当作单个kernel的因果；此次是V15+V16组合实测，非每个优化独立A/B。有限基准行构造的约72%改善或视觉T1的24–30%改善，不能升级为实际整链同幅度提升。

运动/区域/停车及坡道验收由独立evaluator处理，本目录不重复或变更其收据。没有写入旧run、冻结源码、模型或配置。
