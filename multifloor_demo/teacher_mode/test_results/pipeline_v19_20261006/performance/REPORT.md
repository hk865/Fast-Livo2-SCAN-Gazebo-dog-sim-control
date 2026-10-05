# V19 SLAM-only 同包回放：有限工程验证

2026-10-06。最终核心库 `f06e68d0dcbcab9f840077c372d0cda2e128f9935c4fe968987784e2124e00d8`，核心事前收据 `eaff0b9eca952c9fefa8ed80a0194b00ae0c98732b3118eb905cb0ab56f70bec`。

四轮 1x ABBA **全输入头时间戳和严格正常生命周期通过**。没有启动 Gazebo、Teacher、控制器或 SCAN；这不是导航或 Sim2Sim 通过。30 Hz 输入供给下没有观察到明显的位姿时效改善，峰值吞吐尚未验证。

## 数据与冻结条件

同一新完成 60 s MCAP/zstd_fast，626,293,923 B。完整 CDR 索引 27,641 条：IMU 12,002，RGB 1,819，LiDAR 1,819，clock 12,001。传感器输入头全部实际提交，共 15,640；LiDAR/RGB 30.3076 Hz，IMU 200 Hz。记录输入包含初始化前缀，算法从新进程和空地图开始，首个位姿约 3.635 s。

实际 domain 顺序 201/202/203/204，serial→staged→staged→serial。copy_opt=0、LIO4/VIO1、P核 mask 0,2,4,6、同一相机/前端配置与输入 acquisition stamps，真实 `/clock` 从 bag 回放。每轮新进程、实际 `/proc/maps` 私有库绑定与独立正常 SIGUSR1 排空；所有自有进程退出0且无残余。

## 实际结果

|轮次|accepted=delivered=committed|取消/拒绝|peak pending|位姿数/Hz|观察端 age median/p95/max(ms)|StateEst wall/CPU(ms)|VIO wall/CPU(ms)|
|---|---:|---|---:|---|---|---|---|
|A1_serial_1x|27537|0/0|1|1709/30.305|30.00/35.00/45.00|9.703/8.350|1.814/1.812|
|B1_staged_1x|31940|0/0|13|1709/30.305|30.00/35.00/40.00|9.538/8.337|1.838/1.834|
|B2_staged_1x|31946|0/0|13|1709/30.305|30.00/35.00/40.00|9.805/8.596|1.823/1.820|
|A2_serial_1x|27548|0/0|1|1709/30.305|30.00/35.00/40.00|9.699/8.337|1.799/1.796|

age 为观察 node 最后收到的 ROS clock 减 pose header，包含 DDS/观察调度，不能称估计器内部当前时钟或真值误差。所有位姿 header 单调，间隔约30/35ms；新源时间差没有替代原控制器300ms保护。

StateEst/query/handleLIO 是嵌套边界；Process2 包含 deskew。kind300 给每个约1s窗内计数与总时间，表中是总时间除总调用数，不是每调用 p95；最后不足1s尾窗未强行补零。process CPU 包含并发 worker/logger，不能把 wall−CPU 当通信。stage8 的 serial spin_some 含 inline decode，而 staged commit 已把解码移出，stage8 变短不能当整链提速。新的每包23列 CSV分别记录 decode wall、threadCPU、队列等待和原callback时间；原callback不含完整estimator求解。

串行末次计数27537/27548，staged31940/31946，不同native wall timer与callback调度使timer数量不同，不能拿全包数直接算吞吐增益。Mapper 1Hz采样 CPU总秒约43.13/43.12与40.03/40.27，只是这轮描述；线程启动时刻/采样覆盖和topic顺序有差异。

## 数学、顺序与统计限制

新的有限算子回归 [finite_math/atomic_final/FRESH_MATH_FP_TEAM_LOADER_RECEIPT.json](finite_math/atomic_final/FRESH_MATH_FP_TEAM_LOADER_RECEIPT.json) 用最终库实际重跑100进程，729点LIO、VIO9以及41 patch case、FP模式与实际team/private loader通过。它不是完整processFrame/整地图回放。

同bag原CDR索引可复核捕获内容；mapper接收端不热路径hash大body，因此接收端body相等仍UNVERIFIED。每轮实际跨topic admission顺序都不同于bag recorder顺序，原采集mapper全序未保存。Residual调用数亦不同，不能宣称所有帧状态逐字节相同或将细小耗时差强归因于单一模式。完整固定全序 direct-feed harness **NOT IMPLEMENTED**。

当前每模式2个独立进程，共4轮，只是descriptive ABBA，未满足事前每scene/mode≥32批统计门，未覆盖一般场景/峰值吞吐。1x从输入供给限制输出，不能据此宣称并行已解决更高频率所有瓶颈。下一项可独立2x容量探测，再由原保护/来源验收实际短闭环及原46任务；不同release速率的native timer序也不同，不能当数学精确A/B。

## 产物

- [ABBA_1X_DESCRIPTIVE_COMPARISON.json](ABBA_1X_DESCRIPTIVE_COMPARISON.json)、[ABBA_SESSION_BINDINGS.json](ABBA_SESSION_BINDINGS.json)。
- 各 `replays/<轮次>/` 有PLAN、实际LOADED_BINARY、RUNTIME、23列event、7阶段lifecycle、严格STAGE_RECEIPT、INPUT_HEADER_MATCH、shadow_poses及DESCRIPTIVE_SUMMARY；原始输入索引在 `corpus_83e9/`。
- [PROSPECTIVE_ACTUAL_SEQUENCE_ADDENDUM.json](PROSPECTIVE_ACTUAL_SEQUENCE_ADDENDUM.json) 保留最初协议副本，记录用户当前授权的60s保护前缀后46有界尝试；未完成原验收项继续UNVERIFIED/FAIL，旧V18通过不冒充V19。
