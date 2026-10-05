# 同步语义与输出差异核对

此审计读取六轮既有结果，不重跑、不修改同步门、不使用Gazebo真值。它说明同bag内容经过不同callback调度后，前端实际分段和状态并不保证逐字节相同；因此1.5x的计算耗时差不作为纯算子提速强因果。

## complete 的原源码含义

最终V19 `fast_livo2_core/src/LIVMapper.cpp` 的 `sync_packages()`、LIVO的 WAIT/VIO→LIO 分支：

- 行1443–1444取已提交进owner `imu_buffer.back()` 的header时间为 `imu_newest_time`，当前目标是图像时间加冻结曝光offset。
- 行1463–1466：`imu_newest_time < (require_complete_imu ? img_capture_time : meas.last_lio_update_time + 0.01)` 才等待。当前六轮与capture全部 `require_complete_imu=false`，故只达到前一LIO更新+10ms也可继续，并不要求达到当前图像target。
- 行1474–1482创建 `m.lio_time=img_capture_time`，只从当前已提交buffer选 `last_lio_update_time < stamp <= m.lio_time` 的IMU。
- 行1490–1494打印 `complete = int(imu_newest_time >= img_capture_time)`，记录当前LIO组是否被owner buffer内IMU头覆盖；该flag没有计数丢包。
- 行1548–1554保存该组、标记LIO并返回true。下一次caseLIO进入VIO分支，不是等待后重新构建同一旧LIO组，且该VIO分支不打印DEMO_SYNC。因此complete0并非标准LIO/VIO交替中必然一半为false。

每轮恰有1818个不同camera target的DEMO_SYNC。输出中的1–2ns严格比较边界与>1us的明显target未覆盖分别统计。

|轮次|当前LIO组complete0|其中target超过owner最新IMU>1us|最大未覆盖(ms)|
|---|---:|---:|---:|
|A1_serial_1x|23|13|10.000000|
|B1_staged_1x|7|0|0.000002|
|B2_staged_1x|6|0|0.000002|
|A2_serial_1x|21|10|5.000001|
|C1_serial_1p5x|909|890|25.000000|
|D1_staged_1p5x|5|0|0.000002|

串行1.5x有890次明显target未覆盖，最大25ms；流水线1.5x没有>1us未覆盖。所有sensor headers最终都被提交，说明这是处理时刻的buffer不足，不能说890个包永久丢失。忙于估计时串行executor不能及时处理各topic、同步分段选择随调度改变，与这些现象一致；这一小对照没有排除DDS/host调度等因素，不能声称硬件是唯一原因或算子单独变快。

## 原source stamp精确配对位姿

配置与输出 frame均相同；按原整数odom header配对，不做插值、刚体对齐或初始姿态校正。每轮与A1共1709个header完全配对。下列为两个估计结果的差异，**不是对物理真值的精度**。角差为归一化四元数最短旋转角。

|相对A1|位置RMS/max(mm)|角RMS/max(°)|部分完成窗query/StateEst次数|
|---|---|---|---|
|B1_staged_1x|0.191/1.752|0.003640/0.023819|5113/1709|
|B2_staged_1x|0.186/1.641|0.003581/0.021552|5104/1709|
|A2_serial_1x|0.188/1.761|0.003445/0.023367|5112/1709|
|C1_serial_1p5x|3.234/15.978|0.092441/0.720036|5792/1693|
|D1_staged_1p5x|0.190/1.672|0.003670/0.022077|5108/1709|

C1serial1.5x相对A1的误差更大，同时有更多query调用；这种同步/状态/迭代路径变化就是11.21ms与9.25ms不能直接当同数学工作量加速的理由。源码每次StateEst迭代调用一次BuildResidualListOMP，因此kind300提供已完成1秒窗的部分总query/StateEst次数；最后不足1秒窗口未flush，不伪装全运行迭代总数。

本回放detail窗口115–118s，输入仅0–60s，kind100/201全为0。每帧LIO迭代数、VIO pyramid迭代/rollback数仍UNVERIFIED，不能将0条detail记录解释为0次迭代，亦不能从processFrame数倒推。

可复现来源：[EXACT_HEADER_POSE_DIFFERENCES.json](EXACT_HEADER_POSE_DIFFERENCES.json)、[LOGGED_SYNC_COMPARISON.json](LOGGED_SYNC_COMPARISON.json)、[LOGGED_SYNC_READER_BINDING.json](LOGGED_SYNC_READER_BINDING.json)。后者绑定最终源码、读取器与实际log SHA；读取器重复计算与首份结果完全相等。
