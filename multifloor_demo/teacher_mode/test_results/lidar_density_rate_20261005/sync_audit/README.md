# V8 LiDAR 64 线 / 30 Hz 的同步、吞吐与多核只读审计

本审计读取已结束试验、冻结 V7 源码及其实际编译命令，没有运行 ROS/仿真、发送进程信号、修改算法或冻结目录。证据与 SHA256 绑定在 [evidence.json](evidence.json)，复现脚本为 [analyze_sync.py](analyze_sync.py)。没有重新整读 46 GB binary 做哈希；绑定完整 index、writer stats、日志哈希与原二进制字节数，分析只 seek 小型记录字段。

## 结论

当前证据指向前端串行消费吞吐不足和控制保护的一处重复计算时钟误判，不能归结为整机硬件耗尽。两次试验失败原因不同：

| 试验 | 传感器配置 | 结果与直接原因 |
|---|---|---|
| `20261005_192400_closed_loop_cascade_lidar64_30hz_rgb10_r1_f728` | 64×480，LiDAR30 / RGB10 / IMU200 | 22/32；190.365s 报 native guard 时钟重复。实际触发使用的是相同 190.270s 计算时钟，物理与策略交换时钟没有回退。 |
| `20261005_192802_closed_loop_cascade_lidar64_30hz_rgb30_r1_cb5c` | 64×480，LiDAR30 / RGB30 / IMU200 | 3/32；61.285s 报有效 SLAM/云连续失联超 8s。传感器继续采集，前端持续处理旧输入，输出 acquisition header 超过原 300ms TTL，被正确拒绝。 |

20 Hz 会把名义帧预算从 33.3ms 增至 50ms，是合理的独立试验。这里没有20 Hz通过结论，也没有多核提升或导航通过结论。

## acquisition、同步与队列证据

LIVO 的 `sync_packages()` 以相机 header 作为 LIO 更新目标，随后同一目标做 VIO。因此 RGB10 时 LiDAR30 不会直接变成 LIO30：约三帧云合到一次相机区间。`camera30` 的实际点云/图像 header 增量受5ms物理步长量化为30/35ms，全程6364帧覆盖0.010–209.980s，即约30.305个 acquisition header/s。应同时报告墙钟速率，不能拿仿真 Hz 直接与墙钟吞吐比较。

RGB30 的启动成功：6.65s 冻结初始化，26对样本覆盖0.825s，最大因果IMU配对差10ms。原10样本容量不能同时覆盖30Hz下0.8s；V8窗口扩大至 `max(10, ceil(camera_hz*0.8)+2)`，其余稳定性、TTL及因果配对门未放松。

同一个 LIO 处理墙钟区间内的实际到达与消费如下。队列等待是同一相机 header 的 callback wall time 到 LIO Process2 start wall time，非真值位姿误差。

| LIO source区间(s) | RGB到达 wall Hz | 完整 LIO→下一 LIO wall Hz | 相机→LIO开始等待中位数 |
|---|---:|---:|---:|
| 5–20 | 30.036 | 30.035 | 9.20ms |
| 20–40 | 30.031 | 29.699 | 232.71ms |
| 40–53 | 29.952 | 29.869 | 235.11ms |
| 53–100 | 30.087 | 28.970 | 929.21ms |
| 115–165 | 29.599 | 24.004 | 9.287s |
| 170–190 | 30.245 | 29.840 | 14.488s |

在61.255s，raw RGB/LiDAR/CameraInfo墙龄都≤37ms，rawIMU墙龄约0.25ms，但最新body_odom和云header已有435ms旧。控制器最后一次接受的pose是52.475s，callback时ROS52.770s、header龄295ms，后续越过300ms阈值。61.285s状态的pose/cloud墙龄约8.55s，指的是最后一次通过TTL检查的输入，不表示算法8s没发布。

在约200s，rawRGB/CameraInfo/LiDAR/IMU仍持续新鲜，而body_odom/云header已有14.525s旧、callback墙龄仍≤8ms。最终前端kind1 IMU callback只消费到196.070s，raw gate已经收到210.010s；LIO到196.090s、VIO到196.055s。前端线程没有崩溃，owned父进程及子进程均干净退出、Teacher50Hz仍在运行，无停止另一训练的信号。

`[DEMO_SYNC] complete=0`不能一概解释为FIFO后落。相机事件到达时尚无足够IMU也会出现complete=0，早期5–20s有约80.44% complete；20–40s降到7.26%，40–53s及以后为0。冻结配置 `require_complete_imu=false` 只要求覆盖 `last_lio_update_time+0.01`，并不等待到当前camera目标。后来队列后落与不完整coverage并存：通常目标比最新IMU超前20–25ms，每次只选2/3个IMU；50s目标时最新LiDAR已50.230，最新IMU49.975；150s目标时最新LiDAR约161.44。IMU和最新LiDAR callback差值告警支持后落，但不足以排他判定每一条告警原因。

## 计算与资源

探针stage枚举是 `VIO=1 / LIO=2`。下表LIO计时为Process2开始至StateEstimation结束，VIO至processFrame结束。完整循环还包含地图更新、投影/图像输出、序列化、callbacks/sync及等待，不能把其差值全部称为MapUpdate。

| RGB30 source区间 | LIO均值 | VIO均值 | 完整循环均值 |
|---|---:|---:|---:|
| 20–40s | 23.504ms | 2.500ms | 33.671ms |
| 40–53s | 23.504ms | 2.637ms | 33.479ms |
| 115–165s | 28.401ms | 4.177ms | 41.660ms |

此run有20逻辑CPU，记录的最高1分钟load为10.457，可用RAM最低约43.0GiB，GPU利用率最高14%、显存1617MiB。没有全机CPU/RAM/GPU耗尽证据；load也不是单线程CPU利用率。代码 `spin_some()` 后串行推进一个LIO或VIO stage与上述到达/消费差，是更直接的证据。现有日志未单独计量所有发布、GPU渲染或磁盘成本。

实际编译已是Release，`-O3 -DNDEBUG -O3 -funroll-loops -fopenmp`。actual compile_commands没有 `-DMP_EN`，所以源码已有的两个OpenMP循环未开启；链接OpenMP不等于循环并行。CMake没有定义这两个宏，不能仅由缺宏推断历史原因。源码还明确为PCL/Eigen ABI兼容不使用 `-march=native`，不能盲加宽向量对齐或fast-math。

诊断writer写出46,493,267,096字节、259333 records、0drop、队列峰值66、无IO错误。kind101约24.599GB、kind102约15.847GB，kind4约1.785GB。enqueue使用vector移动及异步writer，并无整vector二次deepcopy；但向量填充、Eigen矩阵/查询复制仍发生在估计器主线程，`BuildResidualListOMP`在enabled时每点填60个double的query metadata，即使不在detail窗口。首个失败61.285s早于115s细节窗口，所以大日志不能作为最初失联的唯一原因；115–165s期间吞吐继续明显恶化，可作为独立减开销A/B。

环境变量缩窄 `FASTLIVO_DIAGNOSTIC_BEGIN/END` 并不能关闭全部大日志：`LIVMapper.cpp:979` kind4窗口仍硬编码115–165s。全程保留同步/header/source记录，短detail只对其余详细数据减量。`mat_pre/mat_out`每个LIO/VIO使用 `std::endl` 同步flush；Teacher worker也是写telemetry/height JSON后才回socket。单次策略交换105.54ms但对应网络前向仅0.308ms，说明GPU搬模型无法保证解决交换/前端后落，具体I/O、调度、等待比例还需独立测量。

## 重复计算时钟保护

f728末尾3次墙钟20Hz控制tick使用相同 `compute_ros_clock_ns=190270000000`；feedback/IMU/ACK/path receipt/pose/target/direction/prepared command canonical字段SHA一致。后两次是 `duplicate_SLAM_header`，`controller_updated=false`，没有重复积分。native physics_step42001条、policy_exchange10501条各自t严格递增，无回退。190.270s policy_exchange耗时105.54ms，IMU相邻20ms仿真区间跨124.15ms墙钟，支持短暂停顿。

但不能把相同时钟一概忽略：190.270s guard第一次用cloud190.100，随后真实新cloud190.200在计算时钟尚未推进时收到，第三tick有重新验几何的必要。故最小正确逻辑是：真正compute-clock倒退停车；相同时钟先继续执行原sim/wall300ms freshness；完整guard-input指纹相同才复用guard结果、不给积分/停稳/恢复计时续时间；有新acquisition cloud/pose需重算几何但禁止增加时间；同一acquisition header而payload变化不可当普通重复忽略。300ms保护保持。

## 多核候选与不改数学的减开销

第一候选为独立core build仅给 `src/voxel_map.cpp` 源文件定义 `MP_EN;MP_PROC_NUM=2`，保持VIO串行。`BuildResidualListOMP`（777–995行）中 `pv_list[i]`、query row、PointToPlane均独立；压缩 `vector<bool>`写已有同一mutex保护（874–886行）；递归残差构造只读map/plane，正常map update在同一个主线程StateEstimation之后；parallel barrier后按原i顺序串行收集。没有LIO浮点归约或parallel push_back。没有发现必需的新collector补丁。

这支持优先实际构建/测量，但本审计没有证明多核收益或位级一致。应在隔离fixture/真实输入回放逐项比较accepted residual、iteration、state25、cov361及集合顺序，性能另行测量。原 `omp_set_num_threads(2)`会留在当前serial线程，可能影响其他库后续OpenMP；mutex和scheduling可能限制收益。旧fixture源在 `/tmp/go2_v7_lio_equivalence_fixture.cpp`，旧驱动会写冻结V7目录，须复制到新输出并改路径后使用。

全target `-DMP_EN`会同时启用VIO `updateState()`（1747–1890行），当前inverse_composition=false确实走此分支。其H/z/errors按独立索引写，projection Jacobian输出局部；但float `error`为OpenMP reduction、不同求和次序可能改变 `error<=last_error`接受/回退。因此不能宣称“开两线程绝不改变结果”，VIO应后续独立数值验收。

可按独立A/B实施的其他减负包括：短detail诊断、减少每帧debug文本或flush、无消费者的 `/rgb_img`输出转换、彩色 `/cloud_registered`后处理投影/序列化、完整历史 `/path`发布及无消费者MAVROS pose输出。当前effect points、plane markers、PCD、Colmap、image_save、evo都已关闭，不能再归功这些开关。增大pub_scan_num会攒多帧点云，不能默认降低投影总点数。`dense_map_en`同时影响导航云点数，不能当显示开销开关。

所有候选必须保留原算法RGB/IMU/LiDAR/CameraInfo输入、导航 `/cloud_registered_full`、odom、真实CameraInfo及必要实际记录；不降低安全门或用真值替代导航。以上是源码可隔离的测试候选，不是本报告已经运行、通过或授权修改所有算法的声明。

## 复现

仅离线分析已有两run，不运行仿真：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
python3 -B multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/sync_audit/analyze_sync.py
```

evidence含每份关键日志SHA256、实际core/mapping二进制契约、源码与实际编译flags、原始source/receipt时序、窗内墙钟吞吐、queue等待、首个失败附近与200s采集门、native重复compute-clock比对。保留两次失败原记录，不修改V7/V8归档。
