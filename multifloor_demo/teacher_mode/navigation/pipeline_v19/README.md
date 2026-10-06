# V19：有界接收／解码流水线

本目录独立于已通过的 camera_mode 和旧 V18。冻结 Teacher 仍为 CPU 单线程，导航反馈使用实际 SLAM／IMU／点云和 SCAN；Actor 仍有 232 维仿真特权观测。300 ms 新鲜度、单执行器和停车保护保持。

## 已实现的并行边界

- 快速接收线程只接收原始消息引用与时间戳；LiDAR、图像各有独立解码线程。
- 接收顺序编号、ready 缓冲和单 owner 按序提交。全生命周期 reservation 限制为 512 项／64 MiB；不静默覆盖或丢弃。
- IMU 传播、LIO、VIO、状态及地图更新仍由同一个 owner 顺序执行。LIO 残差／Jacobian 使用 4 线程，VIO patch 保持实测较合适的 1 线程。
- 正常结束通过绑定进程身份的 SIGUSR1：停止接收、排空解码、顺序提交，最后关闭 ROS。紧急 context 无效时明确记录失败和未提交项。

因此理想稳态周期下界为 `max(接收阶段, 点云解码, 图像解码, owner整段估计)`，不是 `max(IMU, LIO, VIO)`；调度、同步屏障、消息大小和输出开销仍会影响实际周期。当前实现没有把共享状态估计器跨帧拆成独立 LIO/VIO。

## 当前证据

最终 C++ 库 SHA256：`f06e68d0dcbcab9f840077c372d0cda2e128f9935c4fe968987784e2124e00d8`。
56 项生产包语义、8 项实际 ROS 生命周期、100 个新进程的有限数值／FP／加载检查通过；这些检查不证明完整前端逐帧数学相同。

同一新 60 秒传感器包：1x ABBA 四轮均完整处理，30 Hz 输入供给下没有明显时效改善；1.5x 单次串行／流水线对照，最大观察位姿滞后分别 2.535 秒／50 ms。同步切片也随接收及时性变化，不能把所有耗时差归因于纯算子加速，亦不宣称普遍 1.5 倍性能提升。详见 [性能报告](../../test_results/pipeline_v19_20261006/performance/REPORT.md) 和 [容量探测](../../test_results/pipeline_v19_20261006/performance/CAPACITY_1P5X_REPORT.md)。

实际 Gazebo 60 秒短程 `20261006_021833_closed_loop_cascade_clock_hold_v19_prefix_r1_9eaf` 已结束：8/32 前缀区域、Actor 无故障、31,858 项全部提交，零取消／拒绝／遗留。随后原46区域两次运动尝试均8/46失败，不能用该前缀或旧V18结果代替。

## 操作

在项目根目录运行；首先检查没有另一个仿真占用本任务执行器，保留其他任务的训练。

```bash
python3 -B multifloor_demo/teacher_mode/navigation/pipeline_v19/run.py --profile pipeline_staged_60 --label v19_prefix --domain 88 --prepare-only
python3 -B multifloor_demo/teacher_mode/navigation/pipeline_v19/run.py --profile pipeline_staged_60 --label v19_prefix --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/pipeline_v19/run.py --profile pipeline_staged_original46 --label v19_original46 --domain 89
```

原 46 区域为探索 18＋返回 14＋本轮 RGB 地图保存＋导航 14，包括原动态障碍。三层用坡道连接，不是真实楼梯。首次 original46 启动在运动前因 ROS Node 属性命名冲突失败，失败 run `20261006_022027_closed_loop_cascade_clock_hold_v19_original46_r1_5a18` 保留，后续修复用新源码收据和新 run。

运行会生成独立目录、源码／模型／配置哈希、进程身份与存储预算。流水线源码变化需重新审核并生成新 gate；旧收据和失败不可改写。浏览器 `http://127.0.0.1:8768/?run=<run_id>` 显示实际 Gazebo 画面及实际 SLAM／SCAN 路线。新远程仓库 V19 当前提供源码与构建入口，移植运行仍阻断，不能沿用本机路径绑定收据。

## 原46区域控制参考修订

第二次实际运行 `20261006_022511_closed_loop_cascade_clock_hold_v19_original46_r2_4fc5` 在8个区域到达后失败：第9个区域前 SCAN 重规划改变局部切线，外层转向门仍锁定0.7799566 rad，串级控制追踪0.6024508 rad；外门误差大于0.1 rad，平移未能释放，触发原90秒航点超时。146,251项输入正常排空，不能把该失败归因于流水线积压。详见[原始日志故障图](../../test_results/pipeline_v19_20261006/root_report/heading_reference_failure.png)。

修订03明确改变控制参考传递：外门align期间，将已记录锁定朝向传入串级转向；drive恢复实际SCAN切线，capture/parking和保护逻辑保持。该修订不再宣称全部Python控制代码与V18逐字节相同；不改变C++估计、Teacher权重、增益、区域、dwell、超时或速度限幅。旧默认调用的等价回归和新参考故障回归分别记录；修订后的第三次实际46结果为8/46失败：旧参考冲突解除，但SCAN微小起始段的切线跳变104次触发原转向门，第9区90秒超时。详见[本轮最终报告](../../test_results/pipeline_v19_20261006/README.md)和[独立失败报告](../../test_results/pipeline_v19_20261006/evaluation/V19_9779_FAILURE_REPORT.md)。同6879 pose窗口源时间30.303 Hz、墙钟22.338 Hz，尚未持续满实时。
