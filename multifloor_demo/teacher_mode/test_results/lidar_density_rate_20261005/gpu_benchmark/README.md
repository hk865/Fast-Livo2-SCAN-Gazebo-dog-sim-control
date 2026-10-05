# Frozen Teacher：CPU / CUDA 实测，2026-10-05

**本次 batch=1 Teacher 推理保留 CPU 单线程。** CUDA 输出正确，但完整调用延迟更高；它没有为 LiDAR/SLAM 错误平面约束提供修复。实际 64 线、30 Hz LiDAR 试验的 Actor 仍使用 CPU，以便单独判断传感器变化的效果。

## 范围和方法

- 模型固定为 `model_1000.pt`，SHA256 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`，没有重新训练或修改模型。
- 输入来自 V7 实际 Gazebo 210 s 运行保存的 `observations_actions.npz`。10,501 个有限 247 维观测中均匀选取 1,000 个；CPU 重放输出与对应历史动作逐位一致。
- CPU 和 CUDA 使用相同 `247→512→256→128→12` ELU 权重；FP32、`eval()`、`inference_mode()`，TF32 关闭，CPU / interop 各 1 线程，无归一化、无量化、无 batch 合并。
- 已有环境 `/home/hyh001/isaacsim/kit/python/bin/python3`，PyTorch `2.10.0+cu128`，CUDA 12.8，RTX 5070 Ti（sm_120）。没有安装依赖。
- 先预热 200 次，然后串行 batch=1，CPU / CUDA 先后顺序逐次交替。CUDA 端到端计时包含 CPU 观测传入设备、forward、12 维动作回 CPU，并显式同步；没有使用异步 launch 时间冒充完整推理时间。
- 另外各测 150 次按 20 ms 周期调用（50 Hz），反映调用间隔对延迟的影响。device-only 指标保持设备输入驻留，分别记录 CUDA Event 时间及包含同步的墙钟时间。
- 计时不含观测构造、ROS/Gazebo IPC、日志、关节 PD、SLAM。该测试读取存档数据，不驱动仿真、不触碰其他训练进程。

## 结果

单位为 ms；“端到端”指上述推理调用范围。

| 测试 | 次数 | p50 | p95 | p99 | 最大值 | 超 20 ms |
|---|---:|---:|---:|---:|---:|---:|
| CPU 连续端到端 | 1,000 | 0.0376 | 0.0410 | 0.0436 | 0.7949 | 0 |
| CUDA 连续端到端 | 1,000 | 0.0873 | 0.1844 | 0.3883 | 0.5686 | 0 |
| CPU 50 Hz 端到端 | 150 | 0.3358 | 0.5239 | 0.6430 | 0.6654 | 0 |
| CUDA 50 Hz 端到端 | 150 | 0.6443 | 0.9568 | 1.2602 | 1.3097 | 0 |
| CUDA device-only 同步墙钟 | 1,000 | 0.0835 | 0.0886 | 0.2792 | 0.6302 | 0 |
| CUDA device-only Event | 1,000 | 0.0312 | 0.0610 | 0.2079 | 0.4441 | 0 |

GPU Event 的 p50 看起来低于 CPU 完整调用，但它省略了设备传输、同步等待及部分主机开销，不能替代端到端指标。50 Hz 下 CPU p50 约为原实际 V7 运行的 0.2899 ms 同一量级；连续紧循环的 0.0376 ms 不应当作为运行时承诺。

CPU 与 CUDA 最大动作绝对差 `7.152557373046875e-7`、平均差 `7.251339440017546e-8`，所有输出有限，`atol=rtol=1e-5` 比较通过。按 `q_target=q_default+0.25×action` 换算，最大单步目标差约 `1.79e-7 rad`。这里只验证同一观测的输出一致性，没有宣称更换设备后的完整闭环轨迹已验证。

## 资源与局限

资源快照在测试前、每秒及测试结束时只读采集。20 个逻辑 CPU、load1 约 3.12–3.22；GPU 总使用显存约 1,337–1,629 MiB、采样利用率 7–35%，基准进程自身约 288 MiB。PyTorch 分配峰值 9,697,280 bytes、预留峰值 23,068,672 bytes，不包含全部 CUDA context / 驱动占用。

没有发现正在执行的 RL 训练进程；本基准没有启动、停止或修改训练。GPU 仍有桌面、浏览器和远程桌面等其他使用者，快照不能将全部利用率归给模型。本测试没有与新 64 线、30 Hz Gazebo 渲染同时运行，也没有在训练饱和负载下测量；并发渲染/训练下的尾延迟需要单独记录。50 Hz 两种设备按阶段测量，阶段次序和操作系统调度也可能影响尾延迟。

当前 CPU 推理已远低于 Teacher 20 ms 周期，CUDA 没有实际延迟优势。Teacher 的 50 Hz 频率对应训练 `0.005 s × decimation 4`，更换设备也不意味着可以直接改变策略频率。CUDA Graph 等优化与多机器人大 batch 不在本次测试范围内。FAST-LIVO2 当前错误平面拟合、冻结和点云处理性能，应通过各自的数据与算法测试判断，不能由 Teacher 设备切换推断解决。

## 证据与复现

- `run_r1/results.json`：原始统计及输出一致性；`latencies.csv`：全部 4,300 条延迟。
- `run_r1/replayed_actions.npz`、`selected_observation_indices.npy`：真实观测选取索引及 CPU / CUDA 输出。
- `run_r1/metadata.json`：模型、观测归档、原策略 manifest、worker 与脚本哈希及完整运行参数。
- `run_r1/resources_before.json`、`resource_samples.json`、`resources_after.json`、`resources_after_exit.json`：只读资源证据。
- `benchmark_r1.log`：本次执行输出；`evidence_manifest.json`：本目录产物哈希。

使用新输出目录复现，不覆盖既有记录：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /home/hyh001/isaacsim/kit/python/bin/python3 -B \
  multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/gpu_benchmark/benchmark_teacher.py \
  --output multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/gpu_benchmark/run_r2
```

结论只适用于这台主机、本模型、batch=1、当前调用链及采样负载。它支持此轮保留 CPU，不是所有模型的 CPU/GPU 性能结论。
