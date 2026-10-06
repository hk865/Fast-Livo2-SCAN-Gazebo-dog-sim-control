# 点云传输独立实证：512 KiB / 64 MiB

本轮完成了两次真实60秒仿真站立测试。两轮使用相同的显式loopback UDP+SHM配置，XML仅`segment_size`不同；CPU Teacher、物理世界、零速度指令、50 Hz策略及300 ms原门限均保持不变。64 MiB轮实际点云交付明显更连续。这个结果支持在同配置下增加容量改善整条ROS传输链，尚不能断言每个消息实际经过SHM，也不是路线控制或导航验收。

独立原始收据与图保存在 [cloud_transport_actual_20261004](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/cloud_transport_actual_20261004/summary_cloud_transport_comparison_independent.json)。分析只读实际日志，未启动ROS、仿真、控制器或任何实体机器人。

## 本轮配置和证据

| 项目 | A：512 KiB | B：64 MiB |
|---|---|---|
| 实际RUN | `20261004_135325_stand_cloud_transport_shm512k_r1_2a0e` | `20261004_135616_stand_cloud_transport_shm64m_r1_abf3` |
| 探针目录 | `shm512k_01/probe` | `shm64m_01/probe` |
| 主进程退出 | 6/6为0 | 6/6为0 |
| 只读探针退出 | 0 | 0 |
| 实际clock观测跨度 | 60.0 sim s | 60.0 sim s |
| 元数据收据写入/提交 | 13,238/13,238 | 13,721/13,721 |
| 源码、队列、时钟回退错误 | 无 | 无 |
| 主进程实际环境/maps快照 | 7个实际进程 | 采样晚于结束，0个；未补造 |
| 探针自身实际环境/maps | 已记录 | 已记录 |

世界XML只将本轮输出目录名称归一化后完全相同，运行与探针各自XML逐字节一致。环境比较只排除各自XML文件路径，其他受控字段相同。探针使用同一冻结源码，显式RELIABLE和BEST_EFFORT各订阅同一`/cloud_registered_full`，同时订阅真实SLAM的`/demo/slam/body_odom`和`/clock`。实际publisher graph分别是`laserMapping`、`demo_slam_odom_adapter`、`ros_gz_bridge`。

原始MessageInfo的DDS序号、GID、源/接收时间字段在本机回执中均不可用，保持null。以下配对严格用原始整数header stamp，未经浮点反建、容差配对或插值。相同header的两点云订阅在shape、fields、stride、`data_bytes`等元数据上完全一致；探针不解码、复制或散列点云payload。

## 实际交付与连续新鲜度

| 测量 | A：512 KiB | B：64 MiB |
|---|---:|---:|
| 可靠点云回调/独立header | 472/472 | 563/563 |
| 尽力点云回调/独立header | 265/265 | 563/563 |
| SLAM body odom回调/独立header | 472/472 | 563/563 |
| 尽力点云与可靠点云的精确共同header | 265；另缺207 | 563；无缺项 |
| 可靠点云与body odom精确共同header | 472，集合相同 | 563，集合相同 |
| 可靠点云最大header间隙 | 0.400000001 s | 0.100000001 s |
| 尽力点云最大header间隙 | 1.4 s | 0.100000001 s |
| 可靠点云最大实际wall回调间隙 | 0.418286 s | 0.149095 s |
| 尽力点云最大实际wall回调间隙 | 1.400220 s | 0.148956 s |
| 可靠点云到包源龄上限 | 0.055000001 s | 0.05 s |
| 尽力点云到包源龄上限 | 0.04 s | 0.05 s |
| 到包时源龄超原300 ms门限 | 两订阅均0 | 两订阅均0 |

“收到的包新鲜”没有保证“包与包之间缓存始终新鲜”。独立重放按探针实际回调先后顺序，在每个已收到的原始整数clock取此前真正交付的最后一帧，同时检查原`[-.05,.3)`源龄和wall缓存龄。预热前没有输入的clock单列，没有补造输入或通过值。此重放是该只读进程的离线缓存审计，本轮未启动导航门禁，不能称实际门禁通过。

| 首次真实交付后的缓存重放 | A过期clock样本/占比 | A实际采样区间过期时长 | B过期样本 |
|---|---:|---:|---:|
| 可靠点云 | 197 / 1.752% | 0.985 sim s | 0 |
| 尽力点云 | 2,337 / 20.810% | 11.695 sim s | 0 |
| body odom | 164 / 1.458% | 0.815 sim s | 0 |

A尽力点云最长缓存过期段1.135 sim s；可靠点云最长0.155 sim s。B三条流从首次真实交付到探针结束，没有原双300 ms门限违例。完整60秒曲线如下，启动阶段没有输入的缓存曲线留空。

![实际交付和缓存源龄](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/cloud_transport_actual_20261004/cloud_transport_actual_comparison.png)

图及原始输入SHA记录在 [figure manifest](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/cloud_transport_actual_20261004/cloud_transport_actual_figure_manifest.json)，已进行像素检查。图不展示合成点云或合成接收事件。

## 消息规模与其他实际消费者

点云`point_step=48`字节。A可靠订阅payload为707,760～2,833,728字节，中位708,432；B为707,760～708,480字节，中位708,432。所有点云payload均大于524,288字节。这个值仅为`PointCloud2.data`长度，不是完整DDS序列化长度。

实际SHM设置还包括`maxMessageSize=65500`、`port_queue_capacity=512`，两轮均未改变。不能仅以payload大于segment就断言某帧必然丢弃或必然走另一transport；本轮没有逐消息路径追踪。配置作用于全部本轮ROS参与者，包含相机、SLAM输入和输出，不只点云消费者。因此A处理切片数量和point payload bundling也与B不同，应将“ROS链整体交付改善”和“某一个订阅的损失机制”分开。

SLAM原日志有A502/B600条`DEMO_LIDAR_SLICE`处理记录。这些是输入切片日志，不是逐调用发布计数，也不是原始整数header发布回执。原始完整DDS发布序列仍未验证。

| 其他真实消费者 | A | B |
|---|---:|---:|
| sensor_shadow实际SLAM回调 | 473 | 564 |
| sensor_shadow实际full cloud回调 | 233 | 324 |
| map archive最终metadata：odom | 473 | 564 |
| map archive最终metadata：full cloud | 272 | 564 |
| map archive最终metadata：colored cloud | 472 | 563 |
| map archive最终metadata：camera | 483 | 601 |

不同消费者并非同一executor或相同负载。B探针完整563帧，不意味着其他进程也完整收取；shadow仍只保存324个真实cloud回调，233/323帧分别能与探针可靠流精确配对，B另一个末帧在探针结束后到达。探针在60.01 s clock达到目标后结束，shadow/archive仍可能收取最后60.0 s的发布，因此其odom/full cloud多1帧不当作重复发布。Archive这里只保存最终汇总metadata，没有逐回调整数记录，不能逐帧配对或证明其完整缓存新鲜度。

本轮没有启动路线controller或sensor_gate，RUN不存在PID cloud callback、gate history、命令转发记录。实际路线消费者连续性必须由后续同64 MiB配置的真实PID轮独立验收；本轮结果不能代替它。

## 归因边界和操作结论

本次A/B仅segment容量不同的配置一致性通过；本轮同环境下64 MiB改善的证据强于历史不同配置的比较。历史`ROS_LOCALHOST_ONLY=1`会改变discovery/transport构造，与本轮`ROS_LOCALHOST_ONLY=0`的显式loopback transport不同，不用于纯容量归因。

每个容量只有一次实际站立诊断，配对可靠订阅也可能影响共同publisher的重传和背压。B主maps未及时采集，主环境依运行manifest，不能把后续PID的实际maps回填为B站立轮的证据。尚未证明所有DDS链路或运动中都始终新鲜，更不能用此轮宣布Teacher/PID、Sim2Sim、多楼层或真机通过。

后续可使用已实测更连续的64 MiB设置做原PID参数的前瞻真实测试；保持300 ms保护、原到达/停车/运动判据不变，并保存实际controller、gate和consumer收据。已有失败与验收原件均未修改。

独立分析命令：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_cloud_transport_actual.py --no-write
```

脚本只读回放；省略`--no-write`只写本轮新独立收据和图，不修改实际RUN、旧PID验收、协议、训练或仿真进程。
