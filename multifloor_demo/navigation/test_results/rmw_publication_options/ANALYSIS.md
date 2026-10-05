# 发布阻塞与异步发布：当前版本下的待验证选项

2026-10-02。只读技术分析，未创建 ROS context/节点，未编译或替换通信库，未修改任何生产环境、QoS、控制消息或健康期限。本报告不选择策略，也不把 Full17 的回调空窗认定为 DDS 发布阻塞。`installed_receipt.json` 保存本机包版本、库/头文件 SHA、分析输入 SHA 和当前分析 shell 环境；`primary_sources/receipt.json` 保存官方精确 tag 源文件 URL、SHA 和字节数。

## 本机及实际加载证据

| 组件 | 已安装版本 |
|---|---|
| `rmw_fastrtps_cpp` / `rmw_fastrtps_shared_cpp` | 8.4.4，Ubuntu 包 `8.4.4-1noble.20260615.*` |
| Fast DDS | 2.14.6，包 `2.14.6-1noble.20260303.233638` |
| Fast CDR | 2.2.7，包 `2.2.7-1noble.20260225.051855` |
| `rmw_implementation` | 2.15.6 |

安装的 `rmw_implementation` CMake 指定默认 `rmw_fastrtps_cpp`；CycloneDDS/Connext 的对应 RMW 包未安装。近期实际组件 `20261001_final_cm_dynamic/actual_gz_process.json` 的 Gz PID 847794 `/proc/maps` 确认加载同一组四个通信 DSO，其 SHA 与本机收据一致。这是该次 Gz 进程的证据，不能代替 Full17 Adapter 进程的环境/实际 writer 模式。

当前分析 shell 的 `RMW_IMPLEMENTATION`、`RMW_FASTRTPS_PUBLICATION_MODE`、`RMW_FASTRTPS_USE_QOS_FROM_XML`、两种 profiles-file 环境变量均未设置。当前生产源码检索未发现设置发布模式的分支。这不排除启动继承环境或工作目录中的外部默认 XML；Full17 进程未保存上述全部参数，需要冷启动诊断重新采集。

## 精确版本能说明什么

1. **默认同步；异步只把发送部分移出调用线程。** 8.4.4 在未指定模式时选择同步。异步会将数据加入内部队列，再由后台线程发送；RMW 调用仍须完成序列化/写入历史等步骤。`publish()` 返回与执行端收到/执行零速是不同事件。[rmw_fastrtps 8.4.4 说明](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/README.md)、[实际 RMW write 包装](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_publish.cpp#L59)

2. **异步不保证调用不阻塞。** Fast DDS 2.14.6 `perform_create_new_change()` 的公共路径先获取 writer mutex，再分配/序列化并加入历史。安装生成的 `fastrtps/config.h` 默认 `HAVE_STRICT_REALTIME=0`；同 tag 在此条件下使用普通等待锁，只有 strict 分支用截止时刻获取锁。生成头文件不是对每条 DSO 指令的反汇编证明；至少不能把 Reliability 默认 `max_blocking_time=100 ms` 当成本机所有调用路径的硬实时上界。330 ms 观测与这一机制并不矛盾，也不能反过来证明那次停顿就是此锁。[DataWriterImpl 精确源码](https://github.com/eProsima/Fast-DDS/blob/v2.14.6/src/cpp/fastdds/publisher/DataWriterImpl.cpp#L977)、[官方非阻塞前提，2.14.x 页面当前标为 2.14.7](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/realtime/blocking.html)

3. **Reliable 与 depth 不提供“零速优先”。** 生产 Adapter 的 Twist、JointTrajectory、String 三个 publisher 均传 depth 10，默认请求 KEEP_LAST/RELIABLE/VOLATILE。Fast DDS 历史满时 KEEP_LAST 可移除最旧样本，KEEP_ALL 会等待已确认样本释放；内部锁和资源依然存在。8.4.4 QoS 转换只保证 DDS depth 不小于 ROS 请求，XML 配置更大的 depth 可能保留。实际组件 graph 返回 history UNKNOWN/depth 0，不能据此称实际队列深度为 0；需采集有效 writer QoS。[历史处理源码](https://github.com/eProsima/Fast-DDS/blob/v2.14.6/src/cpp/fastdds/publisher/DataWriterHistory.cpp#L156)、[RMW depth 转换](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/qos.cpp#L100)

4. **不同话题没有共同原子提交顺序。** 同一 writer 的序列与可靠通信约束不能证明 Twist 零速、关节返回目标、健康状态三者在不同 reader 上同时生效。队列可使旧非零或旧“健康”消息延后到达。增大 history 也可能增加滞留；改 best-effort 会使 publisher 与现有 reliable reader 不兼容，并使零速可能丢失。[Fast DDS 可靠性与兼容规则](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/dds_layer/core/policy/standardQosPolicies.html)

## 当前故障证据边界

Full17 `20261001_232612_ad2317` 的本地日志显示 Adapter 全回调记录约 337 ms 空窗；恢复后 Adapter 仍 idle/未 failed，Bridge 已锁存 `joint_adapter_status_timeout`。独立原始 IMU 当时持续更新。日志缺少对应三种消息的发布 enter/return 与真实执行端 receive，所以发布阻塞、线程未调度、回调/诊断 IO 等仍是候选。不能把发布前/后应用日志当作实际 DDS 或 JTC 收到的时刻。详细证据保存在 `../full17_startup_status_audit/`。

Adapter 健康门限为 **0.30 wall s**；Bridge 导航命令门限为 **0.40 wall s**。Adapter 的 safe-command、新鲜 raw target/native nominal ACK 各有原 0.25 s 门限。这些合同以及原 IMU、停止握手、运动上限、区域/到达期限均保持。本报告没有申请延长它们。

## 最小可验证选择，尚未采用

| 选择 | 只隔离什么 | 能检验的假设 | 仍需证明/不能宣称 |
|---|---|---|---|
| 保持原模式，先补原生边界观察 | 相同三 writer 的 enter/return、reader callback enter，实际消息字段与 writer identity | 哪次调用占用空窗；发送端阻塞还是接收端停顿 | 观察 hook 也有开销；须有无 hook 对照，空窗未复现不能称消除 |
| 仅 Adapter 进程 `ASYNCHRONOUS` 对照 | 同消息对象内容、发布顺序、QoS、控制与 timer；单变量发布模式 | 若真正阻塞位于同步发送，能否减小该回调占用 | 公共 writer 锁/历史仍可能等待；必须测实际零速到达和队列年龄，不能只测返回变快 |
| 仅被证实阻塞的话题 writer 异步 | 按 topic 的 XML profile，其它 writer 保持原模式 | 避免同时改变全部 Adapter writer | 8.4.4 源码显示 `AUTO` 不覆盖 profile 的 publish mode，同时保留默认 memory/datasharing 设置；这是待 native-getter 验证的源码推断，非已验证配置 |
| 若库模式不足，再隔离有界发布 worker | 未来代码候选；不可在本轮直接实施 | 执行回调是否可免受单次发送阻塞 | 新停止/对象生命周期合同需独立实测；后台线程活着不能作为执行健康 |

第三项的具体依据：`publisher.cpp` 先读取以完整 ROS topic 命名的 profile；当 `USE_QOS_FROM_XML` 不是 1 时，只显式覆盖 SYNCHRONOUS/ASYNCHRONOUS，AUTO 分支不覆盖 profile 模式，并维持 PREALLOCATED_WITH_REALLOC/data-sharing-off。若设置 `USE_QOS_FROM_XML=1`，则还可能同时改变 memory policy/datasharing，不能称只改发布模式。实验必须证明实际加载的 XML 文件/SHA、完整 topic profile 和 native writer mode/QoS；当前没有生成或应用任何 XML。[8.4.4 publisher 源码](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_cpp/src/publisher.cpp#L234)、[participant 环境分支](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp#L277)

降低 writer max-blocking-time 或重编 strict-realtime 是另外的候选，不能在当前非 strict 路径上当保证；写超时/失败须继续触发真实错误，不能吞掉错误后报告健康。更换 RMW、增加 depth、只把 depth 降到 1 或延长健康期限不是本报告选定的修复。

## 每个候选共同的测试合同

- 固定实际 writer handle/type/topic/PID/library SHA；记录模式、有效 QoS/depth/resource limits/max-blocking-time、环境和已加载 profile SHA。无 graph 深度证据时明确未知。
- 原 Twist 六分量保持逐值一致；JointTrajectory 保持关节名/位置/schema、header 0、16,666,666 ns horizon 和相同 C0 归位对象；String 的真实状态/failed/nominal/counters 不伪造。未来 worker 必须在入队前冻结对象副本，直到实际写完才释放，不能复用正在序列化的可变对象。
- 保留原零速→native nominal ACK→C0 返回→实际非零恢复合同。零速边沿必须从原发布对象追到 CHAMP/JTC 实际接收/应用；旧非零绝不能在保护/返回期重新生效。跨 writer 顺序必须实测，不能从单一队列顺序推出。
- 记录每条 publisher generation/sequence、真实 source wall 与 receive wall、消息指纹及同进程事件。健康以原真实进展定义评估：排队成功/发布返回/收到旧健康不能被用来刷新一个实际上停顿的执行链。若现有无 Header 消息不足以证明新鲜度，先补只读旁路配对，不先更改健康实现。
- 同源单变量 baseline/candidate，包含消费者暂停/历史压力、发布失败、重复/迟到旧非零、零速紧跟非零、停止握手恢复、断连/重启、健康队列停顿反例。维持所有原 wall/sim 期限；超时即失败，测试 cleanup 必须退出真实 owned 进程。
- 当前冷启动为全零，可证明启动传输边界却不能验证移动中紧急零速。因此冷启动改善仍须后续停止/恢复实际组件验证，不能直接当完整多楼层 Demo 通过。

待 Sim 返回实际发布边界后再决定：若 enter→return 覆盖空窗，按其真实 topic 和调用路径选单变量测试；若调用均快速但 receive 延迟，优先核接收队列/调度；若没有 native publication 事件，则不能用异步发布解释或修复一个尚未定位的停顿。
