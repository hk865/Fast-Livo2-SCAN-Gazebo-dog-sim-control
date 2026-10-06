# Full17 启动故障的导航与执行桥边界

原运行 `20261001_232612_ad2317` 保持 **FAIL**。本审计只读记录和冻结源码，没有启动 ROS、测试节点或物理仿真，没有改生产文件或门限。八个相关当前源文件与 `full17_freeze/source_manifest.json` 逐一匹配；Root 的原终态证明记录全部来源/runtime 未变、owned 进程清理完成。

## 已确认的时间线

| 记录 | 实际数据 | 解释边界 |
|---|---|---|
| 初始门放行 | sim 13.0；10.0–13.0 共 3001 个真实静稳 IMU；Bridge ready；gate 墙钟耗时 17.9904 秒 | 一次启动前静稳检查通过，不保证其后进程持续无阻塞 |
| 放行后 | stack 日志才启动 FAST、SCAN、body adapter、map archive、NAV 等子进程 | 新模块启动与故障相邻；不能仅据先后顺序断言其导致阻塞 |
| 最后已记录的适配器事件 | filtered_target：sim 13.843、adapter-relative wall 18.811190 | 这是一条同步 publish 返回后的日志，其时间戳在日志写入前取得 |
| 下一适配器事件 | actual_champ_command：缓存 sim 13.848、wall 19.148427 | 两条事件间 **337.237 ms** 没有任何记录推进；它并非真实 publisher entry 时间 |
| 恢复回调 | 约 1.8 ms 后缓存 clock 从 13.848 跳到 14.178；raw/filtered/safe 回调随后重新推进 | 多个不同话题一起停顿，单独 status DDS 迟到不足以解释全部现象 |
| 独立原始 IMU | .001→14.399 每帧原始戳步长恰 1 ms；sim≥10 接收墙钟最大间隔 **3.114 ms** | 反证全仿真/所有 ROS 接收同时停顿；没有证明适配器进程自身为何停顿 |
| Root 任务失败 | elapsed 20.366 秒；`joint_adapter_status_timeout` | 通过 Mission 直接接收 failed Bridge 状态结束 startup；未发送路线请求 |
| 终态 Bridge | IMU ready；adapter idle/failed=false/nominal=true，收包 age 13.233 ms，但整合状态仍 failed | 早先超时已锁存；后来的新健康消息不会撤销正确的失败记录 |

以上 wall 字段分别以 gate、Adapter、recorder、Root 的自身起点为基准，**没有将不同进程的相对墙钟当成同一时轴**。原日志没有 Bridge 首次失败回调的单调时间，因此不能给出精确 latch instant；官方 NAV 只留下 6 个 idle 状态，其中最后一份仍缓存 Bridge ready@IMU13.999/adapter13.836、age .163 秒，不能冒充失败时的新消息。

## 门限与传播

`execution_safety.py` 的 `JointAdapterHealth(timeout=.30)` 是本次适配器健康超时。`control_bridge.py` 实际直接构造它，没有覆盖参数。Bridge 的 `.4` 秒是独立的 **导航命令** watchdog；Adapter 自身 safe-command/raw-motion watchdog 是 `.25` 秒。三者均保持原样。

Bridge `state()` 可以在原始 IMU、timer 和 adapter 回调里检查并锁存 `.30` 秒超时。adapter 回调先执行 `before=self.state(now)`，再 `update(data,now)`；因此即使刚到达的消息健康，也可能在更新前确认上一次收包已超时，之后 failure 永久保留。最终短 age 与 failed 并不矛盾。

Mission 的 `on_control_safety()` 独立将 active stage 中的 failed 状态转为任务失败。NAV 此时仍 idle，无 request_id、body odom、SCAN 轨迹或区域收据；控制器自己的运行期安全传播尚未参与这次失败。SCAN 停在 INIT/no odom，FAST 静止初始化因为实际角运动重置，尚未输出 body pose。因此这次不能归因于区域/航点 liveness、导航方向、SLAM 到点精度或控制增益。

## 真正执行输入与证据缺口

5646 条 requested/actual CHAMP 命令全部六分量精确零；6761 条带位置的 raw、filtered、nominal 记录全部关节位置逐值相同。适配器没有 failure 条目，最终仍 idle。本次没有目标改变或移动参考延迟问题。

但相同 nominal **目标**不等于实际关节状态不动，也不证明下游 JTC 接收无空窗。Adapter 日志发生在 publish 返回后；缺口可能跨在某次 `rcl_publish()`、执行器调度/等待或同步诊断写入中。目前没有 native publish enter/return、线程调度、write syscall、JTC 实收时刻证据，不能单独认定 DDS、CPU、磁盘或 CM 是根因。日志空窗邻接位置约 2.966 MiB，并非简单的精确整数 MiB 边界，也不能凭缓冲大小认定 flush 阻塞。

下一最小观测应在原控制/安全不变的条件下，对 Adapter 实际 actuator Twist、filtered JointTrajectory、joint_stop_safety String 三种发布做有界内存 native entry/return 记录，严格校验 topic/type/handle；Bridge 回调入口另记原始消息 sim/counters 和单调时间。若一条 publish 持续覆盖空窗，才隔离其传输边界；若没有，则再看调度或诊断写入。观察器不得刷新或绕开 watchdog，也不扩大 `.30/.40/.25` 门限。

可复现只读分析：`python3 multifloor_demo/navigation/test_results/full17_startup_status_audit/audit.py`。`result.json` 保存输入 SHA、冻结来源匹配、原生数据和声明边界。首版分析器误把 manifest 的 `sha256` 字典当顶层导致 source-match 假值，原分析结果保留为 `first_analysis_manifest_lookup_error.json`；纠正读取字段后全部 8 个源匹配，无生产文件变动。
