# 最终保护与来源审计

2026-10-04北京时间09:16:31完成只读核对，11组检查全部通过。仅在本新目录写入审计脚本与结果，未改package、原README、docs、运行源或原始日志，未启动ROS/Kit/仿真或发送信号。主收据`audit.json` SHA256：`35d1e142483f64bb3d777a2f3d4fd42345ba864f4c6c095ca01ead2426ec80b3`。

冻结checkpoint仍为`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。训练归档env/agent分别为`b48ccebc1ef9ac7aa4d31451c720d38af040997fbc0c84c096cdd12b281e5d79`、`a5dbae28edf797729166044b2d3fda0afcc8812726295e28dfb580a39e862ec3`，均与初始封存一致。

原camera_mode两资产camera_rig/three_floors_camera分别为`ddd00f818081eef6746224b7ab96db97edb99f53152e47d71e7bb8e45ba65e33`、`7e5c74c27c0fb7dda94ab624e521c83154463f861542f096d40b3f8545d4ccbd`，匹配原asset_receipt；原Demo的go2_converted/three_floors也匹配既有保护基线。当前native.so为`3a0ca63fb63c9e20c07639e2faf4d7509fba29a2bd5e987b8875dd50534a2ec2`，与v4.1冻结基线一致。早期pre-step包native不同，这里不将后续已封存的接触证据日志扩展称为“从最初完全未变”。

全局acceptance仍为`0250c047a09ada1b29ddaaa6f7251b3fbb57ad091f75f9ff0273f48ef08ba76c`；其接口passed、运动failed、Sim2Sim failed、导航unverified、真机unverified均未改写。

08点后三次独立来源审计的全部原证据哈希再次匹配；本次未重复CPU推理，引用已完成且源/原始数组现仍匹配的CPU1回放：

| run尾号 | 实际前向速度m/s | 前向位移m | 停车漂移m | 动作回放最大误差 |
|---|---:|---:|---:|---:|
| d6cd | 0.243591 | 1.907742 | 0.000623 | 1.55×10⁻⁶ |
| dbb4 | 0.242763 | 1.891226 | 0.000397 | 1.25×10⁻⁶ |
| a843 | 0.245712 | 1.916545 | 0.000673 | 1.55×10⁻⁶ |

各为18秒/901策略帧，其中t≥3的751帧替换30维：真实`/livox/imu`角速度/重力和按name映射的`/demo/control/measured_joint_states` q/qd。247维重建误差0；其余217维明确分为190特权维（COM速度3＋高度187）和27控制器已知维。真实header≤native worldt−0.005、sim age≤25ms、wall age≤300ms保持不变；三个run全部五个自有进程正常退出，没有来源fault或真值回退。原运动/停车阈值未放宽。

旧080e在18秒的joint来源失败仍失败：sim age5ms虽有效，回调墙钟间隔约0.427秒，旧joint墙龄下界0.426475秒超300ms。原stamp严格递增、invalid为0，因此不是重复stamp；当时精确拒绝payload未封存，不能唯一归因GPU或某一IPC/调度阶段。原loaded v2组是2/3，后续clean idle组三次均通过，两组分开保留。fe74完成18秒运动后runner收到SIGTERM143并在清理时中断，原runtime_manifest缺失；发送者未确定，root仅清理精确自有orphan，不能计为完整runtime pass。

no-fsync候选与离线已测源码一致：CommandFile AST相对旧封存恰删除一次fsync，LatestWriter字节及实际consumer AST保持一致。checkpoint、native、观测/动作契约和全部v4.1冻结runtime SHA匹配；dba1与旧24d9的physics XML以及去除visual/sensor/runtime plugin后的真实机器人collision/inertial/joint/pose XML一致。wall/sim双300ms与原始源戳仍保持；不承诺掉电持久性、实际延迟必然消失或全面运动/Sim2Sim通过。

快照GPU1019/16303MiB、利用率13%，CPU load3.82/3.27/2.63；保存21个相关进程的PID/starttime/argv/状态，其中没有训练候选进程。该瞬时“未发现”不代表本审计停止过历史训练。快照实际运行者已是root的动态fixture run6614；本审计没有控制它。

收据中`no_fsync_scope_review.active_run_status`仍保留从任务起点带入的“dba1 live”描述：此字段是阶段标签错误。dba1实际在快照前已结束并有runtime_manifest，快照进程argv明确为新的6614动态run；dba1仅用于冻结的物理配置比较，未作为当前进程依据。本报告明确纠正标签，不改写原审计收据或任何run的原证据，也不自行授予dba1/dynamic导航pass。
