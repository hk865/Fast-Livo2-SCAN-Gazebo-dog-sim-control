# CHAMP V4 实际力矩来源与采样审计

两轮物理前力矩记录有效：同一 Gazebo iteration、simTime、dt 的 PreUpdate 缓存与 PostUpdate 状态完整配对。实际物理输入具有大量非零值，物理后组件则被清零；旧 dd50 的全零观测缺陷没有被沿用。这里记录的是 **command_feed_to_physics，不是电机实测力矩**。本审计未修改运行源或原验收结果，未启动 ROS/Gazebo 或发送信号。

| 原运行 | b0ab，r1 | 5e9f，r2 |
|---|---:|---:|
| 实际 physics_step 行 | 37601 | 37605 |
| 完整物理前力矩配对行 | 37596 | 37600 |
| 物理前非零行 | 37569 | 37573 |
| 物理后全零组件行（仅诊断） | 37596 | 37600 |
| 最大物理前绝对力矩 Nm | 23.5 | 23.5 |
| 最大实际关节速度 rad/s | 30.0000000068732 | 27.23081070131392 |
| 原固定停车窗 | 83.075–88.075s | 48.075–53.075s |
| 真实 reader 行数 / 原门最低数 | 234 / 248 | 249 / 248 |
| 原停车结果 | unverified | passed |
| 原整轮结果 | failed | failed |

两轮只有最初 .005–.025s 的 5 帧力矩 unavailable，原日志保存 `null`，没有用零伪装。自 .03s 起完整，首非零在 .165s。actual world 的 observer priority 为 1，原 CM 默认 priority 为 0；原 contract、当前源、实际复制 artifacts SHA 全部匹配。ownership 仅一个 gz_ros2_control 执行器，没有 Teacher actor 或 body servo。四足原 contact 均包含 XYZ、normal 与 raw wrench，body contact 为 0；raw wrench 不被解释为已校准垂直载荷。

原 CM throttled 日志独立记录了 r1 52 条、r2 26 条限幅事件，日志中请求力矩最大分别 30.844341、57.247725Nm，limited 输出最大均 23.5Nm。日志墙时没有改名为模拟时间，也不作为全部饱和次数；完整 200Hz 物理前记录支持输出守住上限。r1 关节速度超出数值 30 的量为约 6.9e−9，低于原 runtime 容差 1e−6。两轮都未触发实际超限、native latch 或 reader fault。因此 **实际超限后的停车反应未被本轮触发验证**，离线 guard/cache 检查仍是独立证据。

r1 原固定停车窗的 233 个执行间隔为 170×20ms、63×25ms；没有结束截尾，reader 日志一直到 188s。234 条已录行都为 requested/command 零、health ready、无 fault。另有 1001 条连续 200Hz 物理记录，平移漂移 .0034072m、偏航 .0016220rad、世界 vx/vy RMS .0112308/.0036251m/s、wz RMS .0031624rad/s。物理诊断满足相应数值门，**不能取代原 50Hz 执行记录覆盖门，r1 停车仍 unverified**。r2 原窗 249 行、244×20ms+4×25ms、1001 native 行，平移漂移 .0013795m；原 passed 保留。

整数时钟审计：r1 的 9207 条 reader clock、29829 个独立 shadow clock，以及 r2 的 9279/29696 条都为 5ms 整倍数，没有观测到 1ns 阈值边界误差。r1 的 730 个 25ms reader 间隔中，独立 shadow 有 561 个前一 reader+20ms 精确时钟候选；这证明另一接收者有该消息，不能证明 reader 实际收到。5ms 墙时 timer、callback 调度、时钟合并或 graph/file I/O 延迟仍是候选，缺乏 reader callback 收包/dispatch 记录，不能唯一归因。未来可事前冻结真实 callback 和实际 tick 时序诊断；不能回填旧帧、伪造时戳或放宽本轮门。

r1 原失败项包括 pitch .72273rad 与路线横漂 .65020m/RMS .27788m/行走 heading .51989rad。r2 原路线仍因 heading .44905rad 未通过。有效力矩来源及限幅不改变这些失败，也不赋予导航整包通过。

各运行目录下保存独立 `receipt.json`、`parking_timeline_diagnostic.json`、`cadence_integer_diagnostic.json`，均附原日志与审计源码 SHA。共用脚本位于本目录，`offline_readiness.json` 记录 15 项无 ROS 离线检查。冻结 CHAMP V4 与所有原摘要/原日志未改，报告写入已经结束。
