# V6 图像协方差对照：高度来源失败附录

V6 `9cc6` 完整运行 600s、30001 个 Teacher 帧，worker fault=null，五个主进程及 11 个 launch 子进程退出全部为 0。导航只完成 16/32 个原 SLAM 三维区域；正式 common、完整两坡 ramp 收据仍为 FAILED，heading 来源收据 PASS 不升级运动或导航结论。

生成配置的最小 A/B 是 V5→V6 `vio.img_point_cov: 100000000→1000`；两轮生成 `navigation_fastlivo.yaml` 的完整解析参数树仅这一叶节点不同，在线 `gravity_est_en=true`、`ba_bg_est_en=true` 保持。原始 `sources/external/*fastlivo.yaml` 留作旧配置来源，不是本轮有效生成参数；它仍有原默认开关和 1e8 数值，不应混用。

| 对照 | V5 b182 | V6 9cc6 |
|---|---|---|
| 首次导航失败 | connector_y 90s 截止约 220.38s，首 failed 历史观察 /clock 220.49s，使用原 SLAM 头 220.3s | connector_y 刚激活后约 3.3s；首 failed 历史观察 /clock 131.74s，使用原 SLAM 头 131.699999999s |
| 本轮冻结目标 Z | 1.2056299220538802m | 1.2054456319942866m |
| 失败所引用原 SLAM Z | 0.9830298072292131m，目标高度差 −0.22260011482466713m，未满足 ±0.1m 到达范围 | 0.8989408802532117m；固定路线管道高度差 0.30650475174107483m > 原 0.3m 围栏 |
| 实际达到区域 | 16/32 | 16/32 |
| 更晚的执行器事件 | 429.33s native socket 故障，故障阻尼后出现机身接触；详见原 [V5 时间附录](B182_FAULT_TIMELINE_APPENDIX.md) | 全 600s fault=0、body contact=0；没有同类 socket 故障 |

V6 首 failed 原目标为 `[14.986919731736078, 4.414734855320622, 1.2054456319942866]`，原源测量为 `[14.878118163195165, 2.1843743983558075, 0.8989408802532117]`。此时尚在 connector_y 航段早期；管道投影点为 `[14.896717147414897, 2.1836224521786423, 1.2054456319942866]`，水平偏差仅 0.0186141784m < 原 0.45m，垂直偏差超过 0.3m。原 bridge reason 为 `Measured SLAM body pose left finite route fence; restart after review`，健康置 false、输出命令归零并锁存保护。它是原高度围栏保护，不能写成 90s 超时、XY 到达或最终停车完成。

原始来源与物理变化分开比较：

| 原始源时间 | 原 SLAM camera_init body Z | 离线最近 200Hz native 有效时间 | native base origin world Z | 原生接触/故障 |
|---|---:|---:|---:|---|
| 128.400000000s，connector_mid 完成 | 1.2147991533m | 128.400000000s | 1.5050841427m | body 0，四足均 1，fault 0 |
| 131.599999999s | 0.9084262012m | 131.600000000s | 1.5046418728m | body 0，四足均 1，fault 0 |
| 131.699999999s，首失败所引用源 | 0.8989408803m | 131.700000000s | 1.5048165104m | body 0，四足均 1，fault 0 |

从 128.4s 至 131.7s，原 SLAM Z 降低 0.3158582730m，而 native world Z 只降低 0.0002676322m（约 0.27mm）。两列属于各自坐标系，不能直接比较绝对零点；这里分别列时间配对后的变化和独立物理状态。native 为 `t-dt` 相位（dt=0.005s），原源头保留整数纳秒，最近采样不插值。native 仅离线检查执行，没有回写或修正导航输入。这些证据显示本次冻结路线坐标中的观测高度变化与实际机身下坠不同，不应归称为策略跌倒。

正式 native 全 200Hz 安全门 PASS：最低净空 0.2756869149m，最大 roll/pitch 0.1139752471rad，body-contact 样本 0，fault 样本 0。下层 ramp12 完整通过：COM 入坡 12.835s→退出 98.535s，全部 12×1m 真实脚支撑米段及四脚有序入口/坡/出口支撑通过，最大横偏 0.03720079285m，原 lower_exit 源区域 dwell 100.9..101.5s 并对应实际 floor2。上层 ramp23 未完成，目的地首次 active_hold 固定 5s 窗未验证；后续安全站立到 600s 不替代最终到达停车。

本轮把图像协方差降低后仍出现冻结地图路线中的高度错误，并更早触发垂直围栏；因此该单轮参数调整没有解决此次高度问题。这是最小配置差异的两轮实际观测，原传感数据与锚点各自独立，并非重放同一消息流；不能据此断言图像信息无效、权重永远无用或确定唯一估计根因。原高度/到达/时限/安全判据均未改变，Actor 仍为同冻结 CPU Teacher、232/247 维特权输入，完整三层/SCAN 多层成功与真机部署没有通过。

来源与不可覆盖的原件：

- [V6 common FAILED](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/summary_closed_loop_cascade_independent.json)，SHA `b875631e4fb33701429623e8da072960ce9861b1e873839dc43aaf2239d38583`。
- [V6 ramp FAILED](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/summary_closed_loop_ramp_independent.json)，SHA `8384975078c81d3f202fcdf9f580c1f144939b2caddeb345b5c128ff1426c2d6`。
- [V6 heading 来源 PASS](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/summary_heading_source_independent.json)，SHA `ab4e5fc106dad5c143e6e7174d9fc188e7752b502181d7113ca650985b4a1f04`；实际 76 条 align locked 来源最大差 9.4189e−12rad，无缺失。
- [V6 有效生成参数](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/navigation_fastlivo.yaml)，SHA `10258fa98e46c2ccd622a8791ec41e5e6fe8a31eb6eea2287c6336e2745791ee`；[V5 有效生成参数](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182/navigation_fastlivo.yaml)，SHA `4e722663cdb1ff7d630711e0ffdef8c89f730684a5d107ec1bbe19428a67b39c`。
- [V6 原 SLAM 消息](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/navigation_slam_poses.jsonl)，原 common 验证 SHA `84491d5f3e23a88b885507860a7bc0030a97a6491573bbff0787f4482fe8a10f`。
- [V6 原状态历史](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/navigation_status.jsonl)，原 common 验证 SHA `e687a9596e5bb1c4d42965000d029686d701da95a6ad27461b08706a4437ec58`。
- [V6 原 native 物理记录](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/actuator.jsonl)，原 common 验证 SHA `a78908239aa804229625d852c3cf125d58e0308f8278e560817f3c31d52cac15`。
- [V6 原 worker 结果](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6/worker_result.json)，原 common 验证 SHA `41fee517b0d070bc1c5f18cf2b3078ea0c4d086b9e23fd3b11a04afa8b0f2899`。

本新附录只增加解释，没有修改原日志、冻结源码、合同、参数、判据或旧收据。原 provider 来源 gate 的 observer 包装差异仍在原 ramp 收据中保留 UNVERIFIED，不在此附录升级。
