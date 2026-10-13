# R83：完整导航及已批准的首次实际零命令停车验收

R83 完整验收 **PASS**。判定对象是实际运行 `20261013T185800Z_closed_loop_R83_parking_hysteresis_57e7`，候选为 `engineering_v63_r83_parking_hysteresis_candidate`。结果来自运行后原谓词回放、原始记录身份核验及四项独立复核；未追加仿真。`EVALUATION_FINAL.json` 是最终判定，初次等待独立复核的 `EVALUATION.json` 保留。

## 结果及口径

| 项目 | 实际结果 |
|---|---|
| 原完整顺序 | 46/46；40 普通区域、6 原空间停留区域；探索、返回、导航三阶段原完成谓词通过 |
| 初始化、RGB、地形 | 原初始化/RGB 谓词通过；三次完整 187-ray ACK 逐值差 0，源哈希相符 |
| 动态障碍恢复 | 原实测停车 300ms、新 SCAN/连续清障及恢复原谓词通过 |
| 转向 settle 放行 | 26 次原连续 300ms；新 1s 统计门使用 0 次 |
| 末端切换 | 减速进入 1 次、退回 capture 0 次；对照 R82 为 144/143 |
| 精确捕获资格 | 原 XY≤25mm、yaw≤0.035rad、平面速度<0.03m/s、yawdot<0.06rad/s 条件持续 630ms；原 600ms 代码未改 |
| 首次实际零命令 | producer 66010；Actor 输入 889.310s，next-native ACK 889.330s，首次资格 T0=889.370s |
| 正式固定首 5s | 889.370–894.370s，不重置；原停车验收谓词通过 |
| 正式 5s 指标 | native 平面速度最大 0.0337533m/s、body wz 最大 0.0082281rad/s、native yaw 差分最大 0.00795744rad/s；SLAM 平面/yaw 漂移 0.0202403m/0.0160847rad |
| Actor、原生归档 | 全部 45,112 帧双数组 CRC 和文件 SHA 独立通过；正式窗口的 251 个输入命令观测槽精确零，previous_action 保留前帧实际 action；未替换策略 action |
| 关闭 | 469,690 包全部提交，排空后关闭；无 TERM/KILL，worker/capture/Gazebo 自然退出，无 owned 仿真残留 |
| 冻结输入 | 880 项无变化；准备阶段 29 依赖引用及副本闭合 |
| POSTROUND 容量 | 当前保留 95,559,610,368B，剩余 34,440,389,632B；本轮保守计费 4,268,154,880B；交付完成后再观察最终容量 |

6 个 hard 区域沿用原空间/控制区域停留定义，不新增“物理完全停止”结论。正式 5s 内 native 实际平面漂移约 1.745cm，属于冻结的 5cm 合同内；通过不表示身体完全不动。资格阶段的 0.03m/s 与正式窗口的 0.08m/s 上界是不同原判据。

**旧口径仍 FAIL**：原 25cm 到达时间 882.540s 的首 5s（882.540–887.540s）最大 native 平面速度 0.182724m/s、wz 0.126595rad/s，250 帧 Actor 命令非零。新结果仅按已明确批准的 `ordinary_zero_ack_fixed5` 合同通过；不重标旧到达窗口、R79、R80 或 R82。

## 最小改动与因果边界

R83 的唯一算法改动为 `ordinary_parking.py`：减速进入 XY≤15mm，保持到 XY≤20mm，yaw 条件仍 0.035rad。原 25mm 捕获资格及 600ms 计时不依赖先进入减速，捕获期间满足资格可继续累计。21 个保护模块保持 R82 字节，PI 增益、控制律、Actor/previous_action/native PD、原安全锁、定位/路径接纳及动态恢复条件未修改。

真实 R83 只进入内带一次，无反复 reset；这与 R82 的切换抖动形成对照。但两次独立物理运行不能替代同轨迹反事实实验；不能归因给未使用的新统计门，也不推广为任意地图或 terrain 的稳定性证明。

本轮新跑了 3 个业务 CPU 工作流。76 个既有 mission 测试结果按不变源哈希复用，并非本轮新执行；旧核心 17 项仍为 15 PASS/2 FAIL，与 C79 原失败一致，单独保留，不能宣称全部单测通过。

## 评估绑定修正

第一次 post-run evaluator 错把 stage 写成 R82，实际读取了 R82；对现有 R82 `EVALUATION.json` 的 exclusive-create 抛出 FileExistsError，未覆盖旧证据，也未生成 R83 判定。旧脚本保存为 `evaluate_r80.pre_binding_correction.py`。

只修正非冻结的运行后评估脚本，使 stage 从自身路径解析，并断言 R83 run 名称。修正后回放真实 R83 输入，并逐字节比较原 canonical 证据，不向 run 写入。修正前 SHA `79c2b919694bc58d2fb5da4d1d4c71f03495aeeba1977926c427390c12acc17f`；修正后 SHA `5c52e47e045f7bcd0eab8b47deb2a4b50a502849da9c6f16cb728105942bc457`。回执见 `EVALUATOR_BINDING_CORRECTION.json`。

## 下一步决策

先交付已验证的 R83 快照、新分支和模块数据流文档，本轮不扩展运行源码、不为成功结果重跑。旧附件分析的是 R44；当前已有 bridge 15 类稳定 reason code、部分同语义常量真源、显式定位输入端口、frame-local 执行授权及独立停车 owner。

后续优先做行为等价的 RunContext：prepare/owner/evaluator/closure 从同一声明解析 run/stage/candidate/profile/freeze，并交叉核验，消除已发生的跨轮复制错绑。随后补全 reason/source/恢复条件可观测性，逐项拆 WorkerHooks 与证据 IO；保留现有 ENFORCE 和物理锁。不能根据旧文档把定位新鲜度、轨迹接纳、时钟、命令来源改为只上报，也不能把同为 300ms 的源龄、连续停车和反馈缺口合并为一个无语义常量。

可并行只读审计绑定、门禁因果链、适配与归档；源码变更和实验由唯一集成人串行。行为等价重构的准入是固定输入下命令/状态/积分与 reset/授权和 ACK 时序逐值一致，错轮/陈旧/乱序/缺失输入仍拒绝。通过 CPU 回放并冻结新快照后，下一轮才做单因素完整 46 区仿真，仍同时报告两种停车口径。

## 限制

这是给定 43-box 地图辅助、特定冻结场景中的一次完整仿真 PASS。局部搜索、未校准 covariance、`require_complete_imu=false`、正式 R6 采样未完成的限制保留；不宣称独立传感器全局定位、真实机械 LiDAR 或实机通过。未开展 DSH、训练或实机运动。

独立回执：`INDEPENDENT_FINAL_NAVIGATION_REVIEW.json`、`INDEPENDENT_TERMINAL_ARCHIVE_REVIEW.json`、`INDEPENDENT_FINAL_LOCALIZATION_REVIEW.json`、`INDEPENDENT_FINAL_RESOURCE_REVIEW.json`。原运行和全部失败证据保留。
