# V20 空间参考、走廊观测与轨迹选择方案

2026-10-06。本轮实际运行 `20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea` 到达原前九区中的 **7 个区域**，随后因显存余量保护中止。结果为 `not_completed_resource_abort`；九区任务、末尾固定五秒停车、完整46区域均未通过本轮验收。没有进入第九区，不能据此判断已经解决 V19 的第九区反复转向问题。

用户要求其他任务运行期间先整理轨迹选择修改方案并保存独立分支。因此本轮已停止仿真，不再编译或启动资源密集测试，不停止其他训练或模型服务。源码上传目标为 `hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control` 的 `codex/v20-spatial-corridor-shadow`，基线 main 为 `5237b8865d1f788b88f86fe21b72852db3bedb03`。该分支是候选代码与证据检查点，不是导航通过版本。

## 已实现与尚未实现

独立 `navigation/corridor_tracking_v20/` 已实现默认 0.2 m 有限弧长方向，统一外层朝向门与内层 PD/PI 的方向参考；换路使用实际三维 SLAM 位姿重投影，保留 PI、滤波及执行命令。曲率前馈和转向变化率限速有独立开关，但首轮实际运动关闭。原区域几何、0.4 s dwell、90 s 期限、300 ms 输入与命令保护、唯一执行器和主动停车保留。

隔离 SCAN 只读导出真实 raycast 三态体素和实测支撑点；走廊模块检查扫掠体积、未知空间、支撑覆盖及停止前缀。当前仅 **shadow 观测**，代码拒绝 `retain_valid_plan=true`，没有主动走廊控制，也没有用走廊结果压制 SCAN 重规划。Teacher 仍为冻结 CPU 单线程模型；导航输入为真实 SLAM/IMU/点云，Actor 仍保留232维特权输入、15维已知命令与上一动作。

[下一步轨迹选择修改方案](TRAJECTORY_SELECTION_PLAN.md) 是尚未实现的设计：A 候选准备/元数据原因/原子提交；B 旧路径保留与切换滞回；C 走廊局部计算与测量模型优化。先做有限测试与同九区对照，通过后才扩大46区域。它不修改 PID 增益或放宽原输入保护。

## 验证结果

| 范围 | 结果与限制 |
|---|---|
| 数学、控制接线、走廊与worker有限检查 | 81项通过；启动接线12项通过。不是实际导航验收。 |
| 旧104次换路静态筛查 | 0.2 m参考超过0.2 rad的事件3/104，回退0；只验证固定旧输入，不预测新闭环门复位数。 |
| 原九区实际运行 | 原区域0–6的7个回执独立核验几何、SLAM、0.4 s dwell及最大间隔通过；剩余区域与末尾停车未完成。 |
| 执行与姿态采样 | 8463个原生50 Hz快照，Actor fault=0、机身接触=0、采样安全违反=0，最大间隔20 ms；不覆盖所有200 Hz物理步。 |
| 运行完整性 | 显存保护中止；最终worker结果缺失，clean cleanup标志false保留。审计时本轮已记录的PID/start_ticks均无同一存活进程，不改写为正常完整退出。 |
| 走廊离线复验 | 400/400输入/哈希/路径绑定通过，冻结纯函数结果400/400完全一致；0 certified、241 unavailable、159 blocked。blocked不等于实际碰撞。 |
| 走廊计算时效 | wall中位378.664 ms，49/400返回时活动路径已换；返回时cloud源龄372/400超过300 ms（中位390 ms），pose源龄348/400超过300 ms（中位360 ms）。这些源龄来自原始source clock，非wall时间换算。 |
| 页面 | 28项有限测试通过；真实Gazebo归档画面、实际SLAM/SCAN路线、只读走廊诊断可显示。历史V18通过范围与当前V20未完成分别显示。 |

走廊输入cloud源龄最大205 ms，但结果到达时大多已过期。严格逐free-cell新鲜度、未知空间和支撑覆盖也是拒绝来源；不能通过把unknown当free、刷新缓存时间或随意放宽300 ms来制造通过。发现2个不完整快照缺少provenance字段，使诊断理由显示为`invalid_map_provenance`，仍为拒绝，未错误放行。后续候选应修正诊断而保留本轮冻结原件。

## 证据与复现边界

- [原九区独立报告](evaluation/V20_39EA_PREFIX9_ACTUAL_EVALUATION.json)、[评估器13项有限测试收据](evaluation/PREFIX9_EVALUATOR_FINITE_RECEIPT.json)。
- [走廊完整复验](evaluation/20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea_CORRIDOR_AUDIT.json)、[返回源龄](evaluation/V20_39EA_CORRIDOR_RECEIPT_CLOCKS.json)、[报告哈希](evaluation/V20_39EA_CORRIDOR_REPORT_HASHES.json)。
- [81项收据](root_checks/FINAL_FINITE81_RECEIPT.json)、[源码审查](root_checks/CONTROL_REFERENCE_V20_REVIEW.json)、[静态104事件](root_checks/V20_STATIC_104_EVENTS.json)、[预先判据](PROSPECTIVE_CRITERIA.json)。
- [页面说明](viewer/README.md)。浏览器归档：`http://127.0.0.1:8768/?run=20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea`，不是正在运行的仿真。

原机raw留在 `/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/` 对应run目录。本分支只导出源码、精简报告及小型元数据，不导出模型、build/install、点云、整段JSONL或原生二进制记录。没有这些raw，不能在clone重新独立验收本轮轨迹。保留的哈希只识别证据，不替代证据本体。

已冻结模型SHA256为 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`；运行manifest SHA256为 `3900ac31efb149747b939b2d9caaf138aafe92addca0c064d0c3079ce26c0031`。源码与配置逐文件身份见运行source manifest、V20 gate和导出清单。原机gate含绝对路径与二进制哈希，在clone中只作历史证据，不授权新主机运行；V20便携运行入口尚未适配。

原机已有实际命令（仅记录，不代表当前应重跑）：

```bash
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v20/run.py \
  --profile exporter_shadow_original46_prefix9 --label v20_exporter_prefix9_r1 \
  --domain 89 \
  --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs
```

本轮不改变历史接口/基础运动通过结论；新V20运动闭环仅部分区间有证据，九区导航未完成，完整46区及新闭环Sim2Sim未通过。三层连接是坡道，真机未验证。
