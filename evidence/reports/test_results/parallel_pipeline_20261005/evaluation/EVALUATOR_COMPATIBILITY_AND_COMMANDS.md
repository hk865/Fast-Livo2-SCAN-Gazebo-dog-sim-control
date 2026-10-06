# 独立验收复用范围与操作命令

事前计划为 `PROSPECTIVE_EVALUATION_PLAN.json`，SHA256 `635dd1e8313fda490cbbda3cda5882d5fa7e18a4621e7372d9368d26265d3848`。V17 首轮单接收/解码 executor 补充为 `PROSPECTIVE_V17_SINGLE_EXECUTOR_AMENDMENT.json`，SHA256 `0902dde6d3a3b1657e7a942957e7bd93424cd8cc7aaee83ce16d8e0000ac72dc`。补充只收敛并发边界，不更改任何数值门。

## 冻结对照

V12 实际前缀：`/var/tmp/go2_teacher_simulation_20261005/20261005_205309_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_timing_r1_0b8d`，210 s，25/32 区域，kind300 启用，详细记录真实 115–118 s。它不是完整导航通过。

V12 实际全路线：`/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb`，600 s 为上限，实际 275.18 s；32/32、两条完整 12 m 坡道、固定首次 5 s 停车的来源闭合联合审计通过。原 common/v2/ramp 的 UNVERIFIED 和独立 metadata-v2 修正结论各自保留。kind300 未启用，阶段耗时是 N/A。

V12 核心库 SHA256 为 `8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed`。新候选应证明其自己的实际加载库、源码和编译参数；不能把这个基线哈希复制成新候选的运行证明。

## 读取器适用范围

以下文件只读复用自 `test_results/lidar_density_rate_20261005/evaluation/`，其实际字节已绑定到事前计划。旧封存 run 不再次执行会追加收据的命令。

| 读取器 | 新 V15/V16 适用条件与限制 |
|---|---|
| `evaluate_completed_run.py` | worker 完成、diagnostic writer final、owned cleanup 成立后，从实际新 run 的 source snapshot 运行原 common evaluator；它会拒绝覆盖不同 canonical。来源哈希和实际加载库仍必须是新 run 自身的证据。 |
| `publication_ledger_v11/audit_publication_ledger.py` | 756ad 事前 criterion、7 个 producer/math/helper 字节与 V12 一致；源路径可按 actual snapshot 改变。原 common 19 门保留，显式唯一 PI bookkeeping 替换及 7 个追加门形成 26 门；不允许将缺少 publication/hold 数据当作零。 |
| `sources/acceptance/evaluate_ramp.py` | 调用新 run 自己的冻结 helper；完整 25 门保留。210 s 前缀可以有单段结果，但不等于完整 32 区域/两坡道/停车。plane ID 不跨 run 作为同一实体。 |
| `audit_terrain_metadata.py` 和 `audit_terrain_metadata_v2.py` | v1 是历史 metadata join 修正；v2 进一步封闭三份祖先的全部 input/source binding 和 exact 19/26/25 key sets。只在原其它必需门通过的全路线做联合审计；prefix 的缺失路线/停车门不能绕过。固定只移除生产 history 新增的 2 个 metadata 字段，全部真实 payload 必须精确唯一匹配。 |
| `sample_owned_cpu_v12.py` | root PID 为真实 runner、采样从启动立刻开始；/proc starttime 防 PID 复用。只测 runner 及其真实 descendants，每 2 s 一次，另记录自身开销。viewer、训练、浏览器等外部负载不计 owned；生命周期 `%CPU` 不能代替这段 CPU delta。 |
| `performance_proc_v12.py` | kind300 的 17 个 scope/90-double 布局未变且实际启用；回收完成的真实 wall windows。嵌套 stage 不直接相加，process CPU 不是函数独占 CPU，publication stage 不代表异步 DDS 全部成本。候选追加计时需要另立 schema，不复用旧 offset。 |
| `performance_proc_full.py` | full 未启用 kind300 时明确 N/A；只用真实 /proc 观察区间，不将迟启动进程之前的 CPU 假设为零。保留 sampler 外层退出码独立事实。 |
| `analyze_sensor_experiment.py` | kind1/2/3/12/13/100/101/103 原布局、native schema 未变。全时 actual 输入与 processed output Hz 分开；kind100/201 只有详细窗，缺失不能当作匹配数量零。高度用实际 SLAM/native 离线对照，时间窗相同不等于机器人状态完全相同。 |
| `audit_pipeline_ages.py` | V15/V16 callback/source receipt 语义不变可复用。V17 必须另外记录 receive、decode、enqueue、dequeue、owner commit 的单调时钟与源序号；旧 callback receipt 仍是原入口 receipt，不能刷新成 dequeue。旧函数未观察的段不能统称 communication。 |

候选须事前冻结 top-level profile/scope、C++、库、编译参数、runtime env 和纯测试收据。common/v2 不替候选验证并行数值：另用同输入生产组件结果、真实 omp team/worker 分区和原数值字节对照证明。全局库中出现 4 个线程，或者编译包含 OpenMP，都不是这个内核实际使用 4 线程的证据。

## 新实际 run 完成后的复用顺序

这些命令是准备给 root 的操作说明，未在本阶段启动物理，也未对封存 run 重新执行。`candidate_run` 必须由 root 填为新实际 run 的 canonical realpath，`owned_root_pid` 为其真实 runner；推荐 root 自行创建 private 0700 `/var/tmp/go2_teacher_parallel_20261005`。下列变量不覆盖 HOME/CODEX_HOME。

```bash
eval_old=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/evaluation
eval_new=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/evaluation
# root 填入新 run 和 runner PID；不要填旧封存 run。
candidate_run=/var/tmp/go2_teacher_parallel_20261005/NEW_ACTUAL_RUN
owned_root_pid=ACTUAL_RUNNER_PID
```

runner 启动后的同一次工具编排立刻独立开启只读采样；不等模拟推进后再启动：

```bash
python3 "$eval_old/sample_owned_cpu_v12.py" --run "$candidate_run" --root-pid "$owned_root_pid" --hz 0.5 --max-wall-s 1800
```

确认 runtime/worker/logger 全部结束后，串行运行收据和诊断，不与新的实际试验竞争大日志 I/O：

```bash
python3 "$eval_old/evaluate_completed_run.py" --run "$candidate_run" --suffix parallel_candidate_r1 --output "$eval_new/NEW_ACTUAL_RUN"
python3 "$eval_old/publication_ledger_v11/audit_publication_ledger.py" --run "$candidate_run"
python3 "$candidate_run/sources/acceptance/evaluate_ramp.py" --run "$candidate_run" --receipt-suffix parallel_candidate_r1
python3 "$eval_old/analyze_sensor_experiment.py" --run "$candidate_run" --output "$eval_new/NEW_ACTUAL_RUN/sensor"
python3 "$eval_old/audit_pipeline_ages.py" --run "$candidate_run" --output "$eval_new/NEW_ACTUAL_RUN/pipeline_ages.json"
```

原 ramp helper 默认写带后缀收据；若 browser/联合审计需要 canonical，只能首次将其确切字节复制为 `summary_closed_loop_ramp_independent.json`，已有文件不同则拒绝，不重算门也不覆盖。

210 s 且 kind300 实际启用时：

```bash
python3 "$eval_old/performance_proc_v12.py" --run "$candidate_run" --output "$eval_new/NEW_ACTUAL_RUN/performance.json"
```

完整路线且 kind300 实际关闭时：

```bash
python3 "$eval_old/performance_proc_full.py" --run "$candidate_run" --output "$eval_new/NEW_ACTUAL_RUN/performance.json"
```

全路线全部其它门通过、原 metadata 归档比较造成的 UNVERIFIED 与来源确认后才追加联合修正：

```bash
python3 "$eval_old/audit_terrain_metadata.py" --run "$candidate_run"
python3 "$eval_old/audit_terrain_metadata_v2.py" --run "$candidate_run"
```

外层 sampler 退出状态、completion JSON、runtime exit、各 writer expected/written count 分别保存；不能把外层 SIGTERM 原因未经证实称为正常，也不能把它冒充物理跌倒。全部新增 writer 结束后才可全 hash/seal。

## 存储与试验期间负载

当前报告可用空间约 56 GiB，root 只选择纯测试有收益且无数值退化的候选做 210 s，再择优完整一次。每次启动前保留至少 15 GiB 余量，详细日志窗仍 115–118 s，不删除原导航点云、原始 IMU/反馈/动作/接触或历史失败。实际期间只有轻量 2 s sampler 与小文件/index 尾部读取；大扫描、哈希、图形和微基准在实际退出后执行。纯内核收益不能替代 actual Hz/source age/native/路线/停车验收。
