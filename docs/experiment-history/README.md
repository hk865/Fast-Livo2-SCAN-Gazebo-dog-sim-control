# 实验历史与留存证据索引（2026-10-25）

**当前结论：formal22选定策略下R85单次F1→F3完整任务PASS；original46仍false。** 新质量/terrain语义不等价旧保证，未把一次成功当间歇根治。本历史包在文档分支发布；整理和发布没有删除既有数据。

本地既有资源快照总留存120.278765568GB；本次135个run目录元数据计78.098878464GB。76项重点记录，另列软件/未启动与前史载体。源数据仍在/已知删除/无法确认完整重放分列。

本阶段追加：**整理目录ecb62ad实际到达全路线，但严格固定5秒停车FAIL；动态/RGB终验未完成，原成功不转授。** [逐标签准备/实跑与故障改进](chapters/09_整理目录复现与发布.zh-CN.md)。

原76条重点记录与原存量表保持原时点；新增11标签（6个execute尝试）见[JSON](published_directory_reproduction.json) / [CSV](published_directory_reproduction.csv)。最新收尾快照124.216GB，未永久删除。

## 章节

- [R85任务链：反馈、队列、时钟和停车准备](chapters/02_R85至R117.zh-CN.md)
- [Round27：发布等待、原子准备与放弃压缩路线](chapters/03_round27_A至P14.zh-CN.md)
- [小协议重构及诊断与正式验收的分界](chapters/04_compact01至03.zh-CN.md)
- [正式验收前半：部署覆盖、桥接收与转向闭环](chapters/05_formal01至12.zh-CN.md)
- [正式验收后半：matcher输入、世界事件与真实支撑面](chapters/06_formal13至22.zh-CN.md)
- [结论与证据规则](chapters/01_结论与证据规则.zh-CN.md)
- [留存与不可恢复边界](chapters/07_留存与证据缺口.zh-CN.md)
- [软件/取消/未启动支线](chapters/08_未部署与已停路线.zh-CN.md)

## 机器可读映射

- [按UTC与源提交的history.json](history.json) / [CSV](history.csv)
- [全135目录存量JSON](storage_inventory.json) / [CSV](storage_inventory.csv)
- [精选小收据投影](evidence/receipts.json) / [小日志字段摘录](evidence/selected_logs.zh-CN.md)：原文件SHA及逻辑路径，不上传大raw。

## 快速时间线

|UTC标签|轮次|实现|结果/阶段|后续|
|---|---|---|---|---|
|2026-10-18T09:54:11+00:00|R86-feedback-isolation-round01|928d8b32|failed|R87诊断/R88拆计算|
|2026-10-18T11:30:04+00:00|R87-diagnostic-round02|928d8b32|failed|R88异步计算|
|2026-10-18T12:10:28+00:00|R88-async-admission-round03|0669cbb7|failed|R89–R94时序/调度调查|
|2026-10-18T14:59:24+00:00|R89-thread-attribution-round04|8b5812d5|failed|后续phase观测优化|
|2026-10-18T15:24:29+00:00|R90-positive-yield-round05|未知|failed|R91/R92|
|2026-10-18T18:07:17+00:00|R91-empty-retry-round09|00d8cc84|failed|R92/R93|
|2026-10-18T18:35:07+00:00|R91-recording-round10|673f83a2|failed|R92/R93|
|2026-10-18T18:56:11+00:00|R92-ready-commit-round11|622f5c8b|failed|round12离线burst重放|
|2026-10-19T05:12:33+00:00|R93-handoff-diagnostic-round13|1ce5ed36|failed|R94 poll就绪|
|2026-10-19T06:45:00+00:00|R94-scan-ready-poll-round14|c7a2d7c3|failed|R95审计/R96边界|
|2026-10-19T10:11:20+00:00|R96-cpu-boundary-validation-round16|c69356a4|failed|R97字节预算观测|
|2026-10-19T10:44:03+00:00|R97-diagnostic-bytecap-round17|5a184166|failed|R98 eager发送|
|2026-10-19T12:01:48+00:00|R98-eager-send-round19|0b6a6a43|failed|R99 clock反馈对照|
|2026-10-19T14:21:10+00:00|R99-clock-feedback-round20|b5d821e7|failed|R100收尾修复；之后R99 OFF|
|2026-10-23T06:06:21+00:00|R100-burst-intake-060621|未知|failed|R102多方向/R103结构修复|
|2026-10-23T06:07:13+00:00|R100-burst-intake-060713|未知|failed|R102多方向/R103结构修复|
|2026-10-23T06:14:06+00:00|R100-burst-intake-attempt2|未知|failed|R102多方向/R103结构修复|
|2026-10-23T06:16:42+00:00|R100-clock-witness|未知|failed|R102多方向/R103结构修复|
|2026-10-23T09:04:40+00:00|R102-A-round25|未知|failed|R103分别落地|
|2026-10-23T09:25:11+00:00|R102-B-round25|未知|failed|R103分别落地|
|2026-10-23T09:35:11+00:00|R102-C-round25|未知|failed|R103分别落地|
|2026-10-23T09:44:20+00:00|R102-E-round25|未知|failed|R103分别落地|
|2026-10-23T09:55:40+00:00|R102-D-round25|未知|failed|R103分别落地|
|2026-10-23T10:41:41+00:00|R103-status-round26|19f51660|failed|R104 causal join|
|2026-10-23T10:55:32+00:00|R103-imu-round26|d72dbaec|failed|R104 causal join|
|2026-10-23T11:25:51+00:00|R103-handoff-round26|95c60f08|failed|R104 causal join|
|2026-10-23T11:44:55+00:00|R103-replacement-round26|aa7c23a9|failed|R104 causal join|
|2026-10-23T12:28:42+00:00|R104-causal-join-round26|84392fa5|failed|R105 terminal owner|
|2026-10-23T13:33:56+00:00|R105-terminal-owner-round26|59f96ab3|failed|R106 prequalification|
|2026-10-23T14:08:33+00:00|R106-prequalification-hold-round26|1b7d8208|failed|R108快照/R128触发/R后续版本文件|
|2026-10-23T15:02:23+00:00|R107-queue-timing-round26|3ae0d078|failed|R110结果让出/R114 clock|
|2026-10-23T15:21:10+00:00|R109-r109-speed020-round26|f646d0ba|failed|R111 startup joint proof|
|2026-10-23T15:49:12+00:00|R111-r111-speed020-round26|ac668ee3|failed|R112次生生命周期/R114clock|
|2026-10-23T16:01:59+00:00|R110-poll-yield-round26|f0854aff|failed|R114/R116/R最终short intake|
|2026-10-23T16:40:49+00:00|R114-clock-service-round26|e05475da|failed|R116有界pressure；round27重审|
|2026-10-23T17:07:27+00:00|R115-r115-speed020-round26|4bac533b|runtime_completed_navigation_unverified|R117动态lead设计|
|2026-10-23T17:15:35+00:00|R115-r115-speed050-round26|4bac533b|failed|R117动态lead设计|
|2026-10-23T17:32:09+00:00|R117-r117-speed050-round26|191582d5|runtime_completed_navigation_unverified|round27重回B05/B10控制底座|
|2026-10-24T05:17:02+00:00|A02|未知|prepare_failure_before_physics|A03恢复精确依赖后实跑|
|2026-10-24T05:24:02+00:00|A03|572375ca|failed|B04/B05定向metadata ASYNC|
|2026-10-24T05:53:01+00:00|B04|bab7312a|failed|B05完整继承环境适配|
|2026-10-24T06:02:52+00:00|B05|cd19e89a|failed|R121/R127/R131原子准备调查|
|2026-10-24T06:36:12+00:00|D06|95fa0a4c|failed|B10基线/退出修复|
|2026-10-24T07:06:57+00:00|C07|db8c3211|failed|C08唯一同业务启动复验|
|2026-10-24T07:20:48+00:00|C08|db8c3211|failed|停止叠补丁，B10回B05底座|
|2026-10-24T08:18:22+00:00|B10|f8b0900d|failed|D11更近真实动态trigger|
|2026-10-24T09:34:05+00:00|D11|5757c382|failed|R131保留capture unknown scope gap|
|2026-10-24T11:20:01+00:00|P12|104109a3|failed|R138驻留表示（后被取消）|
|2026-10-24T13:01:05+00:00|P13B|ed249dfa|failed|P14 body ASYNC|
|2026-10-24T13:42:20+00:00|P14|5334217e|failed|用户取消压缩/缓存路线，改小协议|
|2026-10-24T14:33:53+00:00|compact01|cec8a1f1|failed|compact02版本文件/事件终态|
|2026-10-24T15:18:36+00:00|compact02|de5fe1fd|failed|compact03 preT0 recapture|
|2026-10-24T16:32:02+00:00|compact03|ac0dfcef|diagnostic_evidence_complete|formal01/02严格模式|
|2026-10-24T17:28:34+00:00|formal01|未知|prepare_failure_before_physics|formal02准确正式配置|
|2026-10-24T17:33:37+00:00|formal02|baab445e|failed|formal03 heartbeat职责修复|
|2026-10-24T17:59:59+00:00|formal03|ab9e30f2|failed|formal04 actual dispatcher修复|
|2026-10-24T18:19:29+00:00|formal04|340945eb|failed|formal05 late slew+bridge receipt|
|2026-10-24T18:46:28+00:00|formal05|53bd8b51|failed|formal06安全小入口/owner handoff|
|2026-10-24T19:32:54+00:00|formal06|2e715883|failed|formal07显式world event/nav segment v2|
|2026-10-24T20:26:03+00:00|formal07|cfd9b3f9|startup_failure_before_task_acceptance|formal08唯一同代码冷启动复验|
|2026-10-24T20:38:18+00:00|formal08|cfd9b3f9|failed|formal09 gate零后重收敛|
|2026-10-24T21:08:41+00:00|formal09|15a643db|startup_failure_before_task_acceptance|停止原样重试，native IMU优先|
|2026-10-24T21:54:09+00:00|formal10|8ffd2ca7|import_failure_before_Popen|v3导入上下文隔离→formal11|
|2026-10-24T22:16:44+00:00|formal11|8ffd2ca7|failed|formal12普通turn连续交接|
|2026-10-24T22:32:40+00:00|formal12|0c69a852|failed|formal13 SCAN短入口|
|2026-10-24T23:08:34+00:00|formal13|c144ed79|failed|formal14有界真实query捕获|
|2026-10-24T23:42:12+00:00|formal14|aa3e4b67|failed|formal15 NF ray人口|
|2026-10-25T00:00:14+00:00|formal15|9c0c45e5|controlled_interruption_after_stable_deadlock|formal16b/17显式独立world motion|
|2026-10-25T00:20:00+00:00|formal16b|9143f06b|failed|formal17只开既有boundary timing|
|2026-10-25T00:38:22+00:00|formal17|9143f06b|failed|静态支持新模型原型/held-out离线|
|2026-10-25T01:17:42+00:00|formal18|cfe0acc7|approval_denied_not_executed|新增相关授权后18b原审批重新判定|
|2026-10-25T02:48:58+00:00|formal18b|cfe0acc7|failed|formal19定向relay ASYNC|
|2026-10-25T03:08:30+00:00|formal19|c0b0b290|failed|formal20同面connector terrain grade|
|2026-10-25T03:36:15+00:00|formal20|28a9791c|failed|formal21 status准备/发布分类|
|2026-10-25T03:53:08+00:00|formal21|58bdc24e|failed|formal22连续同面terrain/body语义|
|2026-10-25T04:21:30+00:00|formal22|0f356628|succeeded|发布选定策略和事实，保留未决|

## 发布/清理边界

本分支与main已按正常提交保留原仓库历史发布，不force、不删旧分支、不修改可见性。此产物不是逐轮完整Git对象仓库或全raw备份；表中源码SHA用于原机溯源，不保证每个历史SHA在本远端可checkout，也不承诺旧轮全重放。任何永久删除仍待精确清单和后续一次确认。

## 当前清理审查，尚未执行

[分组盘点与边界](cleanup-inventory/README.zh-CN.md)、[逐项不可恢复风险](cleanup-inventory/CONFIRMATION_REVIEW.zh-CN.md)、[公开只读清单](cleanup-inventory/EXACT_CANDIDATE_MANIFEST.json)。首批5个文件分配量上界1,143,734,272B，其中4份独有诊断流仍HOLD，全部未获删除许可/不具执行条件，实际释放0。284个其它大文件仅metadata、不在已哈希候选范围。formal22整轮及关键未决原始窗口保护。公开路径是别名且去除了宿主身份，不可据此执行删除；原机精确清单SHA另列。报告上传≠原始数据全备份。

本分支从正式main的formal22发布提交派生，main保留清晰当前架构与源码；本页提供按时间、实现/部署和证据等级整理的演进。

原历史包的DELIVERABLES_SHA256记录原始文档字节；公开版换行/路径规范化后的对应SHA见PUBLIC_EXPORT.json，两者不混用。

## 后续完整清理容量方案（未删除）

[20.31/30.75/50.91/56.20GB四档、逐文件范围与不可恢复损失](cleanup-capacity/README.zh-CN.md)。本计划取代仅C001的局部建议，当前全部删除授权仍false。原停车命令时序澄清已正常追加到main结果，本次/新测试原始数据继续保护。

前阶段PUBLIC_EXPORT.json/PUBLICATION_APPENDIX.json是各自历史时点清单；本次新增清理附录与当前索引以[cleanup-capacity/export_index.json](cleanup-capacity/export_index.json)校验，不把旧SHA清单冒称当前全树。
