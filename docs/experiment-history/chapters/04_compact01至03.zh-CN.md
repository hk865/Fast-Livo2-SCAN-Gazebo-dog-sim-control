# 小协议重构及诊断与正式验收的分界

按UTC标签排序。每项保留原结果；软件/离线/启动与物理结果不混用。详情和证据SHA见 [history.json](../history.json)，精选小收据投影见 [receipts.json](../evidence/receipts.json)。

## compact01

UTC：2026-10-24T14:33:53+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `cec8a1f1a28c1460f80e04229a560e7a4ba5de8e`；部署 `3c4bcd2a7e282e445486b26379908a4dfb873c38`。

目标与修改：减少高频大JSON；低频route_definition/schema3小局部消息。

相对前阶段：34.967m、原fixed5 PASS、375receive=consume。主要证据与错误：动态真取消+可变reference SHA，不能只改SHA称PASS。

原结果：`failed`；task原失败字段：`actual_dynamic_failed:ValueError: JSON source SHA mismatch: [local:r85_global_reference.json]`。SIM完成：286.83；高水位：34.96679486321258m。

实际触发：见本轮已有收据；未将加载等同触发。后续处理：compact02版本文件/事件终态。

证据边界：实际运行小结果/退出收据；现目录分配1.466016GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/compact01/r85_task_result.json`（SHA e318b3b57b66…）；`RUNS/compact01/run_result.json`（SHA fcad65a19c61…）；`RUNS/compact01/worker_archive_integrity.json`（SHA 3f49946df5a4…）

## compact02

UTC：2026-10-24T15:18:36+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `de5fe1fd61f8021c30eb1cb2a4df4ecebad332f1`；部署 `1ef8d580d957aa58df6fe2d33cc7fc82920898bc`。

目标与修改：完整动态事件窗口；版本化reference/guard descriptor/完成不追溯取消。

相对前阶段：真实九门离线PASS，F3区域到达。主要证据与错误：首T0未建立后永久zero_hold；全程deadline FAIL。

原结果：`failed`；task原失败字段：`whole_task_simulation_deadline_exceeded`。SIM完成：789.905；高水位：34.91595171058685m。

实际触发：动态原实际event验证PASS；不是fullnavPASS。后续处理：compact03 preT0 recapture。

证据边界：实际运行小结果/退出收据；现目录分配3.083170GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/compact02/r85_task_result.json`（SHA 9d88ba1091e8…）；`RUNS/compact02/run_result.json`（SHA 8b0e7c36f205…）；`RUNS/compact02/worker_archive_integrity.json`（SHA c2e63709ef1c…）

## compact03

UTC：2026-10-24T16:32:02+00:00（目录标签）；真实diagnostic minimal；不是formal PASS。实现 `ac0dfcef0d5d5a642f4f9111e1e28c29fad39455`；部署 `f93bc27bcdbd48e33d2c3b19b5e97b1c8619eecc`。

目标与修改：preT0和ACK减负；小协议+恢复/原ACK复用。

相对前阶段：五项actual原谓词真、fixed5真。主要证据与错误：仍diagnostic minimal，不能正式PASS；断线缺原ENTRY_EXIT。

原结果：`diagnostic_evidence_complete`；task原失败字段：`R85 actual task stop: actual_R85_diagnostic_terminated; no autonomous route PASS`。SIM完成：337.065；高水位：34.96679486321258m。

实际触发：新恢复分支缺实触发，不计收益。后续处理：formal01/02严格模式。

证据边界：实际运行小结果/退出收据；现目录分配1.418813GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/compact03/r85_task_result.json`（SHA 9c5b9a04110d…）；`RUNS/compact03/run_result.json`（SHA 48c256711fad…）；`RUNS/compact03/worker_archive_integrity.json`（SHA c8187d7cd9b3…）
