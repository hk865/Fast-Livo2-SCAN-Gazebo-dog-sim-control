# Round27：发布等待、原子准备与放弃压缩路线

按UTC标签排序。每项保留原结果；软件/离线/启动与物理结果不混用。详情和证据SHA见 [history.json](../history.json)，精选小收据投影见 [receipts.json](../evidence/receipts.json)。

## A02

UTC：2026-10-24T05:17:02+00:00（目录标签）；prepare/工具/审批，未物理启动。实现 `未从现存对应收据确认`；部署 `未确认`。

目标与修改：整合R116与观测；候选prepare。

相对前阶段：未进入物理任务。主要证据与错误：缺私有native guard依赖。

原结果：`prepare_failure_before_physics`；task原失败字段：`无/未形成task结果`。SIM完成：None；高水位：Nonem。

实际触发：未启动。后续处理：A03恢复精确依赖后实跑。

证据边界：实际尝试目录/保留报告；缺task/exit的边界明确；现目录分配0.000270GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/A02/run_result.json`（SHA 022fff233778…）

## A03

UTC：2026-10-24T05:24:02+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `572375caaf64b429ff6bbd36dae4d834679324f4`；部署 `8cb4f43928db8bc77c7001f376f276a2fc245086`。

目标与修改：消费与反馈时序；R116+有界probe。

相对前阶段：到34.826m，捕获publish331ms包络。主要证据与错误：原header305ms/body拒；R116 pressure0。

原结果：`failed`；task原失败字段：`actual_navigation_failed:R86 corrected input integrity: commit:ValueError:body_not_accepted_at_commit`。SIM完成：332.75；高水位：34.825842484052m。

实际触发：R116压力0；同源发布/接收证据实有。后续处理：B04/B05定向metadata ASYNC。

证据边界：实际运行小结果/退出收据；现目录分配1.812611GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/A03/r85_task_result.json`（SHA 70e051a35a7f…）；`RUNS/A03/run_result.json`（SHA c974b6a1f93d…）；`RUNS/A03/worker_archive_integrity.json`（SHA dc359a5f7865…）

## B04

UTC：2026-10-24T05:53:01+00:00（目录标签）；真实启动/早期退出，未进入目标任务分支。实现 `bab7312a707fd3db6fed3e4680eb9ab555abf42f`；部署 `6f9a56b2221d538e76b1a107a78e604ebec153c4`。

目标与修改：定向metadata异步；R119 profile预加载。

相对前阶段：启动前环境合同拒绝。主要证据与错误：原SYNCHRONOUS环境处理未覆盖生产。

原结果：`failed`；task原失败字段：`RuntimeError: Owned navigation_stack exited before binary witness: [ERROR] [fastlivo_mapping-4]: process has died [pid 3935909, exit code 2, cmd '[local:fastlivo_mapping] --ros-args -r __node:=laserMapping --params-file [local:navigation_fastlivo.yaml] --params-file [local:navigation_camera.yaml] --params-file [local:launch_params_z7rm68wx] -r /cloud_registered_full:=/demo/slam/raw_registered_full'].`。SIM完成：None；高水位：Nonem。

实际触发：无有效导航比较。后续处理：B05完整继承环境适配。

证据边界：实际运行小结果/退出收据；现目录分配0.075092GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/B04/run_result.json`（SHA 96298580ed04…）；`RUNS/B04/worker_archive_integrity.json`（SHA d732e7565b79…）；`RUNS/B04/worker_archive_close.json`（SHA 9a9ac6fe8788…）

## B05

UTC：2026-10-24T06:02:52+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `cd19e89a4120d13daeb16d38f92d932d41e9a040`；部署 `d6f84da81ced8738b8d7a7c3ff0f1a06639148ba`。

目标与修改：减metadata发布等待；仅metadata ASYNC。

相对前阶段：331ms未复现，至34.905m。主要证据与错误：缺因果IMU在body/cloud后抛错，真实partialaccept。

原结果：`failed`；task原失败字段：`actual_navigation_failed:R86 corrected input integrity: commit:ValueError:queued_parking_pose_without_causal_IMU`。SIM完成：383.555；高水位：34.905134868390334m。

实际触发：metadata实用；不能称时延根治。后续处理：R121/R127/R131原子准备调查。

证据边界：实际运行小结果/退出收据；现目录分配2.036462GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/B05/r85_task_result.json`（SHA ca0c94895440…）；`RUNS/B05/run_result.json`（SHA 8893520c5171…）；`RUNS/B05/worker_archive_integrity.json`（SHA 8622690e1d82…）

## D06

UTC：2026-10-24T06:36:12+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `95fa0a4cda0eb5438ecf886a0b5cf9edc4cc0b29`；部署 `418b071dd7bd6cfb5194353d265a21e3fd6e2cbd`。

目标与修改：原生GC服务队列；R120队列服务。

相对前阶段：trigger/wake/service/consume各73。主要证据与错误：90s未解决protocol wait；归档退出异常。

原结果：`failed`；task原失败字段：`actual_navigation_failed:r85_unresolved_protocol_wait_sim_budget_exhausted`。SIM完成：389.55；高水位：31.824896305021177m。

实际触发：真实GC73，未到停车故障点。后续处理：B10基线/退出修复。

证据边界：实际运行小结果/退出收据；现目录分配1.957925GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/D06/r85_task_result.json`（SHA c8f10a43c7a6…）；`RUNS/D06/run_result.json`（SHA cd8f5a49d903…）；`RUNS/D06/worker_archive_integrity.json`（SHA edec44cc1a21…）

## C07

UTC：2026-10-24T07:06:57+00:00（目录标签）；真实启动/早期退出，未进入目标任务分支。实现 `db8c32118adb165ec939012758cbe056b630e7c3`；部署 `e5bf4cb04494c0accbfbdcec25fb394ea6aa1329`。

目标与修改：缺IMU前置检查；R121 preflight。

相对前阶段：未接任务就源过期。主要证据与错误：首body header310ms；不能归因仅某模块。

原结果：`failed`；task原失败字段：`R85 actual task stop: actual_R85_navigation_status.json_failed; no autonomous route PASS`。SIM完成：None；高水位：Nonem。

实际触发：停车members0。后续处理：C08唯一同业务启动复验。

证据边界：实际运行小结果/退出收据；现目录分配0.176284GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/C07/run_result.json`（SHA 46e76f7741ea…）；`RUNS/C07/worker_archive_integrity.json`（SHA ec580a8611db…）；`RUNS/C07/worker_archive_close.json`（SHA 5e3594ef3a2b…）

## C08

UTC：2026-10-24T07:20:48+00:00（目录标签）；真实启动/早期退出，未进入目标任务分支。实现 `db8c32118adb165ec939012758cbe056b630e7c3`；部署 `f1a867d1ee0165953584ea3777505eae1eb87a3a`。

目标与修改：启动复验；同R121链。

相对前阶段：仍首body315ms。主要证据与错误：Producer入口前已有大年龄；未证数据全链。

原结果：`failed`；task原失败字段：`R85 actual task stop: actual_R85_navigation_status.json_failed; no autonomous route PASS`。SIM完成：None；高水位：Nonem。

实际触发：停车members0，不归功。后续处理：停止叠补丁，B10回B05底座。

证据边界：实际运行小结果/退出收据；现目录分配0.191705GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/C08/run_result.json`（SHA f719224c6a6e…）；`RUNS/C08/worker_archive_integrity.json`（SHA 9972f6ee2195…）；`RUNS/C08/worker_archive_close.json`（SHA f63f06282047…）

## B10

UTC：2026-10-24T08:18:22+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `f8b0900d32de3b4ba0f49ea156b99d6dbaff19e5`；部署 `e2adafa08bbaf3b5ce53ea9fd4ce57064c07a186`。

目标与修改：验证B05业务底座；R125控制基线+固定退出源。

相对前阶段：原fixed5停车PASS。主要证据与错误：整体动态SHA/取消FAIL，IMU间歇未根治。

原结果：`failed`；task原失败字段：`actual_dynamic_failed:ValueError: JSON source SHA mismatch: [local:r85_global_reference.json]`。SIM完成：335.33；高水位：34.91965460818648m。

实际触发：见本轮已有收据；未将加载等同触发。后续处理：D11更近真实动态trigger。

证据边界：实际运行小结果/退出收据；现目录分配1.845424GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/B10/r85_task_result.json`（SHA 01c0a2a2177f…）；`RUNS/B10/run_result.json`（SHA a148d1c66cbd…）；`RUNS/B10/worker_archive_integrity.json`（SHA 93d9315f996a…）

## D11

UTC：2026-10-24T09:34:05+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `5757c38261719a73138d53309a8549461ac73f29`；部署 `6215d846a5cb43e9ddf8d736e19e145c0f1ab267`。

目标与修改：更近动态触发；R1282.4–2.7m场景lead。

相对前阶段：产生局部动态证据。主要证据与错误：395SIM缺20ms IMU pair，body/cloud已partialaccept。

原结果：`failed`；task原失败字段：`actual_navigation_failed:R86 corrected input integrity: commit:ValueError:queued_parking_pose_without_causal_IMU`。SIM完成：395.09；高水位：34.85252213915601m。

实际触发：见本轮已有收据；未将加载等同触发。后续处理：R131保留capture unknown scope gap。

证据边界：实际运行小结果/退出收据；现目录分配2.097566GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/D11/r85_task_result.json`（SHA 1a88ce0ecdc6…）；`RUNS/D11/run_result.json`（SHA 1cf4baf46d5b…）；`RUNS/D11/worker_archive_integrity.json`（SHA edec44cc1a21…）

## P12

UTC：2026-10-24T11:20:01+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `104109a34c1acbda8722e0459d2fb0e19fe97a56`；部署 `2708e8c7e761f384e76cb2552b9ea0b958d3fc94`。

目标与修改：停车原子准备与退出；R131/R134/R136在B05底座。

相对前阶段：退出链通过，至24.544m。主要证据与错误：r88队列2MiB字节上限；不是第33项count证据。

原结果：`failed`；task原失败字段：`actual_navigation_failed:r88_scan_event_queue_bound`。SIM完成：289.275；高水位：24.543604240666397m。

实际触发：R131 activated=false。后续处理：R138驻留表示（后被取消）。

证据边界：实际运行小结果/退出收据；现目录分配1.640141GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/P12/r85_task_result.json`（SHA 3d04774acf57…）；`RUNS/P12/run_result.json`（SHA ff8e42808f2c…）；`RUNS/P12/worker_archive_integrity.json`（SHA 72abd3086125…）

## P13B

UTC：2026-10-24T13:01:05+00:00（目录标签）；真实启动/早期退出，未进入目标任务分支。实现 `ed249dfab5f940973b9cfb4baf486ae618c86aef`；部署 `4a6edf8a0852f69141df645be4a62abf6f00b37f`。

目标与修改：驻留和IMU取数对照；R138/R139/R141。

相对前阶段：未接任务。主要证据与错误：publish_body331ms低CPU等待、首body305ms。

原结果：`failed`；task原失败字段：`R85 actual task stop: actual_R85_navigation_status.json_failed; no autonomous route PASS`。SIM完成：None；高水位：Nonem。

实际触发：R138编码0/R141目标0。后续处理：P14 body ASYNC。

证据边界：实际运行小结果/退出收据；现目录分配0.166277GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/P13B/run_result.json`（SHA b5571aa6c878…）；`RUNS/P13B/worker_archive_integrity.json`（SHA 281ed9182afa…）；`RUNS/P13B/worker_archive_close.json`（SHA 1a419faa315a…）

## P14

UTC：2026-10-24T13:42:20+00:00（目录标签）；真实受控仿真；局部结论不替代全任务。实现 `5334217e13a68da6caea7e60c0a7ef560cd5a471`；部署 `6bc8376d986ce291a8f7dd8c104f09a17da12d35`。

目标与修改：body异步发布；相对P13B仅body ASYNC。

相对前阶段：已接任务并运动3.251m。主要证据与错误：R138第一个queued spline恢复失败；CDR padding重序列化hash设计不成立。

原结果：`failed`；task原失败字段：`actual_navigation_failed:r138_lossless_restore_failed`。SIM完成：39.99；高水位：3.2513878981288573m。

实际触发：restore failed实际触发；旧raw shadow0。后续处理：用户取消压缩/缓存路线，改小协议。

证据边界：实际运行小结果/退出收据；现目录分配0.389239GB，指定旧C删除记录0项。现存目录/指定文件状态已盘点，是否完整可重放需对应CRC/SHA/selection验证。

精选来源：`RUNS/P14/r85_task_result.json`（SHA eeaaf012d43a…）；`RUNS/P14/run_result.json`（SHA 95ea6ec6f79b…）；`RUNS/P14/worker_archive_integrity.json`（SHA 30ad76a967a2…）
