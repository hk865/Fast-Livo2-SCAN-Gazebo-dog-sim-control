# 精选小日志/收据字段投影

以下是既有小JSON字段和既有后验报告的结构化摘要，不是大raw逐行转储；没有补采或重跑。原件逻辑定位及SHA在history.json/evidence/receipts.json。

## A03

```json
{
  "id": "A03",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:R86 corrected input integrity: commit:ValueError:body_not_accepted_at_commit",
  "completed_sim_s": 332.75,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 457.1175514380011,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

原header305ms/body拒；R116 pressure0。R116压力0；同源发布/接收证据实有。

## B05

```json
{
  "id": "B05",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:R86 corrected input integrity: commit:ValueError:queued_parking_pose_without_causal_IMU",
  "completed_sim_s": 383.555,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 524.6293333080102,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

缺因果IMU在body/cloud后抛错，真实partialaccept。metadata实用；不能称时延根治。

## P12

```json
{
  "id": "P12",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:r88_scan_event_queue_bound",
  "completed_sim_s": 289.275,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 407.1049390469998,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

r88队列2MiB字节上限；不是第33项count证据。R131 activated=false。

## P14

```json
{
  "id": "P14",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:r138_lossless_restore_failed",
  "completed_sim_s": 39.99,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 75.05101074599952,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

R138第一个queued spline恢复失败；CDR padding重序列化hash设计不成立。restore failed实际触发；旧raw shadow0。

## compact01

```json
{
  "id": "compact01",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_dynamic_failed:ValueError: JSON source SHA mismatch: [local:r85_global_reference.json]",
  "completed_sim_s": 286.83,
  "parking_pass": true,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 399.599800231008,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

动态真取消+可变reference SHA，不能只改SHA称PASS。见本轮已有收据；未将加载等同触发。

## compact02

```json
{
  "id": "compact02",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "whole_task_simulation_deadline_exceeded",
  "completed_sim_s": 789.905,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 952.2044854339911,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

首T0未建立后永久zero_hold；全程deadline FAIL。动态原实际event验证PASS；不是fullnavPASS。

## compact03

```json
{
  "id": "compact03",
  "mode": "真实diagnostic minimal；不是formal PASS",
  "original_task_or_runtime_state": "diagnostic_evidence_complete",
  "original_failure": "R85 actual task stop: actual_R85_diagnostic_terminated; no autonomous route PASS",
  "completed_sim_s": 337.065,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": null,
  "entry_wall_s": null,
  "closure_complete": false,
  "actor_archive_verified": true
}
```

仍diagnostic minimal，不能正式PASS；断线缺原ENTRY_EXIT。新恢复分支缺实触发，不计收益。

## formal03

```json
{
  "id": "formal03",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "less_than_05m_forward_progress_in_90_sim_seconds",
  "completed_sim_s": 198.075,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 250.58520033900277,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

main观察wrapper复制旧predicate，helper未实际委托。旧软件18fixture不代表本轮生效。

## formal10

```json
{
  "id": "formal10",
  "mode": "prepare/工具/审批，未物理启动",
  "original_task_or_runtime_state": "import_failure_before_Popen",
  "original_failure": null,
  "completed_sim_s": null,
  "parking_pass": null,
  "dynamic_pass": null,
  "full_F3_PASS": null,
  "original46_PASS": null,
  "entry_rc": null,
  "entry_wall_s": null,
  "closure_complete": null,
  "actor_archive_verified": null
}
```

缓存旧slam_workspace导入，非物理FAIL。未启动。

## formal13

```json
{
  "id": "formal13",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "less_than_05m_forward_progress_in_90_sim_seconds",
  "completed_sim_s": 284.93,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 362.5864900209999,
  "closure_complete": null,
  "actor_archive_verified": true
}
```

knownscene quality持续拒/3s保护，非队列首因。短入口真实生效；时序仍未全根治。

## formal14

```json
{
  "id": "formal14",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "less_than_05m_forward_progress_in_90_sim_seconds",
  "completed_sim_s": 281.74,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 355.0991306999931,
  "closure_complete": null,
  "actor_archive_verified": null
}
```

预算内前景把all-ray尾4.56%推至5.29%，静态子集稳定。3输入完整；未改质量门。

## formal15

```json
{
  "id": "formal15",
  "mode": "真实运行后由owner受控SIGINT，不是原90s自然FAIL",
  "original_task_or_runtime_state": "controlled_interruption_after_stable_deadlock",
  "original_failure": "RuntimeError: Current retained census failed",
  "completed_sim_s": null,
  "parking_pass": null,
  "dynamic_pass": null,
  "full_F3_PASS": null,
  "original46_PASS": null,
  "entry_rc": 1,
  "entry_wall_s": 465.1109368899779,
  "closure_complete": true,
  "actor_archive_verified": false
}
```

后续FG>5%与all surface仍拒；navfresh耦合冻结Actor。受控SIGINT interrupted，不冒称90s自然FAIL；capture good-only。

## formal16b

```json
{
  "id": "formal16b",
  "mode": "真实启动/早期退出，未进入目标任务分支",
  "original_task_or_runtime_state": "failed",
  "original_failure": "RuntimeError: navigation_stack exited while Actor remained active",
  "completed_sim_s": null,
  "parking_pass": null,
  "dynamic_pass": null,
  "full_F3_PASS": null,
  "original46_PASS": null,
  "entry_rc": 1,
  "entry_wall_s": 35.02884302800521,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

native sync current identity65536点上限startup失败。新Actor目标0，非live PASS。

## formal17

```json
{
  "id": "formal17",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_dynamic_runtime_fatal:ValueError:actual_reference_scope_changed_during_dynamic_protocol",
  "completed_sim_s": 267.975,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 343.60304973999155,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

无真实obstacle_stop/resume，后续真scope变化FAIL；质量仍拒。捕获3真实输入完整；不能清场替代停走。

## formal18

```json
{
  "id": "formal18",
  "mode": "prepare/工具/审批，未物理启动",
  "original_task_or_runtime_state": "approval_denied_not_executed",
  "original_failure": null,
  "completed_sim_s": null,
  "parking_pass": null,
  "dynamic_pass": null,
  "full_F3_PASS": null,
  "original46_PASS": null,
  "entry_rc": null,
  "entry_wall_s": null,
  "closure_complete": null,
  "actor_archive_verified": null
}
```

精确安全副作用未获启动审批，不能称物理失败。启动拒绝、0物理触发。

## formal18b

```json
{
  "id": "formal18b",
  "mode": "真实启动/早期退出，未进入目标任务分支",
  "original_task_or_runtime_state": "failed",
  "original_failure": "RuntimeError: navigation_stack exited while Actor remained active",
  "completed_sim_s": null,
  "parking_pass": null,
  "dynamic_pass": null,
  "full_F3_PASS": null,
  "original46_PASS": null,
  "entry_rc": 1,
  "entry_wall_s": 21.539215025986778,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

native原Q8 buffered cap/image0/IMU0；新质量0。83加载，quality0，非门有效性反证。

## formal19

```json
{
  "id": "formal19",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:r85_unresolved_protocol_wait_sim_budget_exhausted",
  "completed_sim_s": 421.915,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 522.1172284390195,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

connector把毫米高度残差算.3949rad坡；不是最终ACK缺失根因。见本轮已有收据；未将加载等同触发。

## formal20

```json
{
  "id": "formal20",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_dynamic_runtime_fatal:ValueError:actual_reference_scope_changed_after_dynamic_proof",
  "completed_sim_s": 211.93,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 276.0839848140022,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

新notready epoch误报真reference变化；实际reference未发。connector原拒→新额外允许0。

## formal21

```json
{
  "id": "formal21",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "failed",
  "original_failure": "actual_navigation_failed:r85_unresolved_protocol_wait_sim_budget_exhausted",
  "completed_sim_s": 450.33,
  "parking_pass": false,
  "dynamic_pass": false,
  "full_F3_PASS": false,
  "original46_PASS": false,
  "entry_rc": 1,
  "entry_wall_s": 553.6322636640107,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

31末端候选全grade cone UNKNOWN，body残差导数误当地面坡。connector1真额外允许；整曲线未修。

## formal22

```json
{
  "id": "formal22",
  "mode": "真实受控仿真；局部结论不替代全任务",
  "original_task_or_runtime_state": "succeeded",
  "original_failure": null,
  "completed_sim_s": 350.93,
  "parking_pass": true,
  "dynamic_pass": true,
  "full_F3_PASS": true,
  "original46_PASS": false,
  "entry_rc": 0,
  "entry_wall_s": 456.1250646300032,
  "closure_complete": true,
  "actor_archive_verified": true
}
```

旧46false；策略不等价；间歇/统计稳定未证明。179证书41用新依据45patch；末2旧whole也pass，非唯一PASS根因。

## formal22 关键来源

- `ROUND27/trials/compact_formal_22/POST_RUN_COMPACT.json`：原task/停车/动态/加载/退出并列，非仅软件fixture。
- `ROUND27/investigations/deployment/FORMAL22_ACCEPTANCE_INDEPENDENT_REVIEW_FINAL.json`：fcbb786d20aba7aeb968610e4a821974c1f859413b5ecb73535b83d423a02ce9；19项收据核查通过，不伪称独审重跑CRC/verifier。
- `ROUND27/protocol_reduction/curve_support_grade/FORMAL22_EFFECT.json`：新依据41/179，45patch；区别候选/retention人口、原whole可细分通过及不同计时窗口。
