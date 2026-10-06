# V25 原 46 区只读评价适配器

透明继承冻结 V24 评价器；原 22 项检查和所有数值阈值保持不变。新增来源闭合要求 V25 的 `terrain_provider.py`、`mission46_terrain.py`、`worker.py` 与已保留的 `shared_controller.py`、`replan_policy.py` 均来自该候选的精确绝对路径、固定 SHA 和本次运行的规范归档路径。旧候选同名文件不能代替。

13 项有限测试和 `--check-adapter` 已通过，原检查表达式经完整模块 AST 比较，仅允许候选标识、祖先 SHA 保护、额外来源固定值与说明字段变更。实际准备目录 974b 的五项来源闭合已验证。未读取运行中的 e8de 原始数据。

仅在根任务确认运行已完全结束后执行一次，输出必须不存在且必须位于运行目录之外：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/curvature_full46_20261006/v25_evaluation/evaluate_full46_v25.py \
  --run /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_182824_closed_loop_cascade_clock_hold_terrain_event_V25_full46_r1_isolated_e8de \
  --output multifloor_demo/teacher_mode/test_results/curvature_full46_20261006/v25_evaluation/e8de_FULL46_ACTUAL_EVALUATION.json
```

此命令使用原完整读取流程，没有增加遥测/点云扫描；新增工作只读小型来源 JSON 和五个源码。缺失 `worker_result.json` 等原必需来源仍按原行为失败，不补造文件。完整 200 Hz 物理回放、全部发布/PI/SCAN 几何回放仍为 `UNVERIFIED`；`functional46_limited_pass` 只排除这两项，任何其他失败都不能升级通过。

候选冻结与有限运行准入见 `../v25_actual_review/FULL46_LAUNCH_REVIEW.json`、`PREPARE_SOURCE_CLOSURE.json`；它们不证明实际任务成功。
