# 独立归档字段关联审计的只读展示

本次仅修改 Teacher `scripts/serve.py`，追加折叠块“新增归档字段关联联合审计（原收据保留）”。主选发布账本收据、原 common、原 ramp 均继续显示 **UNVERIFIED**。新卡片展示独立 metadata v2 审计的 **PASS（5/5）**，不覆盖、替换或改写任何旧验收门。

## 独立源码复核

实际 full run 是 `20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb`，canonical 路径位于受控 private `/var/tmp/go2_teacher_simulation_20261005/`，项目 alias 保持同名。冻结 `teacher_wrapper.py` 在 history 中添加 `monotonic_wall`、`ros_sim_time`；atomic 状态文件使用同一原 data。冻结 EvidenceWriter 先同步序列化、再单 FIFO append/atomic。因此旧整行 canonical 比较会误拒正常历史行；并非需要删掉实际状态字段。

新 reader 固定 schema `independent_exact_terrain_status_payload_metadata_join/v2`、SHA `cb2e8aa4b383f5feb26b7ba154158062a289d266b081d2d76e417944c9267c94`，并固定其 v1 依赖 SHA。原 74 个字段包括所有嵌套值精确且唯一关联，只有上述两个生产者证明的归档字段被投影。实际行 wall 449588.381504006 <= read_start .39111505 <= read_end .391601091 <= event .396736713；原 arrival/status/native clocks 为 131.675/131.830/131.850 s。原 terrain_switch helper 全部其他条件原样运行，包括有效物理相位、原请求/SLAM 区域因果绑定、187 点射线和 Actor 实际应用。原 common/v2/ramp 以及 metadata v1 的 SHA 均与复核前一致。

## 显示拒绝条件

新卡片固定 5 审计门、3 个原祖先的身份和 SHA、原 common 19 门、发布账本 26 门（其中 7 个额外门）、正式完整坡道 25 门。所有数值门与单一允许 PI 替换关系保留；缺 ledger、partial prefix、另一原门失败、pilot 坡道、未知 schema/reader/candidate、foreign run、变化 source/input bytes 均不能显示 PASS。闭合记录的全部 byte maps 必须与原三份收据实际 maps 相同；当前文件按原 SHA 核对。独立附加 map 与 v1 原 payload 证明也完整核对。

SHA 计算改为流式读取，保持原摘要结果；stat(dev,inode,size,mtime_ns,ctime_ns) 稳定才复用缓存，编辑或替换重新核对。实际 48 个去重绑定、2.126 GB 的首次只读核对约 0.66 s，缓存后约 0.02 s（此次主机页缓存条件，非算法性能结论）。

## 验证

20 个新边界测试加原 55 个历史边界测试全部通过，共 75 项。新测试使用实际小收据的隔离副本与原 evaluator byte-digest oracle，不改实际记录；实际 run 另进行了真实流式 SHA 及状态选择核对。Python AST 和注入 JavaScript 语法通过。日志、验证摘要、前后源与 diff 保存于本目录。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/viewer_audit_metadata_v2/test_metadata_receipt.py
node --check multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/viewer_audit_metadata_v2/additive_script.js
```

没有重启 viewer，没有运行或修改仿真、Teacher 控制数学、源判据、训练任务、camera_mode 或磁盘旧网页。父任务负责唯一只读服务重启与浏览器实际核对。卡片只支持本轮静态 32 区域、两条完整 12 m 坡道与首次 5 秒停车；Actor 仍含 232 维特权输入。动态障碍、全传感器 Actor、通用 Sim2Sim 与真机保持未验证。
