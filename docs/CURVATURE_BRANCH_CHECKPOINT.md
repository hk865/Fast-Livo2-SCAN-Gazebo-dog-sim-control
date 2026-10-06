# 曲率前馈与限速：独立可回滚检查点

分支 `codex/v22-curvature-prefix9` 从 V20 分支的 `e0533dd3` 创建。V20分支和main不修改。V21修复NumPy布尔/JSON故障；V22补齐五个路线helper直接归档，控制数学与V21完全一致。

原工作区实际运行97ed：原前9区和首固定5秒停车有限PASS，XY漂移4.45mm，原8项门全通过。完整46区的原第十区越界失败保留；跨帧解耦、主动走廊、新律Isaac闭环对照和真机没有在此实现/认证。见[报告](../multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/README.md)和[最终状态](../multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/FINAL_STATUS.json)。

本仓库保存新候选原源码、配置、门、实验报告、精简曲线和失败证据；不含模型、build、完整raw与本机无损点云压缩档。原机gate绑定绝对路径和加载的程序，不能授权clone直接运行；复制后须使用已有构建迁移流程重新构建、审核新主机来源、冻结新门并重新实测，不能用路径替换绕过 gate 或继承97ed PASS。V22原机运行操作在报告；仓库已有portable build工具不自动宣称支持V22执行。

## 回滚

在该源码克隆中使用以下命令可恢复先前源码检查点；未提交改动需先保存。此操作只换代码，不停止任何仿真/训练，也不删除原工作区证据。

```bash
git switch codex/v20-spatial-corridor-shadow
# 回到曲率候选
git switch codex/v22-curvature-prefix9
```

`CURVATURE_SOURCE_CHECKPOINT_20261006.json`和`CURVATURE_COMPACT_FINAL_EXPORT_20261006.json`保存原字节复制证明与精简证据哈希；`CURVATURE_EXPORT_SHA256.json`保存本检查点当前逐文件清单。此前未通过报告仍按原SHA保存。原42轮、原完整46区与新prefix9分别解释，不能凭新PASS修改旧收据。
