# V23完整46区可回滚检查点

本分支从V22检查点`4a3b116fca9980d8057352fee8429d4a93d46ead`建立，保存V23源码和实际失败证据，不覆盖main或V20。原机实际运行57fa到达9/46，第10区超时；原失败和后续未执行范围均保留。

精确复制清单为[provenance/SOURCE_EXPORT_V23_FULL46.json](../provenance/SOURCE_EXPORT_V23_FULL46.json)。仅精简报告、compact和源码，不含完整raw、权重或本地build。点云已无损归档于原机，归档校验和删原件收据保留；要做完整原几何重放必须先恢复归档。

原绝对路径gate与源码快照用于证明原机执行来源，不授权clone运行。新机器仍须迁移路径、构建及重新冻结/预检，不能继承导航通过结论。网页到达盒补画属于运行后的显示修复，原执行控制源码没有修改。

```bash
# 原V22检查点，适合在另一个worktree查看
git worktree add ../go2-v22-checkpoint 4a3b116fca9980d8057352fee8429d4a93d46ead
# 本V23失败检查点
git worktree add ../go2-v23-checkpoint codex/v23-curvature-full46
```

不要用强制reset覆盖其它任务当前工作；没有重训Teacher、真机验证、跨帧解耦或完整新律Sim2Sim认证。
