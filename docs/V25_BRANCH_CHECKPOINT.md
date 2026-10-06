# V25独立检查点

分支 `codex/v25-terrain-event-full46` 从冻结V24 `002716c7705ca3d2e1392c01176970d4865c170d` 建立；冻结候选提交 `b647bf2`。本轮修复仅terrain_provider事件合并，新增有限测试与来源门；原控制数学/增益、曲率开关、300ms/原区域/原期限不变。

实际e8de原46任务11区到达，第12区失败，原22门10通过/4失败/8未验证。Actor仍232维特权输入，导航实际SLAM/IMU/点云/SCAN；无实体机器人与重训操作。详情与精简证据在V25_REPORT.md及provenance/SOURCE_EXPORT_V25_CLOSED_ATTEMPT.json。

回滚可切换 `codex/v24-recovery-replan-full46`、`codex/v23-curvature-full46` 或 `codex/v22-curvature-prefix9` 独立分支，原main不变。不含模型与完整raw，不复用旧PASS。本机绝对来源gate只是已测工作区身份证据，新机器须按tools/README.md重建和重新预检；当前portable入口不构成V25实际部署通过。
