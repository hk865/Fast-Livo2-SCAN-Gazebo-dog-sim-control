# 自动刷新时保留收据卡片展开状态

浏览器实际 QA 发现原注入 renderer 每次自动刷新 `replaceChildren()` 都重建 details，展开的独立审计会立即折叠。本修订仅改 `serve.py` 的注入 JavaScript：重建前保存每个 details 当前 open 属性，以当前 canonical `run_id`、收据 filename 和完整 SHA256 绑定；相同 run/收据才恢复。用户手动关闭同样保留。切换 run 清空全部状态，SHA 或 filename 变化不继承；无 run 身份或无有效 digest 不缓存，已不再显示的 key 清理。

Node 执行实际注入 renderer 的有限 DOM 行为验证 11 项通过，覆盖同 run 两卡片独立保留、手动关闭、SHA/filename 变更、跨 run/返回旧 run、无 digest/run、原主验收和 raw 内容保持。JavaScript 语法及 Python AST 通过。剔除唯一注入 script 常量后，Python AST 与前版完全相同，原 75 收据边界通过结果保持，无需重复重放。

前版 `viewer_audit_metadata_v2/manifest.json` SHA 仍为 `3dbf37d790d6e9dd393f4510e4938acbf15ade46bd53ba7e8f7a6845accfd48d`，前后源、diff、功能日志与本修订 manifest 独立保存。未动 reader、原收据、camera_mode、web 原件或物理进程，没有重启服务。父任务负责唯一 viewer 重启与实际浏览器复核。

```bash
node --check multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/viewer_audit_metadata_v2_refresh_revision/additive_script.js
node multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/viewer_audit_metadata_v2_refresh_revision/test_refresh_state.js
```
