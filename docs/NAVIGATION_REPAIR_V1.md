# 修复候选的构建与运行边界

根 `candidate/` 将冻结业务 `7b954872` 的 DDS 集成、停车 V1、动态坐标参考切换移植到既有公开目录入口。保留 `package_paths.py`、local_setup 与显式 SDK 路径，原 `releases/formal22/` 历史快照未覆盖。逐文件来源见 [移植清单](../provenance/NAVIGATION_REPAIR_V1_PORT.json)。

## 必要输入

1. 先按 [原目录复现说明](PUBLISHED_DIRECTORY_REPRODUCTION.md) 提供合法的固定 Student/Teacher 权重、ROS Jazzy/Gazebo、native、SCAN、SDK 和 actuator。模型及 ELF 不上传；公开包不是包含所有外部依赖的一键发行版。
2. 按 [私有 DDS 构建说明](PRIVATE_DDS.md) 构建固定 2.14.6 补丁及观察器，生成 `.local` 实际库路径、SHA、BuildID 合同。缺少或身份不符必须拒绝，不能回落系统 DDS。
3. 将 [业务覆盖配置](../profiles/navigation_repair_v1.overlay.json) **递归覆盖**到已绑定的原 A 全路线 profile，保留未列出的算法、PID、路线、容量、安全参数。该 JSON 只是业务选择，不是完整可直接运行的 profile。`cascade` 只合并其 `parking_pose10hz` 项，不能替换其余增益。
4. 为本 checkout 重新绑定 native/SCAN、POLICY、几何元数据及所有被引用文件 SHA。合同中的历史本机绑定不能在异机当成已验证依赖。生成新的本轮资源证明、时间窗口与实际加载期望，既有过期行政窗口不续用。
5. 使用现有原机管理入口 `round27_v3.py` 的 propose → 独立审核 → apply → 冻结 → prepare → execute → verify-loaded → closure。它属于当前部署环境，不以文档中的占位路径替代真实依赖。运行命令与最终输入清单保存在该轮回执。

默认保持 serial 输入模式、IMU 优先、LIO 内部原并行、异步中转发布、SHM+UDP/可靠 QoS。没有恢复 V19 staged 输入、替换 SLAM/SCAN 算法或抬高 300ms/容量门。正式选择必须为 `r85_strict_formal_f1_f3/v4`，并实际装载相应停车和动态合同。

## 无仿真检查

```bash
PYTHONDONTWRITEBYTECODE=1 python3 candidate/parking_pose10hz_checks.py
PYTHONDONTWRITEBYTECODE=1 python3 candidate/dynamic_coordinate_handoff_checks.py
PYTHONDONTWRITEBYTECODE=1 python3 candidate/dynamic_formal_selector_checks.py
```

真实 terminal 方法检查还需要 `PARKING_TERMINAL_SOURCE_RUN`、`PARKING_TERMINAL_PROFILE` 和对应消息 SDK；这些外部历史数据未打包。缺失时应报告未运行，不能生成假原始消息。

[两轮自然结果及受控切换重放](../reproduction/results/navigation_repair_v1/RESULT.zh-CN.md) 的范围不等于异机复现。demo 将在发布后用同一业务代码和新的行政绑定录制；旁观镜头与归档开销另记，真实业务验收不放宽。
