# FAST-LIVO2 / SCAN / Gazebo Go2 Frozen Teacher

本仓库保存本地仿真适配的源码、场景资产、冻结配置和精简实验报告。仅仿真；不包含实体机器人部署，也不重新训练 Teacher。

- [总实验报告](docs/EXPERIMENT_REPORT_20261006.md)：通过、失败及尚未验证范围。
- [精简证据与曲线](evidence/INDEX.md)、[证据清单](evidence/EVIDENCE_MANIFEST.json)。
- [原始数据保留边界](docs/EVIDENCE_RETENTION.md)。原始大体积数据按用户要求清理；当前实际删除结果将记录在 `maintenance/PURGE_RECEIPT.json`，未生成前表示尚未完成。
- [原 Teacher 开发说明](multifloor_demo/teacher_mode/README.md)。旧路径与旧 PASS 均为历史记录；新机器必须重新构建和预检。

## 已有实验结果

| 范围 | 历史结论 |
|---|---|
| 冻结网络接口、关节映射、基础运动 | 通过；5/10 cm 持续登台各3次功能通过 |
| 真值反馈 PID 标定及保留场景 | 有限范围通过，不能当成 SLAM 导航 |
| 实际 SLAM/SCAN 导航 | V12、V18 指定静态32区域、两条完整12 m上坡及固定5 s停车各单轮通过 |
| V17 接收队列流水线 | 严格失败：31615条接受、31614条提交、1条取消 |
| 新控制器完整跨引擎 Sim2Sim | 尚未通过；缺少匹配 Isaac 闭环对照，旧失败保留 |
| 新版动态障碍整线、全部真实传感器 Actor、真机 | 尚未验证 |

三层连接为坡道。导航使用实际 SLAM/IMU/点云与 SCAN；当前 Teacher Actor 仍保留232维特权仿真输入。V18采用 LIO 4线程、VIO块处理1线程；局部计算优化已测得，但没有证明整条链加速。完整结论以总报告的具体版本与判据为准。

## 代码与模型

`multifloor_demo/teacher_mode/` 包含推理、观测、唯一执行器、控制器、导航、评估器和各独立优化版本；`scan_multifloor/` 与 `slam5_navigation/` 保存构建所需源包；`go2_sim_control/` 保存模型网格。`tools/` 是本仓库路径迁移和新构建入口；`provenance/SOURCE_EXPORT_INITIAL.json` 记录复制时的源码身份，后续迁移修改另记。

权重不提交Git。将自己的冻结权重放到 `models/model_1000.pt`，或设置 `TEACHER_MODEL_CHECKPOINT`。必须匹配：

```text
bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34
```

Teacher CPU单线程、50 Hz；执行器物理200 Hz；300 ms新鲜度保护与唯一关节执行权保留。动作零值不等于停车。源自上游的代码和资产保留各自许可证；本仓库不重新许可这些内容。

## 新环境复现状态

当前提交首先保存源码和报告；新目录构建及有限数值预检正在单独验证，旧预检不能授权新目录运行。最终迁移说明与构建结果将追加至 `tools/PORTABILITY_CHANGES.md`。完整原始数据删除后不能精确回放或重新独立验收旧实验，源码用于重新运行新的实验。

无需 ROS、torch 即可检查场景资产与配置：

```bash
python3 -B tools/prepare_offline.py --variant v18 \
  --profile combined_lio4_vio1_l64_r30_c30_600 \
  --output /tmp/go2_teacher_source_preview
```

输出目录必须不存在。`PASS_SOURCE_ASSETS_ONLY` 仅表示源码/资产预检，不授权真实运行，也不表示运动、Sim2Sim或导航通过。实际仿真入口必须等待本机源码构建与新数值预检完成。
