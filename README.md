# FAST-LIVO2 / SCAN / Gazebo Go2 Frozen Teacher

本仓库保存本地仿真适配的源码、场景资产、冻结配置和精简实验报告。仅仿真；不包含实体机器人部署，也不重新训练 Teacher。

- [总实验报告](docs/EXPERIMENT_REPORT_20261006.md)：通过、失败及尚未验证范围。
- [精简证据与曲线](evidence/INDEX.md)、[证据清单](evidence/EVIDENCE_MANIFEST.json)。
- [原始数据保留边界](docs/EVIDENCE_RETENTION.md)。较早清理351.5 GiB原始数据；10月6日下午另清理旧raw 68.51 GiB及过期pip缓存8.81 GiB，合计77.32 GiB，见[本轮清理](multifloor_demo/teacher_mode/test_results/corridor_tracking_v20_continue_20261006/cleanup/README.md)。最新401b/dc50原始运行保留本机。历史[清理总计](maintenance/CLEANUP_SUMMARY.json)、[删除回执](maintenance/PURGE_RECEIPT.json)和[核验](maintenance/PURGE_VERIFICATION.json)保留原字节。
- [原 Teacher 开发说明](multifloor_demo/teacher_mode/README.md)。旧路径与旧 PASS 均为历史记录；新机器必须重新构建和预检。

## 2026-10-06 V20候选分支与续测

有限弧长朝向参考续测 `401b` 已实际完成原始 **9/9区域和首次固定5秒停车**，独立有限验收通过，停车SLAM最大XY漂移6.55 mm。第九区23.625秒到达，到达前没有 drive→pre_turn 复位（旧V19为104次并超时）。随后 `dc50` 实际尝试原完整46区，到达9区后在第十区途中越过原0.45 m路线边界而停车，**完整46区失败**。两次SCAN导出配置和任务终点不同，不作严格单变量A/B。

[本轮续测与清理报告](multifloor_demo/teacher_mode/test_results/corridor_tracking_v20_continue_20261006/README.md)、[先前修改方案](multifloor_demo/teacher_mode/test_results/corridor_tracking_v20_20261006/TRAJECTORY_SELECTION_PLAN.md)。走廊仍是只读shadow，本轮447次返回0认证；旧路径保留、主动走廊控制未启用。下一步保留旧路径需要监视实际执行路径，不能依赖SCAN对其最新候选的碰撞检查。

这是可回滚的源码与证据检查点，main基线为 `5237b886`。不含模型、build或完整raw；V20原机gate不授权clone运行，见[导出和回退说明](docs/V20_BRANCH_CHECKPOINT.md)。前九区通过不代表46区、新闭环Sim2Sim或真机通过。先前39ea因显存保护只完成7区的失败仍保留，不回填。

## 2026-10-06 V19实测

多线程接收/点云解码/图像解码流水线已在原工作区实现并实测；IMU→LIO→VIO共享状态仍由唯一owner顺序执行。同包1.5x单次容量对照的最大位姿滞后2.535 s→50 ms，1x ABBA未见明显时效改善，不能宣称所有场景同幅提速。

原46任务启动失败1次，运动尝试2次，最终 **8/46失败**。修复内外朝向参考冲突后，SCAN微小起始段的切线仍104次跳变触发转向门，第9区90 s超时；139,102条输入全部正常提交，0取消。较长任务源时间30.303 Hz、墙钟22.338 Hz，整套系统尚未持续满实时。

[本轮最终报告](multifloor_demo/teacher_mode/test_results/pipeline_v19_20261006/README.md)、[失败因果与曲线](multifloor_demo/teacher_mode/test_results/pipeline_v19_20261006/evaluation/V19_9779_FAILURE_REPORT.md)。V19大raw已在下午按清单部分清理，9779的PID/SLAM/status及样条对照仍留本机；Git只保存精简证据。V19在新目录提供源码和构建入口，便携运行仍阻断，不能用原机gate绕过重新验证。

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

新目录已从源码构建SDK、ROS依赖、SCAN、执行器及V12/V18/V17；150次有限数值检查通过，V18 prepare-only已运行成功（没有启动ROS/Gazebo）。V17保持队列语义预检阻断。操作命令见[构建与运行说明](tools/README.md)，实际验证范围见[迁移说明](tools/PORTABILITY_CHANGES.md)与[验证收据](tools/PORTABILITY_VALIDATION.json)。新目录尚未重跑实际仿真，不继承历史导航PASS。完整原始数据删除后不能精确回放或重新独立验收旧实验，源码用于重新运行新的实验。

无需 ROS、torch 即可检查场景资产与配置：

```bash
python3 -B tools/prepare_offline.py --variant v18 \
  --profile combined_lio4_vio1_l64_r30_c30_600 \
  --output /tmp/go2_teacher_source_preview
```

输出目录必须不存在。`PASS_SOURCE_ASSETS_ONLY` 仅表示源码/资产预检，不授权真实运行，也不表示运动、Sim2Sim或导航通过。实际仿真需按tools说明完成当前主机的新构建和数值预检，再显式选择执行；准备成功不等于实际仿真通过。
