# Go2 C44 核心源码学习快照

本分支保存 2026-10-10 实际运行过的 C44 核心实现，供阅读当前代码、实验进度与失败边界。它是独立根提交的公开源码快照，不继承主分支的实验数据历史。最新这轮完成 5 个普通区、0 个硬区后 FAILED，完整 46 区任务没有通过验收。

## 从哪里开始读

- [实验报告](docs/C44_EXPERIMENT.md)：实际结果、直接失败链、已解决事项及仍未证实的原因。
- [模块进度](docs/PROGRESS.md)：定位、规划、控制、通信、归档和场景验收的具体边界。
- [精选日志](docs/C44_LOG_EXCERPT.log)：少量原工程日志与明确标注的脱敏字段投影。
- `multifloor_demo/teacher_mode/navigation/engineering_v34_r44_active_p95_capture_candidate/`：当前高层控制、已知场景辅助定位、任务、通信、保护和采样归档实现。目录名称保留原版本名；残留 preparation-only 文字属于旧准备阶段，不能据此否定 C44 已实际运行。
- `multifloor_demo/teacher_mode/navigation/engineering_v34_r44_active_p95_capture_candidate/policy_candidate/worker.py`：本轮实际 Actor。共享 observation 实现在 `teacher_mode/policy/observation.py`。
- `multifloor_demo/teacher_mode/simulation/native/current/`：本轮实际加载的 R31E actuator 对应源码与构建文件。
- `multifloor_demo/slam/ros2_ws/src/`：本轮实际加载的 R33 修改版 FAST-LIVO2；没有同时携带旧 Demo FAST-LIVO2。
- `multifloor_demo/navigation/ros2_ws/src/`：本轮 SCAN `protected_baseline` 主实现。消息与轨迹依赖位于 `scan_multifloor/ros2_ws/src/planner/`，Vikit 依赖位于 `slam5_navigation/ros2_ws/src/`。

## 实际链路与边界

```text
Gazebo LiDAR / IMU / 相机 → ROS2 传感器接入 → FAST-LIVO2
  → 已知场景地图辅助匹配、有效修正准入 → SCAN / 46 区任务
  → 路径跟踪、停车保护、进程通信 → CPU Teacher Actor
  → native 50 Hz 策略 / 200 Hz 关节执行 → Gazebo
```

高层反馈使用传感器 SLAM/点云/IMU；已知场景地图辅助匹配仍存在，因此不是独立传感器建图验证。低层 Teacher 的 247 维输入中含 232 维 privileged 仿真状态/高度信息和 15 维命令与上一动作输入，不能称为全链路无真值。

当次直接失败链是可接纳定位修正过期、控制保护保持零命令、目标原 90 秒期限超时。射线 P95 残差为什么超过门槛仍未归因。归档修复得到本轮证据，不能把它等同于导航成功。后续未集成的 GPU 观察诊断候选没有收入；现有 Gazebo GPU LiDAR 的配置与依赖声明保留。

## 快照整理与依赖缺口

源项目保持原样。本快照只整理路径、私人标识与发布说明：外部实际源码放入可读相对布局，私人会话/消息标识、机器路径、来源哈希清单、审批材料和权重路径删除或改为外部依赖占位。控制算法、数值门槛和实验失败结论没有为公开展示而放宽。

这是学习材料，不能直接复现原运行。模型权重、机器人 DAE 资产、安装库、原始运行记录、原冻结与运行授权 metadata 均未提供；空的来源/授权字段不能通过原 preflight。未经另行配置和验证，不要将这里的启动脚本当作已验收的一键演示。

所需外部环境包括 ROS2 Jazzy、Gazebo Harmonic（gz-sim8 / gz-sensors8 / gz-rendering）、Ogre2、PyTorch 与对应机器人/策略资源。文件在源码目录内不等于属于源材料：所有 `.so`、缓存、build/install、数据与旧版本重复均未收入。

本次发布只做了文件范围、敏感信息、引用与 Git 对象核验；没有重新运行构建、测试、仿真、GPU/DSH 诊断或训练。实验报告引用既有 C44 封存证据。

## 许可与来源

保留各包原有版权、许可证、NOTICE 和文件头；新增完整 GPLv3 文本用于 Vikit 的现有许可声明。FAST-LIVO2 包声明 GPLv2，SCAN ROS2 包声明 Apache-2.0，Vikit 声明 GPLv3；Go2 描述/仿真包仍有原 TODO 许可声明。没有用一个新统一许可证覆盖这些材料，也不声称已完成所有组合许可问题的判断。

原公开来源包括 [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)、[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)、[Go2 simulation](https://github.com/khaledgabr77/unitree_go2_ros2)。本轮修改版的外部目录名仅说明实际版本来源，个人报告、审批、哈希清单和会话记录没有发布。

公开整理的 Python/C++/模型模板及调整链接的上游文档已加入带日期的修改说明。JSON 的修改仅涉及私人标识、路径和来源/授权元信息；数值门槛及几何数据保留。原始来源绑定未随快照公开，不能用空元信息绕过真实运行门。
