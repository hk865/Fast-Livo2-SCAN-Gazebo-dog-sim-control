# 代码结构文档索引

本目录是对 `Fast-Livo2-SCAN-Gazebo-dog-sim-control`(`study/go2-core-20261010` @ `2331b93`)的架构解读,分**总体**与**分模块**两层。

## 从这里开始

👉 **[00-overview-dataflow.md](00-overview-dataflow.md) —— 总体数据流与模块划分**

包含:系统一句话概括、9 个模块划分表、总体数据流图、进程拓扑图、一次控制周期时序图、话题与文件交接点、代码结构三平面总览、模块依赖图、本轮失败链在图上的位置、三个易踩的坑、建议阅读顺序。

## 分模块文档

| # | 文档 | 模块 | 覆盖路径 |
|---|---|---|---|
| 01 | [01-simulation-and-robot.md](01-simulation-and-robot.md) | 仿真世界、机器人本体与传感器接入 | `go2_sim_control/`、`multifloor_demo/simulation/` |
| 02 | [02-slam-frontend.md](02-slam-frontend.md) | SLAM 前端(FAST-LIVO2 + Vikit) | `multifloor_demo/slam/`、`slam5_navigation/` |
| 03 | [03-known-scene-localization.md](03-known-scene-localization.md) | 已知场景定位修正与时空关联 | `ENG/known_scene_*.py` |
| 04 | [04-scan-planning.md](04-scan-planning.md) | SCAN 规划与轨迹优化 | `multifloor_demo/navigation/ros2_ws/`、`scan_multifloor/` |
| 05 | [05-mission-and-route.md](05-mission-and-route.md) | 任务状态机、路线分区与 46 区协调 | `multifloor_demo/mission/`、`ENG/mission46_*.py` |
| 06 | [06-high-level-control.md](06-high-level-control.md) | 高层控制、轨迹跟踪与安全保护 | `ENG/controller.py` 等 |
| 07 | [07-teacher-bridge-and-policy.md](07-teacher-bridge-and-policy.md) | Teacher IPC 桥接、策略观测与执行 | `ENG/bridge.py`、`ENG/worker.py` |
| 08 | [08-native-actuator-and-gait.md](08-native-actuator-and-gait.md) | 原生执行器、步态与运动学 | `teacher_mode/simulation/native/` |
| 09 | [09-runtime-orchestration-and-archive.md](09-runtime-orchestration-and-archive.md) | 运行编排、冻结契约与归档证据 | `ENG/run.py`、`ENG/evidence_runtime_v32/` |

## 改造建议(与结构文档分开)

👉 **[10-gates-and-glue-refactoring.md](10-gates-and-glue-refactoring.md)**

不是结构描述,而是**判断与建议**:胶水代码分层、62 条运动门禁全表、三个系统性陷阱(自锁回环 / `execution_frozen` 并行停车 / 四处 300 ms)、安全分档、以及"声明式门禁表 + 显式端口 + 显式适配器"的改造样板与执行顺序。

> `ENG/` = `multifloor_demo/teacher_mode/navigation/engineering_v34_r44_active_p95_capture_candidate/`

## 每份分模块文档的统一结构

1. 一句话职责
2. 在总体数据流中的位置(含 Mermaid 图)
3. 内部子模块划分(表格)
4. 数据流与接口(Mermaid 图 + 接口表)
5. 关键流程与时序(Mermaid 图 + 逐步讲解)
6. 关键数据结构与配置
7. 代码结构讲解(目录树 + 逐文件说明)
8. 与其他模块的边界
9. 已知限制与失败点
10. 建议阅读路线

## 阅读建议

- **想理解数据怎么流**:按 `00 → 02 → 03 → 04 → 06 → 07 → 08 → 05 → 01 → 09` 的顺序。
- **想快速知道本轮跑到哪**:先读仓库自带的 [`docs/C44_EXPERIMENT.md`](../C44_EXPERIMENT.md) 与 [`docs/PROGRESS.md`](../PROGRESS.md)。
- **想直接跑起来**:本快照**不含**策略权重、DAE 资产、安装库,启动脚本不能当作已验收的一键演示,请先读 [`README.md`](../../README.md)。

## 图怎么渲染

所有图用 Mermaid 书写。GitHub 上可原生渲染;本地编辑器需支持 Mermaid 的插件(如 VS Code 的 Markdown Preview Mermaid Support)。

## 说明

- 全部内容基于 `2331b93` 这一份快照的源码实地阅读,关键论断标注了 `文件:行号`。
- 本仓库大量模块是**旁路观测/证据归档**(自述 `observer only`、`no actuator decision`),文档中已逐处区分"实际控制链路"与"仅记录模块",避免误读。
- **只读快照、未改源码**:`git status` 仅显示新增 `docs/architecture/`,HEAD 仍为 `2331b93`;未运行任何构建或仿真。
- **文档间交叉校验过**:所有相对链接可解析;所有 Mermaid 代码块结构合法(围栏闭合 + 图类型可识别);8 份模块文档均为统一的 10 节结构。
- **已知局限**:文档描述的是**这一份快照**。其中若干模块在本快照内**无法真正启动**(缺少冻结见证文件、Linux `/proc` 依赖、外部权重与若干 workspace 契约 JSON)。这类限制在各模块文档的第 9 节逐条标注,不要在别处默认它们可运行。
