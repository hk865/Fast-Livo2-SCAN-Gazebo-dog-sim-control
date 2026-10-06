# FAST-LIVO2 / SCAN / Gazebo Go2 — 当前代码与文字报告

2026-10-06检查点，独立分支`codex/v25-code-reports-only-20261006`。源码来自已测V25检查点`2c3ea68714203290fdfc3db7161d43ea34ba4cc5`；本分支使用无父提交的独立历史，只含源码、测试/分析脚本、配置契约、必要静态机器人网格与文字报告。

[当前实验结论](docs/CURRENT_V25_TEXT_REPORT.md)、[数据排除与使用范围](docs/CODE_REPORTS_ONLY.md)、[导出源码清单](docs/CODE_REPORTS_ONLY_MANIFEST.json)。旧报告保持原文，它们引用的日志、图像与执行收据只保留在原工作区/历史分支，不属于本分支附件。

## 当前实测结论

冻结Teacher的接口与指定基础运动、原前九区有限路线/停车已通过。V25实际完整46区任务到达11区，第12区原90秒超时；原22门为10通过、4失败、8未验证。SCAN没有跳过第10区：本轮48.98秒到达、驻留0.43秒。转向后SLAM高度相对同时间仿真机身固定坐标对照偏离约-0.20m，仿真机身高度稳定；SCAN连续规划失败触发紧停，但缺碰撞体素证据，尚不能认定高度偏离直接导致该故障。

完整Sim2Sim、完整46区导航、全200Hz执行器/完整PI与SCAN几何回放和真机未验证/未通过。导航用实际SLAM/IMU/点云/SCAN，Actor仍232维仿真特权输入。仅仿真，不重新训练、不操作真机。三层由坡道连接。曲率前馈与限速开启；IMU/LIO/VIO跨帧解耦暂缓。

## 源码与重建

`multifloor_demo/teacher_mode/navigation/corridor_tracking_v25_terrain_event`保存当前控制候选；`navigation/pipeline_v19/slam_ws/src`保存实际SLAM输入流水线版本。`multifloor_demo/camera_mode`保留通过的相机模式，`scan_multifloor`、`slam5_navigation`、`go2_sim_control`与`tools/vendor`提供所需源码和静态资产。源代码、控制数值与配置按原blob保存，不包含build或权重。

冻结权重SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。权重另行提供；必须重新构建依赖与源文件、生成当前机器新的来源门并重新仿真验证。旧执行收据/PASS gate已排除，不能从本分支继承实际通过。部分历史测试与分析脚本需要另行提供数据输入，不是开箱即用的数据回放包。构建入口见[tools/README.md](tools/README.md)。

## 单分支获取

```bash
git clone --single-branch --branch codex/v25-code-reports-only-20261006 \
  git@github.com:hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control.git
```

上游各源码与资产保留各自来源及现有许可证，本文没有统一变更其许可。静态网格属于运行资产，不是传感器采集数据。历史分支仍用于追溯。

[回环与局部匹配接线审计](multifloor_demo/teacher_mode/docs/LOOP_HEIGHT_LOCAL_ADAPTATION_AUDIT_20261006.md)列出当前阶段边界、快照接口和坐标版本方案；后端尚未实现。
