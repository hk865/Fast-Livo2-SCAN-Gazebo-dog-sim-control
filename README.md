# Go2 三层场景闭环导航

这是一个由仿真传感器驱动的四足导航工程：FAST-LIVO2估计局部状态，已知场景匹配提供全局导航纠偏，SCAN规划B样条路径，分层PID与Student45/GRU策略把路径变成关节目标，Gazebo执行关节力矩并反馈。默认任务是原三层平台与坡道上的F1→F3。

源码按职责放在`src/`子模块中，只有一个普通启动入口。必要的Student权重、Go2/L1网格和原导航世界/路线已包含在本发行；公开再分发状态见[资产说明](docs/资产与来源.md)。本包不含GrandTour资产。Student和L1随仓库所有者明确指示发布，上游权利说明及尚待确认项继续保留，未另行许可第三方资产。

源码模块：[总编排](src/bringup/README.md) · [通信](src/common/README.md) · [定位](src/localization/README.md) · [规划](src/planning/README.md) · [控制](src/control/README.md) · [仿真](src/simulation/README.md) · [SDK](src/sdk/README.md)。

## 快速开始

需要Linux x86-64与可运行的Docker。Dockerfile默认从官方ROS镜像、Ubuntu/ROS/Gazebo软件源和官方PyTorch CPU源安装环境，再从源码构建全部原生组件。本次验收使用6个CPU额度与12GiB内存上限，构建默认两线程；具体资源与性能仍以本次验收记录为准。

```bash
mkdir -p runs
docker build --build-arg BUILD_JOBS=2 -t go2-navigation:local .
docker run --rm --cpus=6 --memory=12g --shm-size=2g go2-navigation:local --check
docker run --rm --cpus=6 --memory=12g --shm-size=2g \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e EGL_PLATFORM=surfaceless \
  --mount type=bind,src="$(pwd)/runs",dst=/workspace/navigation/runs \
  go2-navigation:local --resource-check
```

启动前先创建本地`runs`目录。`--check`只报告完整依赖缺项，不启动仿真；`--prepare-only`只生成原世界与参数；`--duration 60`限制墙钟运行时间，截断本身不表示任务成功。构建源连接问题与原生Linux构建步骤见[移植、构建与运行](docs/移植构建运行.md)。

动态障碍停走属于独立验收工具：普通入口不导入测试调度器。一次完整检查使用`tools/validation/run_navigation_check.py`，会移动唯一测试障碍、观察原停车规则，再关闭自己启动的任务。不要同时启动两套导航。

## 模块导航

| 模块 | 核心职责 |
| --- | --- |
| [bringup](src/bringup/README.md) | 总配置、唯一生命周期与总launch |
| [common](src/common/README.md) | 公共通信、时钟、协议及状态传输 |
| [localization](src/localization/README.md) | 传感器、FAST-LIVO2、场景匹配/纠偏 |
| [planning](src/planning/README.md) | SCAN、坐标代次、参考与曲线准入 |
| [control](src/control/README.md) | 跟踪、安全、停车、命令门控和Student |
| [simulation](src/simulation/README.md) | 原Go2世界、传感器桥与唯一关节力执行器 |
| [sdk](src/sdk/README.md) | Vikit、Livox和ROS消息包 |

第一次读实现可先看[模块数据流](docs/模块数据流.md)，再看[源码结构与阅读路线](docs/source_architecture.zh-CN.md)；原理、公式与PID历史详见[技术报告](docs/technical_report.zh-CN.md)，也可[下载PDF](docs/pdf/navigation_technical_report.zh-CN.pdf)。实验与诊断留在`experiments/`，不作为正常运行的必需导入。

## 验证范围

本轮新编译源码已完成一次F1→F3、动态停走和固定5s停车PASS，XY最大漂移2.407cm、yaw 0.01543rad；入口、观察器及15个launch组件均正常退出。最终Dockerfile也完成独立全新构建及镜像组件检查，范围见[验证状态](docs/验证状态.md)。原工程第26轮曾完成34.8826m的F1→F3、动态停走与固定5s停车，停车XY最大漂移3.0527cm、yaw 0.01202rad；这些是旧构建的历史证据，不能为本次新构建代证。停车漂移也不是全局定位绝对误差。

该工程面向Gazebo仿真，Actor仍使用明确列出的特权仿真反馈；未验证真机、未知环境、台阶、任意硬件或跨ABI二进制便携性。原FAST-LIVO2等组件保留各自许可证与来源，不能用一个总license覆盖全部资产。
