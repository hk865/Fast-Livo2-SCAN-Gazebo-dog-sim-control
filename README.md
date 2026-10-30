# FAST-LIVO2 / Building / SCAN / Go2

本分支包含真实FAST-LIVO2前端、异步CPU后端、多楼层Building地图、PCT/SCAN规划、Student控制与Gazebo执行模块的完整源码。基于原仓库main的`6401e5fe`继续开发；只发布到本功能分支。

**已知配置出生先验下，85.70m真实SLAM多楼层导航完成一轮完整验收。** 实际回环、运动中短LiDAR断流恢复及原10Hz/5秒/51点停车也有独立通过轮。它不证明粗范围初始化或多轮稳定性；历史失败不计为通过。摘要与指标口径见[验收摘要](docs/Building后端验收摘要.md)，模块关系见[Building模块说明](docs/Building模块说明.md)。

## 构建与运行

环境：Ubuntu 24.04、ROS2 Jazzy、Gazebo Harmonic、Docker与NVIDIA Container Toolkit。数值后端及Student推理使用CPU；GPU用于仿真传感器渲染。完整依赖构建路径保留于Dockerfile，只从官方上游获取源码并构建到项目私有安装前缀。

```bash
docker build --build-arg BUILD_JOBS=2 -t fastlivo2-building:local .
# 在有使用授权的前提下，本地补齐下文两项资产后：
NAVIGATION_IMAGE=fastlivo2-building:local ./build_building.sh
NAVIGATION_IMAGE=fastlivo2-building:local ./run_building.sh --check
NAVIGATION_IMAGE=fastlivo2-building:local ./run_building.sh --mode slam --case stairs_short
NAVIGATION_IMAGE=fastlivo2-building:local ./run_building.sh --mode slam --case multi_floor2
```

本机也可复用已经配置的`go2-navigation:local`依赖镜像。`build_building.sh`只从镜像复制可重定位安装前缀，再编译本工程源码；不读取旧工程路径。Dockerfile新增PCT CLI及原生后端图优化器构建步骤。完整Docker冷构建与导航验收是不同检查，不能把构建成功当作导航通过。

公开分支**不含以下再分发条款尚未明确的文件**，它们仍保留在开发者本机：

| 本地补充路径 | 当前接口与影响 |
| --- | --- |
| `models/student_model_4999_deterministic_jit.pt` | Student45、GRU256、12动作；缺失时不能运行控制链 |
| `assets/meshes/L1_4D_Lidar.dae` | 实测SDF同时引用其视觉与碰撞几何；不以替代网格冒充原验收条件 |

取得相应授权后放入以上路径。`--check`会列出缺项，不能将公开源码检出直接称为开箱运行。模型接口见[models](models/README.md)，资产与适用许可见[来源说明](docs/资产与来源.md)。Building网格、PCD、派生几何/高程和路线已随源码提供并保留对应上游许可。

`start.py`是当前Building唯一入口，必须显式指定`--mode slam`或`--mode truth`。truth模式采用真值定位，专供对照。每轮写入新的`runs/<label>`目录，同名拒绝覆盖。源码分支排除原始实验日志、构建产物、录像和个人环境数据；不删除开发者本地记录。

```bash
python3 -B tools/validation/score_building_run.py runs/<label>
python3 -B tools/validation/score_slam_backend.py runs/<label>
# 已完成轮的明确标注回放，需numpy/scipy/Pillow/VTK/ffmpeg及OpenGL：
python3 -B tools/demo/render_navigation_demo.py --project . --run runs/<label> --output runs/demo.mp4
```

## 实现与边界

- 外部XYZ/RGB PCD无需采集位姿链。Building当前采用配置出生位置/朝向先验的局部地图匹配；未完成粗范围多候选时序初始化、一般失跟/绑架恢复及无初值全图重定位。
- 前端成功LIO/VIO更新后提交同源时间戳的自有LiDAR快照，预分配有界队列。重配准、历史子图保存与图优化异步运行；单一`building_runtime`应用修正，局部odom连续，不回写ESIKF。
- 已接入历史SE3位姿图及历史点云几何回环。已知地图模式下图优化更新历史位姿与子图，在线`map→odom`仍来自外部地图配准，不叠第二套图修正。真正紧耦合滑窗/fixed-lag、IMU联合因子和边缘化未实现。
- 200Hz物理、50Hz策略、300ms来源/命令新鲜度停车、200ms原生响应截止及原停车判据保留。复制、后台处理、源龄和应用延迟分别统计，现有实测不构成通用硬实时保证。
- 在线定位不读取GT；误差按固定坐标与原时间戳离线评分，无事后轨迹拟合。仿真原有特权COM速度内环仍保留并披露。
- 当前Building运行和构建入口不做内容哈希校验，不添加源码/模型版本运行绑定。保留的旧三层兼容模块不属于本分支验收入口。

[后端说明](src/localization/backend/README.md) · [源码模块](src/README.md) · [第三方声明](THIRD_PARTY_NOTICES.md)
