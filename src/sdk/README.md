# 原生ROS消息与视觉/Livox依赖

提供scan_planner_msgs、Livox消息及驱动、CHAMP消息、Vikit视觉库，不是第二套导航入口。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| C++/ROS包源码 | CMake与package.xml | 本包install/ros |
| 官方Livox SDK2源 | 固定commit、公共ABI | Livox驱动/消息；仿真不启动硬件驱动 |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`各package.xml与CMakeLists.txt`。
- 子入口：`由build_support/build.py按依赖编译`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`各原生包配置`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
