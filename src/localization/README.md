# 感知、FAST-LIVO2与全局定位

传感器先完成安装/时间适配；FAST-LIVO2产生局部状态；同源场景匹配组成导航纠偏变换并发布代次。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| Gazebo IMU/LiDAR/RGB经ROS桥 | Imu / PointCloud2 / Image | relay与FAST-LIVO2 |
| raw LIO位姿/扫描 | camera_init、原源戳 | 同源适配与解析场景matcher |
| 导航位姿/点云/metadata | body_odom、registered cloud、localization_bundle | SCAN监督器与控制器 |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`frontend/、sensors/、localization_runtime.py`。
- 子入口：`launch/slam.launch.py`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`config/settings.json、fastlivo.yaml、camera.yaml`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
