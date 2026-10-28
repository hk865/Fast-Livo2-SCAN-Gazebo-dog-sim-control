# 原Go2三层导航仿真

从根assets生成world；Actor真实bind/listen就绪后才启动Gazebo；关节力插件是唯一执行者。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| 原SDF/URDF/mesh与配置 | 静态资产 | world/prepare.py生成本run世界 |
| Actor socket ready | 模型加载完成、bind/listen | 放行Gazebo |
| 12关节目标＋协议字段 | 16 float64、20ms策略周期 | actuator施加关节PD力矩，反馈64状态 |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`world/prepare.py、actuators/teacher_actuator.cpp`。
- 子入口：`launch/simulation.launch.py`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`config/settings.json；根assets/`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
