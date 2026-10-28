# 导航跟踪、安全、停车与Student

SLAM/IMU与已接纳路径产生有界速度命令；bridge检查来源；Student命令PID/GRU输出12关节目标，actual-zero ACK驱动固定停车窗。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| body_odom、IMU、路径、目标 | 实际源戳和同代参考 | cascade/安全/terminal owner |
| 有界速度及来源包 | vx/vy m/s、yaw rad/s | bridge→Student命令PID |
| 原生64状态与当前命令 | 实际obs45/hidden、12动作 | 16 float64响应/下一请求ACK |
| 实际zero ACK与固定目标 | 固定T0、51格pose、5s状态 | 停车与独立验收 |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`controller.py、tracking/、safety/、parking/、mission/、policy/`。
- 子入口：`launch/control.launch.py`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`config/settings.json`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
