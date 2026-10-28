# 公共通信与时钟

提供ROS/SDK环境、patched DDS选择、IPC/命令文件、同源协议和时钟协作；不产生运动授权。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| ROS源戳、receipt与坐标generation | 协议字段/时间基础 | 由原安全owner判断，不在公共层制造运动许可 |
| 命令/ACK/活状态文件 | 原子JSON和有界队列 | bridge、Actor和controller |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`config.py、runtime_environment.py、communication/、time/`。
- 子入口：`由总入口准备`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`config/settings.json；communication/dds_transport.xml`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
