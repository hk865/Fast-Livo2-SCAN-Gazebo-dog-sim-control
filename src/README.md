# 按职责组织的源码

| 模块 | 职责 |
| --- | --- |
| [bringup](bringup/README.md) | 唯一总入口、配置、生命周期 |
| [common](common/README.md) | 公共通信、协议、时钟、状态传输 |
| [localization](localization/README.md) | 传感器、FAST-LIVO2、全局匹配 |
| [planning](planning/README.md) | SCAN、参考、曲线准入 |
| [control](control/README.md) | 跟踪、安全、停车、Student |
| [simulation](simulation/README.md) | 原Go2世界生成、Gazebo与关节力插件 |
| [sdk](sdk/README.md) | 原生消息、Vikit和Livox依赖 |

根目录没有平铺R系列Python。历史编号仍可能出现在兼容消息schema和内部状态名中，防止结构整理改变线上协议；它们不是旧run/源码哈希启动批准。
