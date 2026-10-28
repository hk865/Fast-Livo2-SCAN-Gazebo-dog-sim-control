# SCAN规划与路径准入

监督器独占SCAN子进程；参考随坐标代次提交，候选B样条经静态几何证书与实时来源核对后交控制。

## 输入与输出

| 输入 | 数据/语义 | 输出或消费者 |
| --- | --- | --- |
| 纠偏位姿/点云与全局路线 | 同源键、generation、epoch | SCAN局部地图/路径搜索 |
| B样条/元数据与静态box | 控制点/knots/有限支撑约束 | admission几何worker与原owner提交 |
| 已接纳路径及参考 | 有效代次/来源 | control空间跟踪 |

名义频率、坐标与因果关系见[模块数据流](../../docs/模块数据流.md)，源戳不能用保存时间代替。

## 核心代码与配置

- 入口：`scan/、scan_epoch_supervisor.py、reference/、admission/`。
- 子入口：`launch/scan.launch.py`；日常使用根`start.py`，不重复启动已有模块。
- 参数：`config/settings.json 与原生scan_planner配置`；保留原控制、freshness、几何与停车阈值。

## 使用与调试

先在根执行`python3 start.py --check`查看完整缺项；构建按[移植说明](../../docs/移植构建运行.md)。日志、当前状态和各进程退出记录写入当前`runs/`目录。诊断实现位于根`experiments/`，正常入口不加载它们。具体故障先沿本模块输入来源、时间/坐标代次和输出消费者检查，不能把所有错误归因于DDS。

本轮移植验收状态统一见[验证状态](../../docs/验证状态.md)，构建/组件测试不等于导航通过。
