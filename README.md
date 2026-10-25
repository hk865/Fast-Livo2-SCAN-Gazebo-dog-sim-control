# FAST-LIVO2 / SCAN / Gazebo Go2 — formal22阶段基线

2026-10-25：**当前选定策略下，R85单次F1→F3完整仿真任务通过**。实际部署`536c9d3f4773ef43edef48992c598ccf2fc3f021`，业务候选`0f35662822925a68de9a659558cc5073845b5b87`。真实动态停止/恢复、首次T0起固定5秒停车、当前RGB地图检查通过，独立复核19/19。任务在350.930仿真秒完成，路线高水位34.9079m（不是累计行走里程）。

这是新质量/地形策略下的一次完整任务通过；**原46区域任务未声明通过，间歇时序故障未证明全部根治，也不是实机或所有场景安全证明**。原质量5%限制和车体导数保证并不等价。真实协方差导出候选没有加载；不新增回环、位姿图或全局重定位。

- [正式结果与边界](releases/formal22/evidence/RESULT.zh-CN.md) · [独立19项复核](releases/formal22/evidence/FORMAL22_ACCEPTANCE_INDEPENDENT_REVIEW_FINAL.zh-CN.md)
- [目录和模块职责](docs/FORMAL22_MODULES.md) · [数据流与进程图](docs/FORMAL22_DATAFLOW.md)
- [实际入口、构建和复现边界](docs/REPRODUCE_FORMAL22.md) · [发布来源与许可](releases/formal22/README.md)
- [历史记录入口](docs/EXPERIMENT_HISTORY.md) · [原2026-10-06报告](docs/EXPERIMENT_REPORT_20261006.md)

## 当前版本范围

|项目|本轮真实结果|
|---|---|
|任务|R85单次自动F1→F3，启动、真实感知定位、SCAN规划、跟踪、动态停走、终点、固定5秒停车及本轮RGB保存通过|
|动态事件|stop177.565s → resume203.625s；恢复0.593909m；最终独立物理核验通过|
|停车|首次T0=344.880s，固定结束349.880s，不重置窗口；原6项停车判据通过|
|定位质量|沿用既有robust fit/残差/覆盖/退化/歧义/两帧确认；取消独立5%前景整帧否决，以非前景总体计算表面误差；非旧策略等价|
|连续地形判定|同一有限地面完整支撑曲线及脚印时以真实地形坡度判定，保留原0.35rad、残差/净空/碰撞；非旧bodyZ/XY导数间接界等价|
|结束|自然返回0，6项进程/锁/ROS域退出检查、Actor18295样本CRC及pipeline提交闭合通过|

## 目录

`releases/formal22/`是本次实际冻结版本的源码/配置和小型证据导出；`multifloor_demo/`、`scan_multifloor/`、`slam5_navigation/`、`go2_sim_control/`、`tools/`等原目录保留2026-10-06 main历史，不自动代表formal22入口。不要把旧网页/CHAMP/Teacher命令当成本轮Student入口。

导出包含实际runtime checkout、native571源码以及真实scope中的文本依赖；兼容/历史绑定文件随冻结包保留，不意味着每个模块启用。实际加载和触发以本轮回执为准。权重、ELF、构建输出、点云/bag/大原始流未上传，模型/二进制保留SHA和依赖引用。公开导出规范化了本机路径等私人信息，原部署SHA与公开文件SHA分别登记；**尚未从此脱敏目录重新运行仿真，不能继承新机器的运行PASS**。

不启动ROS或仿真即可核查导出文件：

```bash
python3 -B tools/formal22_manifest_check.py
```

真实运行仍需冻结模型、本机依赖构建、来源/配置重绑定和当次资源/时间证明；本轮旧行政窗口不能复用。详见复现说明。模型是冻结Student45推理，外层导航不用机器人真值；native传感代理与内层PID仍含特权仿真反馈，不声称全链非特权或Sim2Sim通过。仅仿真，不连接实机、不训练。

原main历史保留。源自上游的代码/资产沿用各自许可，本仓库不重新许可模型或第三方内容；[旧README](https://github.com/hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control/blob/5237b8865d1f788b88f86fe21b72852db3bedb03/README.md)及历史失败不改写。
