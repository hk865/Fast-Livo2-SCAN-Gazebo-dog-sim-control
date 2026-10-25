# 整理目录的同策略复现

本目录新增根 `candidate/` 作为 formal22 的路径适配入口。`releases/formal22/runtime/candidate/` 保留原公开快照；两者不是两套同时启动的控制器。旧成功部署 `536c9d3` 的结果不授予新目录通过。

## 本次真实依赖边界

- Python 控制、IMU 接收、SCAN 关联、定位辅助及退出逻辑从本目录 `candidate/` 加载；仅修正目录、来源身份与管理配置，不改变 formal22 策略或数值参数。
- native571 的48份源码从 `releases/formal22/native/src/` 重新编译；SCAN 的 plan_manage 从 `releases/formal22/scan/` 重新编译。构建输出保存在各自 `.local_build/`、`.local_install/`，不上传 ELF。
- SCAN 固定规划库、消息 SDK 和原 actuator 为显式固定依赖；本次复制其已核对字节，不能声称这些第三方库全部从公开包重新构建。
- Livox 与 vikit 运行库、ROS Jazzy/Gazebo、冻结 Student/Teacher 模型及 torch Python 使用原机已安装依赖。新 native/core 与 SCAN 顶层 ELF 不得回落旧工作区。模型不上传，须合法提供并匹配记录的 SHA。
- 仅单独指定所需 SDK 的 lib、消息 Python/ament 搜索路径，不 source 旧 SLAM/SCAN 工作区的完整 install。生成的 native `setup.bash` 含构建时 underlay 记录，因此实际入口使用新 `local_setup.bash`，在 ROS 基础环境之后显式加载。
- 输出池及持久保留账本沿用原机管理边界；账本不清空，旧数据不删除。它们是输出/资源治理依赖，不是旧业务代码回落。

## 实际管理入口

本次仍使用原机只读的 `navigation_acceptance_round27_20261024/deployment/round27_v3.py` 与 `detached_entry_v3.py`，依次执行 propose、独立核对、apply、本地冻结提交、prepare、execute、verify-loaded、closure。它们是共享管理基础设施；本包不伪装成任意机器直接一条命令可跑的自包含发行版。

直接的业务入口是 `candidate/run.py --profile <当轮PROFILE.json> --label <唯一轮次> --domain 231 --real-time-factor 1.0 --surface-validity 0 --cpu-python <冻结Actor解释器> --run-storage-root <受管输出池>`。须先经上述管理流程生成新的具体资源/时间证明，再用 `--prepare-only` 与原受控 execute；不要绕过独占、来源检查或重用已消费的配置。

仍保持170,000,000,000字节保留上限、10GB单轮额度及10GB额外预留、2400秒运行与300秒收尾。R99关闭，不启动训练或实机。不热改运行版本，不沿用旧轮通过。

## 当前复现记录

已完成11个准备/修正标签，其中06–11有execute尝试；这不等于11次完整仿真。01/02是启动前路径/资产遗漏，03–05是来源闭包与审查冻结，06缺Python包路径、07 helper身份不一致、08 ACK契约身份未迁移、09路线来源SHA未迁移、10静态几何证书绑定旧world。上述失败及未启动状态全部保留；修正的是打包和身份，不记作导航算法收益。见[逐标签实录](../reproduction/PUBLICATION_REPRODUCTION_HISTORY.json)。

最终11实际部署`ecb62ad`，实现`e4b120d`，prepare52项/加载91项通过。13.865 SIM接纳任务，34.913044m到终点，389.015 SIM停车失败；首次T0=388.790不重置，完整5秒未完成。在线SLAM速度0.080310m/s越过原0.08m/s界，同期native实际速度最高0.084245m/s。动态业务停走有证据，但最终dynamic/RGB终验未形成。六退出、Actor19582样本CRC、pipeline174583包drain通过；worker EOF fault保留。见[本轮完整报告](../reproduction/results/published_repro_11/RESULT.zh-CN.md)。未继续调参数/改策略或自动再跑。
整理目录还将原 R33/V32 历史来源清单与当前执行来源分开：只对已验证 published-native571 + R85 分支，不再要求旧 V32/R73 的390条历史输入充当当前运行件，历史清单原文保留。当前候选、几何、native、SCAN、路线/PID及运行配置仍完整绑定和归档；其他入口保留原逻辑。新增7个引用对应6个唯一历史JSON保留原始SHA，单独存放 exact_source_inputs，未覆盖公开导出或把历史FAIL改成PASS。

## 文档与外部源码片段

`dependencies/exact_source_inputs/` 保留来源校验实际读取的原始开源文件字节，包含上游 README 片段。该 README 的三个 QUALITY_DECLARATION 相对链接依赖原上游完整仓库布局，本包没有复制那些目录；静态工具会单独列出这类未提供上下文的链接，不改写原文件摘要，也不把它们当作本项目操作文档。项目 README、模块/数据流及复现说明的本地链接仍必须通过。

宿主软件版本实录见 [DEPENDENCY_VERSIONS.json](DEPENDENCY_VERSIONS.json)。没有执行新的异机安装或 Docker 镜像验收。
