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

`published_repro_01` 在启动前因旧 SLAM setup 路径缺失退出；`published_repro_02` 在启动前因导出缺少生成资产输入退出。两次均未启动 ROS/Gazebo/Actor，原日志保留。路径与必要资产的打包修正不记为导航收益；新目录实际加载、动态事件、完整路线、固定5秒停车、RGB和退出/CRC结果另行记录。

整理目录还将原 R33/V32 历史来源清单与当前执行来源分开：只对已验证 published-native571 + R85 分支，不再要求旧 V32/R73 的390条历史输入充当当前运行件，历史清单原文保留。当前候选、几何、native、SCAN、路线/PID及运行配置仍完整绑定和归档；其他入口保留原逻辑。新增7个引用对应6个唯一历史JSON保留原始SHA，单独存放 exact_source_inputs，未覆盖公开导出或把历史FAIL改成PASS。
