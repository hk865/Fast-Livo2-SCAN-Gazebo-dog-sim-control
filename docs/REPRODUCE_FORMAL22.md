# formal22构建、运行入口与复现范围

## 版本与实跑结果

|目录/版本|用途|实际验证|
|---|---|---|
|原机`runtime_compact_formal_22`，候选0f3566 / 部署536c9d3|原成功部署|新策略单次F1→F3、动态最终核验、固定5秒停车、RGB检查通过；独立19/19；原46项未通过|
|`releases/formal22/runtime/candidate/`|原成功源码的公开路径规范化快照|原/公开SHA分别记录；最初脱敏运算符错误已修复；不继承原PASS|
|根`candidate/`，部署ecb62ad|整理目录实际执行入口|新native/SCAN重建并加载91项通过；全路线34.913044m、严格停车失败，见[本轮结果](../reproduction/results/published_repro_11/RESULT.zh-CN.md)|
|根其他旧工程/网页/CHAMP入口|原main历史|不是本轮Student formal22运行入口|

根源码与本轮业务参数冻结在`ecb62adb4e55513e6a11f23f23803838b19e09f6`；后续发布提交追加说明和证据，不把文档提交SHA误称本轮实际加载SHA。模型、ELF、生成库和大原始流没有上传。本轮同机重跑已授权并执行；没有异机安装、Docker实测、实机或训练。

## 依赖与实际重建

宿主是Linux、ROS2 Jazzy、Gazebo Harmonic、Fast DDS/rmw_fastrtps、colcon/CMake、Eigen/PCL/OpenCV/Sophus和Python NumPy/SciPy/rclpy。安装包实录见[版本文件](DEPENDENCY_VERSIONS.json)，查询未找到的包不填造版本。模型和上游来源/许可见[release说明](../releases/formal22/README.md)、[资产说明](../reproduction/assets/UPSTREAM_RESOURCES.md)。本阶段不新建容器工程。

- native571的48份源码从本目录`releases/formal22/native/src/`重编译。实际native ELF SHA `f5b82d9383a3ae5680f1a57d1e33c650bd4aa8dc459e8be8a28d727b9fcc2ca8`，core `e0f0a80a81f94e0b2263b1bf5569b0fc942adbc97f160352bfe2f99f52a811bd`。上游状态协方差新候选未加载。
- SCAN plan_manage从本目录源码重编译，实际顶层ELF SHA `3a44129cb18695437d2131316b656933d611297b6e3743c355582116b8cd2eeb`。固定规划库、消息SDK使用按SHA显式复制的依赖，不能称整个规划依赖树都重新从源构建。
- Livox/vikit SDK、ROS/Gazebo系统安装、固定actuator与Student推理环境仍来自原机合法已装依赖；新顶层native/SCAN没有回落旧构建。详见[实际依赖边界](PUBLISHED_DIRECTORY_REPRODUCTION.md)。
- 冻结Student45 TorchScript SHA `d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185`，原独立torch解释器实际为`/home/hyh001/isaacsim/kit/python/bin/python3`；它不是仅靠system Python即可替代的依赖。原Teacher checkpoint仍有历史绑定，不是本轮实际Actor。

实际native重新构建使用ROS基础环境和独立build/install目录；可据依赖完成本地重建，例如：

```bash
source /opt/ros/jazzy/setup.bash
cd releases/formal22/native
colcon --log-base .local_log build --base-paths src --packages-select fast_livo2_core fast_livo2_ros --executor sequential --build-base .local_build --install-base .local_install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

本轮[构建记录及编译参数](../reproduction/build_records/README.md)另存。命令本身不保证外部SDK齐全或生成同一ELF；须对照构建/加载回执核对编译输入和参数。SCAN plan_manage还需要固定规划库、消息头与type support的同版输入，不可只编一个包并声称全部来源完备。本仓库不上传这些二进制，不以相近库或重命名文件替代身份检查。

实际运行source ROS基础→新native `local_setup.bash`→新SCAN环境→新native `local_setup.bash`，仅显式加入必要Livox/vikit/消息SDK的lib、Python、ament路径。生成native完整`setup.bash`含旧underlay记录，实际入口不source它。模型权重、资源管理账本和共享管理入口是明确外部依赖；不声称完全自包含。

## 实际受控入口与当轮配置

整理目录仍使用原机只读管理文件`navigation_acceptance_round27_20261024/deployment/round27_v3.py`及`detached_entry_v3.py`，真实序列：

```text
round27_v3.py propose --spec FINAL_SPEC.json --output PROPOSAL.json
[核对这份具体代码/配置，生成并冻结独立review SHA]
round27_v3.py apply --proposal PROPOSAL.json --review REVIEW.json --review-sha256 <SHA>
[提交管理文件，冻结具体部署SHA]
round27_v3.py prepare --trial <新的唯一trial>
detached_entry_v3.py --trial <同一trial> --operation execute
[核对实际加载、同owner及递增world clock]
round27_v3.py closure --trial <同一trial>
```

业务入口为`candidate/run.py --profile <当轮PROFILE.json> --label <唯一label> --domain 231 --real-time-factor 1.0 --surface-validity 0 --cpu-python <冻结Actor解释器> --run-storage-root <受管输出池>`；prepare/执行由管理入口调用，不绕过来源/域/锁保护。这里只展示真实入口形式，不把旧profile或历史行政窗口当新启动许可，也不提供会静默落回旧目录的万能命令。

本轮运行目录`20261025T073601Z_closed_loop_R85_published_repro_11_same_formal22_fb3d`及小证据见[索引](../reproduction/results/published_repro_11/EVIDENCE_INDEX.json)。其一次窗口已消费。以后每次都要重新核对当前源码/构建/模型、单一owner、锁/域/CPU/GPU/可用空间，并生成新的有界资源/时间证明；170,000,000,000B保留上限、单轮10GB+额外10GB、2400秒运行+300秒收尾不变，R99 OFF。本阶段不继续自动仿真。

## 离线检查与结果读法

```bash
python3 -B tools/formal22_manifest_check.py
```

检查原公开快照清单、Python语法及路径规范化的非字符串AST、JSON和项目文档链接；不启动ROS/Gazebo，不替代新根目录的实际加载/物理验收。`exact_source_inputs`中保留上游原字节的README有三个未复制上下文的相对链接，工具单独列出，不修改来源SHA。根执行代码和精选证据另以当前提交、运行加载及索引绑定，不能用旧快照清单声称当前全部文件已运行。

原536c9d3结果是真实单轮PASS；新ecb62ad结果是实际全路线到达而停车FAIL。后者task/parking false，dynamic最终结果None，RGB终验None；不是“没有数据”，也不是原46通过。Actor归档CRC/进程清理通过与业务验收分开，EOF worker fault保留。原始大流不在Git，精选小日志不是完整原始备份或所有历史的可重放证明。

outer navigation使用真实传感器/SLAM，未用机器人真值；native传感代理和内层PID仍有特权仿真反馈。Actor实际45维输入与归档247维特权原始观测不是同一数组语义。不声称全链非特权、Sim2Sim通过或实机安全。
