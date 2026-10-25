# formal22构建、运行入口与复现范围

## 先区分三个目录

1. **实际成功运行**：原机`navigation_acceptance_round27_20261024/worktrees/runtime_compact_formal_22`，部署536c9d3，native571/独立SCAN构建；原结果及SHA保存在本包。它在该环境真正运行过。
2. **本公开导出**：`releases/formal22/`，规范化了本机绝对路径/私人标识，按源/公开双SHA登记；本阶段没有在此目录重新构建或仿真。它是可审查源码基线，不把原机PASS自动转授给新目录。
3. **原main旧入口**：根目录`tools/`中旧v12/v18/v19及旧网页/CHAMP脚本保留历史，不能作为formal22启动命令。

## 依赖与构建

原环境为Linux、ROS2 Jazzy、Gazebo Harmonic、Fast DDS/rmw_fastrtps、colcon/CMake、C++依赖(Eigen/PCL/OpenCV/Sophus等)、Python NumPy/SciPy/rclpy，以及可用的冻结Student TorchScript推理环境。Actor实际使用独立Python/torch运行时，不能仅有system python便声称依赖齐全；源码/链接输入及实际ELF SHA以`deployment/SCAN_RUNTIME_CONTRACT.json`、SOURCE_EXPORT和加载回执为准。发布不包含模型、ELF和生成库，外部库须从合法来源按原配置构建，不能只改文件名通过身份检查。

native源码可在新工作区构建，例如（仅说明，没有在发布阶段执行）：

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/formal22_native_ws/src
cp -a releases/formal22/native/src/. /tmp/formal22_native_ws/src/
cd /tmp/formal22_native_ws
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

SCAN实际只修改/绑定部分源包，`scan/src/plan_manage`要与包内`dependencies/`及原仓库`scan_multifloor/`的同版共享源/消息依赖组合，不能只编这一个目录假装完整工作区。该轮实际复用了已绑定的traj_utils/bspline_opt/path_searching/plan_env及消息支持库；这些生成物未上传，需从对应源重建并重新冻结SHA。未知的外部源或许可不能以相近版本代替而继承验收。完整原构建身份见部署contract，未完成新目录整链移植验证是明确限制。

## 实际采用的原机启动序列

该轮使用原管理入口：

```text
round27_v3.py propose --spec FINAL_SPEC.json --output PROPOSAL.json
[对具体候选/四管理文件做独立配置核对]
round27_v3.py apply --proposal PROPOSAL.json --review REVIEW.json --review-sha256 <SHA>
[提交四管理文件并核对精确部署SHA]
round27_v3.py prepare --trial <one-new-trial>
detached_entry_v3.py --trial <one-new-trial> --operation execute
[实际加载SHA/同owner的world时钟递增]
round27_v3.py closure --trial <same-trial>
```

这些是历史实际入口，不是复制即能新运行的现成shell命令。本包formal22单轮窗口已消费且过期（2026-10-25 04:35→05:20 UTC），不能重用或静默延长。新运行需按同一管理流程重新核当前依赖路径/模型/新构建SHA、独占进程/锁/CPU/GPU/ROS域，并生成新的具体资源/时间证明；不放宽170,000,000,000B保留上限、10GB单轮额度+10GB额外预留、2400s运行+300s收尾。没有在本次上传任务中启动新仿真或获得实机/训练授权。

## 离线验证公开文件

```bash
python3 -B tools/formal22_manifest_check.py
```

只校验公开manifest、Python语法和JSON/链接等静态内容；不导入ROS、torch，不启动Node/Gazebo，不声称导航验证。源码导出完整性通过不能替代新目录构建/数值检查/实际加载/物理验收。

模型需自行合法提供。实际Student导出SHA为d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185；旧Teacher checkpoint可作为历史绑定存在，不能误当本轮实际Actor。Actor45维输入与归档247维原始/特权观测不是同一数组语义。outer navigation不用GT，native sensor proxy/inner PID依然使用特权仿真反馈。

## 结果读法

`r85_task_result.json`本轮passed/parking_pass/dynamic_pass/full_F3_PASS true，original46 false。固定停车采用首次T0=344.880→349.880而非延后重选窗口，六检查通过；动态最终独立verification.allowed/passed true。业务event_window中最初physical_verified=false为历史快照，不能取代最终结果。entry/closure本身不授业务PASS，它们的固定false不要改写。原始大流不在Git，精选报告不能保证完整重放或独立重算每个历史结论。
