# 曲率开启实测：原前九区与首次五秒停车通过

2026-10-06 的 V22 独立仿真运行 `20261006_163330_closed_loop_cascade_clock_hold_curvature_on_V22_r1_97ed`，**冻结原八项验收全部通过**：原始9/9区域 dwell、首次固定5秒停车、源码归档、原几何与实际SLAM、采样物理安全、执行合同、真实位姿来源及读取完整性。结束仿真时间149.12 s。这里只认证原前九区与该停车窗；完整46区、主动安全走廊、全200 Hz物理步骤、新控制律匹配Isaac闭环Sim2Sim和真机仍未验证/未通过。跨帧解耦按要求先不做。

[实际浏览器画面与路线](http://127.0.0.1:8768/?run=20261006_163330_closed_loop_cascade_clock_hold_curvature_on_V22_r1_97ed)；[独立正式报告](actual/20261006_163330_closed_loop_cascade_clock_hold_curvature_on_V22_r1_97ed_PREFIX9_ACTUAL_EVALUATION.json)；[诊断](actual/20261006_163330_closed_loop_cascade_clock_hold_curvature_on_V22_r1_97ed_CURVATURE_DIAGNOSTICS.json)；[一次读取及SHA收据](actual/20261006_163330_closed_loop_cascade_clock_hold_curvature_on_V22_r1_97ed_AUDIT_RECEIPT.json)。报告SHA256：`f0e6a2255b4cdcfd9f42bd0910d61a85eb113b36c9bef548b39fc6b8b0c11d30`。

## 开启内容与修复

独立profile `curvature_original46_prefix9_on` 开启 `curvature_feedforward_enabled=true` 和 `curvature_speed_limit_enabled=true`，使用0.2 m有限弧长参考。控制器在行走时提前生成曲率转向参考，并按转向速度、参考角加速度及换路跳变预算限速；无可行预算时进入 `reference_constraint_hold`，冻结积分、输出精确零速度命令并请求重新规划。零速度命令由Teacher继续执行，未用action=0或锁关节充当停车。

前馈日志字段是 `v×κ`；原D项含 `0.15×v×κ`，总Euler yaw参考为 `1.15×v×κ + 1.3×heading_error − 0.15×filtered_yaw_rate`。预算同样使用1.15。曲率预算限制参考项，不保证Teacher实际响应或全部反馈/PI角加速度；原PI限幅、ACK抗饱和、命令slew与全部保护保留。

首个ON运行暴露了真实缺陷：预算返回NumPy bool，使 `is False` 漏过无解分支并导致JSON序列化崩溃。V21只将预算输入/输出转换为Python标量，未改数学、增益或阈值；原输入独立重现为非零drive，修后同输入为零命令保护停车且积分不变。46项有限修复检查和270组新旧数值比较通过，见[v21修复与评审](v21_scalar_fix/README.md)。

V21实测完成九区和停车，但启动归档漏了共用 `mission/route_regions.py`，仍判NOTPASS。V22仅补齐五个几何helper的直接gate、源码快照、原source及规范snapshot SHA、manifest双键与启动拒绝门；控制数学和路线参数与V21字节一致。原V20/V21源码、门和失败收据保留。85项V22有限检查、独立109项归档检查及prepare原字节核对通过，见[v22归档修订](v22_archive/FINAL_READY.json)、[独立review](v22_review/ARCHIVE_REVIEW.json)。这些软件检查单独不认证导航。

## 四次实际运行，失败不回填

| 运行 | 实际功能结果 | 冻结原门结论 |
|---|---|---|
| OFF `97a7` | 九区完成，停车XY最大4.75 mm | NOTPASS：停车status存在同一ROS时间戳的两行，严格覆盖门失败；物理漂移阈值通过，未删重复行制造通过 |
| 初始ON `cc82` | 约102.76 s、7/9区域；CPU Teacher无物理fault | NOTPASS：NumPy bool序列化崩溃；控制器退出后桥按超时停车，异常退出保留 |
| 标量修复ON V21 `dc92` | 九区及停车通过，XY最大14.77 mm | NOTPASS：共用路线源码未被该run直接归档；未利用旧gate间接声明升级结论 |
| 归档修订ON V22 `97ed` | 九区及停车通过，XY最大4.45 mm | **PASS：原8/8检查全部通过** |

## V22运动与停车证据

两个开关实际生效：新数学行中前馈非零2012行、预算开启2322行、实际限速34行、无解9行并进入保护停车9次；最大日志原始`|v×κ|`为0.070390 rad/s。计数是诊断采样，不是全部200 Hz物理步骤证明。原第九区从实际激活到到达为17.115 s。 [逐行compact复核](V22_ACTUAL_CONSTRAINT_HOLD_COMPACT_PROOF.json)确认九行预算显式false、实际最终命令精确零、PI向量与前行不变；这些行的`translation_integral_frozen`诊断标志仍为false，保留其局限，不改原日志。

首次固定停车窗142.115–147.115 s，使用152个真实SLAM位姿、250个native样本、31个status和131个控制记录核对；全部10子门通过。最大XY漂移4.4485 mm、yaw漂移0.0005123 rad，native平面速度最大0.008410 m/s、body-z角速率最大0.003972 rad/s。窗口连续active_hold且无保护；末尾清理时的IMU超时消息未用作停车通过证据。

导航位置、朝向、速度与区域判定来自真实SLAM/IMU/点云/SCAN；Gazebo真值仅供离线物理验收。Actor仍232维特权仿真观测＋15维已知命令/上一动作，不能称全感知策略部署。Teacher CPU单线程50 Hz、原生PD200 Hz、300 ms超时、唯一执行器与0.45 m路线边界不变。本轮没有修改camera_mode、训练或另一任务。既有V19接收/解码流水线复用，IMU/LIO/VIO共享状态顺序未改；正常排空88077条accepted/delivered/committed一致，pending/canceled为0。

## 曲线与描述性比较

[SLAM轨迹](plots_OFF_V22_ON/slam_xy.png)、[曲率前馈/限速](plots_OFF_V22_ON/curvature_speed.png)、[执行命令与实际native速度](plots_OFF_V22_ON/commands_velocity.png)、[路径及朝向误差](plots_OFF_V22_ON/drive_errors.png)。原SLAM采样自己的camera_init坐标与时间，不强制两轮对齐；缺失值不补零，不把IMU body-z当Euler yaw速率。

以下只取新数学更新且处于drive的样本（OFF2414、ON2313）；速度误差为参考COM与原始SLAM测得平面速度模之差，角速度误差为参考body-z与原始IMU body-z之差。

| RMS量 | OFF97a7 | ON97ed |
|---|---:|---:|
| 内环朝向误差 rad | 0.04531 | 0.03151 |
| 当前路径横向控制误差 m | 0.00879 | 0.00793 |
| 平面COM速度模误差 m/s | 0.05690 | 0.05402 |
| IMU body-z角速度误差 rad/s | 0.05435 | 0.04832 |

本次样本指标较低，但两轮SCAN实际路径、采样与源码修订不同，profile有8项字段差异，只有各一次完成试验；不能据此宣称严格单变量因果、最优PID或普遍优于CHAMP。当前路径横向误差也不是相对实验路线fence的误差。[完整差异及比较](OFF_V22_ON_DESCRIPTIVE_COMPARISON.json)。

曲率功能没有补齐“候选SCAN路径必须位于实验路线边界内”的准入门。此前完整46区在第十区越界的 `dc50` 仍失败；走廊仍shadow、旧路径保留未启用。后续需先保证候选路径满足障碍clearance及实验路线边界，再重试全任务；不能只放宽边界。

## 操作命令与回滚

以下从原工作区 `/home/hyh001/projects/1.Project/Ros2_fastlivo2_` 执行，会创建新run。需要本机已冻结的V19 SLAM构建、原SCAN和指定权重；复制到另一机器必须重新构建、冻结gate并实际验收，不继承本次PASS。

```bash
# 不启动ROS/Gazebo的准备；五helper直接归档闭合是启动必需项
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v22_curvature_archive/run.py \
  --profile curvature_original46_prefix9_on --label v22_prepare --prepare-only \
  --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs

# 实际有限验证，独立domain及唯一执行器；需保持150 GiB空余，不降低存储保护
python3 -B multifloor_demo/teacher_mode/navigation/corridor_tracking_v22_curvature_archive/run.py \
  --profile curvature_original46_prefix9_on --label curvature_on_V22_retest --domain 91 \
  --run-storage-root /home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs

# 新run全部进程结束后仅一次独立验收，已有输出拒绝覆盖
python3 -B multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/audit_curvature_prefix9_v22.py \
  --run /绝对路径/新run --condition ON --output-dir /新建证据输出目录

# 只读展示
python3 -B multifloor_demo/teacher_mode/scripts/serve.py --port 8768
```

模型SHA256 `bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`；V22profile `9b3c3e33591a0de3fd91a2e613ddb4f79bbb99c8e11b25daefc10a70dde0f1b4`；新gate `772601d661bd2b9fbf9cef2049e10b828e6c9516b64b2b4a8033bcef57f907e2`。原8门、base observer及版本adapter哈希见AUDIT/COMPOUND收据与最终逐文件清单。

源码和精简证据保存到独立Git分支 `codex/v22-curvature-prefix9`；原V20分支与main保持。切换原分支可退回原源码检查点；V22不合入默认camera Demo，也不继承clone运行授权。仓库不提交权重、build或完整raw。

## 页面与机器可读结论

[实际运动截图](V22_LIVE_BROWSER.png)、[最终真实SLAM路线与独立有限通过截图](V22_FINAL_BROWSER.png)、[浏览器核对收据](BROWSER_FINAL_RECEIPT.json)、[最终分层状态](FINAL_STATUS.json)。原全任务共同收据仍未验证，页面在路线说明中单列已核对的prefix9有限PASS，不将它升级到全任务。

## 存储

四轮大点云逐文件流式压缩至 `/var/tmp/go2_teacher_curvature_20261006_archives/`，全部解压内容SHA/长度核对后仅逐个删除本机`.bin/.npy`原件，保留PID/SLAM/status、实际相机、native/actuator、源码快照及失败。压缩档未提交Git，可按各工具`restore`恢复精确字节。详见 `*_LOSSLESS_ACTUAL_RECEIPT.json`、`*_CLOUD_ARCHIVE_MANIFEST.json.gz` 和[无损归档操作](LOSSLESS_OFF_CLOUDS.md)；最终空间统计在 `STORAGE_FINAL.json`。这与此前77.32 GiB旧raw/cache清理分开计数。
