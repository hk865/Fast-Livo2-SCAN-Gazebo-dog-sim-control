# SLAM 初始化、局部匹配、关键帧与回环审计

审计日期：2026-10-05；写入时刻：2026-10-05T07:38:32.518368+00:00。本文件新增且只读取现有代码/原件。未修改执行源、原配置、原运行、验收或已通过相机模式；没有启动仿真、ROS、模型或其他进程。

当前已经有 IMU 初始化、激光局部几何匹配和图像参考管理，但本 core 未发现全局地点识别/位姿图回环的调用链。彩色点云显示不等于 color ICP。两次平台高度失败证明当前链仍有估计问题；不能仅凭“缺回环”或“初始化时动了”锁定唯一原因。

## 1. 初始化实际做了什么，是否足够静止

两个原件为 `20261005_145640_closed_loop_cascade_actual_multifloor_r1_d42e` 与 `20261005_151217_closed_loop_cascade_online_SLAM_multifloor_r1_b182`。实际 IMU 初始化器均接受 **0.58–3.60秒，605个样本、跨度3.020秒**，此前 **111次 reset**，最后原因 `angular_motion`。原门是 gyro≤0.03rad/s、加速度模长距9.81≤1.2m/s²、每轴std≤0.4m/s²、sample gap≤0.03秒、跨度≥2.9秒及样本≥600；不是仅加载模型后立即初始化。实际gyro峰值d42e/b182分别0.028850/0.028919rad/s。原初始化日志/原IMU前缀SHA均保存在[独立指标](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/init_matching_metrics.json)。

这个门接受的是低运动窗口，**不是完全静止**。同一窗口的native仅离线运动对照是XY位移9.626mm、yaw增加 **2.114798°**、Z范围3.690mm。这个真值只用于说明门的性质，没有进入SLAM/导航。导航 sensor warmup 检查消息连续性与freshness，后续5.8–6.7秒10组实际SLAM/IMU配对冻结场景yaw，是另一项地图注册合同；两者都不能代替IMU初始化质量评估。

原IMU 0–6.7秒前缀中，gyro≤0.01rad/s的最长连续窗口只有0.250/0.245秒，≤0.005时只有0.085秒且发生在早期下落。简单把门改为0.01并等待3秒，在这份实际前缀里不会初始化成功。初始化代码收集mean gyro但明确将bias_g设为零；不能把这段真实慢转的均值直接作为gyro零偏，否则会抹掉真实yaw运动。初始frame小倾斜（离线约0.28°）提示可以改善初始化，但不足以解释后续平台上突然约20厘米偏置。

原IMU归档没有所有逐样本acceleration：受理窗口mean/std来自初始化器实际日志；受理后debug/imu.txt是传播器head/tail平均加速度，不可冒充完整原始加速度流。建议应先在独立版本验证更稳定的初始化阶段或明确允许移动的初始化方法，同时记录原始加速度、重力/bias收敛与残差；这是**建议，尚未验证改善效果**。详见[初始化来源说明](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/INIT_MATCHING_APPENDIX.md)。

## 2. 实际匹配与图像参考管理已经存在

该SLAM core使用FAST-LIVO2。激光链 `handleLIO → StateEstimation → BuildResidualListOMP` 迭代点到平面残差，并以平面/点协方差加权，经ESIKF更新状态；不是把每帧彩色点简单叠加。图像链 `processFrame → retrieveFromVisualSparseMap → computeJacobianAndUpdateEKF → generateVisualMapPoints → updateVisualMapPoints` 将地图点投影到图像、选择参考观测并使用光度patch误差。当前过程将3通道图像转为灰度。源码与论文均支持“激光几何+图像光度+IMU”这一设计区别；论文说明的是算法机制，不是本次适配运行已经达到其论文精度。[FAST-LIVO2 原论文](https://arxiv.org/abs/2408.14035)

当前代码已有局部帧/参考管理：按光度一致性或近视角选择观测，并进行可选NCC/patch拒绝；位移>0.5m、转角>0.3rad或像素移动>40时增添参考观测，观测达到30删除弱参考。它不是完全没有历史帧。`updateReferencePatch`虽有定义，当前`processFrame`未调用，不能把注释中的计时名或论文中的机制当成该运行已执行。可核对[实际VIO过程](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:1806)与[源码调用审计](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/init_matching_metrics.json)；该审计六个源文件SHA在本文核查时全部一致。

“局部参考帧管理”和“全局关键帧图优化”需要分开。前者帮助当前帧与已有局部patch关联；后者还需要场所识别、跨时间约束与全局优化。当前core未发现place recognition、pose graph或全局loop closure调用链。源码中的`lidar/imu/image loop back`是在时间戳倒退时清空缓存，**不是回环检测**。这是限定于所审core和调用链的发现，不声称所有项目模块都不可能提供回环。

## 3. 为什么已开VIO不等于视觉约束有效

d42e/b182实际 `vio.img_point_cov=100000000`，源码参数默认100。代码更新含 `(HᵀH+(P/R)⁻¹)⁻¹`，等价后验信息为 `HᵀH/R+P⁻¹`。在相同H与其他条件下，这个标量使每像素信息比默认100弱 **10⁶倍**；不是说视觉被关闭，也不是说实际位姿误差必然放大10⁶倍。该阶段原VIO位移修正近零与弱权重相容，仍需检查实际patch、残差、标定和信息量，不能仅凭配置作唯一归因。[实际协方差与更新式审计](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/init_matching_metrics.json)；[原更新位置](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:1516)。

V5 b182在新版本开启gravity_est_en/ba_bg_est_en，其他控制合同保留，但 **220.3秒 connector_y仍按原高度门失败**：raw位置 `[14.984697,4.394268,0.983030]`，原目标 `[14.986784,4.414264,1.205630]`，Z差−222.600mm。这个故障前阶段是有效的实际对照，可说明仅开启两项没有消除本轮已观测的heightfail。随后约 **429.33秒 socket timeout/BrokenPipe并发生后续塌落失稳**，故整轮未完成600秒并且FAIL；不能称完整长程对照通过，也不能用后续断管/塌落解释先前高度失败。后段执行链故障与前段SLAM高度偏差分别保留。

**V6目前pending**：根节点已启动新独立版本，将视觉协方差改为1000。相同H下它比1e8配置强10⁵倍，仍比源码默认100弱10倍；控制器/原验收门不因此放宽。[V6事前来源核查](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/V6_PREFLIGHT_REVIEW.json)只是preflight，不是物理或导航通过；本文不预判其结果。

## 4. 地面看得见，为何仍可漂；彩色云是否能匹配

原点云对照显示：d42e130→135秒地面与raw body同步下降0.217600/0.218401m；b182135→140秒下降0.259961/0.263663m。两轮仍保留约6000个地面inliers、RMS约5.5mm，body-origin/地面净高约0.30m。结合原初始真实floor1平面和**已知静态地图层高**，可独立观测约20–22厘米法向偏差，估计过程没有native输入；native仅用于事后比较。[两轮原点云报告](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/REPORT.md)和[地面观测报告](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/final_v3/REPORT.md)。

单个平面几何Jacobian只有rank3：法向位移、两倾角可观；两平面内平移和绕法向旋转不可观，近水平时即XY/yaw不由这个地面面片确定。这是独立观测器的几何proxy，**不是SLAM内部Hessian**。原云还有非地面结构，但没有该段完整内部对应、残差、优化信息矩阵或回环事件，不能仅看点数就断言全6DoF约束充分。SLAM注册点云与raw pose本来共用同一估计，仅有0.30m相对净高不会自动暴露绝对高度漂移；已知地图高度才提供独立参考。

[原点云130/140秒图](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/original_cloud_130_140_comparison.png)区分测量点、拟合面和raw pose；[原地面残差与离线比较图](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/final_v3/plane_vertical_residual_comparison.png)区分原yaw-only地图、实际SLAM+IMU up敏感性与native仅离线误差。不能用目标机身Z盲补：初始与当前支撑净高相差约18mm，这部分是真实姿态/支撑变化而非SLAM漂移。

彩色输出链 `publish_frame_world` 将已注册的XYZ按当前frame投影到RGB图像，读取插值像素给点着色；几何输出位置并未通过RGB最近邻残差重新优化。图像VIO则是灰度patch光度残差。**点云有颜色、渲染看起来重合，并不证明实现了color ICP或可靠全局彩色云匹配。** 可核对[原着色发布代码](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/LIVMapper.cpp:1494)与[灰度图像入口](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/slam/ros2_ws/src/fast_livo2_core/src/vio.cpp:1817)。彩色ICP可作为未来另案设计，但不能当作当前已有能力或自动修复结果。

## 5. ORB-SLAM3关键帧和回环是否值得接

ORB-SLAM3是特征式视觉/视觉惯性SLAM，包含关键帧、局部bundle adjustment、地点识别和多地图合并，并以MAP估计处理IMU初始化；其历史共视关键帧可进入优化。它与当前局部直接法参考管理的用途有重叠，但结构不同。[ORB-SLAM3 原论文](https://arxiv.org/abs/2007.11898)

因此可以研究外部回环/全局图优化来限制长程累积误差；但还没有证据表明接入ORB-SLAM3能解决本次短时平台高度跳变。一路上坡尚未重访同地点时，也不能依赖尚未发生的回环把前段修好。若接入，须独立核对图像纹理/视差、IMU时间外参、单目尺度、camera_init转换、地图修正的连续性与不确定度；融合pose必须有唯一权威来源，原300ms freshness、碰撞guard和到达门继续适用，不能直接把两种pose择优拼接。以上是本文建议，未实现或验收。

可优先继续同路径、同控制器和同门的视觉信息权重对照，并保存有效激光面/视觉patch数量、残差、各次更新增量与信息矩阵；在这些源证据上再判断初始化、局部退化、外参/曝光、回环或已知地面prior的优先级。当前不改判据、不以“需要回环”代替实际问题定位，也不以全局图优化需求阻止已有闭环多层路线继续验证。

## 原件与哈希

下列均为此前已完成原报告/图，本文件仅引用。实际两个run、source prefix字节范围、原始配置、源码hash和断管结果在对应JSON中保留。

| 原件 | SHA256 |
|---|---|
| [初始化与匹配审计](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/init_matching_metrics.json) | `fac6e22ad9f762a9da91e59313a86bf239524ec7fac735afa2e540879aeb26f8` |
| [初始化原始附录](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/INIT_MATCHING_APPENDIX.md) | `9d4675cce1e84545752a12c4581b3cfed2f62e36594bf0635583d3291361a43e` |
| [d42e原高度诊断](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/multifloor_height_diagnostic/DIAGNOSTIC.md) | `8384de4190bbb776c4beb59ef37d94ade90952926a0eadbeae9d3cb92039271a` |
| [d42e地面观测原报告](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/final_v3/REPORT.md) | `645cf904b796ab4ece017ab7f4e8406ba3552ffa5b86faddff18d6df6b0c80f0` |
| [d42e地面观测原摘要](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/final_v3/aggregate.json) | `355c2267b7aeaa5bfd7f40d2082fd13e09c804d8f8259a5de9c176b0525ca4d8` |
| [两run点云几何报告](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/REPORT.md) | `8450faf77605ba29d80dac20b63c8cb64daa4afe0b1c75dd85f78a8cc37ed77d` |
| [两run点云几何摘要](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/aggregate.json) | `da350c8b6a3219ba36d86aa6881dbad6d23107851f85322b9496f0c83c2e0909` |
| [130–140秒原云图](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/compare_b182/final/original_cloud_130_140_comparison.png) | `73b7cd5555eec2c7dd4e7b645a50e3cdfb9ac11b463c7be0b94b8d913d462a11` |
| [地面残差与离线真值对照图](/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/closed_loop_navigation_20261005/ground_plane_fusion_design/final_v3/plane_vertical_residual_comparison.png) | `aad69c105b3404cd261a20b9eb3ca29d64e072eefc4e8e28cd041cbd70b65b43` |

本文件仅记录截止写入时的审计范围：d42e和V5高度失败保留，V5整轮FAIL，V6 pending；没有宣称全多层导航、完整Sim2Sim、全局回环集成或真机部署通过。

## 根节点补充：V6 实际结果

上述 pending 是审计写入时状态。随后 `20261005_153109_closed_loop_cascade_visual_cov1000_multifloor_r1_9cc6` 实际完成600秒/30001次CPU推理，worker fault=null，五角色与11子进程清理完成；导航仍失败。第一次保护原SLAM头131.7秒、命令clock131.74秒，raw位置 `[14.878118163195165,2.1843743983558075,0.8989408802532117]` 对该段路线高度 `1.2054456319942866`，管道高度误差 **0.30650475174107483 m** 超过原0.3m门；水平管道误差0.018614m。已完成16/32区域，随后Actor继续反馈推理、零速度命令保护站立，到600秒结束。正式独立验收另行追加，不以runtime completed作为导航通过。

仅恢复视觉权重没有解决此轮局部高度失效，也不能由这一次负对照断言图像/初始化完全无影响。下一步内部实际匹配诊断插点见 [PROPOSAL](../test_results/closed_loop_navigation_20261005/front_end_observability/PROPOSAL.md)。它仍是提案，没有执行或回环集成。

V6最终三份独立收据已封存：共同与完整双坡道FAILED，heading source 8/8 PASS；全600秒原生安全、运行、控制来源、路线判据通过，但原全3D到达与最终停车未完成。详见 [独立失败附录](../test_results/closed_loop_navigation_20261005/online_SLAM_AB/V6_FAILURE_APPENDIX.md)。
