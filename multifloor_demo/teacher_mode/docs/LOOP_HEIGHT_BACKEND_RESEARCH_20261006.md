# FAST LIVO2 高度误差与回环后端适配选择

日期：2026年10月6日。当前代码和文字报告冻结于 `8cdd80c7e99acd5045746b7a9b76cd2ff769d58d`，分支为 `codex/v25-code-reports-only-20261006`。该提交不含实验数据；后续本文提交只补充调研。当前完整46区仍失败于第12区，实际到达11区。

## 高度问题与两种校正能力

当前固定初始SE3、按同一源时间对照支持：SLAM估计高度偏低约20cm，仿真机身基本稳定。当前故障缺少219秒附近完整关联模型与首个SCAN失败体素，尚不能确定是哪次IMU、LIO、VIO更新引发，也未证明高度偏差直接导致停车。此前V7另一次故障已发现竖墙点匹配到历史近水平平面；它是优先核查方向，不是当前故障的已证根因。

现有LIO已经进行点到平面匹配，VIO已有局部参考图像块管理。新增能力应分两项验证：一是独立关键帧点云子图的局部匹配，检查首次高度偏差和错误约束；二是经过几何验证的历史回环边与SE3图优化，校正重访后的累计漂移。回环需要可靠重访约束，不能保证首次进入新平台就阻止偏差；没有绝对高度来源时也不提供绝对z锚。

## 可参考的上游实现

| 候选 | 已核验能力与适配边界 | 选择 |
|---|---|---|
| [SC FAST LIVO2](https://github.com/jxk6575/SC_FAST_LIVO2/tree/f39e3d0008af7cbd838b05ccfbad794c66161596) | ROS1，接受外部优化关键帧；启动完整后端依赖额外包。源码校正使用最新后端pose和当前pose，未见按关键帧源时间向当前传播；协方差链式赋值改变三轴含义，局部地图提取仍用原pose并追加旧地图。 | 只参考接入位置，不直接移植校正实现。 |
| [Pose Graph Optimization](https://github.com/Kimkyuwon/Pose_Graph_Optimization/tree/894f074c353fc82c5c75815eb918f506b2adbe82) | ROS2，包含SOLiD候选检测、KISS初匹配、NanoGICP/DOP验证、iSAM2和后台线程。实际输入为定制fast_lio消息；当前FAST-LIVO2需新适配。回调计算/写盘、共享容器与协方差顺序需要额外审查。 | 优先参考其独立后端结构，先做最小几何配准与图优化，不引入整套额外功能。 |
| [LTAOM](https://github.com/hku-mars/LTAOM) | 作者仓库集成LIO、回环、长期关联及多会话，并提供多层建筑示例；测试环境是ROS1 Melodic等旧依赖。 | 参考多层重访与长期地图关联，迁移范围更大，不作为当前最小改动入口。 |

上表为源码及接口适配判断，不是构建、仿真性能或本项目高度修复结果。第三项依据作者README；前两项固定提交读取了关键接线源码。

SC候选代码依据：[状态校正与地图更新](https://github.com/jxk6575/SC_FAST_LIVO2/blob/f39e3d0008af7cbd838b05ccfbad794c66161596/src/LIVMapper.cpp)、[外部后端启动](https://github.com/jxk6575/SC_FAST_LIVO2/blob/f39e3d0008af7cbd838b05ccfbad794c66161596/launch/mapping_with_pgo.launch)。ROS2候选依据：[输入与线程实现](https://github.com/Kimkyuwon/Pose_Graph_Optimization/blob/894f074c353fc82c5c75815eb918f506b2adbe82/src/poseGraphOptimization.cpp)、[构建依赖](https://github.com/Kimkyuwon/Pose_Graph_Optimization/blob/894f074c353fc82c5c75815eb918f506b2adbe82/CMakeLists.txt)。固定代码中的部分GitHub网页行号和raw解析行号不同，以函数/提交为准。

## 点云匹配怎样验证高度收益

关键帧由当前估计器唯一owner在明确LIO后或VIO后边界输出不可变快照：整数源时间、阶段、外参、局部body位姿、健康摘要、真实协方差与去畸变lidar坐标点云。不能把LIO后的云与VIO后的位姿混为一组，也不能把已经漂移的世界云当独立高度基准。

先用预测位姿初始化当前云对可信历史子图的完整3D配准，分别记录几何重叠、残差、法向/空间覆盖、z方向信息及双向一致性。ICP收敛或低残差不是正确匹配证明；同XY不同楼层、重复走廊、低重叠和动态点必须作为拒绝样例。独立子图仍来自同一传感器，不是仿真真值或绝对高度计。

彩色点云发布不等于彩色匹配。Colored ICP会显式组合几何和光度目标；纹理可以补充平面切向约束，但其收益受标定、同步与纹理质量影响。先建立几何基线，再做相同输入的颜色辅助对照。[Open3D官方Colored ICP说明](https://www.open3d.org/docs/release/tutorial/pipelines/colored_pointcloud_registration.html)

局部匹配健康检查与后端接点详见[本地适配审计](LOOP_HEIGHT_LOCAL_ADAPTATION_AUDIT_20261006.md)，其中包含原地图平面支撑、冻结、完整state/P与回滚的具体边界。

## 接线与验收顺序

1. 只读质量摘要与有界故障窗口取证，保持原状态估计公式，记录分步高度/垂直速度修正及SCAN首个失败查询。
2. 独立局部关键帧子图匹配先输出建议和拒绝原因，验证正常坡道保留、误匹配拒绝、处理时延；经对照后再决定在线融合方法，避免重复计算同源测量信息。
3. 后台回环候选、几何验证和图优化；原前端保留连续局部odom，后端提供版本化map→odom。后端不能并行写前端state/P或活动体素地图。
4. 点云、目标、路线、占据图及区域到达须使用同一坐标版本。全局校正不进入局部位姿差分，避免假速度或假驻留；转换无法原子完成时保护停车再恢复。
5. 先原第11至12区短验证，再完整46区，另测重访/无回环前缀和错层拒绝。保留原300ms、原3D区域、驻留与90秒期限；真值仅离线验收。分别比较前端、局部匹配、后端的高度误差与导航收益，未验证项保持未验证。

本轮完成的是代码/报告上传及适配研究，没有接入回环后端，也没有新的Gazebo修复运行。IMU/LIO/VIO跨帧解耦继续暂缓；通过的camera_mode及其他训练任务保持。
