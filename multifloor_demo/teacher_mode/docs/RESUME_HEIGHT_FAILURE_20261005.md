# 高度故障诊断后接续

先读 [SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005](SLAM_HEIGHT_FAILURE_DIAGNOSTIC_20261005.md)、V7 README 与 `analysis/actual_r1_lio/LIO_MATCHING_FAULT_REPORT.md`、`actual_r1_plane_geometry_audit.md`、`actual_r1_vio_v2/summary.json`。当前run为 `20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf`，210秒实际完成、159757条完整日志、16/32区域FAILED，首次保护133.13秒。没有在线修复后的通过。

主要证据：131–134秒native机身仅下降3.668mm，内部估计下降336.545mm；初始远东墙单环簇被错误拟合为水平面，约5.7–6.0秒后按点数停止更新，靠近时新竖墙点仍被接受并下修vz；已有负vz积分造成主要高度损失。VIO继续纠偏但高度/俯仰/X耦合方向变弱。限定单迭代移除实验支持错误墙面组的实质贡献，不能连成新轨迹或称导航通过。VIO共享normal进入warp仅有源码链、缺track-plane对应，不能全部归因。

V7执行源码、profiles、schemas、新安装二进制和run快照已冻结，后续修复另建V8。原core/ros、旧安装库、V1–V6、camera_mode、旧收据保留。Teacher只用固定SHA的model_1000.pt，CPU1线程，不重新训练、不操作真机、不停止其他任务。导航仍用实际SLAM/IMU/点云/SCAN，真值仅离线验收；Actor232维特权来源明示。

下一轮先设计平面二维支撑/法向可信度与有界重估规则，处理近线模型过早成熟及永久冻结；正常地面/坡道薄扫描弧必须保留。不要直接部署事后mid/max=.05或姿态Z硬钳制。neighbor无量纲/米制比较bug另做单变量对照，132秒主要误约束来自primary，不能把fallback当唯一根因。先同配置210秒前缀，再完整600秒/32区域、双坡道接触几何、停车与动态障碍。原300mm/300ms保护、唯一执行器和停车持续Teacher推理保持。

全部本轮owned运行已清理，回放入口 http://127.0.0.1:8768/?run=20261005_175143_closed_loop_cascade_height_diag_baseline_r1_6fdf 。原始records.bin约6.44GiB，需保留并按索引seek，别全量常驻内存。可复现命令见总报告和V7 README；新输出用独立目录，不覆盖本轮结果。新源码/结果/README/status完成后最后运行 `python3 -B multifloor_demo/teacher_mode/scripts/update_package_manifest.py --write`，它会归档前清单。

现有接口/运动、有限SLAM导航与单次完整12m ramp12已有通过；完整Sim2Sim未通过、新律Isaac闭环对照未验证；完整多层32区域仍失败；真机未验证。三层为坡道，不能称楼梯。后端回环研究另有报告，不能用它替代当前错误点面约束的前端修正。
