# V11：诊断复制仅按实际3秒窗口执行

已完成独立构建及有限数值检查，可 prepare-only；尚未进行本候选实际运动/吞吐/导航测试。新core SHA：`f8431ddff43e23fe00c99112c19f00ef4530e9447f4ed213c08494141717b6ef`。

本候选保留 V10 的既有 LIO 两线程、串行 VIO 显式残差循环、CPU1线程Teacher及冻结 V9 controller/保护。仅将完整solver/query/VIO诊断复制限实际profile窗口115–118s，窗外保持原来源回调、IMU传播与caller最终LIO/VIO状态/协方差日志。60double查询行还用于kind100聚合，因此kind100一起限窗，不输出伪零匹配计数。plane/frame诊断历史仍全时段维护。点云kind4也使用统一logger谓词和原offset-corrected源header时间；估计输入和时间戳不改。

原V9/V10短profile实际环境仍硬编码115/165，V11才实际读取声明window。有限测试18进程覆盖合成LIO729点、VIO9patch forward/inverse，logger关/窗内/窗外，新旧安装库的状态/协方差/求解输出逐字节一致，窗内非wall日志一致。窗外V11无fullsolver记录，无60double查询分配。完整真实地图/轨迹重放与实时性能仍未验证。详见[依赖审计/有限测试](../../test_results/lidar_density_rate_20261005/logging_acceleration/DEPENDENCY_AUDIT.md)、[诊断契约](LOGGING_ACCELERATION_CONTRACT.json)、[有限preflight](logging_acceleration_preflight.json)。原copiedV10/V9 preflight为历史执行逻辑证据，实际新库由新的logging preflight与per-run加载检查绑定。

仅以下三个组合配置可选；原模型、数学、原300ms有效期及执行权限不变。旧run/训练/camera_mode不改，连接场景为坡道。

```bash
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v11/run.py --profile l64_r30_c30_detail3s_210 --label lidar64_30hz_rgb30_lio2_log3s_prepare --prepare-only
# 唯一仿真退出后由根任务顺序运行
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v11/run.py --profile l64_r30_c30_detail3s_210 --label lidar64_30hz_rgb30_lio2_log3s_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v11/run.py --profile l64_r20_c20_600 --label lidar64_20hz_rgb20_lio2_log3s_full_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v11/run.py --profile l64_r30_c30_600 --label lidar64_30hz_rgb30_lio2_log3s_full_r1 --domain 88
```

30Hz前缀先验证原来源及时性。600s配置是采样、LIO2线程、诊断复制缩窗的组合实验，不是单因变量加速比较。完整32区域、坡道、停车/控制/物理验收均须独立核对；不能把启动成功、前缀或界面显示计作多层导航通过。
