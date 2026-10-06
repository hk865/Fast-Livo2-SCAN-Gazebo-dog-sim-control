# 独立64线与30Hz传感器对照

仿真限定，使用冻结CPU Teacher、原动作/控制增益/唯一执行器/300ms保护。V7 core与mapping源码和安装库原哈希复用，不修改地图平面算法、相机Demo、其他训练或旧失败。导航反馈来自实际SLAM/IMU/点云/SCAN，native真值仅离线验收；Actor232维特权来源仍明示。

三个210s profile：`l64_r30_c10_210`为用户指定LiDAR64×480/30Hz、RGB10Hz；`l64_r10_c10_210`为仅增线数；`l64_r30_c30_210`同步提升车载RGB到30Hz。LIVO由相机时刻驱动，所以第一组仍预期约10Hz估计，第三组才能接近30Hz。控制器50ms墙钟ticker保持，源header的数学反馈频率独立实测；不把ticker、Actor50Hz、nativePD200Hz混为同一种频率。

`sampling.py`只改fresh world的LiDAR水平/竖直采样、LiDAR频率、车载camera频率四个叶子，冻结修改前world、传感器契约、asset哈希与采样证明；物理、噪声、FOV、K/D、外参和碰撞几何保持。初始化配对容量随camera频率扩大，但原10样本/0.8s/20ms因果配对及角度散布门保持。`scan_line`元数据来自契约，generic预处理无32线硬截断，0.15m降采样不改。

另有两个600s profile，仅在对应前缀实测值得推进时使用；未执行的profile不能算通过。正式收据需在运行完整退出后用原common/ramp判据逐项验收。频率须由unique原始header实测，30Hz受5ms物理步长和运行负载影响，不能直接以配置数值宣称实际30Hz。

```bash
# 项目根；每次另建run，不覆盖历史。
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v8/run.py --profile l64_r30_c10_210 --label lidar64_30hz_rgb10_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v8/run.py --profile l64_r10_c10_210 --label lidar64_10hz_rgb10_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v8/run.py --profile l64_r30_c30_210 --label lidar64_30hz_rgb30_r1 --domain 88
```

GPU对比使用同一冻结权重及1000个保存的实际247维观测，含同步传入/传出、单线程CPU与GPU FP32、50Hz节拍尾延迟；当前实测CPU更快，所以本批传感器对照继续CPU，见[test_results/gpu_benchmark](../../test_results/lidar_density_rate_20261005/gpu_benchmark/README.md)。完整结果将另存报告和独立收据，本README的可运行配置不表示导航通过。
