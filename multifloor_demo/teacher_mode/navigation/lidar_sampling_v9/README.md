# 独立64线采样与ROS clock零速度保护（V9）

仿真限定，冻结Teacher继续CPU单线程。SLAM/SCAN导航来自实际传感器；Actor232维特权来源保持明示，native真值仅供离线验收。复用V7已冻结诊断workspace、原估计算法与原安装库；不修改V8/V7/camera、训练或历史验收。`diagnostic_preflight.json`是相同V7 core的已完成诊断等价性证明，不是V9的新运动结论。

V8在正常注册后的短时锁步停顿中，两个20Hz墙钟tick看到相同ROS clock；新合法点云仍可能到达。旧native几何guard将这个重复计算时刻当异常锁死。`clock_hold.py`在整个原control流程之前检查clock：短重复立即发布精确零速度，不计算PI/heading/到达/几何，不刷新任何源receipt；之后clock真正前进，必须使用当前实际云完整重验guard才允许非零。旧native guard严格递增契约原样保留。原始tilt/bridge severe与evidence error保护在hold中仍执行。墙钟stall达到300ms或任何原来源sim/wall TTL不新鲜时，reset受保护的PI与到达/clear dwell，并保留连续失联超过8秒失败。真实clock倒退、已接受同header数据或receipt异值仍锁定失败；正常短重复可恢复。

每个hold另记`navigation_control_clock_hold.jsonl`，含原clock、实际zero publish clock、源header/receipt、零命令、数学/guard调用标志、积分与dwell前后快照；关闭writer的期望条数也单独记录。**验收必须按原时间把这些记录与PID/guard记录合并回放**：长hold的`cascade._protect`及短hold的零命令/slew起点不在旧PID history之内。旧common/ramp原判据与旧收据不改；未追加这种prospective chronological replay时，不能把忽略hold的旧PID回放宣称完整通过。Focused tests只证明安全边界与配置，不表示Gazebo或导航通过。

保持50ms墙钟controller ticker、Actor50Hz、native PD200Hz、原全部增益与唯一执行器、300ms门。源header数学频率另行实测；相机驱动LIVO，LiDAR30/RGB10仍约10Hz估计。profiles包含64×480/10、20、30Hz及RGB10/20/30；20/20的210和600秒profile仅改采样及对应源率元数据。20Hz注册capacity18，18点跨度0.85s，原10样本/0.8s/20ms因果门与散布门保持。`l64_r30_c30_detail3s_210`仅将详细诊断窗口115–165秒缩为115–118秒，其余与V9原30/30相同；可用于日志开销A/B，估计算法未改。

```bash
# 项目根；每次独立run，未运行profile不能算通过。
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v9/run.py --profile l64_r20_c20_210 --label lidar64_20hz_rgb20_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v9/run.py --profile l64_r30_c30_detail3s_210 --label lidar64_30hz_rgb30_detail3s_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v9/run.py --profile l64_r20_c20_600 --label lidar64_20hz_rgb20_full_r1 --domain 88
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v9/run.py --profile l64_r30_c10_600 --label lidar64_30hz_rgb10_full_r1 --domain 88
# 独立无ROS/Gazebo检查；读取历史fixture，仅写临时目录。
cd multifloor_demo/teacher_mode/navigation/lidar_sampling_v9
python3 -B -m unittest -v clock_hold_tests pure_tests sampling_profile_tests
```

GPU同权重、同实际247维观测、同步传入传出节拍测试现阶段CPU更快；参见[GPU实测](../../test_results/lidar_density_rate_20261005/gpu_benchmark/README.md)。本目录新增35项pure/focused检查通过；实际Gazebo运动、完整路线与Sim2Sim需运行后的独立收据。
