# 实际 IMU 与关节：30维 Teacher 输入替换闭环审计

已有真实ROS消息替换和Gazebo物理闭环：初版站立通过、初版前进7.3 s来源拒绝失败；优化消息桥首组三次18 s前进为**2次通过、1次末帧来源拒绝失败**。08:00资源空闲复测另有**3个清洁前进/停车运行全部通过**，中间清理被SIGTERM打断的一轮不计完整运行通过。所有原结果保留，来源与动作回放通过不改写历史失败。COM速度及187点高度仍是仿真特权输入；本结果不能宣称完整感知、整体Sim2Sim、导航或实机通过。

## 独立逐帧验证

`scripts/audit_sensor_replacement.py` 只读取原run，使用其归档broker、观测源码、外参、资产与冻结模型，单线程CPU重放，不导入ROS、不启动Kit、不改原日志。真实来源是 `/livox/imu` 与 `/demo/control/measured_joint_states` 的实际订阅callback，实际发布者唯一`ros_gz_bridge`。姿态与外参读取run/sensor_contract；关节按冻结FR、FL、RR、RL名称明确映射。

0–3 s为记录在案的特权初始化；t≥3 s替换区间`3:6`角速度、`6:9`重力、`12:24`相对q、`24:36`qd，共30维。gyro缩放0.2、qd0.05，q减冻结default；其余217维逐元素保持原输入，其中190维是COM3+height187特权来源，27维是命令3、上次限幅执行器命令12和上一raw action12这些控制器已知状态。软件力矩命令不是实体力矩测量。

每次按header stamp不晚于native有效物理戳`world_t−0.005`取实际历史样本；sim龄≤0.025 s、接收墙龄≤0.3 s，实际源不满足即停止推理、请求原生阻尼并终止，**没有真值回填**。未使用未来消息、动作置零停车或支撑姿态capture。

| 实际run尾名 | 原运动结果 | 真实替换帧 | 247重建最大误差 | CPU动作回放最大误差 | 备注 |
|---|---|---:|---:|---:|---|
| stand `bcdc` | 15 s通过 | 601 | 0 | 9.54e−7 | 5–14 s漂移0.01090 m、yaw0.04363 rad |
| forward v1 `1687` | 7.3 s失败 | 215 | 0 | 1.85e−6 | IMU无因果新鲜样本；未完成停车 |
| forward v2 `c1a0` | 18 s通过 | 751 | 0 | 1.43e−6 | vx0.24502 m/s；前进1.91032 m；15–18 s漂移0.000516 m、yaw0.03141 rad |
| forward v2 confirm r1 `882d` | 18 s通过 | 751 | 0 | 1.07e−6 | vx0.24515 m/s；前进1.91066 m；停车漂移0.000404 m、yaw0.03255 rad |
| forward v2 confirm r2 `080e` | 18 s末帧失败 | 750 | 0 | 1.16e−6 | joint墙龄拒绝；终止帧不运行actor，整体仍failed |
| idle forward v2 `d6cd` | 18 s完整通过 | 751 | 0 | 1.55e−6 | vx0.24359 m/s；前进1.90774 m；停车漂移0.000623 m、yaw0.03108 rad |
| idle forward v2 confirm `dbb4` | 18 s完整通过 | 751 | 0 | 1.25e−6 | vx0.24276 m/s；前进1.89123 m；停车漂移0.000397 m、yaw0.02483 rad |
| idle forward v2 confirm2 `a843` | 18 s完整通过 | 751 | 0 | 1.55e−6 | vx0.24571 m/s；前进1.91655 m；停车漂移0.000673 m、yaw0.02381 rad |

上述有效替换帧的原始项与真实callback、独立外参旋转、按名q/qd、保留217维的最大差均为0，所有因果与龄检查通过。被拒绝帧日志247为NaN且`actor_inferred_this_frame=false`，没有用最后日志action执行特权PD；原生最后记录为`mode=1,terminating=true`的阻尼。进程exit0意味着受控结束，不代表该轮运动通过。所有五轮记录均无机身接触；第三次v2来源失效并非物理跌倒。

## 消息桥v2与剩余墙钟失败

初版broker内部与导出均为80条、50 Hz整包JSON。v1有效替换帧p95 sim龄20 ms，已收到却未导出的更新在forward中有IMU78帧/joints86帧，最多20 ms，消耗25 ms预算。于是仅修改broker为内部80条完整保留、raw JSONL全保留、live payload最近16条、墙钟200 Hz原子导出；记录导出数量和周期。冻结SHA由v1 `9d77e3f6be471abca4b9972f67d9b6864e67bf44882bb69ba81235ed00af3be8`变为v2 `de84a4242279d824e94c5fd0c47978dbbca2c4228009d6ead5cae15c40e1c549`。没有修改策略、PD、物理、坐标、执行器或TTL。

v2三个有效前进段的sim龄p95均0，max分别5/10/5 ms；wall龄p95约6.6–6.9 ms，max约11.8/16.6/15.9 ms。说明通常导出及时性改善，但不能宣称所有源失联已解决。

`080e`末帧拒绝记录为`reference_ns=18005000000,latest_joint_ns=18000000000,age_sim_s=.005,broker_sequence=4516`。该报错分支之前的broker alive/hash/wall、发布者与IMU选择检查已通过；5 ms满足sim门槛，joint无合格样本只能由**实际接收墙龄>300 ms**造成。完整raw与events中两流invalid=0，时间戳严格递增，排除重复stamp拒绝解释。joint 18.000 s于monotonic293297.508232收到，下一18.005 s于293297.935680收到，间隙427.448 ms；IMU对应间隙426.694 ms。若旧joint已过期，所有旧IMU也已过期，而IMU选择通过，说明新的IMU消息已进入文件、对应joint尚未进入该次导出。根据实际回调时间，旧joint墙龄下界约426.475 ms。

最符合证据的是长上游物理/渲染/IPC停顿后，200 Hz定时器在IMU与joint两个callback之间输出单边更新，worker读到它时joint仍旧。**精确拒绝read-wall与当时payload未存档，不能把计时器顺序或GPU调度认定为唯一根因。** 末段资源日志当时GPU约8.1–8.3 GiB/69–86%使用，属于需要复测的背景，不是充分因果证明。较早17.800→17.805 s也有约531 ms墙钟接收间隙而恢复后动作继续；长墙钟停顿并非只末帧一次。

后续建议先记录失败时now、各源最近16条的stamp/receive/双龄、broker导出戳/序号、源图和原生step request/response墙钟，区别物理停顿、DDS/回调延迟与单边导出。若要减少单边payload，应显式发布两路同物理戳的完整bundle并记录ready/缺流；worker仍按同样TTL失败，不允许以等待、宽限或真值回填隐去真实源过期。此为待批准安排的诊断建议，本审计没有再修改broker/runtime或启动试验。

## 08:00资源空闲复测追加

`d6cd/dbb4/a843`各有901个policy帧与751个实际30维替换帧，三轮CPU单推理线程、真实callback重建、因果/双龄、外参/逐名关节、完整247重建与动作重放全部通过，actor最大误差分别1.55e−6/1.25e−6/1.55e−6。三轮runtime记录均为worker、Gazebo、bridge、capture、sensor_feedback五个自有进程exit0，worker/result无fault，50/200 Hz日志连续、原生最后阻尼/终止收据完整。raw保留和严格25 ms/300 ms门槛没有变。

与旧`080e`相比，world.sdf字节、冻结动作模型、原生.so、观测/契约/broker以及测试protocol全部相同；归档worker唯一diff是失败分支新增原broker payload、read-wall、候选双龄快照与详细报错，成功控制分支不变。独立重算5–10 s速度/误差与15–18 sTeacher零命令停车，符合原tracking、最低请求轴比例0.4、停车位移≤0.15 m/yaw≤0.2 rad/线速度RMS≤0.08 m/s/角速度RMS≤0.1 rad/s。停车最大过程位移也分别≤0.000634/0.000397/0.000673 m，不仅终点差合格。另从200 Hz实际姿态与完整场景下方碰撞几何重算离地和姿态安全，没有使用这些诊断值反馈导航或改变policy扫描。

三轮成功帧sim龄p95均0，max为5/5/0 ms，wall龄p95约6.6–6.8 ms，max约11.3/11.8/8.5 ms；激活后真实IMU/关节callback接收最大间隙约10.6/9.9/11.7 ms。同期GPU采样855–1090 MiB、3–21%，旧`080e`约8.1–8.3 GiB和较高GPU使用时出现427 ms长间隙。**负载降低与源时序/重复通过同时出现，仅证明相关；不能唯一归因GPU或宣布所有负载条件可靠。** 没有重训练或修改动作，也没有为通过删除旧失败。

中途`20261004_080553_forward_resource_idle_actual_sensor_v2_r2_fe74`的worker记录18 s/901帧、无fault，但runner清理收到SIGTERM143，原summary/runtime_manifest缺失，留下三个本轮ROS进程。独立读取[打断收据](../runs/20261004_080553_forward_resource_idle_actual_sensor_v2_r2_fe74/runner_interruption_receipt.json)：来源未建立，仅按精确本轮进程组SIGINT清理且退出。该轮标`runtime_unverified_not_counted`，没有加入三轮清洁来源/动作审计，没有虚构整体pass，也不将其视为Teacher物理失败。

## 保存记录与范围

- [v1独立JSON](../test_results/sensor_replacement_audit_20261004_v1.json)、[v2三轮独立JSON](../test_results/sensor_replacement_audit_20261004_v2.json)包含原日志/归档源码SHA、全部checks、因果龄、重建、动作与原运动结果；分析源码SHA `60e34c50a6db19ea5221f2e85fb5b6830fb59a6ceca4581b0cf9c61d22ef3d3c`。
- [资源空闲三轮来源/动作独立JSON](../test_results/sensor_replacement_audit_20261004_idle_v3.json)、[独立runtime/原阈值/停车/资源检查](../test_results/sensor_replacement_idle_runtime_20261004_v2.json)，分别保存原日志和新增分析源码SHA；只生成新审计结果，不覆盖历史JSON或原run。
- [v2完整通过c1a0](../runs/20261004_015357_forward_actual_imu_joints_replacement_v2_r1_c1a0/summary.json)、[确认通过882d](../runs/20261004_015446_forward_actual_imu_joints_replacement_v2_confirm_r1_882d/summary.json)、[末帧失败080e](../runs/20261004_015511_forward_actual_imu_joints_replacement_v2_confirm_r2_080e/summary.json)。
- [初版站立](../runs/20261004_013552_stand_actual_imu_joints_replacement_r1_bcdc/summary.json)、[初版前进失败](../runs/20261004_013614_forward_actual_imu_joints_replacement_r1_1687/summary.json)保持原样。
- [来源与替换矩阵](OBSERVATION_REPLACEMENT.md)、[地形目标/时序审计](TERRAIN_TARGET_TIMING_AUDIT_20261004.md)。

本次仅Gazebo的实际IMU orientation/gyro与关节订阅输入，未验证实体IMU姿态估计器；未替换SLAM COM速度或局部点云高度，未完成其他方向/坡道/低台阶传感器替换重复验收。三轮清洁资源空闲的前进及停车重复验收通过，历史墙龄失败和中断运行保留；有限范围通过不扩称所有资源条件或整个运动/感知/导航系统可靠性通过。
