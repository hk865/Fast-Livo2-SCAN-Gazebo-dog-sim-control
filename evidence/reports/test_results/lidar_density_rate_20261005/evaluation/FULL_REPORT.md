# 冻结Teacher：64线30 Hz真实SLAM/SCAN双坡道完整路线

本次实际仿真完成原始32个三维区域、两条完整12 m坡道及首个声明的固定5 s停车。模型仍在CPU单线程推理，导航反馈来自真实传感器SLAM/IMU及真实SCAN路径。此结论限定本次冻结场景和控制配置，Actor仍使用特权仿真观测；动态障碍停车恢复、全247维真实传感器替换及真机均未由本次证明。

## 实际run及证据结论

真实目录：`/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb`。项目runs同名路径只是symlink别名。600 s是事前上限，实际worker运行至275.18 s、13760个50 Hz样本、fault=null；不能称跑满600 s。原始第32区域dwell为260.045–260.705 s，首declared active-hold的固定native窗口为268.145–273.145 s。

| 验证层 | 本次结论 | 实际依据和范围 |
|---|---|---|
| Teacher接口/执行 | PASS | 原247维输入13760×247、模型/命令/目标角/PD力矩独立回放、显式关节映射与唯一JointForceCmd writer、CPU线程1 |
| 运动控制 | PASS | 原速度/航向/路线tube、安全/接触/关节保护门通过；5 s主动速度hold停车通过 |
| Sim2Sim | 本冻结配置通过 | 两条各12 m真实坡道完整COM出入口、四足有序坡面支持、全12个米段、落地区支持均通过；不是楼梯通过 |
| 导航集成 | 静态32区域双坡路线联合证据PASS | 原真实SLAM三维到达、实际SCAN payload、source时效/guard、PI与每次实际发布slew全部通过；独立metadata v2修正审计补齐terrain-provider因果来源 |
| 尚未由本次验证 | UNVERIFIED | 动态障碍停车恢复、重复运行可靠性、全真实传感器Actor输入、其他路线/速度（包括1 m/s）、GPU推理A/B、真机 |

导航控制没有使用Gazebo真值。native位姿仅在离线验收配准/接触检查以及Actor特权输入中使用。Actor共232维特权观测、15维已知命令/上一动作，不能把本轮写成全感知观测部署。力矩为实际传入仿真物理的限幅JointForceCmd，不是硬件电机力矩测量。

## 来源修正：保留旧UNVERIFIED，追加独立闭合结果

原三份收据保留原字节：

- `summary_closed_loop_cascade_independent.json`：UNVERIFIED，仅`strict_nonflat_complete_contact_geometry`交给独立坡道验收；其余18门全部PASS。
- `summary_closed_loop_clock_hold_independent.json`：UNVERIFIED，仅同一非平地适用性门；原门与全部七个publication/clock-hold附加门PASS。
- `summary_closed_loop_ramp_independent.json`：UNVERIFIED，仅`original_source_authorized_causal_terrain_layer_switch`因整条归档row比较而未连接；两坡道及最终landing/停车的其余24门全部PASS。

只读分析证明这是评估器归档字段比较不匹配。冻结`RecordedPublisher.publish`先将原`data`完整写入history并额外加`monotonic_wall`、`ros_sim_time`，再将同一原`data`写入atomic状态JSON。provider实际读的是atomic JSON，原terrain评估器却要求history整row与该payload完全相等。真实history中有唯一一条全部74个原payload字段精确匹配；仅两个归档字段额外存在。

该原row ROS clock131.83 s、wall449588.381504006；provider实际读取449588.39111505–449588.391601091；switch事件wall449588.396736713。原source arrival131.675 s、native request131.850 s、effective physics131.845 s，原因果关系未改。新增只读审计只对history做虚拟metadata投影；没有修改日志、控制器、原收据、时效或数值门。随后执行冻结原`terrain_switch_audit`的其他全部条件：两provider187条射线各回放误差0、共享floor_2、原native样本、请求/区域/来源哈希及Actor-only使用全部PASS。

新增收据：

- `summary_closed_loop_terrain_metadata_join_independent.json`：v1，保留归档字段精确匹配和原terrain完整回放；初版来源闭合不足的范围如实保留。
- **`summary_closed_loop_terrain_metadata_join_independent_v2.json`：PASS**。精确检查common19、publication-v2共26、ramp25个mandatory门及`status/ passed`旗标；原common检查deep equality，只有事前允许的一个PI门可替换。重新hash三份原收据的全部来源map，包括15个附加publish/hold输入与6个小型source绑定，未遗漏账本门。

v2收据SHA256：`b07b0a4b4b5b80cbd1a48201e745d0241596fdba8c56b93d1cb6da5f85919d43`。v2 reader SHA256：`cb2e8aa4b383f5feb26b7ba154158062a289d266b081d2d76e417944c9267c94`。9组有限负例通过：真实payload改变、缺原字段、重复match、非因果wall/source clock、其他坡道门failed、缺publication/common门、额外输入字节改变均拒绝联合PASS。这是明确的事后只读bookkeeping修正，不假称旧三份收据已经PASS。

## 运动及停车数值

原路线离线native对齐只用一个原始锚点：tube最大0.139573 m、运动段RMS0.038518 m、drive最大航向0.147896 rad、速度MAE0.039399 m/s；实际沿路线COM均速0.169609 m/s，参考均速0.190263 m/s，均通过原门。绝对原场景中心线配准字段仍明确UNVERIFIED；没有将body-relative验收冒充新的绝对全局配准结论。

首固定5 s停车1001个native/152个SLAM样本，native XY drift5.38 mm、yaw drift0.000596 rad；SLAM XY drift5.92 mm、yaw drift0.000803 rad；origin速度峰0.009892 m/s、body wz峰0.001469 rad/s。停车是固定终点主动速度hold，没有将action=0用作停车。

4989次原始几何guard独立回放通过，最大guard source间隔155 ms；300 ms双时效没有放宽。实际5626次发布与writer及原命令计数一致，5341条关联PID、285条PID之外发布（含25个clockhold）；4757次slew回放误差0。4747个实际新源数学更新独立PI回放最大误差1.628×10⁻¹⁵；原13759条ACK按原生产者clock连接。

## 实际频率、完整性与高度

已接受真实SLAM8228个独立headers、平均30.2998 Hz，最大源间隔65 ms、无>300 ms缺口；post-anchor source age p50/p95/max为30/40/60 ms。20 Hz控制ticker与实际PI数学频率分开：4747次更新、17.6836 Hz，包括初始化、朝向、停车及冻结hold阶段。

完整LIO/VIO processed-handler与raw输入频率在`comparison_v12_full/<run basename>/summary.json`中记录。优化器复制诊断仅115–118 s，91/91帧、30.2521 Hz；窗口外数学输出未采集，是N/A。full profile明确关闭kind300，故本轮边界wall/CPU为N/A，不报0耗时；阶段成本使用此前实际210 s V12/V14的独立kind300对照。

诊断writer final，198244/198244、dropped=0、IO失败false；PID/pub writer drained。native连续200 Hz，身体接触/物理fault门通过。全post-anchor SLAM-native相对Z最大108.35 mm，未重现原20 cm突然降低；这不能证明所有环境的高度偏差已消除。

## CPU、GPU与外层采样退出

外部只读CPU sampler从runner启动开始，143条完成记录；其首条只有runner/worker，稍后启动的子进程必须按每个原PID/starttime与TID/starttime独立首次到末次计数，不能用全体首尾交集漏掉SLAM。以下是原/proc观察窗口，含启动至物理末端，不假称每个进程都有完全相同生命周期：

| 已由runtime角色绑定的实际进程 | CPU s / observed wall s | 平均核 |
|---|---:|---:|
| FASTLIVO387054 | 224.76 / 280.00 | .803 |
| 其中main TID | 175.81 / 280.00 | .628 |
| SCAN387057 | 184.36 / 280.00 | .658 |
| Gazebo角色387147（ruby） | 172.89 / 280.00 | .617 |
| Teacher worker386994 | 93.79 / 282.00 | .333 |
| sensor bridge387012 | 44.44 / 280.00 | .159 |
| capture387013 | 34.12 / 280.00 | .122 |

worker整进程CPU含观测/IO等，不等于模型inference。未有实际argv角色证明的其他python3不直接命名为通信。名为dds.*的实际线程累计5.57 CPU s只覆盖这些已命名线程；不含主线程序列化/RMW及off-CPU时间，不能据此称“全部通信2.5%”。V12/V14实测对照显示分核mask正确，但未减少主线程非自愿切换或匹配wall成本，因此full选择V12原调度；无GPU模型A/B结论。

另一训练、浏览器和只读viewer不属于runner PPID descendants，未计入上述owned CPU。环境没有CPU/GPU/内存/磁盘隔离。shared GPU利用率p50/p95/max约13/19/24%、显存约962–1126 MiB；这是系统共享读数，不是Teacher或SLAM独占读数。

独立采样外层工具退出143，而模拟runner退出0。`external_cpu_sampler_tool_outcome.json`保存root实际tool outcome、143及原因UNVERIFIED；completion文件143条完整、自身2.462 CPU s、最大周期18.4 ms。不得将外层143写成clean0，也没有证据将其归因物理fault或已证实runner cleanup。仿真生产进程清理及native安全由各自独立证据判定。

## 模型、软件与复现

Teacher SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`，未重新训练。实际加载core SHA256：`8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed`，加载二进制见证verified。CPU Actor1、50 Hz；native step0.005 s/decimation4、PD25/0.5、限矩23.5 Nm，关节名称映射与初始姿态以run原合同为准。source/config/model及输出文件的最终全hash清单由root封存。

准备新独立仿真目录（不会执行物理）：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
python3 -B multifloor_demo/teacher_mode/navigation/lidar_sampling_v12/run.py \
  --profile multifloor_demo/teacher_mode/navigation/lidar_sampling_v12/profiles/l64_r30_c30_600.json \
  --label full_repeat --run-storage-root /var/tmp/go2_teacher_simulation_20261005 \
  --prepare-only
```

实际仿真启动使用同一命令去掉`--prepare-only`，由runner维护单执行器、时效和owned进程清理；不要手动叠加第二控制器。`camera_mode`与其他训练任务不在本改动范围。

新收据只读复核：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/evaluation/audit_terrain_metadata_v2.py --self-test
```

已有收据禁止覆盖；reader对已有同名append收据拒绝重写。完整height/command图、实际native/SLAM轨迹CSV、`performance_proc_observed_windows.json`及版本化诊断在`comparison_v12_full/`中。浏览器新增独立metadata-v2卡片已显示5/5通过，原common/v2/ramp与主选原收据仍显示UNVERIFIED。75项来源/显示边界通过；另11项JS功能验证证明同run/同SHA保留展开、切换run或SHA时不继承。root唯一只读服务重启后，实际核对卡片跨刷新展开、两路Gazebo画面、真实SLAM8228条/SCAN轨迹86与相机D/K；观察记录见[浏览器核对](../viewer_audit_metadata_v2_refresh_revision/root_actual_browser_verification.json)。不能把新结论回填旧收据或未运行场景。

外部物理目录由[全文件库存](../EXTERNAL_STORAGE_MANIFEST.json)封存：65,469文件、55,004,007,938 bytes，SHA `9c5847719284843d55919b79f1933012837f2bb6c5aa27125400ac201df23af7`；复制包必须带上外部payload。新队列/算法并行的[工程审计](../pipeline_audit/QUEUE_ENGINEERING_AUDIT.md)与[算法审计](../pipeline_audit/ALGORITHM_PARALLEL_AUDIT.md)另列未实施候选，本轮通过不代表这些候选已经运行。
