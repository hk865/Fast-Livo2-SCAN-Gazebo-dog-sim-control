# 2026-10-05 计算/通信拆分与 Teacher–SLAM 续接

仅仿真；冻结 model_1000.pt SHA256 bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34。不得重训、操作真机、改通过的 camera_mode，或停止其他任务的训练/评估。Actor CPU 单线程50Hz，唯一原生TeacherActuator PD200Hz，300ms双时效与停车保护不放宽。

## 当前实现

- V8采样、V9时钟hold、V10两线程、V11诊断减负/完整发布账本历史失败均原样保留。
- V12独立普通副本仅LIO residual循环4线程，packed vector<bool>改为每点byte独立槽，移除对应共享锁，按原点序合并；VIO保持原顺序。冻结core 8303ec94d33a976bc8ac0940e9d3d213120ce4a035ea22385e07f301c019efed。41控制保护测试、18有限LIO/VIO数值及3计时开关对照通过；大点数有限基准4线程优于8线程，跨P/E最大2.22e−18差异如实保留。
- V12真实30/30Hz210s前缀25/32，源年龄max60ms、有效数学17.435Hz，第一12m上行坡道正式通过；完整任务/最终停车当时未验证。
- V14相同V12 core，分开Gazebo P0–3/SLAM P4–7/Actor E18/controller E19/其他自有节点E8–17，真实mask核对通过；25/32前缀、源年龄max65ms。共同窗口没有调度/吞吐收益，主线程抢占反而多。只隔离本任务mask，不独占其他系统/训练、GPU、内存和磁盘。
- V13同IMU区间/完全相同dt的deskew变换缓存只有源码，未构建/数值/实际验证，prepare强制拒绝。没有实现Forster预积分或完整IMU流水线。

## 最新完整路线

实际run：/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb

profile l64_r30_c30_600 SHA af99c5f4fc8bffd9a6e357cd52904ec8c85e0a4cd4741279f4e1c385de7fb897，600s是上限；实际275.18s/13760次CPU推理，worker fault=null，32/32 succeeded后主动终点保持并退出。runtimeSHA bc5f140503a1be578e5ee6ad9c899cc54deb80a8154a139c1622599863cd5162，runner exit0。新增只读metadata-v2联合审计PASSED，SHA b07b0a4b4b5b80cbd1a48201e745d0241596fdba8c56b93d1cb6da5f85919d43，完整19/26/25+7必需门与48来源字节闭合；两条12m上行坡道/landing/首固定5s停车通过。停车XY drift5.38mm/yaw0.000596rad。原common/v2/ramp仍UNVERIFIED并原字节保留；唯一新增是经过冻结producer源码证明的两个归档字段投影，不改任何原payload/数值/路线/来源门。9组负例及第二agent独立复核通过。kind300本轮off，段耗时必须N/A，不能填0。原始IMU/state/cov/传感器/SCAN/真实导航云和实际发布账本全程保留，详细solver仅115–118s。

真实导航反馈仅SLAM/IMU/点云+SCAN，Gazebo真值仅离线验收。Actor仍232维特权状态/高度+15维已知命令/上一动作。此轮没将真实30维替换与导航组合；不能宣称完整真实传感器部署或Sim2Sim闭环对照通过。

CPU只读sampler从runner startup开始143条，后启动SLAM按实际PID/TID首次出现累计，2.462s自身CPU。completion存在但外层工具exit143，原因未验证；独立记录external_cpu_sampler_tool_outcome.json，不与runner物理exit0混淆。其他CPU/图形任务与只读viewer不在owned汇总，系统未隔离。

## 已测瓶颈与下一步

V12 kind300前缀的LIO StateEst50.71s含query29.33s，VIO14.07s，MapUpdate3.35s。publish9.07s含同步转换与RMW调用，不能全称通信；spin_some32.07s含callback/preprocess，wall与threadCPU差9.06s不是纯通信。各段inclusive嵌套、分母不同、并发processCPU不能直接相加。主导可见工作是匹配/估计、视觉与callback数据构造，IMU传播/deskew较小。

并行边界：固定只读地图上的独立点/patch可按索引并行，原顺序归约；统一state/cov的LIO/VIO校正保持单writer；map读写重叠需immutable版本/生命周期；接收预处理可与估计分离但保留原FIFO与同步切分。下一独立候选为同dt变换缓存、每solve/frame prior inverse缓存、patch const引用、保持全部初始化语义的buffer容量复用。只resize保留旧normal会污染视觉地图，不可盲改。

用户新增引用的讨论已与本地源码逐项核对：test_results/lidar_density_rate_20261005/pipeline_audit/QUEUE_ENGINEERING_AUDIT.md和ALGORITHM_PARALLEL_AUDIT.md。工程优先独立接收/解码、按timestamp单assembler、单估计器、不可变发布快照；算法优先H行与VIO patch按索引并行、原顺序汇总。分块正规方程与IMU Delta/FQ前缀扫描另立数值合同；没有构建或实际部署这些新方案，不把V12 full当其验证。

局部预积分Delta可在相同测量区间/偏置线性化下与绝对起始位姿分离复用；当前绝对IMUpose依赖起始state。改变bias/time切分需校正或重算，状态/gravity/cov应用也须更新。不是所有SO3/SE3矩阵运算都可一次复用，当前迭代pose依赖的plane association、投影和Jacobian必须相应重算。引入局部预积分或scan组合属于新结构/浮点顺序，另做精度与实际闭环验收。

## 操作

从teacher_mode目录，在无其他本任务实际执行器时启动新run（domain88若占用会拒绝）：

```bash
python3 -B navigation/lidar_sampling_v12/run.py --profile l64_r30_c30_600 --label lidar64_30hz_rgb30_lockfree4_full --domain 88 --run-storage-root /var/tmp/go2_teacher_simulation_20261005
python3 -B scripts/serve.py --port 8768
```

8768只读viewer PID649260保留，可显示两路实际Gazebo画面、相机640×480/fx=fy381.36/D全0及真实SLAM/SCAN图。新增独立归档字段关联审计卡片已显示metadata v2 PASS5/5，原三份收据UNVERIFIED保持。75项显示边界及Python/JS语法通过；追加11项JS跨刷新展开状态测试通过。仅自有viewer确切PID两次SIGTERM重启，均intentional tool exit143，物理/训练未操作。实际浏览器核对记录于test_results/lidar_density_rate_20261005/viewer_audit_metadata_v2_refresh_revision/root_actual_browser_verification.json。不要另起第二个8768；旧相机Demo8767不动。浏览器新完整run链接：http://127.0.0.1:8768/?run=20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb

新日志从创建起位于私有0700外部root，项目runs仅同名alias；没有搬迁旧日志。外部payload不嵌入项目，复制部署研究包须同时复制外部目录，核对EXTERNAL_STORAGE_MANIFEST.json和PACKAGE_MANIFEST.json。外部库存65,469文件、55,004,007,938 bytes已封存，SHA9c5847719284843d55919b79f1933012837f2bb6c5aa27125400ac201df23af7。所有本任务physics/审计writer已结束才封存；原42轮全局acceptance与完整Sim2Sim、原46区域/当前动态障碍/全感知Actor/真机状态不提升。

详细证据见LIDAR_DENSITY_RATE_REPORT_20261005.md，test_results/lidar_density_rate_20261005/evaluation/及thread_scaling/COMPUTE_DEPENDENCY_AUDIT.md。旧criteria、原42轮失败和所有早期运动/高度/停车失败均不回填。
