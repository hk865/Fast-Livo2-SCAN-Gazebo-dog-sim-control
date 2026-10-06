# FAST-LIVO2 队列与算法并行实验 · 2026-10-05

本轮实际实现独立 V15/V16/V17/V18 候选并运行 Gazebo；没有修改通过的 camera_mode、其他训练、冻结模型或原控制安全门。Teacher 仍 CPU 单线程、50 Hz，关节 PD 200 Hz，原观测/命令双时钟 300 ms 保护保留。导航来自真实 SLAM/IMU/点云和 SCAN；Actor 仍有 232 维特权状态/高度，15 维命令与上一动作。当前不是全感知 Actor 部署。

## 瓶颈与实施

[实现与瓶颈说明](IMPLEMENTATION_AND_BOTTLENECKS.md)记录完整分析。当前算法的状态/协方差、bias、去畸变时间锚及地图有递推依赖，IMU→LIO→VIO仍由单一状态线程依次提交；加入队列不能消除依赖。可重叠的是输入接收/解码与主估计工作，可并行的是同一轮固定状态下的独立观测行或图像块。

- V15：LIO Jacobian 行构造按索引独立并行，约束数≥256时4线程；原表达式、原索引顺序、完整状态先验与求解保留。真实约6k行的有限算子基准耗时约降72%，不能解释为整条SLAM快72%。
- V16：VIO图像块构造重组并保留原序误差归约。最终选1线程，有限大算子中位耗时约降24–30%；4线程版本尾延迟门失败，保留失败。1线程收益不是多核收益。
- V18：独立合并V15行4线程与V16块1线程，重新通过组合状态/协方差/非计时输出逐字节回归后才运行物理实验。
- V17：单独后台SingleThreadedExecutor接收并解码，经512项/64MiB有界FIFO交给唯一主状态线程；保留原source header及接收时刻。后台解码仍可能短暂阻塞下次接收。ready队列限额不代表DDS、估计缓存或总RSS也受同一限额约束。

[实际210秒性能对照](performance_actual/REPORT.md)没有证明整链显著加速：V12→V18 LIO墙钟均值9.127→9.293ms，调用线程CPU8.833→8.568ms；VIO整帧2.555→2.545ms。实际源更新均约30Hz、源年龄p95均40ms。输入轨迹/地图不同，不能把差值归因单个算子。残差查询包含于LIO求解，不能相加；wall减threadCPU也不能直接当通信时间。Teacher推理本轮p50约0.32ms，暂无依据通过GPU推理解决前端瓶颈。

当前输入本身约30Hz，V12和V18均能跟上，没有本轮持续积压证据；因此“仍为30Hz”既不能证明极限吞吐不变，也不能证明上限提高。本轮没有提高输入率来测饱和点。通信与计算的严格独占比例尚未测全：已有阶段/thread-CPU和RX→commit测量，但缺完整DDS传输trace，不能把线程数或CPU空闲当作通信瓶颈证据。

## 实际运行与独立验收

结果以本目录evaluation与pipeline_runtime_audit独立收据为准；运行器的exit0和模型无故障不代表导航通过。

|运行|范围|已记录运行结果|独立结论|
|---|---|---|---|
|ac02：V18 prefix|210s，原32区域路线前缀|25/32；Teacher10501样本、fault=null、清理通过|必要源/保护/运动/路线门通过、第一条12m坡道通过；全路线/第二坡/停车未完成，原总体failed保留|
|ab53：V18 full|原32区域及两条12m上行坡道、固定5s停车|264.60s、13231样本、32/32状态、fault=null、清理通过|独立联合验收通过：32/32、两条12m坡、首固定5s停车；原common/v2/ramp未验证收据保留，新增metadata-v2只修归档字段来源关联|
|efde：V17 smoke|独立60s，原路线前缀|8区域、3001样本、fault=null、清理通过|全生命周期31615接收、31614提交、1取消，严格失败；已提交FIFO/源时间戳/不同RX与owner/容量/库绑定通过；原16项前缀运动/源/安全门及7项发布保护门通过；有限通过不抵消取消失败|

一次指定路线通过不覆盖原46区域全任务、当前配置动态障碍、真实感知Actor、新控制律Isaac闭环对照或真机。场景使用坡道连接三层，不能称真实楼梯通过。原相机Demo与全部旧失败保留。

组合完整运行的首固定5s停车窗口为257.345–262.345s：1001个原生200Hz采样，XY漂移2.14mm、yaw漂移0.00468rad、峰线速度0.01831m/s。路线最大偏离0.13891m、RMS0.03728m，速度MAE0.03828m/s。无机身接触或Teacher故障。新增来源联合审计SHA256为`1398f53ec33310c510944ba5100d62d67d205c7596250b972a89332bb57e5280`，原三份UNVERIFIED不被改名为PASS。

V17原始队列CSV连续1..31614，末行是timer且出队后pending为0；关闭队列后汇总却有1项取消。被取消的31615未记录类型或精确时间，因此只确认“close时有1项剩余”，不能断言它是timer。既定零取消验收失败，本轮不将它合入导航默认栈。下一步需要在新独立版本中明确停止接收、完成在途解码、主线程排空已接受项的收尾屏障，并记录被拒绝/取消项身份；保持本轮失败原件和原门，另跑新实验。

完整路线小采样的共同sim30–260s窗也未证明整链变快：FASTLIVO进程CPU约0.826→0.837核，主线程0.644→0.631核；阶段计时关闭，均标N/A。264.60s较旧运行完成更早，受实际运动和闭环轨迹影响，不能作为计算加速证据。

[完整路线独立报告](evaluation/actual_v18_full/FULL_REPORT.md)记录：SLAM原header实测30.29583Hz，最大间隔65ms；在实际控制行取样的位姿源年龄p50/p95/max为45/65/80ms，真实控制数学17.97408Hz（独立于20Hz墙钟ticker）。它与前缀中“位姿接收时年龄”是不同测量位置，不能混为一项。

[V17运行期与取消诊断](pipeline_runtime_audit/actual_efde/README.md)记录有效LIO/VIO约30Hz，LiDAR/RGB/IMU接收至提交p95分别0.738/4.109/15.585ms；队列峰值15项/4211680 bytes。这里只是该60秒运行的测量，不是同输入A/B加速证明。原实际收据SHA256：`80efc11ed7c5537c7ef70493f58527db5b8f28ef20cdcfe3bcc2e2dd5df4af20`。

[V17原运动与保护独立报告](evaluation/actual_v17_smoke/SMOKE_REPORT.md)另行绑定原common/v2前缀必需门全部通过。原管线收据中的delegated-native UNVERIFIED保持原字节；新增报告闭合该来源，但不改变已失败的零取消门。60秒只完成8个区域，不声明完整路线或最终停车。

## 可复现入口

从项目根运行。必须先确保只有本任务一个仿真控制器，且所有离线大基准/全包哈希已停止。`--label`须使用未存在的新名称；运行器按原保护与既有清理合同执行。以下命令会实际启动仿真，仅用于已授权仿真范围。

```bash
python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/launch_observed.py \
  --candidate combined_compute_v18 --profile combined_lio4_vio1_l64_r30_c30_210 \
  --label combined_v18_prefix_r2 --domain 86

python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/launch_observed.py \
  --candidate combined_compute_v18 --profile combined_lio4_vio1_l64_r30_c30_600 \
  --label combined_v18_full_r2 --domain 86

python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/launch_observed.py \
  --candidate ingress_pipeline_v17 --profile l64_r30_c30_ingress_smoke60 \
  --label ingress_v17_smoke_r2 --domain 90
```

V17是失败保留的实验版本，尚未作为默认导航栈。其pure preflight证明有限接口语义，不替代实际队列生命周期通过。当前根盘空间不足以继续无界保存新全路线，先检查空间并保留既有证据。

只读回放：

- [V18完整运行](http://127.0.0.1:8768/?run=20261005_234102_closed_loop_cascade_clock_hold_combined_v18_full_r1_ab53)
- [V17队列试验](http://127.0.0.1:8768/?run=20261005_234627_closed_loop_cascade_clock_hold_ingress_v17_smoke_r1_efde)

恢复服务命令：`python3 -B multifloor_demo/teacher_mode/scripts/serve.py --host 127.0.0.1 --port 8768`。不得重复启动已占用端口。回放页仅监听本机且拒绝写入请求。

## 证据与局限

离线证据分别在`lio_v15/`、`vio_v16/`、`combined_v18/`、`pipeline_v17/`；正式批次、较早先导、构建/fixture失败全部保留。V15历史HTH重建有约3–4e−8差异，原因未验证，不能把有限逐字节回归称完整历史回放。V16 T4性能失败保留；V17退出时未提交项不能被事后解释成通过。

新实际payload从创建起位于`/var/tmp/go2_teacher_parallel_20261005/`，项目runs只有同名alias。最终独立外部清单与项目PACKAGE_MANIFEST需同时核对；不能只复制alias。旧`/var/tmp/go2_teacher_simulation_20261005/`和其封存清单未修改。旧current_status/README原字节在`prior_status/`，不回填历史结果。

冻结模型SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`。每次实际执行绑定自己的源码快照、模型/config/runtime哈希；冻结纯验证合同中的NOT_RUN为当时前置证明，不是可覆盖的最终结果字段。

封存的新外部库存包含50,288个文件、41,677,218,574 bytes，SHA256 `cf2ab4991af32eb39830dfc766c0fc5ced18c324c90ce45b507739fe109e12cf`，见[EXTERNAL_STORAGE_MANIFEST.json](EXTERNAL_STORAGE_MANIFEST.json)。包含1次prepare-only及3次实际运行，不把准备目录计为物理测试。
