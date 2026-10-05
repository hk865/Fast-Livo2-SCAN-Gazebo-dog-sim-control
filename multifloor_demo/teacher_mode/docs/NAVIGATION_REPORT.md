# 有限SLAM/SCAN导航独立审计（2026-10-04）

V4三轮**有限平地区域往返通过（3/3）**，最新V4.5a dc6f又完成一次**限定平地动态障碍停车、连续点云清障、新SCAN恢复、两个原始SLAM区域与最终Teacher停车的严格闭环验收（19/19 passed）**。真实SLAM/SCAN产生非零命令，CPU Teacher实际行走/转向；原目标中心相隔1米而机身最大离起点约0.83–0.86米，不能称精确1米平移。先前站立失败、动态失败和1536原XYZ覆盖不足总项都保留。整体Sim2Sim仍failed，完整坡道、多楼层、无界动态任务、原46区域任务与真机仍未因此通过。

## 范围与证据

真实运行目录：[20261004_012446_navigation_slam_scan_roundtrip_first_r1_02f7](../runs/20261004_012446_navigation_slam_scan_roundtrip_first_r1_02f7)。独立收据：[summary_navigation_independent.json](../runs/20261004_012446_navigation_slam_scan_roundtrip_first_r1_02f7/summary_navigation_independent.json)；只读分析器：[analyze_navigation.py](../scripts/analyze_navigation.py)。没有改导航源码、发布ROS消息、重放控制或启动额外仿真，原始日志和通用摘要保留。

本轮 `navigation_scope.json` 仅授权已测平地起点 `[6,−.7,.4,0]` 的有限实验，仍为 `experimental_unverified`，整体Sim2Sim保留failed。Teacher的247维运动观测仍为Gazebo特权输入；导航anchor、目标、轨迹执行反馈和到达区域只来自传感器SLAM。一次冻结的SLAM起点和SLAM航向定义一米前方与返回原点，未调用仿真真值建立导航目标。

区域沿用预先冻结的disc prism：外半径0.22米、高度±0.10米，停车控制半径0.17米，连续原始SLAM时间戳停留0.6秒且单次缺口≤0.2秒。bridge及worker保持0.30秒新鲜度门，不能通过增大超时、计时走完或只画路线来宣称到达。仿真真值仅另行用一次固定变换进行记录后的误差评估，从未送回导航。

## 首轮实际结果

| 证据 | 实际记录 | 判定含义 |
|---|---:|---|
| Teacher运行 | 120秒、6001次50Hz策略样本 | 推理与站立运行，不代表导航运动 |
| 非零平移指令时长 | 0秒 | 没有真正执行导航行走 |
| 实际XY最大偏移 | 0.01731米 | 初始化/站立小幅运动，不能当一米前进 |
| 原始SLAM body pose | 1027条；周期中位0.1秒、最大0.4秒 | 实际定位已运行，仍有新鲜度缺口 |
| 控制器接受的注册点云输入 | 668条 | 保存实际stamp、frame、点数、近邻SLAMstamp和XYZ缓冲哈希 |
| 传感器门 | 17条ready、1270条hold | 整段未保持执行就绪 |
| 命令bridge | 1条ready、1324条hold；所有指令为零 | 未形成持续运动控制链 |
| 已接受SCAN轨迹 | 0 | 不能算路线跟踪 |
| 原始SLAM区域到达 | 0/2 | 往返未完成 |
| 机身接触/缺失接触 | 0/0 | 站立安全通过 |
| 200Hz最大滚转/俯仰、最小clearance | 0.11396rad、0.27568米 | 站立安全通过 |
| worker/Gazebo/bridge/capture/shadow/navigation_stack | 退出码全部0 | 正常结束，不代表路线成功 |

传感器预热反复重启；日志中857条等待连续预热，149条单独报告车载图像与CameraInfo未在50ms内配对，另有点云/位姿/时钟新鲜度等待。采集回调异步下，单独比较“最新图像”和“最新CameraInfo”会把实际上有匹配时间戳的消息暂时报成未配对；后续接口修复需新run，不能修改这轮失败。真实缺帧也继续保持0.30秒停车门。

SLAM一次冻结anchor可在原始位姿日志中精确找到，其一米前方和返回目标坐标与SLAM航向计算一致。导航仍停在首目标index0，最新状态为等待Teacher执行门或传感器预热、停车。未产生trajectory数组或到达region，分析器明确给有限路线failed，而不是把站立测量包装成融合完成。

## 第二轮接口修复后的实际结果

[V2运行目录](../runs/20261004_013855_navigation_slam_scan_roundtrip_v2_r1_e591) 保存修复后的真实图像/CameraInfo配对、一次完成的初始预热、失效恢复的新鲜位姿历史。新鲜度仍为0.30秒，未修改首轮失败。独立结果为 [summary_navigation_independent.json](../runs/20261004_013855_navigation_slam_scan_roundtrip_v2_r1_e591/summary_navigation_independent.json)。

120秒/6001条Teacher样本、全部六类进程退出码0、200Hz站立安全通过。传感器门ready779/1257，命令bridge ready880/2188，1081条原始SLAM位姿、686份控制器接受的注册点云；87个真实SCAN样条的coefficients、knots、samples、metadata保存齐全，数组SHA和payload逐项相等，独立三次B-spline重构一致。这证明规划接口已实际接通，不能证明轨迹已执行。

全部Teacher速度指令仍为 `[0,0,0]`，实际XY最大偏移0.01731米；往返region为0/2，首航点90秒期限到后停车。status中526条`pre_turn`、22条`settle`、0条`drive`。bridge健康连续快照跨度最长0.760仿真秒，短于既有HeadingGate前制动1秒、对准后停稳1秒要求。bridge hold被共享保护门处理为`execution_bridge`保护，恢复后重置HeadingGate/轨迹；`tilt_stops`累计100次。该计数不能解释为100次机器人倾倒，首航点运行期间rawIMU最大倾角约0.0255rad、全程原生姿态最大0.114rad。传感器门本身最长ready5.635秒，bridge的时钟、位姿和命令年龄检查还造成额外断续。

1934个实际worker接受帧可逐条匹配原始command sequence和canonical JSON哈希，记录的sim age与world clock差一致，wall age因JSON读取稍晚于`read_monotonic_wall`而不小于其因果下界；所有接受帧满足0.30秒门。4034帧有效但健康/时效不满足、33帧读取拒绝/缺失；这些帧requested均为零。仍未出现“非零行走→过期→实际停车”事件，因此TTL物理停车继续unverified。

CPU推理p50约0.269ms、p95约0.336ms，单帧最大56.0ms；因此不能宣称所有实时推理均小于1ms。原生物理步长及策略模拟时间频率仍连续。V2独立真值配对误差p95约0.00256米仍是站立测量，不能称移动SLAM精度。

## 第三轮半速诊断

[rtf05运行目录](../runs/20261004_014409_navigation_slam_scan_roundtrip_v2_rtf05_r1_2970) 保持V2导航源、新鲜度0.30秒、物理步长0.005秒、decimation4、50Hz模拟时间策略，仅设置仿真运行速度0.5，扩大记录进程壁钟时限。独立结果为 [summary_navigation_independent.json](../runs/20261004_014409_navigation_slam_scan_roundtrip_v2_rtf05_r1_2970/summary_navigation_independent.json)。

仍完成120仿真秒/6001策略帧、所有六类进程退出码0，命令全部为零、原生站立轨迹与前两轮一致。1037条SLAM位姿和659份注册点云已实际记录；传感器门ready0/2459，bridge ready0/7027，初始2秒模拟时间连续预热未完成，没有冻结的SLAM anchor/request，也无SCAN轨迹和region。日志1529条等待连续初始预热、553条单独报告cloud stale。半速使10Hz模拟时间传感器周期接近0.2秒壁钟，真实点云漏帧或延迟仍可超过既有0.30秒门；本轮未恢复执行就绪。

实际worker读取时标测得120秒模拟时间跨243.09秒壁钟，比例0.49365；四个30秒区段为0.4866、0.4952、0.4968、0.4962。前两轮同法测得0.94975和0.97360。慢速诊断不能称实时导航通过，改变wall timeout也不能将本轮追认为路线成功。

| 实际运行 | 传感器门ready/总数 | bridge ready/总数 | 已接受SCAN | Teacher非零平移命令 | 原始SLAM到达 | 有限导航 |
|---|---:|---:|---:|---:|---:|---|
| 首轮 | 17/1287 | 1/1325 | 0 | 0秒 | 0/2 | failed |
| V2 | 779/1257 | 880/2188 | 87 | 0秒 | 0/2 | failed |
| V2半速 | 0/2459 | 0/7027 | 0 | 0秒 | 0/2 | failed |

三轮只读时序收据均保存为各目录 `navigation_timing_audit.json`，记录原始输入哈希、实际执行速度、连续就绪快照、worker envelope来源和年龄检查，未覆盖原始日志、通用摘要或global acceptance。

## 08:01空闲资源下的原V2复测

[空闲资源baseline](../runs/20261004_080143_navigation_resource_idle_v2_baseline_r1_137e) 正常完成120仿真秒/6001策略帧，85份真实SCAN、0秒非零Teacher平移指令、0/2 region到达，实际XY最大偏移仍0.01731米。六类进程退出码全0。接口、真实SLAM流与规划通过，实际Teacher导航运动、区域到达失败；TTL物理停车和到达后停车继续unverified。新 [独立收据](../runs/20261004_080143_navigation_resource_idle_v2_baseline_r1_137e/summary_navigation_independent.json) 没有覆盖原摘要。

[只读资源对照](../test_results/navigation_resource_comparison_20261004.json) 核对了实际文件：navigation全部、policy全部和原生执行器源SHA与夜间V2一致，共享navigation/core及SCAN二进制引用SHA也相同；profile、scenario、FAST-LIVO2与相机YAML完全同SHA。runner与prepare已有可选RTF/运行记录改动，但生成world去除本次路径后，唯一文本差为`real_time_factor`的`1`与`1.0`，数值相同；统一这一数字拼写后world完全一致。因此这是原V2控制语义的资源环境复测，不能笼统称所有源码字节都相同。

GPU运行前快照由8090MiB/74%变为944MiB/10%；相对独立shadow订阅同一SLAM stamp的bridge回调差如下。差值由bridge记录时刻减`pose_wall_s`恢复回调时刻，再减shadow同stamp实际接收时刻，不使用真值；负值表示bridge早于shadow，不表示未来消息。

| 同stamp独立接收对照 | 夜间V2 | 空闲资源V2 |
|---|---:|---:|
| 匹配并去重的SLAM stamp | 612 | 640 |
| bridge回调相对shadow中位差 | 91.36ms | 20.32ms |
| p95差 | 487.77ms | 488.86ms |
| 最大差 | 819.24ms | 892.02ms |
| 差≥300ms数量 | 124 | 119 |
| 最长bridge健康快照跨度 | 0.760s | 1.780s |
| 最长sensor gate ready跨度 | 5.635s | 14.200s |

空闲轮sensor ready991/1236、bridge ready1945/3261；95次保护恢复，状态快照`pre_turn`494、`settle`51、`drive`3。然而状态名称出现drive也未产生任何Teacher非零命令，不能作为实际行走证据。资源减负改善了部分中位延迟与连续就绪，但尾延迟仍存在且路线仍失败。这支持“GPU忙不是已证明的唯一原因”，不证明GPU负载没有影响，也不是受控的单一GPU因果试验。sensor_gate云回调与shadow同stamp的p95差仅约0.15/0.17ms，延迟较明显的链路是bridge位姿订阅，后续接口修复仍需另存实际run。

独立评估器在候选新版本前补充了实际command envelope逐帧哈希/年龄/sequence、请求到actor命令slew和247D观测输入、连续推理及真实停车窗口关联。先前三轮脚本、报告和收据已封存在 `test_results/navigation_independent_audit_20261004/first_three_frozen/`；新逻辑以只读方式复查先前三轮仍失败、原收据bytes不变。缺失历史`actor_inferred_this_frame`字段记unverified，未把历史接口证据改为失败。

## V4真实有限平地闭环通过

V4通过异步证据/命令写入、最新传感器订阅和后台完整性检查，保留原始command获取时间而不以写入时刻刷新0.30秒门；Teacher转向阶段使用真实SLAM停稳证据，保护解除后不会反复丢失已完成的停稳过程。运行代码冻结于 [navigation_v4_freeze.json](../test_results/navigation_v4_freeze.json)。这不是重新训练或修改Teacher，不扩大region、新鲜度、姿态或停车阈值。独立评估器也在本轮结果评估前冻结，首轮与重复轮用相同新进程加载版本，SHA为 `5ed1af467538af3eb96239d456d2e5f63bbf0df74a3a4cee908c635d917effc0`。

| 实际指标 | V4首轮9c33 | 同设置重复1c57 | 同设置重复bd19 |
|---|---:|---:|---:|
| 持续运行/策略帧 | 180秒/9001 | 180秒/9001 | 180秒/9001 |
| 完整真实SCAN payload | 6 | 7 | 11 |
| Teacher非零平移命令时长 | 33.44秒 | 22.00秒 | 29.10秒 |
| 原生实际XY最大离初站偏移 | 0.84727米 | 0.83407米 | 0.85801米 |
| 外行region原始SLAM dwell | 50.8..51.4秒，7条 | 25.4..26.0秒，7条 | 57.0..57.6秒，7条 |
| 返回region原始SLAM dwell | 133.8..134.4秒，7条 | 92.3..92.9秒，7条 | 104.1..104.7秒，7条 |
| 连续零命令Teacher停车评估窗 | 137.53..140.53秒 | 96.155..99.155秒 | 107.82..110.82秒 |
| 3秒停车最大XY漂移 | 0.001726米 | 0.000581米 | 0.000241米 |
| 3秒停车偏航漂移 | 0.038706rad | 0.014047rad | 0.010524rad |
| worker实际stale或unhealthy标记帧 | 2697/9001，29.96% | 2801/9001，31.12% | 3347/9001，37.18% |
| 后验固定SE3因果配对位置误差p95 | 0.00955米 | 0.00766米 | 0.00816米 |
| 实际模拟时间/壁钟比 | 0.99338 | 0.99318 | 0.99322 |
| worker/Gazebo/bridge/capture/shadow/navigation_stack | 全部退出0 | 全部退出0 | 全部退出0 |
| 独立有限导航/评估错误 | passed/0 | passed/0 | passed/0 |

每个区域的start/end时间戳都在原始SLAM日志精确存在；7条位姿全位于预冻结0.17米控制半径、高度±0.10米内，最大间隔约0.1秒，连续0.6秒且未保护。首轮到达时距区域中心分别0.1568米和0.1515米，说明“区域到达”不等于落在目标中心。Teacher指令逐帧关联到真实accepted envelope的sequence/SHA/原始时效，247D输入命令切片与实际slew后指令一致；动作与保存数组及关节PD目标关系一致，全程CPU推理，无capture/关节冻结或机身伺服。三个run的独立样条coefficients/knots重建、105份源快照哈希、独占执行器和200Hz安全均通过，导航和策略运行源与预冻结V4一致。

三轮body接触0、未知接触0，最大原生滚转/俯仰0.11396rad、最小脚下支撑clearance0.27568米。每个停车窗601条200Hz原生样本，requested和actor速度命令均为零，策略仍每帧推理；首轮线速度RMS约0.00040/0.00111m/s、偏航速度RMS0.01290rad/s。两个重复轮相同判据停车通过。首轮CPU推理p50/p95/max约0.285/0.322/0.658ms；不能将该单轮最大值迁移成所有运行的实时保证。

异步history只记实际已写入envelope，sequence跳号表示pending被替代，不伪造50Hz送达。独立hash明确剔除5个写后transport字段，base envelope内嵌状态仍完整核对；首轮最大排队约19.85ms、单次写入返回耗时516.44ms，该阻塞期间原始source age继续增长并使消费者过期停车。新实现减少了ROS回调阻塞，但三轮仍约30%–37%策略帧拿到过期/不健康输入，不能宣称连续无丢失控制或实时链完全解决。

首轮41个移动附近的过期边沿中，2个之后有足够连续零命令与实际停车样本，39个在凑满停车窗之前恢复运动或缺少完整窗。这2个合格量测窗邻近返回到达的停车阶段，不是专门控制的命令生产者断开试验；因此组件总体 **TTL物理停车仍unverified**，没有用到达停车代替完整超时故障注入验收。有限路线通过也不宣称动态障碍停车恢复。

独立收据：[V4首轮](../runs/20261004_082258_navigation_slam_scan_roundtrip_v4_r1_9c33/summary_navigation_independent.json)、[同设置重复1c57](../runs/20261004_082715_navigation_slam_scan_roundtrip_v4_confirm_r1_1c57/summary_navigation_independent.json)、[同设置重复bd19](../runs/20261004_083026_navigation_slam_scan_roundtrip_v4_confirm_r2_bd19/summary_navigation_independent.json)；[三轮汇总与输入哈希](../test_results/navigation_v4_independent_campaign/navigation_v4_independent_campaign.json)。原generic `summary.json`套用了站立窗口而判motion失败，三份原bytes及SHA均保留；它不是此导航运行的有效路线判据。独立通过依据真实指令、SCAN、原始SLAM到达及原生停车，未修改全局acceptance。

[完整180秒命令与真实速度图](../test_results/navigation_v4_independent_campaign/navigation_three_runs_commands_physical_180s.png) 显示每轮actual actor vx/wz、requested和真实200Hz机身COM速度/角速度；保留全部样本，额外100ms后向平均线只为看清变化，不进入验收。绿线为原始SLAM到达时刻，绿带为3秒停车量测窗。[真实SLAM XY与注册目标区域图](../test_results/navigation_v4_independent_campaign/navigation_three_runs_actual_slam_xy.png) 使用原始位姿而不插值；灰色Gazebo轨迹仅经过后验一次固定刚体变换，明确不进入导航。对应绘图数组及每个输入/输出SHA随汇总保存，避免浏览器只展示末30秒站立而遗漏早先真实行走。

## 专项10秒命令生产者断开、实际停车与恢复

[24d9实际run](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9) 完成180模拟秒/9001策略帧，六类own进程全部正常退出、worker无fault。其[常规有限导航收据](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/summary_navigation_independent.json)仍用冻结逻辑，路线passed、全部transient TTL总体unverified；原generic摘要SHA `047913e98746bed3fcacb31a66939d7d311ab9a9b6d9f326e589588e8ece32cc`保持。独立新增[专项停车/恢复收据](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/summary_ttl_dropout_independent.json)主结论 **passed**，与原始SLAM两区域到达分别列层，不覆盖普通TTL项或全局acceptance。

夹具先观察到原始SLAM stamp7.099999999..7.4秒连续0.300000001秒body vx≥0.03m/s，以及真实healthy前进envelope，才暂停唯一own bridge。原PID1744350/starttime31801905经pidfd和内核T状态确认；真实gate `/clock`7.43..17.43秒保持10秒，随后同identity恢复。冻结文件seq473、原source sim7.475秒、原monotonic wall318029.039871015、vx0.135m/s；文件5135bytes/SHA `9ef116eb…`与mtime保持。gate的7.43秒比source stamp早45ms，不用于计算TTL；夹具只暂停命令生产者，未改机器人控制、joint target、导航输出或原文件时戳。[独立夹具来源收据](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/ttl_fixture_evidence/independent_fixture_audit.json)不自行宣称物理停车通过。

| 专项真实响应 | 测量与原判据 |
|---|---|
| 原source时标TTL | 7.475+0.300=7.775秒；最后accepted world7.77秒、sim/wall age约0.295秒 |
| 第一实际stale消费者帧 | world7.79秒、sim age0.315秒/wall age0.314713秒，requested立即变零；暂停段483条stale请求均零 |
| actual actor制动 | 既定加速度[0.6,0.6,0.8]逐帧slew误差0，world8.01秒actor速度命令全零，响应过渡0.22秒 |
| 预定实际停车窗 | firststale world7.79+原settling3秒→10.79..13.79秒；601条200Hz原生状态、151条50Hz持续CPU Teacher推理，全部在CONT17.43之前 |
| 3秒最大XY漂移 | 0.005012米 ≤ 原0.15米 |
| 3秒最大偏航漂移 | 0.036420rad ≤ 原0.2rad |
| 机身COM线速度RMS | 0.000988/0.002300m/s ≤ 原0.08m/s |
| 偏航速度RMS | 0.013978rad/s ≤ 原0.1rad/s |
| 停车姿态/接触 | 原生最大roll/pitch0.043127rad；body contact0、未知contact0；全程原生安全/clearance与独占执行器通过 |
| 健康命令和实际运动恢复 | world17.73秒出现新的accepted非零请求；之后16.9秒accepted运动命令、13.52秒真实速度>0.05m/s、恢复段最大实际XY推进0.78945米 |
| 原始SLAM区域路线 | outward27.4..28.0、return103.8..104.4秒，各7条原位姿连续≥0.6秒；与专项停车/恢复分层passed |

停车期间requested与actor速度输入全零，action仍由CPU Teacher每帧推理，未用action=0、capture姿态、关节冻结或机身伺服停车。原生物理时标使用PreUpdate `t−0.005秒`。[真实掉线—停车—恢复曲线](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/navigation_ttl_injection_physical.png)及对应原样选取数组NPZ保存；source age在停写期间连续增长，恢复后新文件也需通过原0.30秒gate，而不刷新旧seq473。夹具保持期间真实SLAM仍有0.3秒最大缺口，不宣称传感器全程无丢帧。

本项仅证明**这一次移动时10秒producer断开**的真实停车/恢复；不将数百个短暂过期事件或所有故障模式升级为通过。其后的有限区域路线完成也不替代动态障碍出现/清除试验。

## 未验证项与误差口径

前四轮没有真实行走或到达停车；V4三轮、专项24d9与V4.1普通dba1均已完成有限区域到达与实际Teacher停车。一次10秒命令生产者断开的专项物理停车与恢复通过，常规全部transient TTL项仍unverified。动态障碍6614实际停车/恢复与两区域功能通过，但其服务桥包装进程退出失败；V4.2的10c7完整收尾通过，返回区域90秒期限失败且独立clear连续1秒证据不足。V4.3的2f70又保留固定5秒停车峰值0.033278m/s超0.03、actualguard0.55秒间隙后新SCAN过早、出程最长raw SLAM dwell0.5秒不足0.6秒的真实失败；虽然正常退出，仍没有完成任何区域。V4.4的1536随后实际停车/clear同callbackrelease/新SCAN/恢复、两区域35.6..36.2与80.4..81.0及最终84.105..87.105停车均passed，所有进程正常退出，但进入前40毫秒的实际点云原XYZ没被阶段记录器保存，2个guard覆盖gate failed，冻结总项仍failed；全部功能完成与证据缺口明确分层。随后V4.5a dc6f保持V44控制/物理/profile，新增被动原点云pre-roll，限定动态严格总项19/19 passed：fixture原XYZ覆盖缺失0、固定5秒停车、32.775秒原始点云连续clear并同actual callback release、新SCAN、真实恢复、raw两区域50.6..51.2/90.9..91.5及终点94.69..97.69停车全部实证通过，六owned0/15childclean；仍仅单次限定平地场景，详情见[DYNAMIC_OBSTACLE_REPORT.md](DYNAMIC_OBSTACLE_REPORT.md)。完整三层路线、坡道导航、原46区域任务和真机仍未验证。

## V4.5a限定动态路线严格通过

[dc6f独立冻结专项收据](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/summary_dynamic_obstacle_independent.json) main19/19 checks passed，240模拟秒/12001 CPU策略帧，无fault，六owned0/15实际launch child clean。原始SLAM outward stamp50.6..51.2与return90.9..91.5各7真实样本、连续0.6秒、control radius0.17米/height±0.1米、arrival最大gap0.2秒与每目标90秒期限全部保持；按原目标确认到达，没有以tracking预测、壁钟或Gazebo真值替代。Actor非零命令33.86秒、实际原生运动16.55秒、最大离起点0.846669米。

真实existing box连续阻挡20.16..30.16秒，Teacher原固定停车窗21.16..26.16最大XY漂移0.003758米、planar峰值0.002245m/s、yaw漂移0.014919rad，原标准通过。原XYZ/精确integer pose/filter/双corridor实证clear31.725..32.775，seq149同真实guard callback在32.775释放，新SCAN5/6/7之后32.89 accepted实际恢复、物理推进0.666881米。519个actual guard日志完整；110次可由原XYZ独立重放，fixture规定窗口原payload缺失0；录制期外409行缺XYZ仍明确保留几何unverified覆盖限制。被动pre-roll只保存过去真实7份数组/77header回执，控制、TTL、区域、clear、停车和物理未改变；1536不回填，不追认其failed总项。

最终94.69..97.69三秒持续Teacher0速度输入停车，XY漂移0.000846米、yaw漂移0.013697rad、lineRMS0.0000445/0.0005413m/s、wzRMS0.0045793rad/s。[全程实际命令/速度和动态停止恢复图](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/dynamic_obstacle_actual_stop_restore.png)及[原始SLAM路线/注册目标/dwell图](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/dynamic_obstacle_actual_slam_xy.png)已QA；其后的142秒零命令站立另记最大XY偏离0.002434米/yaw0.043790rad，只作诊断而不新增gate。1536曾有晚段Teacher0输入脉冲，不能以此单轮较平稳消除该限制。

这一结果只升级限定`finite_flat_dynamic_stop_resume_dwell_v1`实际试验，不修改global acceptance：整体Sim2Sim failed、全局navigation/multifloor/realrobot unverified。三层通过坡道连接，完整坡道运动此前真实失败，尚无坡道导航或真实楼梯通过的证据。

## V4.1普通路线与实际命令传输对照

[dba1实际run](../runs/20261004_091122_navigation_slam_scan_atomic_transport_v41_r1_dba1/summary_navigation_independent.json)完成180模拟秒/9001策略帧，六owned进程全部exit0，worker无fault；同冻结有限路线判据 **passed**。原始SLAM outward22.899999999..23.5、return91.799999999..92.4秒，各7原样本/0.600000001秒停留；真实机身最大XY离站立点0.832229米，CPU actor实际非零运动命令34.28秒。到达后95.59..98.59秒原生停车窗漂移0.000507米、偏航0.008212rad，持续Teacher推理。原generic摘要SHA `0165b4c4…`原字节保留，不使用其套用站立全窗的失败诊断代替路线验收。

[实际传输对照收据及源diff](../test_results/navigation_transport_v41_comparison/transport_comparison.json)比较三份V4与该一份V4.1。导航profile原字节相同，生成世界仅run目录/名字规范化后字节相同，Teacher policy与Gazebo执行器/prepare来源SHA相同。命令writer去掉`fsync`，保留write/flush/close/atomic replace；普通controller仅把scope字面值改为profile读取，实际值相同。其余源变化增加了此普通路线未使用的独立动态分支。不同次运行有调度和负载变化，不将此对照称唯一因果证明。

| 实际写入/消费记录 | V4三轮范围 | V4.1 dba1 |
|---|---:|---:|
| 每次写入返回耗时p95 | 195.01–286.74ms | 0.744ms |
| 每次写入返回耗时p99 | 300.49–396.60ms | 0.914ms |
| 每次写入返回耗时最大 | 485.27–577.39ms | 8.301ms |
| 排队耗时最大 | 19.85–104.27ms | 286.72ms |
| worker stale-or-unhealthy原flag | 2697/2801/3347（29.96–37.18%） | 1804（20.04%） |
| 真实原envelope源龄超出[-0.05,0.30]秒 | 1222/1468/1638帧 | 2帧 |
| 源龄有效但healthy=False | 1475/1333/1709帧 | 1802帧 |

`command_expired`的原worker语义是**stale或unhealthy**，不是纯TTL过龄；真实源龄与健康标志已分别统计。四次普通运行的实际导航status `obstacle_hold`快照均0，accepted的controlled stop也可能来自转身、停留、到达或其他保护，不能归为障碍停车。第一版新comparison误查了不存在的`read_status='expired'`而得到0，错误脚本/收据保留在`incorrect_read_status_expired_v1`并显式附erratum；旧run日志、independent/timing原收据均未改变。[另一agent独立计数](../test_results/transport_counts_independent_20261004.json)与四次原flag完全一致，源龄的单项统计与联合范围统计分别注明口径。

测得写入尾延迟改善，V4.1仍有真实健康门hold及最大286.72ms队列等待，不能称完全连续或无时效问题。原0.30秒时效门和原时间戳没有放宽或刷新。

记录后的真值误差只在SLAManchor时冻结一次刚体变换，按原生PreUpdate状态的实际物理时刻 `t−0.005秒` 作因果配对；1027条配对的站立位置误差中位约0.000981米、p95约0.002749米、最大约0.005489米。该结果仅为**站立误差**，不能称移动过程中的SLAM精度，也不进入目标、SCAN、速度命令或区域判定。对应数组单独保存为 `navigation_independent_truth_error.npz`。

实际command envelope每次落盘的sequence/时钟/wall/sim age/健康门/acceptance，传感器门的publisher graph和预热历史均保留。原始注册点云本轮保留的是处理stamp/点数/hash，未逐条存全XYZ；不能据此宣称独立重放了所有碰撞检查。后续已接受的SCAN若有原始coefficients/knots/samples与metadata，分析器会核对数组哈希、真实payload一致性并独立重建样条；没有payload时保留unverified。

## 只读复核

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_navigation.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_012446_navigation_slam_scan_roundtrip_first_r1_02f7
```

只追加独立分析，不启动任何ROS或仿真。后续接口修复与实际复测另列，不覆盖首轮站立失败证据。
