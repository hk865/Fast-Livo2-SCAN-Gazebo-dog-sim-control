# 有限SLAM/SCAN导航独立审计（2026-10-04）

三轮有限一米往返**均失败，实际仅站立**。Teacher的CPU推理、独占执行、真实车载RGB/IMU/LiDAR→FAST-LIVO2位姿和点云链已运行；第二轮接受了87份真实SCAN轨迹，但三轮Teacher实际导航命令都为零，没有区域到达证据。通用摘要的接口/安全或motion标记不能作为路线通过。本文件只讨论有限平地实验，不改写整体Sim2Sim、完整坡道、多楼层、动态障碍或真机结论。

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

## 未验证项与误差口径

“运动中命令失效→停车”没有在三轮发生，因此真实导航TTL物理停车仍unverified。过期/缺失指令被置为零证明等待门工作，不能替代行走后的停车试验。没有到达目标，也没有到达后停车窗口，故该项同样unverified。动态障碍没有出现/清除事件；完整三层路线、坡道导航、原46区域任务和真机仍未验证。

记录后的真值误差只在SLAManchor时冻结一次刚体变换，按原生PreUpdate状态的实际物理时刻 `t−0.005秒` 作因果配对；1027条配对的站立位置误差中位约0.000981米、p95约0.002749米、最大约0.005489米。该结果仅为**站立误差**，不能称移动过程中的SLAM精度，也不进入目标、SCAN、速度命令或区域判定。对应数组单独保存为 `navigation_independent_truth_error.npz`。

实际command envelope每次落盘的sequence/时钟/wall/sim age/健康门/acceptance，传感器门的publisher graph和预热历史均保留。原始注册点云本轮保留的是处理stamp/点数/hash，未逐条存全XYZ；不能据此宣称独立重放了所有碰撞检查。后续已接受的SCAN若有原始coefficients/knots/samples与metadata，分析器会核对数组哈希、真实payload一致性并独立重建样条；没有payload时保留unverified。

## 只读复核

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_navigation.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_012446_navigation_slam_scan_roundtrip_first_r1_02f7
```

只追加独立分析，不启动任何ROS或仿真。后续接口修复与实际复测另列，不覆盖首轮站立失败证据。
