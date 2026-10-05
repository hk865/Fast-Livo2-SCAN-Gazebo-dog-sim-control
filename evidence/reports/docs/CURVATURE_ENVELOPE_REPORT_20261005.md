# Teacher 实测转弯包络、圆圈与连续 S 转向（2026-10-05）

已按“先测转弯半径，再测圆圈和不同曲率 S 转向”的顺序完成实际 Gazebo 测试。36次转弯包络测试全部通过；随后14次连续曲线的行驶跟踪全部通过，包含固定停车验收则13/14通过。紧S的25Hz轮次因停车yaw漂移0.114522rad超过事前0.1rad门限而失败，原件保留。

本轮采用独立60×60m无障碍平面，不是原SLAM5地图。Teacher仍是冻结model_1000、CPU单线程、50Hz策略及200Hz唯一关节PD；不重新训练、不改camera_mode、不操作真机、不停止另一任务训练/评估。控制反馈来自Gazebo原生状态，Actor输入有232维原生特权状态/高度及15维命令/上一动作。本轮属于真值控制器标定，不能当作SLAM/SCAN导航融合。

## 先测实际转弯包络

事前矩阵为Teacher输入vx=0.3/0.5/1.0m/s，各配wz=±0.2/±0.4/±0.6/±0.8rad/s，共24轮。0–3s站立，3–6s加速，6–26s持续运动，26–33s归零并停车；所有轮次保持同一执行器、速度变化率及原协议。

半径从原生200Hz实际机身原点XY轨迹拟合，不能用请求vx/wz替代实际半径。18项事前门限要求持续实际前行、速度/角速度跟随、至少60度真实弧转、圆拟合RMS≤0.05m、分段半径变异≤0.2、无机身接触/fault、力矩/关节速度限制、原记录覆盖以及固定5s零速度停车XY≤0.05m/yaw≤0.1rad。原地转向、停下再转或未实际走弧不能通过。

每个速度/方向选择这组测试内最小合格半径，再固定相同case各补两轮。因此每个边界有三次独立实际运行，共36轮通过。下表标的是Teacher输入速度，实际前行速度另列；不是以该准确物理速度求出的全局最小半径。

|Teacher输入vx m/s|左转最小已测合格半径 m|右转最小已测合格半径 m|左/右实际机身前行均速 m/s|边界重复|
|---:|---:|---:|---:|---|
|0.3|0.355448|0.366177|0.28220 / 0.28673|各3/3通过|
|0.5|0.602845|0.618007|0.47990 / 0.48651|各3/3通过|
|1.0|1.284869|1.296682|1.02674 / 1.02968|各3/3通过|

这六个边界均出现在输入|wz|=0.8rad/s；没有测试更高角速度或更小半径，不能称物理极限。固定初态和无噪声使三次数组相同，这不是随机扰动鲁棒性证明。规划暂以1.5倍最差已测实际半径留中心线余量：输入0.3m/s至少约0.55m，0.5m/s约0.93m，1.0m/s约1.95m；机器人足迹及障碍膨胀需另外计算。1m/s的圆圈/S跟踪尚未测试。

第一组24轮的外层批次工具最终返回143，原因未验证；其24个实际run原独立收据、native日志和自有进程退出均完整且通过。该外层异常另存`sweep_outer_launcher_receipt_v1.json`，不改成整体退出0。12轮边界复测及后续曲线批次退出均为0。

来源：[24轮独立包络分析](../test_results/curvature_campaign_20261005/analysis/final_/20261004T181443_819883Z/summary_radius_campaign.json)、[三次边界复测](../test_results/curvature_campaign_20261005/boundary_analysis/final_/20261004T182346_462506Z/boundary_receipts.json)。目录名`plant_radius_prepare`是历史命名；这些36轮有实际33s物理/Actor记录，不能因名字含prepare把它们算成仅准备。

## 连续曲率控制与位置误差

新`curvature_cascade_pi`保留已选COM速度PI、路径/朝向PD增益，添加几何曲率前馈wz=κv及参考角速度对应的朝向微分。路径为事前固定空间曲线，按机身原点在局部有向曲线上的几何投影计算带符号横向误差；单调弧长进度防止圆圈重叠处跳到终点。没有按时间生成一个理论位置点，没有停转门，圆圈必须真实走满一圈。

路径与朝向PD为0.8/0.2及1.3/0.15；vx PI=0.6/0.5、vy PI=0.4/0.3、角速度PI=0.3/0.2，测量滤波0.2/0.1s。外环只在新鲜反馈更新，测试10/25/50Hz；Teacher始终50Hz，native关节PD始终200Hz。双时钟TTL仍300ms，命令变化率仍[0.6,0.6,0.8]/s，反积分饱和包含最终执行端slew。

圆圈为完整2π、前后各2m切线段。S转向以弧长u定义theta(u)=A sin³(2πu/L)，位置是cos(theta)、sin(theta)对弧长的积分；轨迹是平滑侧移再返回的连续反向转弯，并非方波折线。gentle为L=8m/A=0.6rad，最小几何半径约1.838m；tight为L=6m/A=1rad，约0.827m。κ及路径切线由解析公式计算，固定路线不会因漂移移动。

全部14轮参考速度为0.3m/s。路径profile先校验同速度、同转向的实际plant收据与1.5倍半径余量，再冻结几何/源码哈希。圆圈连接处切线连续但曲率可跳变；S的曲率渐变。前馈预算≤0.64rad/s，曲线vx命令上限0.8m/s；更高速曲线不属于本轮结论。

## 实际曲线结果

每轮必须通过原13项平地共同验收及新增15项来源/真实连续前行/完整曲线进度等检查。路线max/RMS≤0.20/0.08m、drive朝向≤0.2rad、COM速度MAE≤max(0.05,0.25×参考速度)，首个合格固定5s停车XY/yaw≤0.05m/0.1rad；不降低门限，不选择更安静的后续停车窗口。

|路线|实际重复/反馈Hz|完整结果|实际drive路线RMS|
|---|---|---|---|
|R=0.6m左/右完整圆圈|25Hz，各3轮|6/6通过|约8.8/10.2mm|
|R=1.2m左/右完整圆圈|25Hz，各1轮|2/2通过|约6.3/7.3mm|
|gentle S反向转弯|10、25、50Hz各1轮|3/3通过|约7.3/6.4/6.4mm|
|tight S反向转弯|10、25、50Hz各1轮|2/3通过；25Hz停车失败|约7.8/8.3/7.6mm|

动态七项14/14（98/98）通过；新增15项208/210通过，两项失败都由tightS25原停车失败引起。整项仍是13/14，不包装成全部运动通过。全部实际进程正常退出，无fault或机身接触。几何投影公式与冻结控制器共享，实际200Hz运动量、因果参考、PI及slew重建由另一分析器只读核对；这是独立物理测量复核，不宣称完全独立推导几何公式。

仅弧区的heading RMS在10/25/50Hz为：gentle 0.007565/0.005434/0.004689rad，tight 0.012092/0.007447/0.005765rad。50Hz在这两轮弧区heading较小，但速度MAE不是单调改善、每种S/Hz仅一轮。不能用整个drive被相同起步段主导的峰值，或一次停车分数，推断最佳反馈频率。无需使PID频率成为Teacher的多倍才能在这些路线跟踪成功。

完整14个run、原始200Hz轨迹、弧区误差/速度、PI重建及逐份hash见[14轮独立曲线报告](../test_results/curvature_campaign_20261005/curve_analysis/REPORT.md)。

## 从曲线得到的下一步调优方向

紧圈比宽圈的入/出弧heading峰值明显大。R0.6原前馈在25Hz连接处跳变率12.5rad/s²，R1.2为6.25rad/s²；均高于执行端wz变化率0.8rad/s²。紧右圈入弧同时发生yaw限幅及slew阻止积分，紧左圈整体峰值出现在出圆后的直线段，日志保留因果头/κv/反馈/实际命令。下一项合理A/B是曲率渐变连接和曲率变化率限速，先保持增益，不能先把全部误差归因于PID更新慢。当前只有关联证据，没有完成这个A/B，不宣称其已改善。

tightS25停车窗37.905–42.905s原生1001帧、Teacher251帧，实际Actor速度命令持续为0；XY漂移4.315mm、yaw漂移0.114522rad，整项FAILED。停车阶段外环与内PI输出被绕过，Teacher持续零速度站立；所以这是停车姿态保持能力暴露，不是25Hz停车速度闭环跟踪失败。频率改变到达时刻/足步相位可能间接改变漂移，但尚未证实唯一原因。下一项停车改进应另立“主动速度/位置/朝向保持”契约、预登记同窗口和到达标准后A/B，不能偷偷改变旧零命令门或重选停车窗。该功能尚未实现/验证。

实际SLAM旧profile平地6m及往返已6/6通过，详见[真实SLAM迁移报告](SLAM_CONTROLLER_TRANSFER_REPORT_20261005.md)；本次连续曲线core尚未迁移SLAM、SCAN或多楼层。原坡道闭环真值35轮留出测试另见[真值标定报告](TRUTH_PID_CALIBRATION_REPORT_20261005.md)。这些证据不合并成新曲线控制器完整导航/跨仿真/真机通过。

## 原始相机、浏览器与复现

圆圈pilot及gentle S三个频率均采集实际两路RGB和CameraInfo；其余10轮没有运行capture，不能声称每轮都有影像。固定旁观相机是[2,-4,3,0,0.65,π/2]，gentle S的12/15/18/20s原始帧可见机器狗，末帧走出视野而为空平面；不生成替代帧。两路实际D为五个0，车载在无障碍平面上看到地面/天空符合场景。原camera_mode未修改。

[gentle S实际画面与原始曲线](http://127.0.0.1:8769/?run=20261005_023202_truth_pid_S_gentle_curvature_cascade_pi_25hz_5c8b)、[保留的tight S25失败](http://127.0.0.1:8769/?run=20261005_023511_truth_pid_S_tight_curvature_cascade_pi_25hz_0b78)。页面只读归档，曲线badge以原独立收据及哈希链判定，不以模型加载成功当通过。

在teacher_mode目录执行；profile已冻结，新运行总会创建新run，旧记录不覆盖：

```bash
# 包络实际复测：新33s测试，不是prepare-only
python3 -B navigation/curvature_tuning/run.py --plan test_results/curvature_campaign_20261005/boundary_plans_v3/v0.3_wp0.8_r2.json

# 已冻结的完整圆圈或S路线，默认CPU Actor；加--camera保存两路图像
python3 -B navigation/curvature_tracking/run.py --profile test_results/curvature_campaign_20261005/curve_profiles_v1/04_S_gentle_25hz_r1.json --camera

# 对新run写追加独立收据；已有收据会拒绝覆盖
python3 -B runs/本次新目录/sources/truth/curve_receipt.py --run runs/本次新目录

# 只读结果页，不启动控制
python3 -B navigation/truth_tuning_design/dashboard.py --port 8769
```

## 冻结与验收层级

|来源|SHA256|
|---|---|
|冻结Teacher|`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`|
|包络原协议/独立验收器|`2ad596f0c3815c8a12ba7f919b24f8087115ea9a510aa4fa7a8301300f459847` / `b2663570a8ae9dcc53b26638a81e9b86389087e9cd488cbe22aa4e48a70ffc07`|
|三次边界原件|`2d9346d5c2ed7cbe73a4d8f468aaef0523cc2d039cac3f20a70753076b9d18e1`|
|连续曲线core / profiles|`82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d` / `d3aab4ebf73fb9951ec21da1dacf33e1432fbbda7c02d04a9b04191b9e4160ed`|
|新增15项独立收据分析器|`14adc95fc60bd65f4b9a8c0dd456c4af680c8315a191016a7531e328582aeb28`|
|14轮只读重建分析器|`9853ebe4336e0b5ee9c5b35712052820c8c64ba29064aa793aee265a183ff9de`|
|14轮aggregate / manifest|`1f53db51bfc3a808aa40edb8778433a385836f22ed6320643aa2de48df06e4cb` / `d08907c0d170b9a6c6aba28ef471492a1bceed8eff30b5b9d7e3e572e645bd43`|

全输入冻结见`test_results/curvature_campaign_20261005/curve_freeze_v1.json`，逐轮source/runtime与失败归档不改写。接口通过；Gazebo已测包络和连续行驶通过，曲线含停车整项有1轮失败；新曲线律Isaac对照、真实SLAM曲线/SCAN/多层导航、真机未验证。历史全局Sim2Sim失败收据保留，不因真值闭环改善而提升。
