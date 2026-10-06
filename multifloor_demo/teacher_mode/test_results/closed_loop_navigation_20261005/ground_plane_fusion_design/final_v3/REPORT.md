# d42e地面平面垂直漂移观测：纯离线设计

19个预选原始点云样本（6.0–7.0秒11帧、30/50/80/100/120/130/160/210秒）均通过原NPY文件、原payload、decoded float64 SHA和实际源header/pose核对，均得到唯一局部支撑面。本报告没有修改SLAM、导航、Actor、运行原件或既有验收，未启动ROS/Gazebo/模型。结论是**垂直漂移可被点云+已知地图高程观测到**，不是融合部署通过。

初始11帧地面在冻结anchor XY处的中位数为−0.313883420m，MAD0.000084180m；原anchor机身z=0.006227940m，因此h0=0.320111360m。按原yaw-only地图约定，floor2地面期待z=anchor.z−h0+1.2=0.886116580m。160/210秒原点云地面却在0.688543528/0.687375189m，得到独立水平地图观测δz=+0.197573052/+0.198741391m。160秒6176个平面inliers、RMS5.497mm，210秒6161个、RMS5.483mm，均有8方向支持；机身相对地面约0.30205m。

同floor2平台130→160秒，原地面下降0.212908920m、原SLAM机身下降0.210384070m；离线native机身反而只升0.002514756m。这种同层差分直接说明约21厘米垂直地图漂移，不能解释为换到较低楼层或真实下坡。

原地图轴只冻结yaw。额外保留一份**独立坐标敏感性**：利用原6.7秒实际SLAM/body quaternion与因果IMU CUSTOM/world quaternion（body mount identity）计算完整map-up，再只用原初始地面height+已知1.2m层高；未读取native作为估计输入。160/210秒观测δz=+0.204885319/+0.206048470m，native单次初始SE3对齐的离线垂直误差为+0.203919474/+0.205040958m，相差约0.97/1.01mm。它支持测量可观测性，不能追认原yaw-only地图为完整SE3注册。单纯把初始地面拟合normal外推15米则给出约0.231/0.232m，误差约27mm；说明微小法向估计偏差会累积，不能盲用地面normal校正全地图。

| 原header秒 | raw SLAM body z | measured ground z | body/ground clearance | 原水平map δz | 原SLAM+IMU up敏感性δz | native仅离线误差 |
|---:|---:|---:|---:|---:|---:|---:|
| 30 | 0.220339 | -0.073517 | 0.293856 | +0.005004 | +0.000862 | +0.000185 |
| 50 | 0.495101 | 0.200010 | 0.295091 | +0.005018 | -0.002610 | -0.003309 |
| 80 | 0.951215 | 0.651892 | 0.299322 | -0.008420 | -0.021541 | -0.022553 |
| 100 | 1.187913 | 0.899015 | 0.288898 | -0.012898 | -0.029416 | -0.031960 |
| 120 | 1.217985 | 0.910142 | 0.307844 | -0.024025 | -0.039659 | -0.040780 |
| 130 | 1.200978 | 0.901452 | 0.299526 | -0.015336 | -0.021480 | -0.022463 |
| 160 | 0.990594 | 0.688544 | 0.302051 | +0.197573 | +0.204885 | +0.203919 |
| 210 | 0.989426 | 0.687375 | 0.302051 | +0.198741 | +0.206048 | +0.205041 |

目标机身z=1.206227940m与160秒raw SLAM body z的差为0.215633775m，但这不全部是垂直漂移。原水平map观测修正后body z约1.188167217m，仍低目标0.018060723m，正是初始h0=0.320111360m与当前0.302050637m的支撑高度变化。完整IMU-up约定还改变固定地图本身的坐标表达。不能把目标机身高度误差直接当作δz，不得用goalZ强行补齐。

## 支撑面选择与层判别范围

只用原点云与实际raw SLAM body origin取2.5m横向局部ROI；3点RANSAC+SVD优化，normal倾角≤15°、残差≤15mm、≥80点、8方向中≥6个方向、XY协方差小特征值≥0.04m²。最终支撑面还必须满足**body-origin到平面**clearance0.20–0.45m，且唯一。它不是最低机身附体clearance、足底位置或载荷测量，不能替代既有200Hz安全门。

30/50/80秒坡面倾角为5.715/5.747/5.777°，与已知坡度atan(.1)=5.711°一致，支撑clearance0.294–0.299m；不把坡面当水平层。初始帧同时存在向前坡面候选，但只覆盖2方向，未误作脚下地面。扩大ROI到8m的独立诊断在100秒得到低平面candidate的body clearance1.467m，及高平面负clearance，均被排除；但大ROI也会出现多个合格拟合片段，因此**唯一性拒绝**，不能按最大点数偷偷挑一个。8m的候选没有实体标签，不能称每个就是某一楼层。原局部数据没有在同时可见的floor3顶板与floor1之间全面验证自动楼层识别；合成多层负例只是数学拒绝验证。

floor2=1.2m来自既有地图静态高程；样本关联来自实际SLAM/路线阶段，本设计尚不是可运行的自动support-layer selector。未来必须冻结因果区域到达+静态地图层候选，处理重叠层/遮挡/入口seam，缺面或多面返回unverified并安全停车，不造平地fallback。

## 原始源与坐标

原cloud是camera_init下实际registered XYZ，来自SLAM的对应原header，已含lidar→IMU/body外参；本观察器不再套一次外参。原navigation_fastlivo.yaml的extrinsic_T=[0.2,0,0.1177]、R=I，原sensor_contract一致；原planner cloud_is_world=true/need_extrinsic=false。30/80秒的原self-filter pose分别落后cloud header100ms，其他预选帧同header；这是实际回调顺序记录，报告保留它，分别核对过滤pose自己的原integerstamp和cloud时刻原SLAM pose，未伪称全部同步。点云源缺失、hash错、非finite或源不因果都不可通过。

该run的gravity_est_en=false/ba_bg_est_en=false/imu_rate_odom=false仅是已冻结参数事实，不足以独立证明漂移的唯一原因。根节点后续独立测试开启在线估计与本纯离线数学互不修改。

## 可复用数学与上线前条件

`plane_observer.py`提供fit_candidate_planes/select_support_plane/vertical_observation，只接受numpy点云、原始SLAM body origin及静态高程；无原生状态/仿真API。`frame_sensitivity.py`给出单独map-up敏感性数学。12项合成几何检查和2项坐标检查全部通过，涵盖水平/坡面、上下层、模糊多面、无支撑、非finite和输入不变；这些不是物理融合测试。

如果未来融合：保留raw/corrected两套原stamp及plane/covariance/层选择证据，300ms freshness和原安全guard不变，delta只从点云+地图prior产生；导航pose与当前registered cloud须应用同一有界、连续、时间一致的坐标校正，不能只改pose而让碰撞图仍在旧z，也不能通过忽略高度到达门绕过验收。固定XY注册±0.15m在0.1坡度上本身约±0.015m高程不确定性；法向外推和seam也须进入不确定度。这里只证明d42e所测阶段的可观测性，未验证ramp23、自动换层或真实机器人。

两个分析编排错误留在final/和final_v2/：最初误要求self-filter pose与cloud同header，后一次JSON遇numpy bool；均未生成正式结论，不影响原件。final_v3分别验证各原source stamp并成功完成，原两目录保留analysis_failure.json。
