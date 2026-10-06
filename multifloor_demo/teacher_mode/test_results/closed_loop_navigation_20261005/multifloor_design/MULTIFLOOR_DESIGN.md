# 原场景两坡闭环连续路线与Actor支持层设计

本文件是离线静态地图/源码设计审计，没有运行仿真、ROS或策略，没有修改冻结closed_loop_cascade/v1。原世界、Teacher网络、PD/DCMotor、187点网格、+20m起点、base_z−hit_z−.5和[-1,1]裁剪均不应因本设计悄悄改变。新实现由root在独立版本目录冻结后运行。

## 路线与坐标注册

原ramp12沿世界+x，x2..14、y2、宽2m、支持面z0→1.2；ramp23沿−x，x14..2、y7、宽2m、z1.2→2.4。两坡水平长12m，坡角5.7106°。floor2为x14..18/y0..9，连接两条坡道。不是楼梯。

推荐保守连通路线：

|节点|场景prior支持面XYZ|作用|
|---|---|---|
|lower_entry|1.2,2,0|入口平台；spawn基座可为1.2,2,.4,yaw0|
|lower_seam_start|2,2,0|精确支持高度折点|
|lower_seam_end|14,2,1.2|完整12m坡道终点|
|lower_landing|14.8,2,1.2|全脚上floor2后的落平台|
|connector_start|16,2,1.2|平台内部转向|
|connector_mid|16,4.5,1.2|两扫描provider共同平台切换节点|
|connector_end|16,7,1.2|平台内部转向|
|upper_entry|14.8,7,1.2|进入上坡前平台|
|upper_seam_start|14,7,1.2|精确支持高度折点|
|upper_seam_end|2,7,2.4|完整12m坡道终点|
|upper_landing|1.2,7,2.4|floor3终点平台|

水平总长34.6m。connector改x15.5时可缩至33.6m，但到floor2左边缘的网格余量更小；最短不是全局避障最优证明。推荐首次实际用x16。支持高度seam knots应保留在路径/guard几何里，不一定全部作为停车目标。若把1.2m高度差线性摊到13.6m入口至停车段，支持高度最大差.070588m，在低姿态时可使地面过滤边界错误。

下行可反向使用同一已注册prior：floor3(1.2,7,2.4)→ramp23全段→floor2(14.8,7,1.2)→connector(16,7),(16,4.5),(16,2)→ramp12高端(14.8,2,1.2)→完整ramp12→floor1(1.2,2,0)。初始spawn可[1.2,7,2.8,0]，上下行单独登记，不拿上行通过替代下行。

每轮route必须在本次camera_init中注册并冻结一次。已有heading_alignment.py允许10个实际SLAM姿态header与实际CUSTOM/world IMU方向配对确定scene轴向；配对时间≤20ms、跨度≥.8s、方向散布≤1°/重力残差≤3°。绝对平移仍来自已知初始放置的场景prior或真实点云地标注册，不能用live Gazebo pose。旧相机注册数值禁止复制到本轮。

公开导航z应是本次初始SLAM基座原点高度加“支持面高度相对初始支持面的差”，不是裸floor_z、旧相机.75m基座高度或再次附加SCAN的.4m。已有共享request_plan先减.4、SCAN REFERENCE_PATH再加.4的接口必须保持一致。

注册不确定度不可省略。13.6m距离下1°方向误差可产生.237m横向误差；加上初始XYprior误差后可能超过路线门。建议保存本次源姿态配对、稳定段SLAM分布、IMU外参、anchor声明和变换SHA；将实际registration误差作为独立离线诊断，不能依靠truth修正导航目标。旧单轴fence不能用于整条曲折路线；需注册活动polyline段的原横偏门及对应层高度，不把±.45m放大为全场围栏。

当前冻结v1 route.py/request.py仅生成初始暖机body yaw方向上的一条直线及返回，尚无scene-axis多段注册。新多层必须另建版本，不能热改正在运行的v1。

## 187点扫描的数学证据

来源为冻结observation.py、contract.json与原three_floors.sdf。height_provider_math.json保存各来源SHA及所有静态样本结果。该结果是几何计算，不是Actor/导航/物理通过。

网格为17×11、x=-.8..+.8、y=-.5..+.5、.1m间隔，仅绕yaw旋转；任意yaw时世界任一水平轴半幅不超过√(.8²+.5²)=.943398m。+20m是射线起点提升，raw仍base_z−hit_z−.5。当前all_static选从起点向下的第一表面，无法自行判断脚下层。

|假定静态基座|all_static|正确provider|错误provider|
|---|---|---|---|
|x1.2,y2,z.4|187点命中floor3，187 overhead，raw=-2.5→全-1|lower12：176 floor1＋11 ramp12，raw≈-.1005..-.1|upper23同样命中floor3全-1|
|x2.8,y2,z.38|11点floor3＋176 ramp12，11 overhead|lower12：187 ramp12，raw≈-.2805..-.1205|upper23：11 ceiling＋176 missing→全-1|
|x2.9,y2,z.39|187 ramp12，无overhead|lower12同样187 ramp12|upper23全部missing→-1|
|x12,y2,z1.3|187 ramp12|lower12同样187 ramp12|upper23全部missing→-1|
|x16,y4.5,z1.5|187 floor2|lower12与upper23均187 floor2，raw≈-.2|无逐元素差|
|x8,y7,z2.1|187 ramp23|upper23：raw≈-.2805..-.1205|lower12：187 floor1，raw=1.6→全+1|
|x1.2,y7,z2.7|176 floor3＋11 ramp23|upper23相同|lower12：187 floor1，raw=2.2→全+1|

精确x2.8样本和浮点arange网格的边界可相差一个ulp，因此不能用微小XY扩张“修复”这一真实楼板边。固定精确边界的原算法不应改。扫过低端时all_static的ceiling问题客观存在；ramp23中心线路径不存在同种overhead，但扫出侧边仍会出现下层/缺失输入差异。

## 连续provider转换

建议只使用已冻结lower12与upper23，转换位置为真实SLAM注册的connector_mid=(16,4.5,1.2)，不能按native Z宣称NAV到达。native Z单独不足以决定层：ramp12上半段z1.3时upper23仍无任何坡面命中，提前切会把187点全部变missing；同一floor2又同时属于两provider。

离线检查了x15.5/16/16.5 × y4/4.5/5 × 37种yaw，共333个姿态、每姿态187点。lower12与upper23全部只命中共同floor2，hit_z逐元素最大差0；任意yaw的连续包络x14.5566..17.4434/y3.0566..5.9434仍位于floor2，避开ramp12(y≤3)和ramp23(y≥6)。这是±.45m控制偏差加.05m先验余量的几何区域检查。它不证明真实机器人一定落在此区域。

root已选择新版本按实际SLAM region_arrivals中的connector_mid事件锁定切换。worker须验证本轮request/goal SHA、原始源pose stamp≤当前native clock、原事件不变，再在**当时原生基座的同一187射线**上比较两provider；任一missing、不相同floor2命中或hit_z最大差>1e-9，则拒绝转换并请求零速度，不能造平地fallback。成功事件只能向Actor自身日志写provider ID、源SLAM事件/hash、nativeclock和原raw bytes SHA，不能向NAV回写native位置/高度或改到达。

转换后upper23一次锁定；双坡下行新协议反向锁定upper23→lower12，同一共同平台节点。若未来多次往返，需要显式有序状态机和新的原SLAM区域事件身份，不允许旧connector_mid事件被复用。

动态provider仅改变特权Actor扫描目标集合。所有原物理碰撞、native独立安全ray/接触、SCAN实际点云碰撞保护保持全场。Actor排除上方deck不能用来删除NAV障碍；真实局部点云替换Actor187维尚未验证。当前all_static与两个局部manifest都标strict_training_geometry_equivalence=false，必须保留这一限制。

## 实际运行与验收边界

新closed-loop v1首次平地已由root启动，本文不读取live重日志、不赋予其结果。连续两坡应在独立新版本冻结、真实SLAM/SCAN源健康和命令300ms因果守卫下运行；若失败保留失败而非等开环再过才能跑。

每坡必须记录12m实际足支持coverage、入口/出口平台接触、SLAM原header区域到达、坡面clearance、原姿态/COM/原点速度、q/qd/JointForceCmd以及唯一执行器/全部退出。region到达与active endpoint hold不同；hold仍必须服从实际cloud/SCAN/stale保护。NAV不得订阅或读取native pose/Actor高度值。native只供Actor特权输入与独立离线物理验收。

本设计没有新的完整多层闭环、SCAN支持层自动建图、SLAM曲线或真机通过结论。
