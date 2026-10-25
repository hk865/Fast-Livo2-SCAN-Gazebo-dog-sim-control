# formal22目录与模块职责

整理目录的实际运行入口现为根 `candidate/`，对应冻结部署 `ecb62ad`。`releases/formal22/runtime/candidate/` 保留源公开快照及原formal22证据。native/SCAN从本包源码重编，模型与固定SDK/执行插件按明确来源使用；具体边界见 [新目录复现](PUBLISHED_DIRECTORY_REPRODUCTION.md)。以下职责不因路径适配改变。


本说明只对实际部署536c9d3负责。下表中的路径相对于`releases/formal22/`。兼容模块、历史源码绑定与实际启用分开看；32项关键源绑定、91项实际加载检查及worker回执确定本轮来源。

|层|主要文件/入口|输入→输出及职责|
|---|---|---|
|启动/所有权|`deployment/round27_v3.py`、`detached_entry_v3.py`；runtime/candidate/run.py|提案→独立配置审→四项行政配置→prepare→execute→实际加载→原closure；唯一仿真owner、私有ROS域231、三锁、2400s运行+300s收尾|
|物理/唯一执行|runtime/candidate/bridge.py、ipc_request_window.py、Student/native适配器|外层速度意图→实际Actor命令观测/推理→native关节执行；50Hz Actor/200Hz物理、原执行限幅/故障规则；不是几何轨迹直接搬动机器人|
|传感与relay|runtime/candidate/sensor_relay.py；依赖内Gazebo传感适配|原`/livox/imu`、`/demo/camera`等→SLAM输入；仅IMU/RGB writer设ASYNC，状态writer同步，payload/stamp不伪造|
|FAST-LIVO2前端|native/src/fast_livo2_core及fast_livo2_ros|真实LiDAR/IMU/RGB→LIO/VIO融合状态、当前注册扫描/里程计、增量地图；唯一状态owner先服务IMU；不是新增回环或全局位姿图|
|里程计/场景辅助|依赖中的odom_adapter.py；runtime/candidate/known_scene_runtime.py、known_scene_matcher.py|原生里程计/注册扫描→已知43静态盒体的局部几何匹配与修正；支持、残差、退化、歧义、两帧确认；坐标generation变更使旧轨迹失效并重新准备|
|完整反馈组包|runtime/candidate/known_scene_inputs.py、r86_prepared_feedback.py|匹配同source的odom/cloud/metadata→单控制owner提交；保留精确身份、stamp、300ms等契约；prepared不是control-ready或实际接纳|
|SCAN与交接|scan/src/plan_manage；runtime/candidate/scan_epoch_supervisor.py、trajectory_association.py|位姿/当前障碍云/全局路线→局部B样条、attempt、metadata；route definition创建/更新时发送一次，局部消息用route id/version引用；不是每条局部候选重传全局大曲线|
|短接收/关联|runtime/candidate/r88_async_admission.py、teacher_wrapper.py|ROS take/短入口→原单一有界FIFO→单owner关联/事务；不等于DDS自己的网络队列，也不由message_filters代替版本/负事件/事务语义|
|几何worker|runtime/candidate/r88_physical_worker.py、r85_geometry_admission.py|局部候选+几何先验→连续静态空间证书；独立进程；父端仍有准入/提交负担。证书不等于动态可跟踪/导航PASS|
|跟踪/停车|runtime/candidate/controller.py、cascade_core.py、ordinary_parking.py、imu_causal_buffer.py|已准备且接纳的姿态/局部路径+原始gyro因果配对→速度/朝向闭环；巡航0.5m/s、纵向Actor上限0.6、其它原限制；停车用精确零输入ACK及首次T0固定5s|
|动态业务|runtime/candidate/r85_dynamic_obstacle_runtime.py|实际SLAM触发→独立世界障碍运动→当前云guard stop token→恢复/清除→最终独立物理核验；prepared epoch不当作已发布reference，真实reference变更规则仍保留|
|记录/验收|runtime/candidate/runtime_io.py、compact_archives.py、任务/停车验证器|当前输入/输出、回执、归档和本轮RGB地图；流式/有界记录不等于无同步编码成本，不能凭“后台写盘”断言callback轻|

## 地图先验与真实观测的边界

`world.sdf`的43静态盒体用于已知场景辅助匹配和静态空间/坡面支撑判断，不直接把Gazebo机器人真值位姿送给导航。匹配以预测XY/朝向为局部种子，不是全局重定位。动态前景点不用于该地图修正的面误差总体，但当前完整障碍扫描继续用于避障，不能删除动态观测来“改善匹配”。拒绝一次地图修正不等于删掉连续里程计。

增量地图没有因此获得回环后端，历史点云不因known_scene修正而整体重投影。RGB归档从本轮`/cloud_registered`生成；回到原点或几何看似闭合不是位姿图闭环优化证明。

## 当前选项及未启用项

已选：native571、Student45、ASYNC relay、compact route protocol、短SCAN intake、明确非前景质量模型、prepared状态分类修复、同有限地形支撑判定。R99未来帧等待OFF、R116压力旁路OFF；不恢复已停止的R138压缩/还原路线。真实协方差导出仅另一个离线构建候选，本包未混入；原生cov未正确输出时adapter固定回退不是校准置信度，不能据此控制恢复。

## 整理目录的来源链

`candidate/package_paths.py` 明确本包位置；`candidate/slam_workspace.py` 读取新native构建来源，`scan_workspace.py`读取新SCAN来源。来源归档将当前执行输入与旧R33/V32历史输入分开，保留实际actor/guard/native/SCAN来源，不把历史PASS当授权。运行输出和持久资源账本沿用原机受管池。

`reproduction/typed_route/`保存坐标不变的337点路线及来源迁移记录。`dependencies/derived_inputs/typed_route/world_prepared_package.sdf` 是固定新目录world模板；它相对旧world只改资源URI/插件路径。几何metadata、PCT来源身份和证书对齐这份固定模板，原严格world SHA、43boxes、transform、limits和连续曲线判定保留。

`student_lower_loop_ack.py` 等当前合同pin绑定实际迁移后合同，原模型/45维输入、PID数值、限幅、时间/序列/同帧校验不变。旧源文件与失败结果保留于来源和历史记录，不能将身份修正解释成新的控制律。
