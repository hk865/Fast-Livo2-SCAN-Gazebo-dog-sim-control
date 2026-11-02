# Building资产与坐标

本目录包含原网格Building.dae、200万点XYZ/法向PCD、PCT高程切片安全NPZ、实际网格三角面和原生A*路线；没有机器人采集轨迹。PCD相对于Gazebo物理网格固定平移约`[-.000002,-.000001,-.1]m`，楼层支撑约1.47、2.98、4.43m，不能以切片编号乘层高。

上游和精确原路径见SOURCE.json。Building网格位于上游unitree_ros目录，BSD-3-Clause副本见UPSTREAM_UNITREE_BSD；PCD与tomogram位于PCT_planner目录，GPL-2.0及NOTICE副本见UPSTREAM_PCT_GPL_2.0、UPSTREAM_PCT_NOTICE。UPSTREAM_LICENSE是上游根集成代码Apache声明，不代替嵌套组件条款。

本地派生修改：Building.dae提取世界三角面为building_triangles.npz；tomogram转为无pickle依赖的building_tomogram_safe.npz；以原生PCT A*生成路线，并按物理支持面修正Z。A*兼容合法零代价与高度连续候选层，未调用GTSAM高度平滑器。routes/multi_floor2.json长85.69597066074994m。

world.sdf.in引用项目内Go2/L1模型，Student自训部署权重已提供；L1许可说明仍未补齐，当前公开分支不携带L1网格；不改变碰撞几何替代缺项。已知出生先验下的完整真实SLAM通过及边界见根README和验收摘要。
