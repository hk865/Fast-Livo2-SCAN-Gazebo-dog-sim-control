# 单一转向参考坡道第三轮：只读诊断

实际原件：20261005_144730_closed_loop_cascade_single_heading_ramp12_r3_2d63。新父层与CORE均使用实际SCAN最近3D投影切线；原20Hz/10Hz真实源、Teacher执行链没有被本报告修改。

首次围栏失败在39.430s，原SLAM位置[5.8308089668587355, -0.6752850139683859, 0.4833963310891637]；原生前一时相机身位置[7.061035171778299, 1.5698507100425285, 0.8073868860621246]。固定地图先验route error=0.450151m，超过原0.45m门；CORE对其当前SCAN路径cross error=-0.024657m，yaw error=0.009967rad。接受的SCAN路径相对固定注册中心线最大偏离0.606298m。这证明跟踪的局部路径已经偏离地图中心线，而不是仅控制器离开自己的参考。

首失败前原guard union={'blocked': False, 'clearance_m': None, 'point_count': 0}。所选原点云header=39299999999、14634点；NPY文件/rawpayload/decoded float64/原guard SHA逐项相同。图中点云只是实际registered XYZ，**不是内部占据图**。当前run未归档planner occupancy/inflation或优化代价状态，不能据这些点云直接断言具体障碍体素或代价权重造成绕行。body_height在该链用于目标Z的加/减转换，不能未经证据称它影响坡面分类。

首失败后停车保护由真实SLAM围栏触发。完整坡道和出口停车仍未验证。原运行收尾role返回值={'worker': 0, 'bridge': -6, 'capture': 0, 'navigation_stack': 0, 'gazebo': 0}；bridge=-6是另一项真实收尾失败，保留原runtime error，不伪称完整退出通过。

后续可采用更密的已注册坡中心线区域目标来约束局部规划跨度，同时保留原真实点云union guard、0.45m围栏和到达门；仍需另版实测。若需要解释规划器绕行的确切原因，应新增只读内部occupied/inflated voxel与对应优化路径/cost记录，不能用模拟真值补规划器观测。
