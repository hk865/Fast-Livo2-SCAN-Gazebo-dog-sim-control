# V4.2：动态服务桥退出修复

V4.1实际动态轮 `20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614` 完成240sim s、12001个Teacher帧，actor没有故障，路线状态succeeded。其导航launch父进程退出−15，其余五个runner子进程退出0。该−15原始记录和运行结果不修改，不能表述为全进程正常退出。

`navigation_stack.log` 的15个launch子进程中14个收到SIGINT后报告clean exit，包括mover、真实位置C++观察器和被动传感器存证；存证清单已写，queue_error=null。唯一未报告退出的是 `ros2-12`，命令为 `ros2 run ros_gz_bridge parameter_bridge /world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose`。

本机安装的 `/opt/ros/jazzy/lib/python3.12/site-packages/ros2run/api/__init__.py` 中 `run_executable` 创建子进程后循环communicate，捕获KeyboardInterrupt但不转发SIGINT；源码假定终端把信号同时送到子进程。ROS launch发送到该CLI包装进程的SIGINT没有这个终端组广播效果。runner等待launch5秒后向自身组升级SIGTERM，父launch于是−15。这也与launch自身默认5秒升级窗口重合。

唯一runtime改动是 `navigation/stack.launch.py` 的服务桥从ROS CLI包装进程换为 `launch_ros.actions.Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[原服务参数])`。没有改runner清理期限、物理、Actor、控制器、300ms TTL、地形、障碍运动协议或验收条件。原V4.1stack和freeze原字节保存在本目录。

`offline_cleanup_probe.py` 仅用新建的自有Python mock进程执行安装ros2run函数的AST，未启动ROS/Gazebo服务或机器人控制：包装进程仅收SIGINT后子进程没有收到、仍等待，组升级后−15；直接mock目标收到SIGINT后0退出，约3.4ms。实际Node命令离线解析到已安装桥二进制、相同服务参数，没有ros2包装层。八个专项检查通过。这个结果解释进程转发边界，不替代真实桥清理验收。

新 `test_results/navigation_v42_cleanup_freeze.json` 继承未变部分的91项V4.1离线检查，并记录8项专项检查、唯一source差异与新的只准备动态scope。下一实际动态轮仍需要确认所有自有进程正常退出；历史6614失败保持。

包清单生成器只落盘与语法检查，尚未运行。正式写入要求 `current_status.finalized=true`；新实验结束、独立报告与当前范围状态完整后，主任务再运行它。它不改历史验收，不把仅准备目录计入实际运行。
