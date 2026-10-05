# 关节发布边界观测器（测试专用）

不发布命令、不启动物理、不修改原消息、原函数参数或返回值。`rcl_publisher_init` 成功后用实际解析话题 `/joint_states` 和生成的 C++ `sensor_msgs/JointState` 类型支持句柄身份确认内存布局；同话题 C ABI 不进行转换。句柄销毁和地址复用会撤销旧标签。

Gz **单个启动动作**的环境配置：

```text
LD_PRELOAD=/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/navigation/test_results/publish_boundary/librcl_publish_audit.so
DEMO_PUBLISH_AUDIT_OUTPUT=<本次运行绝对目录>/joint_publications
```

原调用前后记录整数 Header 纳秒、单调墙钟、线程 ID 和真实 `rcl_ret_t`。65536 条固定内存，逐消息没有分配或文件写入。正常退出产生 `joint_publications.<实际PID>.jsonl`，零调用进程不写文件，文件以 O_EXCL 创建；因此 `gz` 包装器不会占用服务器收据。需要核对实际 owned Gz PID、verified publisher、调用数、overflow=0、incomplete=0、ret=0。异常崩溃/SIGKILL不能提供完整收据。

已执行的隔离接口检查：

```sh
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 python3 multifloor_demo/navigation/test_results/publish_boundary/test_ros77.py
```

本次 `ros77_v1/result.json` 的 14 项实际 ROS 检查通过，无 hook 和有 hook 均接收全部 500 条精确字段/顺序，其他话题及同话题 C ABI 没有误采。四个进程正常退出。原调用中位耗时 10.017 µs、有 hook 11.780 µs；这是单次空闲接口观测，不能据此声称启动负载时完全没有时序影响。独立审查在 `slam/test_results/oct1_rcl_publish_observer_independent_review.json`。

源码 SHA256：`d9d0c5fbe7698beea90795fefa1a23d764b1ed9485cf3b81057f518c9fc9f7f6`。共享库 SHA256：`2c17086a81cad7ed2e3b18dafcf03ce1243436d5a05ad27b207656c0479cd4a4`。构建命令和各夹具 SHA 在 `build_receipt.json`。

发布边界有空窗且 IMU/clock 连续，支持发布或更新线程侧原因；publish_enter 连续而 return 阻塞指向 RMW 发布路径；发布连续但两个消费者缺收，指向 DDS/消费者路径；只有反馈节点缺收，才进一步审其 executor/运算。该 hook 不记录 ros2_control update 入口，不能仅凭发布空窗进一步断言是更新循环、RealtimePublisher 工作线程或调度器中的哪一个。
