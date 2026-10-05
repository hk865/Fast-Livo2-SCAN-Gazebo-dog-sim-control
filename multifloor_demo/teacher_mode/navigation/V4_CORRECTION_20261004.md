# V4 导航接口修正及待实测项目

原四次真实导航试验全部保留失败。08:00资源空闲复测 `20261004_080143_navigation_resource_idle_v2_baseline_r1_137e` 有85条真实SCAN样条、0区域、6001帧Teacher全零命令。GPU前快照944MiB/10%，原忙资源V2为8090MiB/74%；同stamp实际SLAM回调晚于旁路的p95仍约489ms，超过300ms者119个。因此不能把旧阻断唯一归因GPU。比较收据为 `test_results/navigation_resource_comparison_20261004.json`。

V4 针对已量化问题改变独立Teacher导航接口，原相机Demo、FAST-LIVO2/SCAN共享源码、policy网络/观测/PD/物理、checkpoint、global acceptance 不变：

- **回调积压与双时钟**：bridge/sensor gate 使用 rclpy 唯一 ROS TimeSource 的实际`/clock`（本机Jazzy实现depth1），移除第二自建depth5 clock订阅；sensor、pose、cloud最新depth1，消费前仍核源时间戳。恒定clock不能被timer刷新墙龄，真实回跳锁定失败。
- **文件与完整性检查堵塞**：graph/hash轮询、原子command持久化、JSON日志、SCAN NPZ压缩离开ROS回调。命令最多一帧写入中和一帧待写，新的停车覆盖待写运动。command保留创建时真实wall/sim戳，不能延迟落盘后冒充新鲜；worker300ms与sequence/hash校验未放宽。实际写入历史记录queue延迟、write耗时、superseded数量；后台失败/证据队列溢出锁定停车。
- **初始化历史**：600个原始IMU戳/2.9秒跨度；body pose/注册云/raw LiDAR/image/CameraInfo各10个不同戳/2秒跨度。短缺帧不清除这些真实初始化历史。当前全部源、publisher链、实际image/K/D配对仍必须满足300ms；每次恢复仍要求2个新pose/0.1秒跨度。
- **请求期限**：90实际sim秒启动预算及600wall秒有限上界；超时锁定，不能超时后再偷偷冻结新路线。替代原固定100wall秒，区域原90sim期限不变。
- **Teacher转换**：通信hold立即归零但不冒充实际tilt，不清除仍关联的已检查路径。实际tilt高/严重/恢复阈值保持原样。Teacher停车证据来自零命令下SLAM body速度、SLAM角速和原始body gyro的0.3秒窗口，禁止纯等待计时证明停车；SLAM heading独占定位。每次真正转后重新确认停止，再前进。真实云障碍检查、清空1秒、重新SCAN路径保留。

离线 `navigation_interface_v4_offline_checks.json` 的37项以及 `navigation_interface_v4_controller_method_checks.json` 的9项通过；没有调用rclpy.init，没有启动ROS节点/Gazebo。慢磁盘阻塞期间101次提交不会阻塞回调，运动pending被零速替换；只记录实际写入的序列1/102，跳号不假冒送达。实际idleV2的SLAM/body twist与因果配对原始IMU回放仅证明这些真实数据能够形成停车窗口，不改旧试验为通过。所有具体字段、源文件SHA和计数在离线收据保存。

新配置只授权原safe flat的一米往返实验。运行后必须核验：实际command/action非零、SCAN来源关联、SLAM两个区域的原始到达与停留、物理停车/倾角/接触/围栏、源TTL与实际写入延迟、true sensor callback相对旁路延迟是否消除。缺帧应实际停车，不能因离线检查通过直接标导航passed。动态障碍、原46区域、多楼层、完整坡道以及真机均未在V4验证。
