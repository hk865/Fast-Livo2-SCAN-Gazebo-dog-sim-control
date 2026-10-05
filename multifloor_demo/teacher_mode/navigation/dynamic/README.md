# 有限平地动态障碍实验

本范围仍是待实测的独立实验。已有三轮真实有限平地 SLAM→SCAN→Teacher 往返通过只提供准入依据；全局 motion/Sim2Sim 历史失败、全局导航未验证结果保持原样。

唯一变化是已有 `moving_obstacle` 的初始位置和该模型的 Gazebo 位姿服务调用。生成器保留原世界/资产字节，只修改障碍初始 pose，校验其余模型、机器人、关节执行器和物理参数一致。Actor 仍使用原来的静态地形高度扫描，明确排除动态障碍；实际 LiDAR/相机可以看到这个真实碰撞箱。

冻结协议在 `protocol.json`。真实 SLAM 初始化后的固定原点和航向定义 1m 去、返回区域，单个目标期限仍90sim s；世界位置只用于放置障碍和诊断，不作为导航定位、路线或到达判定。原三轮V4因果离线回放在 `test_results/dynamic_flat_draft_20261004/trigger_replay_receipt.json`，均在 SLAM 前进 .118–.147m 时具备触发条件；这不是新动态通过证据。

实际 SLAM 前进 .10–.30m，连续 .3sim s/四个不同 SLAM 帧有 body vx≥.03，Teacher 接收的健康前进命令≥.08，才启动物理障碍。错过 .30m 门则拒绝晚入场。箱子在 x7.15 从 y−1.6 以.5m/s、10sim Hz 服务请求进入 y−.7；实际 Pose_V 在5mm内确认到位后保持10sim s，再撤回。服务 success 只记命令回执，实际位置另由只读 C++ 订阅器记录。位置缺失/陈旧、时钟回退、服务失败都停止进一步移动，不在清理时横扫已停车机器人。

独立分析必须确认真实点云触发障碍保护、Teacher 实际停车≥5sim s且≥1s连续平面速度≤.03m/s/角速≤.05rad/s，停车漂移≤.03m/.10rad，无200Hz实体碰撞/跌倒/执行器故障；真实注册云的guard连续清空1s、新的实际SCAN轨迹、实测复走≥.2m和2个SLAM区域分别记录。NAV恢复可以发生在leaving期间，完全按实际云/guard、新SCAN及实测停车条件；mover的clear仅表示实际箱子撤回原位，作为另一个fixture条件。导航不读取mover phase或模型真值。路线超时仍按失败保留，不能靠增加实验总时长免除90s目标期限。

`dynamic_obstacle.py` 的 mover 只调用模型服务，不发布 cmd_vel、导航安全旗、机器人位姿、关节或假点云。sensor_observer 只保存实际 raw LiDAR、注册云、车载80° RGB/CameraInfo 和独立 overview RGB/CameraInfo。云在进入/阻挡/撤离及撤离后5sim s保留原始XYZ，另保留header/接收时间/字段/原data哈希。两路真实RGB每sim秒存JPEG；顺序结束后输出全部云/图片SHA清单。车载内参与 overview 不混用。

构建（不启动仿真）：

```bash
cmake -S navigation/dynamic -B navigation/dynamic/build
cmake --build navigation/dynamic/build -j2
python3 navigation/dynamic/offline_checks.py --output test_results/navigation_v41_dynamic_offline_checks.json
```

主任务的已接 runner 接口：

```bash
python3 scripts/run_test.py navigation --navigation-stack --navigation-duration 180 --label slam_scan_atomic_transport_v41
python3 scripts/run_test.py navigation --navigation-dynamic --navigation-duration 240 --label slam_scan_dynamic_v41
```

两个命令都只能在主任务授权的资源窗口中执行；本代理没有启动它们。runner先生成10Hz全传感器资产，然后动态轮调用 `prepare.py --stage asset --run RUN`、`scoped_profile.py --dynamic --run RUN`。后者以三轮真实有限通过和所有动态源码/二进制/世界/协议哈希生成 `navigation_scope.json` 与 `dynamic_scope.json`。launch按显式scope启动服务桥、实际模型位置观察器、被动传感器存证和mover。普通flat轮不会启动动态fixture。

命令传输V4.1仅取消每条临时JSON的fsync，保留 flush→close→同目录atomicreplace/独占锁/原始stamp/300ms双龄门/至多一个在写和一个可替换待写请求。并发及故障收据在 `test_results/atomic_transport_review_20261004/`；不承诺掉电持久性，也不由离线通过推断实际延迟、停车或导航改善。
