# 仿真相机与畸变核验（2026-10-04）

结论：当前Gazebo观察相机和车载相机均未启用镜头畸变，实际ROS `CameraInfo` 中的五个畸变系数都是0，矫正矩阵R为单位阵。没有漏做去畸变的证据。观察相机的103°宽视野会产生明显的透视比例变化；这与镜头径向/切向畸变不同。浏览器按原4:3比例显示，采集器只解码颜色字节并编码JPEG，没有图像拉伸、重投影或人工画面。

| 相机 | 用途 | 水平视野 | 实际fx / fy | 图像 / CameraInfo |
|---|---|---|---|---|
| overview | 查看Teacher实际运动，未用于SLAM | 103.1324° | 253.93637 / 253.93636 px | `/demo/teacher/overview` / `/demo/teacher/overview_info` |
| vehicle | 车载RGB，后续视觉SLAM使用其自身内参 | 80° | 381.36116 / 381.36114 px | `/demo/camera` / `/demo/camera_info` |

两路实际图像均为640×480，主点320/240，`distortion_model=plumb_bob`、`D=[0,0,0,0,0]`。每路采到31条真实CameraInfo；6张存档overview图像全部能找到同一时间戳的CameraInfo。FOV计算的焦距与实际发布焦距最大差为0.0000107 px，符合浮点精度。车载参数还与原camera_mode的Pinhole模型、fx381.36114963和全零畸变一致。不能把overview图像与vehicle内参配对。

发现并修复了独立Teacher模式的接口遗漏：此前SDF已配置两个CameraInfo发布话题，但ROS桥只接图像。现增加两路实际CameraInfo桥接，并逐条保存源时间戳、frame、尺寸、D/K/R/P。未改旧camera_mode、原场景、Teacher权重或关节控制。

实际验证运行：[20261004_004114_stand_camera_intrinsics_check_r1_152b](../runs/20261004_004114_stand_camera_intrinsics_check_r1_152b)。15秒/751次CPU策略推理，站立和接口通过，worker/Gazebo/bridge/capture退出码全为0。该验证证明相机参数接口，未证明视觉SLAM或导航已通过。

原始收据：[camera_audit_20261004.json](../test_results/camera_audit_20261004.json)；逐条消息在该run的 `camera_info/overview.jsonl` 和 `camera_info/vehicle.jsonl`，实际图像仍在 `frames/`。

后续观察画面可用较窄实际视野并拉远/提高相机，减少边缘透视拉伸；不应用虚构的非零畸变系数去“修正”这些零畸变图像。独立启动器新增 `--overview-fov`（弧度）只修改无碰撞观察相机，默认值和历史录像不变。真实实体相机的标定、畸变和矫正仍未验证。

本机依据：Gazebo实际发布消息、生成SDF、原camera_mode/slam/camera.yaml，以及本机sdformat14相机schema的k1/k2/k3/p1/p2默认0。传感器/图像源与文件哈希均保存在本次run和审计收据中。

## 完整坡道两轮的实际80°相机补充

2026-10-04的两个完整ramp23首轮保留原机器人和场景碰撞，仅将无碰撞观察相机拉高/拉远至 `[8,15,7,0,.576,−π/2]`、水平FOV80°，没有用非零畸变系数或图像变形调整视图。两轮观察相机与车载相机实际发布均为640×480、D五项全零、R单位阵、fx381.3611603/fy381.3611412px；由fx反推FOV79.9999984°。观察图与自身 `teacher_overview_frame` 内参匹配，车载图与 `demo_camera_optical_frame` 内参匹配。

| 实际运行 | overview CameraInfo /存档图 | vehicle CameraInfo /存档图 | 同时间戳匹配 |
|---|---:|---:|---|
| ramp23上行首轮 | 46 / 9 | 46 / 8 | 所有存档图及最后实际图均匹配 |
| ramp23下行首轮 | 51 / 10 | 51 / 8 | 所有存档图及最后实际图均匹配 |

两路来源、时间戳、参数和原图哈希记录于 [full_ramp_audit_20261004/camera_evidence.json](../test_results/full_ramp_audit_20261004/camera_evidence.json)。四张证据图是原始JPEG的逐字节副本，未裁剪、去畸变、合成或改变宽高比。观察相机能呈现场景全貌，但640×480全景中的机器人较小；2Hz最后画面分别早于原生安全故障约0.40/0.49秒，因此实际接触瞬间的结论来自200Hz接触XYZ/normal日志。两轮完整运动均失败，正常相机参数和正常进程退出不能改写运动结果，详见 [FULL_RAMP_REPORT.md](FULL_RAMP_REPORT.md)。

## 8点接续：真实SLAM/SCAN导航三轮相机

V4三轮平地路线试验将无碰撞观察相机置于 `[6.5,3,2.2,0,.5,−π/2]`、FOV80°，实际画面能看见机器人走向目标后返回。观察与车载两路实际CameraInfo均为640×480、D五项全零、R单位阵、fx/fy约381.361px；图像仍按原4:3比例显示，没有人工畸变矫正。逐帧存档及最终RGB的传感器时间戳均能与自身CameraInfo配对。

独立来源、图像/内参哈希和数量见 [navigation_v4_camera_audit_20261004.json](../test_results/navigation_v4_camera_audit_20261004.json)。这三轮相机证据不能替代运动或导航验收；相应实际SLAM区域与Teacher停车由独立导航报告判定。
# 2026-10-04 动态与原子传输追加核验

新增只读核验 `test_results/camera_records_dynamic_20261004_v1.json`，涵盖普通V4.1 `dba1` 与动态 `6614/10c7/2f70` 共四轮。每轮overview/vehicle两路的所有存档JPEG及最后一帧都在实际CameraInfo中找到完全相同的integer stamp；全部CameraInfo为640×480、D五个0、R单位阵，fx约381.36116/fy约381.36114，水平FOV约80°。原JPEG不修改，页面按4:3原始比例展示。执行审计源码与输入文件SHA均保存；这证明当前模拟针孔相机配置和图像匹配，不是实体镜头标定。

随后V4.4 `1536` 和最终V4.5a `dc6f` 的两路实际存档RGB及CameraInfo分别通过同一只读核验，见 `test_results/camera_records_dynamic_v44_20261004.json` 和 `test_results/camera_records_dynamic_v45a_20261004.json`。尺寸、D、R、K以及逐张integer stamp配对均通过；当前图像没有需要去除的模拟镜头畸变。最终dc6f实际录像由90张原始overview JPEG按0.50–238.00秒传感器时间排列，间隔内保持已有画面，不插帧；编码在仿真结束后进行。相机通过与独立运动、导航、障碍判据分别验收。
