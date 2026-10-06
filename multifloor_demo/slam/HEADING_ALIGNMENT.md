# 场景航点与 SLAM 初始朝向：最小静态校准方案

纯函数在 `heading_alignment.py`，8 项测试在 `tests/test_heading_alignment.py`。任务协调器已接入启动校准：缓存 400 条 IMU 与最多 15 对姿态，只在 `waiting_sensors` 中校准一次，校准成功后才允许起步。定位、轨迹反馈、地图仍使用 FAST-LIVO2，校准输入只有 IMU、SLAM 与声明的安装／参考外参，不使用 `/demo/ground_truth`。

## 当前 IMU 的参考语义

Gazebo Harmonic 的 `gz-sim8` 在创建 IMU 时把传感器初始旋转设为参考；未提供 `orientation_reference_frame` 时，输出相对于该初始参考。传感器计算为 `q_reported = inverse(q_reference) × q_world_sensor`。因此当前 boot-reference 消息不能无条件声称“绝对世界朝向”。已知 spawn yaw=0、安装旋转=0 时可组成已知参考，但换出生朝向或参考创建时刻不能继续假定 identity。[gz-sim8 源码](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/imu/Imu.cc)、[gz-sensors8 源码](https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8/src/ImuSensor.cc)

第五轮 `20260922_210015_6329a3` 的运行资产仍为 boot-reference。随后 `simulation/prepare.py` 已重新生成以下显式 world 配置，当前 generated URDF 和转换后的 SDF 均已核对，供后续新运行使用：

```xml
<sensor name="imu_sensor" type="imu">
  <imu>
    <orientation_reference_frame>
      <localization>CUSTOM</localization>
      <custom_rpy parent_frame="world">0 0 0</custom_rpy>
    </orientation_reference_frame>
  </imu>
</sensor>
```

当前世界没有 spherical-coordinates heading 偏置，故该参考为 world identity。若将来设置地理 heading，须按 Gazebo 的 world reference 定义重新核对。不能用已删除的旧 `initial_orientation_as_reference` plugin 标签代替此 SDF 元素。本机 sdformat14/1.9 的 `imu.sdf` 与官方定义一致。[SDFormat IMU 参考定义](https://sdformat.org/spec/1.11/sensor/)

这个模拟器从物理姿态生成 orientation，并未模拟真实 AHRS 的航向估计误差。真实六轴 IMU 没有磁北或其他绝对参考时，yaw 可以是任意启动方向；应使用已校准 AHRS／磁罗盘、已知初始朝向、视觉标记或地图重定位来提供同等初始参考。`orientation_covariance` 全零表示未给出协方差，不能据此推断现实航向完美。[ROS IMU 驱动约定](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0145.rst)

## 公式与接入契约

用 `R_AB` 表示把 B 坐标向量变到 A。设 W=场景世界、C=`camera_init`、B=机身、I=IMU、Ref=IMU 的已声明参考。取**同一传感器时刻**的 `/livox/imu` 与 `/demo/slam/body_odom`：

```text
R_WB = R_WRef × R_RefI × transpose(R_BI)
R_CW = R_CB × transpose(R_WB)
theta = yaw(R_CW)
p_C_target = origin_C + Rz(theta) × delta_world
```

- `R_WRef` 必须显式提供；显式 CUSTOM/world 且当前世界 heading=0 时为 identity。boot-reference 时必须使用已知创建参考，不能猜测。
- `R_BI` 来自机身与 IMU 安装外参，当前为 identity。不能忽略未来非零安装角。
- `delta_world` 是相对于场景锚点、沿场景 world 轴定义的航点偏移。绝对 world 航点先减去已知场景锚点。
- `origin_C` 是本次静止初始化机身位置。该最小方案只校准轴方向，**不估计初始化之前的未知平移漂移**。
- 只保留绕重力 Z 轴的旋转；楼层相对高度不随机身 roll/pitch 倾斜。先做完整三维姿态与安装旋转的抵消，再取 yaw。
- 初始化冻结一次后，机器人转弯不改变这个校准。不能每次行走都用当前机身 yaw 重新旋转场景路线。

例如世界机身 yaw=+5°，SLAM 当前 body yaw=0°，则 theta=−5°。15 m 的世界 +X 航点在 C 中需有约 −1.307 m 的 y 分量，变回世界后严格为 `[15,0]`。

接入时先缓存 IMU 时间序列，用 SLAM stamp 找相同／最近 IMU，时间差不超过 20 ms。只在启动静止窗口累积至少 10 对、跨度至少 0.8 s；圆周平均后的最大偏差不超过 1°，两参考的重力轴残差不超过 3°。还应由任务层确认机身位置／速度稳定、IMU orientation 有效且未用 covariance[0]=-1 标记缺失。失败时保持等待或明确失败，不能静默使用零校准。

应把函数返回的校准角、采样时间、来源描述、配对误差、参考 quaternion 与安装 quaternion 写入本次 mission／run 存档；阶段航点统一通过 `scene_waypoints_in_slam()` 变换。浏览器地图和 SLAM 位姿仍是 C frame，不能只改 frame 字符串。

评估器的 `floor3_target_matches_scenario` 与 `all_prescribed_waypoints_in_order` 均支持 `relative_world_axes`：用本次记录的静态校准旋转 scenario 全部航点，再加本轮 origin。它从 `attitude_pairs` 重算冻结角与采样统计，并核对每个 SLAM 姿态属于真实 `pose_audit.jsonl` 的初始化窗口，配对期间位移不超过 0.08 m。缺少实际配对、校准、来源包含真值、参考／安装 quaternion 不一致、采样不满足门槛或校准晚于移动均失败。旧 `relative_initial_body` 记录保持原平移语义；独立真值误差仍采用整程首个匹配的唯一 SE(3)，没有用验收真值生成导航校准。

## 已做与待做验证

```bash
python3 multifloor_demo/slam/tests/test_heading_alignment.py
```

8 项通过：5°符号／15 m 回投、非零 IMU 安装角、已知 boot 参考组合、±180°环绕、身体倾斜抵消、过期／倒退时序拒绝、无参考／非法 quaternion／不稳定估计拒绝、转弯后校准保持固定。

没有将倒地后的第五轮数据当成有效静止校准。新运行应在任何移动命令之前记录实际 IMU 与 SLAM 配对窗口，检查上述门槛并保存结果，再用 SLAM 反馈执行规定路线。评估器现有 34 项合成回归用例通过，涵盖旋转后的全部阶段航点与独立真值同时正确、未旋转目标、缺少／篡改配对、缺失／真值来源／迟到／矩阵不一致的校准、跳过中间点、乱序和分时抵达均被拒绝。实际长期跨层表现仍须由最终 acceptance 报告验证。
