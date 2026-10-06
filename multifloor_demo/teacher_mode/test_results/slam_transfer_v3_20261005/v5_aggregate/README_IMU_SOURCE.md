# 实际 IMU 来源与历史诊断更正

本次以原始 `adapter_IMU_inputs.jsonl` 字段为准：六次 V5 和保留的七次 V4 均声明 `orientation_available=true`，实际 ROS `orientation_covariance` 九项均为0。页面显示 unavailable headers 为0是正确的。这些实际试验没有运行“姿态不可用但 gyro 有效”的分支；该分支目前只有纯离线负例测试，不能将其称为实际验证通过。

历史 V2 没有保存原始 IMU 消息，只保存了拒绝理由。不能用静态 SDF 中的 −1 声明推断当时实际 ROS 消息也为 −1，更不能据此断言姿态源实际不可用。V2 源码表达式 `msg.orientation_covariance[0] >= 0` 可能返回 `numpy.bool_`，随后以 `is not True` 做身份检查，会把数值为真的 NumPy 布尔也当作不是真。V3 的记录器又实际报出 `numpy.bool_` 不可序列化，因此 Python 标量类型处理缺陷是有证据的解释；缺少 V2 原 raw 时不把它扩称唯一根因，不回填任何历史消息。

V4 已显式转为 Python bool 并对 NumPy scalar 序列化进行处理。V5 仅修复实际 JSON 读取完成时钟的竞态，仍保持300ms双TTL与原始头、收到墙钟、controller、gains、Teacher和physics。

逐 run 原始文件路径、SHA、true/false/缺失计数及 covariance 校核见 [actual_IMU_orientation_source_correction.json](actual_IMU_orientation_source_correction.json)，复核脚本为 [audit_actual_imu_orientation.py](audit_actual_imu_orientation.py)。全部旧 raw、收据及失败状态保持原样。
