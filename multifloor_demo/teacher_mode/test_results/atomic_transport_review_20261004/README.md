# 原子命令文件的离线审查

2026-10-04，仅审查并执行独立源码副本；未修改运行源、启动ROS/Kit/Gazebo或发送信号。实际候选`navigation/bridge.py` SHA256为`c858e6898a284cb34a95de7d7e58fa0d4909e6e187fc82eab84b2c3cabc56ee5`。

结论：本地文件传输规则通过15项离线检查。四个并发reader对200次大小交替的真实写入完成887次完整JSON读取，没有半截或混合JSON。NaN序列化拒绝、注入的flush/rename异常均保留完整旧文件，临时文件正常清除，第二个writer因独占锁被拒绝。

真实`LatestCommandWriter`只写入阻塞中的第1条和最终第101条停车envelope，99条中间pending被显式替换并记录；原始wall/ROS源戳没有被写入完成时间覆盖。第1条移动envelope被刻意延迟后，真实`worker.navigation_command`在sim age仍为0时按wall age 0.3651秒判为stale，并给出零速度请求。新鲜envelope仍可接受；单独增加sim age至0.31秒也按原规则超时。TTL仍为wall/sim各0.3秒；测试没有放宽阈值。

与实际TTL run `20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9`的封存源比较，`CommandFile`的AST差异恰为删除一次`os.fsync(stream.fileno())`；`runtime_io.py`完整字节不变，`navigation_command`函数AST不变。新注释不计入AST；这里不对其他整份文件作无变化声明。

源和结果已保存：`sources/`是运行前副本，`review.json`是15项运行结果，`source_diff_review.json`是三项只读差异检查；全部通过。运行程序SHA256为`d6700795fe8ee7a4aaf9cbfa4942685b3b9ea5c9007dc87f209756d5bc8cf2aa`，结果SHA256为`08a85d545699cedc6ccdf8ce1000bbccb484b0955b9d67c6a0db412d329d2c15`，差异收据SHA256为`e47eacad534984ec5f1a04c0f7bf218097ba33d0d9c0ad3bca6a2f10c5bb440e`。

保留边界：同目录close后原子replace提供普通并发读取的完整旧/新文件，但不承诺掉电持久性；原先仅fsync文件、未fsync目录的代码也未完整保证rename在掉电后持久化。序列化、文件系统元数据操作和追加审计仍可能阻塞，不得用这次离线测试宣称实际延迟必然消失。注入测试只模拟Python异常，未进行真实崩溃/断电。零速度请求仍需Teacher停车过渡，物理停车与恢复须由新的实际仿真日志独立验证。

重复运行必须使用新的输出目录，避免改写本收据：

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/tests/audit_atomic_command_transport.py \
  --output /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/atomic_transport_review_new
```
