# V19 分阶段性能与匹配输入协议

已完成最终 V19 库的新鲜有限数学/FP/team/loader 核对，以及新60秒传感器包的四轮 SLAM-only 1x ABBA。四轮全输入头与严格正常生命周期通过；未得到一般统计性能、导航或 Sim2Sim 通过结论。结果见 [REPORT.md](REPORT.md)。所有脚本只在本新目录写新产物，不修改 V17/V18 或已冻结历史证据；历史原始实验数据已删除，不能从摘要恢复完整回放。

## 比较范围

受控 `FASTLIVO_PIPELINE_MODE=serial|rx_decode|staged`：serial 为 V18 数学的串行工程对照，rx_decode 为一个 RX 同步解码线程与一个 estimator owner，staged 为快速 RX、点云和图像两个解码线程、按原 admission 序提交的单 owner。`FASTLIVO_IMAGE_COPY_OPT=0|1` 是另一个变量，主模式 A/B 先固定相同值；复制优化另作同模式 A/B，不能混成线程收益。

这些模式保留 IMU 递推、LIO→VIO 更新、迭代求解和地图更新的 owner 顺序。每次迭代只并行已冻结状态下的独立点或 patch；缓存随 pose/bias/map 更改失效。IMU 预积分与流水线不等于把每个 SO3 乘法开成独立线程。

正式事前门在 [PROSPECTIVE_PERFORMANCE_PROTOCOL.json](PROSPECTIVE_PERFORMANCE_PROTOCOL.json)。新实际运行必须重新绑定源码、加载库、环境、配置、模型、计时 schema 和控制门；旧 PASS 不能授权新模式。

## 一次采集，内容与顺序分别核对

由 root 在独占新 domain 中启动一次已有实际 Gazebo 任务。在首个传感器消息和静止 IMU 初始化之前开始录制 FAST-LIVO2 真正输入，首先 30 s，最多延到 60 s。不能另开一个 Gazebo 来产生“相同”输入。总原始预算 6 GiB；达到预算需正常停止整个录制，`--max-bag-size` 只分卷并不能限制总量。压缩/录制负载单独记录，录制当轮不作为未插桩性能基线。

```bash
# 采集由 root 在其实际启动窗口执行；离线 reader 不启动 ROS/Gazebo。
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<该次任务的独占domain>
ros2 bag record --storage mcap --storage-preset-profile zstd_fast \
  --max-cache-size 67108864 --max-bag-size 1073741824 \
  --output /var/tmp/go2_teacher_pipeline_v19_20261006/<新capture目录> \
  /demo/slam/lidar_filtered /demo/teacher/slam/imu /demo/teacher/slam/image /clock
```

停止录制并确认 writer 退出后，离线建立完整 CDR 身份索引：

使用当前安装 MCAP 的原生 chunk Zstd preset，使普通只读 storage reader 可直接读取；不用 rosbag 外层 file compression 解压时在 bag 目录产生新文件。若拿到另一种已完成 bag，索引器会拒绝外层 file/message compression，先在独立工作目录转换并绑定原来源。

```bash
python3 index_rosbag.py --bag <新完成bag> --storage-id mcap --out <新索引目录>
python3 compare_inputs.py --left <A索引/captured_inputs.jsonl> \
  --right <B索引/captured_inputs.jsonl> --out <新identity收据.json>
```

索引绑定每条消息的 topic、ROS type、原整数 acquisition stamp、完整 CDR 字节数和 SHA256，同时保存 bag record 顺序和 recorder timestamp。它没有 ROS node，没有 `rclpy.init()`，不操作传感器/机器人。CDR 相同只证明捕获内容相同；CDR padding 的不同也不自动证明传感器字段不同，失败时保留并进一步逐字段诊断。

**bag recorder 顺序不等于原 production callback 顺序。** ROSbag 回放同 bytes 经 DDS/executor 后，跨 topic callback 可以换序。exact 有限 A/B 需 test-only production direct-feed harness：读取同一完整 corpus，明确固定 admission 全序和 timer 全序，以新鲜、相同参数和初始化前缀从空 estimator/map 开始；或使用完整状态/地图 checkpoint。所有 modes 必须报告实际 admission 顺序 SHA。没有这一证据，就把 exact-order 因果标为 UNVERIFIED，不能用各 topic stamp 匹配替代。

新 capture 必须保留图像、原点云、IMU、calibration/offset、初始化前缀、输入类型/encoding/step/endian 和模拟 clock。只有 kind201 Hessian、残差摘要或状态表不能重建完整 `processFrame`。本目录提供离线索引、比较和单独授权的 ROSbag SLAM-only 回放；固定原全序的 direct-feed harness 仍为 NOT IMPLEMENTED，普通 DDS 回放不生成完整前端 exact math PASS。

## 独立 SLAM-only 回放操作

`run_shadow_replay.py` 需要显式 `--execute`，只使用私有 domain201–205。它启动自己的 camera parameter、mapper、只读 observer 和 bag player；核对 PID/start tick/PGID、实际 `/proc/maps` 加载库，结束只清理本轮进程。没有 Gazebo、Teacher、导航或执行器。正式候选库 SHA 和新数学收据硬绑定，改变库需重新验证；本入口不是物理运行门。

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -B run_shadow_replay.py \
  --capture-run <已完成capture> --index-dir <已核完整CDR索引> \
  --mode staged --rate 1 --domain 201 --out <不存在的新输出目录> --execute
/usr/bin/python3 -B summarize_shadow.py --run <该新输出目录> --out <新描述收据.json>
```

回放必须在与物理实验分离的独占性能窗口执行。1x ABBA 当前每模式2进程，只是描述性比较，不满足下述32独立批次门。

## 独立批次与 headroom

```bash
python3 schedule_abba.py --candidate rx_decode --scenes small medium large \
  --out <新serial_rx_decode_schedule.json>
python3 schedule_abba.py --candidate staged --scenes small medium large \
  --out <新serial_staged_schedule.json>
```

每 scene、每 mode 至少 32 个独立进程，交替 ABBA/BAAB；每批至少 10 warmup 与 30 有效事件/迭代。重复处理一次完整 corpus 必须恢复初态，不能把已有地图第二遍当同一输入条件。一个长进程的 30 repeats 不是 30 个独立 process batches。冻结相同 P 核 affinity、FP mode、线程实际 team、CPU 频率与外部负载；报告所有批次的 median/p95/p99/max，保留小数据和退化结果。

有限 headroom 用同 corpus 的 wall release schedule 倍率 0.5/1/1.25/1.5，保持原 acquisition stamps 和 CDR 字节、相同初始状态及同 timer schedule。记录入流速率、成功更新、队列/最旧 age、CPU、取消/拒绝/丢失。倍率改变的是离线释放速度；不称它是机器人或 Gazebo 实际速度，也不将 ROSbag 自己的 `/clock` 变速与 native wall timer 不同事件冒充 exact replay。之后才用实际模式做独立 60→210→完整 32 区域、停车和坡道验证，再扩到新 46 目标任务。

## 计时边界

V19 producer 的 schema 与字段必须固定后才可运行正式 reader；reader 拒绝旧 V17 CSV 或不同 kind300 语义混读。每条 committed 事件关联 admission seq、kind、source stamp、各阶段 wall/threadCPU/TID 和有界 bytes；生命周期另记录 admission 停止、inflight 结束、owner drain、context-valid close。

分别报告 receive→raw enqueue、raw enqueue→decoder pop、decode wall、decode threadCPU、ready enqueue→owner pop、owner commit wall/CPU，以及 receive→commit。未经过的队列是 N/A，不补 0 ms；RX 同步 decode 的 receive CPU 与 decode CPU可能父子嵌套，不能相加。owner Process2、LIO query/Jacobian/solver/map、VIO 与 output conversion/enqueue 是另一组计算计时，需 parent 标记：query⊂StateEst⊂handleLIO，不能相加。

`wall − threadCPU` 不是纯通信，包含被调度、锁等待和其他等待；process CPU 还含并发 decoder/OpenMP/logger。relay publish→mapper entry 是端到端 hop（序列化、传输、排队、调度），也不单独等于 DDS 网络通信。源 ROS acquisition time 与 monotonic wall不在同一个时钟域，不直接相减。source age需明确模拟 clock 或同 source head 的定义，继续保留控制器 300 ms 双时钟原保护。

## 生命周期与验收限制

严格正常完成要求 `accepted=delivered=committed=trace rows`，`canceled=rejected=missing=0`，failure 为空，context-valid drain，全部有界计数/因果/source 配对成立。V17 efde 的 accepted31615、committed31614、canceled1 历史 FAIL 保留；末条类型仍未验证。正常停机必须在 context 有效时先停止 admission、完成 inflight、原 owner 排空再关闭。紧急 context 无效/异常允许记录失败和取消，但没有清理豁免。

本性能 reader 不替代原单执行器、原实际 SLAM/SCAN、native/发布账本、原 300 ms、路线、坡道与停车验收；60 s 没有完成全路线时，管线完整性可以单独通过，导航仍未通过。actor 特权输入替换范围和真机未验证也保持原结论。
