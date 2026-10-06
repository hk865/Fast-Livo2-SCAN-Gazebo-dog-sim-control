# V17 actual smoke 的只读管线验收

准备状态，尚未读取或声明任何实际 V17 smoke 通过。所有文件限本独立目录；V16 源码／原证据完全不改。

`audit_pipeline.py` 固定事前60s标准 SHA `85a39bc1adc538e468207fd785348bf32630c66772ebadc985f9fcf096342867`，时间分段标准 SHA `709d4cf1e906cb869b82411c5a399ac0e85597645e438bd513518404b1cbbb96`，V17实际 callback 源码 SHA `39195ef48be4a72a01d988aba8c75d2b3fadb8c71c296d4cb8b6a9999427f727`。

运行必须在 root 给出已结束的 V17真实 run 后，避免与实际物理或性能窗口争 CPU/I/O：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/pipeline_runtime_audit/audit_pipeline.py \
  --run /absolute/actual/V17/run \
  --output /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/test_results/parallel_pipeline_20261005/pipeline_runtime_audit/ACTUAL_RUN_RECEIPT.json
```

输出新文件使用 `open('x')`，拒绝覆盖，不写实际 run 或其他目录。读取 CSV、最终summary、原二进制/index/writer-final、小 scope/runtime/env/实际加载库与来源快照。

严格生命周期判据：accepted＝delivered＝committed＝CSV完整行数；canceled/rejected/missing 均0。FIFO sequence 必须从1连续，不能删除末段；唯一RX TID、唯一owner TID且不同，owner绑定实际mapping PID。接收／解码、owner提交必须不重叠且单调。post-pop计数最多511、字节小于64MiB；最终峰值不超过512／64MiB且覆盖所有post-pop观察。字节统计仅代表 ready queue，不能声称覆盖全部RSS／DDS backlog或精确重建未记录的每包capacity。

原 RAW kinds1/2/3 与每个已提交传感器事件按全序逐项比较整数source_ns、precision17转回的IEEE754 receipt；不能用dequeue墙钟刷新receipt。Timer kind0需保留source0／decode-wall等于receipt／decode-CPU0。未记录完整点／图像 bytes，不把原RAW元数据匹配当作本次实际传感器解码逐字节证明。

输出 receive→decoded、decoded→pop（含push/FIFO驻留）、pop→commit_end 与RX线程CPU分布；后两者不是独立IPC传输分段。LIO/VIO有效频率只计原diag_stage flag1，flag0跳过另报。源头年龄输出最新因果已接收sensor header与处理目标的差，不把它当作当前ROS `/clock` age，也不改变原控制器300ms双时钟TTL门。

kind300 stage8是owner commit batch，stage9/11排除已移出的解码，stage12对当前pipeline为N/A，不能用其极小数值宣称原preprocessing免费。嵌套段不相加；进程CPU包含并行RX/writer等线程；wall−CPU不能一概称为通信。

`pipeline_integrity_status` 与整体smoke分开。原native运动／发布账本／300ms门必须由原common和独立publication reader审；此脚本明确留该项UNVERIFIED，不推造PASS。GPS由实际冻结配置、真实加载同源startup guard和事前17case semantic receipt关联，缺字段不猜false。60s不能通过完整32区域、两条12m坡道或最终5s停车，本脚本永不声明导航PASS。

`finite_reader_tests_v2.json` 的22项极小人工负例仅证明reader拒绝canceled清理豁免、rejected、delivered未提交、缺raw、换header、receipt刷新、外国TID、sequence／全序错误、逆时钟、容量／峰值矛盾和非法timer。有限参考样本不是实际仿真证据；原首次测试收据保留。准备时没有启动ROS、publisher、Gazebo、训练或CPU统计基准。
