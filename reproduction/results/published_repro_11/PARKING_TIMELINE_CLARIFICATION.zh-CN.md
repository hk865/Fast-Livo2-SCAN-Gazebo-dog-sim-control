# 停车命令与首次失败时刻澄清（只读核查追加）

对象仍为本轮published_repro_11，原task/失败/停车结果不改，原始日志全部保留。此追加复用独立只读核查结果，没有新实验、控制器修改或调参。

|仿真时刻|真实事件|
|---|---|
|388.705|上层生成零意图|
|388.710|上层发布零；最后非零Actor帧的输入时间也是388.710，但属于不同链路边界|
|388.715|bridge接收零|
|388.730|Actor实际首次全零输入|
|388.750|native ACK，对应input19437|
|388.790|首次T0；上层/Actor/PID命令全零|
|388.910|超限源样本stamp|
|388.960|收到该SLAM样本|
|388.965|Controller真正锁定停车失败|
|389.015|task获知Controller失败；不是首次Controller失败时刻|

最后非零Actor实际输入是`(0.09810854, 0.01230069, 0.00477446)`，不是ACK内的`previous_slew_command`。后者`(0.08610854,0.00030069,0)`只是置零前中间值；此前若仅称“零输入前slew”，不得据此推断最后实际Actor输入。

T0时SLAM平面速度0.007266109m/s（stamp388.745、callback388.785），native0.020722347m/s。超限样本SLAM0.080310030m/s、对应native0.082239058m/s；native actuator在388.930达到0.084244623m/s。本轮不是T0起就持续高速，也不能以零速度目标证明物理静止。

146个后续Actor帧（388.730–391.630）输入全零；末段包含过期零保持，尚不足完整固定5秒，不授停车PASS。T0时rf_upper_leg_joint qtarget0.820510、tau−10.599420Nm，mode0/fault0；零速度命令并不等于零关节目标或零力矩。当前证据没有唯一确证控制根因。

只读来源位置：navigation_pid_history.compact.jsonl行5522/5545/5556/5557/5559；navigation_command_publications.jsonl行8555–8556/8562；student_actual_inputs.jsonl行19436–19452；navigation_slam_poses.jsonl行11444/11449；ordinary_terminal_failure_history.jsonl第1行；actuator.jsonl行97184/97199/97234/97242/97255；observations_actions.npz动作索引19435/19436/19439/19450/19451。这里是查阅索引，完整源日志/Actor数组没有上传，也不在清理候选中。
