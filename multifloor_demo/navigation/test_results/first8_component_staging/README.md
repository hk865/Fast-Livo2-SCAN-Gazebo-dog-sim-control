# 严格前八探索航点组件驱动（尚未执行）

由已经实际通过的单坡驱动派生；只提供测试目录中的驱动，不修改生产源码。实际启动与清理由 simulation owner 的隔离 runner/launch 负责，必须先归档此驱动、held helper 与 runner/launch 逐字内容和 SHA。

在完整生产传感器、SLAM、SCAN、NAV 启动并健康后，只执行一次真实 IMU/SLAM 标定，发送 `scenario.exploration[:8]`。只有 NAV 发布机器狗速度；GT 只在结果中按一个初始 SE3、最大 0.15 秒真值配对间隔检查每个实际 NAV index 对应的同时到达。动态箱保持禁用。

每航点仍由生产 NAV 执行 90 秒、原始 SLAM 0.22 米与 0.4 秒保持，独立到达仍为 0.30 米；原始 IMU 0.30/0.50 保护、纯转 0.12、行走偏航 0.08、前速 0.12 均不变。750 秒是整组件清理上限，900 秒是驱动外层墙钟上限，均不改变每航点合同。

实际只读状态必须含生产 `motion_limits`，不用旧版本 strength wrapper。现代 adapter 的状态由 aggregate Bridge 传播，同时此驱动订阅实际 `/demo/control/joint_stop_safety`。

启动形状（仅 owner 的 launch 执行）：

```bash
DEMO_TEST_ROOT=/abs/path/multifloor_demo DEMO_TEST_BODY_FEEDBACK_ENABLED=0 python3 /abs/output/probe_first_eight.py /abs/output/first_eight_result.json
```

候选 body feedback 要通过 Bridge 汇总 required health；驱动不接管或伪造该安全状态。启用/禁用对照必须使用相同完整生产源、运行库、驱动和初始场景，只允许改变明确记录的 body feedback enable 值。结果中无效/缺失样本不能视为成功。

实际 spawn 与生产 `simulation.launch.py` 默认一致为 `[0,0,0.30]`。`DEMO_TEST_BODY_FEEDBACK_ENABLED` 必须明确为 `0`（基线）或 `1`（候选）；标定前可等待 health 尚未出现，出现后或标定后，required 标志缺失、模式缺失/错误立即失败并请求 NAV 停车。aggregate failed 在标定前后都立即停止，不等单点超时。

11 个纯方法合同测试执行实际 staged result/health 方法（无 ROS）；包含真值尾端缺失失败、任务外 cleanup 样本排除、内部大间隔失败、候选错误 disabled 旁路失败、布尔类型与 required 标志、启动前 aggregate failure 立即停止。结果见 `acceptance_contract_result.json`，物理尚未执行。

新的唯一驱动变化是评估用 `/demo/ground_truth` BEST_EFFORT depth2000；其他订阅完全保留。13 个纯方法合同通过（原 11 验收反例加 2 QoS 选择性反例）。独立真实 ROS77：80ms 暂停，500 个 1ms 唯一 Odom 戳，旧 depth5 收424，新实际方法 depth2000 收500且顺序/字段完整，actual subscription 对象实测5/2000，两个进程正常清理。DDS graph 对 depth 返回0未知，首版因此额外断言 FAIL 原样保留；v2使用实际 subscription 对象收据通过。详见 `truth_buffer_freeze_receipt.json`。原 A 组件 FAIL 不回填。
