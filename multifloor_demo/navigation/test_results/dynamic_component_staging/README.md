本目录是暂存的实际组件测试，尚未执行，未修改冻结的生产导航/SLAM/SCAN/步态。先完成 Sim owner 的同版本物理 A/B 并交接域 73，再由 root 授权这一组件运行。不能与其他域 73 仿真并行。

最短启动命令（新输出目录必须不存在）：

```sh
bash multifloor_demo/navigation/test_results/dynamic_component_staging/run_dynamic_component.sh \
  --output-dir "$PWD/multifloor_demo/navigation/test_results/dynamic_actual_001"
```

默认保持生产执行链。如果 root 在实际 A/B 后明确选择了测试关节停车适配器，才在同一命令追加 `--joint-stop-adapter`；manifest 会记录选择。该选项不改变 NAV 的 0.22 m、0.4 秒稳定到达、每点 90 仿真秒、联合 0.30 m 到达或倾角/障碍保护。

runner 将 driver、launch、runner 三个文件逐字复制到输出目录并保存三个 SHA256，另复制环境加载 wrapper。实际 launch/driver 从这些已归档文件执行，避免 `test_results` 被通用源码 snapshot 排除后无法追溯。完整生产源及解析后的动态库另存 `source_snapshot.tar.gz`、`source_manifest.json`、`runtime_manifest.json`。全套子进程只有一个自有进程组，退出后检查并清理该组。

真实启动顺序是完整 Go2 物理与传感器、原静止初始化门、FAST-LIVO2、SCAN、NAV。driver 等 SLAM/相机健康、原始 IMU/SLAM 时间配对与执行桥 ready，只做一次固定校准，把 `[0,2,0]`、`[2,2,0]` 转到实际 SLAM 帧；机器人初始创建位置为 `[0,0,0.30]`，没有运行中姿态重置。只有 NAV 发布 `/demo/cmd_vel`。driver 仅发布任务、实际状态的 mission heartbeat、障碍启用和终止消息；不发布身体速度，不调用机器人位姿服务，不用真值校准或转向。

SceneTrigger 收到的 NAV request/state/index 都来自实际订阅。它在相同 request 的真实 `running/index1` 后按原 1.4 m 半径自然触发箱体的 8+20+8 秒运动，不制造 index、提前触发或强制箱体留场。实际箱体状态及 Gazebo pose 服务成功次数、requested/safe 指令、完整 Bspline/Path、原始传感器、native 首障碍精确输入均保存。

必须同时满足以下结果才能 PASS：

- 实际 NAV 按原始门限确认两个点；同一初始 SE3 的独立真值检查和已观测 NAV index 对齐，两个有序航点均在 0.30 m 内。
- 箱体实际经历进入、阻塞、离开和清除，真实服务更新成功；NAV 真实 obstacle hold，同时 requested/safe 为零。
- 至少一个匹配 request 的 native 精确触发云 JSON+NPZ 完整提交，SHA 一致；既有保护阈值保持。
- 清障后接受新的当前参考/轨迹，并观察到实际执行桥恢复前进；不能仅凭 metadata 或计数认定恢复。
- 身体无接触，四足接触仅来自地面/坡道，原始倾角保持在安全包络内；只存在 NAV 身体指令发布者。

任一缺项会在 `dynamic_result.json/missing_acceptance` 明确失败。比如 SCAN 在 guard 之前停车导致没有 obstacle hold/native 事件，也会保留为失败，不替代成功证据。210 仿真秒是组件清理边界，两点各自仍为原 90 秒合同；420/450 秒墙钟边界只约束 driver/runner 生命周期。完整多楼层 Demo 的通过仍需另行整合验收。
