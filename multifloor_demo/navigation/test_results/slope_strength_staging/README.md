# 实际坡道 .08 单变量验证

仅测试入口暂存于 excluded `test_results`。生产 NAV、SLAM、SCAN、步态与保护源保持原 SHA。沿用两点 B 的同一个 test-only wrapper，将行走偏航 cap 设为 0.08；纯转仍 0.12、增益 0.5，原始 IMU / SLAM / 独立执行桥保护、0.20/0.55 航向停车、原始到点 0.22 米保持 0.4 秒、NAV 90 秒和独立 0.30 米到点均不变。

完整传感器、FAST-LIVO2、SCAN、实际 NAV 和 test-only joint-stop adapter；初始 Gazebo spawn 为 `[4,2,.5]`，单次实际 IMU/SLAM 冻结场景轴后，请求相对世界 `[3,0,.3]`。仅 spawn 一次，导航不写 Gazebo 机身 XYZ。动态箱禁用，不混入正在准备的主动机身反馈。

严格 driver 由已审两点 driver 派生为一个目标，继续使用最大 0.15 秒真实 GT 插值括号、一次初始 SE3、原始 SLAM / 世界目标真值 / 同一 SE3 目标真值三项同时 ≤0.30、真实 NAV 到达确认、四足真实地面/坡道接触和机身零接触。没有外推、逐帧 GT 校正或 GT 导航。105 秒仅为组件清理界限，不修改 NAV 90 秒航点合同。

runner 在启动前检查完整 source_manifest、所有 runtime 库/二进制、同一个 wrapper、相同 cap / adapter、前次清理都与已完成的实际两点 B 一致。逐字复制所有测试入口和 SHA，记录实际 runtime cap 状态与 receipt。

域 73 唯一 owner 执行：

```sh
bash multifloor_demo/navigation/test_results/slope_strength_staging/run_slope_strength.sh --joint-stop-adapter --walk-yaw-cap .08 --match-control-baseline "$PWD/multifloor_demo/navigation/test_results/dynamic_strength_b_008" --output-dir "$PWD/multifloor_demo/navigation/test_results/slope_strength_008_actual"
```

失败原样保留并清理进程，组件通过也不能替代完整多楼层 Demo。
