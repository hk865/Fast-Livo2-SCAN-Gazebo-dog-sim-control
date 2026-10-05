# 精简实验记录 / Compact experiment evidence

本目录保存已有实验的原始小型收据、报告、配置来源、派生曲线及有限真实画面。`history_manifests/*.json.gz` 是历史全字节清单的无损压缩副本，列出文件不代表那些文件仍存在。

用户已要求删除大体积原始数据。删除后，本目录不能从原始点云、诊断、物理和策略逐样本日志重新执行历史完整验收。保存源码可以重新运行新实验；不能还原已删除的历史输入。`EVIDENCE_MANIFEST.json` 记录本目录每个文件的实际SHA、原来源、尺寸和用途；旧原始数据SHA来自删除前封存清单，不是新的完整重审。

## 关键结论与位置

|记录|范围|精简内容|
|---|---|---|
|V12_fbcb|单次实际32区域、两条12m坡道、首固定5s停车联合通过|`receipts/V12_fbcb/`，旧common/v2/ramp未验证状态保留；新metadata-v2修正独立通过|
|V18_prefix_ac02|210s有限前缀、25/32；不能冒充完整导航|`receipts/V18_prefix_ac02/`|
|V18_full_ab53|单次264.6s实际32区域、两条完整12m坡道和停车联合通过|`receipts/V18_full_ab53/`，原收据与metadata-v2独立结论分别保留|
|V17_efde|60s前缀8/32，严格管线失败|`receipts/V17_efde/`；accepted31615、committed31614、canceled1，不豁免尾包|
|height_failure_6fdf|旧真实SLAM高度故障诊断|`receipts/height_failure_6fdf/`及保留的高度派生图和指标|

完整报告复制在 `reports/docs/`；较新的独立报告复制在 `reports/test_results/parallel_pipeline_20261005/evaluation/{actual_v18_full,actual_v18_prefix,actual_v17_smoke}/`。V12完整报告在 `reports/test_results/lidar_density_rate_20261005/evaluation/FULL_REPORT.md`。

`curves/<label>/` 有每0.2s时间桶首行加最终端点的有限字段JSONL；`figures/<label>/` 有轨迹/速度PNG和三张实际相机画面。采样可能遗漏跌倒、尖峰、接触、TTL失联等瞬态，不能用它重新认证原安全门。原SLAM camera_init与native world画在独立坐标面板；相对高度只去除各自首z，没有SE3注册，也不是定位误差图。

Teacher导航仍使用实际SLAM/IMU及SCAN，Actor保留232维特权仿真输入；15维是命令与上一动作。这里的三层连接是坡道，不能称真实楼梯。未证明随机鲁棒性、动态障碍整线、全部传感器替换Actor、真实机器人或新控制器完整Isaac跨仿真等价。

旧5/10cm持续登台各3次功能通过，旧机身增高比例与兼容summary失败保留。真值PID标定、曲率tight-S失败、初版完整开环坡道失败、IMU/关节末帧新鲜度失败和各次20/30Hz实验不因最终单轮通过而抹去。

## 构建本目录

`assemble_compact_evidence.py` 只读原目录，复制小型证据并抽样；需要原数据尚未删除、Python NumPy/Matplotlib。脚本保留旧机器绝对路径用于历史溯源，不是新机器运行器。新实验复现应使用仓库的独立构建和启动说明。
