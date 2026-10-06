# 冻结 Teacher 的训练契约与 CPU 对照

2026-10-03 实际运行；所有物理和策略推理为 CPU、单环境、线程数 1，无优化器、无重新训练。未修改或停止 PID 3603641 的其他训练。CPU Kit 无画面渲染，但首次测量仍分配约 288 MiB 的图形 CUDA context；不能把这称为完全没有 GPU 资源占用。

汇总收据为 `../test_results/training_reference_summary_20261003.json`。本文只说明独立 PhysX 参考，不构成 Gazebo、Sim2Sim、导航或真机部署的通过结论。

## 冻结契约

精确机器可读契约在 `../policy/contract.json`，实现和特权来源在 `../policy/observation.py`。

- checkpoint SHA256：`bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34`；直接 CPU 加载 `actor_state_dict/mlp.*`。ELU 247→512→256→128→12，无输入归一化，确定性均值动作。
- 247 输入依次为 COM 点线速度（机身坐标）3、角速度3、机身重力方向3、命令3、相对默认 q12、相对默认 qd12、DC 限幅后的实际 applied torque12、上一原始 action12、局部 height187。
- 各本体项先噪声、后 clip±100、后缩放；角速度×0.2、qd×0.05、力矩×0.01。height clip±1。参考禁噪声；训练噪声的范围与开启方式仍写在契约中。
- 关节训练顺序 FR/FL/RR/RL，各 hip/thigh/calf；默认 q 为 `[-.1,.8,-1.5,.1,.8,-1.5,-.1,1,-1.5,.1,1,-1.5]`。
- qtarget=default+0.25×raw action。没有 action clip 或通用目标裁剪。PD=25/0.5，DC effort/saturation23.5、no-load velocity30；physics0.005s、decimation4。
- height grid17×11，y 外层 x 内层，yaw-only 足迹旋转。20m offset 只抬高射线起点，观测参考仍 base frame z；height=base z−真实 hit z−0.5。因此 flat base z.35 时为−.15，不是常量1或5。扫描只使用 terrain/static mesh，排除机器人、自身和 moving_obstacle。
- 这里的SDF参考把所有静态碰撞组合成一个地形网格，因此上方楼板会进入该参考的扫描。归档训练只选择 `/World/ground/terrain/mesh`，并不要求扫描所有Gazebo摆件和楼板。固定地形目标或SLAM支撑层可作为新的明确provider评估，但不能宣称其几何集合与原训练完全等价。2026-10-04同物理原起点A/B及源码证据见 `../docs/TERRAIN_TARGET_TIMING_AUDIT_20261004.md`；保留本参考的原始输入和失败。

## 实际 flat 匹配命令对照

`../runs/isaac_cpu_matched_reference_20261003_c/` 记录 9 case、167 仿真秒、8350帧，其中45帧为初始化 PD，其余8305帧为实际 Teacher。每个 case 0–.1s 默认 PD，随后 Teacher 零命令至3s；单轴3–11s运动、其余零命令，slew=.6/.6/.8。switch 和 command_timeout 的精确时段在 schedule/protocol 中。timeout 此处是同命令时序参考，不是通信故障注入。

| 命令 | 5–10s平均实测 | 参考判据 |
|---|---:|---|
| vx +.3 | +.24332 m/s | 通过 |
| vx −.3 | −.27227 m/s | 通过 |
| vy +.2 | +.15302 m/s | 通过 |
| vy −.2 | −.15842 m/s | 通过 |
| wz +.3 | +.28691 rad/s | 通过 |
| wz −.3 | −.27049 rad/s | 通过 |

9 case 均无跌倒。停车持续 Teacher 零速度命令；没有把 action0 当停车。停车窗口最大漂移.006080m、平均XY速度.001261m/s、平均绝对角速度.01389rad/s。完整指标和提前冻结的参考阈值在 `matched_analysis.json` 与 `analysis_criteria.json`。

直接记录247输入→独立 CPU actor 回放，8305 Teacher 帧最大 action 差1.61e−6，通过。**新鲜物理快照重建全部247的严格 parity 未通过**：9个 reset 帧 gravity 缓存最大差.01591；79个 switch 帧的 height sensor 采样比当前快照迟一帧，最大差.002218。其他本体量最大差2.98e−8，记录 rawheight 裁剪后与 actor height 完全相等。量化在 `recorded_timing_diagnostic.json`，原失败和 NPZ 都保留。未用这些修正值替换真实 rollout 输入。

初版 `isaac_cpu_reference_20261003_b` 的8×10s对照曾遗漏 observation manager 的 `update_history=True`，造成三处命令切换一帧历史延迟；3992非reset帧纠正实际 actor 命令后回放误差1.27e−5。该早期 baseline 不冒充最终匹配时序。8个缺少原始247的reset帧保持未验证。初始 a 的日志接口失败同样保留。

## 真实坡道和低台阶 fixture

读取完整 Gazebo 静态 SDF，精确 box/plane 世界变换转成一个真正的 PhysX 三角碰撞 mesh；Isaac 原生187垂直 raycaster 命中同一 mesh。不是给 actor 合成坡度或假 flat height。各 run 保存 `fixture_world.sdf`、`fixture_collision_mesh.obj`、`fixture_manifest.json`、完整 hitXYZ、真实247、关节/力矩/接触记录与哈希。

参考仍为训练模型约16.087kg、nominal材料1/1；Gazebo Demo约16.512kg、机器人摩擦.7/世界1。随机化、噪声和 pushes 均禁用。命令、spawn、bootstrap与slew匹配，动力学不完全相同。

| Run / case | settled起点2–3s到最后1s | 同冻结数字判据 |
|---|---|---|
| ramps e / up | progress1.98318m；升.20973m | 通过 |
| ramps e / down | progress1.47281m；降.16918m | 未达1.5m，失败 |
| step05 f | x8.55876；机身升.024344m | 未达.03m，失败 |
| step10 g | x8.54068；机身升.065311m | 达x8.4及.06m，通过 |

上/下坡各18s，台阶各25s、3–14s运动。4个 case 都无跌倒，停车仍为 Teacher。实际 ray 对精确 SDF 非边界几何的最大误差≤4.58e−6m，记录247→actor最大差≤2.51e−6。各 `terrain_analysis.json` 使用未放宽的 Gazebo 数字阈值。四 calf 实测 net force 配合训练URDF足碰撞中心 FK 显示终点四足支撑；10cm记录姿态验证FK转动误差≤1.45e−6。但没有原生逐三角/逐地物 contact ID，报告将地物支撑归属明确标为几何推断。

坡道不是楼梯；低台阶仅验证上台阶平台，不是完整越障。d 首次新增 terrain import 在 Kit 启动前载入 pxr，触发本机 grpc重复注册，未进入物理步；失败文件和日志保留，e 修复了导入时机。

## 原导航起点诊断失败

`../runs/isaac_cpu_original_origin_20261003_h/`：spawn0,0,.4,yaw0、全程零命令，原三层静态碰撞与+20m扫描保留。原 native base_height 使用扫描最近ray，会把上方楼板当支撑面；此单独诊断明确禁用该提前终止，只用从 base 下方射线量真实离地高度检查物理失败。**策略247没有更改**。

计划15s，实际61帧/1.22s即失败。末快照1.20s真实离地.14754m，roll−.72168rad，XY位移.54688m。并非成功完成15s原训练环境。

首帧实际102个上层楼板 hit、85个底层 hit。`default_native_height_termination_receipt.json` 在已录tensor上执行精确原 source 函数，得到frame0 native scan clearance−2.0m、原 term=true，首个post-reset true=.12s。此收据是 counterfactual 函数回放，不冒充默认环境真实自动终止。`height_ray_hit_sources.json` 将实际XYZ几何匹配到SDF表面；来源归属不是原生triangle ID。最终训练进程存活和所有参考Kit关闭的收据为 `../test_results/training_reference_final_resource_evidence_20261003.json`。

**本起点存在独立边界 parity 失败**：t.02–.10五帧，每帧17中心行ray，共85ray，在floor3 y=0边界外3e−13至3e−11m。PhysX triangle ray错开上层命中底层；h运行时解析slab的1e−9边界容差包含上层，跨层误差2.4000038m。原数据和最大值保留。sensor pose匹配当前root至5.29e−12m，上帧pose不能修复；仅分析专用去XY容差后误差≤4.20e−6。见 `analytic_ray_boundary_diagnostic.json`。随后主任务保存旧实现并修正部署边界容差，相关Gazebo复测另归主任务报告；不重写h的历史输入和失败。两仿真在同起点失稳提示模型/场景域限制，但这不是唯一因果证明，也不能据此声称全输入精确匹配。

## 运行与复查

每个成功执行目录的 `execution.json` 包含完整 argv、线程设置、模型/脚本/schedule/world哈希、训练进程存活收据。`executed_source.py` 已对该 run 的 protocol hash 校验，避免后续扩展脚本混淆历史行为。

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /home/hyh001/IsaacLab/_isaac_sim/python.sh \
  multifloor_demo/teacher_mode/scripts/isaac_cpu_reference.py \
  --checkpoint /home/hyh001/projects/1.Project/RL_for_unitree/logs/rsl_rl/rl_unitree_go2_aer_height_distribution/2026-10-01_17-23-23_STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A-FORMAL-4096ENV-3000ITER-SEED42/model_1000.pt \
  --output multifloor_demo/teacher_mode/runs/NEW_UNIQUE_OUTPUT \
  --schedule multifloor_demo/teacher_mode/scripts/isaac_terrain_fixtures_20261003/ramps_schedule.json \
  --terrain-sdf multifloor_demo/teacher_mode/scripts/isaac_terrain_fixtures_20261003/ramps.sdf \
  --device cpu --viz none --kit_args=--/log/level=error
```

新输出目录必须不存在。flat参考去掉terrain参数，使用 `isaac_matched_reference_schedule_20261003.json`；original-origin使用相应schedule并明确加入 `--physical-clearance`。不要让这些参考脚本接管现有训练进程。
