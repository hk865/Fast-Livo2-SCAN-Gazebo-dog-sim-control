# 第15轮实测里程计运动窗口对照

结论：真实 SLAM 位姿可以识别低频运动失配，当前导航也已经使用它修正位置和航向。增加前进速度 P 反馈几乎没有上调余量；增加偏航速度反馈有可验证的候选价值，但现有观察器尚不能直接接入。较长窗口提高 SLAM/独立真值的一致性，也增大延迟并使现有线性拟合残差门大量拒绝真实步态摆动。先用相同分析检查实际 CM 时基修复后的新运行，再决定是否需要额外反馈。

本目录是新建的离线 excluded 证据。没有 ROS 节点、物理运行、Twist 输出或生产源码修改；没有使用真值生成控制。第15轮原来的区域验收失败保持原样。

## 输入与公平比较

重放 `runs/20261001_203152_88725b` 全部 10987 个 active SLAM 原始整数 Header，包含探索和实际 `returning`。GT 仅送入另一个评估实例，使用先前冻结的初始 SE(3)、最大 0.15 秒真实括号内插值；没有重新拟合或外推，本轮所有 active pose 都有对应 GT。

使用原 `measured_motion_design/motion_observer.py` 不改一行，分别声明 0.4/0.6/0.8/1.2 秒窗口，最小 span 为各窗口减 0.05 秒，保留相同 .02m/.04rad 拟合门、150ms pose gap、100ms applied-command gap、运动模式/适配器状态/目标与轨迹身份切换拒绝。0.4 秒结果准确复现既有报告的 6186 个 valid 与 4449 个 valid walk。

实际 CHAMP command 的时间是 adapter 归档的浮点仿真接收时间重建 ns，并非 Twist 原生 Header；轨迹身份来自稀疏实际状态的因果匹配。SLAM pose 日志没有 callback 墙钟，故没有验证实时消息年龄，重放按 ideal age 处理。现有 `observed_motion_ready` 的 `driving`/实际 `walk` 枚举缺口保持记录，`feedback_eligible` 始终 false。

## 窗长、延迟和切换

| 窗口 | 有效总窗 / 10987 | 可拟合 walk / 有效 walk | 拟合时刻距末端均值 / P95 | 实际执行切换拒窗 | 拟合残差拒窗 |
|---|---:|---:|---:|---:|---:|
| .4s | 6186 | 6521 / 4449 | .210 / .233s | 1912 | 2853 |
| .6s | 1892 | 6208 / 1106 | .311 / .338s | 2571 | 6473 |
| .8s | 787 | 5911 / 413 | .416 / .440s | 3101 | 7031 |
| 1.2s | 206 | 5357 / 76 | .614 / .643s | 3894 | 6781 |

这些是重叠窗口，不是独立试验。延迟尚未包含发布/传输/消费延迟。

切换拒窗中，实际 walk/turn/zero 模式变化分别影响 1050/1521/1984/2705 窗；仅轨迹身份变化分别影响 421/630/839/1240 窗。不同原因可以重叠。长窗不能在恢复、归位或新 SCAN 后继续使用旧响应；没有为了覆盖率把这些门绕开。

walk 的残差拒绝主要是角度线性模型失配：2072/5102/5498/5281 个 walk 拟合被拒，独立 GT 同窗也超过同一拟合门的数量为 2045/5074/5476/5272。GT 验证说明许多拒绝对应真实非线性身体摆动，不能把它们直接当 SLAM 不准，也不能降低安全门来获得更多反馈。需要另行验证适合周期运动的估计/质量模型。

在四种窗口都能拟合的同一 5357 个 walk 末端时刻，SLAM 减 GT 的绝对误差 P95 如下。它们覆盖不同历史区间，不能据此算 plant gain。

| 窗口 | body vx误差 P95 | body omega_z误差 P95 | world heading-rate误差 P95 |
|---|---:|---:|---:|
| .4s | .00987m/s | .00253rad/s | .00255rad/s |
| .6s | .00710m/s | .00140rad/s | .00144rad/s |
| .8s | .00559m/s | .000985rad/s | .00103rad/s |
| 1.2s | .00385m/s | .000699rad/s | .000699rad/s |

## 反号是否持续一个步态周期

名义周期为 `.25s swing + .35s stance = .60s`。stance 来源是本次不可变源码档，phase generator .25s 来源是外部 CHAMP 源码，当前 CHAMP 库 SHA 与 full15 原始 runtime SHA 相同；该 header 本身没有收进旧源码档，证据明确写出这一边界。没有由足接触推断真实周期相位，故这里只说**名义周期**。

比较 actual averaged body yaw command 与测得 body omega_z，分别保留 world heading rate，绝不把二者在坡上无条件互换。符号比较要求两侧绝对值均大于 .02rad/s；连续段不跨 invalid、目标/轨迹身份或命令符号变化。另排除同窗内 meaningful yaw command 改符号的诊断分类。

| 窗口 | 同窗命令符号一致且SLAM/GT共同确认的反号窗 | 同context连续分类最长时长 | 连续分类达到名义 .60s 的段数 |
|---|---:|---:|---:|
| .4s | 1008 | .300000001s | 0 |
| .6s | 228 | .700s | 1 |
| .8s | 52 | .399999999s | 0 |
| 1.2s | 5 | .199999999s | 0 |

不能由 0 的段数推断没有持续偏航：拒窗会中断分类，长窗有明显选择偏差。

明确的跨周期证据是返航 `request ...:return_origin:2 / index1 / trajectory120`，1031.0–1031.7s 连续八个 .6s 窗口。actual 平均 yaw command 为 −.0477…−.08rad/s，SLAM body omega_z 为 +.0446…+.1892rad/s，独立 GT 为 +.0451…+.1884rad/s；命令同窗始终为负。连续分类覆盖 .7s，各测量窗口本身覆盖一个完整名义周期。这比单帧反号更强，但仍不是固定反向增益或所有地形/历史的证明。

最后七个末端 actual command 已为 −.08rad/s，继续负向修正余量为零。新增保持 ±.08 的反馈项无法在这些时刻施加更强负指令。它至多可能在更早尚未饱和的阶段改变响应，必须实际单变量验证，不能据此宣称已修复。

## 调节余量与最小策略

0.4 秒有效 walk 的实际末端 vx 平均上调余量仅 .00147m/s，中位 .000281m/s，3988/4449 末端已经 ≥.118m/s；而平均实际 vx 约 .0785m/s，相比平均 applied .1172m/s 亏损约 .0387m/s。前进 P/积分不能在原 .12 cap 内补足这个差值。纯转实测 retreat 仍须由执行层证据解决，不能偷加前进或负向速度抵消。

0.6 秒同符号、GT确认反号的 228 窗中，109 个末端已经 yaw ≥.078rad/s；同方向修正余量中位仅 .00426rad/s。1.2 秒的五个同类窗全部末端近饱和。因此“加反馈”不是普遍有执行余量的修复。

建议最小顺序：

1. 保持 NAV/gait/caps 原样，在 CM 时基候选通过实际验证后，用相同脚本检查新 full run 的响应、纯转退移和反号是否仍存在。JTC stale effort 的时基问题不能靠 SLAM 增益掩盖。
2. 先把观察器接成只读接口，保存 native SLAM Header、callback 墙钟、actual applied command、phase、归位/保护、当前 goal/SCAN 身份；明确实际 `walk` 枚举并验证状态失联/重复 stamp/零边沿。没有实时健康证明时反馈仍关闭。
3. 若仍出现未饱和、同符号、同实际 drive/SCAN、覆盖至少一至两个名义周期的可信偏差，才作单变量低频 yaw-rate P 候选。候选只能消费 SLAM 的 body omega_z 与相同有限窗的 actual body yaw reference；不要拿 world heading rate 直接减 body command，不把慢窗当即时速度。可预声明小修正上限例如 .01rad/s，不使用积分，最终仍裁剪 ±.08、保留现有角加速度/heading .20/.55 制动/IMU .30/.50 保护。增益必须先声明并经隔离试验，不按 GT 拟合。
4. hold、归位、pureturn、路径更换、陈旧/不可信窗口均禁止该项，清旧窗；没有可靠数据时保留原 NAV 行为。上限饱和只报告“无执行余量”，不扩 cap、不伪造恢复或到达。
5. 同源 A/B 先比较 fresh 平地混合运动及“先转后走”的短坡两种历史；GT 仅一次固定 SE3 后独立评估。唯一变量是反馈开关，保存实际 JTC period/关节/foot/IMU/指令与响应，再决定是否进入路线试验。当前四窗离线结果不足以采用其中任何一个作反馈。

## 复现

```sh
python3 multifloor_demo/navigation/test_results/motion_window_comparison/test_analysis.py
python3 multifloor_demo/navigation/test_results/motion_window_comparison/compare_windows.py \
  --output multifloor_demo/navigation/test_results/motion_window_comparison/new_output
python3 multifloor_demo/navigation/test_results/motion_window_comparison/summarize_comparison.py \
  multifloor_demo/navigation/test_results/motion_window_comparison/new_output
```

新 full run 可指定 `--run` 和已冻结初始变换文件 `--alignment`，后者须含 `initial_SE3`，不得按每段重新拟合。默认 observer 保持原版不变。7 个纯分析反例通过：常速延迟、真模式切换、仅轨迹切换、周期摆动、持续反号、invalid/context/sign 不跨段、GT 缺尾/大括号不外推。首次序列化 numpy.bool_ 错误及部分输出留在 `full15/failure.json`，修正只在新分析脚本显式 bool 转换，实际结果在 `full15_v2`。
