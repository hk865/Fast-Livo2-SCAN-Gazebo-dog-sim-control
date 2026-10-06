# V4 capture→active_hold 源码只读复审

结论：未发现阻止冻结实测的相位切换、重复积分重置或保护绕过。仅为源码审查，不是物理停车通过；本次未启动仿真、Teacher、Actor推理或ROS，也未改 core/profiles/旧run。root报告两解释器13项pure checks均PASS；本审查不将纯检查写为实际运动通过。

|冻结候选来源|SHA256|
|---|---|
|core.py|`9eca389f54277986656c0ccda3da42f01d9cbc25ed15d99b57cc9a1e51087384`|
|profiles.py|`52453b746c5933a63d9153c8bcccd99324813026b5150346e76ecc7d95208c54`|
|pure_tests.py|`734bfd39ee4c99ce5fa32c3daf58073bc6172c9e5b9921380034980dce5542fd`|
|v1完整前21098字节|`82d07762f2d44c969abafb65a792881fddce83dc5ca22851b376468ec48e597d`|

1. 首次旧完整进度/到达候选进入capture时，独立hold_integral清零恰好一次；接管前的drive源码完整字节prefix未改。保存parent的slew估计旧值，再恰好执行一次20ms Teacher slew，避免将旧zero slew与新active输出重复推进。entry_integral_reset仅该帧true，之后元数据false。
2. `_hold_fresh`先按新鲜真实stamp和位置/朝向/速度判断capture驻留，再设置唯一hold_stamp/completed_t；随后才计算mode/phase。因此首次declared帧已经使用hold P=.18、D=.2、refXY≤.025，而不是capture P=.8、D=.2、refXY≤.05。两段yaw P/D=.65/.18、参考yaw≤.07均保持。
3. `_limit_hold`与_metadata都由同一个hold_stamp选择相位限制：capture为[.15,.07,.10]，active_hold为[.06,.04,.10]，并按该相位vx限平移范数；两次raw候选/限幅重算均使用同一相位。日志新declared帧的mode、参考和parking_command_limits一致。
4. 切换不另重置hold_integral。内环保留原COM速度/ωz PI、滤波、candidate裁剪，先按最终轴/范数限制阻止积分，再按原[.6,.6,.8]/s downstream slew阻止积分；held帧只保持数学状态、每Teacher帧推进slew。首次capture已被parent滤波的那帧不重复滤波。
5. capture驻留仍为entry XY≤.025/yaw≤.035，驻留保持XY≤.03/yaw≤.045，真实原点速度<.03和Euler yawdot<.06，fresh stamp连续.6s；没有扩大阈值或用held重复头凑时间。目标为事前固定endpoint/heading及其canonical SHA，实测arrival只诊断；元数据list复制防止日志污染内部锚。
6. 后续每Teacher tick先检查当前仿真头dt和previous wall receipt gap，之后才在updated分支重置last_feedback/last_wall。completed后仍继续检查300ms；非法状态、回退钟、目标丢失保持保护。原fixed5s/.05m/.1rad停车契约与原Teacher连续推理/action非0停车方式保持。
7. 跨NumPy修复使用已冻结profile target的exact canonical身份，再与相同pathSHA的解析endpoint数值≤1e−12比较（比此前建议1e−10更严格）。已知两个解释器末位差约1.8e−15，未改变真实固定终点或物理门。

执行和验收需保留：capture→hold前已接受的Teacher命令只能按原slew衰减，requested限幅收紧不保证同一帧已应用命令也立即落入hold envelope；实际read/requested/command记录及首declared固定5s窗必须如实审核，不能为避免过渡峰值重选后窗。捕获时COM杠杆的期望omega_xy=0假设仍与原版一致，实测原点与Euler yawdot使用完整ω。未因本次审查宣称capture/hold峰值、随机扰动、真实SLAM/SCAN/多层或真机通过。

确认source/protocol/receipt都READY且事前注册后，可由root独占执行新先导。旧v1零命令停车FAILED、v2启动无physics失败和v3捕获问题仍按各自原件保留。
