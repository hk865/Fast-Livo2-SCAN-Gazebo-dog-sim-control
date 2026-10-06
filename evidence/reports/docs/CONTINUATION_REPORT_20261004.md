# 2026-10-04 08:00 接续报告（最终分范围结果）

定时任务已唤醒并停用单次定时；沿既有交接、真实源码快照与失败记录继续，没有重新训练Teacher、操作真机或覆盖相机Demo。

## 当前已证实

| 层级/测试 | 实际结论 | 范围 |
|---|---|---|
| CPU策略与关节接口 | 通过 | 冻结247→12网络、逐名关节映射、200/50Hz、训练PD/DCMotor限幅、唯一执行器 |
| 基础运动/低台阶 | 通过 | 平地各命令原三次；5/10cm持续登台各三次；不再以机身升高比例判登台 |
| 完整坡道/全场景Sim2Sim | 未通过 | 原ramp23上下行均失败；lower12 provider也未到出口；三层连接为坡道 |
| 真实SLAM/SCAN有限导航 | 通过 | V4三轮和V4.1一轮；目标中心相距1m；0.17m控制区域/连续0.6s；不是精确走满1m |
| 真实命令中断停车/恢复 | 通过 | 新单次10sim秒唯一命令bridge暂停；原时间戳与300msTTL保留 |
| 动态物理障碍有限闭环 | V4.5a完整运行通过 | 240秒/12001帧，19/19独立判据、原始点云覆盖、两SLAM区域、停车恢复与全部进程退出通过；此前四版失败原样保留 |
| 多楼层导航/真机 | 未验证 | 完整坡道运动已失败，不能宣称全路线或真机通过 |

资源空闲V2对照仍85条SCAN、Teacher全零、0/2到达，说明GPU竞争不是唯一原因。V4修复ROS回调积压、重复时钟、预热及共享CHAMP停车/heading复位后，三个真实180sim秒运行均完成非零Teacher控制、两处原始SLAM区域判定与最终停车；全部6个自有进程退出0。控制仍有约30–37%过期/不健康输入帧，源年龄继续计算，没有刷新旧包或放宽300ms。

| V4轮次 | 有限导航 | 3秒最终停车XY最大漂移 | 偏航漂移 |
|---|---|---:|---:|
| 9c33 | passed | 1.726mm | 0.038706rad |
| 1c57 | passed | 0.581mm | 0.014047rad |
| bd19 | passed | 0.241mm | 0.010524rad |

真实单次中断发生在/clock7.43..17.43s，原envelope source7.475s；最后accepted world7.77s age.295s，firststale world7.79s age.315s，requested立即归零，actor按既有限速到8.01s归零并持续推理。预定world10.79..13.79s停车窗漂移5.012mm/偏航.03642rad，601原生样本、151CPU策略帧，无机身碰撞或助力；恢复新鲜非零命令后实际推进.78945m并完成两处区域到达。普通短暂过期事件总项仍unverified，专项通过仅覆盖这一完整注入事件。

实际IMU/关节共30维替换的新三轮清洁空闲前进均18秒、来源/247重建/CPU回放/停车通过；旧优化三轮2通过1墙龄拒绝保留，另一次runner清理终止不计完整运行通过。该旁路还剩190维COM/高度特权与27维控制器已知状态。导航测试Actor尚未启用这30维替换，使用220维原生特权状态/高度与27维控制器已知状态；不能将两组独立验证说成已联测。局部点云不能直接宣称训练187点扫描等价。导航目标、SCAN、速度、区域均来自实际SLAM/IMU/注册点云，Gazebo机器人真值只供离线误差和运动验收。

两路CameraInfo实测D全零、R单位阵、640×480，与各自实际图像逐stamp配对；无漏做去畸变证据。观察相机改为80度和近场视角，图像未拉伸/重投影。原42轮及其generic摘要保留；generic导航套站立15..180s窗口的失败不是有效路线判据，专项结果另存。

## 证据入口

- [导航报告](NAVIGATION_REPORT.md)、[三轮全程曲线](../test_results/navigation_v4_independent_campaign/navigation_three_runs_commands_physical_180s.png)、[真实SLAM路线图](../test_results/navigation_v4_independent_campaign/navigation_three_runs_actual_slam_xy.png)。
- [真实中断专项](../runs/20261004_084733_navigation_slam_scan_ttl_drop_v4_r1_24d9/summary_ttl_dropout_independent.json)及run内ttl_fixture_evidence。
- [相机核验](CAMERA_AUDIT.md)、[实际30维替换](SENSOR_REPLACEMENT_REPORT.md)、[完整坡道失败](FULL_RAMP_REPORT.md)、[Isaac对照](RAMP23_PREFIX_COMPARISON.md)。
- [真实相机回放](../runs/20261004_082258_navigation_slam_scan_roundtrip_v4_r1_9c33/frame_replay.mp4)：原图按传感器时间戳排列，不插帧，不是实况。

## 原子传输和动态障碍的实际复测

V4.1保留原时间戳、flush/close/atomic replace与300ms保护，取消瞬时命令文件逐条fsync。普通180sim秒往返 `dba1` 独立通过，两处原始SLAM区域与停车均通过，6个自有进程退出0。命令文件写入p95从V4的195/274/287ms降到0.744ms；队列最长仍为286.72ms，不能说整个ROS传输始终毫秒内完成。此前性能v1读取错误字段而报告0次expiry，错误版本、勘误与原始源文件均封存。

实际原始顶层 `command_expired` 标志统计依次为2697/2801/3347/1804，即29.96/31.12/37.18/20.04%；标志覆盖源过期或健康门拒绝。按实际consumer的[-.05,.3]s双龄规则独立重算，源年龄不合格分别1222/1468/1638/2，其余年龄合格但不健康1475/1333/1709/1802。因此写入等待明显缓解，而SLAM健康连续性问题仍存在，不能把所有拒绝帧都称为超时。

动态障碍都是实际Gazebo移动箱、真实点云/SLAM/SCAN决定停车与路线，移动箱真值仅供事后核对。协议固定箱体进入、实位保持10sim秒、撤离；5秒停车、原guard连续1秒清空、新SCAN命令、实际恢复，以及两处区域到达均预先规定。

* V4.1 `6614`：240sim秒，首次功能专项通过；障碍停车漂移1.156mm/偏航.06937rad，恢复实际推进.52077m，51.4..52.0s与111.3..111.9s两区域通过。navstack退出-15，其他5个自有进程0，故完整运行失败。旧功能摘要与额外清理失败摘要分别保存，不回写旧结果。
* V4.2 `10c7`：把服务桥ROS命令包装器改成直接Node，保持控制/物理/TTL与原dynamic profile。240sim秒，6个自有进程和15个launch子进程均正常退出，清理修复实际通过。停车漂移2.035mm/偏航.02353rad；去程区域通过，返回在131.1s的90秒截止时失败，剩余原点误差.61349m。独立完整专项失败。
* 返回诊断：V4.2反向对齐持续78s，未见phase反复复位；返回阶段源年龄不合格为0，但1588/4505帧由健康门拒绝（35.25%）。对齐时近零Actor转速共28.56s，实际SLAM累计转角2.98581rad。不能把返回超时重新归因于已修复的HeadingGate复位。
* V4.2清空证据：101个原始cloud过滤后SHA全部一致，60个可与4Hz状态/steering配对、40个缺状态、1个状态过旧；浮点时间戳查询遗漏为0。退出后最长有证据的连续清空只有.6s，未满足1s，保留验收失败；这说明采样证据不足，不能据此说障碍从未清空。V4.3拟逐次保存已有guard实际调用结果与精确integer stamps，不增加几何运算或修改停车判据。

先冻结18秒正向0.2rad/s运动协议后实际运行 `789a`，901策略帧、独立CPU/q_target/来源回放通过，实际5..10s平均转速约.18985rad/s；15..18s停车漂移1.034mm/偏航.02392rad，4个自有进程退出0。这仅验证正向速度校准。V4.3将使用独立turn20 profile缩短180度对齐所需运动时间，原flat及原dynamic的0.12rad/s配置保持，仍须重新实际验证完整路线与停车。

V4.3 `2f70` 随后实际完成240sim秒/12001策略帧，全部6个自有进程和15个启动子进程正常退出，436条逐次guard记录已全部排空、序号完整。127条在被动点云保存窗内有原XYZ可独立重算，全部原双走廊/union、过滤与route SHA一致；另309条在保存窗外缺原XYZ，不据hash单独声称全帧几何重放通过。独立连续clear终于有证据通过，但确认27.69s晚于新SCAN ref26.69s，严格新SCAN失败、严格恢复未验证。固定5秒停车16.345..21.345s漂移.020714m/偏航.04827rad合格，但200Hz最大平面速度.033278m/s超过原.030，所以停车失败。去程在原90s截止失败，0/2区域。

末段rawSLAM确实曾进入.17区域：408个running状态中最小XY误差.158704m，97条controlinside，但原连续dwell最长只有.500s。40..96s一进边界立即zero，之后轻微摆动/估计回弹使计时重启；SCAN仍指向真实checked endpoint约.994m且exhausted=false，Gate一直drive，无预测假到达或路径末端死锁。新候选V4.4将仅在独立Teacher profile等待原rawSLAM连续.6s确认才停车；边界区域、计时、deadline、保护和原checkedSCAN保持，不能事后追认V4.3。原guard实际clear在25.69/26.09s，下一次到26.64s，0.55s缺口超过原0.3s；runtime timer没有重置，并在26.69s cached tick释放。独立helper自leaving24.245s开始，没有等待模型完全撤回，也没有float查找错误。V4.4仅在新profile按真实guard缺口重置timer，且达到1秒的当次scheduled guard才能释放；不增加几何，不用未调用的guard补齐。新候选必须事前冻结再实际测试。

V4.3 `motion_limits.pure_turn_rate_rad_s=.12` 是继承status的共享CHAMP固定元数据；真实Teacher Gate/bridge按新profile .2，worker总界.3。本轮没有进入返回大角度转身，实际最大|wz命令|约.0345，不称已验证本轮.2导航转身。动态clear后累计的 `trigger_window_missed=true` 是Trigger继续observe的统计，只在waiting会阻止入场；实际enter12.425s时为false并满足原4条/0.3s实测触发窗。原status、旧收据未改，解释另存。

V4.4 `1536` 已实际240sim秒/12001帧。原固定5秒停车16.065..21.065s最大平面速度.00378044m/s，偏航漂移.0922888rad，通过原.030m/s/.1rad门槛；真实guard连续清空28.505s由当次实际检测确认并释放，新SCAN后29.63s有真实恢复。两区域由原始SLAM35.6..36.2s、80.4..81.0s各7样本连续.6s确认；最终84.105..87.105s停车漂移.000277m。6个自有及15个启动子进程全部正常退出，394条guard全部排空。修复后的实际运动、导航和清空时机均通过。

该轮候选4完整记录仍failed：enter13.24s后seq29/30的实际guard（13.245/13.305s）消费header13.199999999s的原点云，而passive仅从enter开始保存XYZ，缺了进入前40ms的一帧原XYZ。其余所有严格检查passed；这不是运动失败，但不能把完整源码/原始观测覆盖验收改为passed。V4.5仅拟为只读observer增加有界真实点云pre-roll，控制、profile、物理、模型与所有验收阈值均保持，旧数据不回填；再次事前冻结并实际运行补齐证据。

额外长时站立诊断另存：1536在原84.105..87.105s正式3秒停车通过之后，87.105..240.005s始终Actor输入0，但实际平面速度峰值.0668095m/s、yaw率.164595rad/s、相对起点最多漂移.0549099m，无fault或机身接触。这不是事前新增门槛，也没有改写正式停车通过；它明确限制“全240秒稳定无漂移”的说法。冻结Teacher仍存在长时零速自然脉冲。

最终实测与包清单见下方；校准、离线检查和有限动态通过均不替代完整场景Sim2Sim验收。


## V4.5a最终实际闭环结果

同一V4.4控制/profile/物理/模型和原判据，在执行前冻结V4.5a只读observer预录与异常收尾后，实际运行 `20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f`。240秒/12001帧，冻结candidate5a独立验收19项全部passed、errors=0、unverified=0；6个自有进程和全部15个启动子进程清洁退出，无Actor故障或机身接触。原generic运动诊断在导航全程套固定运动/站立窗而失败，原字节保留；本轮路线结论来自事前冻结的专项，不覆盖generic或历史42轮验收。

| 事前判据 | 实测 |
|---|---|
| 实体移动箱进入/保持/撤回 | 真实Pose_V记录保持阻挡10秒；模型真值仅离线核验，不作为触发或导航输入 |
| 阻挡期固定5秒连续Teacher停车 | 21.16..26.16秒；XY漂移3.758mm、偏航0.014919rad、最大平面速度0.002245m/s、最大yaw率0.004040rad/s |
| 实际点云连续清空与当次guard释放 | 31.725..32.775秒连续1.05秒；32.775秒当次真实guard确认并解除保持，无cached tick提前释放 |
| 清空后新SCAN及实际复走 | 新轨迹5/6/7；32.89秒恢复，实际物理推进0.666881米 |
| 原始SLAM两区域连续到达 | 50.6..51.2及90.9..91.5秒各7样本连续0.6秒；原半径0.17米/最大样本间隔0.2秒/单段90秒截止保持 |
| 最终3秒实际停车 | 94.69..97.69秒；XY漂移0.846mm、偏航0.013697rad |
| 原始观测覆盖和完整退出 | 7份原点云预录；障碍全过程guard原XYZ缺失0；519guard序号完整排空、6自有/15子进程正常退出 |

519条实际guard中110条在原XYZ记录窗内独立重放，两走廊/union、实际self-filter位姿、整数原stamp、source/route哈希均一致；其余409条在记录窗外，几何重放未验证。点云预录保留真实ROS payload的stamp/frame/rawSHA/receive-wall，flush时间另记，没有刷新源龄、仿真几何补数据或事后回填。预录/写入错误会记失败并清理后继续抛错。V4.4缺进入前一帧XYZ的原失败收据完全保留。

当前NAV Actor来源仍为220维特权原生状态/高度及27维控制器已知值。另测的真实IMU/q/qd30维前进三轮与本次导航没有合并；当前结论是实际传感器SLAM/SCAN驱动特权Actor的有限闭环，不是已完成无特权感知部署。完整12米坡道失败，多楼层动态路线与真机未验证；既有长时零速漂移诊断亦保留，不能将正式3/5秒停车通过扩展到任意长时静止。

## 最终证据、哈希和操作入口

- [最终独立动态收据](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/summary_dynamic_obstacle_independent.json)，SHA256 `c1e26edf0bd71b284ed48012cda6faebba2615743d103609493b3387f02f9865`。
- [执行前34源码冻结](../test_results/navigation_v45a_observer_cleanup_freeze.json)，SHA256 `7d54a8ee0fd0a1763e0ed15c57b1a67ee33d3d5658f0b65b02ac58b07336bae9`；独立验收器及其冻结见 `test_results/dynamic_independent_candidate5_v45a_20261004/`。
- [相机来源审计](../test_results/camera_records_dynamic_v45a_20261004.json)：overview481内参/90存档图，vehicle2401内参/469存档图，两路640×480、D全零、80°，所有图像integer stamp匹配。最终[实际相机回放](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/frame_replay.mp4)由90张原JPEG组成，保持实际间隔，不插帧。
- [README操作命令](../README.md)、[当前范围](../current_status.json)、[完整包清单](../PACKAGE_MANIFEST.json)。包清单逐文件保存模型/归档、当前及实际逐轮源码、配置、原始日志、失败、报告与图像哈希；旧包清单原字节另存history，原全局acceptance不改。
- [浏览器最终实测画面与SLAM/SCAN路线](http://127.0.0.1:8768/?run=20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f)，存档画面与Gazebo诊断轨迹明确标记，不冒充导航真值输入或当前实况。


最终dc6f额外长时零速诊断单列于run的 `post_arrival_long_standing_diagnostic.json`：97.69..240.005秒Actor与requested全部零，最大平面速度0.000442728m/s、yaw率0.003962rad/s、XY偏离0.00243422m、偏航偏离0.0437901rad，机身接触/fault为0。本轮后段稳定，1536的零速脉冲记录仍保留；这不是事后新增判据，不改变两轮冻结专项结论。

[最终只读保护审计](../test_results/final_v45a_protection_audit_20261004/REPORT.md)共19项保护/进程检查及5项被动recorder元数据检查通过。34个当前冻结输入哈希匹配，实际归档31份源码逐字节匹配；另2个编译二进制与历史acceptance未复制进run源码目录，其当前值与冻结/保护基准相同，包清单另收两个实际二进制，不将它们称为run归档副本。checkpoint、训练env/agent、原camera_mode/原Demo、native执行器和原acceptance全部保持。最终GPU956MiB/21%、CPU load0.91/1.85/1.93只是11:02:03的只读快照；未向其他训练进程发送信号或修改训练文件。CPU Actor仍单线程、唯一关节执行器，300ms保护保持。

浏览器两路最终证据与来源哈希见 [browser proof manifest](../test_results/browser_proof_20261004/dynamic_v45a_final_browser_manifest.json)、[场外画面和路线](../test_results/browser_proof_20261004/dynamic_v45a_final_overview.png)、[车载画面和路线](../test_results/browser_proof_20261004/dynamic_v45a_final_vehicle.png)。屏幕显示独立19/19通过和已存档/非实时标记，保护历史失败与当前有限结论分开显示。
