# 实际平地动态障碍独立验证

最新dc6f在预冻结的限定平地动态场景中 **严格验收passed，19/19项通过**：实际箱体进入阻挡、Teacher停车、原始点云连续清障、同实际guard callback释放、新SCAN与真实运动恢复、两个原始SLAM区域及最终停车、完整进程退出均实证通过。该单次有限路线结果不扩大为全局导航或Sim2Sim；完整坡道仍failed，多楼层与真机仍未验证。此前四次动态失败及1536功能完成但原XYZ覆盖不足的failed总项全部保留。

## 事前范围与判据

本项使用独立`finite_flat_dynamic_stop_resume_v1` scope，保留三份实际V4平地闭环basis与全局excluded场景。唯一现有静态箱`moving_obstacle`从(7.15,-1.6,0.6)以预定0.5m/s服务步进进入(7.15,-0.7,0.6)，实际Pose_V确认阻挡10模拟秒再撤回；服务ack不能代替实际模型位姿。机器人仅由native Teacher执行器控制，Actor在CPU上运行；箱体位置/机器人真值只用于独立物理诊断与后验几何对应，从未进入SLAM、SCAN、速度命令或区域判断。

实际SLAM连续4位姿/0.3模拟秒及accepted前进命令触发箱体；NAV根据真实注册点云guard保护停车。原事前要求：先连续1秒真实低速，再固定5秒停车，最大平面速度0.03m/s、wz0.05rad/s、XY漂移0.03m、yaw漂移0.1rad；持续CPU Teacher零速度输入，不用action=0、关节capture或机身位置伺服。实际registered cloud/self-filtered buffer与原steering corridor连续clear至少1模拟秒、最大记录间隙0.3秒，随后须有新实际SCAN及真实推进≥0.2米。NAV可以在箱体仍处leaving时依真实点云恢复；箱体完全回initial是独立fixture最终条件，不是导航输入。两个原始SLAM区域继续使用0.17米control band、0.6秒dwell、每目标90秒期限。

原独立候选0c7608在6614之前冻结。V4.2候选2a574d在10c7之前冻结，物理/clear/90秒判据相同，精确授权新的runtime来源，另外要求六个owned角色exit0及全部实际launch child clean退出，没有信号豁免或延长收尾宽限。原common分析只gate worker/Gazebo正常退出，因此6614的原functional摘要passed不能解释为完整进程收尾passed。

## 两次实际结果

| 实测 | 6614 / V4.1 | 10c7 / V4.2 |
|---|---|---|
| 策略运行 | 240秒/12001帧，CPU Teacher | 240秒/12001帧，CPU Teacher |
| 原生安全 | body/未知contact0，姿态/clearance通过 | body/未知contact0，姿态/clearance通过 |
| 箱体实际阻挡 | 17.265..27.265秒，10秒 | 14.645..24.645秒，10秒 |
| 固定5秒实际停车 | 18.265..23.265秒 | 15.645..20.645秒 |
| 停车最大XY漂移 | 0.001156米 | 0.002035米 |
| 停车最大yaw漂移 | 0.069371rad | 0.023525rad |
| 停车最大planar speed / wz | 0.002670m/s / 0.024567rad/s | 0.000668m/s / 0.006346rad/s |
| 独立连续点云clear | 29.6秒确认，passed | 最长仅0.600000001秒可完整配对，冻结验收failed |
| 新SCAN后的实际恢复 | world34.35秒后accepted，最大XY推进0.520774米，passed | runtime真实resume1与重新行走存在；严格新SCAN/恢复项因clear缺证据unverified |
| outward原始SLAM dwell | 51.399999999..52.0秒，7样本 | 40.2..40.9秒，已到达 |
| return原始SLAM dwell | 111.2..111.9秒，8样本，0.7秒 | world131.1秒原90秒目标超时，未到达；raw SLAM仍距return center XY0.612878米 |
| 到达后Teacher停车 | 114.98..117.98秒漂移0.000204米 | return未到达，最终到达停车未验证 |
| owned/child退出 | 5/6 owned0，navstack=-15；14/15 child clean，唯一ros2-12缺clean行 | 6/6 owned0、15/15实际child clean |
| 完整动态运行 | **failed：收尾失败**；功能原收据passed | **failed：clear证据不足、return超时**；收尾passed |

6614的真实注册点云native guard事件被独立重放为blocked，原result与重算完全一致，527个走廊点，后验实际箱体表面对应1462点；不是仅凭box服务回执停止。两路实际RGB、decoded XYZ及manifest/source hashes都保存。没有量测/解码每脚竖直承重；接触信息与施加到仿真物理的力矩也不当成实体传感器测量。

6614：[原冻结专项收据](../runs/20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614/summary_dynamic_obstacle_independent.json)、[完整收尾失败补充](../runs/20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614/summary_dynamic_cleanup_independent.json)、[240秒及停车恢复局部曲线](../runs/20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614/dynamic_obstacle_actual_stop_restore.png)、[原SLAM路线与实际区域](../runs/20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614/dynamic_obstacle_actual_slam_xy.png)。原generic SHA `6329e566…`、原functional SHA `7f3743ab…`及候选代码都封存，补充没有覆盖它们。

10c7：[候选2独立失败收据](../runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7/summary_dynamic_obstacle_independent.json)、[真实完整曲线](../runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7/dynamic_obstacle_actual_stop_restore.png)、[原SLAM路线](../runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7/dynamic_obstacle_actual_slam_xy.png)。V4.2唯一runtime修改是服务桥由ros2run包装改为直接launch Node；模型、physics、Teacher和物理停车判据没有变化。

## 返回慢转身与证据缺口

[只读转身对照/输入哈希](../test_results/dynamic_return_turnaround_20261004/turnaround_diagnosis.json)与[实际heading/命令/速度/健康保护图](../test_results/dynamic_return_turnaround_20261004/actual_dynamic_return_turnaround_comparison.png)保留所有原始数组。6614 return52.085..111.98秒，对齐45.16秒，first recorded drive world98.38秒，实际actor drive11.44秒；10c7 return41.0..131.1秒，对齐78秒，直到121.8秒才有recorded drive vx>0.05，实际drive5.74秒。

10c7 align期间实际actor wz积分4.45357rad、机身native quaternion heading变化2.98581rad；近零actor wz28.56秒、|actor wz|>0.08仅30.72秒。6614相应积分3.47093rad/heading变化2.92113rad、近零10.16秒。10c7 return1588/4505（35.25%）帧被原healthy门拒绝，6614为580/2995（19.37%）。两段原source sim/wall age均没有超出worker[-0.05,0.30]门；全部拒绝帧匹配原producer `healthy=False/state=hold`与实际sensor warmup/health reason，不能称文件TTL过龄。断续保护与Teacher实际转身响应共同消耗原目标期限，不将仅提高yaw命令预设为成功。

[10c7 clear覆盖诊断](../runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7/dynamic_clear_evidence_coverage_diagnostic.json)指出101个实际registered cloud buffer都与消费hash精确相同，其中60个可配原steering/control pose，40个没有同cloud stamp的4Hz status快照，1个快照超出原0.3秒因果窗；由float重建control ns查找失败为0。撤离后34帧仅19可完全配对，最长29.899999999..30.5秒连续clear约0.6秒。所有可配clear样本都显示clear，但不足以填补未记录区间。冻结failed保留，含义是独立验收缺证据，不能说真实走廊始终有障碍；也不按模型真值补齐。

789a随后单独实际positive yaw0.2校准完成18秒/901帧：[独立来源收据](../runs/20261004_094018_turn_positive_yaw02_navigation_calibration_r1_789a/summary_yaw02_calibration_independent.json)核对131归档源、CPU frozen checkpoint、247/12有限输出、目标关联与4/4 owned exit0。5..10秒actor输入均0.2rad/s，实际wz平均约0.18985rad/s，原停车与安全判据passed。这只证明单次正向0.2校准；新动态导航、负向0.2或所有丢失健康门情形仍需各自实际证据。

V4.3已经加入每次实际guard的integer pose/cloud时戳、原filtered hash/geometry inputs/result，并采用实际正向0.2rad/s校准对应的新scope。其实际失败如下；原2次失败不改变。

## V4.3实际原始证据与失败

[2f70冻结候选3专项收据](../runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/summary_dynamic_obstacle_independent.json)主结论 **failed**：240模拟秒/12001策略帧，六owned exit0及15个实际launch child clean，真实CPU Teacher持续推理、原生安全、实际箱体阻挡、registered cloud检测停车均通过；两个原始SLAM区域与最终到达停车没有通过，不能称完整动态闭环。

停车采用原first1秒连续低速后的固定16.345..21.345秒，requested与actor速度输入均0，策略仍持续推理。1001条200Hz物理样本中34条在18.040..18.205秒速度超过0.03m/s，18.145秒真实机身COM planar speed峰值 **0.0332781291m/s**。XY漂移0.020714米、yaw漂移0.048270rad、最大wz0.028426rad/s分别在原阈值内，但速度峰值单项仍failed。不能只证明初始1秒低速后豁免固定5秒内的峰值，也不另选安静窗口。[停车峰值与逐帧邻域](../runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/dynamic_failure_diagnostics/parking_peak_diagnostic.json)保存实际joint目标/力矩/接触、Teacher action和零命令邻帧；它不预断策略或适配根因。

真实guard日志436行，writer drained/count一致。127次在fixture录制期内的actual invocation能用原始registered XYZ、精确integer SLAM control/filter pose及原buffer SHA完全重放两次corridor和union；其余309行没有录制期外的原XYZ，明确不凭hash声称全部240秒几何已重放。独立clear允许从实际leaving24.245秒开始，**不是**从模型完全撤回26.145秒开始。seq111..114的25.69..26.09秒clear均实证通过，下一次seq115在26.64秒，真实计算间隔0.55秒大于原0.3秒上限；独立连续计时因此重置，直到27.69秒确认1秒clear。旧runtime保留25.69计时器，cached tick在26.69秒已释放并请求新SCAN；后续没有reference stamp≥27.69秒的新SCAN。因此clear最终证据passed，release/newSCAN时序failed，严格恢复项unverified；真实resume1和再次行走记录仍保留，不否认发生过运动。[真实长间隙、integer trace与原XYZ哈希](../runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/dynamic_failure_diagnostics/guard_gap_release_diagnostic.json)证明此失败不是float ns配对或mover phase截止误差。

原始SLAM最小出程XY误差0.158704米，tracking pose最小0.165649米；97条4Hz running status显示raw pose进入原0.17米control band，这97条controller速度均立即为0，但原stamp最长dwell仅 **0.5秒<0.6秒**，从未进入return stage。首次failed status在world96.195秒观察到原90秒航点超时，该4Hz快照不是精确deadline时钟。40秒至此观测窗内455/2810 worker帧为healthy=False，真实source age超原[-.05,.3]门为0；速度小脉冲、零速边界来回与健康hold持续打断停留。tracking没有伪造到达。[原始raw/tracking距离、dwell、实际Actor/COM速度与健康门图](../runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/dynamic_failure_diagnostics/actual_arrival_boundary_failure.png)及[独立诊断收据](../runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/dynamic_failure_diagnostics/arrival_boundary_diagnostic.json)完整保留。0.2rad/s返回转身在这一run没有发生，不能以候选上限或单独校准称已验证返回。

后续V4.4仅在新`finite_flat_dynamic_stop_resume_dwell_v1`范围内，事前冻结`after_measured_dwell`到达停车，以及真实guard长间隙重置/实际guard callback才能完成原1秒clear。原raw0.17米/0.6秒/最大arrival gap0.2秒/90秒期限、source TTL0.3秒、clear gap0.3秒、固定5秒停车0.03m/s峰值及全部物理标准不变。[候选4冻结清单](../test_results/dynamic_independent_candidate4_v44_20261004/freeze_manifest.json)明确MAIN还必须两区域、最终Teacher停车、六owned与全部child正常退出；实际1536结果见下节。

## V4.4实际功能完成，原XYZ覆盖总项仍失败

[1536冻结候选4专项收据](../runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536/summary_dynamic_obstacle_independent.json)保存 **MAIN failed**，唯一失败项为`original_xyz_covers_actual_fixture_guard_window`。**全部预冻结物理、停车/清障/恢复和两个SLAM区域/最终停车条件都已实际通过**；记录覆盖缺口与功能结果分别列明，没有重写原failed总项。

240模拟秒/12001帧持续CPU Teacher，六owned exit0、15个实际launch child clean，worker无fault；394个实际guard按seq1..394写完，writer drained、queue error0，连续clear timer实际reset记录0且计数一致。模型/关节执行器/物理参数与原冻结Teacher一致，最大原生roll/pitch0.113963rad、最小支撑clearance0.275679米，body/未知contact均0。实际非零actor命令21.66秒、真实原生速度>0.05m/s运动16.655秒，最大实际XY离起点0.860283米；不是站立模型加载测试。

| 原判据实测 | 1536结果 |
|---|---|
| 实际唯一箱阻挡 | world15.065..25.065，10模拟秒 |
| 固定5秒Teacher停车 | 16.065..21.065，1001原生帧/250策略帧 |
| 停车planar/wz峰值 | 0.00378044m/s / 0.0339264rad/s，passed |
| 停车XY/yaw最大漂移 | 0.00143069米 / 0.0922888rad，passed |
| 严格实际点云clear | 27.505..28.505，原integer上下文/XYZ独立重放，连续1.0秒，passed |
| release因果 | seq51实际native guard callback world28.505，同callback held→released；没有cached提前release |
| clear之后新SCAN | ids10/11原实际reference之后，新样条payload/coefficients/knots重建通过 |
| Teacher实际恢复 | world29.63 accepted非零起，真实XY恢复段推进0.528139米，13.82秒原生速度>0.05m/s，passed |
| 原raw SLAM outward区域 | stamp35.6..36.2秒，7样本连续0.6秒，在0.17米control band内 |
| 原raw SLAM return区域 | stamp80.4..81.0秒，7样本连续0.6秒，同原区域/90秒期限，passed |
| 原最终3秒Teacher停车 | world84.105..87.105，漂移0.000277米、yaw0.007292rad，passed |
| 完整记录总项 | **failed：2个进入边界actual guard缺原XYZ** |

缺失的两次actual guard是seq29 compute13.245、seq30 compute13.305；它们消费同一真实cloud stamp13.199999999，即entering13.24前约40毫秒到来的输入。被动recorder从entering阶段才存decoded原XYZ，因此原hash/context存在但raw payload没有保存。全部48个有原XYZ的actual guard原filter/control/双corridor/union重放精确一致，包括实际检测停车和整个清除/release区间；其余346行中344行在fixture录制期外、2行正是进入边界覆盖失败。不能把hash当成原几何，不用Gazebo箱真值补这些输入，也不回填1536历史日志。

[240秒与实际停车/清障恢复曲线](../runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536/dynamic_obstacle_actual_stop_restore.png)以及[原SLAM路线与真实dwell区域](../runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536/dynamic_obstacle_actual_slam_xy.png)已逐图核验，输入/输出hash与绘图原数组保存。原generic SHA`bd505189…`与专项SHA`1dd73ff2…`原bytes保持。图中晚段零命令仍有Teacher自然运动脉冲：[独立长站立诊断](../runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536/post_arrival_long_standing_diagnostic.json)记录87.105..240.005秒Actor输入全0，planar峰值0.0668095m/s、wz峰值0.164595rad/s、最大XY偏离0.0549099米，body contact/fault0。原3秒停车passed不能扩写成240秒全部稳定或所有故障停车通过；该诊断不替换冻结验收窗。

V4.5a仅增加被动原cloud pre-roll缓存，进入时写出过去真实stamp/原XYZ/hash，以及记录器异常收尾保证失败manifest与非正常退出。V4.4profile/controller/guard/physics/runner保持原SHA；[候选5a事前清单](../test_results/dynamic_independent_candidate5_v45a_20261004/freeze_manifest.json)精确绑定最新V45a freeze`7d54a8ee…`，同coverage、固定停车、连续clear、两区域与完整退出标准，另外核原cached header/rawSHA/receivewall/decodedXYZ和记录器错误。独立候选SHA`cc1e2d1e…`，没有长期站立新gate或放宽阈值。dc6f实际结果见下一节。1536证据总项不会追认通过，全局Sim2Sim、三层路线和真机仍不因局部功能完成升级。

## V4.5a最终实际严格验收通过

[dc6f原始独立专项收据](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/summary_dynamic_obstacle_independent.json)采用运行前已冻结的候选5a `cc1e2d1e…`，**MAIN passed，19/19 checks通过、errors0、unverified0**。限定experiment仍为`finite_flat_dynamic_stop_resume_dwell_v1`，V4.4控制/profile/Actor/PD/physics/runner原SHA不变，34个exact V45a来源全部匹配。240模拟秒/12001策略帧，worker faultNone；worker/Gazebo/bridge/capture/shadow/navigation_stack六owned returncode0，15个实际launch child clean。真实导航仅使用SLAM和registered cloud，Gazebo robot/box真值仅在独立物理与后验诊断中使用。原generic站立15..240窗口不适用这段真实非零导航，其failed记录原bytes保留。

| 原冻结条件 | dc6f实证 |
|---|---|
| Teacher实际运行/运动 | 12001帧CPU Actor；33.86秒实际非零actor命令、16.55秒原生速度>0.05m/s运动；最大XY离起点0.846669米 |
| 实际唯一箱fixture | entering18.335、blocking20.16..30.16连续10秒、完全撤回32.06；唯一existing模型，只该box服务 |
| 预定5秒持续Teacher停车 | world21.16..26.16，1001个200Hz原生状态/250个50Hz策略样本；requested与actor速度全0，持续推理 |
| 停车峰值与漂移 | planar0.00224509m/s、wz0.00403959rad/s、XY0.00375773米、yaw0.01491868rad；原.03/.05/.03/.1全部通过 |
| 严格实际clear | 31.725..32.775，原XYZ/精确integer SLAM与滤除姿态、native双corridor/union独立重放连续1.05秒；最大实际guard gap门仍0.3秒 |
| 实际callback release | seq149 world32.775同真实native guard callback held→released；与独立clear确认同刻，无cached提前release |
| 新SCAN/真实恢复 | post-clear ids5/6/7；10条全程实际trajectory payload核验；world32.89新accepted非零输入、恢复段真实XY推进0.666881米，14.075秒真实速度>0.05m/s |
| outward raw SLAM区域 | stamp50.6..51.2，7原始样本，连续0.6秒、最大间隙约0.1秒、全在原control radius0.17m/高度范围内且未保护 |
| return raw SLAM区域 | stamp90.9..91.5，7原始样本、同0.6秒dwell；两目标均在原90秒期限内 |
| 到达后3秒Teacher停车 | world94.69..97.69，601原生帧；漂移0.00084630米、yaw0.01369686rad，线RMS0.0000445/0.0005413m/s、wz RMS0.0045793rad/s |
| 原生安全/独占控制 | 全程200Hz与原source/body/限幅/target/Actor关系check通过，无body/未知contact，只有native Teacher执行器拥有执行权 |
| 完整退出/记录总项 | 六owned0、15实际children clean；19项全部passed |

原被动PointCloud2输入保留 **77条cached header回执、7份真实pre-roll XYZ数组**，原integerstamp/frame/rawSHA/received wall完全相同，原NPZ dtype/shape/decoded bytes SHA及独立filtered float64 buffer均核对；peak cache9帧/1593384bytes，未超过1秒/32帧/16MiB限额，queue/observer/pre-roll/cleanup错误均0。没有回填1536，也没有刷新旧输入时间戳。519条实际native guard日志seq1..519完整drained，其中110次actual guard invocation可由原XYZ精确重放；另有109条registered cloud独立重放记录。**fixture进入至完全撤回+5秒内所需actual guard的原XYZ缺失0**。409条缺原XYZ的guard均在这个录制期外，保留unverified几何覆盖限制，不声称全240秒每个guard都已独立重放。

[全240秒实际command/COM速度/wz/box曲线及停车恢复局部图](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/dynamic_obstacle_actual_stop_restore.png)、[原始SLAM XY、实际注册goal discs和原dwell样本](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/dynamic_obstacle_actual_slam_xy.png)均已肉眼核验。plot数组、输入/输出SHA、执行源随run保存；图中100ms后向均线仅为观察，不参与验收。GPU用于实际Ogre2图形上下文，Actor全程CPU，单次CPU forward p50/p95/max为0.287083/0.322792/0.793997ms；不把这个最大值称所有场景实时保证。

[长零命令站立诊断](../runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f/post_arrival_long_standing_diagnostic.json)另记97.69..240.005秒requested/actor全0，最大XY偏离0.00243422米、yaw0.0437901rad、planar0.00044273m/s、wz0.0039620rad/s，无body contact或native fault。该轮长站立较1536平稳，但仅诊断，**没有新增事后gate，也不能抹去1536的长站立脉冲或全局Teacher漂移限制**。

原generic SHA`0cf84a0320dc30dffc0b6befe38b1d4eb07f6217aa8337ab481d9e4f98d2eab4`、专项SHA`c1e26edf0bd71b284ed48012cda6faebba2615743d103609493b3387f02f9865`保持。结论是这一次**限定平地动态停车/恢复与两区域闭环通过**，目标中心相隔1米而实际离起点0.846669米，不称精确1米机器人平移。三层坡道路线、完整46区域任务、无界动态障碍、全局Sim2Sim及真机仍未因此通过。

## 可重复的只读审计

```bash
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_dynamic_obstacle.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_091513_navigation_slam_scan_dynamic_flat_v41_r1_6614 --read-only
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_dynamic_obstacle_v42.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7 --read-only
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_dynamic_obstacle_v44.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_102555_navigation_slam_scan_dynamic_flat_v44_dwell_r1_1536 --read-only
python3 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/scripts/analyze_dynamic_obstacle_v45a.py \
 /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/runs/20261004_105315_navigation_slam_scan_dynamic_flat_v45a_preroll_r1_dc6f --read-only
```

只读模式不重写原收据、不启动ROS/Gazebo。曲线的额外100ms后向均线仅用于观察，验收全部采用真实原始时戳/200Hz原生数据。目标区域中心相距1米，实走约0.84米再落入区域，不能称精确1米往返。完整坡道、多楼层、原46区域任务、全局Sim2Sim和真机不因本项升级。
