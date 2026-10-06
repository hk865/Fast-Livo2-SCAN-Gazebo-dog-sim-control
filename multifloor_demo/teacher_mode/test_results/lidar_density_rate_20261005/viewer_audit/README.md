# Teacher 只读页面：V9 / V10 时钟保护联合收据

仅修改 `teacher_mode/scripts/serve.py`。没有改磁盘上的web页面、camera_mode、控制器、试验源码或原收据；没有重启server、启动ROS/仿真。浏览器实测由主任务在重启其8768服务后完成，本目录的验证不代表浏览器已核对或导航通过。

当前 `closed_loop_receipt_view()` 优先选择新 `summary_closed_loop_clock_hold_independent.json`，schema为 `independent_actual_SLAM_SCAN_clock_hold_navigation/v1`；并列保留原common、完整坡道、先导诊断和runtime收据。无新收据或JSON尚未完整写完时，V9/V10显示“事前时钟保护联合验收待完成”，不会用不含hold复核的旧common认证新控制逻辑。原无clock contract的历史run保留原选择行为。

新增六项逐条展示：事前判据/存档来源、实际同钟零速/短暂停顿冻结、长停顿/来源过期/复位、非零恢复的新时钟与原生几何、含hold的发布过渡、执行回执与显式hold复位的PI重放。完整区域项显示实测到达数，部分路线不升级为完整多层通过。

显示完整性验证包括：

- 新receipt的run必须是当前目录、导航不能使用真值；claimed PASS的所有check必须同时 `status=passed / passed=true`。
- 判据SHA固定为 `9c2b5f2ff6a8f6525df662411ca51976ee3aa6196661f3b774b8a30bf4259f65`，检查权威文件、执行run内快照及scope引用，同时核control-clock contract和旧evaluator身份。
- 实际核六个小文件的SHA：navigation_scope、source_manifest、navigation_source_snapshots、navigation_profile、runtime_manifest、slam_loaded_binary；再核旧common绑定中的profile/request等。不会在每次页面请求重新哈希大日志或46GB诊断文件，也不会在viewer重跑数学与几何验收。
- 原common收据SHA、criteria、scope、verified inputs必须保持；只有事前声明的一个PI bookkeeping check允许替换。其余check完整payload/status必须相同，六个extra必须齐全，archive绑定必须与本run snapshots及refs一致。
- 原common需要完整执行/source/math/guard/32区域/停车等检查；若完整区域项声称通过，必须32个与实际request顺序一致的到达、各自pass且最终state=succeeded。执行phase原始state出现failed时，即便flags伪造也不能显示绿。
- 任何完整性拒绝的主结果显示unverified和明确理由，原始payload仍可展开查看。真实failed/未验证项目原样保留。

16个纯显示边界fixture通过，包括外国run、缺extra、extra失败、source真假标志、实际source hash变化、criteria/foreign snapshot、旧common hash、altered共同判据、21/32伪完整、phase failed伪绿、真实failed partial与parking unverified、pending/truncated以及历史兼容。fixture的人工PASS只验证显示器，不是运动或导航结果。

实际20Hz样例 `20261005_195524_closed_loop_cascade_clock_hold_lidar64_20hz_rgb20_r1_2a1a` 使用evaluator修正后的v2 canonical alias，SHA为 `dcbe644cb094bf2770786c43ddc550bc11e97d6bb3881580c8347a3944d359df`。页面函数独立核其小文件绑定无误：整体FAILED，25/32区域，停车unverified；共25项中22通过、1失败、2未验证；六个extra全通过。原common仍单独保留。v1 reader错误、v2追加及显示alias纠正证明由evaluator归档，未被本任务编辑。

103个保护源hash前后均相同，serve/web不在当前V9 run的260 refs中。边界记录见 `change_boundaries.json`，最终hash和实际样例见 `validation.json`，源码前后版本与差分已保存在本目录。

复现显示边界测试，不启动server或仿真：

```bash
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_
python3 -B multifloor_demo/teacher_mode/test_results/lidar_density_rate_20261005/viewer_audit/test_viewer_receipts.py
```
