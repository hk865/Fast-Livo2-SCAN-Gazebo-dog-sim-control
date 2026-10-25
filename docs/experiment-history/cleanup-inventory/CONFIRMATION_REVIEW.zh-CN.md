> 本页为公开只读脱敏投影，不是删除执行清单；原件及精确身份保留本地。全部删除授权仍为 false。

# 2026-10-25 数据清理盘点与待确认清单

此次仅盘点、提炼与待确认清单；没有删除、移动、压缩替换或上传raw。报告上传不是raw备份。旧清理授权不适用于本批。

原formal22 census allocated为120,275,996,672B；本次14根metadata扫描普通文件分配量120,265,240,576B（不计目录块，时点非原子，不能把差额当已释放）。135个主run，另522个legacy run区目录项（含离线prepare/freeze，不能全称实际仿真）；legacy有460项选定结果收据，UTC没有Z的标签不冒充已校时。

主run根约78.099GB；teacher_mode整体约28.540GB，其中legacy runs约22.009GB；旧Documents约8.036GB；development约5.455GB。代码/native构建/模型/配置/证书/结果/分析均保留。formal22整轮1.935GB全部保护，17/18b/16b/21及关键质量/启动/几何轮暂整轮保护，参见manifest保护列表。

## 首批5个精确候选（逻辑1,143,707,705B；分配上界1,143,734,272B）

所有候选nlink=1，顺序nice19/ionice3读取SHA且前后dev/inode/size/mtime/nlink稳定。实际释放0；未来真实释放仍受openFD、reflink/快照等影响，不能保证上界全部立即释放。

### C001: LOWER_RISK_PROPOSAL_ONLY_AWAIT_EXPLICIT_CONFIRMATION_AND_PUBLICATION_VERIFICATION

精确路径：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261017T204349Z_closed_loop_R85_R74_night_round14_9ae5/r85_reference_attempts.jsonl.gz.partial`

SHA256：`e160a40ecf7352fe1349df0dcfeafde6e1fcc780c5b2010835d51694a75f902a`。逻辑536,542,129B，allocated 536,547,328B；dev/inode=66313/543748；mtime_ns=1792273842242772242；nlink=1。

不可恢复损失：The exact failed compressed partial bytes and aborted-writer intermediate artifact would be permanently lost. Original3,357,631,202-byte logical source remains recoverable from preserved complete manual2gzip under existing decompression proof, whose exact compressedSHA is freshly revalidated. This partial is not claimed byte-identical to fullgzip; original failure713B receipt remains.

逐文件retain映射、原退出/commit和精确引用在EXACT_CANDIDATE_MANIFEST.json。本候选不是执行许可。

### C002: HOLD_REQUIRES_EXPLICIT_ACCEPTANCE_OF_IRREVERSIBLE_REPLAY_LOSS

精确路径：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T173337Z_closed_loop_R85_compact_formal_02_round27_d66d/fastlivo_diagnostics/records.bin`

SHA256：`7c3a716c51649947fcd72500f70e5142d5338aa5c647bd8da83e64fb303f7e55`。逻辑150,662,016B，allocated 150,667,264B；dev/inode=66313/2246376；mtime_ns=1792863508103118838；nlink=1。

不可恢复损失：All binary diagnostic packets in this stream and any unique source/solver/timing excerpts would be unrecoverable. Preserved index and writer statistics are NOT a substitute for packet payload or a rawbackup. Future old-run packet/native timing replay may fail even while published result/certificates remain readable. No complete alternate copy was proven.

逐文件retain映射、原退出/commit和精确引用在EXACT_CANDIDATE_MANIFEST.json。本候选不是执行许可。

### C003: HOLD_REQUIRES_EXPLICIT_ACCEPTANCE_OF_IRREVERSIBLE_REPLAY_LOSS

精确路径：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T175959Z_closed_loop_R85_compact_formal_03_round27_3c67/fastlivo_diagnostics/records.bin`

SHA256：`d5f327044ab60b3f8b25924a1ed90367afa3b720b4fd3a02ce972e666110a3e4`。逻辑154,081,912B，allocated 154,087,424B；dev/inode=66313/2249444；mtime_ns=1792865106263761810；nlink=1。

不可恢复损失：All binary diagnostic packets in this stream and any unique source/solver/timing excerpts would be unrecoverable. Preserved index and writer statistics are NOT a substitute for packet payload or a rawbackup. Future old-run packet/native timing replay may fail even while published result/certificates remain readable. No complete alternate copy was proven.

逐文件retain映射、原退出/commit和精确引用在EXACT_CANDIDATE_MANIFEST.json。本候选不是执行许可。

### C004: HOLD_REQUIRES_EXPLICIT_ACCEPTANCE_OF_IRREVERSIBLE_REPLAY_LOSS

精确路径：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T181929Z_closed_loop_R85_compact_formal_04_round27_58b4/fastlivo_diagnostics/records.bin`

SHA256：`3e6e58dd755bc97e891e9a0eda12d131f321b7cff27121588b5c1f132c1b62ab`。逻辑151,435,056B，allocated 151,441,408B；dev/inode=66313/2251742；mtime_ns=1792866278432060757；nlink=1。

不可恢复损失：All binary diagnostic packets in this stream and any unique source/solver/timing excerpts would be unrecoverable. Preserved index and writer statistics are NOT a substitute for packet payload or a rawbackup. Future old-run packet/native timing replay may fail even while published result/certificates remain readable. No complete alternate copy was proven.

逐文件retain映射、原退出/commit和精确引用在EXACT_CANDIDATE_MANIFEST.json。本候选不是执行许可。

### C005: HOLD_REQUIRES_EXPLICIT_ACCEPTANCE_OF_IRREVERSIBLE_REPLAY_LOSS

精确路径：`/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261024T184628Z_closed_loop_R85_compact_formal_05_round27_ceea/fastlivo_diagnostics/records.bin`

SHA256：`5e6e20c527cc577203e79625f6da72a2c4001402fd90c8669e143f52eab907d1`。逻辑150,986,592B，allocated 150,990,848B；dev/inode=66313/2254671；mtime_ns=1792867882954838910；nlink=1。

不可恢复损失：All binary diagnostic packets in this stream and any unique source/solver/timing excerpts would be unrecoverable. Preserved index and writer statistics are NOT a substitute for packet payload or a rawbackup. Future old-run packet/native timing replay may fail even while published result/certificates remain readable. No complete alternate copy was proven.

逐文件retain映射、原退出/commit和精确引用在EXACT_CANDIDATE_MANIFEST.json。本候选不是执行许可。

## 依赖覆盖与尚未完成部分

独立小文本扫描2582文件/106,498,138字节，另选定runtime/source/closure收据；137大文件等跳过，未扫描所有JSONL/binary/动态构造路径和外部消费者。唯一精确路径命中是C001失败归档收据，保留该收据；其它run-label或basename命中不当作无依赖。未命中≠安全。4个records.bin没有证明完整替代副本，保留index/stat/results不能恢复packet。

仍有284个≥64MiB文件仅metadata、未计算SHA，不是下一批删除范围。全部legacy目录暂保留，旧run实际commit缺失处明确UNKNOWN，不从当前checkout回填。

## 给父对话的人类确认草案（尚未请求或获得执行许可）

请先验证相关日志报告/精确manifest已上传成功，再只对这份manifest的精确candidate IDs与SHA请求一次不可恢复删除确认。建议先单独考虑C001（保留已核完整manual2载体）；C002–C005必须另明确接受丢失原二进制诊断/逐帧时序回放。也可以选择全部保留。当前没有任何文件获永久删除授权。

确认范围必须逐项ID/绝对路径/当前manifest SHA，不包含通配符或目录，不自动扩展到284个未哈希大文件或formal22/保护轮。执行前须重新检查原身份、三锁/无owner/进程与文件stat和SHA；任一变化即停止并重新核范围。不要靠“旧/FAIL/文本无命中”代替这一步。

本目录不包含可运行删除脚本，没有新增框架/大备份。
