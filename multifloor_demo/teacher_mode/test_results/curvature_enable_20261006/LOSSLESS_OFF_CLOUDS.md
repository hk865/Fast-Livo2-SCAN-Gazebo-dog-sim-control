# OFF97a7 云文件无损归档

只针对已正常关闭的 `20261006_152930_closed_loop_cascade_clock_hold_curvature_off_AB_r1_97a7/navigation_cloud_arrays`，不改变运行源码、配置、原 manifest 或其他实验。必须先由负责人完成 OFF 独立审计；`--off-audit-complete` 是操作者对此的确认，工具不冒充导航验收器。

目录包含 5,527 个 `.npy`（3,865,399,664 bytes）和 5,527 个 `.bin`（7,729,384,464 bytes），总计 **11,594,784,128 bytes，约 10.80 GiB**。两类全部保存原字节，不假设 `.bin` 可由 `.npy` 重建。

目的地固定默认 `/var/tmp/go2_teacher_curvature_20261006_archives`，由工具按当前用户创建为 `0700`。它与 `/home` 所在设备不同；归档后删除原件，可使 `/home` 逻辑数据减少 10.80 GiB，同时 `/var/tmp` 增加实际归档大小。全机净减少为 **gross originals − archive**。同盘归档净腾空间也只能按此差值报告。实际压缩大小、allocated bytes 和各磁盘变化由 MANIFEST/删除收据记录；现在没有测量真实数据的压缩比。

在项目根目录运行，先替换最终 OFF 审计收据路径：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/lossless_off_clouds.py archive \
  --off-audit-complete --audit-receipt /absolute/path/to/final_OFF_independent_audit.json

python3 -B multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/lossless_off_clouds.py prune \
  --off-audit-complete
```

`archive` 使用 `/usr/bin/zstd -1 -T1`，逐源文件读取 SHA256、大小和 inode/device/mtime/ctime；tar 流关闭后，**实际完整解压**，逐文件核 SHA 和字节数。源身份和运行元数据必须仍不变，才写外置 `MANIFEST.json`。本命令不删任何原件。

`prune` 再核归档 SHA、全部源身份、关闭运行/保存进程身份、审计收据 SHA；再完整解压校验一次。随后逐个读原文件确认 SHA，并在最终身份检查后仅 `unlink` 那一个文件，逐项 fsync `PRUNE_JOURNAL.jsonl`，完成才写 `PRUNE_RECEIPT.json`。任何异常立即停止；完整已验证归档继续保留。若崩溃恰发生在 unlink 与 journal 之间，重新 prune 会拒绝未记录的缺失文件，先按恢复命令恢复即可。不会自动清理或覆盖异常遗留文件。

恢复原路径与原字节：

```bash
python3 -B multifloor_demo/teacher_mode/test_results/curvature_enable_20261006/lossless_off_clouds.py restore
```

恢复先核归档，再流式读取成员；文件只在 SHA/大小匹配后从临时文件改名。已有文件仅在自身 SHA 和大小完全一致时保留，不覆盖不同内容；可补齐部分删除。保留原 mtime，但新 inode 是恢复生成的，不能宣称原 inode 保持。恢复需要 `/home` 至少原件总大小的空闲空间。恢复不会改变原 runtime/config/source manifest。

工具拒绝符号链接、非普通/多硬链接原件、陌生文件名、活着的保存 owner、非正常 drain、变化的源/归档、私有目录以外的锁和重复归档。独占锁覆盖同一目的地的 archive/prune/restore。目的地空间按未压缩总量加 256 MiB 保守检查，不降低既有每 run 的 150 GiB 门。

8 项临时小样测试通过，覆盖无损往返、单独删除、符号链接、原件换 inode、损坏压缩包、异常关闭、活 owner、部分恢复/拒绝覆盖和未完成审计确认。初版临时测试发现原件换 inode 时可能先删较早的另一个样例文件；已在删除前加入全目录身份预查，再跑全部通过。此失败仅发生在自动清理的临时测试目录。**截至工具交付，没有读取真实云内容、压缩或删除真实 OFF 文件。**

```bash
python3 -B -m unittest discover \
  -s multifloor_demo/teacher_mode/test_results/curvature_enable_20261006 \
  -p 'lossless_off_clouds_tests.py' -v
```
