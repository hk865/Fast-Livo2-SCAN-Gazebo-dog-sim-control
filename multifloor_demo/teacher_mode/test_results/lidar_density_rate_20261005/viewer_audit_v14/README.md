# V14固定Viewer候选扩展

在Teacher scripts/serve.py 的固定756ad publication-ledger候选tuple增加 `navigation/lidar_sampling_v14/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json`，原V11/V12与V9/V10 v1不变。没有改数学、时序、导航门、外部存储边界、原camera_mode或原收据，未重启服务或启动仿真。

新增三个合成显示完整性边界：允许固定V14且七个预冻结源同字节；拒绝同字节但未知V13候选；拒绝声称通过但producer源已变的V14。三项通过。原v1 16、v2 26、private-storage10项回归及Python内存编译均通过。V14真实七源与criterion756ad字节绑定已核对，真实外部V12 0b8d仍显示有效FAILED23/1/2，不把25/32前缀升为全程通过。

保持唯一 `/var/tmp/go2_teacher_simulation_20261005` 私有0700、owner/current UID、非symlink root以及严格同basename alias的原边界。V14实际run在产生后由root浏览器核对；此fixture不认证绑核性能或导航。

新证据仅保存在viewer_audit_v14，未写旧viewer_audit_v11或thread_scaling manifest。serve.diff只有固定候选路径增量，以及由固定V14 criterion引用与cpu_affinity_enabled共同识别的显示标签。它不改变选取状态：实际联合收据待完成时明确“V14 固定绑核实验；实际验收尚未完成”，保持unverified，不回退到共同PASS。

实际外部V14 6eb0已用小scope/receipt只读核对：canonical alias正确，V14实验标签出现；本次观察为未验收状态，不宣称物理/路线通过。
