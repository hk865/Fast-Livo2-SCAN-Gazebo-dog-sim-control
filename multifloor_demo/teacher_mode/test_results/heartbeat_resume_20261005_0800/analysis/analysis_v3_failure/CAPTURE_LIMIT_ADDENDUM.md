# V3弱捕获限幅与低速响应补充（只读诊断）

两轮native首次goal_dwell/capture都为37.285s。至此的全部200Hz有效物理状态逐样本比对，未把新computed command向前绑定此前cached native。详细误差和限幅分解见capture_limit_stasis_supplement.json。

新capture新鲜请求平面范数达到.06包络比例 93.528%，vx≥.059m/s比例 93.780%；严格vx==.06仅26.702%，原因包括vy需求触发平面范数缩放，不能把它当作全部饱和占比。原fresh vx轴反饱和93.696%，raw vx median .0611902、I贡献median .0429453m/s。

最后160.005..180.005s实际Teacher after-slew body命令median [0.0599999999035608, -3.4018676635003022e-06, -0.02565879669157391]；实际body origin vx均值3.306799e-8m/s，目标距离约.066460m。全段Actor仍推理，已验证真实命令非零而平移响应趋近零；这说明本配置弱捕获无法继续逼近25mm入界，不证明所有低速/坡道/扰动下Teacher必有相同死区。没有GPU唯一因果或改参数已经解决的结论。

新正式active_hold未声明，首5s停车未验证；旧0b78原停车FAILED、新common completion/parking UNVERIFIED、三次确认未运行均保持。报告中capture+[.6,5.6]只作原先登记的早期诊断，绝不替代正式停车。
