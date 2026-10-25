# 必要vertical-only增量

最终候选 `0f35662822925a68de9a659558cc5073845b5b87`（父20f31fb），单源只补exact `XY全部等于首控制点 && Z极差>0` 时禁止terrain override。与原公共whole-span拒绝使用相同原布尔条件，无epsilon/阈值变化；原公共规则原样。

1个直接actual _curve_patch合成反例PASS（0.017s）：同一平面、全footprint及原残差/净空均通过的constantXY/nonconstantZ控制点，旧20f31 override会放行；最终源码拒绝，terrain_grade_evaluation=None。此为helper层数学反例，非真实传感器/全公共certifier重放。

旧20f31的FREEZE/CHECKS/REPORT及25检查原样保留，未重复suite；本次只补此一例。最终证据组合为旧25项与该必要增量，不声称最终全套复跑。原新地形语义与旧body垂向导数限制不等价的披露继续有效；静态证书不是动力学/控制/停车/导航PASS。无部署/ROS/仿真。
