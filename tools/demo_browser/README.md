# R85 单轮只读浏览器视图

入口 `serve_r85.py --run /absolute/path/to/one/run --port 8769`，仅监听 `127.0.0.1`。
必须显式指定一轮目录；不会扫描运行池、订阅 ROS、启动或停止仿真。
根页面来自现有 `multifloor_demo/web/index.html` 的局部适配副本，原文件未修改。

在仓库根目录运行：`python3 -B tools/demo_browser/serve_r85.py --run /absolute/path/to/one/run --port 8769`。
浏览器每 0.8 秒读取一次显示状态；这不是传感器采样率或验收时钟。
vehicle RGB 来自本轮实际 30Hz 相机归档的最后一帧，overview 独立归档为 2Hz；
HTTP 只读取已存在帧，不额外采集。浏览器窗口实际屏录采用外部录制器的 5Hz 墙钟，
与 UI 刷新和两路源图像频率分别记账；不能将屏录帧数当实际传感器样本数，
也不能声称录制启动前的任务过程已被覆盖。本适配器不启动编码器或录屏。

仅开放根页面及 GET `/api/state`、`/api/map/map_metadata.json`、
`/api/map/colored_map.NNNNNN.bin`、`/api/frame`、`/api/acceptance`。
POST/PUT/PATCH/DELETE 均返回 405。帧优先本轮 vehicle RGB，否则本轮 overview；
结束后的画面标为归档帧。原始 metadata 的 counts/ages 保留原值，不把归档值当当前鲜度。

位姿与 yaw 只来自本轮 SLAM journal，不读 native 真值。轨迹缓存最近 20000 条、
最多显示 2000 点；仅显示降采样，不参与验收。路径仅绘制导航反馈接纳 ID 对应的
实际 controller 路径 journal，且核验浮点点数组 SHA；未知路径留空。
验收直接返回原 `r85_task_result.json`，未知项不补零、不生成 PASS。

纯文件检查用于已结束且文件稳定、具有 SLAM/点云/图像的单轮归档，不要求该轮通过或失败。
它不创建 socket、ROS Node 或仿真，验收分支直接随原结果判断：
`PYTHONDONTWRITEBYTECODE=1 python3 -B tools/demo_browser/check_readonly.py --run /absolute/path/to/one/run`。

该适配器只是显示，不声明浏览器负载、实际运行或完整 demo 已通过。
代码复用本仓库已有页面与本次只读适配，不引入第三方 JS/CDN 或新的第三方许可；
既有上游代码和资产沿用仓库的来源与许可说明。
