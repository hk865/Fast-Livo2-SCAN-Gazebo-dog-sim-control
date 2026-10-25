# 本轮原机实际构建记录

本页记录整理目录本轮的真实输入和输出，不是Docker/异机安装通过声明。native571的48源重新构建，SCAN plan_manage重新构建；共享SDK、固定规划库/消息支持与actuator仍按SHA复用。具体来源、编译器和编译argv见 [native回执](NATIVE_BUILD_RECEIPT.json)、[SCAN回执](SCAN_BUILD_RECEIPT.json)、[SCAN参数](SCAN_BUILD_ARGV.json)。回执含原机路径以对应实际加载，不代表这些路径在其他机器存在；输出ELF和模型未上传。

native实际命令（先显式准备ROS基础与vikit/Livox依赖搜索路径、线程数1，详见复现说明）：

```bash
colcon --log-base .local_log build --base-paths src --packages-select fast_livo2_core fast_livo2_ros --executor sequential --build-base .local_build --install-base .local_install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

新ELF对应本轮实际91项加载通过；业务停车仍失败，构建成功不等于导航通过。模型、构建日志环境变量全文、凭据和二进制不随本页上传。
