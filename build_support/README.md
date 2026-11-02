# 源码依赖构建

Dockerfile从官方ROS/Ubuntu/Gazebo源安装系统依赖，build.py从官方Fast-DDS/Asio/Livox源URL获取源包并编译到install/私有前缀；保留DDS生产补丁和回归测试。不会写系统ROS库，不读取旧工程路径，不进行内容哈希校验或版本运行合同。

该路径编译SDK/FAST-LIVO2/SCAN/关节插件/通信/PCT CLI及原生SE3图求解器。build_building.sh则可复用已有镜像安装前缀并重编本工程修改模块、生成PCT路线。源码构建不等于完整ROS/Gazebo导航验收；当前已验收轮使用本机既有依赖镜像。
