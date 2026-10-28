# 从公共源码构建

`build.py`是唯一全量构建入口，默认两线程。它下载固定官方Fast DDS/Asio/Livox SDK2源，应用现有双端heartbeat补丁与构建兼容修正，编译SDK、FAST-LIVO2、SCAN、actuator和writer helper，输出本树`build/`、`install/`。不会读取原机器`.local`或旧run。

```bash
source /opt/ros/jazzy/setup.bash
python3 -B build_support/build.py --jobs 2
```

公共开发依赖由根Dockerfile安装，具体见[移植说明](../docs/移植构建运行.md)。`--skip-dds`只用于继续本副本已完成的DDS构建，不是复用系统库。DDS fresh工作目录保护、下载来源/hash和环境清单是构建记录，不是运行批准门禁。

Fast DDS的5项SHM组件测试会实际执行；没有嵌套Docker网络测试或Docker socket依赖。正常启动不调用本构建工具。
