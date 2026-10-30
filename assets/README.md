# 场景、路线与几何资产

本目录保存世界/机器人结构、静态几何参考和预备路线，以及传感器/机器人视觉mesh。它们在数据流中用途不同，不能统称为点云地图。

## 内容与上下游

| 目录/文件 | 表示 | 消费者 |
| --- | --- | --- |
| `world/three_floors.sdf` | 三平台、坡道及静态/动态物体 | `simulation.prepare`生成当前world |
| `world/go2_converted.sdf`、`go2_measured.urdf` | 刚体、关节、惯量、碰撞/传感器安装 | Gazebo及生成器 |
| `geometry/reference.json` | 43个静态解析有向box/有限表面 | 路线静态空间检查 |
| `geometry/metadata.json` | 历史表面采样/坐标元信息 | 几何约定；本身不是完整云数组 |
| `geometry/route.json` | 337点ground/body有序路线与support connector | 当前自动参考/路线围栏 |
| `geometry/canonical_frame.json` | 历史初始化/场景轴证据 | 保留元信息；当前运行不读取 |
| `meshes/` | DAE/STL外观与部分传感器模型 | 世界/机器人描述 |

历史metadata记录43盒、258面、679434采样点、≤.05m网格，但当前未包含对应完整XYZ/face_ids数组。日常定位地图由本次world重新提取解析box；SCAN地图由当前实际dense云构建；RGB档案走raw坐标，不从这些元信息加载导航占据图。

## 坐标与参数

world平台高度0/1.2/2.4m。静态几何路径检查使用metadata中的`T_navigation_world`，当前平移(0,0,−.3)m；运行定位另由当前scenario原点构造导航变换，必须核对二者契合。路线真实body/ground差约.32m；SCAN Path传输减/加.4是配对编码，不能当真实高度。

## 使用、许可与调试

根`start.py`的prepare复制/转换资产到新run，不应手改历史生成world代替正式源。修改几何后须检查碰撞/visual一致、参考/路线同坐标、支撑与机身空间限制；当前没有验证任意替换场景。

Unitree、VLP16、RealSense等来源许可保留在根`LICENSES/`。`L1_4D_Lidar.dae`的再分发条款尚未补足，当前分支树排除而本机保留；它同时用于碰撞，不以替代几何继承验收。Student权重同样未随本分支公开，见[models](../models/README.md)。从源码获取资产后仍需构建公共依赖，不能把下载完成当作导航验证完成。详细数据流见[技术报告第2章](../docs/technical_report.zh-CN.md)。
