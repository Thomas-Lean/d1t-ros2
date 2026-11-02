# 01_vendor_work：宇树原始资料

本目录内容来自宇树（Unitree）及其舵机供应商，原样保存，仅供阅读参考；本仓库的 ROS2 代码不编译、不依赖本目录。

| 目录 | 内容 | 来源 |
|---|---|---|
| `D1_SDK_Origin/` | 宇树 D1 SDK 示例（5 个 C++ 例程 + 消息定义），依赖 unitree_sdk2 | 宇树文档中心「D1 机械臂服务」：https://support.unitree.com/home/zh/developer/D1Arm_services （页面内 `D1_SDK.zip`） |
| `d1_550_description/` | 宇树官方 D1-550 URDF 与网格 | 同上页面「机械臂 URDF 下载」 |
| `D1_Onboard_Origin/marm_code/`、`autoStart*.sh` | 臂上源码：D1-T 臂载计算机上出厂控制程序的源码与自启脚本 | 臂载计算机 `/home/ubuntu`（`ssh ubuntu@192.168.123.100`，出厂密码 `123`） |
| `D1_Onboard_Origin/fashionstar-uart-servo-cpp/` | 华馨京（FashionStar）串口总线舵机 C++ 库（臂上控制程序所用） | 臂载计算机副本；MIT 许可，见该目录 `LICENSE`；内含 CSerialPort（LGPL-3.0） |

宇树官方说明：https://support.unitree.com/home/zh/developer/D1Arm_services

版权归原作者所有。使用前请以宇树及各库的官方许可为准。
