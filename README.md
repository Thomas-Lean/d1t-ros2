# D1-T ROS2 控制工作区（参考版）

用 ROS2 Humble + MoveIt 2 控制宇树（Unitree）D1-T 桌面机械臂的工作区。本仓库是一个实际项目整理出来的**参考版本**：只保留能用的代码、两份 DDS 配置、宇树原始资料，以及一份操作手册，供同样在用 D1 / D1-T 的人参考。

> **安全第一条：唯一能立即停住机械臂的是断电。** Ctrl-C、取消（cancel）、停止发命令都不是急停。连实机前，先在手边装一个能一按就切断机械臂电源的开关。

---

## 1. 这个工作区能做什么

| 能力 | 说明 |
|---|---|
| ROS2 直接接上机械臂 | 用宇树官方 DDS 接口（`rt/arm_Command` / `rt/arm_Feedback`），**不改臂上任何程序** |
| 关节运动 | 上位机算好梯形速度，用 funcode 1 一次下发，七轴同时到达；按关节角反馈判断到位，不依赖厂商回执 |
| MoveIt 规划执行 | 标准 `/follow_joint_trajectory` 接口；`point`（逐点到位）和 `stream`（经由点不停车）两种执行方式 |
| 网页示教器 | 浏览器里关节 / 笛卡尔点动、夹爪、上下电、刚度、预设点位 |
| 夹爪判定 | 按夹爪停在哪判断空夹 / 夹到 / 不确定 / 没动（没有力传感） |
| 假控制器 | `fake_d1t` 扮演机械臂，不接实机也能跑通整条链路 |
| 视觉标定（参考实现） | RealSense 眼在手上：采样、手眼求解、像素→`base_link` 换算；附开发记录与实测数据，见 `03_d1t_ros2_ws/src/d1t_vision/README.md` |

已在一台 D1-T 上实机使用过：关节运动、MoveIt 执行、示教器、固定位置取放。**没有做成通用的视觉定位抓取**（视觉部分停在标定与受限测试阶段，原因见 `d1t_vision/README.md`），不要把本仓库当成即插即用的抓取方案。

## 2. 仓库结构

```text
.
├── README.md                  本文件
├── LICENSE                    Apache-2.0（本仓库自写代码）
├── cyclonedds.xml             实机 DDS 配置（只走直连网卡）
├── cyclonedds_sim.xml         仿真 DDS 配置（只走 127.0.0.1）
├── 01_vendor_work/            宇树原始资料（SDK 示例、URDF、臂上程序副本），说明见该目录 README
├── 03_d1t_ros2_ws/
│   ├── d1t_env.bash           实机环境入口（域 0、直连网卡）
│   ├── d1t_env_sim.bash       仿真环境入口（域 7、loopback）
│   └── src/                   9 个 ROS2 包
└── manifest/OPERATIONS.md     操作手册：节点、参数、话题、服务、底层协议、排障
```

`manifest/OPERATIONS.md` 原是项目内部手册，文中引用的 `START_HERE.md`、`forlearning/` 等内部文档**不在本仓库**；手册里的命令、参数、协议表仍然有效。

### 2.1 九个 ROS2 包

| 包 | 作用 |
|---|---|
| `unitree_arm` | 宇树 `ArmString` 消息定义（ROS2 直接收发臂上 DDS 话题靠它） |
| `d1t_interfaces` | 本项目的消息、服务、动作定义 |
| `d1t_driver` | `d1t_feedback_node`：把臂的 JSON 反馈拆成结构化的关节角、状态、回执 |
| `d1t_arm` | `arm_driver`（关节运动 + 上下电 / 倍率 / 刚度 / 夹爪服务）、`trajectory_server`（执行 MoveIt 轨迹）及两个调试工具 |
| `d1t_description` | URDF、网格、`joint_state_bridge`（度 → 弧度，处理 J4/J6 反号）及瓶架建模工具 |
| `d1t_moveit_config` | MoveIt 配置、碰撞体脚本、工具坐标与抓取场景工具 |
| `d1t_sim` | `fake_d1t`：假控制器 |
| `d1t_teach` | 网页示教器 `teach_server` |
| `d1t_vision` | 眼在手上视觉标定参考实现（离线求解 + 只读采样 / 坐标换算节点），开发记录见包内 README |

## 3. 关于这台臂，先知道这些（实测）

### 3.1 官方功能码哪些能用

| 功能码 | 实测结果 | 结论 |
|---|---|---|
| 1 单关节 + 自定时间 | 7 条连发全进；七轴到达时刻极差 0.000 s；只回 recv | 能用（本工作区全部运动都走它） |
| 2 mode 0 | 到位时间写死 40 ms；10 Hz 连续发目测明显抖 | 不建议用 |
| 2 mode 1 | 约 15°/s 各轴各算；执行期间臂上通信节点被堵，最长约 6 s | 不建议用 |
| 4 / 5 卸力、阻尼 | mode 0 卸力可用；mode 1 不能重新上使能；阻尼 0~1000 有梯度，1000 以上饱和，满档也撑不住自重 | 部分可用 |
| 6 供电开关 | 臂上程序里是空操作 | **不能当急停** |
| 7 回零 | 回全 0 位（不是折叠姿态），固定 6 s | 不建议用 |

没有独立的「上使能」指令：使能是运动命令的副作用。

### 3.2 哪些读数能信

| 读数 | 结论 |
|---|---|
| 关节角反馈 | 约 8.98 Hz（官方标 10 Hz），会断帧 0.41–0.61 s，断后首帧可能是旧值 |
| recv 回执 | 可信：说明 JSON 到了臂上 |
| exec 回执 | 不可信：臂上按算出的时间延时发，不看臂到没到；funcode 1 不回 |
| 状态字段 | 不可信：`enable_status` 在臂下沉时也报 1；`error_status` 恒为 0；`power_status` 与使能无关 |

判断到没到位，只看关节角反馈。

### 3.3 夹爪换算

夹爪命令和反馈都是无量纲的 raw 值，不是毫米。空载时：

**爪间距(mm) ≈ 0.85 × raw + 19.8**（6 组尺量拟合，残差 < ±2 mm，适用 raw 0~50）

| raw | 尺量 | 说明 |
|---|---|---|
| 0 | ≈ 20 mm | 命令 0 不是合拢 |
| 49.7 | 60.8 mm | 出厂上限附近 |
| 64.5 | 70 mm | 放宽限位后的上限；50 以上公式外推不准（会算成 74.6） |

夹住物体后换算出的毫米会偏大 3 mm 以上，判断夹没夹住看 raw 停在哪，不看毫米。

### 3.4 其他

- 舵机：7 颗深圳华馨京（FashionStar）总线舵机，共用一根串口 115200 bps；J0–J2 为 RX18-U100H-M，J3–J6 为 HX8-U50H-M。出厂固件 V311 不支持一次读电压电流温度（0x16）、软停止（0x18）、同步读写（0x19），这些需要 V330。本工作区在 V311、V330 上都能用。
- 本工作区的关节限位按实测行程放宽过（`d1t_arm/trapezoid_planner.py`、URDF）。**真正截断命令的是臂上程序**：臂上不改，放宽无效，机械臂停在原厂限位且不报错。原厂限位：J0/J3/J5 ±135°，J1/J2/J4 ±90°，夹爪 −20~50。改臂上程序属于高风险操作，请自行评估。
- 机身 Type-C 口不对外开放，开发一律走 RJ45 网口（宇树售后答复）。

### 3.5 进入臂上控制器（臂载计算机）

臂上控制器是一台 Ubuntu 小计算机，网线直连后可以 SSH 登录（宇树官方文档给出的出厂账号）：

```bash
ssh ubuntu@192.168.123.100
```

出厂密码：`123`。臂上出厂控制程序由 4 个 systemd 服务开机自启（`marm_controller`、`marm_communication`、`marm_control`、`marm_subscripber`，最后一个是原文拼写），源码在 `/home/ubuntu/marm_code`，本仓库副本见 `01_vendor_work/D1_Onboard_Origin/`。舵机串口 `/dev/ttyS4` 同一时间只能被一个程序占用，要直接访问舵机须先停这 4 个服务；断电重启后它们会自启。

宇树官方说明（接口协议、SDK、URDF、D1-T 双臂配置）：https://support.unitree.com/home/zh/developer/D1Arm_services

## 4. 部署

### 4.1 环境

| 项 | 要求 / 已验证 |
|---|---|
| 系统 | Ubuntu 22.04 x86_64（已验证 22.04.5） |
| ROS2 | Humble（`/opt/ros/humble`，已验证 `ros-humble-desktop` 0.10.0） |
| DDS | CycloneDDS（已验证 `ros-humble-rmw-cyclonedds-cpp` 1.3.4） |
| MoveIt | MoveIt 2 for Humble（已验证 2.5.9） |
| 网络 | 一个有线网口，网线直连机械臂 RJ45 |

### 4.2 安装依赖

先按 ROS2 官方文档装好 `ros-humble-desktop`，再：

```bash
sudo apt update
```

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-moveit ros-humble-control-msgs ros-humble-joint-state-publisher-gui ros-humble-xacro python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy python3-yaml python3-matplotlib libfcl-dev nlohmann-json3-dev
```

### 4.3 获取代码与配置网卡

```bash
git clone <本仓库地址> d1t && cd d1t
```

仓库可以放在任意路径，环境脚本会按自身位置找工作区。只有一处要按你的电脑改：**直连网卡名**（默认 `enp5s0`）。查网卡名：

```bash
ip -br link
```

如果不是 `enp5s0`，改两处：`cyclonedds.xml` 里的 `<NetworkInterface name="enp5s0" .../>`，以及运行前设置环境变量（或直接改 `03_d1t_ros2_ws/d1t_env.bash` 里的默认值）：

```bash
export D1T_IFACE=<你的网卡名>
```

### 4.4 构建

```bash
cd 03_d1t_ros2_ws && source /opt/ros/humble/setup.bash && colcon build
```

预期最后是 `Summary: 9 packages finished`。部分包向 stderr 打信息属正常。不要混用 `--symlink-install`；改代码后重新 build，并在新终端重新 source。

## 5. 不接机械臂：仿真跑通

每个终端先（在仓库根目录）：

```bash
source 03_d1t_ros2_ws/d1t_env_sim.bash
```

预期输出 `ROS_DOMAIN_ID : 7`。仿真和实机不要在同一个终端切换。

终端 0：确认域 7 干净，再起假控制器：

```bash
d1t_sim_check
```

```bash
ros2 run d1t_sim fake_d1t --ros-args -p speed_deg_per_s:=60.0
```

终端 1、2：

```bash
ros2 run d1t_driver d1t_feedback_node
```

```bash
ros2 run d1t_arm arm_driver
```

终端 3：发一个关节目标（7 个值是控制器轴 a0~a6，单位度；a6 是夹爪 raw）：

```bash
ros2 action send_goal /move_to_joints d1t_interfaces/action/MoveToJoints "{target_deg: [0.0, -60.0, 60.0, 0.0, 0.0, 0.0, 30.0], timeout_sec: 0.0, speed_ratio: 0.3, accel_ratio: 1.0}"
```

预期 `arrived: true`、`SUCCEEDED`。连 MoveIt 一起测，再开：

```bash
ros2 launch d1t_moveit_config d1t_moveit.launch.py
```

```bash
ros2 run d1t_arm trajectory_server --ros-args -p execution_mode:=point -p max_points:=400
```

## 6. 连接实机

### 6.1 网络

| 设备 | 地址 |
|---|---|
| 机械臂（出厂） | `192.168.123.100` |
| 本机直连网卡 | 静态 IP `192.168.123.x/24`（不能用 `.100`） |

```bash
ping -c 5 192.168.123.100
```

预期约 1 ms、0% 丢包。收不到数据时先查：网线；代理软件的虚拟网卡（Clash / Meta TUN）会劫持 `192.168.123.0/24`，要关掉。

### 6.2 启动

每个实机终端先：

```bash
source 03_d1t_ros2_ws/d1t_env.bash
```

预期 `ROS_DOMAIN_ID : 0`、网卡 `operstate=up`。确认收得到反馈（约 9 Hz）：

```bash
ros2 topic hz /arm_Feedback
```

| 组合 | 终端顺序 |
|---|---|
| 点到点 | 反馈节点 → `arm_driver` |
| MoveIt 全栈 | 反馈节点 → `arm_driver` → MoveIt → `trajectory_server` |
| 网页示教器 | 反馈节点 → `arm_driver` → MoveIt → `teach_server`（不要自己起 `trajectory_server`） |

```bash
ros2 run d1t_driver d1t_feedback_node
```

```bash
ros2 run d1t_arm arm_driver
```

```bash
ros2 launch d1t_moveit_config d1t_moveit.launch.py
```

`trajectory_server` 二选一（启动时定死）。`point` 每个经由点等到位，靠近物体用它；`stream` 经由点不停车，只用于开阔区域。`max_points:=400` 必须带（默认 64，MoveIt 轨迹常超）：

```bash
ros2 run d1t_arm trajectory_server --ros-args -p execution_mode:=point -p max_points:=400
```

```bash
ros2 run d1t_arm trajectory_server --ros-args -p execution_mode:=stream -p zone_deg:=2.0 -p max_points:=400
```

网页示教器，本机浏览器打开 `http://127.0.0.1:8175`：

```bash
ros2 run d1t_teach teach_server --ros-args -p bind_host:=127.0.0.1 -p bind_port:=8175 -p max_scaling:=1.0 -p vel_scaling:=1.0 -p acc_scaling:=1.0 -p csv_dir:=/tmp
```

从另一台电脑用，先在那台电脑上做 SSH 端口转发（`<主机名>` 换成你 SSH 登录本机用的名字）：

```bash
ssh -N -L 8175:127.0.0.1:8175 <主机名>
```

### 6.3 第一次动臂、回 HOME、下电

1. 臂周围清空，手放在断电开关上；先用小幅、低速（`speed_ratio` 0.1~0.3）的目标试。
2. 回 HOME（折叠姿态；本工作区 HOME 的 a2 是 89）。这一步不经过碰撞检查，先目视确认路径：

```bash
ros2 action send_goal /move_to_joints d1t_interfaces/action/MoveToJoints "{target_deg: [0.0, -90.0, 89.0, 0.0, 0.0, 0.0, 0.0], timeout_sec: 0.0, speed_ratio: 0.3, accel_ratio: 1.0}"
```

3. 在折叠姿态下电：

```bash
ros2 service call /set_servo d1t_interfaces/srv/SetServo "{enable: false}"
```

4. 逐个终端 Ctrl-C（后起的先停），最后断电。

## 7. 安全须知

| 事实 | 做法 |
|---|---|
| 唯一急停 = 断电 | cancel、Ctrl-C 只是「不再发下一段」，当前这段会走完 |
| 卸力后臂会下垂，阻尼开满也撑不住自重 | 折叠姿态以外下电，臂会跌落，下方必须清空 |
| 折叠姿态下 a4 往正方向会撞 a1 关节中心 | 折叠姿态附近只让 a4 往负方向走 |
| 示教器「按住即走」不经 MoveIt、不做碰撞检查 | 靠目视，速度先调低 |
| `stream` 模式下经由点没走到也会发下一点 | 靠近物体用 `point` |
| `trajectory_server` 不能和手发 `/move_to_joints` 并发 | 轨迹执行期间不调 `/set_servo`、`/set_stiffness`、`/set_override` |
| 碰撞体脚本 `set_desk_collision` 按作者自己的桌面写死 | 用之前按你的现场实测值改 |

## 8. 常见问题

| 现象 | 先查 |
|---|---|
| 收不到 `/arm_Feedback` | 网线、网卡 `operstate`、代理 TUN、`ping`；是否误 source 了仿真入口 |
| 无法解析 `unitree_arm` 类型 | 没 build，或没 source `install/setup.bash` |
| `arm_driver` 目标全回 `not_ready` | 反馈节点没起；仿真里 `fake_d1t` 没起 |
| 轨迹被整条拒绝 | `trajectory_server` 没带 `-p max_points:=400` |
| `stream` 一启动就退出 | `/arm_Command` 上还有别的发布者 |
| `ros2 topic pub --once` 臂没反应 | 臂的订阅端 BEST_EFFORT，首条会丢；用 `-w 1 -r 1 -t 4`，`arm_driver` 在跑时不要手发 |
| 仿真里 `/set_servo enable:false` 回 `move_failed` | 正常：`fake_d1t` 不认卸力命令 |

更多节点参数、话题、协议细节见 [`manifest/OPERATIONS.md`](manifest/OPERATIONS.md)。

## 9. 第三方资料与来源

| 内容 | 来源 | 许可 |
|---|---|---|
| `01_vendor_work/D1_SDK_Origin/`（SDK 示例）、`03_d1t_ros2_ws/src/unitree_arm/`（消息定义） | 宇树 D1 机械臂服务文档与 SDK：https://support.unitree.com/home/zh/developer/D1Arm_services | 以宇树官方为准 |
| `01_vendor_work/d1_550_description/`（URDF、网格） | 同上，官方 URDF 下载 | `package.xml` 声明 BSD |
| `01_vendor_work/D1_Onboard_Origin/marm_code/`、`autoStart*.sh` | 臂上源码：D1-T 臂载计算机上出厂控制程序的源码与自启脚本（`/home/ubuntu`） | 以宇树官方为准 |
| `01_vendor_work/D1_Onboard_Origin/fashionstar-uart-servo-cpp/` | 华馨京（FashionStar）串口总线舵机 C++ 库 | MIT（见其 LICENSE） |
| 上述库内 `dependency/CSerialPort/` | CSerialPort | LGPL-3.0（见其 LICENSE） |

宇树官网 D1-T 页面：https://www.unitree.com/cn/D1-T/

## 10. 许可证

本仓库自写代码（`03_d1t_ros2_ws/src/` 下除 `unitree_arm` 消息定义外的各包、环境脚本、DDS 配置、文档）以 Apache-2.0 发布，见 [`LICENSE`](LICENSE)。`01_vendor_work/` 与 `unitree_arm` 的内容版权归原作者，按第 9 节各自的许可。
