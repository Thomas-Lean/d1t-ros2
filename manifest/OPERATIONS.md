# D1-T 操作手册（现阶段）

> **2026-09-19 适用边界提醒**：正文保留 OPS-01 的核实日期。TCP、`/grip` 与示教抓放的后续用法参考 `forlearning/S4b-4-tcp-pose/README.md`；本文旧末端名和旧演示不能覆盖该阶段记录。VISION-01 的 Mac 离线/ubox 现场流程按正式任务书执行，尚无新的现场通过结论。本次未读取源码重核命令，也未运行本手册命令。

> 取代原节点速查表（2026-09-08 版，OPS-01 删除，找回方式见 `docs/OPS-01-report.md`）。本文只回答：有什么节点、怎么启动和停、怎么下命令、每个字段填什么、合法范围多少、填错或越界会怎样。
> 原理不在这里讲，只给阶段文档链接。测试与探针工具（`*_fake.sh`、pytest、`traj_test_client`、`s4b3_run10`、`s4b3_clearance`）不在本手册，归《开发配置及测试手册》（未写）。

- **适用代码**：`main` 的 `d772f46` 起（2026-09-17 大扫除之后、S4b-3 验收之后）。代码改了而本文没跟上时，**以代码为准**。
- **核实**：2026-09-17，OPS-01 代码子窗口。读了当前源码与厂商源码；A、J 节命令在仿真（域 7）跑过，原始输出在 `docs/OPS-01-logs/`。**本次没有跑实机**，实机数值只引用已有记录。
- 当前状态、验收结论、待决事项以 `manifest/START_HERE.md` 为准；安全限制原文见其 ·「2.3 运行限制（安全相关，不许为压行数删）」。

---

## 读前须知

### 数值标签

| 标签 | 含义 |
|---|---|
| 【代码】 | 从当前源码逐字抽取，文件名写在旁边 |
| 【实测】 | 引用 START_HERE 或阶段文档里的实机记录，出处写在旁边；本次没重测 |
| 【厂商源码】 | 读 `01_vendor_work/D1_Onboard_Origin/marm_code/src/` 得出，没实测 |
| 【推断】 | 由上面几类推出来，没单独验证 |
| 【未验证】 | 没有证据，或记录互相矛盾 |

「仿真核对：c1/G2」= 本次仿真里跑过，日志在 `docs/OPS-01-logs/c1_p2p/G2_*.log`。目录对照：c1 = `c1_p2p`，c2 = `c2_fjt`，c3 = `c3_moveit`，c4 = `c4_view_env`，c5 = `c5_extra`。
**仿真不代表实机**：`fake_d1t` 各轴独立匀速走、不按 `delay_ms` 走、不认卸力命令，所以仿真里的耗时、路径形状、卸力结果都不作数。

### 六个词（第一次看先读）

| 词 | 人话 | 产线类比 |
|---|---|---|
| 节点（node） | 一个正在跑的程序 | 一台设备的控制器 |
| 话题（topic） | 单向广播，发的人不管谁收 | 传感器信号线、看板 |
| 服务（service） | 一问一答，马上回结果 | 按钮 + 指示灯 |
| 动作（action） | 长任务：发目标 → 过程中报进度 → 最后报结果；中途可以请求取消 | 启动一个加工程序，过程中看状态，最后报完成或报警 |
| QoS（Quality of Service，服务质量） | 消息怎么送。`RELIABLE` 丢了重发，`BEST_EFFORT` 丢了不管；`KEEP_LAST(n)` = 接收方只留最近 n 条 | 挂号信 / 平信；信箱有几个格子 |
| DDS 与域号 `ROS_DOMAIN_ID` | ROS2 底下的通信层；域号相同的节点才互相看得见 | 对讲机频道 |

---

## A. 启动与环境

适用：S4b-3 之后｜核实：2026-09-17，读两个环境脚本 + 仿真 c1~c4

### A.0 先记住三件事

1. **唯一能立即停住机械臂的是断电。** Ctrl-C、取消（cancel）、停发命令都不是急停（START_HERE 2.3）。
   - 在 `ros2 action send_goal` 那个终端按 Ctrl-C = 发取消请求。`/follow_joint_trajectory` 会**把当前这段走完**才停（仿真核对：c2/P5，终端打印 `Canceling goal...`，结果「收到取消请求时已完成 2/3 段，当前段走完后停止」）。`/move_to_joints` 取消时不发任何刹车命令，臂上已收到的目标照走【代码：arm_driver `_tick` 收到取消只结束等待】（仿真核对：c5/X3，取消后结果 `reason: canceled`，假控制器仍走到目标 a0 = 90°）。
   - 在节点终端按 Ctrl-C = 节点退出。退出流程不给臂发命令【代码】；已经发到臂上的目标照走【推断】。
   - `/set_servo enable:false`、`/set_stiffness`（`mode` < 1000）是卸力，展开姿态下臂会砸下来，**不是急停**。
2. 实机命令由 Thomas 执行，执行时手放在电源上。
3. 仿真域 7 是全机共享的：起仿真前先 `pgrep -af fake_d1t`，有别人的进程就等，不杀别人的进程。

### A.1 两个环境入口

| 项 | 实机 `d1t_env.bash` | 仿真 `d1t_env_sim.bash` |
|---|---|---|
| 完整路径 | `<仓库根>/03_d1t_ros2_ws/d1t_env.bash` | `<仓库根>/03_d1t_ros2_ws/d1t_env_sim.bash` |
| 加载 ROS | `/opt/ros/humble/setup.bash` | 同左 |
| 加载工作区 | **主仓库** `03_d1t_ros2_ws/install/setup.bash`（路径写死在脚本里） | 同左 |
| `RMW_IMPLEMENTATION`（选哪家 DDS） | `rmw_cyclonedds_cpp` | 同左 |
| `ROS_DOMAIN_ID` | `0` | `7` |
| `ROS_LOCALHOST_ONLY` | `0` | `0` |
| DDS 配置 `CYCLONEDDS_URI` | 仓库根 `cyclonedds.xml`：只走网卡 `enp5s0` | 仓库根 `cyclonedds_sim.xml`：只走 `127.0.0.1`，关组播，单播发现 |
| 配置文件不在时 | 不设 `CYCLONEDDS_URI`，照常继续（DDS 自己挑网卡，可能被代理 Meta TUN 劫持） | **中止**（防止仿真流量跑进实机网段） |
| 自检 | 打印 `enp5s0` 的 `operstate`，不是 `up` 就提示 | 提供函数 `d1t_sim_check`：域 7 上看到 `/arm_Feedback` 就报 ✗ |
| 用法 | 必须 `source`，直接执行会报错退出 | 同左 |

以上全部【代码：两个脚本】。实机网络：ubox 侧 `enp5s0` = `192.168.123.233/24`，D1-T = `192.168.123.100`（`.claude/CLAUDE.md` §1）。

- 实机、仿真各开各的终端，同一个终端不要先后 source 两个入口（仿真脚本头注释）。
- 仿真核对：c4/E1（实机脚本只 source、不起节点：`ROS_DOMAIN_ID : 0`、`operstate=up`）；c1/00（仿真脚本 + `d1t_sim_check` 打出 ✓）。

### A.2 在 worktree 里跑：两行加载，顺序固定

两个入口脚本都把工作区写死成主仓库。**在 worktree 里只 source 第一行，`ros2 run` 会静默跑主仓库的旧代码，不报错。**

- 第 1 行：`source <仓库根>/03_d1t_ros2_ws/d1t_env.bash`（仿真换成 `d1t_env_sim.bash`）
- 第 2 行：`source` 该 worktree 自己的 `03_d1t_ros2_ws/install/setup.bash`（完整路径由该任务的报告给出）
- 确认跑的是哪份代码：

```bash
# 输出路径在 worktree 下 = 两行都加载了；在 <仓库根>/03_d1t_ros2_ws/install/ 下 = 跑的是主仓库
ros2 pkg prefix d1t_arm
```

仿真核对：c4/E2（只加载第一行输出主仓库路径，再加载第二行输出 worktree 路径）。

### A.3 常用启动组合与终端顺序

| 组合 | 用途 | 终端 0 | 终端 1 | 终端 2 | 终端 3 | 终端 4 | 终端 5 |
|---|---|---|---|---|---|---|---|
| ① 只看反馈 | 读关节角和状态，不动臂 | — | 反馈节点 | J.1 看数 | | | |
| ② 点到点 | 手发 `/move_to_joints`、调服务 | — | 反馈节点 | `arm_driver` | J.2 / J.3 发命令 | | |
| ③ MoveIt 全栈（实机） | 规划避障、执行轨迹 | — | 反馈节点 | `arm_driver` | MoveIt launch | `trajectory_server` | 碰撞体脚本，然后 J.5 发目标 |
| ④ 仿真全栈 | 不碰实机，验证命令与链路 | 自检 + `fake_d1t` | 同 ③ | 同 ③ | 同 ③ | 同 ③ | 同 ③ |

顺序为什么这样排：

| 要求 | 不这样会怎样 |
|---|---|
| 仿真先起 `fake_d1t` | `arm_driver` 发现 `/arm_Command` 没有收件方，所有运动目标回 `not_ready`【代码：arm_driver `_run`】 |
| 反馈节点先于或同时于 `arm_driver` | 规划起点需要一帧不超过 0.5 s 的反馈，否则 `not_ready`【代码：`feedback_stale_sec`】 |
| `arm_driver` 先于 `trajectory_server` | point 模式每段都要调 `/move_to_joints`；stream 模式启动时只允许 `d1t_trajectory` 和 `d1t_arm` 两个发布者【代码】 |
| MoveIt launch 先于碰撞体脚本 | 碰撞体服务 `/apply_planning_scene` 由 `move_group` 提供；脚本等 10 s 没等到就退出码 1【代码：set_desk_collision.py】 |
| `trajectory_server` 起好再发 MoveIt 目标 | MoveIt 执行时找 `/follow_joint_trajectory`【代码：moveit_controllers.yaml】，没人提供就执行失败【推断】 |

**用 Web 示教界面（`teach_server`）时不要自己起 `trajectory_server`**：`teach_server` 会用 `-p execution_mode:=stream -p max_points:=400 -p zone_deg:=2.0` 自己托管一个（主窗口 2026-09-22 裁定），也不跑碰撞体脚本（Thomas 2026-09-22 裁定）。终端顺序：反馈节点 → `arm_driver` → MoveIt launch → `teach_server`；完整命令见 TEACH-01 报告 ·「19. 本轮（第 12~18 节）汇总回归与当前启动命令」。

`trajectory_server` 同一时间只能起一个（两个都提供 `/follow_joint_trajectory`）。point / stream 在启动时定死，**切换要 Ctrl-C 后重起**。

#### A.3.1 每个终端先加载环境

实机：

```bash
# 适用：实机｜核实：2026-09-17 仿真 c4/E1（只 source，未起节点）
# 看输出两行：ROS_DOMAIN_ID : 0；网卡 enp5s0 : operstate=up（不是 up 先查网线、代理 Meta TUN）
source <仓库根>/03_d1t_ros2_ws/d1t_env.bash
```

仿真：

```bash
# 适用：仿真｜核实：2026-09-17 仿真 c1/00
# 看输出：ROS_DOMAIN_ID : 7；CYCLONEDDS_URI 指向 cyclonedds_sim.xml
source <仓库根>/03_d1t_ros2_ws/d1t_env_sim.bash
```

#### A.3.2 终端 0（只在仿真）：查占用 → 自检 → 假控制器

```bash
# 适用：仿真｜核实：2026-09-17 仿真 c5/X0（无输出，退出码 1 = 没人在跑）
# 有输出 = 别人正在跑 fake_d1t，等他跑完，不要杀
pgrep -af fake_d1t
```

```bash
# 适用：仿真｜核实：2026-09-17 仿真 c1/00（✓ 无 /arm_Feedback，可以启动 fake_d1t）
# 打出 ✗ 就停：域 7 上已经有 /arm_Feedback（别人的仿真，或没隔离干净）
d1t_sim_check
```

```bash
# 适用：仿真｜核实：2026-09-17 仿真 c1/T1（fake_d1t started: ... speed=60.00 deg/s）
# speed_deg_per_s:=60.0 → 假控制器各轴匀速 60 °/s。默认 11.5 太慢，stream 最后一点「规划时长 + 3 s」会超时误报
ros2 run d1t_sim fake_d1t --ros-args -p speed_deg_per_s:=60.0
```

#### A.3.3 终端 1：反馈节点

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/T2
# 无参数。启动行：d1t_feedback_node started: subscribing /arm_Feedback ...
# 之后每 2 s 打一行关节角和状态——这是日志限流，不是反馈频率
ros2 run d1t_driver d1t_feedback_node
```

#### A.3.4 终端 2：arm_driver

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/T3
# 无参数 = 全部默认值（F.1）。启动行：d1t_arm ready: action /move_to_joints, services /set_servo /set_override /set_stiffness
ros2 run d1t_arm arm_driver
```

#### A.3.5 终端 3：MoveIt

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c5/T4（逐字）、c3/T4——本会话没有显示器，RViz 起不来（rviz2 退出码 -6），
#   move_group、robot_state_publisher、joint_state_bridge 正常；RViz 窗口本身【未验证】
# 无参数。拉起 robot_state_publisher、joint_state_bridge、move_group、RViz（B.2）
ros2 launch d1t_moveit_config d1t_moveit.launch.py
```

实机在 ubox 屏幕上看 RViz 时，先在这个终端执行 `export DISPLAY=:0` 再起 launch（S4b-3 阶段文档 ·「6. 复现步骤」）。

#### A.3.6 终端 4：trajectory_server（二选一）

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c2/T4（启动行 execution_mode=point max_points=400）
# execution_mode:=point → 每个点都经 /move_to_joints 等到位。靠近物体、障碍、下探抓取必须用它（START_HERE 2.3）
# max_points:=400 → 一条轨迹最多 400 个点。默认 64，MoveIt 轨迹常超，超了整条拒绝（K.2）
ros2 run d1t_arm trajectory_server --ros-args -p execution_mode:=point -p max_points:=400
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c2/T5、c3/T5（「stream 入口独占检查通过」）
# execution_mode:=stream → 经由点不停车。只用于开阔区域的转运段（START_HERE 2.3）
# zone_deg:=2.0 → 本段要动的轴全部进入目标 ±2.0° 就发下一点（2026-10-27 起这就是默认值）
# max_points:=400 → 同上（点数检查在抽稀之前做）
# 启动约 2 s 后必须看到「stream 入口独占检查通过」；看到「拒绝启动 stream」并退出 = /arm_Command 上还有别的发布者
ros2 run d1t_arm trajectory_server --ros-args -p execution_mode:=stream -p zone_deg:=2.0 -p max_points:=400
```

#### A.3.7 终端 5：碰撞体

```bash
# 适用：实机 / 仿真（S4b-3 起）｜核实：2026-09-17 仿真 c3/M1
# 无参数。成功输出含 apply_planning_scene success=True 和 SAFETY_MARGIN=0.030；每次重起 MoveIt launch 后要重跑
ros2 run d1t_moveit_config set_desk_collision
```

### A.4 停机与下电顺序

1. 等当前运动结束：`send_goal` 终端打出 `Goal finished with status`，或 J.1 读到的角度不再变。
2. 回 HOME（J.2 第一条）。路径不经过碰撞检查，从远处回来先自己看一眼路径。
3. 要下电时：臂在折叠区内调 `/set_servo enable:false`（J.3），回 `success=True` 再往下。HOME `[0,-90,90,0,0,0,0]` 接近自然掉电姿态【实测：START_HERE 档案 ·「高频协议测试位姿」】。
4. 后起的先停，逐个终端 Ctrl-C：发命令的终端 → `trajectory_server` → MoveIt launch → `arm_driver` → 反馈节点 →（仿真）`fake_d1t`。
5. 断电。

- 仿真核对：c1~c4 都按逆序 Ctrl-C，停完 `ros2 node list` 为空（各目录 `Z_after_stop.log`）。
- 停机时会看到的两段报错不代表出了问题：`trajectory_server` 打一段 `ExternalShutdownException` Traceback 并 `Process exited with failure 1`（仿真核对：c2/T4、T5）；MoveIt launch 里 `move_group` 可能 `exit code -11`（仿真核对：c3/T4，1 次）。见 K.4。

---

## B. 节点清单

适用：`main` `d772f46`｜核实：2026-09-17 源码（`setup.py`、`CMakeLists.txt`、各节点构造函数）+ 仿真 `ros2 node list`（c1/C2、c2/P0、c3/M0、c4/V1）

三个名字容易混：**包名**（`ros2 run` 第 1 个参数）、**可执行名**（第 2 个参数）、**节点名**（日志 `[...]` 里、`ros2 node list` 里显示的）。

### B.1 可执行程序

| 包 | 可执行名 | 节点名 | 作用 | 类别 |
|---|---|---|---|---|
| `d1t_driver` | `d1t_feedback_node` | `d1t_feedback_node` | 把 `/arm_Feedback` 的 JSON 拆成 3 条结构化话题；不发任何命令 | 生产，常驻 |
| `d1t_arm` | `arm_driver` | **`d1t_arm`**（≠ 可执行名，与包名相同） | 点到点运动 `/move_to_joints` + 三个服务；常驻的 `/arm_Command` 发布者 | 生产，常驻 |
| `d1t_arm` | `trajectory_server` | **`d1t_trajectory`**（≠ 可执行名） | 把标准轨迹（MoveIt 发来的）拆成逐段运动；point / stream 两种跑法 | 生产，用 MoveIt 时常驻 |
| `d1t_description` | `joint_state_bridge` | `joint_state_bridge` | 度数反馈 → `/joint_states`（弧度，供 RViz / MoveIt） | 生产，一般由 launch 起 |
| `d1t_moveit_config` | `set_desk_collision` | `set_desk_collision` | 往 MoveIt 场景加桌面、木工夹、木板三个碰撞盒，发一次就退出 | 生产，一次性脚本 |
| `d1t_moveit_config` | `s4b2_demo` | `s4b2_demo` | 一键演示：HOME → 目标 A → 绕行目标 B → 上抬 0.10 m → HOME | 常用演示（**与当前碰撞体冲突，见 K.3**） |
| `d1t_teach` | `teach_server` | `teach_server` | Web 手动控制界面（示教器第一版），默认 `127.0.0.1:8175`；托管一个 stream 模式的 `trajectory_server` 子进程；「按住即走」直接发 `/arm_Command`、**不经 MoveIt、不做任何碰撞检查** | 生产，人工示教时常驻 |
| `d1t_sim` | `fake_d1t` | `fake_d1t` | 假控制器：收 `/arm_Command` 回 recv，发 10 Hz 反馈 | 仿真专用 |

不在本手册：`d1t_arm` 的 `traj_test_client`，`d1t_moveit_config` 的 `s4b3_run10`、`s4b3_clearance`、`s4b3_run10_all.sh`（测试工具）。

### B.2 launch 文件（一次拉起多个节点的启动文件）

| 包 | launch 文件 | 拉起 | 不拉起 |
|---|---|---|---|
| `d1t_description` | `view_live.launch.py` | `robot_state_publisher`、`joint_state_bridge`、`rviz2` | 驱动、反馈节点 |
| `d1t_description` | `view_gui.launch.py` | `robot_state_publisher`、`joint_state_publisher_gui`（滑块）、`rviz2` | 不连臂、不读反馈 |
| `d1t_teach` | `teach.launch.py` | `teach_server` 一个节点 | 其余全部要单独起 |
| `d1t_moveit_config` | `d1t_moveit.launch.py` | `robot_state_publisher`、`joint_state_bridge`（固定 `source=feedback`）、`move_group`、`rviz2`（MoveIt 面板） | `fake_d1t`、反馈节点、`arm_driver`、`trajectory_server`——要单独起 |

`move_group` 进程还会带出 `move_group_private_*`、`moveit_simple_controller_manager`、`transform_listener_impl_*` 几个节点名，正常（仿真核对：c3/M0）。

### B.3 每个节点收发什么

| 节点名 | 订阅 | 发布 | 提供服务 / 动作 | 调用别人的 |
|---|---|---|---|---|
| `d1t_feedback_node` | `/arm_Feedback` | `/d1t/joint_feedback`、`/d1t/arm_status`、`/d1t/command_ack` | — | — |
| `d1t_arm` | `/d1t/joint_feedback`、`/d1t/command_ack`、`/d1t/arm_status` | `/arm_Command` | 动作 `/move_to_joints`；服务 `/set_servo`、`/set_override`、`/set_stiffness` | — |
| `d1t_trajectory` | `/d1t/joint_feedback`；stream 另订 `/move_to_joints/_action/status` | stream 模式才发 `/arm_Command` | 动作 `/follow_joint_trajectory` | 动作 `/move_to_joints` |
| `joint_state_bridge` | `/d1t/joint_feedback`（`source=feedback` 时） | `/joint_states` | — | — |
| `set_desk_collision` | — | — | — | 服务 `/apply_planning_scene` |
| `s4b2_demo` | `/d1t/joint_feedback`、TF | — | — | 动作 `/move_action`、`/execute_trajectory`；服务 `/compute_cartesian_path` |
| `fake_d1t` | `/arm_Command` | `/arm_Feedback` | — | — |
| `move_group`（MoveIt 自带） | `/joint_states` 等 | — | 动作 `/move_action`、`/execute_trajectory`；服务 `/apply_planning_scene`、`/compute_cartesian_path`、`/compute_fk`、`/compute_ik`、`/get_planning_scene`、`/plan_kinematic_path` 等 | 动作 `/follow_joint_trajectory` |

全部【代码】；`move_group` 一行的服务 / 动作名由仿真核对 c3/M0 列出。

---

## C. 话题

适用：`main` `d772f46`｜核实：2026-09-17 源码 + 仿真 `ros2 topic info -v`（c1/C1、c2/P0、c2/Q0）

### C.1 话题总表

| 话题 | 类型 | 谁发 → 谁收 | QoS | 频率 |
|---|---|---|---|---|
| `/arm_Command` | `unitree_arm/msg/ArmString` | `d1t_arm`（常驻）、`d1t_trajectory`（仅 stream）→ 臂上控制器（仿真：`fake_d1t`） | 我方发布端 `BEST_EFFORT` / `KEEP_LAST(1)` / `VOLATILE`【代码】；臂上读端同为 `BEST_EFFORT` / `KEEP_LAST(1)`【代码注释 arm_driver.py；S0 阶段文档】 | 按需。一个点到点目标 = 7 条（每轴一条）【代码】 |
| `/arm_Feedback` | `unitree_arm/msg/ArmString` | 臂上控制器（仿真：`fake_d1t`）→ `d1t_feedback_node` | 订阅端 `RELIABLE` / `KEEP_LAST(10)` / `VOLATILE`【代码】；`fake_d1t` 发布端 `RELIABLE` / `KEEP_LAST(1)`【代码】；臂上发布端【未验证】 | 关节帧约 8.98 Hz（周期 0.110~0.117 s）【实测：START_HERE 档案 ·「实现进度」S2 条】；会丢帧，出现过 0.41~0.61 s 的洞，洞后首帧可能是旧值【实测：START_HERE 2.3】；状态帧约 10 Hz【厂商源码：100 ms 定时，未实测】；仿真两类帧各 10 Hz【代码】 |
| `/d1t/joint_feedback` | `d1t_interfaces/msg/JointFeedback` | `d1t_feedback_node` → `d1t_arm`、`d1t_trajectory`、`joint_state_bridge`、`s4b2_demo` | `RELIABLE` / `KEEP_LAST(10)` / `VOLATILE`【代码】 | 每收到一帧完整关节帧发一条：实机约 8.98 Hz【实测】；仿真 10.0 Hz（仿真核对：c1/C1） |
| `/d1t/arm_status` | `d1t_interfaces/msg/ArmStatus` | `d1t_feedback_node` → `d1t_arm` | 同上【代码】 | 每收到一帧状态帧一条 |
| `/d1t/command_ack` | `d1t_interfaces/msg/CommandAck` | `d1t_feedback_node` → `d1t_arm` | `RELIABLE` / `KEEP_LAST(20)` / `VOLATILE`【代码】 | 每条回执一条 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_bridge`（或 `joint_state_publisher_gui`）→ `robot_state_publisher`、`move_group` | 发布端 depth 10【代码】 | `source=feedback`：跟随反馈（仿真 9.99 Hz，c4/V1）；`source=static`：10 Hz【代码】 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` → RViz、`move_group`、`s4b2_demo` | ROS 默认 | 跟随 `/joint_states` |
| `/move_to_joints/_action/status` | `action_msgs/msg/GoalStatusArray` | `d1t_arm`（动作框架自动发）→ `d1t_trajectory`（仅 stream，用来发现别人的目标） | 动作状态默认 QoS【代码】 | 目标状态变化时 |

### C.2 字段

`/d1t/joint_feedback`（`JointFeedback`）

| 字段 | 类型 | 单位 | 含义与范围 |
|---|---|---|---|
| `seq` | uint32 | — | 帧序号。实机臂上写死为 10【厂商源码：marm_controller_node.cpp 关节帧】；仿真递增；缺失或越界填 0【代码】。**不能用来判断丢帧** |
| `angle0`~`angle5` | float64 | 度 | a0~a5 关节角，厂商原始值；分辨率 0.1°【实测：实机日志均为一位小数；arm_driver 注释「反馈 0.1° 量化」】；范围见 H.1 |
| `angle6_raw` | float64 | 原始值（**不是角度**） | 夹爪开度原始读数，增大 = 张开【实测 S3】；换算见 C.3 |

- 7 个字段缺一个或不是数字，这一帧整帧不发【代码：`handle_joints`】。
- 臂上读舵机超时时沿用上一次成功值：**一根掉线的轴在反馈里表现为「数不动」，不是报错**【厂商源码：`docs/onboard-source-survey.md` ·「Q7. 异常怎么处理？」】。

`/d1t/arm_status`（`ArmStatus`）——三个状态都**不可信**（START_HERE 2.3）

| 字段 | 类型 | 含义 |
|---|---|---|
| `seq` | uint32 | 实机写死 10【厂商源码】 |
| `enable_status` | int32 | 臂上通信节点的软件标志：开机 0；收到 funcode 1 / 2 / 7，或 funcode 4 / 5 且 `mode ≥ 1000` 置 1；funcode 4 / 5 且 `mode < 1000` 置 0【厂商源码：marm_communication_node.cpp】。臂在下沉时也会报 1【实测：START_HERE 2.3】 |
| `power_status` | int32 | 只在收到 funcode=6 时置 1，从不清零；与使能无关【厂商源码；实测：START_HERE 2.3】 |
| `error_status` | int32 | 永远是 0，源码从不赋值【厂商源码；START_HERE 2.3】 |

`/d1t/command_ack`（`CommandAck`）

| 字段 | 类型 | 含义 |
|---|---|---|
| `seq` | uint32 | 回执对应的命令 seq，原样带回；缺失时这条回执不发【代码】 |
| `ack_type` | uint8 | `1` = RECV（臂上通信节点收到并解析了 JSON）；`2` = EXEC（执行回执） |
| `status` | int32 | `1` 成功；`0` 失败（JSON 解析失败、不认识的 address / funcode）【厂商源码】 |

- **回执不证明运动。** recv=1 只证明 JSON 到了臂上通信节点；exec 是按算出的时间延时发的、不看反馈，不代表到位；funcode=1 只回 recv、不回 exec（START_HERE 2.3；厂商源码）。

`/joint_states`（`joint_state_bridge` 发）

| 字段 | 单位 | 算法【代码：joint_state_bridge.py】 |
|---|---|---|
| `name` | — | `Joint1`~`Joint6`、`Joint_L`、`Joint_R` |
| `position[0..5]` | rad | `angle0..5 × π/180 × joint_signs[i]`，`joint_signs` 默认 `[1, 1, 1, -1, 1, -1]`（J4、J6 取反） |
| `position[6]`、`position[7]` | m | 每爪 = `clamp(0.033 − 爪间距mm/2000, 0, 0.033)`；0 = 最大张开（URDF 约 66 mm），0.033 = 闭合【代码；实测 S3 FK】 |
| `velocity`、`effort` | — | 空 |

### C.3 夹爪 `angle6_raw` → 爪间距

**爪间距(mm) = clamp(0.85 × angle6_raw + 19.8, 0, 62)**【代码：joint_state_bridge.py `_publish`】

| 项 | 值 | 来源 |
|---|---|---|
| 系数 | 斜率 0.85 mm / 单位，截距 19.8 mm | 【代码】；Thomas 实测 6 组最小二乘拟合【实测：S3 阶段文档 ·「5.3 夹爪三层换算（`_publish` 内）」】 |
| 误差 | 标定区间内残差 < ±2 mm | 【实测 S3】 |
| 量程 | 0~62 mm（两端截断）；实机最大张开 62 mm，URDF 建模 66.3 mm | 【实测：START_HERE 档案 ·「实现进度」S3 条】 |
| 已标定区间 | `angle6_raw ≥ 0`（6 组）；负区间 (−30, 0) 只有 1 个点：完全闭合 raw ≈ −30 → 0 mm | 【实测 S3】；负区间中间开度【未验证】 |
| 参考点 | raw = 0 → 19.8 mm；raw = 49.8 → 62 mm（截断）；raw ≈ −23.3 → 0 mm（公式零点） | 【推断：代入公式】 |
| 特殊值 | raw = 9999.0 按闭合（0 mm）处理 | 【代码】；当前反馈节点缺字段时整帧丢弃、不会发 9999，这个分支实际不触发【代码】 |

命令侧（发给 a6 的值）：

- 控制器把 a6 目标截到 `[-20, 50]`【厂商源码 + 代码 `LIMIT_MIN/MAX`】。
- 命令 0 不闭合（残约 20 mm）；命令 ≥ 50 饱和在 62 mm【实测：START_HERE 档案 ·「实现进度」S3 挂起项】。
- 负值命令能不能驱动夹爪：同一条记录前后矛盾（一处写「命令通道不接受负值」，一处写「负方向也能运动」）【未验证】。
- 夹爪不参与到位判定；夹持力没测（START_HERE 2.3）。

---

## D. 服务

适用：S-aux4 之后，S4b-3 起未改｜核实：2026-09-17 源码（arm_driver.py、`d1t_interfaces/srv/*.srv`）+ 仿真 c1/S1~S5

三个服务都由 `arm_driver`（节点 `d1t_arm`）提供。

- `/set_servo`、`/set_stiffness` 与运动目标互斥：有目标在跑 → 回 `busy`；服务在跑时新运动目标被拒【代码：`_active`】（仿真核对：c5/X4）。`/set_override` 不互斥。
- **轨迹执行期间不调这三个服务**（START_HERE 2.3 ·「ROUTE-01 缺口的操作规避」）。stream 经由段里 `arm_driver` 并不知道轨迹在跑，互斥挡不住【代码】。

### D.1 `/set_override`（`d1t_interfaces/srv/SetOverride`）

| 方向 | 字段 | 类型 | 范围 | 含义 |
|---|---|---|---|---|
| 请求 | `ratio` | float64 | 0.01~1.0；越界截到边界，不报错 | 全局速度倍率：每个运动目标的规划总时长 ÷ ratio |
| 返回 | `success` | bool | 恒为 true | — |
| 返回 | `applied_ratio` | float64 | — | 截断后实际生效的值 |

- 节点级状态，一直保留到下次修改；`arm_driver` 重启回到 1.0【代码】。
- 只对**下一个**目标生效，正在跑的不重算【代码注释】。
- 生效范围：`/move_to_joints` 全部目标、`/set_servo` 回 HOME、point 模式全部段、stream 模式最后一点。**stream 经由点不受它影响**（固定按 1.0）【代码：trajectory_server `_stream_via`】。
- srv 文件注释「仅对 use_planner=true 的 goal 生效」已过时（该字段 2026-09-11 已删），以本表为准。
- 仿真核对：c1/S1（0.5 → 0.5；1.5 → 1.0）。

### D.2 `/set_servo`（`d1t_interfaces/srv/SetServo`）

| 方向 | 字段 | 类型 | 含义 |
|---|---|---|---|
| 请求 | `enable` | bool | `true` 上伺服；`false` 下伺服（卸力）。字段名是 `enable`，不是 `on`（YAML 会把 `on` 当布尔） |
| 返回 | `success` | bool | — |
| 返回 | `enabled` | bool | 结束时 `enable_status == 1`（软件标志，见 C.2） |
| 返回 | `final_deg` | float64[7] | 结束时最近一帧 angle0~5 + angle6_raw |
| 返回 | `reason` | string | 见下表 |

做什么【代码：arm_driver `_servo_on` / `_servo_off`】：

- **`enable: true`**：先等一帧新鲜反馈（≤ 0.5 s）；a0~a5 读数超出「实测硬件行程 ±3°」（H.1）→ `out_of_limits`，疑似传感器异常，不动。然后按是否在**折叠区**（`|a1 + 90| ≤ 15` 且 `|a2 − 90| ≤ 15`，只看 a1、a2）分两支：
  - 在折叠区：规划回精确 HOME `[0, -90, 89, 0, 0, 0, 0]`（a2 2026-09-23 前是 90，见 H.1），速度倍率 0.1。**a6 也会被送到 0，夹爪到约 20 mm 开口。** 到位超时 `max(20 s, 规划时长 × 1.5 + 2 s)`；到位后 `enable_status == 1` 才算成功。
  - 不在折叠区：原地使能——只给 a0~a5 发「当前读数」（截到控制器限位），不碰夹爪。截断量 > 2° **只告警不拒**（Thomas 2026-09-23 裁定），照发、臂会突跳这么多，`reason` 报 `ok_clamped_X.Xdeg`；`enable_status` 已是 1 → `already`；否则等 3 s 看它变 1。
  - 成功后日志打一行 `target_deg: [...]`，可直接粘进 `/move_to_joints` 目标。
- **`enable: false`**：折叠区内直接发 funcode=5 `mode=0`（卸力），等 0.5 s 看 `enable_status` 变 0。**折叠区外不再拒绝**（Thomas 2026-09-23 裁定）：先发 `mode=1000`（阻尼档，撑不住自重）让臂带阻尼下坠 2.0 s，再发 `mode=0`。**臂一定会跌落**，下方清空、手放断电处；2.0 s 够不够、坠落要多久无实机标定。

| reason | 什么情况 | 怎么办 |
|---|---|---|
| `ok` | 成功 | — |
| `already` | 已经是目标状态（折叠区外上伺服时已为 1；下伺服前后都是 0） | — |
| `ok_clamped_X.Xdeg` | 折叠区外上伺服成功，但控制器截了 X.X°、臂上电瞬间突跳了这么多 | 看臂有没有撞到东西 |
| `not_folded` | 2026-09-23 起 `/set_servo` 不再返回；`/set_stiffness` 低刚度在折叠区外仍返回它 | 下电改走 `/set_servo enable:false` |
| `out_of_limits` | 上伺服时读数超实测行程 ±3°（2026-09-23 起截断量 > 2° 不再触发） | 先查反馈是否异常；再用 `/move_to_joints` 走进控制器限位 |
| `timeout` | 折叠区内回 HOME 到位超时 | 看 `arm_driver` 日志 `arrival_timeout 逐轴偏差` |
| `move_failed` | 回 HOME 以其它原因失败；等不到 `enable_status` 变化 | 看 `arm_driver` 日志。**仿真里 `enable: false` 必然是它**（`fake_d1t` 不认 funcode=5） |
| `not_ready` | `/arm_Command` 没有收件方（控制器 / fake 没连上），或等不到新鲜反馈 | 先起反馈节点、确认臂在线 |
| `busy` | 有运动目标或另一个服务在跑 | 等它结束 |

危险提示：

- **卸力后臂持续下垂**（a4 数十秒内从 7.6° 沉到 0.7°）【实测：START_HERE 档案 ·「高频协议测试位姿」】；折叠区外卸力会砸下来。**2026-09-23 起服务不再挡折叠区外卸力**，只加 2 s 阻尼段。
- 折叠姿态（a1 = −90、a2 = 90）下 **a4 只能往负方向走**，正向会撞 a1 关节中心【实测 2026-09-09，START_HERE 2.3】。
- 仿真核对：c1/S2（折叠区内上伺服 `ok`）、c1/S4（仿真 `enable: false` → `move_failed`）、c1/S5（离开折叠区 `enable: false` → `not_folded`，**已失效**：2026-09-23 改为先阻尼再卸力，仿真见 TEACH-01 `logs/sim/run13`、`run14`）。

### D.3 `/set_stiffness`（`d1t_interfaces/srv/SetStiffness`）

| 方向 | 字段 | 类型 | 范围 | 含义 |
|---|---|---|---|---|
| 请求 | `mode` | int32 | 0~1000；< 0 或 > 1000 拒绝，不发命令 | funcode=5 全关节阻尼档位；0 = 完全卸力 |
| 返回 | `success` | bool | — | — |
| 返回 | `applied_mode` | int32 | — | 请求值原样回显（被拒也回显） |
| 返回 | `enabled` | bool | — | 发送 0.5 s 后 `enable_status == 1` |
| 返回 | `reason` | string | — | `ok` / `out_of_range` / `not_folded` / `not_ready` / `busy` |

- `mode < 1000` 视同卸力，走折叠区检查，不在折叠区 → `not_folded`；`mode = 1000` 照发并打 WARN「带力命令」【代码】。
- 发出后固定等 0.5 s 回 `ok`，**不检查臂的实际反应**【代码】。
- 有效量程 0~1000，1000 以上全部饱和；**满档 1000 也撑不住自重，不能用来保持位置**【实测 2026-09-11，START_HERE 2.3】。臂上字段是 uint16，> 65535 会回绕（80000 → 14464）【厂商源码】——服务在 1000 处已挡掉。只作诊断用。
- 仿真核对：c1/S3（2000 → `out_of_range`；0 → `ok`，fake 忽略）、c1/S5（折叠区外 0 → `not_folded`）。

---

## E. 动作

### E.1 `/move_to_joints`（`d1t_interfaces/action/MoveToJoints`，服务端 `d1t_arm`）

适用：S-aux4c 之后，S4b-3 加起动自救｜核实：2026-09-17 源码（arm_driver.py、trapezoid_planner.py、MoveToJoints.action）+ 仿真 c1/G1~G4、S5

**目标（goal）**

| 字段 | 类型 | 默认 | 单位 / 范围 | 含义；填错会怎样 |
|---|---|---|---|---|
| `target_deg` | float64[7] | 无 | 度；a0~a5 + a6 夹爪命令 | 必须 7 个数，顺序 a0~a6。**a6 也会发出去**：想保持夹爪不动，填当前 `angle6_raw`。任一轴超「实测硬件行程 ±3°」→ `out_of_limits`，一条命令都不发；在实测行程内但超控制器限位 → 截到控制器限位照走，结果里报截断量（限位见 H.1） |
| `timeout_sec` | float64 | 0.0 | 秒 | > 0：到位超时就是它（从首次发送算起）。≤ 0：`max(20, 规划时长 T × 1.5 + 5)` |
| `speed_ratio` | float64 | 0.3 | 0.01~1.0 | 各轴最高速度 = 114.59 °/s × 它；越界截断并置 `speed_limited` |
| `accel_ratio` | float64 | 1.0 | 0.01~1.0 | 规划加速度 = 161.1 °/s² × 它；越界同上。只改总时长，不传加减速形状（形状由控制器按 `delay_ms` 自算）【代码注释；实测 S-aux4c】 |
| `duration_sec` | float64 | 0.0 | 秒 | > 0：总时长取 `max(它, 按两个倍率算出的最短时长)`，比最短还短就拉长并置 `speed_limited`。想让下限等于硬件最快，`speed_ratio`、`accel_ratio` 都填 1.0。≤ 0：不指定 |

规划怎么算【代码：trapezoid_planner.py `plan`】：

- 起点 = 最近一帧反馈。7 轴（**含夹爪**）各算最短时间，取最长那根当全臂总时长 T，7 轴同时到。**夹爪要走的行程大，整条运动也跟着变慢**（仿真核对：c1/G1 行程最大的是 a6 的 49.8，`stroke_max=49.80deg`）。
- T 再除以 `/set_override` 倍率。
- 下发 `delay_ms = round(T × 1000 / 1.27)`，截到 [1, 32767]，再按控制器削顶门槛抬一道下限。
- 实际到位时间 ≈ `1.27 × delay_ms / 1000 + 0.23 s`【实测 S4a-1b：j0 40° 行程四点拟合，残差 < 15 ms】，也就是 ≈ T + 0.23 s【推断】。

**反馈（feedback）**：每来一帧关节反馈发一次（实机约 9 Hz）

| 字段 | 类型 | 含义 |
|---|---|---|
| `current_deg` | float64[7] | 最近一帧 angle0~5 + angle6_raw |
| `max_err_deg` | float64 | a0~a5 里「实际 − 目标」绝对值的最大值（目标是截断后的） |
| `phase` | string | `waiting_recv`（等 7 条 recv）/ `moving`（已收齐，等到位）/ `waiting_retry_recv`（起动自救重发后等 recv）。action 文件注释里的 `sending` 实际不会出现【代码】 |

**结果（result）**

| 字段 | 类型 | 含义 |
|---|---|---|
| `arrived` | bool | true = 到位 |
| `final_deg` | float64[7] | 结束时最近一帧反馈 |
| `reason` | string | 见下表 |
| `actual_duration_sec` | float64 | **规划**总时长 T（含 override），**不是实测耗时** |
| `speed_limited` | bool | 倍率被截、`duration_sec` 被拉长、`delay_ms` 撞 32767、或被削顶下限抬高，任一成立即 true |
| `executed_path` | string | `funcode1_planner` = 发过命令；`none` = 没发（被拒） |
| `target_clamp_deg` | float64[7] | 每轴截断量 = 下发值 − 目标值；没截为 0 |
| `target_clamped` | bool | 任一轴被截断 |

| reason | 动作状态 | 什么情况 | 发没发命令 |
|---|---|---|---|
| `arrived` | SUCCEEDED | a0~a5 全部在到位带内，连续 `stable_frames` 帧，且 7 条 recv 已收齐 | 发了 |
| `out_of_limits` | ABORTED | 目标超实测行程 ±3°（仿真核对：c1/G2） | 没发 |
| `not_ready` | ABORTED | `/arm_Command` 没有收件方；或最近一帧反馈超过 0.5 s | 没发 |
| `no_recv` | ABORTED | 每轮等 1 s，发了 `max_sends`（3）轮仍没收齐 recv | 发了 |
| `rejected_recv` | ABORTED | 任一条 recv `status = 0` | 发了 |
| `arrival_timeout` | ABORTED | 超时仍没到位；日志打逐轴偏差 | 发了 |
| `canceled` | CANCELED | 收到取消请求；**不发刹车命令，臂照走** | 发了 |
| （没有结果） | 终端打 `Goal was rejected.` | 另一个目标或 `/set_servo`、`/set_stiffness` 正在跑（仿真核对：c5/X4） | 没发 |

到位判据【代码：arm_driver `_on_feedback`】：

- 只判 a0~a5，**夹爪 a6 不参与**：夹住物体停在半途不会被判失败（START_HERE 2.3）。仿真核对：c1/G2 的 `final_deg` 里 a6 = 1.8，说明 G1 判到位时夹爪还在走。
- `tolerance_deg` 默认 3.0°（2026-10-27 Thomas 裁定，原 2.0°），`stable_frames` 默认 3 帧（实机约 0.33 s）。
- 7 条 recv 没收齐之前不判到位。
- 截断照走的例子：a0 = 140 → 截到 135 执行，`target_clamp_deg[0] = -5.0`、`target_clamped: true`（仿真核对：c1/G3）。

起动自救（S4b-3）【代码】：发令后 0.5 s 检查一次，按规划曲线此刻应走 ≥ 1.0°、实际离起点 < 0.3° 的轴，单独用新 seq 重发一次，日志 `STALL_AXIS`。之后不再重发，交给到位超时。已登记缺口：重发后缺 recv 时仍会走通用重发分支，与注释「不再重发第二次」不符（START_HERE ·「2.1a 路线核实 ROUTE-01 裁定（主窗口，2026-09-17）」）。

### E.2 `/follow_joint_trajectory`（`control_msgs/action/FollowJointTrajectory`，服务端 `d1t_trajectory`）

适用：S-ovl4 之后（point / stream），S4b-3 加 `PATH_DEV` 日志｜核实：2026-09-17 源码（trajectory_server.py、flyby.py）+ 仿真 c2/P1~P5、Q1~Q3，c3/M2、M6

**目标字段**（只列本实现用到或特殊处理的）

| 字段 | 本实现怎么用 |
|---|---|
| `trajectory.joint_names` | 只认 `Joint1`~`Joint6`，顺序随意（与 `positions` 一一对应）。含 `Joint_L` / `Joint_R` 或其它名字 → `INVALID_JOINTS`（−2）。**没列出的轴保持目标开始时的反馈值不动**；a6 夹爪始终保持开始时的值 |
| `trajectory.points[].positions` | 弧度，URDF 方向。转度数后 J4、J6 再乘 −1（`joint_signs`）→ a3、a5。长度必须等于 `joint_names` 长度，否则 `INVALID_GOAL`（−1） |
| `trajectory.points[].time_from_start` | 每段要求时长 = 相邻两点之差（第一点相对 0）。**全部为 0** → 每段用节点参数 `speed_ratio`（默认 0.3）算时长；否则每段按 `speed_ratio = 1.0` 的最短时长兜底，要求比它快就拉长 |
| `velocities` / `accelerations` / `effort` | 忽略 |
| `path_tolerance` / `component_path_tolerance` | **不实现**，收到只打日志。MoveIt 以为有路径偏差保护，实际没有（START_HERE 2.3） |
| `goal_tolerance` / `component_goal_tolerance` | 忽略；到位用 `arm_driver` 的 `tolerance_deg`（默认 3.0°，2026-10-27 起） |
| `goal_time_tolerance` | 低于 1 s 抬到 1 s，只打日志，不影响超时 |
| `multi_dof_trajectory` | 忽略 |

点数 > `max_points`（默认 64）→ `INVALID_GOAL`，一段都不执行，检查在抽稀之前（仿真核对：c2/P4「points 数 401 超过 max_points=400」；c3/M6 MoveIt 轨迹 127 点 > 64，MoveIt 报 −4 `CONTROL_FAILED`）。

**反馈**：`joint_names` = 目标里的名字；`actual` = 当前反馈（弧度，URDF 方向）；`desired` = 当前段目标；`error` = actual − desired。

**结果**

| error_code | 名字 | 什么情况 | 臂动没动 |
|---|---|---|---|
| 0 | `SUCCESSFUL` | 全部段完成，`error_string` 报段数与被拉长段数。**取消也回 0**：动作状态是 CANCELED，`error_string` 以「已取消」开头 | 动了 |
| −1 | `INVALID_GOAL` | 点为空；超 `max_points`；`positions` 长度不符；还没收到任何反馈；某段 `/move_to_joints` 回 `out_of_limits`；stream 开始时发现外来发布者 | 前四种和外来发布者：没动。`out_of_limits` 出在第 k 段时，**前 k 段已经走完** |
| −2 | `INVALID_JOINTS` | 关节名不认识（含夹爪） | 没动 |
| −5 | `GOAL_TOLERANCE_VIOLATED` | 某段 `/move_to_joints` 失败（`no_recv`、`arrival_timeout`、`not_ready`、`rejected_recv`）或被拒；stream 经由点发送前发现外来发布者或别人的 `/move_to_joints` 目标 | 可能停在半路 |

仿真核对：c2/P1（point 成功，`error_code: 0`）、c2/P2（Joint2 = 2.0 rad → −1）、c2/P3（含 `Joint_L` → −2）、c2/Q1（stream 成功）、c2/Q3（stream 唯一一点越限 → −1）。

**point 与 stream 的差别**

| 项 | point（默认） | stream |
|---|---|---|
| 每个点怎么走 | 都经 `/move_to_joints` 等到位 | 除最后一点外：直接发 funcode=1，本段动轴进入 `zone_deg` 就发下一点；最后一点经 `/move_to_joints` 等到位 |
| 抽稀 | 不抽稀 | 按 `thin_tol_deg` 把近似直线的点合并，只留拐点 |
| 每点停车开销 | 约 0.33~0.45 s / 段【实测：S-ovl4 报告 ·「4.3 V4 每段耗时与 STALL（执行器 SUMMARY 原文摘录）」】 | 经由点不停 |
| 越限检查 | 每段由 `arm_driver` 查实测行程，超了该段 `out_of_limits` → 中止 | **经由点不查实测行程**，只截到控制器限位照发；只有最后一点查（仿真核对：c2/Q2，经由点 a1 = 114.6° 被截到 90° 照走，结果 SUCCESS） |
| 经由点没走到 | — | 等兜底时间 `1.27 × delay_ms / 1000 + 0.5 s` 后照发下一点，只记日志（START_HERE 2.1a） |
| `/set_override` | 全部段生效 | 经由点不生效，最后一点生效 |
| 谁发 `/arm_Command` | 只有 `d1t_arm` | `d1t_trajectory` + `d1t_arm`；启动时有第三个发布者 → 进程退出码 1 |
| 取消 | 当前段走完、不发下一段（仿真核对：c2/P5） | 当前经由点进区域或兜底后返回，此时臂还在走向该经由点 |
| 最后一点超时 | `arm_driver` 默认公式（≥ 20 s） | 规划 T + 3 s；超时记 STALL、同目标重发一次，再失败才中止 |
| 什么时候用 | **靠近物体、障碍、下探抓取**（START_HERE 2.3 强制） | 只用于开阔区域的转运段 |
| 日志 / 落盘 | 每段 `SEG`，结尾 `SUMMARY` | 另有 `THIN`、`PATH_DEV`、`CORNER_SUMMARY`；反馈 CSV 写到 `csv_dir`（默认 `/tmp/sovl4_fb_<时间>.csv`） |

---

## F. 节点参数

适用：`main` `d772f46`｜核实：2026-09-17 源码（各节点 `declare_parameter`）+ 仿真 `ros2 param dump`（c1/C2、c2/P0、c2/Q0）

- 设参数：启动时加 `--ros-args -p 名字:=值`（A.3 有例子）。
- **所有参数只在节点启动时读一次**。运行中 `ros2 param set` 显示 `Set parameter successful`、`ros2 param get` 也读回新值（仿真核对：c5/X5），但节点内部仍用启动时的值【代码：各节点只在构造函数读参数，没有参数回调】。要改就 Ctrl-C 重起。
- 看当前值：`ros2 param dump /d1t_arm`、`ros2 param dump /d1t_trajectory`（J.1）。

### F.1 `arm_driver`（节点 `d1t_arm`）

| 参数 | 默认【代码】 | 代码校验 | 建议范围【推断，依据见右列】 | 含义 | 改了会怎样 |
|---|---|---|---|---|---|
| `tolerance_deg` | 3.0 | 无 | 1.3~3.0 | a0~a5 到位带（度）。`trajectory_server`、MoveIt 执行的到位也用它 | 调小：某些位姿静差达 1.3°【实测：START_HERE 档案 ·「高频协议测试位姿」】，判不到位 → `arrival_timeout`。**不放到 3.0 以上**：到位带同时兜底「根本没走到」，太松会把失败报成成功（START_HERE 2.3） |
| `stable_frames` | 3 | 无 | ≥ 1 | 连续几帧在带内算到位 | 调大到位判定变慢（实机每帧约 0.111 s）；1 帧容易被单帧噪声骗【推断】 |
| `default_timeout_sec` | 20.0 | 无 | > 0 | 目标没给 `timeout_sec` 时的超时下限，实际用 `max(它, T × 1.5 + 5)` | 调小：慢速长行程可能误报 `arrival_timeout` |
| `recv_timeout_sec` | 1.0 | 无 | > 0 | 每轮等 recv 的时间 | 调小：网络抖一下就多发一轮 |
| `max_sends` | 3 | 无 | ≥ 1 | 缺 recv 时最多发几轮（含首发），每轮新 seq | 1：缺一条 recv 就 `no_recv` |
| `feedback_stale_sec` | 0.5 | 无 | > 0.111 | 规划起点反馈允许的最大年龄 | 太小：正常 0.111 s 间隔加丢帧洞（0.41~0.61 s）会误报 `not_ready`；太大：反馈节点停了还拿旧位置规划（历史事故「规划起点算错 → 满速冲」，START_HERE 档案 ·「实现进度」） |
| `funcode1_gap_ms` | 2.0 | 无 | ≥ 0 | 一轮 7 条命令之间的间隔（毫秒） | 0：臂上读端只留 1 条，中间帧可能被顶掉，靠缺 recv 重发兜底【代码注释】。模块头注释写默认 0.0，以代码 2.0 为准 |
| `startup_retry_enabled` | true | 无 | true / false | 起动自救开关（E.1） | false：「回执了但轴不动」只能等到位超时 |
| `startup_check_sec` | 0.5 | ≤ 0 启动报错退出 | — | 发令后多久检查一次 | 太早：规划位移还小，多数轴不查；太晚：浪费时间 |
| `startup_expected_deg` | 1.0 | 同上 | — | 规划此刻应走 ≥ 它的轴才检查 | — |
| `startup_stall_deg` | 0.3 | 同上 | 高于反馈 0.1° 量化 | 实际离起点 < 它 = 没动 | 与 `startup_expected_deg` 取一样大时，慢起步被误判过卡死（S4b-3 阶段文档 ·「7. 踩过的坑」坑 3） |

不是参数、改要改代码重编的常量【代码】：

| 常量 | 值 | 文件 | 含义 |
|---|---|---|---|
| `V_MAX` | 114.59 °/s | trapezoid_planner.py | 规划最高速度 = 控制器削顶门槛 `maxomega` |
| `A_MAX` | 71.62 °/s² | 同上 | 控制器削顶门槛 `maxalpha`，只用来算下限 |
| `A_PLAN` | ≈ 161.1 °/s² | 同上 | 规划曲线用的加速度 = 4·A_MAX / (CLAMP_MARGIN·COMP)² |
| `COMP` | 1.27 | 同上 | 实际到位时间 / delay_ms【实测 S4a-1b；推导 1.318，差 3.6% 未解】 |
| `CLAMP_MARGIN` | 1.05 | 同上 | delay_ms 比削顶门槛多留 5%；1.00 是硬底线（START_HERE 1.3） |
| `DELAY_MS_MIN` / `DELAY_MS_MAX` | 1 / 32767 | 同上 | 臂上 delay_ms 是 int16【厂商源码：SetServoAngle_.hpp】 |
| `RATIO_MIN` | 0.01 | 同上 | 三个倍率的下限 |
| `HW_MARGIN` | 3.0° | arm_driver.py | 实测硬件行程外放宽量 |
| `FOLD_TOL` | 15.0° | arm_driver.py | 折叠区半宽 |
| `ENABLE_CLAMP_TOL` | 2.0° | arm_driver.py | 原地使能允许的最大截断量 |
| `STIFFNESS_MODE_MAX` / `HOLD_MODE_MIN` | 1000 / 1000 | arm_driver.py | 阻尼量程上限 / 视为「带力」的下限 |
| `HOME_DEG` | (0, −90, 90, 0, 0, 0, 0) | arm_driver.py | `/set_servo` 回 HOME 的目标 |

### F.2 `trajectory_server`（节点 `d1t_trajectory`）

| 参数 | 默认【代码】 | 代码校验 | 含义 | 改了会怎样 |
|---|---|---|---|---|
| `execution_mode` | `point` | 只能 `point` / `stream`，否则启动报错 | 执行方式（E.2 表） | 按 START_HERE 2.3：靠近物体、障碍、下探抓取用 point |
| `max_points` | 64 | 无 | 一条轨迹最多几个点，超了整条拒绝 | **MoveIt 轨迹常超 64**：缩放 0.1 时 193 点【实测 S4b-2，START_HERE 档案 ·「MoveIt 上线路线（2026-09-16 重排；原「冲刺计划/明天给领导演示」是主窗口自己加的期限，Thomas 没下过，已作废）」】。接 MoveIt 一律 `max_points:=400`（S4b-3 实机用值）。这是一道绊线，不是硬件限制 |
| `speed_ratio` | 0.3 | 无 | 轨迹 `time_from_start` 全为 0 时每段用的速度倍率 | 只影响时间戳全零的手写轨迹；MoveIt 轨迹不受影响 |
| `zone_deg` | 2.0 | 必须 > 0，否则启动报错 | stream：本段动轴全部进入「目标 ± 它」就发下一点 | 大：切角多、偏离 MoveIt 检查过的路径更多；小：更接近逐点停车。靠近障碍 `2.0`（START_HERE 2.3），S4b-3 实机连跑用 2.0 |
| `thin_tol_deg` | 1.0 | 无（≤ 0 = 不抽稀） | stream：被合并的点离保留弦线的最大单轴偏差 | 大：点更少、偏离更多；0：每个点都是经由点 |
| `joint_signs` | [1, 1, 1, −1, 1, −1] | 无 | Joint1..6 → a0..a5 的方向符号 | **必须与 `joint_state_bridge` 的 `joint_signs` 一致**，否则 J4 / J6 反转、MoveIt 永远判不到位【代码注释】 |
| `csv_dir` | `/tmp` | 无（空字符串 = 不写） | stream 每条轨迹的反馈 CSV 目录 | `/tmp` 每次开机清空 |
| `discovery_wait_sec` | 2.0 | 无 | stream 启动时等 DDS 发现多久再查外来发布者 | 太短可能漏查 |

常量【代码：flyby.py / trajectory_server.py】：按名义路点判动轴阈值 0.5°；兜底余量 0.5 s（兜底 = 1.27 × delay_ms / 1000 + 0.5 s）；STALL 判轴 1.0°；最后一点超时余量 3.0 s；`goal_time_tolerance` 下限 1.0 s；经由点各轴命令间隔 2 ms；允许的 `/arm_Command` 发布者 `d1t_trajectory`、`d1t_arm`。

### F.3 `d1t_feedback_node`

无参数【代码】。日志限流 2 s 是常量。

### F.4 `joint_state_bridge`

| 参数 | 默认【代码】 | 范围 | 含义 |
|---|---|---|---|
| `joint_signs` | [1.0, 1.0, 1.0, −1.0, 1.0, −1.0] | 6 个 ±1 | J4、J6 取反（S3 实测实机方向与 URDF 相反）；与 `trajectory_server` 保持一致 |
| `source` | `feedback` | `feedback` / `static`（其它值按 feedback 处理） | `static`：不订阅反馈，10 Hz 发 `static_angles` |
| `static_angles` | [0, 0, 0, 0, 0, 0] | 6 个度数；长度不对截断或补 0 并警告；不校验范围 | 只在 `static` 下用；夹爪固定显示约 20 mm 开口 |

launch 参数：`view_live.launch.py` 有 `source`（默认 `feedback`）、`static_angles`（默认 `"[0,0,0,0,0,0]"`）；`d1t_moveit.launch.py` 固定 `source=feedback`，没有 launch 参数；`view_gui.launch.py` 没有参数【代码】。

### F.5 `fake_d1t`（仿真）

| 参数 | 默认【代码】 | 含义 |
|---|---|---|
| `drop_rate` | 0.0 | 0~1，整条命令按概率丢弃（不回 recv） |
| `speed_deg_per_s` | 11.5 | 各轴独立匀速；**不按 delay_ms 走**。本手册仿真命令用 60.0（默认太慢，stream 最后一点会超时误报；S-ovl4 同） |
| `start_delay_sec` | 0.3 | 收到命令到开始动 |
| `initial_angles` | [0.0, −90.4, 89.6, −0.3, 8.9, −0.3, 49.8] | 启动姿态（度） |
| `stall_axes` / `stall_axes_freeze` | 未设 | 故障注入，测试用 |

行为【代码】：只认 funcode=1；funcode=5 等其它一律 WARN 忽略；收到 funcode=1 就把 `enable_status` 置 1，永不回 0；只回 recv，不回 exec。

### F.6 `s4b2_demo`

| 参数 | 默认【代码】 | 含义 |
|---|---|---|
| `velocity_scaling` | 0.1 | MoveIt `max_velocity_scaling_factor` |
| `acceleration_scaling` | 0.1 | MoveIt `max_acceleration_scaling_factor` |

笛卡尔上抬段的轨迹超过 10 点会被脚本等间隔抽到 10 点【代码】。

### F.7 `set_desk_collision`

无参数；尺寸与 30 mm 余量是常量（I.2）【代码】。

---

## G. 底层协议

适用：臂上源码本地副本（臂上实际运行的二进制是否与副本一致【未验证】，START_HERE 2.4）｜核实：2026-09-17 读厂商源码 `marm_communication_node.cpp`（收 JSON）、`marm_controller_node.cpp`（管串口）

### G.1 两根线

- `/arm_Command`：我们 → 臂（臂上 DDS 名 `rt/arm_Command`）。
- `/arm_Feedback`：臂 → 我们（`rt/arm_Feedback`）。
- 类型都是 `unitree_arm/msg/ArmString`，只有一个字段 `string data`，里面装 JSON【代码 + 厂商源码】。
- 命令 JSON：`{"seq": N, "address": 1, "funcode": F, "data": {...}}`。

### G.2 funcode 全表（address = 1）【厂商源码：`subArmCommand_callback`】

| funcode | data 字段 | 臂上做什么 | 回执 | `enable_status` | 我方现状 |
|---|---|---|---|---|---|
| 1 | `id`（0~6）、`angle`（度）、`delay_ms`（整数） | 单关节转到 `angle`（G.4） | 只回 recv=1，**不回 exec** | 置 1 | **唯一运动通路**（`arm_driver`、stream 经由点） |
| 2 | `angle0`~`angle6`、`mode`（0 / 1） | 7 轴同时；mode 0 的 delay_ms 固定 40，mode 1 按 200×Δ/3 算 | recv=1；在回调线程里等到算出的时间再发 exec=1 | 置 1 | **2026-09-11 退役**（S-aux4），我方不再发 |
| 3 | `pose_x/y/z`、`roll/pitch/yaw` | **空壳**：只解析，不动（换算代码被注释掉） | 只回 recv=1 | 不变 | 不用 |
| 4 | `id`、`mode` | 单关节阻尼 = `mode` | recv=1 + 立即 exec=1 | `mode < 1000` 置 0，否则置 1 | 不用 |
| 5 | `mode` | 7 个关节阻尼 = `mode`（塞进 uint16，> 65535 回绕） | recv=1 + 立即 exec=1 | 同上 | `/set_stiffness`、`/set_servo enable:false` 用；有效量程 0~1000【实测】 |
| 6 | `power` | **空壳**：只打印，置 `power_status = 1` | recv=1 + 立即 exec=1 | 不变 | 不用 |
| 7 | 无 | 7 轴回角度 0（**不是 HOME**），delay_ms 固定 6000；**回调线程等 6 s，这期间收不了新命令** | recv=1 + 另一进程立即一帧 exec + 6 s 后再一帧 exec | 置 1 | 不用 |
| 其它 | — | 不执行 | recv=0 + exec=0 | 不变 | — |

JSON 解析失败时回 recv=0（seq 填 10）【厂商源码】。

### G.3 反馈帧（`/arm_Feedback`）【厂商源码】

| address | funcode | data | 谁发 | 频率 |
|---|---|---|---|---|
| 2 | 1 | `angle0`~`angle6` | 串口进程：每轮先睡 100 ms，再串行查 7 个舵机 | 约 8.98 Hz【实测】 |
| 2 | 3 | `enable_status`、`power_status`、`error_status` | 通信进程 100 ms 定时 | 约 10 Hz【未验证】 |
| 3 | 1 | `recv_status` | 命令回执 | 按命令 |
| 3 | 2 | `exec_status` | 执行回执 | 按命令 |

关节帧、状态帧的 seq 臂上都写死 10。

### G.4 funcode=1 在臂上怎么执行【厂商源码：marm_controller_node.cpp `subServoAngle_callback`】

1. `delay_ms = 0` 当 1。
2. 目标截到控制器限位（G.5）。
3. 当前读数不在 [−180, 180]（读数离谱）→ 不动，改发 `setDamping(500)`（该轴变软）。
4. `omega = |Δ| / delay_ms × 1000`，截到 114.59 °/s；`alpha = omega / delay_ms × 1000`，截到 71.62 °/s²。
5. 算实际时间 `testdelay_ms = ((√(ω² + 2α|Δ|) − ω) / α) × 1800`，调 `setRawAngle(angle, testdelay_ms)`。
6. 不发 exec，不看是否到位。

`delay_ms` 与实际到位时间：没撞截断时行程约掉，实际 ≈ `1.27 × delay_ms / 1000 + 0.23 s`【实测 S4a-1b：j0 40° 行程、delay_ms 1600~3600 四点拟合，残差 < 15 ms；推导值 1.318，差 3.6% 未解】。撞了 omega / alpha 截断后，delay_ms 再小也不会更快，规划器用 `CLAMP_MARGIN` 留 5% 避开【代码】。

### G.5 控制器限位表【厂商源码 `maxangle` / `minangle`；代码 `trapezoid_planner.LIMIT_MIN/MAX` 同值】

| 轴 | a0 | a1 | a2 | a3 | a4 | a5 | a6 |
|---|---|---|---|---|---|---|---|
| 下限（度） | −163 | −90.3 | −97 | −166 | −96 | −166 | −20 |
| 上限（度） | 163 | 97 | 89 | 163 | 113 | 162 | 64.5 |

**2026-09-23 起为 Thomas 放宽后的值**（TEACH-01 报告 ·「16. 关节 / 夹爪限位放宽到 Thomas 实测行程」；臂上 `marm_controller_node.cpp` 由 Thomas 本人修改重编，步骤与回滚见 `forlearning/TEACH-01-web-jog/ARM-LIMITS-checklist.md`）。原厂值 ±135 / ±90 / ±90 / ±135 / ±90 / ±135、a6 −20~50。**刷机或换臂后臂上会回到原厂值、ubox 侧不会报错**：症状是界面放行、臂停在 135° 不动，复验方法见该清单第 9 步。

超出的目标**截到边界照走，不报错**。

### G.6 手发 `/arm_Command`

本手册不提供手发命令：

- `arm_driver` 在跑时手发 = 两个发布者，控制器每收一条就丢旧目标重新规划，臂走到哪不可预测【代码注释：trajectory_server 入口独占】；stream 模式下会被判外来发布者中止。
- 确实要发（`arm_driver` 没在跑）：`ros2 topic pub --once` 对 `BEST_EFFORT` 读端发不进去，要用 `-w 1 -r 1 -t 4`【实测：S0 阶段文档 ·「7. 踩过的坑」坑 1；`d1t_env_sim.bash` 提示】。

### G.7 臂上已知问题（臂上源码只读，改不改由 Thomas 定）

- 串口命令回调与定时查询共用一个协议对象、无锁 → 混帧竞争在源码层面成立；是不是「回执了但不动」的根因【未验证】（START_HERE 2.1a、2.2）。
- 零剩余行程时 `omega = alpha = 0`，时间公式 0/0，未见运行后果（START_HERE 2.1a）。
- 舵机超时沿用旧值、校验错帧照收、`error_status` 恒 0、堵转不上报【厂商源码：`docs/onboard-source-survey.md` ·「Q7. 异常怎么处理？」】。

---

## H. 限位与单位换算总表

适用：`main` `d772f46`｜核实：2026-09-17 源码（trapezoid_planner.py、arm_driver.py、d1t.urdf、joint_limits.yaml）

### H.1 逐轴表

| 控制器轴 | URDF 关节 | 方向符号 | 控制器限位（截断）【厂商源码】 | 实测硬件行程【代码 `HW_LIMIT`】 | `arm_driver` 拒绝线（实测行程 ±3°）【代码】 | URDF 限位【代码】 | URDF 速度【代码】 | MoveIt 速度 / 加速度【代码】 | HOME【代码】 |
|---|---|---|---|---|---|---|---|---|---|
| a0 | Joint1 | +1 | ±163 | −164.0 ~ 165.0 | < −167.0 或 > 168.0 | ±163.2° | 1.05 rad/s = 60.2 °/s | 1.05 rad/s / 1.25 rad/s² | 0 |
| a1 | Joint2 | +1 | −90.3 ~ 97 | −90.4 ~ 98.0 | < −93.4 或 > 101.0 | −90.5° ~ 97.2° | 60.2 °/s | 1.05 / 1.25 | −90 |
| a2 | Joint3 | +1 | −97 ~ 89 | −96.5 ~ 89.6 | < −99.5 或 > 92.6 | −97.2° ~ 89.2° | 60.2 °/s | 1.05 / 1.25 | **89**（原 90） |
| a3 | Joint4 | **−1** | −166 ~ 163 | −166.0 ~ 163.0 | < −169.0 或 > 166.0 | Joint4 −163.2° ~ 166.2°（符号反） | 1.73 rad/s = 99.1 °/s | 1.73 / 1.25 | 0 |
| a4 | Joint5 | +1 | −96 ~ 113 | −97.3 ~ 113.0 | < −100.3 或 > 116.0 | −96.2° ~ 113.2° | 99.1 °/s | 1.73 / 1.25 | 0 |
| a5 | Joint6 | **−1** | −166 ~ 162 | −167.2 ~ 162.9 | < −170.2 或 > 165.9 | Joint6 −162.2° ~ 166.2°（符号反） | 99.1 °/s | 1.73 / 1.25 | 0 |
| a6 | 夹爪（URDF `Joint_L` / `Joint_R` 各 0~0.033 m） | — | −20 ~ 64.5（`/grip` 目标 20~64.5） | −30.4 ~ 64.5 | < −33.4 或 > 67.5 | 0~0.033 m | 0.02 m/s | 不在规划组 | 0 |

- **控制器限位、URDF 限位、HOME a2 三列是 2026-09-23 TEACH-01 合并后的值**（main 合并 `afb701b`）：Thomas 按实测行程放宽，URDF = 控制器限位外扩 0.2°；a2 上界取 89、比实测行程 89.6 紧 0.6°，HOME 与「折叠姿态」预设跟着从 90 改 89。夹爪 raw>50 对应的 mm 是线性外推、无标定点。
- 实测硬件行程：Thomas 2026-09-10 手动测量，一次采样、有几度不确定度（trapezoid_planner.py 注释）。
- 三层判定【代码：arm_driver `_run`】：超拒绝线 → 拒绝 `out_of_limits`；拒绝线内、超控制器限位 → 截到控制器限位照走，结果报截断量；控制器限位内 → 原样。
- HOME 实际读数 a1 约 −90.2 ~ −90.5，贴着或低于 URDF 下限（放宽前 −89.95°，现 −90.5°）；MoveIt 靠 `FixStartStateBounds` 钳回（`start_state_max_bounds_error` 0.1 rad）【实测：START_HERE 档案 ·「S4b-0 MoveIt 配置与仿真全链路（2026-09-15 Thomas 裁定现在开，任务书已出）」里「主窗口开工前核实的事实（2026-09-15）」；代码 ompl_planning.yaml】。

### H.2 三套速度、加速度

| 谁 | 速度上限 | 加速度 |
|---|---|---|
| 控制器削顶门槛 | 114.59 °/s | 71.62 °/s²【厂商源码】 |
| `arm_driver` 规划 | 114.59 × `speed_ratio` °/s | 161.1 × `accel_ratio` °/s²【代码】 |
| MoveIt 规划 | J1~J3 60.2 °/s、J4~J6 99.1 °/s，× 速度缩放 | 71.6 °/s² × 加速度缩放【代码：joint_limits.yaml】 |

MoveIt 算出的时间只决定每段「要求时长」；实际执行时长由 `arm_driver` 规划取两者较大的那个【代码】。

### H.3 到位带与判据

| 层 | 值 | 来源 |
|---|---|---|
| `arm_driver` 到位带 | a0~a5 ≤ 3.0°，连续 3 帧（2026-10-27 起） | 【代码】 |
| stream 经由点区域 | 动轴 ≤ `zone_deg`（默认 2.0，2026-10-27 起） | 【代码】；START_HERE 2.3 |
| 臂自身静差 | 0.06° ~ 1.3°，与位姿强相关 | 【实测：START_HERE 档案 ·「高频协议测试位姿」；arm_driver 注释】 |
| 2° 到位带在指尖约多少 | 约 14 ~ 17 mm | 【推断：S4b-3 阶段文档 ·「5. 程序结构与实现方法」】 |
| MoveIt 关节目标容差 | ±0.01 rad（`s4b2_demo` 与本手册 J.5 命令） | 【代码】 |

### H.4 单位与方向换算

| 从 | 到 | 公式 |
|---|---|---|
| URDF 弧度 Joint1..6 | 控制器度 a0..a5 | `a = rad × 180/π × joint_signs`（J4、J6 乘 −1）【代码】 |
| 控制器度 a0..a5 | `/joint_states` 弧度 | `rad = a × π/180 × joint_signs`【代码】 |
| `angle6_raw` | 爪间距 mm | `clamp(0.85 × raw + 19.8, 0, 62)`【代码；实测 S3】 |
| 爪间距 mm | URDF 每爪 m | `clamp(0.033 − mm/2000, 0, 0.033)`【代码】 |
| 规划总时长 T | `delay_ms` | `round(T × 1000 / 1.27)`，再截断、抬下限【代码】 |
| `delay_ms` | 实际到位秒 | ≈ `1.27 × delay_ms / 1000 + 0.23`【实测 S4a-1b】 |
| 度 | 弧度 | 20° = 0.3491 rad；90° = 1.5708 rad |

方向约定：

- J4、J6 实机转向与 URDF `<axis>` 相反【实测：S3 阶段文档 ·「7. 踩过的坑」坑 1】。
- a0 正方向为左转；折叠姿态下 +40° 净空充足【实测 2026-09-09：START_HERE 档案 ·「高频协议测试位姿」】。
- 折叠姿态 a4 正方向朝下，正向会撞 a1 关节中心【实测 2026-09-09】。**同一关节角在不同构型下扫过的空间不同，位姿安全性不能跨构型套用**（START_HERE 2.3）。

### H.5 命名位姿【记录：START_HERE 档案 ·「高频协议测试位姿」】

| 名字 | a0~a6 | 备注 |
|---|---|---|
| HOME / 安全下电位置 | [0, −90, 90, 0, 0, 0, 0] | 接近自然掉电姿态；`/set_servo` 回的也是它 |
| 原点 | [0, 0, 0, 0, 0, 0, 0] | funcode=7 回的是它，不是 HOME |
| 动作准备位置 | [0, 0, 0, 0, 90, 0, 65] | a6 = 65 超控制器上限 50，会被截到 50【推断：按 G.5】 |

---

## I. MoveIt 配置速查

适用：S4b-0 起；S4b-3 改 `longest_valid_segment_fraction` 与碰撞余量｜核实：2026-09-17 源码（`d1t_moveit_config/config/*`、launch）+ 仿真 c3

### I.1 配置

| 项 | 值【代码】 | 文件 |
|---|---|---|
| 规划组 | `arm`：链 `base_link` → `Empty_Link6`，Joint1~Joint6；夹爪不建组 | d1t.srdf |
| 末端 link | `Empty_Link6`（`s4b2_demo` 的笛卡尔直线也用它） | d1t.srdf、s4b2_demo.py |
| 命名状态 | `home` = [0, −1.5708, 1.5708, 0, 0, 0] rad | d1t.srdf |
| 关掉的碰撞检查 | 只关物理相邻的 8 对连杆 | d1t.srdf |
| robot name | `d1_550_description`（URDF 没改名，SRDF 跟着它） | d1t.srdf |
| 逆解（IK） | KDL，`search_resolution` 0.005，`timeout` 0.005 s，`attempts` 3 | kinematics.yaml |
| 规划器 | OMPL，只挂 RRTConnect；`longest_valid_segment_fraction` 0.001（S4b-3 由 0.005 改） | ompl_planning.yaml |
| 请求适配器 | AddTimeOptimalParameterization、ResolveConstraintFrames、FixWorkspaceBounds、FixStartStateBounds、FixStartStateCollision、FixStartStatePathConstraints；`start_state_max_bounds_error` 0.1 rad；`jiggle_fraction` 0.05 | ompl_planning.yaml |
| 执行器 | `moveit_simple_controller_manager`，控制器名 `follow_joint_trajectory` → `/follow_joint_trajectory` | moveit_controllers.yaml |
| 执行时长监控 | 关（`execution_duration_monitoring: false`） | moveit_controllers.yaml |
| 规划管线 | 只加载 `ompl`（不带 Pilz、CHOMP） | d1t_moveit.launch.py |
| 速度 / 加速度限位 | 见 H.1 | joint_limits.yaml |

### I.2 碰撞体（`set_desk_collision`，`base_link` 坐标系，单位米）

尺寸【代码：set_desk_collision.py；Thomas 2026-09-16 实测】；外扩 `SAFETY_MARGIN` = 30 mm：四周与顶面各扩，底面不动（S4b-3 裁定）。

| 名字 | 实测范围 / 尺寸 | 实测顶面 z | 进 MoveIt 的碰撞盒（外扩后） |
|---|---|---|---|
| `desk` 桌面 | x −0.200 ~ 1.210，y −0.110 ~ 0.700（范围外是悬空） | −0.020 | x −0.230 ~ 1.240，y −0.140 ~ 0.730，顶面 0.010，往下厚 0.510 |
| `desk_clamp` 木工夹 | 中心 (0.220, −0.110)，X 20 × Y 90 × 高 70 mm | 0.050 | X 80 × Y 150 × 高 100 mm，顶面 0.080 |
| `mount_board` 木板 | 中心 (0.028, 0.010)，X 440 × Y 220 × 厚 20 mm | 0.000 | X 500 × Y 280 × 厚 50 mm，顶面 0.030 |

仿真核对：c3/M1 打印与本表一致。碰撞体只存在 `move_group` 内存里，**重起 MoveIt launch 后要重跑 `set_desk_collision`**【推断：脚本只调一次 `/apply_planning_scene`】。

### I.3 规则、实测效果与限制

| 项 | 内容 | 来源 |
|---|---|---|
| 目标点离障碍 | 名义 ≥ 45 mm（30 mm 余量 + 到位误差上限） | START_HERE 2.3（S4b-3 裁定） |
| 靠近障碍的运动 | `zone_deg:=2.0`；并按 ROUTE-01 规避一律 `execution_mode:=point` | START_HERE 2.3 |
| 失败线 | 逐帧最近距离 < 15 mm 即该遍失败并停跑（距离由测试工具算，不在本手册） | START_HERE 2.3 |
| 速度缩放的实际效果 | 0.1：193 点、65.5 s；0.3：70 点、24.5 s；0.5：46 点、19.7 s（point 模式）。**三档抖动程度一样**：缩放只改点数和总时长，不消除抖动 | 【实测 S4b-2：START_HERE 档案 ·「MoveIt 上线路线（2026-09-16 重排；原「冲刺计划/明天给领导演示」是主窗口自己加的期限，Thomas 没下过，已作废）」】 |
| point 与 stream 耗时 | 同三个目标：point 64.0 s，stream 20.4 s（各跑一遍） | 【实测：S-ovl4 报告 ·「4.1 V2 每次移动耗时（demo 脚本打印，单位 s）」】 |
| 实际路径偏离规划 | 已测最大 1.75°（S4b-3 绕障，stream） | 【实测：START_HERE 1.3】 |
| 连跑可靠性 | T1 / T2 / T3 60/60 首次成功；绕行首次 19/20、重试后 20/20（缩放 0.3，stream，zone 2.0） | 【实测 S4b-3】 |
| 不实现 `path_tolerance` | MoveIt 以为有路径偏差保护，实际没有；抽稀和区域切换会让实际路径偏离 MoveIt 检查过的路径；开阔区域以外的通用避障可靠性【未验证】 | START_HERE 2.3 |
| 臂停在碰撞盒里 | MoveIt 拒绝一切规划（日志 `Start state appears to be in collision`）；没有自动退出动作，只能手发 `/move_to_joints`（不经碰撞检查） | START_HERE 2.2；S4b-3 阶段文档坑 2 |
| 结果里 `planning_time` | 「规划 + 执行」模式下恒为 0，不代表没花时间 | 【实测：S4b-3 阶段文档坑 7】；仿真核对 c3/M2 |
| MoveGroup 常见 error_code | 1 SUCCESS；99999 FAILURE（如目标在碰撞里，c3/M4）；−1 PLANNING_FAILED；−2 INVALID_MOTION_PLAN；−4 CONTROL_FAILED（执行器拒绝或失败，c3/M6）；−10 START_STATE_IN_COLLISION；−12 GOAL_IN_COLLISION | 【代码：moveit_msgs/msg/MoveItErrorCodes】 |

---

## J. 常用命令集

适用：S4b-3 之后｜核实：2026-09-17 仿真（每条标日志位置）

- 每个终端先按 A.3.1 加载环境；在 worktree 里按 A.2 两行加载。
- 节点按 A.3 的顺序起好再发命令。
- 命令里的 YAML（花括号那一串）不能插注释，参数说明都写在命令上方。

### J.1 看状态（不动臂）

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/C1
# --once → 收到一帧就退出；打印 seq、angle0~angle5（度）、angle6_raw（夹爪原始值）
ros2 topic echo --once /d1t/joint_feedback
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/C1
# enable_status / power_status / error_status 都不可信（C.2）
ros2 topic echo --once /d1t/arm_status
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/C1、c5/X2（约 10.0 Hz）
# 实机正常约 8.98 Hz；Ctrl-C 结束
ros2 topic hz /d1t/joint_feedback
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c5/X1、c2/P0、c2/Q0
# -v → 列出每个发布者、订阅者的节点名和 QoS。point 模式发布者只应有 d1t_arm；stream 模式是 d1t_trajectory + d1t_arm
ros2 topic info -v /arm_Command
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c2/P0、c3/M0
# -t → 同时显示动作类型。点到点组合应有 /move_to_joints；MoveIt 全栈另有 /follow_joint_trajectory、/move_action、/execute_trajectory
ros2 action list -t
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/C2
# 打印 arm_driver 当前全部参数（F.1）
ros2 param dump /d1t_arm
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c2/P0、c2/Q0
# 打印 trajectory_server 当前全部参数（F.2），先确认 execution_mode、zone_deg、max_points
ros2 param dump /d1t_trajectory
```

### J.2 点到点 `/move_to_joints`（组合 ②）

回 HOME：

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/G1（SUCCEEDED，reason: arrived）
# 路径不经过碰撞检查：从远处或障碍附近回来，先自己看路径
# /move_to_joints → 动作名；d1t_interfaces/action/MoveToJoints → 动作类型
# target_deg → a0~a6 七个目标（度）。最后一个 0.0 是夹爪命令 → 夹爪到约 20 mm 开口；夹着东西时改成 J.1 读到的 angle6_raw
# timeout_sec: 0.0 → 超时 = max(20 s, 规划时长 × 1.5 + 5 s)
# speed_ratio: 0.3 → 各轴最高速度的 30%（0.01~1.0）
# accel_ratio: 1.0 → 规划加速度的 100%（0.01~1.0）
# duration_sec: 0.0 → 不指定总时长，由两个倍率算
# --feedback → 过程中打印 current_deg / max_err_deg / phase。结果看 reason 和最后一行 Goal finished with status
ros2 action send_goal /move_to_joints d1t_interfaces/action/MoveToJoints "{target_deg: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0], timeout_sec: 0.0, speed_ratio: 0.3, accel_ratio: 1.0, duration_sec: 0.0}" --feedback
```

越限示例（应被拒，臂不动）：

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/G2（ABORTED，reason: out_of_limits，executed_path: none）
# a1 = 120.0 超过 a1 拒绝线 101.0°（实测行程 98.0 + 3），一条命令都不发；其余字段同上
ros2 action send_goal /move_to_joints d1t_interfaces/action/MoveToJoints "{target_deg: [0.0, 120.0, 90.0, 0.0, 0.0, 0.0, 0.0], timeout_sec: 0.0, speed_ratio: 0.3, accel_ratio: 1.0, duration_sec: 0.0}"
```

取消：在 `send_goal` 那个终端按 Ctrl-C，终端打印 `Canceling goal...`、`Goal canceled.`。**不是急停，已发出的目标臂照走**（A.0；仿真核对：c5/X3）。

### J.3 服务（组合 ②，轨迹执行期间不要调）

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/S1（applied_ratio=0.5）
# ratio: 0.5 → 之后每个运动目标的总时长 × 2（0.01~1.0，越界截断）；stream 经由点不受影响
ros2 service call /set_override d1t_interfaces/srv/SetOverride "{ratio: 0.5}"
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/S1
# ratio: 1.0 → 恢复全速倍率
ros2 service call /set_override d1t_interfaces/srv/SetOverride "{ratio: 1.0}"
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/S2（success=True，reason='ok'）
# enable: true → 上伺服。折叠区内：慢速回精确 HOME，夹爪 a6 也会被送到 0；折叠区外：原地使能，不碰夹爪（D.2）
ros2 service call /set_servo d1t_interfaces/srv/SetServo "{enable: true}"
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c1/S4（仿真必回 move_failed：fake 不认卸力命令）、c1/S5（折叠区外 → not_folded）
# enable: false → 卸力。只在折叠区（a1≈-90、a2≈90，±15°）内执行；卸力后臂会持续下垂
ros2 service call /set_servo d1t_interfaces/srv/SetServo "{enable: false}"
```

```bash
# 适用：实机 / 仿真（诊断用）｜核实：2026-09-17 仿真 c1/S3（ok；仿真里 fake 忽略）、c1/S5（折叠区外 → not_folded）
# mode: 0 → 全关节阻尼 0 = 完全卸力，效果同上一条，但返回时不检查臂的反应；有效量程 0~1000，满档也撑不住自重
ros2 service call /set_stiffness d1t_interfaces/srv/SetStiffness "{mode: 0}"
```

### J.4 轨迹 `/follow_joint_trajectory`（组合 ③，手写轨迹不经过 MoveIt 碰撞检查）

point 示例（从 HOME 起 a0 转到 +20° 再回 0）：

```bash
# 适用：实机 / 仿真，trajectory_server 以 point 起（A.3.6 第一条）｜核实：2026-09-17 仿真 c2/P1（error_code: 0，SUCCEEDED）
# 只动 Joint1（= a0）；没列出的轴保持当前值不动，夹爪不动
# joint_names: [Joint1] → 本轨迹只给 Joint1
# points[0]: positions [0.3491] → 0.3491 rad = +20°；time_from_start 2 s → 第 1 段要求 2 s 走完（比硬件最快还快会被拉长）
# points[1]: positions [0.0] → 回 0°；time_from_start 4 s → 第 2 段要求 4 − 2 = 2 s
# --feedback → 打印 actual / desired / error（弧度）
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [Joint1], points: [{positions: [0.3491], time_from_start: {sec: 2, nanosec: 0}}, {positions: [0.0], time_from_start: {sec: 4, nanosec: 0}}]}}" --feedback
```

stream 示例：命令与上一条完全相同，区别只在 `trajectory_server` 以 stream 起（A.3.6 第二条）。第 1 点是经由点（不停车），第 2 点是最后一点（等到位）。只用于开阔区域。
核实：2026-09-17 仿真 c2/Q1（`SEG 0/2 stream-via ... entered=True`，`SEG 1/2 stream-final ... reason=arrived`，SUCCEEDED）。

越限示例（应被拒）：

```bash
# 适用：实机 / 仿真，trajectory_server 以 point 起｜核实：2026-09-17 仿真 c2/P2（error_code: -1，ABORTED，reason=out_of_limits）
# joint_names: [Joint2] → 只给 Joint2（= a1）
# positions [2.0] → 2.0 rad = 114.6°，超过 a1 拒绝线 101.0° → 第 1 段 out_of_limits，一条命令都不发
# time_from_start 3 s → 要求 3 s（这里用不到）
# 注意：同样的越限点放在 stream 的经由点位置不会被拒（K.2）
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [Joint2], points: [{positions: [2.0], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

取消：在 `send_goal` 终端 Ctrl-C。当前段走完才停（仿真核对：c2/P5）。

### J.5 用命令行让 MoveIt 规划并执行（组合 ③ + 碰撞体已加）

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c3/M2（val: 1 = SUCCESS，SUCCEEDED）
# /move_action → MoveIt 的「规划 + 执行」动作；输出很长（含整条规划轨迹），只看末尾 error_code 的 val 和 Goal finished
# group_name: arm → 规划组
# num_planning_attempts: 5 → 最多规划 5 次；allowed_planning_time: 5.0 → 每次最多 5 s
# max_velocity_scaling_factor: 0.3 / max_acceleration_scaling_factor: 0.3 → 速度、加速度缩放 30%（S4b-3 实机连跑用值）
# joint_constraints → 6 个关节目标（弧度，URDF 方向），tolerance_above/below 0.01 → 允许 ±0.01 rad
#   本例：Joint1 = 0.3491（a0 = +20°），Joint2 = -1.5708、Joint3 = 1.5708（折叠），Joint4~6 = 0
# planning_options: {plan_only: false} → 规划完直接执行（经 /follow_joint_trajectory）
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: arm, num_planning_attempts: 5, allowed_planning_time: 5.0, max_velocity_scaling_factor: 0.3, max_acceleration_scaling_factor: 0.3, goal_constraints: [{joint_constraints: [{joint_name: Joint1, position: 0.3491, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint2, position: -1.5708, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint3, position: 1.5708, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint4, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint5, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint6, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}]}]}, planning_options: {plan_only: false}}"
```

```bash
# 适用：实机 / 仿真｜核实：2026-09-17 仿真 c3/M3（val: 1，SUCCEEDED）
# 经 MoveIt（带碰撞检查）回 HOME：参数同上，只是 Joint1 = 0.0
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: arm, num_planning_attempts: 5, allowed_planning_time: 5.0, max_velocity_scaling_factor: 0.3, max_acceleration_scaling_factor: 0.3, goal_constraints: [{joint_constraints: [{joint_name: Joint1, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint2, position: -1.5708, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint3, position: 1.5708, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint4, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint5, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}, {joint_name: Joint6, position: 0.0, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}]}]}, planning_options: {plan_only: false}}"
```

- 臂停在碰撞盒里时这两条都会被拒（I.3），只能手发 J.2。
- 目标点离障碍名义 ≥ 45 mm（I.3）。

### J.6 演示 `s4b2_demo`

```bash
# 【未验证：仿真失败，原因见 docs/OPS-01-report.md】仿真 c3/M4：回HOME_起步 SUCCESS；关节空间目标A error_code=99999(FAILURE)，
#   move_group 日志「desk 与 Empty_Link_L 接触」——目标 A (-70°, 20°, 60°) 是 S4b-2 在零余量碰撞体下定的，
#   在现在外扩 30 mm 的碰撞体里判碰撞（S4b-3 的测试脚本 s4b3_run10 已把 A 点改成 (-70°, 13°, 54°)，本脚本没跟着改）。主窗口裁定前不要在实机上跑
# 适用：组合 ③ + 碰撞体已加；失败即停，不继续后面的步骤
# 无参数 = velocity_scaling 0.1、acceleration_scaling 0.1（F.6）
ros2 run d1t_moveit_config s4b2_demo
```

### J.7 看 3D（RViz）

```bash
# 适用：实机 / 仿真（需反馈节点在跑）｜核实：2026-09-17 仿真 c4/T3、V1——robot_state_publisher、joint_state_bridge 正常，/joint_states 9.99 Hz；
#   本会话无显示器，RViz 窗口本身【未验证】（rviz2 退出码 -6）
# 无参数 = source:=feedback，RViz 里的模型跟随实际反馈
ros2 launch d1t_description view_live.launch.py
```

```bash
# 适用：不需要臂和反馈｜核实：2026-09-17 仿真 c4/V2（/joint_states 里 Joint5 = 1.5708）；RViz 窗口【未验证】
# source:=static → 不读反馈，10 Hz 发固定姿态
# static_angles:="[0,0,0,0,90,0]" → a0~a5 六个度数（这里 a4 = 90°），只预览臂姿态，不控制臂
ros2 launch d1t_description view_live.launch.py source:=static static_angles:="[0,0,0,0,90,0]"
```

```bash
# 适用：不需要臂｜核实：2026-09-17 仿真 c4/T5——本会话无显示器，滑块窗口和 RViz 都起不来（退出码 -6），robot_state_publisher 正常；窗口本身【未验证】
# 无参数。滑块手动摆姿态，只影响 RViz 显示，不控制臂
ros2 launch d1t_description view_gui.launch.py
```

实机在 ubox 屏幕上看时，先在同一终端 `export DISPLAY=:0`（A.3.5）。

### J.8 下电

按 A.4：J.2 回 HOME → J.3 `/set_servo enable:false` → 逆序 Ctrl-C → 断电。

---

## K. 已知坑与安全须知

适用：2026-09-17 现状｜核实：每行「来源」列

### K.1 必须遵守：ROUTE-01 缺口的操作规避（START_HERE 2.3，EXEC-01 暂缓期间强制，主窗口 2026-09-17）

| 现象 | 原因 | 怎么办 |
|---|---|---|
| stream 下经由点没走到，轨迹照样往下发 | `trajectory_server._stream_via` 等兜底超时只记日志、照常返回（源码核对） | **靠近物体、障碍、下探抓取的轨迹一律 `execution_mode:=point`**：point 模式不抽稀，每点经 `/move_to_joints` 等到位，任一段没到位即中止。stream 只用于开阔区域的转运段 |
| 轨迹执行中调服务或手发目标，臂行为不可预测 | `trajectory_server` 与 `arm_driver` 两个节点没有统一的「运动权」 | **轨迹执行期间不调 `/set_servo`、`/set_stiffness`、`/set_override`，不手发 `/move_to_joints`** |
| 夹住物体停在半途，结果仍是「到位」 | `arm_driver` 只判 angle0~5，夹爪 a6 不参与到位判定 | 夹取是否成功另看 `angle6_raw` 和人眼；夹持力大小未测 |

### K.2 命令与参数

| 现象 | 原因 | 怎么办 | 来源 |
|---|---|---|---|
| 按了 Ctrl-C 或取消，臂还在动 | 取消只是「不发下一段」，已发出的目标臂照走 | 唯一急停是断电，手放在电源上 | START_HERE 2.3；仿真核对 c2/P5、c5/X3 |
| 回 HOME 时夹着的东西掉了 | HOME 目标 a6 = 0（夹爪命令 0 ≈ 20 mm 开口）；`/set_servo enable:true` 在折叠区内也把 a6 送到 0 | 夹着东西回 HOME 用 J.2，把 a6 改成当前 `angle6_raw`；不要调 `/set_servo` | 【代码：J.2 命令、arm_driver `HOME_DEG`】 |
| 夹爪动作大时整条运动变慢 | 规划时 7 轴（含夹爪）取最慢的一根定总时长 | 夹爪和手臂大动作分两个目标发 | 【代码：trapezoid_planner `plan`】 |
| MoveIt 规划成功但执行失败，`error_code` −4，臂不动 | `trajectory_server` 默认 `max_points` 64，MoveIt 轨迹常超（缩放 0.1 时 193 点） | 起 `trajectory_server` 一律带 `-p max_points:=400` | 仿真核对 c3/M6（127 点 > 64）；【实测 S4b-2】 |
| stream 手写轨迹的经由点越限，没被拒，臂走到控制器限位 | stream 经由点只截到控制器限位，不查实测行程；只有最后一点查 | 手写 stream 轨迹自己先按 H.1 查限位；MoveIt 轨迹在 URDF 限位内，不会触发【推断】 | 【代码：`_stream_via` 只调 `clamp_to_ctrl`】；仿真核对 c2/Q2 |
| stream 下改了 `/set_override`，经由段速度没变 | stream 经由点规划固定按倍率 1.0 | 用 stream 时不依赖 `/set_override` 调速；要慢就降 MoveIt 缩放 | 【代码】；S-ovl4 报告 ·「6. 偏离清单」#6 |
| `ros2 param set` 显示成功，行为没变 | 参数只在启动时读一次 | Ctrl-C 后带 `--ros-args -p` 重起节点 | 【代码】；仿真核对 c5/X5 |
| `tolerance_deg` 放宽后失败被报成「到位」 | 到位带同时兜底「根本没走到」 | 不放到 3.0° 以上 | START_HERE 2.3 |
| 结果 `actual_duration_sec` 和实际耗时对不上 | 它是**规划**时长 T，实际约 T + 0.23 s，加上等 recv、等 3 帧 | 实际耗时看终端时间戳或日志 | 【代码】；【实测 S4a-1b】 |
| 命令全回执（recv=1），个别轴完全不动 | 根因未查（已累计 3 次）；候选「臂上命令与反馈查询共用串口、无互斥」【未验证】 | 起动自救会单轴重发一次（日志 `STALL_AXIS`）；仍不动则 20 s 后 `arrival_timeout`，同一命令重发过能走 | START_HERE 2.2；S-ovl4 报告 ·「5.2 恢复回 HOME：第 1 次「7 条都回执，a1/a2 整整 20 s 不动」（实测）」 |
| `enable_status = 1`，臂却在往下沉；`error_status` 永远 0；exec 回执来了臂没到 | 这几个都是臂上的软件标志或按时间算的回执，不看实际状态 | 到位只看 `/d1t/joint_feedback` | START_HERE 2.3；【厂商源码】 |
| 反馈突然停 0.4~0.6 s，之后第一帧数值是旧的 | 7 轴齐发后臂上反馈查询超时（候选机理：命令与查询在串口上撞包【未验证】） | 不要用单帧判断「没动」；看连续几帧 | 【实测：START_HERE 2.3】 |
| 卸力后臂下沉；折叠区外卸力会砸 | 阻尼只加阻力不驱动，满档 1000 也撑不住自重 | 只在折叠区卸力（服务已挡）；不用 `/set_stiffness` 保持位置 | 【实测 2026-09-09 / 09-11，START_HERE 2.3】 |
| 折叠姿态下 a4 正向运动撞到 a1 关节中心 | 折叠构型下 a4 正方向朝下 | 折叠姿态 a4 只往负方向走；位姿安全不跨构型套用 | 【实测 2026-09-09，START_HERE 2.3】 |
| `ros2 topic pub --once /arm_Command ...` 没反应 | 读端 `BEST_EFFORT`，单发的第一条容易丢 | 本来就不该手发（G.6）；确需时用 `-w 1 -r 1 -t 4` | 【实测：S0 阶段文档坑 1】 |
| 臂上 `autoStart*.sh` 手动跑了一下，臂掉下来 | 脚本会重启臂上控制程序 | 臂上源码只读：不改、不 sudo、不手动跑 `autoStart*.sh` | START_HERE 2.3 |

### K.3 MoveIt

| 现象 | 原因 | 怎么办 | 来源 |
|---|---|---|---|
| 连续 `INVALID_MOTION_PLAN` / 规划全被拒，日志 `Start state appears to be in collision` | 臂当前停在外扩 30 mm 的碰撞盒里（常见于目标点离障碍 < 45 mm，停位偏了零点几度） | 手发 J.2 把臂挪出盒子（不经碰撞检查，自己看路径）；目标点名义距离 ≥ 45 mm | START_HERE 2.2、2.3；S4b-3 阶段文档坑 2 |
| `s4b2_demo` 在「关节空间目标A」失败，`error_code=99999` | 脚本里的 A 点 (−70°, 20°, 60°) 是零余量时代定的，在外扩 30 mm 的桌面盒里判碰撞 | 主窗口裁定前不在实机上跑；需要演示先改目标点（改代码，由主窗口出任务） | 仿真核对 c3/M4 与 c3/T4 日志；S4b-3 阶段文档 ·「5. 程序结构与实现方法」第 2 轮 |
| 规划、执行都成功，臂却蹭到障碍 | 随机规划器贴边走 + 执行偏差；抽稀、区域切换让实际路径偏离规划 | 碰撞盒外扩 30 mm（`set_desk_collision` 已带）；靠近障碍 point + `zone_deg:=2.0`；开阔区域外的避障可靠性未验证 | S4b-3 阶段文档坑 1；START_HERE 2.3 |
| 重起 MoveIt 后规划穿过桌面 | 碰撞体只在 `move_group` 内存里 | 每次起 MoveIt launch 后跑 `set_desk_collision`，确认 `success=True` | 【推断】 |
| 调速度缩放后抖动不变 | 缩放只改点数与总时长 | 不靠缩放改善流畅；开阔段用 stream | 【实测 S4b-2】 |
| 结果里 `planning_time` 总是 0 | 「规划 + 执行」模式下不回填 | 规划耗时看 `move_group` 日志时间戳 | 【实测：S4b-3 阶段文档坑 7】 |

### K.4 环境、仿真与停机

| 现象 | 原因 | 怎么办 | 来源 |
|---|---|---|---|
| worktree 里改了代码，跑起来还是老行为，不报错 | 环境脚本写死加载主仓库 install | 两行加载（A.2），用 `ros2 pkg prefix d1t_arm` 确认 | START_HERE 2.3；仿真核对 c4/E2 |
| 实机看不到 `/arm_Feedback` | `enp5s0` 没插网线（DOWN），或代理 Meta TUN 劫持了 `192.168.123.0/24` 路由 | 看环境脚本打印的 `operstate`；关代理 TUN；`ping 192.168.123.100` 应约 1 ms、0% 丢包 | `.claude/CLAUDE.md` §1 |
| `d1t_sim_check` 打出 ✗ | 域 7 上已有 `/arm_Feedback`：别人在跑仿真，或隔离出问题 | 停下，不启动 `fake_d1t`；先 `pgrep -af fake_d1t` 看是谁 | 【代码：d1t_env_sim.bash】 |
| 仿真里 `/set_servo enable:false` 总是 `move_failed` | `fake_d1t` 不认 funcode=5，`enable_status` 永远是 1 | 仿真里不验卸力 | 【代码】；仿真核对 c1/S4 |
| 仿真 stream 最后一点超时 / STALL | `fake_d1t` 默认 11.5 °/s 太慢，跟不上「规划时长 + 3 s」 | 仿真起 `fake_d1t` 带 `-p speed_deg_per_s:=60.0` | S-ovl4 报告 ·「3.2 仿真 MoveIt 三目标（日志 `logs/sim/moveit_*`）」 |
| 仿真里反馈路径和规划路径差很多 | `fake_d1t` 各轴独立匀速，不按比例同步 | 仿真只验链路和结果码，路径、耗时、距离看实机 | S4b-3 阶段文档坑 6 |
| Ctrl-C 停 `trajectory_server` 时打一段 Traceback，`Process exited with failure 1` | 程序只接住 `KeyboardInterrupt`，没接住 `ExternalShutdownException` | 外观问题，不影响臂（退出时不发命令）；已登记代码缺陷 | 仿真核对 c2/T4、T5；【代码：trajectory_server `main`】 |
| Ctrl-C 停 MoveIt launch 时 `move_group` 报 `exit code -11` | MoveIt 进程退出时崩溃（原因未查） | 已停机时出现，不影响臂 | 仿真核对 c3/T4（1 次）【未验证：实机是否也出现】 |
| RViz、滑块窗口起不来，`Couldn't open X display`、退出码 -6 | 终端没有图形显示 | 实机在 ubox 屏幕看：先 `export DISPLAY=:0` | 仿真核对 c3/T4、c4/T3、T5 |
| MoveIt 日志 `Start state appears to be in collision` 后又 `Found a valid state near the start state` | `FixStartStateCollision` 把起点挪到附近的合法状态再规划；仿真 `fake_d1t` 初始姿态（a4 = 8.9°）判 Link1–Link6 接触，起点被挪 0.26 | 规划起点不等于实际起点；实机出现时先把臂挪到明确合法的姿态 | 仿真核对 c3/T4（1 次）【未验证：实机】 |
