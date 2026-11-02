#!/usr/bin/env bash
# d1t_env_sim.bash — D1-T 仿真（fake_d1t）环境入口，与 d1t_env.bash 互斥
# 用法：  source <仓库根>/03_d1t_ros2_ws/d1t_env_sim.bash
# 与实机入口的三处差异：DDS 走 loopback、ROS_DOMAIN_ID=7、不检查 enp5s0。
# 本入口不连机械臂；跑实机请开新终端 source d1t_env.bash，不要在同一终端切换。

# --- 0. 必须被 source，不能直接执行 ---
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
  echo "[d1t_env_sim] 请用 source 执行： source ${0}" >&2
  exit 1
fi

# --- 1. 常量 ---
D1T_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
D1T_SIM_DDS_CONF="$(dirname "${D1T_WS}")/cyclonedds_sim.xml"
D1T_SIM_DOMAIN="7"

# --- 2. ROS2 Humble ---
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[d1t_env_sim] 未找到 /opt/ros/humble/setup.bash，中止" >&2
  return 1
fi
source /opt/ros/humble/setup.bash

# --- 3. 工作区 overlay（未 build 时不报错，仅提示） ---
if [ -f "${D1T_WS}/install/setup.bash" ]; then
  source "${D1T_WS}/install/setup.bash"
  D1T_WS_STATE="overlay 已加载"
else
  D1T_WS_STATE="overlay 不存在（尚未 colcon build）；ros2 run d1t_sim fake_d1t 无法启动"
fi

# --- 4. CycloneDDS loopback ---
# 配置缺失时直接中止：Cyclone 会自动选网卡，可能把仿真流量发到实机所在网段。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
if [ ! -f "${D1T_SIM_DDS_CONF}" ]; then
  echo "[d1t_env_sim] 未找到 ${D1T_SIM_DDS_CONF}，中止（缺配置会导致仿真流量外泄）" >&2
  return 1
fi
export CYCLONEDDS_URI="file://${D1T_SIM_DDS_CONF}"
export ROS_DOMAIN_ID="${D1T_SIM_DOMAIN}"

# --- 5. 隔离自检（手动调用，需要起 ros2 daemon，不在 source 时自动跑） ---
# 用法：d1t_sim_check —— 看到 /arm_Feedback 说明没隔离干净，必须停下排查。
d1t_sim_check() {
  echo "[d1t_env_sim] domain=${ROS_DOMAIN_ID} 上的话题："
  local topics
  # --no-daemon：绕开 ros2 daemon 的话题缓存，避免拿到上一次会话的残留结果
  topics="$(timeout 15 ros2 topic list --no-daemon 2>/dev/null)"
  echo "${topics}"
  if echo "${topics}" | grep -q '^/arm_Feedback$'; then
    echo "  ✗ 看到 /arm_Feedback：未隔离干净，停止，不要启动 fake_d1t" >&2
    return 1
  fi
  echo "  ✓ 无 /arm_Feedback，可以启动 fake_d1t"
}

echo "================== d1t_env_sim =================="
echo " 模式               : 仿真（fake_d1t），不连真实机械臂"
echo " ROS_DISTRO         : ${ROS_DISTRO:-<未设置>}"
echo " 工作区             : ${D1T_WS}"
echo "                     ${D1T_WS_STATE}"
echo " RMW_IMPLEMENTATION : ${RMW_IMPLEMENTATION}"
echo " ROS_DOMAIN_ID      : ${ROS_DOMAIN_ID}   （实机为 0，本入口固定 7）"
echo " ROS_LOCALHOST_ONLY : ${ROS_LOCALHOST_ONLY}"
echo " CYCLONEDDS_URI     : ${CYCLONEDDS_URI}"
echo "                     127.0.0.1 单播，组播关闭，不经 enp5s0"
echo " 启动前自检         : 运行 d1t_sim_check"
echo " 启动仿真器         : ros2 run d1t_sim fake_d1t"
echo " 已知坑             : /arm_Command 订阅端是 BEST_EFFORT，匹配后首条命令会丢；"
echo "                     ros2 topic pub 用 -w 1 -r 1 -t 4，不要用 --once"
