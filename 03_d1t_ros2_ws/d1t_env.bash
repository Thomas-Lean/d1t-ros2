#!/usr/bin/env bash
# d1t_env.bash — D1-T ROS2 直连环境统一入口
# 用法：  source <仓库根>/03_d1t_ros2_ws/d1t_env.bash

# --- 0. 必须被 source，不能直接执行 ---
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
  echo "[d1t_env] 请用 source 执行： source ${0}" >&2
  exit 1
fi

# --- 1. 常量 ---
D1T_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
D1T_DDS_CONF="$(dirname "${D1T_WS}")/cyclonedds.xml"
D1T_IFACE="${D1T_IFACE:-enp5s0}"
D1T_ARM_IP="192.168.123.100"

# --- 2. ROS2 Humble ---
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[d1t_env] 未找到 /opt/ros/humble/setup.bash，中止" >&2
  return 1
fi
source /opt/ros/humble/setup.bash

# --- 3. 工作区 overlay（未 build 时不报错，仅提示） ---
if [ -f "${D1T_WS}/install/setup.bash" ]; then
  source "${D1T_WS}/install/setup.bash"
  D1T_WS_STATE="overlay 已加载"
else
  D1T_WS_STATE="overlay 不存在（尚未 colcon build）；ros2 topic echo 无法解析 unitree_arm 自定义类型"
fi

# --- 4. CycloneDDS 直连 ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
if [ -f "${D1T_DDS_CONF}" ]; then
  export CYCLONEDDS_URI="file://${D1T_DDS_CONF}"
  D1T_DDS_STATE="${D1T_DDS_CONF}"
else
  unset CYCLONEDDS_URI
  D1T_DDS_STATE="未找到 cyclonedds.xml，Cyclone 自动选网卡（可能被 Meta TUN 劫持）"
fi

# --- 5. 只读自检（不修改系统） ---
IFACE_STATE="$(cat /sys/class/net/${D1T_IFACE}/operstate 2>/dev/null || echo 'not-found')"

echo "==================== d1t_env ===================="
echo " ROS_DISTRO        : ${ROS_DISTRO:-<未设置>}"
echo " 工作区            : ${D1T_WS}"
echo "                    ${D1T_WS_STATE}"
echo " RMW_IMPLEMENTATION : ${RMW_IMPLEMENTATION}"
echo " ROS_DOMAIN_ID      : ${ROS_DOMAIN_ID}"
echo " ROS_LOCALHOST_ONLY : ${ROS_LOCALHOST_ONLY}"
echo " CYCLONEDDS_URI     : ${CYCLONEDDS_URI:-<未设置>}"
echo "                    ${D1T_DDS_STATE}"
echo " 网卡 ${D1T_IFACE}    : operstate=${IFACE_STATE}"
if [ "${IFACE_STATE}" != "up" ]; then
  echo "   ⚠ 网卡非 up：未插网线为 down；代理 Meta TUN 会劫持 192.168.123.0/24"
fi
