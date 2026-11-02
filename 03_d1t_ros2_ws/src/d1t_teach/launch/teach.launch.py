"""teach.launch.py — TEACH-01：只拉 teach_server 一个节点。

**不拉** fake_d1t / d1t_feedback_node / arm_driver / move_group：那几个的启动时机和参数
各自要单独控制（与 d1t_moveit.launch.py 同一边界）。trajectory_server 由 teach_server
自己作为子进程拉起（固定 execution_mode:=stream），所以也不在这里拉，
避免同一个动作名起两份。

用法（仿真，域 7）：
  ros2 launch d1t_teach teach.launch.py
  ros2 launch d1t_teach teach.launch.py bind_host:=127.0.0.1 bind_port:=8175
从别的机器打开界面：不要改 bind_host 对外开放，用 SSH 端口转发
  ssh -N -L 8175:127.0.0.1:8175 ubox
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bind_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('bind_port', default_value='8175'),
        DeclareLaunchArgument('vel_scaling', default_value='0.3'),
        DeclareLaunchArgument('acc_scaling', default_value='0.3'),
        DeclareLaunchArgument('manage_trajectory_server', default_value='true'),
    ]
    node = Node(
        package='d1t_teach',
        executable='teach_server',
        output='screen',
        parameters=[{
            'bind_host': LaunchConfiguration('bind_host'),
            'bind_port': LaunchConfiguration('bind_port'),
            'vel_scaling': LaunchConfiguration('vel_scaling'),
            'acc_scaling': LaunchConfiguration('acc_scaling'),
            'manage_trajectory_server': LaunchConfiguration('manage_trajectory_server'),
        }],
    )
    return LaunchDescription(args + [node])
