"""view_live：跟随真实/仿真反馈，或静态预览目标姿态。

robot_state_publisher + joint_state_bridge + rviz2。
source:=feedback（默认）时 bridge 订阅 /d1t/joint_feedback；source:=static 时 bridge 以
10 Hz 发布 static_angles。bridge 输出 /joint_states，robot_state_publisher 据此发布 tf，
RViz 的 RobotModel 按 tf 摆网格。

参数：
  source          feedback | static
  static_angles   六个度数，例 static_angles:="[0,0,0,0,90,0]"
"""

import ast
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn(context):
    pkg = get_package_share_directory('d1t_description')
    with open(os.path.join(pkg, 'urdf', 'd1t.urdf'), 'r') as f:
        robot_description = f.read()
    rviz = os.path.join(pkg, 'rviz', 'd1t.rviz')
    source = LaunchConfiguration('source').perform(context)
    static_angles = [float(x) for x in
                     ast.literal_eval(LaunchConfiguration('static_angles').perform(context))]

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                robot_description, value_type=str)}],
        ),
        Node(
            package='d1t_description',
            executable='joint_state_bridge',
            output='screen',
            parameters=[{'source': source, 'static_angles': static_angles}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='feedback'),
        DeclareLaunchArgument(
            'static_angles', default_value='[0,0,0,0,0,0]',
            description='六个度数，例 static_angles:="[0,0,0,0,90,0]"'),
        OpaqueFunction(function=_spawn),
    ])
