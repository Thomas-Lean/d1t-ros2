"""view_gui：滑块手动摆姿态。

robot_state_publisher + joint_state_publisher_gui + rviz2。
joint_state_publisher_gui 的滑块发布 /joint_states，robot_state_publisher 据此发布 tf，
RViz 的 RobotModel 按 tf 摆网格。不连机械臂、不订阅反馈。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('d1t_description')
    urdf = os.path.join(pkg, 'urdf', 'd1t.urdf')
    rviz = os.path.join(pkg, 'rviz', 'd1t.rviz')
    with open(urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                robot_description, value_type=str)}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz],
            output='screen',
        ),
    ])
