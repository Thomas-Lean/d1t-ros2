"""d1t_moveit.launch.py — S4b-0：仿真全链路里 MoveIt 那一半。

拉起 robot_state_publisher + joint_state_bridge(source:=feedback) + move_group +
RViz(MotionPlanning)。不拉 fake_d1t / d1t_feedback_node / arm_driver /
trajectory_server——那四个由测试脚本起（任务书边界，这几个节点各自的启动时机、
参数需要单独控制，不适合捆在这一个 launch 里）。

配置来源：手写的 config/*.yaml + config/d1t.srdf，用 moveit_configs_utils 的
MoveItConfigsBuilder 拼装——这是官方提供的 Python 装配器，不是 Setup Assistant
图形界面，用它只是省得自己重新实现"读 yaml 拼 move_group 参数字典"这一层。
Pilz 本阶段不配：pipelines=["ompl"] 显式收窄，否则 builder 默认 load_all=True
会把 moveit_configs_utils 自带 default_configs 目录下的 chomp/pilz 也一起塞进来。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    d1t_description_share = Path(get_package_share_directory('d1t_description'))
    urdf_path = d1t_description_share / 'urdf' / 'd1t.urdf'
    rviz_path = str(Path(get_package_share_directory('d1t_moveit_config'))
                     / 'rviz' / 'd1t_moveit.rviz')

    moveit_config = (
        MoveItConfigsBuilder('d1t', package_name='d1t_moveit_config')
        .robot_description(file_path=str(urdf_path))
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # source 固定 feedback：本 launch 只服务「仿真/实机反馈驱动 RViz」这一种用法，
    # source:=static 那种预览模式走 d1t_description 自己的 view_live.launch.py。
    joint_state_bridge_node = Node(
        package='d1t_description',
        executable='joint_state_bridge',
        output='screen',
        parameters=[{'source': 'feedback'}],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_bridge_node,
        move_group_node,
        rviz_node,
    ])
