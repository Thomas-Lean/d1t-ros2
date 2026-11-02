"""GRASP-01 owned fake stack; launch only through grasp01_session.py occupancy gate."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from grasp01_core import guard_environment


def generate_launch_description():
    guard_environment()
    if not os.environ.get('GRASP01_TOKEN'):
        raise RuntimeError('use grasp01_session.py; missing ownership token')
    description=Path(get_package_share_directory('d1t_description'))
    pkg=Path(get_package_share_directory('d1t_moveit_config'))
    cfg=(MoveItConfigsBuilder('d1t',package_name='d1t_moveit_config')
         .robot_description(file_path=str(description/'urdf/d1t.urdf'))
         .robot_description_semantic().robot_description_kinematics()
         .joint_limits().trajectory_execution().planning_pipelines(pipelines=['ompl'])
         .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True).to_moveit_configs())
    initial=[float(x) for x in os.environ['GRASP01_INITIAL'].split(',')]
    nodes=[
        Node(package='d1t_sim',executable='fake_d1t',parameters=[{'initial_angles':initial,'speed_deg_per_s':60.,'start_delay_sec':0.}]),
        Node(package='d1t_driver',executable='d1t_feedback_node'),
        Node(package='d1t_arm',executable='arm_driver'),
        Node(package='d1t_arm',executable='trajectory_server',parameters=[{'execution_mode':'point','max_points':400}]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[cfg.robot_description]),
        Node(package='d1t_description',executable='joint_state_bridge',parameters=[{'source':'feedback'}]),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[cfg.to_dict()]),
        Node(package='d1t_moveit_config',executable='grasp01_desk_visual',arguments=['--installation',os.environ['GRASP01_INSTALLATION']]),
    ]
    if os.environ.get('GRASP01_RVIZ')=='1':
        view='grasp01_physical.rviz'
        nodes.append(Node(package='rviz2',executable='rviz2',name='grasp01_rviz',arguments=['-d',str(pkg/'rviz'/view)],parameters=[cfg.robot_description,cfg.robot_description_semantic,cfg.robot_description_kinematics,cfg.planning_pipelines]))
    return LaunchDescription(nodes)
