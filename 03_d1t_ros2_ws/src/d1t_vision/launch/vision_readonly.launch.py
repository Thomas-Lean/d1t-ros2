"""Starts only the explicitly selected read-only node; never starts robot/camera drivers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_node(context):
    mode=LaunchConfiguration('mode').perform(context)
    executables={'capture':'vision_capture','point':'vision_point_to_base','mount':'vision_camera_mount'}
    if mode not in executables:
        raise ValueError('mode must be capture, point or mount')
    config=LaunchConfiguration('config').perform(context)
    if not config:
        raise ValueError('explicit config file required')
    parameters={'config_path':config,'use_sim_time':False}
    if mode in ('point','mount'):
        parameters['calibration_file']=LaunchConfiguration('calibration').perform(context)
        if not parameters['calibration_file']:
            raise ValueError('explicit real calibration file required')
    if mode=='mount':
        parameters['camera_root_frame']=LaunchConfiguration('camera_root_frame').perform(context)
        if not parameters['camera_root_frame']:
            raise ValueError('driver root frame required for optional TF mount')
    return [Node(package='d1t_vision',executable=executables[mode],output='screen',parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',description='Completed and field-verified runtime JSON'),
        DeclareLaunchArgument('mode',default_value='capture'),
        DeclareLaunchArgument('calibration',default_value=''),
        DeclareLaunchArgument('camera_root_frame',default_value=''),
        OpaqueFunction(function=launch_node)])
