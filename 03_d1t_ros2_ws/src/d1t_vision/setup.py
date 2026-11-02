from glob import glob
from setuptools import setup

setup(
    name='d1t_vision', version='0.1.0', packages=['d1t_vision'],
    data_files=[('share/ament_index/resource_index/packages',['resource/d1t_vision']),
                ('share/d1t_vision',['package.xml']),
                ('share/d1t_vision/launch',glob('launch/*.launch.py')),
                ('share/d1t_vision/config',glob('config/*.json'))],
    install_requires=['setuptools','numpy'], zip_safe=True,
    maintainer='Thomas', maintainer_email='173587389+Thomas-Lean@users.noreply.github.com', license='Apache-2.0',
    url='https://github.com/Thomas-Lean/d1t-ros2',
    description='Read-only eye-in-hand calibration and stamped optical-to-base point conversion.',
    entry_points={'console_scripts':[
        'vision_calibrate = d1t_vision.cli:main',
        'vision_capture = d1t_vision.ros_nodes:capture_main',
        'vision_point_to_base = d1t_vision.ros_nodes:point_main',
        'vision_camera_mount = d1t_vision.ros_nodes:mount_main']},
)
