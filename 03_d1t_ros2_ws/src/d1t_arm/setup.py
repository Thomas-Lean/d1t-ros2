from setuptools import setup

package_name = 'd1t_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas',
    maintainer_email='173587389+Thomas-Lean@users.noreply.github.com',
    description='Resident MoveToJoints action driver for D1-T.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_driver = d1t_arm.arm_driver:main',
            'trajectory_server = d1t_arm.trajectory_server:main',
            'traj_test_client = d1t_arm.traj_test_client:main',
            'grip_probe = d1t_arm.grip_probe:main',
        ],
    },
)
