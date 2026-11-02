from setuptools import setup

package_name = 'd1t_sim'

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
    description='fake_d1t: D1-T controller protocol simulator.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_d1t = d1t_sim.fake_d1t:main',
        ],
    },
)
