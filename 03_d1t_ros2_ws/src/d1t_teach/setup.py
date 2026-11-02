from setuptools import setup

package_name = 'd1t_teach'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teach.launch.py']),
        ('share/' + package_name + '/web',
         ['d1t_teach/web/index.html', 'd1t_teach/web/app.js', 'd1t_teach/web/style.css']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas',
    maintainer_email='173587389+Thomas-Lean@users.noreply.github.com',
    description='TEACH-01 web manual control panel backend for D1-T.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teach_server = d1t_teach.teach_server:main',
        ],
    },
)
