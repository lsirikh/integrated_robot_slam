from setuptools import setup

package_name = 'integrated_robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cartographer.launch.py', 'launch/map_saver.launch.py','launch/occupancy_grid.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_lds_2d.lua', 'config/map_saver_params.yaml', 'config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsirikh',
    maintainer_email='lsirikh@gmail.com',
    description='Integrated SLAM package with Cartographer and teleop keyboard',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = integrated_robot_slam.teleop_keyboard:main',
            'map_saver_node = integrated_robot_slam.map_saver_node:main',
            'data_report_node = integrated_robot_slam.data_report_node:main',

        ],
    },
)
