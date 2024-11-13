#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # .................. Configurable Arguments .....................
    use_sim_time = True
    map_saver_params_file = 'map_saver_params.yaml'
    map_file_path = '/home/lsirikh/maps/my_map'  # 저장할 맵 경로 설정
    # ...............................................................

    dir = get_package_share_directory('integrated_robot_slam')
    
    # Use PathJoinSubstitution for dynamically joining paths with LaunchConfiguration
    # map_saver_config_dir = LaunchConfiguration('map_saver_config_dir', default=os.path.join(dir, 'config'))
    # map_save_config = PathJoinSubstitution([map_saver_config_dir, map_saver_params_file])
    # 패키지 내 경로 설정
    dir = get_package_share_directory('integrated_robot_slam')
    map_save_config = os.path.join(dir, 'config', 'map_saver_params.yaml')
    print(f"map_save_config : {map_save_config}")

    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", default_value=str(use_sim_time), description="Use simulation/Gazebo clock"),
        DeclareLaunchArgument("map_saver_params_file", default_value=map_saver_params_file, description="Map Saver Configuration File"),
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, map_save_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_saver']}
            ]
        ),

        # MapSaverClient Node to call save_map service
        TimerAction(
            period=5.0,  # 서비스 요청 전 대기 시간 설정 (초)
            actions=[
                Node(
                    package='integrated_robot_slam',
                    executable='map_saver_node',  # MapSaverClient 스크립트를 실행
                    name='map_saver_node',
                    output='screen',
                    parameters=[
                        {'map_path': map_file_path}
                    ],
                )
            ]
        ),

    ])
