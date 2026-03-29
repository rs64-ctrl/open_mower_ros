"""
Use this file to test your move_base config.
You should be able to move the bot around the map using RVIZ.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    include_dir = os.path.join(pkg_share, 'launch', 'include')
    params_dir = os.path.join(pkg_share, 'params')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_params_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_comms_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_move_base_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_localization_launch.py'))
        ),

        Node(
            package='mower_map',
            executable='mower_map_service',
            name='map_service',
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[
                ('cmd_vel_out', '/ll/cmd_vel'),
            ],
            parameters=[
                os.path.join(params_dir, 'twist_mux_topics.yaml'),
            ],
        ),
    ])
