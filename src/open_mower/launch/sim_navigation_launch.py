"""
Use this file to record new mowing and navigation areas in the simulator.
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
    rviz_dir = os.path.join(pkg_share, 'rviz')

    sim_share = get_package_share_directory('mower_simulation')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_params_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', '_mower_simulation_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_move_base_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_teleop_launch.py'))
        ),

        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(rviz_dir, 'sim_navigation_test.rviz')],
            on_exit='shutdown',
        ),
        Node(
            package='mower_map',
            executable='mower_map_service',
            name='mower_map',
            on_exit='shutdown',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[
                ('cmd_vel_out', '/cmd_vel'),
            ],
            parameters=[
                os.path.join(params_dir, 'twist_mux_topics.yaml'),
            ],
        ),
    ])
