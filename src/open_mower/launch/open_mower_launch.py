import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    launch_dir = os.path.join(pkg_share, 'launch')
    include_dir = os.path.join(launch_dir, 'include')
    params_dir = os.path.join(pkg_share, 'params')

    om_heatmap_sensor_ids = os.environ.get('OM_HEATMAP_SENSOR_IDS', 'UNSET')

    return LaunchDescription([
        # Include sub-launch files
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_teleop_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_record_launch.py')),
            launch_arguments={'prefix': 'mow_area'}.items(),
        ),

        # Nodes
        Node(
            package='mower_map',
            executable='mower_map_service',
            name='map_service',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
        ),
        Node(
            package='mower_logic',
            executable='mower_logic_node',
            name='mower_logic',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
        ),
        Node(
            package='slic3r_coverage_planner',
            executable='slic3r_coverage_planner_node',
            name='slic3r_coverage_planner',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('cmd_vel_out', '/ll/cmd_vel'),
            ],
            parameters=[
                os.path.join(params_dir, 'twist_mux_topics.yaml'),
            ],
        ),
        Node(
            package='xbot_monitoring',
            executable='xbot_monitoring',
            name='xbot_monitoring',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('/xbot_monitoring/remote_cmd_vel', '/joy_vel'),
            ],
        ),
        # heatmap_generator - only launch if OM_HEATMAP_SENSOR_IDS is set
        *([
            Node(
                package='xbot_monitoring',
                executable='heatmap_generator',
                name='heatmap_generator',
                output='screen',
                respawn=True,
                respawn_delay=10.0,
            ),
        ] if om_heatmap_sensor_ids != 'UNSET' else []),
        Node(
            package='xbot_remote',
            executable='xbot_remote',
            name='xbot_remote',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('/xbot_remote/cmd_vel', '/joy_vel'),
            ],
        ),
        Node(
            package='mower_logic',
            executable='monitoring',
            name='monitoring',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
        ),
    ])
