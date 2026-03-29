import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    params_dir = os.path.join(pkg_share, 'params')

    return LaunchDescription([
        Node(
            name='move_base_flex',
            package='mbf_costmap_nav',
            executable='mbf_costmap_nav_node',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('cmd_vel', '/nav_vel'),
            ],
            parameters=[
                # Load costmap params with namespaces
                {'global_costmap': {}},
                os.path.join(params_dir, 'costmap_common_params.yaml'),
                os.path.join(params_dir, 'local_costmap_params.yaml'),
                os.path.join(params_dir, 'global_costmap_params.yaml'),
                os.path.join(params_dir, 'move_base_flex.yaml'),
                os.path.join(params_dir, 'ftc_local_planner.yaml'),
                os.path.join(params_dir, 'docking_ftc_local_planner.yaml'),
                os.path.join(params_dir, 'global_planner_params.yaml'),
            ],
            arguments=['--ros-args', '--log-level', 'local_costmap:=warn', '--log-level', 'global_costmap:=warn'],
        ),
    ])
