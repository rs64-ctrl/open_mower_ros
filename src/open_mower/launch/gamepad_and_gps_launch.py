import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    include_dir = os.path.join(pkg_share, 'launch', 'include')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_params_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_comms_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_localization_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_record_launch.py')),
            launch_arguments={'prefix': 'gamepad_gps'}.items(),
        ),

        Node(
            name='joy',
            package='joy_linux',
            executable='joy_linux_node',
            on_exit='shutdown',
            parameters=[{
                'autorepeat_rate': 10.0,
                'coalesce_interval': 0.06,
            }],
        ),
        Node(
            name='joy_teleop',
            package='teleop_twist_joy',
            executable='teleop_node',
            on_exit='shutdown',
            parameters=[{
                'scale_linear': 0.5,
                'scale_angular': 1.5,
                'scale_linear_turbo': 1.0,
                'scale_angular_turbo': 3.0,
                'enable_turbo_button': 4,
            }],
        ),
    ])
