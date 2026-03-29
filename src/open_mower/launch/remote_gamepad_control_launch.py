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

    gamepad = os.environ.get('OM_MOWER_GAMEPAD', 'xbox360')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_params_launch.py'))
        ),
    ]

    if gamepad == 'xbox360':
        actions.append(
            Node(
                name='remote_joy',
                package='joy_linux',
                executable='joy_linux_node',
                on_exit='shutdown',
                parameters=[{
                    'autorepeat_rate': 10.0,
                    'coalesce_interval': 0.06,
                }],
            )
        )
    else:
        gamepad_yaml = os.path.join(params_dir, 'gamepads', f'{gamepad}.yaml')
        node_params = [{'coalesce_interval': 0.06}]
        if os.path.exists(gamepad_yaml):
            node_params.append(gamepad_yaml)

        actions.append(
            Node(
                name='remote_joy',
                package='joy_linux',
                executable='joy_linux_node',
                on_exit='shutdown',
                remappings=[
                    ('/joy', '/teleop_joy'),
                ],
                parameters=node_params,
            )
        )

    return LaunchDescription(actions)
