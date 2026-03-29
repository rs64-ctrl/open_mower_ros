import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    gamepad = os.environ.get('OM_MOWER_GAMEPAD', 'xbox360')

    actions = []

    if gamepad == 'xbox360':
        actions.append(
            Node(
                name='joy',
                package='joy_linux',
                executable='joy_linux_node',
                respawn=True,
                respawn_delay=10.0,
                parameters=[{
                    'autorepeat_rate': 10.0,
                    'coalesce_interval': 0.06,
                }],
            )
        )
        actions.append(
            Node(
                name='joy_teleop',
                package='teleop_twist_joy',
                executable='teleop_node',
                respawn=True,
                respawn_delay=10.0,
                remappings=[
                    ('cmd_vel', '/joy_vel'),
                ],
                parameters=[{
                    'scale_linear': 0.5,
                    'scale_angular': 1.5,
                    'scale_linear_turbo': 1.0,
                    'scale_angular_turbo': 3.0,
                    'enable_turbo_button': 4,
                }],
            )
        )
    else:
        # Non-xbox360 gamepad
        gamepad_yaml = os.path.join(pkg_share, 'params', 'gamepads', f'{gamepad}.yaml')
        actions.append(
            Node(
                name='joy',
                package='joy_linux',
                executable='joy_linux_node',
                respawn=True,
                respawn_delay=10.0,
                remappings=[
                    ('/joy', '/teleop_joy'),
                ],
                parameters=[{
                    'coalesce_interval': 0.06,
                }],
            )
        )
        actions.append(
            Node(
                name='joy_teleop',
                package='joy_teleop',
                executable='joy_teleop',
                respawn=True,
                respawn_delay=10.0,
                remappings=[
                    ('/joy', '/teleop_joy'),
                ],
                parameters=[gamepad_yaml] if os.path.exists(gamepad_yaml) else [],
            )
        )

    return LaunchDescription(actions)
