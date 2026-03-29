from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower_simulation',
            executable='mower_simulation',
            name='mower_simulation',
            remappings=[
                ('~/xb_pose_out', 'xbot_positioning/xb_pose'),
            ],
            on_exit='shutdown',
        ),
    ])
