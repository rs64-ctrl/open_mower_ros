from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbot_positioning',
            executable='xbot_positioning_node',
            name='xbot_positioning',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('~/imu_in', '/ll/imu/data_raw'),
                ('~/twist_in', '/ll/diff_drive/measured_twist'),
                ('~/xb_pose_in', '/ll/position/gps'),
                ('~/xb_pose_out', 'xbot_positioning/xb_pose'),
            ],
        ),
    ])
