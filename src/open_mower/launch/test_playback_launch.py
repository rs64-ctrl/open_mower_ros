import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    rviz_dir = os.path.join(pkg_share, 'rviz')

    bagfile_arg = DeclareLaunchArgument('bagfile')
    s_arg = DeclareLaunchArgument('s', default_value='0')
    r_arg = DeclareLaunchArgument('r', default_value='1')
    rqt_reconfigure_arg = DeclareLaunchArgument('rqt_reconfigure', default_value='False')

    return LaunchDescription([
        bagfile_arg,
        s_arg,
        r_arg,
        rqt_reconfigure_arg,

        # ros2 bag play replaces rosbag play
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                LaunchConfiguration('bagfile'),
                '--clock',
                '--start-offset', LaunchConfiguration('s'),
                '--rate', LaunchConfiguration('r'),
                '--remap', '/tf:=/tf_null',
                '--remap', '/xbot_positioning/odom_out:=/null/xbot_positioning/odom_out',
            ],
            name='player',
            output='log',
        ),
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(rviz_dir, 'playback.rviz')],
            on_exit='shutdown',
        ),
        Node(
            name='converter1',
            package='mower_utils',
            executable='xbot_pose_converter',
            on_exit='shutdown',
            parameters=[{
                'topic': '/ll/position/gps',
                'frame': 'map',
            }],
        ),
        Node(
            name='converter2',
            package='mower_utils',
            executable='xbot_pose_converter',
            on_exit='shutdown',
            parameters=[{
                'topic': '/xbot_positioning/xb_pose',
                'frame': 'map',
            }],
        ),
        Node(
            package='xbot_monitoring',
            executable='xbot_monitoring',
            name='xbot_monitoring',
            output='screen',
            on_exit='shutdown',
        ),
        Node(
            package='mower_map',
            executable='mower_map_service',
            name='map_service',
            output='screen',
        ),
        Node(
            package='mower_logic',
            executable='monitoring',
            name='monitoring',
            output='screen',
        ),
        Node(
            package='xbot_positioning',
            executable='xbot_positioning_node',
            name='xbot_positioning_playback',
            output='screen',
            on_exit='shutdown',
            remappings=[
                ('~/imu_in', '/ll/imu/data_raw'),
                ('~/twist_in', '/ll/diff_drive/measured_twist'),
                ('~/xb_pose_in', '/ll/position/gps'),
                ('~/xb_pose_out', 'playback_pose'),
            ],
            parameters=[{
                'max_gps_accuracy': 0.2,
                'skip_gyro_calibration': True,
                'gyro_offset': 0.01,
                'antenna_offset_x': float(os.environ.get('OM_ANTENNA_OFFSET_X', '0')),
                'antenna_offset_y': float(os.environ.get('OM_ANTENNA_OFFSET_Y', '0')),
            }],
        ),
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
            condition=IfCondition(LaunchConfiguration('rqt_reconfigure')),
        ),
    ])
