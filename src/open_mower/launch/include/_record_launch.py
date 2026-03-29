"""
Include this file to use these utilities to record all low level comms,
so you can playback later on for debugging.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    recordings_path = os.environ.get('RECORDINGS_PATH', '/tmp/recordings')
    om_enable_recording = os.environ.get('OM_ENABLE_RECORDING', 'False')
    om_enable_recording_all = os.environ.get('OM_ENABLE_RECORDING_ALL', 'False')
    snapshot_duration = int(os.environ.get('OM_SNAPSHOT_DURATION', '1200'))
    snapshot_memory = float(os.environ.get('OM_SNAPSHOT_MEMORY_PER_TOPIC', '0.0'))

    prefix_arg = DeclareLaunchArgument('prefix', default_value='record')

    actions = [prefix_arg]

    # ROS2 uses ros2 bag record instead of rosbag record
    if om_enable_recording.lower() in ('true', '1', 'yes'):
        actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', [recordings_path, '/', LaunchConfiguration('prefix')],
                    '/clock',
                    '/time_reference',
                    '/tf',
                    '/tf_static',
                    '/joy',
                    '/cmd_vel',
                    '/mower/status',
                    '/mower/imu',
                    '/imu/data_raw',
                    '/imu/mag',
                    '/ublox/navrelposned',
                    '/ublox/fix',
                ],
                name='rosbag_record_diag',
                output='log',
            )
        )

    if om_enable_recording_all.lower() in ('true', '1', 'yes'):
        actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', [recordings_path, '/all_', LaunchConfiguration('prefix')],
                    '-a',
                ],
                name='rosbag_record_diag_all',
                output='log',
            )
        )

    # rosbag_snapshot equivalent - in ROS2, this would need a different approach
    # For now, include it as a node if the package exists
    if snapshot_duration > 0 and snapshot_memory > 0:
        actions.append(
            Node(
                name='snapshot',
                package='rosbag_snapshot',
                executable='snapshot',
                parameters=[{
                    'default_duration_limit': snapshot_duration,
                    'default_memory_limit': snapshot_memory,
                    'clear_buffer': False,
                    'compression': 'LZ4',
                    'record_all_topics': True,
                }],
            )
        )

    return LaunchDescription(actions)
