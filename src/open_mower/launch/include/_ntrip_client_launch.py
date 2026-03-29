"""Standalone launch file for an ntrip client."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments with default values
    authenticate_arg = DeclareLaunchArgument('authenticate', default_value='true')
    ntrip_version_arg = DeclareLaunchArgument('ntrip_version', default_value='')
    ssl_arg = DeclareLaunchArgument('ssl', default_value='false')
    cert_arg = DeclareLaunchArgument('cert', default_value='')
    key_arg = DeclareLaunchArgument('key', default_value='')
    ca_cert_arg = DeclareLaunchArgument('ca_cert', default_value='')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    rtcm_message_package_arg = DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs')

    ntrip_client_node = Node(
        name='ntrip_client',
        package='ntrip_client',
        executable='ntrip_ros.py',
        output='screen',
        respawn=True,
        respawn_delay=10.0,
        parameters=[{
            'ntrip_version': LaunchConfiguration('ntrip_version'),
            'authenticate': LaunchConfiguration('authenticate'),
            'ssl': LaunchConfiguration('ssl'),
            'cert': LaunchConfiguration('cert'),
            'key': LaunchConfiguration('key'),
            'ca_cert': LaunchConfiguration('ca_cert'),
            'rtcm_frame_id': 'odom',
            'nmea_max_length': 82,
            'nmea_min_length': 3,
            'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),
            'rtcm_timeout_seconds': 30,
        }],
        remappings=[
            ('/nmea', '/ll/position/gps/nmea'),
            ('/rtcm', '/ll/position/gps/rtcm'),
        ],
    )

    return LaunchDescription([
        authenticate_arg,
        ntrip_version_arg,
        ssl_arg,
        cert_arg,
        key_arg,
        ca_cert_arg,
        debug_arg,
        rtcm_message_package_arg,
        ntrip_client_node,
    ])
