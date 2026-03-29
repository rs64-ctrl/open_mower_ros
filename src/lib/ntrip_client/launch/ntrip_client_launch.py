from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments with default values
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('node_name', default_value='ntrip_client'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('host', default_value='20.185.11.35'),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value='VTC1_RTCM3'),
        DeclareLaunchArgument('authenticate', default_value='true'),
        DeclareLaunchArgument('username', default_value='user'),
        DeclareLaunchArgument('password', default_value='pass'),
        DeclareLaunchArgument('ntrip_version', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('cert', default_value=''),
        DeclareLaunchArgument('key', default_value=''),
        DeclareLaunchArgument('ca_cert', default_value=''),
        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs'),

        # Set the debug environment variable
        SetEnvironmentVariable('NTRIP_CLIENT_DEBUG', LaunchConfiguration('debug')),

        # NTRIP Client Node
        Node(
            package='ntrip_client',
            executable='ntrip_ros',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'mountpoint': LaunchConfiguration('mountpoint'),
                'ntrip_version': LaunchConfiguration('ntrip_version'),
                'authenticate': LaunchConfiguration('authenticate'),
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),
                'ssl': LaunchConfiguration('ssl'),
                'cert': LaunchConfiguration('cert'),
                'key': LaunchConfiguration('key'),
                'ca_cert': LaunchConfiguration('ca_cert'),
                'rtcm_frame_id': 'odom',
                'nmea_max_length': 82,
                'nmea_min_length': 3,
                'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,
                'reconnect_backoff_base': 1.8,
                'reconnect_backoff_max_seconds': 300,
                'rtcm_timeout_seconds': 4,
            }],
        ),
    ])
