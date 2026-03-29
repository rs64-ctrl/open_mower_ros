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
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs'),

        # Set the debug environment variable
        SetEnvironmentVariable('NTRIP_CLIENT_DEBUG', LaunchConfiguration('debug')),

        # NTRIP Serial Device Node
        Node(
            package='ntrip_client',
            executable='ntrip_serial_device_ros',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'rtcm_frame_id': 'odom',
                'nmea_max_length': 82,
                'nmea_min_length': 3,
                'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,
                'reconnect_backoff_base': 1.8,
                'reconnect_backoff_max_seconds': 300,
            }],
        ),
    ])
