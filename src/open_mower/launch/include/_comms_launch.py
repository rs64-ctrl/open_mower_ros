"""
This launch file starts all communication needed in order to control and localize the bot.
I.e. the raw data comms to the Low Level Board, the GPS, etc.
"""
import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')

    hardware_platform = os.environ.get('HARDWARE_PLATFORM', '2')
    om_no_comms = os.environ.get('OM_NO_COMMS', 'False').lower() in ('true', '1', 'yes')
    om_no_gps = os.environ.get('OM_NO_GPS', 'False').lower() in ('true', '1', 'yes')
    om_use_ntrip = os.environ.get('OM_USE_NTRIP', 'True').lower() in ('true', '1', 'yes')

    actions = []

    # v1 if HARDWARE_PLATFORM == 1
    if hardware_platform == '1':
        if not om_no_comms:
            actions.append(
                Node(
                    package='mower_comms_v1',
                    executable='mower_comms_v1',
                    name='ll',
                    output='screen',
                    respawn=True,
                    respawn_delay=10.0,
                )
            )
        # GPS driver only for v1 builds
        if not om_no_gps:
            actions.append(
                Node(
                    package='xbot_driver_gps',
                    executable='driver_gps_node',
                    name='gps',
                    namespace='ll/services',
                    output='screen',
                    respawn=True,
                    respawn_delay=5.0,
                    remappings=[
                        ('/ll/services/rtcm', '/ll/position/gps/rtcm'),
                        ('/nmea', '/ll/position/gps/nmea'),
                        ('~/xb_pose', '/ll/position/gps'),
                        ('~/wheel_ticks', '/mower/wheel_ticks'),
                    ],
                )
            )

    # v2 if HARDWARE_PLATFORM == 2
    if hardware_platform == '2':
        if not om_no_comms:
            actions.append(
                Node(
                    package='mower_comms_v2',
                    executable='mower_comms_v2',
                    name='mower_comms_v2',
                    output='screen',
                    respawn=True,
                    respawn_delay=10.0,
                )
            )

    # NTRIP for both: v1 and v2
    if not om_no_gps and om_use_ntrip:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'include', '_ntrip_client_launch.py')
                )
            )
        )

    return LaunchDescription(actions)
