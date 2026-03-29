"""
Use this file to record new mowing and navigation areas in the simulator.
"""
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
    rviz_dir = os.path.join(pkg_share, 'rviz')

    sim_share = get_package_share_directory('mower_simulation')

    om_start_rviz = os.environ.get('OM_START_RVIZ', 'True').lower() in ('true', '1', 'yes')
    om_heatmap_sensor_ids = os.environ.get('OM_HEATMAP_SENSOR_IDS', 'UNSET')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_params_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', '_mower_simulation_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_move_base_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include_dir, '_teleop_launch.py'))
        ),
    ]

    # Load simulation params
    # In ROS2, rosparam loading at launch level is done via node parameters
    # simulation_params.yaml will be loaded by relevant nodes

    if om_start_rviz:
        actions.append(
            Node(
                name='rviz',
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(rviz_dir, 'sim_mower_logic.rviz')],
                on_exit='shutdown',
            )
        )
        actions.append(
            Node(
                package='rqt_reconfigure',
                executable='rqt_reconfigure',
                name='rqt_reconfigure',
            )
        )

    actions.extend([
        Node(
            package='mower_map',
            executable='mower_map_service',
            name='mower_map',
            on_exit='shutdown',
        ),
        Node(
            package='mower_logic',
            executable='mower_logic_node',
            name='mower_logic',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
        ),
        Node(
            package='slic3r_coverage_planner',
            executable='slic3r_coverage_planner_node',
            name='slic3r_coverage_planner',
            output='screen',
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
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            remappings=[
                ('cmd_vel_out', '/ll/cmd_vel'),
            ],
            parameters=[
                os.path.join(params_dir, 'twist_mux_topics.yaml'),
            ],
        ),
        # Use V2 Comms
        Node(
            package='mower_comms_v2',
            executable='mower_comms_v2',
            name='mower_comms_v2',
            output='screen',
            parameters=[{
                'wheel_ticks_per_m': 0,
                'wheel_distance_m': 0,
                'dfp_is_5v': os.environ.get('OM_DFP_IS_5V', 'False').lower() in ('true', '1', 'yes'),
                'language': os.environ.get('OM_LANGUAGE', 'en'),
                'volume': int(os.environ.get('OM_VOLUME', '-1')),
            }],
        ),
        Node(
            package='xbot_monitoring',
            executable='xbot_monitoring',
            name='xbot_monitoring',
            output='screen',
            respawn=False,
            parameters=[{
                'external_mqtt_enable': os.environ.get('OM_MQTT_ENABLE', 'False').lower() in ('true', '1', 'yes'),
                'external_mqtt_hostname': os.environ.get('OM_MQTT_HOSTNAME', ''),
                'external_mqtt_port': os.environ.get('OM_MQTT_PORT', ''),
                'external_mqtt_username': os.environ.get('OM_MQTT_USER', ''),
                'external_mqtt_password': os.environ.get('OM_MQTT_PASSWORD', ''),
                'external_mqtt_topic_prefix': os.environ.get('OM_MQTT_TOPIC_PREFIX', ''),
            }],
            remappings=[
                ('/xbot_monitoring/remote_cmd_vel', '/joy_vel'),
            ],
        ),
        Node(
            package='xbot_remote',
            executable='xbot_remote',
            name='xbot_remote',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            remappings=[
                ('/xbot_remote/cmd_vel', '/joy_vel'),
            ],
        ),
        Node(
            package='mower_logic',
            executable='monitoring',
            name='monitoring',
            output='log',
            respawn=True,
            respawn_delay=10.0,
        ),
    ])

    # heatmap_generator - only if OM_HEATMAP_SENSOR_IDS is set
    if om_heatmap_sensor_ids != 'UNSET':
        actions.append(
            Node(
                package='xbot_monitoring',
                executable='heatmap_generator',
                name='heatmap_generator',
                output='screen',
                respawn=True,
                respawn_delay=10.0,
                parameters=[{
                    'sensor_ids': om_heatmap_sensor_ids,
                }],
            )
        )

    return LaunchDescription(actions)
