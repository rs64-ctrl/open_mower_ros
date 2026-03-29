import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mower_utils_share = get_package_share_directory('mower_utils')

    start_planner_arg = DeclareLaunchArgument(
        'start_planner',
        default_value=EnvironmentVariable('START_PLANNER', default_value='True'),
        description='Whether to start the planner test node'
    )

    area_index_arg = DeclareLaunchArgument(
        'area_index',
        default_value=EnvironmentVariable('AREA_INDEX', default_value='0'),
        description='Mowing area index'
    )

    outline_count_arg = DeclareLaunchArgument(
        'outline_count',
        default_value=EnvironmentVariable('OM_OUTLINE_COUNT', default_value='10'),
        description='Outline count for planner'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(mower_utils_share, 'rviz', 'planner_test.rviz')],
    )

    mower_map_node = Node(
        package='mower_map',
        executable='mower_map_service',
        name='mower_map',
    )

    slic3r_node = Node(
        package='slic3r_coverage_planner',
        executable='slic3r_coverage_planner_node',
        name='slic3r_coverage_planner',
    )

    planner_test_node = Node(
        condition=IfCondition(LaunchConfiguration('start_planner')),
        package='mower_utils',
        executable='planner_test',
        name='planner_test',
        parameters=[{
            'area_index': LaunchConfiguration('area_index'),
            'outline_count': LaunchConfiguration('outline_count'),
        }],
    )

    return LaunchDescription([
        start_planner_arg,
        area_index_arg,
        outline_count_arg,
        rviz_node,
        mower_map_node,
        slic3r_node,
        planner_test_node,
    ])
