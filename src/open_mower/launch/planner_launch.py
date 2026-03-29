from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slic3r_coverage_planner',
            executable='slic3r_coverage_planner_node',
            name='slic3r_coverage_planner',
            output='screen',
            parameters=[{
                'visualize_plan': False,
            }],
        ),
    ])
