import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')
    rviz_dir = os.path.join(pkg_share, 'rviz')

    return LaunchDescription([
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(rviz_dir, 'record_map.rviz')],
            on_exit='shutdown',
        ),
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
        ),
    ])
