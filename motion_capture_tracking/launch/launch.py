import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('motion_capture_tracking'),
        'config',
        'rviz.rviz')

    node_config = os.path.join(
        get_package_share_directory('motion_capture_tracking'),
        'config',
        'cfg.yaml')

    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[node_config]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
