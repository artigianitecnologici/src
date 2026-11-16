import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('marrtinorobot2_recognition')
    params = os.path.join(package_share, 'params', 'person_follower.yaml')
    return LaunchDescription([
        Node(
            package='marrtinorobot2_recognition',
            executable='person_follower',
            name='person_follower',
            output='screen',
            parameters=[params],
        )
    ])
