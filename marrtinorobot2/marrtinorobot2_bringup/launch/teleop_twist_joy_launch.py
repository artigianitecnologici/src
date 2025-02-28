from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('marrtinorobot2_bringup'),
        'config',
        'teleop_twist_joy.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_twist_joy',
            name='teleop_twist_joy',
            parameters=[config_file],
        ),
    ])
