import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marrtinorobot2_vision',
            executable='getimage',
            name='image_grabber',
            output='screen',
            parameters=[
                {'log_level': 'info'}
            ]
        )
    ])