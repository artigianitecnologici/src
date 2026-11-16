from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RGB Camera Node
        Node(
            package='depthai-ros',
            executable='rgb_node',
            name='rgb_node',
            output='screen',
            parameters=[
                {'camera_fps': 30},  # Adjust FPS if needed
            ],
            remappings=[
                ('/rgb/image', '/rgb/image_raw')
            ]
        ),

        # Stereo Depth Node
        Node(
            package='depthai-ros',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[
                {'confidence_threshold': 200},  # Adjust confidence threshold if needed
                {'publish_pointcloud': True}  # Enable point cloud publishing
            ],
            remappings=[
                ('/stereo/depth', '/stereo/depth'),
                ('/stereo/points', '/stereo/points')
            ]
        ),
    ])
