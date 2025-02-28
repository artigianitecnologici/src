from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_examples',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'output_frame': 'base_link',
                'range_min': 0.2,
                'range_max': 10.0,
                'scan_height': 1,
                'depth_image_topic': '/stereo/depth',
                'scan_topic': '/scan'
            }]
        ),
    ])
