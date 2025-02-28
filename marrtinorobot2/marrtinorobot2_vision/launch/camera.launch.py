import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'video_device': '/dev/video0',  # Adjust the video device path accordingly
                'image_size': [640,480],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical',
                'calibration_file': os.path.expanduser('~/marrtinorobot2_ws/src/marrtinorobot2_vision/calibration/arducam/ost.yaml')
            }]
            
        )
    ])
