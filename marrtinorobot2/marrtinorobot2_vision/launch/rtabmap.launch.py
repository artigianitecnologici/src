from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodo principale di RTAB-Map
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'frame_id': 'base_link'},
                {'subscribe_depth': True},
                {'subscribe_rgb': True},
            ],
            remappings=[
                ('rgb/image', '/color/video/image'),
                ('depth/image', '/stereo/depth'),
                ('rgb/camera_info', '/color/video/camera_info')
            ]
        )
    ])
