from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marrtinorobot2_voice',
            executable='asr_tts_node',
            name='asr_tts_node',
            output='screen'
        )
    ])
