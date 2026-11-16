# tts_node_launch.py 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 
    tts_node = Node(
        package='marrtinorobot2_voice',
        executable='tts_node',
        name='tts_node',
        output='screen'
    )

    # 
    ld = LaunchDescription()
    ld.add_action(tts_node)

    return ld
    