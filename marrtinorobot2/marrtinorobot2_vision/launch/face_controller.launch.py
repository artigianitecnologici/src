from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #
    face_tracker_controller = Node(
        package='marrtinorobot2_vision',
        executable='face_tracker_controller',
        name='face_tracker_controller',
        output='screen'
         
    )
    # 
    ld = LaunchDescription()
    ld.add_action(face_tracker_controller)

    return ld