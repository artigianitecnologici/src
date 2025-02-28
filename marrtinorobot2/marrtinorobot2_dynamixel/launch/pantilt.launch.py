from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    dynamixel_config_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_dynamixel'), 'config', 'dynamixel_config.yaml']
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[dynamixel_config_path],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='pan_tilt_controller',
            arguments=['pan_tilt_controller'],
            output='screen'
        )
    ])
