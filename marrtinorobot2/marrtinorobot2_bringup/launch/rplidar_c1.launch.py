from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='lidar_serial_port',
            default_value='/dev/rplidar',
            description='Serial port per il LIDAR'
        ),
        DeclareLaunchArgument(
            name='lidar_serial_baudrate',
            default_value='460800',
            description='Baudrate per il LIDAR'
        ),
        DeclareLaunchArgument(
            name='frame_id',
            default_value='laser',
            description='Frame ID del LIDAR'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True
            }],
            remappings=[
                ('/scan', '/scan_unfiltered')  # 🔁 Remapping vero e funzionante
            ]
        )
    ])
