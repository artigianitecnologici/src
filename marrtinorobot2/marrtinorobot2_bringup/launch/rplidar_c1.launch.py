# Copyright 2025 robotics-3d.com 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com

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
