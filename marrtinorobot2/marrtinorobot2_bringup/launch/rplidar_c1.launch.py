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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to the second launch file
    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_c1_launch.py'  # Replace with the actual name of your second launch file
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),
        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),
        DeclareLaunchArgument(
            name='lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Lidar serial port device name'
        ),
        DeclareLaunchArgument(
            name='lidar_serial_baudrate',
            default_value='460800',
            description='Lidar serial baudrate'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'angle_compensate': 'true',  # You can adjust or pass this as an argument too
            }.items(),
        ),
    ])
