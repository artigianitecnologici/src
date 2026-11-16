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
#
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    laser_sensor_name = os.getenv('MARRTINOROBOT2_LASER_SENSOR', '')
    # Path to description.launch.py
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_description'), 'launch', 'description.launch.py']
    )

    # Path to EKF configuration
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_base"), "config", "ekf.yaml"]
    )

    # Path to TTS node launch file
    tts_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_voice'), 'launch', 'tts_node.launch.py']
    )

    # Path to Camera node launch file
    camera_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_vision'), 'launch', 'camera.launch.py']
    )

    # Path to LIDAR node launch file (if needed)
    ldlidar_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_bringup'), 'launch', 'ldlidar.launch.py']
    )

    rplidar_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_bringup'), 'launch', 'rplidar.launch.py']
    )
    
    rplidar_c1_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_bringup'), 'launch', 'rplidar_c1.launch.py']
    )


    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
            description='MARRtino robot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='use_camera',
            default_value='false',
            description='Condition to launch camera node'
        ),

        DeclareLaunchArgument(
            name='use_tts',
            default_value='false',
            description='Condition to launch TTS node'
        ),

        DeclareLaunchArgument(
            name='use_ldlidar',
            default_value='false',
            description='Condition to launch LIDAR node'
        ),
        DeclareLaunchArgument(
            name='use_rplidar_c1',
            default_value='false',
            description='Condition to launch RPLIDAR C1 node'
        ),
        DeclareLaunchArgument(
            name='use_rplidar',
            default_value='false',
            description='Condition to launch RPLIDAR node'
        ),


        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        # Micro-ROS Agent Node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("base_serial_port")]
        ),

        # Include Description Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        # Include Camera Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_camera'))
        ),

        # Include TTS Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tts_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_tts'))
        ),

        # Include LIDAR Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_ldlidar'))
        ),
        # Include RPLIDAR C1 Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_c1_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_rplidar_c1'))
        ),
        # Include RPLIDAR Node Launch (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_robot_launch_path),
            condition=IfCondition(LaunchConfiguration('use_rplidar'))
        )
    ])
