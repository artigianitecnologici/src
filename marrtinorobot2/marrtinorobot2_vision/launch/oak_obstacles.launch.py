import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_share = get_package_share_directory("marrtinorobot2_vision")

    oak_launch = os.path.join(vision_share, "launch", "oak_d_lite.launch.py")
    scan_params = os.path.join(
        vision_share, "config", "oak_pointcloud_to_laserscan.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera_name", default_value="oak"),
            DeclareLaunchArgument("input_cloud", default_value="/oak/stereo/points"),
            DeclareLaunchArgument("output_scan", default_value="/scan_oak"),
            DeclareLaunchArgument("params_file", default_value=scan_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oak_launch),
                condition=IfCondition(LaunchConfiguration("start_camera")),
                launch_arguments={
                    "name": LaunchConfiguration("camera_name"),
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
                remappings=[
                    ("cloud_in", LaunchConfiguration("input_cloud")),
                    ("scan", LaunchConfiguration("output_scan")),
                ],
            ),
        ]
    )

