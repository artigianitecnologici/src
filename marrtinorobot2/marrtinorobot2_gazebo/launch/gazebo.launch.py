import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Use simulation time (synchronized with Gazebo)
    use_sim_time = True

    # Get the package share directory
    pkg_share = get_package_share_directory("marrtinorobot2_gazebo")

    # Set the path to the SDF model files
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = f"{gazebo_models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"

    # Set the path to the world file
    world_file_name = 'new.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Path to the EKF configuration file
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_base"), "config", "ekf.yaml"]
    )

    # Path to the robot description launch file
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        # Declare the 'world' argument, allowing it to be passed at runtime
        DeclareLaunchArgument(
            name='world',
            default_value=world_path,
            description='Gazebo world file path'
        ),

        # Launch Gazebo with the specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        # Spawn the robot entity in Gazebo using the 'robot_description' topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "marrtinorobot2"]
        ),

        # Node to handle command timeouts for safety
        Node(
            package='marrtinorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        ),

        # Launch the EKF node for sensor fusion and localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},  # Use simulation time
                ekf_config_path  # EKF configuration file path
            ],
            remappings=[("odometry/filtered", "odom")]  # Remap the filtered odometry topic
        ),

        # Include the robot description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),  # Pass simulation time argument
                'publish_joints': 'false',  # Disable joint state publication (optimization)
            }.items()
        ),
    ])
