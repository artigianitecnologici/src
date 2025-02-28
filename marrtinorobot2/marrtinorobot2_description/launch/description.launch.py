import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Function to generate the launch description
def generate_launch_description():
    # Retrieve the base configuration from the environment variable or set a default
    robot_base = os.getenv('MARRTINOROBOT2_BASE')
    if not robot_base:  # Check if the variable is not set or is empty
        robot_base = "2wd"

    # Define the path to the URDF file dynamically based on the robot base type
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("marrtinorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )

    # Define the path to the RViz configuration file
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_description'), 'rviz', 'description.rviz']
    )

    # Return the LaunchDescription containing all nodes and configurations
    return LaunchDescription([
        # Declare a launch argument for specifying the URDF path
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
        # Declare a launch argument to enable or disable joint state publisher
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        # Declare a launch argument to enable or disable RViz
        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run RViz'
        ),

        # Declare a launch argument for using simulation time
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        # Launch the joint state publisher if publish_joints is set to true
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints"))
            # The parameters line is commented out because Galactic automatically handles use_sim_time
            # Uncomment and modify if needed in other versions or contexts
            # parameters=[
            #     {'use_sim_time': LaunchConfiguration('use_sim_time')}
            # ]
        ),

        # Launch the robot state publisher to publish the robot description (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),

        # Launch RViz if the rviz argument is set to true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

# References:
# - ROS 2 Navigation Setup Guide: https://navigation.ros.org/setup_guides/index.html
# - Example ROS 2 launch file: https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
# - ROS 2 issue with Galactic: https://github.com/ros2/rclcpp/issues/940
