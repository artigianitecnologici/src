from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Percorsi relativi ai file e pacchetti
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_navigation'), 'rviz', 'marrtinorobot2_navigation.rviz']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('marrtinorobot2_navigation'), 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        # Dichiarazione degli argomenti di launch
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulated time'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            name='map',
            default_value='',
            description='Map file path for navigation'
        ),

        # Inclusione di SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),

        # Inclusione di Nav2 per la navigazione
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_config_path,
                'map': LaunchConfiguration('map')  # Specifica il file di mappa se necessario
            }.items()
        ),

        # Nodo di RViz (opzionale)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

      
    ])
