import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    stream_port = int(LaunchConfiguration('stream_port').perform(context))
    stream_quality = int(LaunchConfiguration('stream_quality').perform(context))
    match_threshold = float(LaunchConfiguration('match_threshold').perform(context))
    process_every_n_frames = int(LaunchConfiguration('process_every_n_frames').perform(context))
    publish_annotated_image = _as_bool(LaunchConfiguration('publish_annotated_image').perform(context))

    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device').perform(context),
                'image_size': [image_width, image_height],
                'time_per_frame': [1, fps],
                'camera_frame_id': 'camera_link_optical',
            }],
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': stream_port,
                'quality': stream_quality,
                'default_stream_type': 'mjpeg',
            }],
        ),
        Node(
            package='marrtinorobot2_vision',
            executable='face_recognition_node',
            name='face_recognition_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'reference_dir': LaunchConfiguration('reference_dir').perform(context),
                'person_name': 'Fabo',
                'greeting_text': 'Buongiorno Fabo, cosa facciamo oggi?',
                'match_threshold': match_threshold,
                'process_every_n_frames': process_every_n_frames,
                'greeting_cooldown_sec': 300.0,
                'publish_annotated_image': publish_annotated_image,
            }],
            condition=IfCondition(LaunchConfiguration('enable_face_recognition')),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('marrtinorobot2_vision')
    default_reference_dir = os.path.join(package_share, 'config', 'faces', 'fabo')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/webcam'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('stream_port', default_value='29090'),
        DeclareLaunchArgument('stream_quality', default_value='65'),
        DeclareLaunchArgument('reference_dir', default_value=default_reference_dir),
        DeclareLaunchArgument('match_threshold', default_value='0.72'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('enable_face_recognition', default_value='true'),
        DeclareLaunchArgument('process_every_n_frames', default_value='3'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
