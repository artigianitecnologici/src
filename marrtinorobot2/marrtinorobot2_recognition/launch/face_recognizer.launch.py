import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marrtinorobot2_recognition',
            executable='face_recognizer',
            name='face_recognizer',
            output='screen',
            parameters=[{
                'db_dir': os.path.expanduser('~/src/marrtinorobot2/marrtinorobot2_recognition/data'),
                'threshold': 0.45,
                'providers': ['CPUExecutionProvider'],  # GPU: ['CUDAExecutionProvider']
                'det_size_w': 640,
                'det_size_h': 640,
                'topic_image': '/camera/image_raw',
                'topic_output': '/recognized_face',
                'topic_command': '/face_recognizer/command',
                'topic_status': '/face_recognizer/status',
                'topic_speech': '/speech/to_speak',
                'enroll_samples': 7,
                'enroll_timeout_sec': 45.0,
                'enroll_interval_sec': 3.0,
                'enroll_instruction_delay_sec': 2.5,
                'process_every_n': 2                 # process 1 frame every 2
            }]
        )
    ])
