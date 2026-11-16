from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    candidates = [
        '~/src/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel',
        '~/marrtinorobot2_ws/src/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel',
        '~/marrtinorobot2_ws/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel',
    ]
    pantilt_dir = next(
        path for path in (os.path.expanduser(candidate) for candidate in candidates)
        if os.path.isfile(os.path.join(path, 'pan_tilt_controller.py'))
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'pan_tilt_controller.py'],
            cwd=pantilt_dir,
            output='screen',
        )
    ])
