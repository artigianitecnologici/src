#!/bin/bash
#set -x  # Abilita il debug
v4l2-ctl -d /dev/webcam --set-parm=10
source install/setup.bash
ros2 launch marrtinorobot2_vision camera.launch.py
