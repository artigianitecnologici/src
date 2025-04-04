#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
#ros2 launch depthai_examples stereo.launch.py camera_model:=OAK_D_LITE
ros2 launch marrtinorobot2_vision oak_d_lite.launch.py