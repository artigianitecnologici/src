#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera image_topic:=image_raw
# ros2 launch marrtinorobot2_vision apriltag.launch.py