#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch marrtinorobot2_navigation rtabmap_localization_launch.py use_rviz:=true
