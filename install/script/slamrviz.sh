#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch marrtinorobot2_navigation slam.launch.py rviz:=true 
