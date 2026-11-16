#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch marrtinorobot2_description description.launch.py rviz:=true
