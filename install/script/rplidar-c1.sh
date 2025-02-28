#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch marrtinorobot2_bringup rplidar_c1.launch.py 
