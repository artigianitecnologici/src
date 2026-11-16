#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
