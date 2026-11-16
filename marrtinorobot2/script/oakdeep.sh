#!/bin/bash
#set -x  # Abilita il debug
source ~/marrtinorobot2_ws/install/setup.bash
ros2 launch depthai_examples stereo_inertial_node.launch.py
