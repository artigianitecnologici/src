#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
#cd launch marrtinorobot2_navigation navigation.launch.py map:=<path_to_map_file>/<map_name>.yaml
ros2 launch marrtinorobot2_navigation navigation.launch.py rviz:=true