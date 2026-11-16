#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
cd src/marrtinorobot2_navigation/maps
if [ -z "$1" ]; then
  DEFAULT_MAP_NAME="home_map"
  echo "Nessun nome file fornito, uso il valore di default: $DEFAULT_MAP_NAME"
  MAP_NAME=$DEFAULT_MAP_NAME
else
  MAP_NAME=$1
fi
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME  --ros-args -p save_map_timeout:=10000.
