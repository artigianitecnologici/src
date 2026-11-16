#!/bin/bash
#set -x  # Abilita il debug
source install/setup.bash
ros2 launch vizanti_server vizanti_server.launch.py
