#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

workspace="${MARRTINOROBOT2_SOURCE:-/home/ubuntu/src/marrtinorobot2}"
package_dir="$workspace/marrtinorobot2_recognition"

cd "$package_dir"
colcon build --symlink-install --packages-select marrtinorobot2_recognition
source "$package_dir/install/setup.bash"

exec ros2 launch marrtinorobot2_recognition person_follower.launch.py
