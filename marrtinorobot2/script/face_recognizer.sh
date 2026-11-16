#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

workspace="$HOME/src/marrtinorobot2"
package_dir="$workspace/marrtinorobot2_recognition"
setup_file="$package_dir/install/setup.bash"

if [ ! -f "$setup_file" ]; then
  echo "Compilo marrtinorobot2_recognition..."
  cd "$package_dir"
  colcon build --symlink-install --packages-select marrtinorobot2_recognition
fi

source "$setup_file"

echo "Avvio face_recognizer..."
exec ros2 launch marrtinorobot2_recognition face_recognizer.launch.py
