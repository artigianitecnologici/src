#!/bin/bash
#set -x  # Abilita il debug 
source install/setup.bash
ros2 launch marrtinorobot2_vision web_video_server.launch.py start_camera:=false process_every_n_frames:=8 publish_annotated_image:=false stream_quality:=50
