#!/bin/bash
#set -x
source install/setup.bash
cd "$HOME/src/marrtinorobot2/marrtinorobot2_vision/marrtinorobot2_vision"
python3 node_getimage.py
