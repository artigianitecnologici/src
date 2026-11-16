#!/bin/bash
#set -x
source install/setup.bash
cd "$HOME/src/marrtinorobot2/marrtinorobot2_voice/marrtinorobot2_voice"
python3 asr_chatbot_bridge.py
