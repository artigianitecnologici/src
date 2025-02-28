#!/bin/bash
# Script to set up and manage a tmux session for running robot-related processes
# The script creates a tmux session, opens multiple windows, and executes commands.
# It also logs the output of each command to log files.

# Get the current date and print it
date

# Define the name of the tmux session
SESSION=init

# Check if the session already exists
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  # If the session doesn't exist, set up a new tmux session
  tmux -2 new-session -d -s $SESSION
  tmux rename-window -t $SESSION:0 'config'  # Window 0 is renamed to 'config'
  tmux new-window -t $SESSION:1 -n 'rosbridge'  # Window 1 named 'docker'
  tmux new-window -t $SESSION:2 -n 'cmdexe'  # Window 2 named 'cmdexe'
  tmux new-window -t $SESSION:3 -n 'robot_bringup'  # Window 3 named 'robot_bringup'
  tmux new-window -t $SESSION:4 -n 'autostart'  # Window 3 named 'robot_bringup'
fi

# Log files for command output
CMD_EXE_LOG="/tmp/cmdexe.log"
ROBOT_BRINGUP_LOG="/tmp/robot_bringup.log"
AUTOSTART_LOG="/tmp/autostart.log"

# Commands to be executed in window 2 ('cmdexe')
tmux send-keys -t $SESSION:1 "cd \$MARRTINOROBOT2_WS" C-m
tmux send-keys -t $SESSION:1 "./rosbridge.sh > $CMD_EXE_LOG 2>&1 &" C-m  # Log output to cmdexe.lo

# Commands to be executed in window 2 ('cmdexe')
tmux send-keys -t $SESSION:2 "cd ~/src/marrtinorobot2/marrtinorobot2_webinterface/marrtinorobot2_webinterface" C-m
tmux send-keys -t $SESSION:2 "python3 command_executor.py > $CMD_EXE_LOG 2>&1 &" C-m  # Log output to cmdexe.log

# Commands to be executed in window 3 ('robot_bringup')
tmux send-keys -t $SESSION:3 "cd ~/src/marrtinorobot2/marrtinorobot2_webinterface/marrtinorobot2_webinterface" C-m
tmux send-keys -t $SESSION:3 "python3 robot_bringup.py > $ROBOT_BRINGUP_LOG 2>&1 &" C-m  # Log output to robot_bringup.log

# Commands to be executed in window 4 ('robot_bringup')
tmux send-keys -t $SESSION:4 "cd ~/src/marrtinorobot2/bringup" C-m
tmux send-keys -t $SESSION:4 "python3 autostart.py > $AUTOSTART_LOG 2>&1 &" C-m  # Log output to robot_bringup.log

# Optional: Start other processes if needed (currently commented out)
# sleep 5
# tmux send-keys -t $SESSION:3 "cd \$MARRTINO_APPS_HOME/blockly" C-m
# tmux send-keys -t $SESSION:3 "python websocket_robot.py > /tmp/websocket_robot.log 2>&1 &" C-m  # Log to websocket_robot.log

# Uncomment the lines below if you want to check for a quit request
# while [ ! -f "/tmp/quitrequest" ]; do
#   sleep 5
# done

# Instructions:
# - The output of 'command_executor.py' will be logged in /tmp/cmdexe.log.
# - The output of 'robot_bringup.py' will be logged in /tmp/robot_bringup.log.
# - You can access the tmux session with 'tmux attach-session -t init' to view the commands being run in real time.

