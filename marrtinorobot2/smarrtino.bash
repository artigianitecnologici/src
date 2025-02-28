#!/bin/bash

# Definizione della sessione tmux
SESSION=init

# Funzione per killare la sessione
kill_session() {
  tmux has-session -t $SESSION 2>/dev/null
  if [ $? == 0 ]; then
    tmux kill-session -t $SESSION
    echo "Sessione '$SESSION' killata con successo."
  else
    echo "Nessuna sessione '$SESSION' trovata."
  fi
  exit 0
}

# Gestione del parametro --kill
if [ "$1" == "--kill" ]; then
  kill_session
fi

# Controllo se la sessione esiste già
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  # Creazione della sessione tmux
   
  tmux -2 new-session -d -s $SESSION
  tmux rename-window -t $SESSION:0 'bringup'  # Window 0 is renamed to 'config'
  tmux new-window -t $SESSION:1 -n 'camera'  # Window 1 named 'docker'
  tmux new-window -t $SESSION:2 -n 'dynamixel'  # Window 2 named 'cmdexe'
  tmux new-window -t $SESSION:3 -n 'tts'  # Window 3 named 'robot_bringup'
  tmux new-window -t $SESSION:4 -n 'apriltag'  # Window 3 named 'robot_bringup'
  tmux new-window -t $SESSION:5 -n 'blockly'
  tmux new-window -t $SESSION:6 -n 'getimage'
  tmux new-window -t $SESSION:7 -n 'dynamixel'
  tmux new-window -t $SESSION:8 -n 'explorer'
  tmux new-window -t $SESSION:9 -n 'navigation'



  # Log files for command output
  CMD_EXE_LOG="/tmp/cmdexe.log"
  ROBOT_BRINGUP_LOG="/tmp/robot_bringup.log"
  AUTOSTART_LOG="/tmp/autostart.log"


  # Commands to be executed in window 0
  tmux send-keys -t $SESSION:0 "cd \$MARRTINOROBOT2_WS" C-m
  tmux send-keys -t $SESSION:0 "./bringup.sh " C-m  # Log output to cmdexe.lo

  # Commands to be executed in window 1
  tmux send-keys -t $SESSION:1 "cd \$MARRTINOROBOT2_WS" C-m
  tmux send-keys -t $SESSION:1 "./webcam.sh " C-m  # Log output to cmdexe.lo

  # Commands to be executed in window 2 
  tmux send-keys -t $SESSION:2 "cd ~/src/marrtinorobot2/marrtinorobot2_dynamixel/marrtinorobot2_dynamixel" C-m
  tmux send-keys -t $SESSION:2 "python3  pan_tilt_controller.py " C-m  # Log to websocket_robot.log

  # Commands to be executed in window 3 ('tts')
  tmux send-keys -t $SESSION:3 "cd \$MARRTINOROBOT2_WS" C-m
  tmux send-keys -t $SESSION:3 "./tts.sh " C-m  # Log output to cmdexe.lo

  # Commands to be executed in window 4 ('slam')
  tmux send-keys -t $SESSION:4 "cd \$MARRTINOROBOT2_WS" C-m
  tmux send-keys -t $SESSION:4 "./apriltag.sh " C-m  # Log output to cmdexe.lo
  # sleep 5
  tmux send-keys -t $SESSION:5 "cd ~/src/marrtinorobot2/marrtinorobot2_webinterface/marrtinorobot2_webinterface" C-m
  tmux send-keys -t $SESSION:5 "python3 websocket_robot.py " C-m  # Log to websocket_robot.log

  tmux send-keys -t $SESSION:6 "cd ~/src/marrtinorobot2/marrtinorobot2_vision/marrtinorobot2_vision" C-m
  tmux send-keys -t $SESSION:6 "python3  node_getimage.py " C-m  # Log to websocket_robot.log

  tmux send-keys -t $SESSION:4 "cd \$MARRTINOROBOT2_WS" C-m
  #tmux send-keys -t $SESSION:7 "python3  pan_tilt_controller.py " C-m  # Log to websocket_robot.log

  
  tmux send-keys -t $SESSION:8 "cd ~/marrtinorobot2_ws" C-m
  tmux send-keys -t $SESSION:9 "cd ~/marrtinorobot2_ws" C-m

fi

# Apertura della sessione tmux finale
tmux attach -t $SESSION
