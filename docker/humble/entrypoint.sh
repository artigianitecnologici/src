#!/bin/bash

# Create User
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* enable custom user: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    if [ -z "$PASSWORD" ]; then
        echo "  set default password to \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || echo ""
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME} 2>/dev/null
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi

# VNC password
VNC_PASSWORD=${PASSWORD:-marrtino}

mkdir -p $HOME/.vnc
echo $VNC_PASSWORD | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd
chown -R $USER:$USER $HOME
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# xstartup
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
mate-session
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH

# vncserver launch
VNCRUN_PATH=$HOME/.vnc/vnc_run.sh
cat << EOF > $VNCRUN_PATH
#!/bin/sh

# Workaround for issue when image is created with "docker commit".
# Thanks to @SaadRana17
# https://github.com/Tiryoh/docker-ros2-desktop-vnc/issues/131#issuecomment-2184156856

if [ -e /tmp/.X1-lock ]; then
    rm -f /tmp/.X1-lock
fi
if [ -e /tmp/.X11-unix/X1 ]; then
    rm -f /tmp/.X11-unix/X1
fi

if [ $(uname -m) = "aarch64" ]; then
    LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver :1 -fg -geometry 1920x1080 -depth 24
else
    vncserver :1 -fg -geometry 1920x1080 -depth 24
fi
EOF

# Supervisor
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf
cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root
[program:vnc]
command=gosu '$USER' bash '$VNCRUN_PATH'
[program:novnc]
command=gosu '$USER' bash -c "websockify --web=/usr/lib/novnc 8085 localhost:5901"
EOF

# colcon
BASHRC_PATH=$HOME/.bashrc
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" $BASHRC_PATH || echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC_PATH
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" $BASHRC_PATH || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $BASHRC_PATH
chown $USER:$USER $BASHRC_PATH

# Fix rosdep permission verificare 
# mkdir -p $HOME/.ros
# cp -r /root/.ros/rosdep $HOME/.ros/rosdep
# chown -R $USER:$USER $HOME/.ros
sudo chown -R ubuntu:ubuntu /home/ubuntu/.ros
# Add terminator shortcut
mkdir -p $HOME/Desktop
cat << EOF > $HOME/Desktop/terminator.desktop
[Desktop Entry]
Name=Terminator
Comment=Multiple terminals in one window
TryExec=terminator
Exec=terminator
Icon=terminator
Type=Application
Categories=GNOME;GTK;Utility;TerminalEmulator;System;
StartupNotify=true
X-Ubuntu-Gettext-Domain=terminator
X-Ayatana-Desktop-Shortcuts=NewWindow;
Keywords=terminal;shell;prompt;command;commandline;
[NewWindow Shortcut Group]
Name=Open a New Window
Exec=terminator
TargetEnvironment=Unity
EOF

cat << EOF > $HOME/Desktop/codium.desktop
[Desktop Entry]
Name=VSCodium
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=/usr/share/codium/codium --unity-launch %F
Icon=vscodium
Type=Application
StartupNotify=false
StartupWMClass=VSCodium
Categories=TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;application/x-codium-workspace;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/usr/share/codium/codium --new-window %F
Icon=vscodium
EOF
chown -R $USER:$USER $HOME/Desktop

# clearup
PASSWORD=
VNC_PASSWORD=

# echo "=Disabilita risparmio energetico e screen saver"
# echo "============================================================================================"

# gsettings set org.mate.power-manager sleep-display-ac 0
# gsettings set org.mate.power-manager sleep-display-battery 0
# gsettings set org.mate.power-manager sleep-computer-ac 0
# gsettings set org.mate.power-manager sleep-computer-battery 0
# gsettings set org.mate.power-manager idle-dim-ac false
# gsettings set org.mate.power-manager idle-dim-battery false
# gsettings set org.mate.screensaver idle-activation-enabled false
# gsettings set org.mate.screensaver lock-enabled false
# # xset s off
# # xset -dpms
# # xset s noblank

echo "============================================================================================"
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
tmux send-keys -t $SESSION:4 "cd ~/src/marrtinorobot2" C-m
tmux send-keys -t $SESSION:4 "./startsession.bash"
# tmux send-keys -t $SESSION:4 "cd ~/src/marrtinorobot2/bringup" C-m
# tmux send-keys -t $SESSION:4 "python3 autostart.py > $AUTOSTART_LOG 2>&1 &" C-m  # Log output to robot_bringup.log

echo "============================================================================================"
echo "NOTE 1: --security-opt seccomp=unconfined flag is required to launch Ubuntu Jammy based image."
echo -e 'See \e]8;;https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e\\https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e]8;;\e\\'
echo "============================================================================================"

exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf
#
