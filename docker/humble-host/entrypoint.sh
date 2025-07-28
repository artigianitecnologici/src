#!/bin/bash

# Imposta l'utente e la home directory
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* Abilitazione dell'utente personalizzato: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    if [ -z "$PASSWORD" ]; then
        echo "  Impostazione della password predefinita su \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || echo ""
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME} 2>/dev/null
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi

# Imposta la password VNC
VNC_PASSWORD=${PASSWORD:-marrtino}

mkdir -p $HOME/.vnc
echo $VNC_PASSWORD | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd
chown -R $USER:$USER $HOME
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# Configura xstartup
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
mate-session
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH

# Script per l'avvio del server VNC
VNCRUN_PATH=$HOME/.vnc/vnc_run.sh
cat << EOF > $VNCRUN_PATH
#!/bin/sh

# Rimuove eventuali lock file esistenti
if [ -e /tmp/.X1-lock ]; then
    rm -f /tmp/.X1-lock
fi
if [ -e /tmp/.X11-unix/X1 ]; then
    rm -f /tmp/.X11-unix/X1
fi

# Avvia il server VNC
if [ \$(uname -m) = "aarch64" ]; then
    LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver :1 -fg -geometry 1920x1080 -depth 24
else
    vncserver :1 -fg -geometry 1920x1080 -depth 24
fi
EOF
chmod +x $VNCRUN_PATH
chown $USER:$USER $VNCRUN_PATH

# Configura supervisord
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf
cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root

[program:vnc]
command=gosu $USER bash $VNCRUN_PATH
autorestart=true

[program:novnc]
command=gosu $USER bash -c "websockify --web=/usr/lib/novnc 8085 localhost:5901"
autorestart=true
EOF

# Configura ROS e colcon
BASHRC_PATH=$HOME/.bashrc
grep -F "source /opt/ros/\$ROS_DISTRO/setup.bash" $BASHRC_PATH || echo "source /opt/ros/\$ROS_DISTRO/setup.bash" >> $BASHRC_PATH
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" $BASHRC_PATH || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $BASHRC_PATH
chown $USER:$USER $BASHRC_PATH

# Corregge i permessi per rosdep
mkdir -p $HOME/.ros
cp -r /root/.ros/rosdep $HOME/.ros/rosdep
chown -R $USER:$USER $HOME/.ros

# Aggiunge i collegamenti sul desktop
mkdir -p $HOME/Desktop

# Collegamento per Terminator
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

# Collegamento per VSCodium
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

# Pulizia delle variabili sensibili
unset PASSWORD
unset VNC_PASSWORD

echo "============================================================================================"
echo "NOTA: il flag --security-opt seccomp=unconfined è richiesto per avviare l'immagine basata su Ubuntu Jammy."
echo "Vedi: https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56"
echo "============================================================================================"

# Avvia supervisord
exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf
