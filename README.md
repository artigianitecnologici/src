# Download Repository Branch Dev
    mkdir ~/src
    
    cd ~/src
    
    git clone   --single-branch --filter=tree:0 https://github.com/marrtino/MARRtinoROS2 .

# prerequisiti
sudo apt install git terminator curl openssh-server
sudo apt install chromium-browser
sudo systemctl enable ssh

# prerequisiti x audio
sudo apt update

sudo apt install sox libsox-fmt-all libttspico-utils pulseaudio

speaker-test -t wav -c 2 -l 1


# Forzare su ubuntu con gnome il rendering su X11
sudo nano /etc/gdm3/custom.conf

Assicurati che questa riga NON sia commentata: 

    WaylandEnable=false 
    
sudo systemctl restart gdm3


# Disabiltare il rendering grafico su ssh

ssh -x user@remote_host
ssh -x marrtino@10.3.1.1

# configurare env 

    echo "export MARRTINOROBOT2_WEBI=/home/marrtino/src/marrtinorobot2/marrtinorobot2_webinterface/www" >> /home/marrtino/.bashrc
    echo "export MARRTINOROBOT2_HOME=/home/marrtino/src/marrtinorobot2" >> /home/marrtino/.bashrc


# Modifica della configurazione su ssh
sudo nano /etc/ssh/sshd_config

    Setting 
    X11Forwarding no
    
    Restart service ssh
    sudo systemctl restart sshd

# disinstallare il portachiavi 

    sudo apt remove --purge gnome-keyring
    rm -rf ~/.local/share/keyrings
    sudo reboot

# ripristinare i widget o applet
    mate-panel --reset
# disable screen  saver e risparmio energetico monitor

    mkdir -p ~/.config/autostart
    cat > ~/.config/autostart/x11-nosleep.desktop << 'EOF'
    [Desktop Entry]
    Type=Application
    Name=X11 No Sleep
    Exec=/home/$USER/bin/x11-nosleep.sh
    X-MATE-Autostart-enabled=true
    EOF

# push repository
git add .
git commit -m "Update e fix"
git push origin main
# ------------------------
# tmux list-sessions  
# tmux a -t start-nodes
# -------------------------------
# ps aux | grep nome processo
# kill -9 pid 
