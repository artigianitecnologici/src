# Download Repository
mkdir ~/src
cd ~/src
git clone https://github.com/marrtino/MARRtinoROS2 . --filter=tree:0


# Disabiltare il rendering grafico su ssh

ssh -x user@remote_host
ssh -x marrtino@10.3.1.1

echo "export MARRTINOROBOT2_WEBI=/home/marrtino/src/marrtinorobot2/marrtinorobot2_webinterface/www" >> /home/marrtino/.bashrc
echo "export MARRTINOROBOT2_WS=/home/marrtino/marrtinorobot2_ws" >> /home/marrtino/.bashrc


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
