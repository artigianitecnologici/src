mkdir ~/src
cd ~/src
git clone https://github.com/artigianitecnologici/MARRtinoROS2 . --filter=tree:0


disabiltare il rendering grafico su ssh

ssh -x user@remote_host
ssh -x marrtino@10.3.1.1



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
