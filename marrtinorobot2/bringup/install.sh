
sudo cp bingup.service  /etc/systemd/system/bringup.service
systemctl status bringup.service
sudo systemctl enable bringup.service
sudo systemctl start bringup.service
