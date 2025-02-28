#!/bin/bash

# Aggiungi configurazioni a .bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ubuntu/.bashrc
echo "source /home/ubuntu/marrtinorobot2_ws/install/setup.bash" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_BASE=2wd" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_LASER_SENSOR=rplidar" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_DEPTH_SENSOR=oakdlite" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_WEBI=/home/ubuntu/src/marrtinorobot2/marrtinorobot2_webinterface/www" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_WS=/home/ubuntu/marrtinorobot2_ws" >> /home/ubuntu/.bashrc
echo "export MARRTINOROBOT2_HOME=/home/ubuntu/src/marrtinorobot2" >> /home/ubuntu/.bashrc
echo "alias cdmd='cd ~/src/marrtinorobot2/docker'" >> /home/ubuntu/.bashrc
echo "alias cdm='cd ~/src/marrtinorobot2'" >> /home/ubuntu/.bashrc
echo "alias cb='cd ~/marrtinorobot2_ws && colcon build'" >> /home/ubuntu/.bashrc

# Sourcing delle configurazioni per la sessione corrente
source /home/ubuntu/.bashrc

# Log di completamento
echo "Environment setup complete."
