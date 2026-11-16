echo "source /opt/ros/$ROSDISTRO/setup.bash" >> $HOME/.bashrc
echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
echo "source /home/marrtino/marrtinorobot2_ws/install/setup.bash" >> $HOME/.bashrc
echo "export MARRTINOROBOT2_BASE=2wd" >> $HOME/.bashrc
echo "export MARRTINOROBOT2_LASER_SENSOR=rplidar" >> $HOME/.bashrc
echo "export MARRTINOROBOT2_DEPTH_SENSOR=oakdlite" >> $HOME/.bashrc
echo "export MARRTINOROBOT2_WEBI=$HOME/src/marrtinorobot2/marrtinorobot2_webinterface" >> $HOME/.bashrc
echo "export MARRTINOROBOT2_WS=$HOME/marrtinorobot2_ws" >> $HOME/.bashrc
echo "alias cdmd='cd ~/src/marrtinorobot2/docker'" >> $HOME/.bashrc
echo "alias cdm='cd ~/src/marrtinorobot2'" >> $HOME/.bashrc
echo "alias cb='cd ~/marrtinorobot2_ws && colcon build'" >> $HOME/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
