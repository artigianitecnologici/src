#!/usr/bin/env bash

set -e
echo "install prerequisite"
sudo apt update && sudo apt install -y curl gnupg2 lsb-release openssh-server net-tools terminator curl git tmux terminator
 

ARCH=$(uname -i)
RELEASE=$(lsb_release -c -s)

sudo apt update && sudo apt install -y locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

if [ $RELEASE == "bionic" ]
    then
        ROSDISTRO=eloquent

elif [ $RELEASE == "focal" ]
    then
        ROSDISTRO=galactic

elif [ $RELEASE == "jammy" ]
    then
        ROSDISTRO=humble

else
    echo "$RELEASE OS Release not supported. Exiting now"
    exit
fi

if [ $ARCH != "x86_64" ] && [ $ARCH != "aarch64" ]
    then
        echo "$ARCH architecture not supported. Exiting now"
        exit
fi

RELEASE_FILE=$ROSDISTRO$ARCH

# Add the ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing ROS $ROSDISTRO on $ARCH architecture"
sudo apt update

# Install colcon
sudo apt install -y python3-colcon-common-extensions python3-colcon-core python3-rosdep

if [ $ARCH == "x86_64" ]
    then
        sudo apt install -y ros-$ROSDISTRO-desktop

elif [ $ARCH == "aarch64" ]
    then
        sudo apt install -y ros-$ROSDISTRO-ros-base
fi

# Install python3 libraries
sudo apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete pytest-rerunfailures

# Remove conflicting em package
pip3 uninstall em

sudo rosdep init
rosdep update

echo ""
echo "ROS $ROSDISTRO installation complete!"
echo ""

echo "Set up your environment by sourcing the following file."
echo "source /opt/ros/$ROSDISTRO/setup.bash"
echo ""

echo -n "Or do you want to save this on your .bashrc (y/n)? "
read answer

if [ "$answer" != "${answer#[Yy]}" ] ;
    then
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
fi
sudo apt install gazebo -y
echo "To start using ROS2, run: source $HOME/.bashrc"
