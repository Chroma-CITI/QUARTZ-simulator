#!/bin/bash

# Install tools 
sudo apt update
if [ `rosversion -d` = "noetic" ]
then
	sudo apt install python3-catkin-tools xterm
	CUR_DIR=`pwd`
	source /opt/ros/noetic/setup.bash
	cd $CUR_DIR
else
	sudo apt install python-catkin-tools xterm
fi

# Install ROS dependencies
rosdep update
rosdep install --from-paths packages --ignore-src --default-yes

# Install mavros Dependencies
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Set scripts
chmod +x scripts/*.sh

# Install others dependencies
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt install libignition-math4 libignition-math4-dev
