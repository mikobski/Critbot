#!/bin/bash

ABS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $ABS_PATH

#############################################################################
#
#############################################################################

## Bash script for setting up ROS Melodic environment for Ubuntu LTS (18.04). 
## Includes:
## - Common dependencies libraries and tools
## - Gazebo 9
## - MAVROS
## - Ardupilot SITL

#############################################################################
# 				Prepare
#############################################################################
## Check if the script is running as root
if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

## Check OS version
echo "!  OS version detected as $(lsb_release -sc)."
if [[ $(lsb_release -sc) != *"bionic"* ]]; then
  echo "!  ROS Melodic requires bionic (Ubuntu 18.04)."
  echo "!  Exiting ...."
  return 1;
fi

### Delete exisitng ROS
#echo "!  Remove existing ROS"
#sudo apt-get remove ros-* -y
#sudo apt-get autoremove -y
#sudo rm -r /etc/ros
#sudo rm -r ~/.ros

# Ubuntu Config
echo "!  Remove modemmanager"
sudo apt-get remove modemmanager -y
echo "!  Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER

#############################################################################
# 				ROS Melodic 
#		http://wiki.ros.org/melodic/Installation/Ubuntu
#############################################################################
## Setup keys
echo "!  Setting up keys"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

## Update repo
echo "!  Updating Ubuntu repositories"
sudo apt update
sudo apt-get update

## Gazebo simulator dependencies
echo "!  Installing gazebo dependencies"
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## Get ROS/Gazebo
echo "!  Installing ROS and GAZEBO"
sudo apt install ros-melodic-desktop-full -y

## Install rosinstall and other dependencies
echo "!  Installing rosinstall and other dependecies"
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-rqt -y
#sudo apt-get install python-catkin-tools -y

## Initialize rosdep
echo "!  Initializing rosdep"
sudo rosdep init
rosdep update

### Create catkin workspace
#echo "!  Creating workspace"
#mkdir -p ~/catkin_ws/src
#cd ~/catkin_ws
##catkin init
##wstool init src
mkdir -p $ABS_PATH/external

#############################################################################
# 				MAVROS 
#    https://github.com/mavlink/mavros/tree/master/mavros#installation
#############################################################################
## Install MAVLink
echo "!  Installing Mavros"
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras -y

## Install geographiclib
echo "!  Installing geographiclib"
sudo apt install geographiclib-tools -y
echo "!  Downloading dependent script 'install_geographiclib_datasets.sh'"
### Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
## If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "!  Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

#############################################################################
## 			     Ardupilot Sitl 
##	https://ardupilot.org/dev/docs/building-setup-linux.html
##############################################################################
### Clone ardupiltot from git
#echo "!  Cloning Ardupilot"
#cd ~/
#git clone https://github.com/ArduPilot/ardupilot.git
#cd ~/ardupilot
#git submodule update --init --recursive
#
### Execute install-prereqs-ubuntu.sh script
#echo "!  Install ardupilot prereqs"
#./Tools/environment_install/install-prereqs-ubuntu.sh -y
#
### Reload Paths
#echo "!  Reload paths"
#. ~/.bashrc
#. ~/.profile
#
### Build ardupilot
#echo "!  Build ardupilot SITL"
#./waf configure --board Pixhawk1
#./waf copter
#
### Install APM Planner
#echo "!  Installing APM Planner"
#cd ~/
#wget https://firmware.ardupilot.org/Tools/APMPlanner/apm_planner_2.0.26_bionic64.deb
#sudo dpkg -i apm_planner_2.0.26_bionic64.deb
#sudo apt-get -f install -y
#sudo dpkg -i apm_planner_2.0.26_bionic64.deb
#rm apm_planner_2.0.26_bionic64.deb

## Due to BUG in ROS Gazebo, we need to update Gazebo from OSR Foundation
echo "!  Updating Gazebo"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9 libgazebo9-dev -y
sudo apt upgrade libignition-math2 -y

## Install some additional plugins
sudo apt-get install ros-melodic-hector-gazebo-plugins -y

## Install Ardupilot Gazebo plugin
echo "!  Installing Gazebo ardupilot plugin"
#git clone https://github.com/khancyr/ardupilot_gazebo
cd $ABS_PATH/external
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$ABS_PATH/models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=$ABS_PATH/worlds:\${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc

echo "!  Reload paths"
. ~/.bashrc

#############################################################################
#				Termination
#############################################################################
## Build!
#echo "!  Build!"
#cd ~/catkin_ws
#catkin_make

## Setup environment variables
echo "!  Setting up enviroment variables"
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

wssource="source ~/catkin_ws/devel/setup.bash"
eval $wssource

## End of script
echo "!  End of enviroment installation..."

## Ask for reboot
while true; do
    read -p "!  Do you want to reboot now? (Required) [Y/n]" yn
    case $yn in
        [Yy]* ) reboot; break;;
        [Nn]* ) echo "!  Please reboot later"; break;;
        * ) echo "!  Please answer yes or no.";;
    esac
done
