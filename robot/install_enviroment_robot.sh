#!/bin/bash

ABS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd $ABS_PATH

#############################################################################
#
#############################################################################

## Bash script for setting up ROS Melodic environment for Ubuntu LTS (18.04). 
## Includes:
## - Common dependencies libraries and tools
## - MAVROS

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

## Get ROS/Gazebo
echo "!  Installing ROS"
sudo apt install ros-melodic-desktop -y

## Install rosinstall and other dependencies
echo "!  Installing rosinstall and other dependecies"
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-rqt -y
#sudo apt-get install python-catkin-tools -y

## Initialize rosdep
echo "!  Initializing rosdep"
sudo rosdep init
rosdep update

### Create catkin workspace
echo "!  Creating workspace"
mkdir -p $ABS_PATH/src
mkdir -p $ABS_PATH/external
cd $ABS_PATH
#catkin init
#wstool init src

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
# 			     Ardupilot Sitl 
#	https://ardupilot.org/dev/docs/building-setup-linux.html
#############################################################################
### Clone ardupiltot from git
#echo "!  Cloning Ardupilot"
#cd $ABS_PATH/external
#git clone https://github.com/ArduPilot/ardupilot.git
#cd $ABS_PATH/external/ardupilot
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
#

############################
#	Cameras
############################
export ROS_VER=melodic
sudo apt-get install ros-$ROS_VER-realsense2-camera
sudo apt-get install ros-$ROS_VER-depthimage-to-laserscan
cd $ABS_PATH/external
git clone https://github.com/Slamtec/rplidar_ros.git
#############################################################################
#				Termination
#############################################################################
## Build!
echo "!  Build!"
sudo apt install catkin
cd $ABS_PATH
catkin_make

## Setup environment variables
echo "!  Setting up enviroment variables"
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

wssource="source $ABS_PATH/devel/setup.bash"
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

