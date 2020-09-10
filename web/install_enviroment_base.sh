#!/bin/bash

#############################################################################
#
#############################################################################

## Bash script for setting up ROS Melodic environment for Ubuntu LTS (18.04). 
## Includes:
## - Common dependencies libraries and tools

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

## Delete exisitng ROS
#echo "!  Remove existing ROS"
#sudo apt-get remove ros-* -y
#sudo apt-get autoremove -y
#sudo rm -r /etc/ros
#sudo rm -r ~/.ros

# Ubuntu Config
echo "!  Remove modemmanager"
#sudo apt-get remove modemmanager -y
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

## Create catkin workspace
echo "!  Creating workspace"
export WEB_DIR=$PWD
mkdir -p $WEB_DIR/src
cd $WEB_DIR
#catkin init
#wstool init src

#############################################################################
#				Termination
#############################################################################
## Build!
echo "!  Build!"
cd $WEB_DIR
catkin_make

## Setup environment variables
echo "!  Setting up enviroment variables"
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

wssource="source $WEB_DIR/devel/setup.bash"
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

