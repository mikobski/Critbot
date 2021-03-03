#!/bin/bash
cd ~/Critbot/robot/launch
source /opt/ros/melodic/setup.bash
source ../devel/setup.bash
source ../extern/devel/setup.bash
roslaunch full_critbot_init.launch
