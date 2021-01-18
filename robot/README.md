# Tutorial
## Basic specification
1. Arduino - steering wheels, wheels encoders, RC controller
2. Ublox M8N - GPS
3. T265, Intel Real Sense - IMU
4. D435, Intel Real Sense - image sent to webapp
5. RP lidar S1 - detecting obstacles
6. Intel NUC i5 - central unit
## Installation
To install required software, please run script
```
./install_enviroment_robot.sh
```
If everything went fine, you should be able to start robot software
## Running
To run robot software:
```
cd launch
roslaunch critbot_init.launch
```
This command should run all required packages.
