#!/bin/bash
#killall -9 gzserver gzclient
rosnode kill -a
killall -9 roscore
killall -9 rosmaster
