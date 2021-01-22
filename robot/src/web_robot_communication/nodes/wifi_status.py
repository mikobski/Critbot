#!/usr/bin/env python

import os
import re
import subprocess
import rospy
import std_msgs.msg
from std_msgs.msg import String


def wifi_status_server():
    pub = rospy.Publisher('/wifi_status', String, queue_size=10)
    rospy.init_node('wifi_status_server')
    rate = rospy.Rate(10) # 10Hz
    
    pattern = r'signal avg:\t(-\d+ dBm)'

    while not rospy.is_shutdown():
        result = subprocess.check_output(['iw', 'dev', 'wlx001f1f697d88', 'station', 'dump'])
        stations = len([m.start() for m in re.finditer("Station", result)])

        if stations != 1:
            status_string = ""
        else :
            status_string = (re.findall(pattern, result))[0]
            
        # rospy.loginfo(status_string) # Debug
        pub.publish(status_string)
        rate.sleep()


if __name__ == "__main__":
    try:
        wifi_status_server()
    except rospy.ROSInterruptException:
        pass
