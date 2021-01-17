#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import rospy

def callback(data):
    pub= rospy.Publisher("/odom/linear_twist", Vector3, queue_size=0, latch=False)
    pub.publish(data.twist.twist.linear)

def odom_sub():
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()

if __name__ == "__main__":
    
    try:
        rospy.init_node('convert_odom_to_twist', anonymous=True)
        odom_sub()
    except rospy.ROSInterruptException:
        pass

