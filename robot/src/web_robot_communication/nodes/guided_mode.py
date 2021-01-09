#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

value = 0
pub = rospy.Publisher('/mavros/setpoint_velocity_unstamped', Twist, queue_size=10, latch=True)

def callback(data):
    rospy.loginfo(data)
    pub.publish(data)

def listener():
    rospy.Subscriber("/web/manual_control", Twist, callback)  
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listener_and_publisher', anonymous=True)
    listener()

