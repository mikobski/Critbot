#!/usr/bin/env python2
# license removed for brevity
import rospy
import mavros
from mavros_msgs.msg import PositionTarget

def talker():
    pub_local = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    msg_local = PositionTarget()
    msg_local.header.stamp = rospy.Time.now()
    msg_local.header.frame_id = "mavi_test_local"

    msg_local.position.x = 23.23
    msg_local.position.y = 150.2
    msg_local.position.z = 0.0
    msg_local.type_mask = 3576

    while not rospy.is_shutdown():	
        pub_local.publish(msg_local)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

