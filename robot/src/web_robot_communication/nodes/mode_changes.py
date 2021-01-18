#!/usr/bin/env python

from __future__ import print_function
from web_robot_communication.srv import ModeChanges, ModeChangesResponse
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import rospy

actual_mode = ''
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0, latch=False)

def autonomic_callback(data):
    if actual_mode == 'autonomic':
        pub.publish(data)

def autonomic_mode():
    rospy.Subscriber("/critbot/autonomic_control", Twist, autonomic_callback)

def manual_callback(data):
    if actual_mode == 'manual':
        pub.publish(data)

def manual_mode():
    rospy.Subscriber("/critbot/manual_control", Twist, manual_callback)

def handle_mode_changes(msg):
    global actual_mode    
    mode_type = msg.mode
    if mode_type in ['manual']:
        if actual_mode != 'manual':
            try:
                actual_mode = 'manual'
                manual_mode()
                return ModeChangesResponse('manual')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ModeChangesResponse("Error: %s"%e)
        else:
            return ModeChangesResponse('manual') 
    elif mode_type in ['autonomic']:
        if actual_mode != 'autonomic':
            try:
                actual_mode = 'autonomic'
                autonomic_mode()
                return ModeChangesResponse('autonomic')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return ModeChangesResponse("Error: %s"%e)
        else:
            return ModeChangesResponse('autonomic')
    elif mode_type in ['emergency_stop']:
        try:
            actual_mode = 'hold'
            return ModeChangesResponse('emergency_stop')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return ModeChangesResponse("Error: %s"%e)
    else:
        return ModeChangesResponse('not_exist')

def mode_changes_server():
    rospy.init_node('mode_changes_server')
    s = rospy.Service('critbot/mode_changes', ModeChanges, handle_mode_changes)
    print("Ready to change mode.")
    rospy.spin()

if __name__ == "__main__":
    mode_changes_server()
