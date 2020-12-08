#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import std_msgs.msg
from web_robot_communication.srv import *

def mode_changes_client(mode):
    rospy.wait_for_service('mode_changes')
    try:
        mode_changes = rospy.ServiceProxy('mode_changes', ModeChanges)
        resp = mode_changes(mode)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "Bad arguments"

if __name__ == "__main__":
    if len(sys.argv) == 2:
        mode = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print(mode_changes_client(mode))