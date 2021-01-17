#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from waypoint_navigation.srv import *

def add_waypoints_client(points):
    rospy.wait_for_service('add_waypoints')
    try:
        add_waypoints = rospy.ServiceProxy('add_waypoints', AddWaypoints)
        resp1 = add_waypoints(points)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        points = sys.argv[1]
        print(points)
    else:
        print(usage())
        sys.exit(1)
    add_waypoints_client(points)