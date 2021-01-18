#!/usr/bin/env python
from __future__ import print_function
from haversine import haversine
import rospy
from waypoint_navigation.srv import SetWaypoints, SetWaypointsResponse
from sensor_msgs.msg import NavSatFix

currentWaypoints = []

def conv_geo_to_odom(reference_point, point):
    distance_y = haversine((reference_point[0], 0), (point[0], 0))
    distance_x = haversine((0, reference_point[1]), (0, point[1]))
    if reference_point[0] > point[0]:
        distance_y = -distance_y
    if reference_point[1] > point[1]:
        distance_x = -distance_x
    return (distance_x, distance_y)

def handle_set_waypoints(msg):
    print(msg.waypoints)
    #https://www.latlong.net/place/gda-sk-poland-1379.html
    #reference_point_msg = rospy.wait_for_message(GPS_TOPIC, NavSatFix)
    reference_point = (54.372158, 18.638306)
    print(rospy.get_param("~navsat_src"))
    #reference_point = (reference_point_msg.latitude, reference_point_msg.longitude)
    for waypoint in msg.waypoints:
        pose_geo = (waypoint.x, waypoint.y)
        print("Wspolrzedne geograficzne %f, %f" % pose_geo)
        pose_odom = conv_geo_to_odom(reference_point, pose_geo)
        print("Odleglosc %f, %f" % pose_odom)
        currentWaypoints.append(pose_odom)
    return SetWaypointsResponse(0, "OK")

def server():
    rospy.init_node('waypoint_navigation')
    s_get_waypoints = rospy.Service('set_waypoints', SetWaypoints, handle_set_waypoints)
    #s_clear_waypoints = rospy.Service('clear_waypoints', AddWaypoints, handle_add_waypoints)
    print("Ready to set waypoints.")
    rospy.spin()

if __name__ == "__main__":
    server()