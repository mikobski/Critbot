#!/usr/bin/env python

from __future__ import print_function

from waypoint_navigation.srv import AddWaypoints, AddWaypointsResponse
from haversine import haversine
from sensor_msgs.msg import NavSatFix
import rospy

GPS_TOPIC = "/fix"
WAYPOINTS = []

def convert_geographical_to_odom(reference_point, point):
    distance_y = haversine((reference_point[0], 0), (point[0], 0))
    distance_x = haversine((0, reference_point[1]), (0, point[1]))

    if reference_point[0] > point[0]:
        distance_y = -distance_y
    if reference_point[1] > point[1]:
        distance_x = -distance_x
    
    return (distance_x, distance_y)

def handle_add_waypoints(waypoints):
    print(waypoints.points)
    converted_waypoints = []
    #https://www.latlong.net/place/gda-sk-poland-1379.html
    reference_point_msg = rospy.wait_for_message(GPS_TOPIC, NavSatFix)
    #reference_point = (54.372158, 18.638306)
    reference_point = (reference_point_msg.latitude, reference_point_msg.longitude)
    for waypoint in waypoints.points:
        waypoint_geo = (waypoint.x, waypoint.y)
        print("Wspolrzedne geograficzne %f, %f" % waypoint_geo)
        converted_waypoint = convert_geographical_to_odom(reference_point, waypoint_geo)
        print("Odleglosc %f, %f" % converted_waypoint)
        WAYPOINTS.append(converted_waypoint)
    
    return AddWaypointsResponse("Dodano waypointy!")

def handle_clear_waypoints():
    WAYPOINTS = []
    return AddWaypointsResponse("Usunieto waypointy")

def add_waypoints_server():
    rospy.init_node('add_waypoints_server')
    s_addwaypoints = rospy.Service('add_waypoints', AddWaypoints, handle_add_waypoints)
    s_clearwaypoints = rospy.Service('clear_waypoints', AddWaypoints, handle_add_waypoints)
    print("Ready to add waypoints.")
    rospy.spin()

def tryfun():
    print("Ready try fun")

if __name__ == "__main__":
    add_waypoints_server()
    tryfun()