#!/usr/bin/env python
from __future__ import print_function
from pyproj import Proj, transform
import rospy
from waypoint_navigation.srv import SetWaypoints, SetWaypointsResponse
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

current_waypoints = []
current_wp_index = 0

def conv_geo_to_odom(in_geo, ref_geo, ref_odom):
    proj_cart = Proj(init='epsg:3857') # Projection like in Google Maps (carthesian in meters)
    proj_geo = Proj(init='epsg:4326') # Projection with latitude and longitude
    ref_cart = transform(proj_geo, proj_cart, ref_geo[0], ref_geo[1])
    print("ref_cart = %f, %f" % ref_cart)
    in_cart = transform(proj_geo, proj_cart, in_geo[0], in_geo[1])
    print("geo_diff %f, %f" % ((in_cart[0] - ref_cart[0]), (in_cart[1] - ref_cart[1])))
    result = (-(in_cart[0] - ref_cart[0] + ref_odom[0]), in_cart[1] - ref_cart[1] - ref_odom[1])
    return result

def handle_set_waypoints(msg):
    global current_wp_index
    global current_waypoints
    ref_geo_msg = rospy.wait_for_message(rospy.get_param("~navsat_src"), NavSatFix)
    ref_geo = (ref_geo_msg.latitude, ref_geo_msg.longitude)
    ref_odom_msg = rospy.wait_for_message(rospy.get_param("~odom_src"), Odometry)
    ref_odom = (ref_odom_msg.pose.pose.position.x, ref_odom_msg.pose.pose.position.y)
    print("ref_odom= %f, %f" % ref_odom)
    current_waypoints = []
    current_wp_index = 0
    for waypoint in msg.waypoints:
        pose_geo = (waypoint.x, waypoint.y)
        pose_odom = conv_geo_to_odom(pose_geo, ref_geo, ref_odom)
        print("Odleglosc %f, %f" % pose_odom)
        current_waypoints.append(pose_odom)
    print("---")
    return SetWaypointsResponse(0, "OK")

def server():
    global current_wp_index
    rospy.init_node('waypoint_navigation')
    s_get_waypoints = rospy.Service('set_waypoints', SetWaypoints, handle_set_waypoints)
    #s_clear_waypoints = rospy.Service('clear_waypoints', AddWaypoints, handle_add_waypoints)
    print("Ready to set waypoints.")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():        
        if len(current_waypoints) > current_wp_index:
            print(current_waypoints)
            current_wp = current_waypoints[current_wp_index]
            current_wp_index += 1
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = current_wp[0]
            goal.target_pose.pose.position.y = current_wp[1]
            goal.target_pose.pose.orientation.w = 1.0      
            client.send_goal(goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                rospy.loginfo(client.get_result())
        rate.sleep()

if __name__ == "__main__":
    server()