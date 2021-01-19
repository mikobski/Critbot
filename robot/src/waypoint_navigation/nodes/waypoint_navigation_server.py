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
    proj_utm = Proj(proj="utm", zone="32U", ellps="WGS84") # Change zone to Poland, Gdansk
    ref_cart = proj_utm(ref_geo[1], ref_geo[0]) # ref_cart[0] - Northing, ref_cart[1] - Easting
    in_cart = proj_utm(in_geo[1], in_geo[0])
    result = (in_cart[1] - ref_cart[1] + ref_odom[0], ref_cart[0] - in_cart[0] + ref_odom[1])
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

def test_single_conv(in_geo, expected, geo_ref, odom_ref):
    result = conv_geo_to_odom(in_geo, geo_ref, odom_ref)
    print("odom = %f (%f[%f]), %f (%f[%f])" % (result[0], expected[0], (result[0]-expected[0])/expected[0]*100, result[1], expected[1], (result[1]-expected[1])/expected[1]*100))

def test_conv():
    geo_refs = (
        (49.899998370215485, 8.900000243996704),
        (49.900059815677295, 8.899926038771365),
        (49.89998484767207, 8.900066494436913)
    )
    odom_refs = (
        (0.125588970714454071, -0.00793456781548877),
        (7.233748723521987, 5.111974719152003),
        (-0.9266802440019012, -4.813575833743801)
    )
    for i in range(len(geo_refs)):
        test_single_conv((49.90008158778205, 8.900055637432734), (9.258129441861051, -3.9731864270136255), geo_refs[i], odom_refs[i])
        test_single_conv((49.899925385678934, 8.900043771216522), (-7.7698147312076316, -3.047976896220604), geo_refs[i], odom_refs[i])
        test_single_conv((49.899941994014085, 8.89993320265618), (-5.951942669191572, 4.779438298430898), geo_refs[i], odom_refs[i])
        test_single_conv((49.89994240335897, 8.90011080084284), (-5.786151777065306, -7.97497992169883), geo_refs[i], odom_refs[i])
        test_single_conv((49.900128080313735, 8.899832716182761), (14.811653527311073, 11.86251022448606), geo_refs[i], odom_refs[i])

if __name__ == "__main__":
    test_conv()
    server()   
