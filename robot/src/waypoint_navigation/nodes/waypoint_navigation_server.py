#!/usr/bin/env python
from __future__ import print_function
import rospy
from waypoint_navigation.srv import SetWaypoints, SetWaypointsResponse
from waypoint_navigation.srv import CancelMission, CancelMissionResponse
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from conv_geo_to_odom import conv_geo_to_odom

current_waypoints = []
current_wp_index = 0
client = None

def handle_set_waypoints(msg):
    global current_wp_index
    global current_waypoints
    global client
    ref_geo_msg = rospy.wait_for_message(rospy.get_param("~navsat_src"), NavSatFix)
    ref_geo = (ref_geo_msg.latitude, ref_geo_msg.longitude)
    ref_odom_msg = rospy.wait_for_message(rospy.get_param("~odom_src"), Odometry)
    ref_odom = (ref_odom_msg.pose.pose.position.x, ref_odom_msg.pose.pose.position.y)
    current_waypoints = []
    current_wp_index = 0
    client.cancel_all_goals()
    for waypoint in msg.waypoints:
        pose_geo = (waypoint.x, waypoint.y)
        pose_odom = conv_geo_to_odom(pose_geo, ref_geo, ref_odom)
        current_waypoints.append(pose_odom)
    next_goal()
    return SetWaypointsResponse(0, "OK")

def handle_cancel_mission(msg):
    global client
    global current_waypoints
    global current_wp_index
    client.cancel_all_goals()
    client.stop_tracking_goal()
    current_waypoints = []
    current_wp_index = 0
    return CancelMissionResponse(0, "OK")

def next_goal_handler(status=None, result=None):
    global current_wp_index
    current_wp_index+=1
    next_goal()

def next_goal(status=None, result=None):
    global current_wp_index
    global current_waypoints
    if len(current_waypoints) > current_wp_index:
        current_wp = current_waypoints[current_wp_index]
        print(current_wp)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = current_wp[0]
        goal.target_pose.pose.position.y = current_wp[1]
        goal.target_pose.pose.orientation.w = 1.0 
        client.send_goal(goal, done_cb=next_goal_handler)
    else:
        current_waypoints = []
        current_wp_index = 0
        print("Mission completed")

def server():
    global current_wp_index
    global current_waypoints
    global client
    rospy.init_node('waypoint_navigation')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    s_set_waypoints = rospy.Service('set_waypoints', SetWaypoints, handle_set_waypoints)
    s_clear_waypoints = rospy.Service('cancel_mission', CancelMission, handle_cancel_mission)
    pub_status = rospy.Publisher("/mission_status", String, queue_size=0, latch=True)
    print("Ready to set waypoints.")
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():        
        if len(current_waypoints) > 0 and len(current_waypoints) > current_wp_index:        
            if client.get_state() == GoalStatus.REJECTED:
                rospy.logerr("Goal rejected!")
            pub_status.publish("IN_MISSION")
        else:
            pub_status.publish("IDLE")
        rate.sleep()

if __name__ == "__main__":
    server()   
