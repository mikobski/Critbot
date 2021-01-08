#!/usr/bin/env python3

from __future__ import print_function
from web_robot_communication.srv import ModeChanges,ModeChangesResponse
from mavros_msgs.srv import SetMode,CommandBool
from mavros_msgs.msg import State
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import Header
import rospy
import std_msgs.msg
import sys
import re
import subprocess
import psutil

GUIDED_MODE_SCRIPT = 'guided_mode.py'
PGREP_CMD = 'pgrep -af '
#pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10, latch=True)

def check_if_script_running(mode):
    output = subprocess.check_output(PGREP_CMD+mode, shell=True)
    splitted_output = output.split()
    print(splitted_output)
    if splitted_output[2] == mode:
        return psutil.Process(int(splitted_output[0]))
    else:
        return None


def handle_mode_changes(req):
        #    header = Header()
        #    header.stamp = rospy.get_rostime()
        #    header.frame_id = ''
        #    header.seq = 0
        #    pos = GeoPointStamped()
        #    pos.header = header
        #    pos.position.latitude = 54.0
        #    pos.position.longitude = 18.0
        #    pos.position.altitude = 10.0
        #    pub.publish(pos)
    guided_check = check_if_script_running(GUIDED_MODE_SCRIPT)
    mode_type = re.search('\"(.+?)\"', str(req)).group(1)
    rospy.wait_for_service('/mavros/set_mode')
    mode_change_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)       
    if mode_type in ['guided']:
        if guided_check is None:
            print('That script is not running')
            print('1')
            try:
                print('3')
                resp = mode_change_srv(base_mode=0, custom_mode='GUIDED')
                print('4')
                script = subprocess.Popen([sys.executable, GUIDED_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                return ModeChangesResponse('Mode changed to guided')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            return ModeChangesResponse('Mode guided already running') 
    elif mode_type in ['emergency_stop']:
        try:
            resp = mode_change_srv(base_mode=0, custom_mode='HOLD')
            return ModeChangesResponse('Robot disarmed')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    else:
        return ModeChangesResponse('This kind of mode does not exist')
 

def mode_changes_server():
    rospy.init_node('mode_changes_server')
    s = rospy.Service('mode_changes', ModeChanges, handle_mode_changes)
    print("Ready to change mode.")
    rospy.spin()


if __name__ == "__main__":
    
    mode_changes_server()
