#!/usr/bin/env python

from __future__ import print_function
from web_robot_communication.srv import ModeChanges,ModeChangesResponse
from mavros_msgs.srv import SetMode,CommandBool
from mavros_msgs.msg import State
import rospy
import std_msgs.msg
import sys
import re
import subprocess
import psutil

MANUAL_MODE_SCRIPT = 'manual_mode.py'
AUTONOMOUS_MODE_SCRIPT = 'autonomous_mode.py'
PGREP_CMD = 'pgrep -af '

#def manual_callback(data):
#     rospy.loginfo(data.armed)
#     rospy.loginfo(data.mode)
#     rospy.Service('mode_changes', ModeChanges, handle_mode_changes)
#     if 
#     if not data.armed:
#         try:




def check_if_script_running(mode):
    output = subprocess.check_output(PGREP_CMD+mode, shell=True)
    splitted_output = output.split()
    print(splitted_output)
    if splitted_output[2] == mode:
        return psutil.Process(int(splitted_output[0]))
    else:
        return None

def handle_mode_changes(req):
    autonomous_check = check_if_script_running(AUTONOMOUS_MODE_SCRIPT)
    manual_check = check_if_script_running(MANUAL_MODE_SCRIPT)
    mode_type = re.search('\"(.+?)\"', str(req)).group(1)
    if mode_type in ['autonomous']:
        if manual_check is None:
            print('That script is not running')
        else:
            manual_check.terminate()
        if autonomous_check is None:
            print('That script is not running')
            script = subprocess.Popen([sys.executable, AUTONOMOUS_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            return ModeChangesResponse('Mode changed to autonomous')
        else:
            return ModeChangesResponse('Mode autonomous already running') 
             
    elif mode_type in ['manual']:
        if autonomous_check is None:
            print('That script is not running')
        else:
            autonomous_check.terminate()
        if manual_check is None:
            print('That script is not running')
            #rospy.Subscriber("mavros/state", State, manual_callback)
            #try:
                #subprocess.run(['rosrun mavros mavsys mode -c 15'])
                #subprocess.run(['rosrun mavros mavsafety arm'])
                    #
            rospy.wait_for_service('set_mode')        
            try:
                manual_srv = rospy.ServiceProxy('set_mode', SetMode)
                base_mode = 216
                resp = manual_srv(base_mode)
                return resp
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            #except:
                #print('Could not change mode or arm')
            script = subprocess.Popen([sys.executable, MANUAL_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            return ModeChangesResponse('Mode changed to manual')
        else:
            return ModeChangesResponse('Mode manual already running') 
    elif mode_type in ['emergency_stop']:
        try:
            subprocess.run(['rosrun mavros mavsafety disarm'])
        except:
            print('Could not disarm, probably already disarm')
    else:
        return ModeChangesResponse('This kind of mode does not exist')
    

def mode_changes_server():
    rospy.init_node('mode_changes_server')
    s = rospy.Service('mode_changes', ModeChanges, handle_mode_changes)
    print("Ready to change mode.")
    rospy.spin()

if __name__ == "__main__":
    
    mode_changes_server()
