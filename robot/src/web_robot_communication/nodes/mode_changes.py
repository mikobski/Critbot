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
            rospy.wait_for_service('set_mode')           
            try:
                manual_srv = rospy.ServiceProxy('set_mode', SetMode)
                resp = manual_srv(base_mode=220, custom_mode='')
                script = subprocess.Popen([sys.executable, AUTONOMOUS_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                return ModeChangesResponse('Mode changed to autonomous')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            return ModeChangesResponse('Mode autonomous already running')              
    elif mode_type in ['manual']:
        if autonomous_check is None:
            print('That script is not running')
        else:
            autonomous_check.terminate()
        if manual_check is None:
            print('That script is not running')
            rospy.wait_for_service('/mavros/set_mode')
            print('1')
            try:
                print('2')
                manual_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                print('3')
                resp = manual_srv(base_mode=216, custom_mode='')
                print('4')
                script = subprocess.Popen([sys.executable, MANUAL_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                return ModeChangesResponse('Mode changed to manual')
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        else:
            return ModeChangesResponse('Mode manual already running') 
    elif mode_type in ['emergency_stop']:
        try:
            emergency_srv = rospy.ServiceProxy('set_mode', SetMode)
            resp = emergency_srv(base_mode=64, custom_mode='')
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
