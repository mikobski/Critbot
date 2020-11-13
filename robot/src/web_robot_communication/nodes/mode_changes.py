#!/usr/bin/env python

from __future__ import print_function
from web_robot_communication.srv import ModeChanges,ModeChangesResponse
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
        print('istnieje')
        return psutil.Process(int(splitted_output[0]))
        #p.terminate()
    else:
        print('That script is not running')
        return None

def handle_mode_changes(req):
    global manual_script
    mode_type = re.search('\"(.+?)\"', str(req)).group(1)
    if mode_type in ['autonomous']:
        try:
            check_if_script_running(MANUAL_MODE_SCRIPT).terminate()
        except:
            #print('That script is not running')
            pass
        try:
            check_if_script_running(AUTONOMOUS_MODE_SCRIPT).pid()
            return ModeChangesResponse('Mode already changed to autonomous')
        except:
            print('That script is not running')
            script = subprocess.Popen([sys.executable, AUTONOMOUS_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            return ModeChangesResponse('Mode changed to autonomous')
             
    elif mode_type in ['manual']:
        try:
            check_if_script_running(AUTONOMOUS_MODE_SCRIPT).terminate()
        except:
            print('That script is not running')
        try:
            check_if_script_running(MANUAL_MODE_SCRIPT).pid()
            return ModeChangesResponse('Mode already changed to manual')
        except:
            print('That script is not running')
            script = subprocess.Popen([sys.executable, MANUAL_MODE_SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            return ModeChangesResponse('Mode changed to manual')
    else:
        try:
            check_if_script_running(AUTONOMOUS_MODE_SCRIPT).terminate()
            check_if_script_running(MANUAL_MODE_SCRIPT).terminate()
        except:
            print('That script is not running')
        return ModeChangesResponse('this kind of mode does not exist')
    

def mode_changes_server():
    rospy.init_node('mode_changes_server')
    s = rospy.Service('mode_changes', ModeChanges, handle_mode_changes)
    print("Ready to change mode.")
    rospy.spin()

if __name__ == "__main__":
    
    mode_changes_server()
