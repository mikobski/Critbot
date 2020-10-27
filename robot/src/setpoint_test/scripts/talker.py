#!/usr/bin/env python
# license removed for brevity
import rospy
import mavros
#from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import PositionTarget

def talker():
    #pub_global = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    pub_local = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    msg_local = PositionTarget()
    msg_local.header.stamp = rospy.Time.now()
    msg_local.header.frame_id = "mavi_test_local"

    msg_local.position.x = 23.23
    msg_local.position.y = 150.2
    msg_local.position.z = 0.0
    msg_local.type_mask = 3576

    '''
    msg_local.FRAME_LOCAL_NED=1
    msg_local.FRAME_LOCAL_OFFSET_NED=7
    msg_local.FRAME_BODY_NED=8
    msg_local.FRAME_BODY_OFFSET_NED=9
    msg_local.IGNORE_PX=1
    msg_local.IGNORE_PY=2
    msg_local.IGNORE_PZ=4
    msg_local.IGNORE_VX=8
    msg_local.IGNORE_VY=16
    msg_local.IGNORE_VZ=32
    msg_local.IGNORE_AFX=64
    msg_local.IGNORE_AFY=128
    msg_local.IGNORE_AFZ=256
    msg_local.FORCE=512
    msg_local.IGNORE_YAW=1024
    msg_local.IGNORE_YAW_RATE=2048
    msg_local.coordinate_frame
    msg_local.type_mask
    
    msg_local.velocity.x
    msg_local.velocity.y
    msg_local.velocity.z

    msg_local.acceleration_or_force.x
    msg_local.acceleration_or_force.y
    msg_local.acceleration_or_force.z

    msg_local. yaw
    msg_local. yaw_rate

    

    msg_global = GlobalPositionTarget()
    msg_global.header.stamp = rospy.Time.now()
    msg_global.header.frame_id = "mavi_test_global"
    msg_global.latitude = 54.18
    

    msg_global.coordinate_frame
    msg_global.FRAME_GLOBAL_INT = 5
    msg_global.FRAME_GLOBAL_REL_ALT = 6
    msg_global.FRAME_GLOBAL_TERRAIN_ALT = 11

    msg_global.type_mask
    msg_global.IGNORE_LATITUDE = 1 # Position ignore flags
    msg_global.IGNORE_LONGITUDE = 2
    msg_global.IGNORE_ALTITUDE = 4
    msg_global.IGNORE_VX = 8 # Velocity vector ignore flags
    msg_global.IGNORE_VY = 16
    msg_global.IGNORE_VZ = 32
    msg_global.IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
    msg_global.IGNORE_AFY = 128
    msg_global.IGNORE_AFZ = 256
    msg_global.FORCE = 512 # Force in af vector flag
    msg_global.IGNORE_YAW = 1024
    msg_global.IGNORE_YAW_RATE = 2048

    msg_global.latitude
    msg_global.longitude
    msg_global.altitude # in meters, AMSL or above terrain
    ##geometry_msgs/Vector3 velocity
    ##geometry_msgs/Vector3 acceleration_or_force
    msg_global.yaw
    msg_global.yaw_rate

    '''

    while not rospy.is_shutdown():	
        #pub_global.publish(msg_global)
        pub_local.publish(msg_local)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

