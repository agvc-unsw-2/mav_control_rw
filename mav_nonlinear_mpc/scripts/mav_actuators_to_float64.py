#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from mav_msgs.msg import Actuators


myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    #print(uav_num)
    #print(mav_name)
    #print(myargs)
    sys.exit()
mav_name = myargs[1]
#mav_name = 'ardrone'
#mav_name = 'firefly'

uav_num = str(myargs[2])

rospy.init_node('ActuatorsToFloat64Pub' + uav_num, anonymous=False)

pub = rospy.Publisher("/simulation/uav" + uav_num + "/command/motor_speed", Float64MultiArray, queue_size=10)

# Publishes angular velocities of mav_msgs/Actuators as Float64MultiArray
def callback(data):
    #print(data.angular_velocities)
    pub.publish(data=data.angular_velocities)
    
# Listens to mav_msgs/Actuators message from mav_linear_mpc
def listener():
    rospy.Subscriber('/' + mav_name + '/command/motor_speed', Actuators, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Starting...")
    listener()
