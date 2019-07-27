#!/usr/bin/env python

import sys
import math

import rospy
from std_msgs.msg import Time
from sensor_msgs.msg import Joy
import std_msgs.msg

from mav_nonlinear_mpc.cfg import ThrustRescalerConfig

class JoyEcho(object):
    def __init__(
        self, 
        mav_name, 
        uav_num,
        invert_arr=[]
    ):
        self.msg_to_publish = Joy()
        self.pub = rospy.Publisher('/mavros/rc/in_remapped_and_inverted', Joy, queue_size=1)
        #self.pub = rospy.Publisher('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        #rospy.Subscriber('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust_raw', RollPitchYawrateThrust, self.read_callback)
        rospy.Subscriber('/mavros/rc/in_remapped/', Joy, self.read_callback)
        self.invert_arr = invert_arr

    def invert_msg(self, msg):
        for axis in self.invert_axes:
            msg.axes[axis] = -msg.axes[axis]
        return msg
    
    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg # Note no deepcopy here
        self.msg_to_publish = self.invert_msg(self.msg_to_publish)
        #print("Scaled RC message")
        #self.msg_to_publish.yaw_rate *= -1 # flip yawrate
        self.pub.publish(self.msg_to_publish)

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) < 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM [AXES_TO_INVERT]")
    sys.exit()
mav_name = myargs[1]
uav_num = str(myargs[2])

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    add_input_delay = False
    input_delay = 0.15
    rospy.init_node(
        'joy_invert' + uav_num,
        anonymous=False
    )

    invert_arr = []
    for i, arg in enumerate(myargs):
        if i >= 3:
            invert_arr.append(int(arg))
    echo_node = JoyEcho(
        mav_name, 
        uav_num, 
        invert_arr
    )
    print('-----------------------')
    rospy.spin()
