#!/usr/bin/env python

import sys
import math

import rospy
from std_msgs.msg import Time
from mav_msgs.msg import RollPitchYawrateThrust
import std_msgs.msg

from dynamic_reconfigure.server import Server

class Echo_From_Vrep(object):
    def __init__(
        self, 
        mav_name, 
        uav_num="1", 
        add_input_delay=False, 
        input_delay=0,
        invert_arr=[False, False, False, False]
    ):
        if (len(invert_arr) != 4):
            print("Invalid invert_arr. Please check")
            sys.exit()
        self.add_input_delay = add_input_delay
        self.input_delay = rospy.Duration(input_delay)
        self.updated = False
        self.msg_to_publish = RollPitchYawrateThrust()
        self.pub = rospy.Publisher('/mavros/setpoint_raw/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        #self.pub = rospy.Publisher('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        #rospy.Subscriber('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust_raw', RollPitchYawrateThrust, self.read_callback)
        rospy.Subscriber('/' + mav_name +  uav_num + '/' + 'thrust_scaling_factor', Float32, self.update_scaling_factor_cb)
        rospy.Subscriber('/mavros/setpoint_raw/roll_pitch_yawrate_thrust_N', RollPitchYawrateThrust, self.read_callback)
        self.invert_roll = invert_arr[0]
        self.invert_pitch = invert_arr[1]
        self.invert_yawrate = invert_arr[2]
        self.invert_thrust = invert_arr[3]
        self.scale_factors_initialized = False

    # Input in degrees for simplicity
    def initialise_scale_factors(
            self, 
            thrust_scaling_factor
        ):
        self.thrust_scaling_factor = thrust_scaling_factor
        print("thrust_scaling_factor:")
        print(self.thrust_scaling_factor)
        self.scale_factors_initialized = True

    def scale_msg(self, msg):
        if self.scale_factors_initialized == False:
            msg.thrust.z = 0
            return msg
        max_output_thrust = 1.0
        msg.thrust.z = msg.thrust.z * self.thrust_scaling_factor
        msg.thrust.z = min(msg.thrust.z, max_output_thrust)
        msg.thrust.z = max(msg.thrust.z, 0)
        return msg

    def invert_msg(self, msg):
        if self.invert_roll:
            msg.roll = -msg.roll
        if self.invert_pitch:
            msg.pitch = -msg.pitch
        if self.invert_yawrate:
            msg.yaw_rate = -msg.yaw_rate
        if self.invert_thrust:
            msg.thrust.z = -msg.thrust.z
        msg.thrust.z = max(msg.thrust.z, 0)
        return msg
    
    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg
        self.msg_to_publish = self.scale_msg(self.msg_to_publish)
        self.msg_to_publish = self.invert_msg(self.msg_to_publish)
        #print("Scaled RC message")
        #self.msg_to_publish.yaw_rate *= -1 # flip yawrate
        self.pub.publish(self.msg_to_publish)

    def update_scaling_factor_cb(self, msg):
        print("Python rescaler updating thrust scaling factor")
        print("Thrust scaling factor: " + str(msg.data))
        self.thrust_scaling_factor = msg.data

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 8:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM THRUST_SCALING_FACTOR \
INVERT_ROLL INVERT_PITCH INVERT_YAWRATE INVERT_THRUST")
    sys.exit()
mav_name = myargs[1]
uav_num = str(myargs[2])
input_thrust_scaling_factor = float(myargs[3])
invert_roll = True if (str(myargs[4]) == "True") else False
invert_pitch = True if (str(myargs[5]) == "True") else False
invert_yawrate = True if (str(myargs[6]) == "True") else False
invert_thrust = True if (str(myargs[7]) == "True") else False

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    print("==================================")
    print("WARNING: THIS SCRIPT WILL NOT WORK AS INTENDED WITHOUT BUILDING APPROPRIATE CFG FILE.")
    print("THE CFG FILE IS NOT BUILT BY DEFAULT BECAUSE OF ISSUES WITH HAVING 2 CFG FILES")
    print("==================================")
    add_input_delay = False
    input_delay = 0.15
    rospy.init_node(
        'attitude_cmd_echo_' + uav_num,
        anonymous=False
    )

    echo_node = Echo_From_Vrep(
        mav_name, 
        uav_num, 
        add_input_delay, 
        input_delay,
        [invert_roll, invert_pitch, invert_yawrate, invert_thrust]
    )

    echo_node.initialise_scale_factors(
        input_thrust_scaling_factor
    )

    rospy.spin()
