#!/usr/bin/env python

import sys
import math

import rospy
from std_msgs.msg import Time
from mav_msgs.msg import RollPitchYawrateThrust
import std_msgs.msg

class Echo_From_Vrep(object):
    def __init__(
        self, 
        mav_name, 
        uav_num="1", 
        add_input_delay=False, 
        input_delay=0
    ):
        self.add_input_delay = add_input_delay
        self.input_delay = rospy.Duration(input_delay)
        self.updated = False
        self.msg_to_publish = RollPitchYawrateThrust()
        self.pub = rospy.Publisher('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust_scaled', RollPitchYawrateThrust, queue_size=1)
        rospy.Subscriber('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, self.read_callback)

        self.scale_factors_initialized = False

    # Input in degrees for simplicity
    def initialise_scale_factors(self, max_rollpitch, max_yawrate, hover_thrust, min_thrust, max_thrust):
        self.max_rollpitch = math.radians(max_rollpitch)
        self.max_yawrate = math.radians(max_yawrate)
        self.hover_thrust = hover_thrust
        self.min_thrust = min_thrust
        self.max_thrust = max_thrust

        self.scale_factors_initialized = True

    def scale_msg(self, msg):
        input_max_yawrate = math.radians(90)
        input_max_rollpitch = math.radians(45)
        input_min_thrust = 0
        input_max_thrust = 1
        thrust_grad = (self.hover_thrust - self.min_thrust) * 2
        msg.roll = (msg.roll / input_max_rollpitch) * self.max_rollpitch
        msg.pitch = (msg.pitch / input_max_rollpitch) * self.max_rollpitch

        msg.thrust.z = self.hover_thrust + (msg.thrust.z - 0.5) * thrust_grad
        msg.thrust.z = min(msg.thrust.z, 1)
        msg.thrust.z = max(msg.thrust.z, 0)
        return msg

    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg
        self.msg_to_publish = self.scale_msg(self.msg_to_publish)
        self.pub.publish(self.msg_to_publish)

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    sys.exit()
mav_name = myargs[1]

uav_num = str(myargs[2])

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    add_input_delay = False
    input_delay = 0.15
    rospy.init_node('attitude_cmd_echo_' + uav_num, anonymous=False)

    echo_node = Echo_From_Vrep(mav_name, uav_num, add_input_delay, input_delay)
    echo_node.initialise_scale_factors(25, 90, 0.2, 0, 1)
    rospy.spin()