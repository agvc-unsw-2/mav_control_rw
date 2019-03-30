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
        self.pub = rospy.Publisher('/mavros/setpoint_raw/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        #self.pub = rospy.Publisher('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        rospy.Subscriber('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust_raw', RollPitchYawrateThrust, self.read_callback)
        rospy.Subscriber('/mavros/setpoint_raw/roll_pitch_yawrate_thrust_N', RollPitchYawrateThrust, self.read_callback)

        self.scale_factors_initialized = False

    # Input in degrees for simplicity
    def initialise_scale_factors(
            self, 
            max_rollpitch, 
            max_yawrate, 
            hover_thrust, 
            out_min_thrust, 
            out_max_thrust,
            in_min_thrust,
            in_max_thrust,
            thrust_scaling_factor
        ):
        self.max_rollpitch = math.radians(max_rollpitch)
        self.max_yawrate = math.radians(max_yawrate)
        self.hover_thrust = hover_thrust
        self.out_min_thrust = out_min_thrust
        self.out_max_thrust = out_max_thrust
        self.in_min_thrust = in_min_thrust
        self.in_max_thrust = in_max_thrust
        self.thrust_scaling_factor = thrust_scaling_factor
        self.scale_factors_initialized = True

    def scale_msg(self, msg):
        if self.scale_factors_initialized == False:
            msg.thrust.z = 0
            return msg
        input_max_yawrate = math.radians(90)
        input_max_rollpitch = math.radians(45)
        input_min_thrust = self.in_min_thrust
        input_max_thrust = self.in_max_thrust
        thrust_grad = (self.out_max_thrust - self.out_min_thrust)
        msg.thrust.z = msg.thrust.z * self.thrust_scaling_factor
        msg.thrust.z = (msg.thrust.z / input_max_thrust) * thrust_grad + self.out_min_thrust

        #msg.roll = (msg.roll / input_max_rollpitch) * self.max_rollpitch
        #msg.pitch = (msg.pitch / input_max_rollpitch) * self.max_rollpitch

        #msg.thrust.z = self.hover_thrust + (msg.thrust.z - 0.5) * thrust_grad
        #msg.thrust.z = min(msg.thrust.z, 1)
        #msg.thrust.z = max(msg.thrust.z, 0)
        return msg

    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg
        self.msg_to_publish = self.scale_msg(self.msg_to_publish)
        #print("Scaled RC message")
        self.pub.publish(self.msg_to_publish)

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 4:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM THRUST_SCALING_FACTOR")
    sys.exit()
mav_name = myargs[1]
uav_num = str(myargs[2])
input_thrust_scaling_factor = myargs[3]

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    add_input_delay = False
    input_delay = 0.15
    rospy.init_node('attitude_cmd_echo_' + uav_num, anonymous=False)

    echo_node = Echo_From_Vrep(mav_name, uav_num, add_input_delay, input_delay)

    max_rollpitch = 25 #unused
    max_yawrate = 90 #unused
    output_min_thrust = 0.0
    output_max_thrust = 0.6
    input_min_thrust = 0.0
    input_max_thrust = 0.89
    #input_thrust_scaling_factor = 0.066
    hover_thrust = 0.2 # unused
    echo_node.initialise_scale_factors(
        max_rollpitch,
        max_yawrate,
        hover_thrust,
        output_min_thrust,
        output_max_thrust,
        input_min_thrust,
        input_max_thrust,
        input_thrust_scaling_factor
    )


    rospy.spin()
