#!/usr/bin/env python

import sys
import math

import rospy
from std_msgs.msg import Time
from mav_msgs.msg import RollPitchYawrateThrust
import std_msgs.msg

from dynamic_reconfigure.server import Server
from mav_nonlinear_mpc.cfg import ThrustRescalerConfig

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
        #rospy.Subscriber('/' + mav_name +  uav_num + '/command/roll_pitch_yawrate_thrust_raw', RollPitchYawrateThrust, self.read_callback)
        rospy.Subscriber('/mavros/setpoint_raw/roll_pitch_yawrate_thrust_N', RollPitchYawrateThrust, self.read_callback)

        self.scale_factors_initialized = False
        self.ignore_first_dyn_config_callback = True
        self.first_dyn_config_callback = True

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
        max_output_thrust = 0.5 # TODO: Change to not hard coded
        msg.thrust.z = msg.thrust.z * self.thrust_scaling_factor
        msg.thrust.z = min(msg.thrust.z, max_output_thrust)
        msg.thrust.z = max(msg.thrust.z, 0)
        return msg

    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg
        self.msg_to_publish = self.scale_msg(self.msg_to_publish)
        #print("Scaled RC message")
        #self.msg_to_publish.yaw_rate *= -1 # flip yawrate
        self.pub.publish(self.msg_to_publish)

    def dyn_config_callback(self, config, level):
        if(self.ignore_first_dyn_config_callback):
            if (self.first_dyn_config_callback):
                print("Ignoring first dyn_config callback")
                self.first_dyn_config_callback = False
                return config
            else:
                pass #continue to below
        else:
            pass #continue to below
        print("Received reconfigure request")
        print("Config thrust_scaling_factor:")
        print(config.thrust_scaling_factor)
        self.thrust_scaling_factor = config.thrust_scaling_factor
        return config

    def update_scaling_factor(self, new_factor):
        self.thrust_scaling_factor = new_factor

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 4:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM THRUST_SCALING_FACTOR")
    sys.exit()
mav_name = myargs[1]
uav_num = str(myargs[2])
input_thrust_scaling_factor = float(myargs[3])

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    print("==================================")
    print("WARNING: THIS SCRIPT MAY NOT WORK AS INTENDED WITHOUT BUILDING WITH APPROPRIATE CFG FILE.")
    print("THE CFG FILE IS NOT BUILT BY DEFAULT BECAUSE OF ISSUES WITH HAVING 2 CFG FILES")
    print("==================================")
    add_input_delay = False
    input_delay = 0.15
    rospy.init_node('attitude_cmd_echo_' + uav_num, anonymous=False)

    echo_node = Echo_From_Vrep(mav_name, uav_num, add_input_delay, input_delay)

    srv = Server(ThrustRescalerConfig, echo_node.dyn_config_callback)

    echo_node.initialise_scale_factors(
        input_thrust_scaling_factor
    )

    rospy.spin()
