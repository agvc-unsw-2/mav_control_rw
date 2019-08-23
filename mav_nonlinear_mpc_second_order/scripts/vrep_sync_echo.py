#!/usr/bin/env python

import sys

import rospy
from std_msgs.msg import Float64
from dynamic_reconfigure.msg import Config
import std_msgs.msg

class Echo_From_Vrep(object):
    def __init__(self, mav_name, uav_num):
        self.step_cmd = 1.0
        self.pub = rospy.Publisher("/simulation/step_cmd", Float64, queue_size=1)
        rospy.Subscriber('/' + mav_name + '/PID_attitude_controller/parameter_updates', Config, self.step_callback)
        self.count = 0

    def step_callback(self, msg):
        self.count += 1
        print(msg)
        self.pub.publish(self.count)

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
    echo_node = Echo_From_Vrep(mav_name, uav_num)
    rospy.spin()