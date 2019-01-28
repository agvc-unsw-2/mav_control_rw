#!/usr/bin/env python

import sys

import rospy
from std_msgs.msg import Time
from mav_msgs.msg import RollPitchYawrateThrust
import std_msgs.msg

class Echo_From_Vrep(object):
    def __init__(self, mav_name, uav_num):
        self.updated = False
        self.msg_to_publish = RollPitchYawrateThrust()
        self.pub = rospy.Publisher("/simulation/uav" + uav_num + "/command/roll_pitch_yawrate_thrust", RollPitchYawrateThrust, queue_size=1)

        rospy.Subscriber('/sim_time', Time, self.write_callback)
        rospy.Subscriber('/' + mav_name + '/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, self.read_callback)

    def read_callback(self, msg):
        #print(data.angular_velocities)
        self.msg_to_publish = msg
        self.updated = True

    def write_callback(self, msg):
        if self.updated == True:
            self.msg_to_publish.header.stamp = msg.data
            self.pub.publish(self.msg_to_publish)
            self.updated = False

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    sys.exit()
mav_name = myargs[1]

uav_num = str(myargs[2])

if __name__ == '__main__':
    print("Starting...")
    rospy.init_node('attitude_cmd_echo_' + uav_num, anonymous=False)
    echo_node = Echo_From_Vrep(mav_name, uav_num)
    rospy.spin()