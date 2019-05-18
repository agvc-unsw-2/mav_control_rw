#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

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

if 'quad' in mav_name:
    uav_type = 'quad'
elif 'hex' in mav_name:
    uav_type = 'hex'

print(uav_type)

pub = rospy.Publisher("/vrep_" + uav_type + uav_num + "/command/motor_speed_arr", Float64MultiArray, queue_size=10)

# Publishes angular velocities of mav_msgs/Actuators as Float64MultiArray
def callback(data):
    #print(data.angular_velocities)
    pub.publish(data=data.angular_velocities)
    
# Listens to mav_msgs/Actuators message from mav_linear_mpc
def listener():
    rospy.Subscriber('/' + mav_name + uav_num + '/command/motor_speed', Actuators, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Starting...")
    listener()
