#!/usr/bin/env python

# USE LAUNCH FILE SO THIS CAN USE SIM TIME

import sys

import rospy
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import std_msgs.msg
import collections
from copy import deepcopy

class OdomDelay(object):
    def __init__(self, mav_name, uav_num, odom_period, delay_to_add):
        self.ros_duration_to_add = rospy.Duration(delay_to_add)
        print("ROS duration to add: " + str(self.ros_duration_to_add.secs) + "s, " + str(self.ros_duration_to_add.nsecs) + "ns")
        self.delay_to_add = delay_to_add
        print("Delay to add: " + str(self.delay_to_add))
        self.buf_len = (int(delay_to_add / odom_period)) + 10
        #self.messages_to_wait = int(delay_to_add / odom_period)
        self.msg_to_publish = Odometry()

        if 'quad' in mav_name:
            uav_type = 'quad'
        elif 'hex' in mav_name:
            uav_type = 'hex'
            
        self.pub = rospy.Publisher('/' + mav_name + uav_num + "/mavros/local_position/odom", Odometry, queue_size=1)

        self.sub = rospy.Subscriber('/' + mav_name + uav_num + '/ground_truth/mavros/local_position/odom', Odometry, self.read_callback, queue_size=1, buff_size = 2**24)
        self.prev_position = Point()
        self.prev_init = False
        self.msg_buf = collections.deque(maxlen=self.buf_len)

    def read_callback(self, msg):
        # ADD TO BUFFER

        # VERSION: SEND PREVIOUS POSITION
        """
        if (self.prev_init == False):
            self.prev_position = deepcopy(msg.pose.pose.position)
            self.prev_init = True
        else:
            #self.msg_to_publish = Odometry()
            self.msg_to_publish = deepcopy(msg)
            #print("Prev position:")
            #print(self.msg_to_publish)
            self.msg_to_publish.pose.pose.position = deepcopy(self.prev_position)
            self.pub.publish(self.msg_to_publish)
            #self.prev_position = Point()
            self.prev_position = deepcopy(msg.pose.pose.position)
            #print(self.prev_position)
            #print(self.msg_to_publish)
        """
        #self.pub.publish(msg)
        if (self.delay_to_add <= 0.01):
            self.pub.publish(msg)
        else:
            self.msg_to_publish = deepcopy(msg)

            self.msg_buf.append(deepcopy(msg))
            msg_from_buf = self.msg_buf.popleft()
            prev_position = deepcopy(msg_from_buf.pose.pose.position)

            ros_time = rospy.get_rostime()
            msg_time = deepcopy(msg_from_buf.header.stamp)
            time_to_publish = msg_time + self.ros_duration_to_add
            #print_time_msg(ros_time, "ROS time: ")
            #print_time_msg(time_to_publish, "time_to_publish: ")
            if (ros_time >= time_to_publish):
                self.msg_to_publish.header.stamp = ros_time
                self.msg_to_publish.pose.pose.position = deepcopy(prev_position)
                self.pub.publish(self.msg_to_publish)
            else:
                self.msg_buf.appendleft(deepcopy(msg_from_buf))
            
myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    sys.exit()

mav_name = myargs[1]
uav_num = str(myargs[2])

def print_time_msg(time_msg, prefix):
    print(str(prefix) + " " + str(time_msg.secs) + "s, " + str(time_msg.nsecs) + "ns")

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    delay_to_add = 0.25
    odom_period = 0.01
    rospy.init_node('odom_delay' + uav_num, anonymous=False)
    echo_node = OdomDelay(mav_name, uav_num, odom_period, delay_to_add)
    rospy.spin()