#!/usr/bin/env python

import sys
import math
import tf

import rospy
from std_msgs.msg import Time
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import std_msgs.msg
from math import cos
from math import sin

class Echo_From_Vrep(object):
    def __init__(self, mav_name, uav_num, circle_d, altitude, vel_mag):
        self.pub = rospy.Publisher('/' + mav_name + uav_num + '/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
        self.circle_d = circle_d
        self.altitude = altitude
        self.vel_scale_factor = vel_mag/circle_d * math.sqrt(2)
        self.t0 = -1
        #rospy.Subscriber('/mavros/imu/data', Imu, self.write_callback)
        rospy.Subscriber('/sim_time', Time, self.write_callback)
    
    def write_callback(self, msg):
        t_read = msg.data.secs + msg.data.nsecs * 1e-9
        #print(t_read)
        if self.t0 == -1:
            self.t0 = t_read
        t = t_read - self.t0
        traj_msg = MultiDOFJointTrajectory()
        d = self.circle_d
        mu = self.vel_scale_factor
        #print(d)
        #print(mu)
        #print(t)
        p_ref = [d*cos(mu*t), d*sin(mu*t), altitude]
        p_ref_d = [-mu*d*sin(mu*t), mu*d*cos(mu*t), 0.0]
        p_ref_dd = [-mu**2*d*cos(mu*t), -mu**2*d*sin(mu*t), 0.0]
        #print(traj_msg.points)
        traj_point = MultiDOFJointTrajectoryPoint()
        zero_vec = Vector3(0.0, 0.0, 0.0)
        translation = Vector3(p_ref[0], p_ref[1], p_ref[2])

        roll = 0
        pitch = 0
        yaw = math.atan2(p_ref_d[1],p_ref_d[0])

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        vel_lin = Vector3(p_ref_d[0], p_ref_d[1], p_ref_d[2])
        vel_ang = zero_vec
        acc_lin = Vector3(p_ref_dd[0], p_ref_dd[1], p_ref_dd[2])
        acc_ang = zero_vec
        traj_point.transforms = [Transform(translation, rotation)]
        traj_point.velocities = [Twist(vel_lin, vel_ang)]
        traj_point.accelerations = [Twist(acc_lin, acc_ang)]
        traj_point.time_from_start = rospy.Time.from_sec(t)
        traj_msg.points = [traj_point]

        traj_msg.header.stamp = msg.data
        self.pub.publish(traj_msg)

myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    sys.exit()
mav_name = myargs[1]

uav_num = str(myargs[2])

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    rospy.init_node('attitude_cmd_echo_' + uav_num, anonymous=False)
    circle_d = 5.0
    altitude = 1.0
    vel_mag = 2.0
    echo_node = Echo_From_Vrep(mav_name, uav_num, circle_d, altitude, vel_mag)
    rospy.spin()