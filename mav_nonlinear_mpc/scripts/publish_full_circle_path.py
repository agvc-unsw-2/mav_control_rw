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

class Path_Publisher(object):
    def __init__(self, mav_name, uav_num, circle_r, altitude, vel_mag, ref_time_step, publish_interval):
        rospy.init_node('publish_full_circle_path' + uav_num, anonymous=False)
        self.pub = rospy.Publisher(
            '/' + mav_name + uav_num + '/command/trajectory', MultiDOFJointTrajectory, queue_size=1
        )
        self.circle_r = circle_r
        self.altitude = altitude
        self.vel_scale_factor = vel_mag/circle_r
        self.t0 = -1
        self.t_last_published = -1
        self.publish_interval = publish_interval
        self.ref_time_step = ref_time_step
        self.traj_msg = MultiDOFJointTrajectory()
        self.N = int(publish_interval / ref_time_step) * 2
        self.traj_msg.points = [None]*self.N
        self.rate = rospy.Rate(1.0/publish_interval)
        self.cur_ref = MultiDOFJointTrajectory()
        for i in range(self.N):
            self.traj_msg.points[i] = MultiDOFJointTrajectoryPoint()

        rospy.Subscriber(
            '/' + mav_name + uav_num + '/command/current_reference', 
            MultiDOFJointTrajectory, 
            self.get_cur_ref
        )

    def get_cur_ref(self, data):
        self.cur_ref = data

    def publish_path(self):
        d = self.circle_r
        mu = self.vel_scale_factor
        zero_vec = Vector3(0.0, 0.0, 0.0)
        vel_ang = zero_vec
        acc_ang = zero_vec
        while not rospy.is_shutdown():
            now_secs = rospy.get_time()
            now_obj = rospy.get_rostime()
            if now_secs == 0: # simulation hasn't started
                #print(now_secs)
                pass
            else:
                #print(now_secs)
                if self.t0 == -1:
                    self.t0 = now_secs
                t_since_start = now_secs - self.t0
                if t_since_start < 0:
                    self.t0 = now_secs
                    t_since_start = 0.0
                rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
                # generate 2 * points required before next publish interval
                for i in range(self.N):
                    t = t_since_start + (i * self.ref_time_step)
                    p_ref = Vector3(d*cos(mu*t), d*sin(mu*t), self.altitude)
                    v_ref = Vector3(-mu*d*sin(mu*t), mu*d*cos(mu*t), 0.0)
                    a_ref = Vector3(-mu**2*d*cos(mu*t), -mu**2*d*sin(mu*t), 0.0)
                    #roll = 0
                    #pitch = 0
                    #yaw = math.atan2(v_ref[1],v_ref[0])
                    #quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                    #rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                    self.traj_msg.points[i].transforms = [Transform(p_ref, rotation)]
                    self.traj_msg.points[i].velocities = [Twist(v_ref, vel_ang)]
                    self.traj_msg.points[i].accelerations = [Twist(a_ref, acc_ang)]
                    self.traj_msg.points[i].time_from_start = rospy.Time.from_sec(t)
                    
                self.traj_msg.header.stamp = now_obj
                self.pub.publish(self.traj_msg)
                self.t_last_published = t_since_start
            self.rate.sleep() #wait until next time instance
        self.pub.publish(self.cur_ref)
myargs = rospy.myargv(argv=sys.argv)
if len(myargs) != 3:
    print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
    sys.exit()
mav_name = myargs[1]

uav_num = str(myargs[2])

if __name__ == '__main__':
    print('-----------------------')
    print("Launching " + myargs[0] + '...')
    circle_r = 2.0
    altitude = 1.0
    vel_mag = 1.0
    publish_interval = 2.0 #s
    ref_time_step = 0.01 #s
    publisher_obj = Path_Publisher(
        mav_name, 
        uav_num, 
        circle_r, 
        altitude, 
        vel_mag, 
        ref_time_step,
        publish_interval
    )
    try:
        publisher_obj.publish_path()
    except rospy.ROSInterruptException:
        # Read current reference and publish it as new position reference to stop the drone quickly
        print("Error or ending publisher")
