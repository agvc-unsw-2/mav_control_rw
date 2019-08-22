#!/usr/bin/env python

import hlc as hlc_lib
import math

import time
import os
import rospy
import math
import copy
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from std_msgs.msg import Bool
from math import cos
from math import sin
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Transform
#from trajectory_msgs.msg import MultiDOFJointTrajectory

#moduleName = "hlc"
#importlib.import_module(moduleName)

def step_response(hlc, start, end, period, cycles, longways_direction):
    print("X step response")
    hlc.takeoff()
    position = Point(0, 0, hlc.hover_height)
    hlc.publishPositionForTime(position, period)
    for i in range(cycles):
        if (longways_direction == "x"):
            position = Point(start, 0, hlc.hover_height)
            hlc.publishPositionForTime(position, period)
            position = Point(end, 0, hlc.hover_height)
            hlc.publishPositionForTime(position, period)
        elif (longways_direction == "x"):
            position = Point(0, start, hlc.hover_height)
            hlc.publishPositionForTime(position, period)
            position = Point(0, end, hlc.hover_height)
            hlc.publishPositionForTime(position, period)

# TODO Fix so that step response duration is fixed
# read time before sending setpoint and 

def move_in_square(hlc):
    print("Moving in square")
    hlc.takeoff()
    time_before_start = 2.0
    position = Point(-1, -1, hlc.hover_height)
    hlc.goToPosition(position)
    time.sleep(time_before_start - hlc.wait_time)
    position = Point(1, -1, hlc.hover_height)
    hlc.goToPosition(position)
    position = Point(1, 1, hlc.hover_height)
    hlc.goToPosition(position)
    position = Point(-1, 1, hlc.hover_height)
    hlc.goToPosition(position)
    position = Point(-1, -1, hlc.hover_height)
    hlc.goToPosition(position)
    hlc.land()

def generate_circle_path(r, vel_mag, altitude, cycles):
    ref_time_step = 0.01
    mu = vel_mag/r # t scale factor to scale velocity
    zero_vec = Vector3(0.0, 0.0, 0.0) # For angular velocity and acceleration
    rotation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
    now_secs = rospy.get_time()
    now_obj = rospy.get_rostime()
    # speed = dist/time -> t = dist / speed
    traj_time = (2.0 * math.pi * r) / vel_mag
    num_points = int((float(traj_time) / float(ref_time_step))) + 1 # + 1 just in case
    num_points = num_points * cycles # Repeat for some number of cycles
    traj_msg = MultiDOFJointTrajectory()
    traj_msg.points = [None]*num_points
    for i in range(num_points):
        traj_msg.points[i] = MultiDOFJointTrajectoryPoint()
    #traj_msg.points = 
    if now_secs <= 0: # simulation hasn't started
        # Hover in space
        traj_msg.header.stamp = rospy.Time.from_sec(0.0)
    else:
        for i in range(num_points):
            t = now_secs + (i * ref_time_step)
            p_ref = Vector3(r*cos(mu*t), r*sin(mu*t), altitude)
            v_ref = Vector3(-mu*r*sin(mu*t), mu*r*cos(mu*t), 0.0)
            a_ref = Vector3(-(mu*mu)*r*cos(mu*t), -(mu*mu)*r*sin(mu*t), 0.0)
            traj_msg.points[i].transforms = [Transform(p_ref, rotation)]
            traj_msg.points[i].velocities = [Twist(v_ref, zero_vec)]
            traj_msg.points[i].accelerations = [Twist(a_ref, zero_vec)]
            traj_msg.points[i].time_from_start = rospy.Time.from_sec(t)
        traj_msg.header.stamp = now_obj
    return traj_msg

def generate_lemniscate_traj(max_dist_from_origin, vel_max, altitude, longways_direction):
    sqrt_2 = math.sqrt(2)
    a = max_dist_from_origin / sqrt_2
    ref_time_step = 0.01
    zero_vec = Vector3(0.0, 0.0, 0.0) # For angular velocity and acceleration
    rotation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
    #To have vel_max = 1, we have mu = 1/(a * sqrt_2)
    #To have vel_max = 2, we have mu = 2 / (a * sqrt_2)
    mu = vel_max / (a * sqrt_2) # velocity is 1 for (a * sqrt_2 == 1)
    # takes 2pi seconds for one period of loop. (For max velocity of a * sqrt_2)
    # For a normalised velocity of 1, vel scaler is division by (a * sqrt_2)
    # Therefore it would take (a * sqrt_2) times longer than 2 * pi for a speed of 1
    traj_time = (2.0 * math.pi) * (a * sqrt_2) / vel_max # t = dist / speed
    num_points = int((float(traj_time) / float(ref_time_step))) + 1 # + 1 just in case
    num_points = num_points * cycles # Repeat for some number of cycles
    traj_msg = MultiDOFJointTrajectory()
    traj_msg.points = [None]*num_points
    for i in range(num_points):
        traj_msg.points[i] = MultiDOFJointTrajectoryPoint()
    now_secs = rospy.get_time()
    now_obj = rospy.get_rostime()
    if now_secs <= 0: # simulation hasn't started
        # Hover in space
        traj_msg.header.stamp = rospy.Time.from_sec(0.0)
    else:
        for i in range(num_points):
            t = now_secs + (i * ref_time_step)
            t0 = (i * ref_time_step) + ((math.pi / 2) / mu)
            mu_t = mu * t0
            cos_t = cos(mu * t0)
            sin_t = sin(mu * t0)
            sin_2_t = sin_t * sin_t
            sin_4_t = sin_2_t * sin_2_t
            cos_2_t = cos_t * cos_t
            x_ref = (a * sqrt_2 * cos_t)/(sin_2_t + 1)
            y_ref = (a * sqrt_2 * cos_t * sin_t)/(sin_2_t + 1)
            dx_ref = - mu * a * sqrt_2 * sin_t*(1 + 2*cos_t*cos_t + sin_2_t) / ((1 + sin_2_t)*(1 + sin_2_t))
            dy_ref = - mu * (a * sqrt_2 * sin_4_t + sin_2_t + (sin_2_t - 1) * cos_2_t) / ((1 + sin_2_t)*(1 + sin_2_t))

            if (longways_direction == "y"):
                tmp_ref = x_ref
                x_ref = y_ref
                y_ref = tmp_ref
                tmp_ref = dx_ref
                dx_ref = dy_ref
                dy_ref = tmp_ref
            
            p_ref = Vector3(x_ref, y_ref, altitude)
            v_ref = Vector3(dx_ref, dy_ref, 0.0)
            a_ref = Vector3(0, 0, 0)
            traj_msg.points[i].transforms = [Transform(p_ref, rotation)]
            traj_msg.points[i].velocities = [Twist(v_ref, zero_vec)]
            traj_msg.points[i].accelerations = [Twist(a_ref, zero_vec)]
            traj_msg.points[i].time_from_start = rospy.Time.from_sec(t)

        traj_msg.header.stamp = now_obj
    return traj_msg

def circle_path(hlc, r, vel_mag, altitude, cycles):
    print("Generating circle path")
    now_secs = rospy.get_time()
    traj_msg = generate_circle_path(r, vel_mag, altitude, cycles)
    if(traj_msg.header.stamp <= rospy.Time(0.0)):
        hlc.takeoff()
        return
    traj_end_time = traj_msg.points[-1].time_from_start.to_sec()
    traj_start_time = traj_msg.points[0].time_from_start.to_sec()
    traj_time = (traj_end_time - traj_start_time)
    print("Following circle path for " + str(traj_time) + "secs")
    #print(traj_msg)
    hlc.followTraj(traj_msg, traj_time)

def lemniscate_path(hlc, max_dist_from_origin, vel_max, altitude, cycles, longways_direction):
    now_secs = rospy.get_time()
    traj_msg = generate_lemniscate_traj(max_dist_from_origin, vel_max, altitude, cycles, longways_direction)
    if(traj_msg.header.stamp <= rospy.Time(0.0)):
        hlc.takeoff()
        return
    traj_end_time = traj_msg.points[-1].time_from_start.to_sec()
    traj_start_time = traj_msg.points[0].time_from_start.to_sec()
    traj_time = (traj_end_time - traj_start_time)
    print("Following lemniscate " + longways_direction + " path for " + str(traj_time) + "secs")
    hlc.followTraj(traj_msg, traj_time)

def main(mav_name, uav_num):
    print("Initialising tester")
    rospy.init_node('python_hlc', anonymous=False)
    hlc = hlc_lib.High_Level_Controller(mav_name, uav_num)
    start = 0.0
    end = 10.0
    period = 10.0
    cycles = 4
    while(rospy.get_time() < 1.0):
        time.sleep(0.1)
    while(rospy.get_time() < 1.0):
        time.sleep(0.1)
    print(rospy.get_time())
    hlc.takeoff()
    step_response(hlc, start, end, period, cycles, "x")
    #step_response(hlc, start, end, period, cycles, "y")

    #####################
    # Circle path
    #####################
    r = 1
    vel_mag = 1
    altitude = 1
    #circle_path(hlc, r, vel_mag, altitude, cycles)
    #####################
    # Lemniscate path
    #####################
    x_max = 1.5
    vel_max = 1
    altitude = 1
    #lemniscate_path(hlc, x_max, vel_max, altitude, cycles, "x")
    hlc.land()

if __name__ == "__main__":
    myargs = rospy.myargv(argv=sys.argv)
    print("ROS time:")
    print(myargs)
    if(len(myargs) == 1):
        mav_name = "vrep_quad"
        uav_num = "1"
    elif (len(myargs) == 3):
        mav_name = str(myargs[1])
        uav_num = str(myargs[2])
    else:
        print("USAGE: ./" + str(sys.argv[0]) + " <mav_name> <uav_num>")
        sys.exit()
    main(mav_name, uav_num)
