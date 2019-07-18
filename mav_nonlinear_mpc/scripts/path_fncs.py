#!/usr/bin/env python

import importlib
import sys
import rospy

from hlc import *

#moduleName = "hlc"
#importlib.import_module(moduleName)

def step_response_x(hlc, start, end, period, cycles):
    print("X step response")
    hlc.hover()
    for i in range(cycles):
        position = Point(start, 0, hlc.hover_height)
        hlc.publishPositionForTime(position, period)
        position = Point(end, 0, hlc.hover_height)
        hlc.publishPositionForTime(position, period)

def step_response_y(hlc, start, end, period, cycles):
    print("Y step response")
    hlc.hover()
    for i in range(cycles):
        position = Point(0, start, hlc.hover_height)
        hlc.publishPositionForTime(position, period)
        position = Point(0, end, hlc.hover_height)
        hlc.publishPositionForTime(position, period)

# TODO Fix so that step response duration is fixed
# read time before sending setpoint and 

def move_in_square(hlc):
    print("Moving in square")
    hlc.hover()
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

def generate_circle_path(r, vel_mag, altitude):
    ref_time_step = 0.01
    mu = vel_mag/r # velocity scale factor
    zero_vec = Vector3(0.0, 0.0, 0.0) # For angular velocity and acceleration
    rotation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
    now_secs = rospy.get_time()
    now_obj = rospy.get_rostime()
    traj_time = vel_mag * (2.0 * math.pi * r) # t = speed * distance
    num_points = int((float(traj_time) / float(ref_time_step))) + 1 # + 1 just in case
    
    traj_msg = MultiDOFJointTrajectory()
    if now_secs <= 0: # simulation hasn't started
        # Hover in space
        traj_msg.header.stamp = rospy.Time.from_sec(-1.0)
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

def circle_path(hlc, r, vel_mag, altitude):
    traj_msg = generate_circle_path(r, vel_mag, altitude)
    if(traj_msg.header.stamp == rospy.Time.from_sec(-1.0)):
        hlc.hover
        return
    traj_time = (traj_msg.points[-1].time_from_start - traj_msg.points[0].time_before_start)
    hlc.followTraj(traj_msg, traj_time)

def main(mav_name, uav_num):
    print("Initialising tester")
    rospy.init_node('python_hlc', anonymous=False)
    hlc = High_Level_Controller(mav_name, uav_num)
    start = 0
    end = 2
    period = 2.0
    cycles = 3
    step_response_y(hlc, start, end, period, cycles)
    hlc.land()

if __name__ == "__main__":
    if(len(sys.argv) == 1):
        mav_name = "vrep_quad"
        uav_num = "1"
    elif (len(sys.argv) == 3):
        mav_name = str(sys.argv[1])
        uav_num = str(sys.argv[2])
    else:
        print("USAGE: ./" + str(sys.argv[0]) + " <mav_name> <uav_num>")
        sys.exit()
    main(mav_name, uav_num)
