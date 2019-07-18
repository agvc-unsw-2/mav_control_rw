#!/usr/bin/env python

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
from std_msgs.msg import Bool

####################
# Basic landing, object pick up, and flying test
####################

# Does not check for collisions while testing

class High_Level_Controller():
    __initialized = False
    def __init__(self, mav_name, uav_num):
        if self.__initialized:
            raise Exception("High_Level_Controller already initialised")
        self.__initialized = True
        self.retVal = 0

        self.search_height = 1.0
        self.hover_height = 1.0
        self.land_height = 0.05
        self.suction_height = 0.0 #offset for suction cup
        self.wait_time = 1.0

        self.odom_subber = odom_subscriber(mav_name, uav_num)
        self.pose_pubber = pose_publisher(mav_name, uav_num)
        self.traj_pubber = traj_publisher(mav_name, uav_num)
        self.gripper = gripper_controller(mav_name, uav_num)

    def goToWaypoint(self, pose_cmd):
        self.pose_pubber.publish(pose_cmd)
        while(not at_waypoint(pose_cmd, self.odom_subber.msg)):
            #print("At: ")
            #print(self.odom_subber.msg)
            position = copy.deepcopy(self.odom_subber.msg.pose.pose.position)
            print("At [" + str(position.x) + ", " + str(position.y) + ", " + str(position.z)) + ']'
            self.pose_pubber.publish(pose_cmd)
            time.sleep(0.5)
        time.sleep(self.wait_time) # Wait
    
    # Same as goToPose but position Point msg instead of Pose cmd
    def goToPosition(self, position):
        print("Moving to [" + str(position.x) + ", " + str(position.y) + ", " + str(position.z)) + ']'
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
        pose_cmd = Pose(position, orientation)
        self.goToWaypoint(pose_cmd)

    def publishPositionForTime(self, position, pos_hold_time):
        now_secs = rospy.get_time()
        end_time = now_secs + pos_hold_time
        print("Moving to [" + str(position.x) + ", " + str(position.y) + ", " + str(position.z)) + ']'
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
        pose_cmd = Pose(position, orientation)
        self.pose_pubber.publish(pose_cmd)
        print(pose_cmd)
        while(rospy.get_time() < end_time):
            time.sleep(0.1)

    def followTraj(self, traj_msg, traj_time):
        # PUBLISH_TRAJ_CMD
        self.traj_pubber.publish(traj_msg)
        final_point = traj_msg.points[-1].transforms[0].translation
        now_secs = rospy.get_time()
        now_obj = rospy.get_rostime()
        # Wait required time
        while(rospy.get_time() < now_secs + traj_time):
            time.sleep(0.1)
        # Check at final point
        while(not at_waypoint(final_point, self.odom_subber.msg)):
            time.sleep(0.1)
        # Sleep
        time.sleep(self.wait_time) # Wait
 

    def hover(self):
        position = copy.deepcopy(self.odom_subber.msg.pose.pose.position)
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
        pose_cmd = Pose(position, orientation)
        print("Going to hover waypoint:")
        self.goToWaypoint(pose_cmd)
        #position = Point(0, 0, self.hover_height)
        #print("Moving to [" + str(position.x) + ", " + str(position.y) + ", " + str(position.z)) + ']'
        #self.publishPositionForTime(position, self.wait_time)
        print("Hover finished")

    def land(self):
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # For rotation
        self.hover()
        position = Point(0, 0, self.hover_height)
        self.goToPosition(position)
        position = Point(0, 0, self.land_height)
        self.goToPosition(position)

# Check if value in range
def inRange(setpoint, value, threshold):
    if (abs(value - setpoint) < threshold):
        return True
    else:
        return False

def at_vector3(vec1, vec2, threshold):
    is_at_vector3 = False
    if inRange(vec1.x, vec2.x, threshold):
        if inRange(vec1.y, vec2.y, threshold):
            if inRange(vec1.z, vec2.z, threshold):
                is_at_vector3 = True
    return is_at_vector3

def at_quat(q1, q2, threshold):
    is_at_quat = False
    if inRange(q1.x, q2.x, threshold):
        if inRange(q1.y, q2.y, threshold):
            if inRange(q1.z, q2.z, threshold):
                if inRange(q1.w, q2.w, threshold):
                    is_at_quat = True
    return is_at_quat

def at_waypoint(pose_cmd, odom):
    is_at_waypoint = True
    pose = odom.pose.pose
    twist = odom.twist.twist
    pos_cmd = pose_cmd.position
    pos = pose.position
    quat_cmd = pose_cmd.orientation
    quat = pose.orientation

    # Check position
    if not at_vector3(pos_cmd, pos, 0.3):
        is_at_waypoint = False
    # Check orientation
    if not at_quat(quat_cmd, quat, 0.5):
        is_at_waypoint = False
    # Check linear velocity
    if not at_vector3(twist.linear, Vector3(), 0.2):
        is_at_waypoint = False
    # Check angular velocity
    if not at_vector3(twist.angular, Vector3(), math.radians(5.0)):
        is_at_waypoint = False

    return is_at_waypoint

class odom_subscriber:
    def __init__(self, mav_name, uav_num):
        # Initialise subscriber
        sub_topic =  '/' + mav_name + str(uav_num) + '/ground_truth/mavros/local_position/odom'
        self.sub = rospy.Subscriber(
            sub_topic,
            Odometry,
            self.odom_callback,
            queue_size=1
        )
        self.msg = Odometry()

    def odom_callback(self, msg):
        # Callback function for subscribed topic
        self.msg = msg
        #print(self.odom)


class pose_publisher:
    def __init__(self, mav_name, uav_num):
        # Initialise ros publisher
        pub_topic =  '/' + mav_name + str(uav_num) + '/command/pose'
        self.pub = rospy.Publisher(
            pub_topic,
            PoseStamped, 
            queue_size=1
        )
        self.setpoint = PoseStamped()
    def publish(self, pose_cmd):
        self.setpoint.header.stamp = rospy.get_rostime()
        self.setpoint.header.seq += 1
        self.setpoint.pose = pose_cmd
        self.pub.publish(self.setpoint)

class traj_publisher:
    def __init__(self, mav_name, uav_num):
        # Initialise ros publisher
        pub_topic =  '/' + mav_name + str(uav_num) + '/command/trajectory'
        self.pub = rospy.Publisher(
            pub_topic,
            MultiDOFJointTrajectory, 
            queue_size=1
        )
        self.setpoint = MultiDOFJointTrajectory()
        self.t0 = -1.0

    def publish(self, traj_points):
        self.setpoint.header.stamp = rospy.get_rostime()
        self.setpoint.header.seq += 1
        self.setpoint.points = traj_points
        self.pub.publish(self.setpoint)

class gripper_controller:
    def __init__(self, mav_name, uav_num):
        # Initialise ros publisher
        self.pub = rospy.Publisher(
            '/' + mav_name + str(uav_num) + '/suction_pad_state', Bool, queue_size = 1
        )
        self.status = Bool()

    def update(self, msgData):
        self.status.data = msgData
        self.pub.publish(self.status)

