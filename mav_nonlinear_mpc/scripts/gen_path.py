import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from tf import transformations
import time
import random
import signal
import sys
import math
import numpy as np

def exit_handler(signal, frame):
    print("exiting...")
    sys.exit(0)


def poseXY(x, y, heading=0):
        point = PoseStamped()
        point.header.frame_id = "map"
        point.header.stamp = rospy.Time.now()

        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0

        orient = transformations.quaternion_from_euler(0.0, 0.0, heading)
        point.pose.orientation.x = orient[0]
        point.pose.orientation.y = orient[1]
        point.pose.orientation.z = orient[2]
        point.pose.orientation.w = orient[3]
        return point


def singlePointPath(x, y, heading):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    point = poseXY(x, y, heading)
    pathData.poses.append(point)
    return pathData


def linePath(length, orientation, size, rotations=0):
    orientation = orientation*180.0 / 3.14159

    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()
    
    start = 0
    stopx = length*math.cos(orientation)
    stopy = length*math.sin(orientation)
    xx = np.linspace(stopx, start, size)
    yy = np.linspace(stopy, start, size)
    headings = np.linspace(0, 2.0*2.14159*rotations, size)
    for (x, y, heading) in zip(xx, yy, headings): 
        point = poseXY(x, y, heading)
        pathData.poses.append(point)
    return pathData


def squarePath(cx, cy, size):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()
    for i in np.arange(0, 2.0*3.14159, 3.14159/2.0):
        x = cx + math.cos(i)*size / 2.0
        y = cy + math.sin(i)*size / 2.0
        point = poseXY(x, y)
        pathData.poses.append(point)

    pathData.poses.append(pathData.poses[0])
    return pathData


def circlePath(radius, size, revolutions=0):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    circleTheta = np.linspace(0, 2*3.14159, size)
    xx = radius*np.cos(circleTheta)
    yy = radius*np.sin(circleTheta)
    headingTheta = np.linspace(0, revolutions*2.0*3.14159, size)
    headingTheta = np.fmod(headingTheta, 2.0*3.14159)

    for (x, y, heading) in zip(xx, yy, headingTheta):
        point = poseXY(x, y, heading)
        pathData.poses.append(point)
    return pathData



def randPath(limits, numPts):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    for i in range(0, numPts):
        x = random.uniform(-limits, limits)
        y = random.uniform(-limits, limits)
        point = poseXY(x, y)

        pathData.poses.append(point)

    pathData.poses.append(pathData.poses[0])
    return pathData


def stretchySine(length, amplitude, startFreq, stopFreq, numPts):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    for i in np.linspace(1.0, 0.0, numPts):
        freq = startFreq + i*(stopFreq - startFreq)
        x = i*length
        y = amplitude * np.sin(2*3.14159*freq*i)
        point = poseXY(x, y)

        pathData.poses.append(point)
    return pathData


def starPath(inner, outer, numPts):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    for n, theta in enumerate(np.linspace(0, 2*3.14159, 2*numPts)):
        if n % 2 == 0:
            rad = inner
        else:
            rad = outer

        x = rad*math.cos(theta)
        y = rad*math.sin(theta)
        point = poseXY(x, y)

        pathData.poses.append(point)
    return pathData


def spiralPath(radius, turns, numPts):
    pathData = Path()
    pathData.header.frame_id = "map"
    pathData.header.stamp = rospy.Time.now()

    theta = np.linspace(0, 2.0*2.14159*turns, numPts)
    radii = np.linspace(0, radius, numPts)
    xx = radii*np.cos(theta)
    yy = radii*np.sin(theta)

    for (x, y) in zip(xx, yy):
        point = poseXY(x, y)
        pathData.poses.append(point)
    return pathData


def main():
    signal.signal(signal.SIGINT, exit_handler)

    rospy.init_node('path_publisher', anonymous=True)
    pathPublisher = rospy.Publisher("/navigation/path",Path,queue_size=100)
    print("opened node, publishing paths...")

    while True:
        pathData = linePath(100.0, 45.0, 50.0, 200)
        #pathData = squarePath(0.0, 0.0, 10.0)
        #pathData = circlePath(5.0, 100.0, -4.0)
        #pathData = randPath(4.0, 5)
        #pathData = starPath(1.0, 3.0, 8)
        #pathData = spiralPath(5.0, 2.0, 150)
        #pathData = stretchySine(10.0, 2.0, 0.5, 2.0, 30)
        #pathData = singlePointPath(10000, 10000, 0)
        pathPublisher.publish(pathData)
        time.sleep(1)


if __name__ == "__main__":
    main()
