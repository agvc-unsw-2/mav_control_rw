#!/usr/bin/env python3

# Read: www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

import time
import os
import vrep
import rospy
from std_msgs.msg import Float64

from subprocess import call

#import vrepCollisionTester

###############################
# Test semantics stuff
###############################

# Call this function to start running your test, with testFnc being the name of the test function.
def runTest(testFnc):
    clientID = connectToServer()
    error = test(clientID, testFnc)
    disconnect(clientID)
    return error

# The function does the necessary error handling for running the test.
# Returns 1 on error, otherwise returns the test function's return value.
def test(clientID):
    if clientID = -1:
        print ('Failed connecting to remote API server')
        return 1
    print ('Connected to remote API server')
    returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    if (returnCode != 0 and returnCode != 1):
        print("An error occured when starting the simulation. End of test.")
        print("Error number: ")
        print(returnCode)
        return 1
    time.sleep(1) # Wait 1 second before starting your test.
    print("Starting simulation")
    # Insert testing code here as python remoteAPI functions.

    retVal = main(clientID)
    
    returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    if (returnCode != 0 and returnCode != 1):
        print("An error occured when stopping the simulation.")
        print("Error number: ")
        print(returnCode)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    print("Test completed.")
    return retVal

def connectToServer():
    portNum = 19997
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',portNum,True,True,5000,5) # Connect to V-REP
    return clientID

def disconnect(clientID):
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

def main(clientID):
    rospy.init_node('VREP synchroniser', anonymous=False)
    time_step_msg = '/simulation/step'
    rospy.Subscriber(time_step_msg, Float64, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming
    retCode, q1p1 = vrep.simxGetCollisionHandle(clientID, "prop1#0", vrep.simx_opmode_blocking)
    retCode, q1p2 = vrep.simxGetCollisionHandle(clientID, "prop2#0", vrep.simx_opmode_blocking)
    retCode, q1p3 = vrep.simxGetCollisionHandle(clientID, "prop3#0", vrep.simx_opmode_blocking)
    retCode, q1p4 = vrep.simxGetCollisionHandle(clientID, "prop4#0", vrep.simx_opmode_blocking)
    
    collisionState = False
    retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p1, vrep.simx_opmode_streaming)
    if retCode==vrep.simx_return_ok:
        collisionState |= collisionStateTemp
    retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p2, vrep.simx_opmode_streaming)
    if retCode==vrep.simx_return_ok:
        collisionState |= collisionStateTemp
    retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p3, vrep.simx_opmode_streaming)
    if retCode==vrep.simx_return_ok:
        collisionState |= collisionStateTemp
    retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p4, vrep.simx_opmode_streaming)
    if retCode==vrep.simx_return_ok:
        collisionState |= collisionStateTemp
    
    retVal = 0
    while vrep.simxGetLastCmdTime(clientID) < 3000:
        collisionState = False
        retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p1, vrep.simx_opmode_buffer)
        if retCode==vrep.simx_return_ok:
            collisionState |= collisionStateTemp
        retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p2, vrep.simx_opmode_buffer)
        if retCode==vrep.simx_return_ok:
            collisionState |= collisionStateTemp
        retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p3, vrep.simx_opmode_buffer)
        if retCode==vrep.simx_return_ok:
            collisionState |= collisionStateTemp
        retCode, collisionStateTemp = vrep.simxReadCollision(clientID, q1p4, vrep.simx_opmode_buffer)
        if retCode==vrep.simx_return_ok:
            collisionState |= collisionStateTemp
        #print (time.time() - startTime)
        print(collisionState)
        if (collisionState):
            retVal = 1
        time.sleep(1)
    return retVal