
import time
import os
import vrep

from vrepTestLib import *

class tester(object):
    __initialized = False
    def __init__(self):
        if tester.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        tester.__initialized = True
    def run(self):
        self.retVal = runTest(self.test_function)
    def test_function(self, clientID):
        ##############################
        test_run_time_ms = 5
        ##############################
        vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming) # Initialize streaming
        retCode, q1p1 = vrep.simxGetCollisionHandle(clientID, "prop1#0", vrep.simx_opmode_blocking)
        retCode, q1p2 = vrep.simxGetCollisionHandle(clientID, "prop2#0", vrep.simx_opmode_blocking)
        retCode, q1p3 = vrep.simxGetCollisionHandle(clientID, "prop3#0", vrep.simx_opmode_blocking)
        retCode, q1p4 = vrep.simxGetCollisionHandle(clientID, "prop4#0", vrep.simx_opmode_blocking)
        
        # Initialise reading
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
        
        # Continue reading
        if collisionState:
            retVal = 1
        else:
            retVal = 0
        while (vrep.simxGetLastCmdTime(clientID) < test_run_time_ms):
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
            if (collisionState):
                print("COLLISION DETECTED!!! I'm gonna go fail this test now")
                retVal = 1
                break
            else:
                print("No collision. I'm surprised.")
            time.sleep(1)
        return retVal

"""
def import_vrep():
    print("==================================")
    print("Importing vrep to vrepCollisionTest")
    print("==================================")
    try:
        import vrep
    except:
        print ('--------------------------------------------------------------')
        print ('"vrep.py" could not be imported. This means very probably that')
        print ('either "vrep.py" or the remoteApi library could not be found.')
        print ('Make sure both are in the same folder as this file,')
        print ('or appropriately adjust the file "vrep.py"')
        print ('--------------------------------------------------------------')
        print ('')
"""
