import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt

noDetectionDist = 1
maxDetectionDist = 0.5
detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
usensors = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
v0 = 0.8
t = 0


# --------------------Initial Code To Connect with V-REP--------------------#

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')


for x in range(1, 16 + 1):  # Sensor Handles Access
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles
    usensors[x-1] = sensor_handle


    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)

errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                         vrep.simx_opmode_oneshot_wait)

# ---------------------Main Code for object Avoidance---------------------#


while t < 60:
    for s in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, usensors[s - 1], vrep.simx_opmode_oneshot_wait)  # Measure distance using Ultrasonic Sensor
        dist = np.linalg.norm(detectedPoint)
        if detectionState and (dist < noDetectionDist):
            if dist < maxDetectionDist:
                dist = maxDetectionDist
            detect[s-1] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else:
            detect[s-1] = 0
    vLeft = v0
    vRight = v0
    for s in range(1, 16 + 1):
        vLeft = vLeft + braitenbergL[s-1] * detect[s-1]
        vRight = vRight + braitenbergR[s-1] * detect[s-1]
    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, vLeft, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, vRight, vrep.simx_opmode_streaming)
    t = t + 1