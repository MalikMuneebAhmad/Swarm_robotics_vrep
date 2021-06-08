# Import Libraries:
import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib as mpl  # used for image plotting

# Pre-Allocation

PI = math.pi  # pi=3.14..., constant

vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

k = [' ', '0', '1', '2', '3', '4']
static_object_handles = list()

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')

else:
    print('Connection not successful')
    sys.exit('Could not connect')

# retrieve motor  handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                         vrep.simx_opmode_oneshot_wait)

# retrieve motor  handles
errorCode, left_motor_handle1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                         vrep.simx_opmode_oneshot_wait)
sensor_h = []  # empty list for handles
sensor_val = np.array([])  # empty array for sensor distance measurements
sensor_map = [0] * 8  # empty list for sensor mapping environment
for x in range(1, 8 + 1):
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait)
    sensor_h.append(sensor_handle)  # keep list of handles
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)
    sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))
t = time.time()

for i in k:
    errorCode, object_handle = vrep.simxGetObjectHandle(clientID, 'Cuboid' + i, vrep.simx_opmode_blocking)
    static_object_handles.append(object_handle)
print(static_object_handles)

while (time.time() - t) < 60:
    # Loop Execution
    sensor_val = np.array([])
    for x in range(1, 8 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_h[x - 1], vrep. simx_opmode_oneshot_wait)
        print('Normal Vector', detectedSurfaceNormalVector)
        sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))  # get list of values
        #errorCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_visible0', vrep.simx_opmode_blocking)
        #print('Detected Sensor', robot_handle)
        if detectedObjectHandle in static_object_handles:
            print('It is an obstacle at distance', np.linalg.norm(detectedPoint))
            sensor_map[x-1] = -1
        elif detectedObjectHandle == 1 or detectedObjectHandle > 1000:
            print('Nothing is in front of sensor')
            sensor_map[x - 1] = 0
        else:
            print('it is a robot at distance', np.linalg.norm(detectedPoint))
            sensor_map[x - 1] = 1
        print('detected Object {0} by sensor number {1}'.format(detectedObjectHandle, x))
    print('Sensor Val', sensor_val)
    print('Sensor map', sensor_map)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.0, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.0, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle1, 0.0, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle1, 0.0, vrep.simx_opmode_streaming)