import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
from robot_epuck import Robot
from utlis import *


vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit()


num_robots = 1
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
errorCode, point_cloud = vrep.simxGetObjectHandle(clientID, 'Point_cloud',vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles
returnCode,linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(clientID,robots[0].robot_handle,vrep.simx_opmode_streaming )
#robots = Robot(clientID, 'ePuck#' + str(0), 'ePuck_proxSensor#' + str(0))
print('point_cloud', point_cloud)
print('returnCode', returnCode)
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()

#while current_time - loop_start_time < 200:
v_l, v_r, disp_t = robots[0].displacement_robot(0.5)
#previousTime = vrep.simxGetLastCmdTime()
print('v_l', v_l)
print('v_r', v_r)
x1, disp, dir = robots[0].get_position(-1)
print('position of robot is', x1)
print('displacement of robot is', disp)
print('direction of robot is', dir)
robots[0].movement(v_l, v_r)
time.sleep(disp_t/2)
returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(clientID, robots[0].robot_handle,
                                                                             vrep.simx_opmode_streaming)
v_res = math.sqrt(linearVelocity[0] **2 + linearVelocity[1] **2)
print('Control parameters are', 2 * v_l)
print('Vx of robot is ', linearVelocity[0])
print('Vy of robot is ', linearVelocity[1])
print('Velocity of robot is ', v_res)
print('Total time duration', disp_t)
time.sleep(disp_t/2)
robots[0].wait(0.0)
x2, disp, dir  = robots[0].get_position(-1)
type(x2)
#delta_x = np.array(x2) - np.array(x1)
#dist = math.sqrt(np.sum(np.square(delta_x)))
#print('Distance in m', dist)