import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
from robot_epuck import Robot


vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

num_robots = 2
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
while current_time - loop_start_time < 20:
    for i in range(num_robots):
        robots[i].wait(0.0)
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()
        print('Robot {} Sensor raw values {}'.format(i, robots[i].sensor_raw_values))
        print('Robot {} Sensor Detection state {}'.format(i, robots[i].detectionStates))