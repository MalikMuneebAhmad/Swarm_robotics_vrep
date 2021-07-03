import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
from robot_epuck import Robot
from utlis import *


vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')


num_robots = 1
#robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
#errorCode, Vision_sensor = vrep.simxGetObjectHandle(clientID, 'Vision_sensor#3',
                            #vrep.simx_opmode_blocking)  # Retrieving Sensor handles
errorCode, point_cloud = vrep.simxGetObjectHandle(clientID, 'Point_cloud',
                            vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles
#returnCode, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, robots[3].Vision_sensor, vrep.simx_opmode_streaming)
robots = Robot(clientID, 'ePuck#' + str(0), 'ePuck_proxSensor#' + str(0))
print('returnCode', returnCode)
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
returnCode, target_pos = vrep.simxGetObjectPosition(clientID, point_cloud, -1,
                                                                  vrep.simx_opmode_oneshot_wait)
returnCode, front_sensor_pos = vrep.simxGetObjectPosition(clientID, robots.sensor_handles[2], -1,
                                                                  vrep.simx_opmode_oneshot_wait)
robots.get_position()
print('robot position', robots.robot_position)
print('front_sensor_pos', front_sensor_pos)
print('target_point', target_pos)
dir_rob_wax = angles_calcu(front_sensor_pos[0], front_sensor_pos[1], robots.robot_position[0], robots.robot_position[1])
target_angle = np.arctan2(target_pos[1], target_pos[0])
print('robot direction', dir_rob_wax * 180/ math.pi)
print('target_angle', target_angle * 180/ math.pi)
rot_angle = target_angle - dir_rob_wax
print('rotational angle', rot_angle * 180/ math.pi)
'''while current_time - loop_start_time < 20:
    robots.get_position()
    print('robot position', robots.robot_position)
    robots.wait(0.2)
    #returnCode, detectionState, auxPackets = vrep.simxReadVisionSensor(clientID, robots.Vision_sensor, vrep.simx_opmode_streaming)
    #ir = robots.vision_sensor()
    #sensor_raw1, det_state1 = robots.ultrasonic_values()
    if robots[i].detectionStates[2]:  # MOve backword
        v_l, v_r, rot_t = robots[i].random_straight(-1)
    elif robots[i].detectionStates[6]:  # move forward
        v_l, v_r, rot_t = robots[i].random_straight(1)
    else:
        v_l = 2.5
        v_r = 2.5
        rot_t = 1
        #robots[i].wait(0.0)
        robots[i].movement(v_l, v_r, rot_t)
        #print('returnCode in loop', returnCode)
        #print('Robot {} Sensor raw values {}'.format(i, robots[i].sensor_raw_values))
        #print('Robot {} Sensor Detection state {}'.format(i, robots[i].detectionStates))
        #print('Vision Sensor AuxPackets', auxPackets)
    current_time = time.time()
        #print('Vision Sensor min intensity', auxPackets[0][0])
        #print('Vision Sensor max intensity', auxPackets[0][5])
    print('target_point', target_point)'''