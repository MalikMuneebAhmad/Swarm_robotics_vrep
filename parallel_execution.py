from robot_epuck import Robot
import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import random
import psutil

#----------------My Functions----------------#


def cluster(robots_position, robots_det_rob):
    for i, robot_det_rob in enumerate(robots_det_rob):
        if not robot_det_rob:
            robots_det_rob.remove(robot_det_rob)
            robots_position.remove(robots_position[i])
        print('robots_position',robots_position)
        print('robots_det_rob', robots_det_rob)

#-------------------------6-6-2021----------------------#
# Robot aggregation with out object avoidance working fine with
# e_puck_aggregation_ultrasonic V-rep file

vrep.simxFinish(-1)  # just in case, close all opened connections
ID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
type(ID)
if ID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(ID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

ID = 0
t = 0  # Start of Iterations
num_robots = 10  # No of Robots
rob = list(range(10))
robots = [Robot(ID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]  # Instances of each robot
start_time = [float(0.0)] * num_robots  #
v_l1 = [float(0.0)] * num_robots
v_r1 = [float(0.0)] * num_robots
rot_t1 = [float(0.0)] * num_robots
robots_position = list()
robots_det_rob = list()
current_time = time.time()
vrep.simxStartSimulation(ID, vrep.simx_opmode_blocking)
while t < 60:
    rob = list(range(10))
    for i in range(num_robots):  # Loop to find out control parameters
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()
        robots[i].sensor_chemo(0.15, 0.20)
        theta1, det1 = robots[i].chemotaxis_gradient()
        v_l1[i], v_r1[i], rot_t1[i] = robots[i].change_direction()
    f_st = time.time()
    for j in range(num_robots):  # Loop to start moving
        robots[j].movement(v_l1[j], v_r1[j])
        start_time[j] = time.time()
    f_ft = time.time()
    it_diff = (f_ft-f_st)
    #print('Time consumed by Joint loop', it_diff)
    #for k in range(num_robots):
    while rob != []:
        current_time = time.time()
        for k in rob:
            diff = (current_time - start_time[k])
            #print('difference of time', diff)
            if diff >= rot_t1[k]:
                print('For robot {} Target time is {} and Passed time is {}'.format(k, rot_t1[k], diff))
                rob.remove(k)
                robots[k].wait(0.0)
    vrep.simxPauseCommunication(ID, False)
    if t == 50:
        for l in range(num_robots):
            robots_position.append([robots[l].robot_position[0], robots[l].robot_position[1]])
            robots_det_rob.append(robots[l].det_rob)
            #robots_sensors_state.append(robots[l].detectionStates)
    t += 1
cluster(robots_position, robots_det_rob)
vrep.simxStopSimulation(ID, vrep.simx_opmode_blocking)
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))