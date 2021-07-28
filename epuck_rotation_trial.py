import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
import numpy as np  # array library
import math
from robot_epuck import Robot
from utlis import *

#----------------6-7-2021--------------#
# Rotational angle to move towards target position is Calculated
# Vrep recommended file is "robot_approach_target"

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    vrep.simxSynchronous(clientID, True)
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit()

num_robots = 1
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
#robots = Robot(clientID, 'ePuck#' + str(0), 'ePuck_proxSensor#' + str(0))
print('returnCode', returnCode)
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
t = 0
str_mov = [True] * num_robots
while current_time - loop_start_time < 200:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    for i in range(num_robots):  # measure control parameter for each robot
        print('Robot number = ', i)
        rot_theta = math.pi
        if str_mov[i]:
            print('Robot is rotating')
            v_l[i], v_r[i], rot_t[i] = robots[i].rotation_robot(rot_theta)
            str_mov[i] = False
        else:
            print('Robot is moving straight')
            v_l[i], v_r[i], rot_t[i] = robots[i].random_comp_straight(1, 1)
            str_mov[i] = True
    time.sleep(2)
    for j in range(num_robots):  # Starting movement of all robots Loop
        robots[j].movement(v_l[j], v_r[j])
        start_time[j] = time.time()  # Calculation of start time
    while rob:  # Loop will end when 'rob' list will be empty
        current_time = time.time()
        for k in rob:
            diff = (current_time - start_time[k])
            # print('difference of time', diff)
            if diff >= rot_t[k]:
                # print('For robot {} Target time is {} and Passed time is {}'.format(k, rot_t[k], diff))
                rob.remove(k)
                robots[k].wait(0.0)  # Stop of robot
    #for i in range(num_robots):
        #robots_position[i] = robots[i].get_position()
    t += 1
vrep.simxPauseCommunication(clientID, False)