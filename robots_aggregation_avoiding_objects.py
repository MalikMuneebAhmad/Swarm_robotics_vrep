import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import psutil
from robot_epuck import Robot


# ----------------My Functions----------------#

def cluster(robots_pos, robots_det_rob):  # Calculate position and detected robot of a robot
    robots_detrob_dic = dict()
    robots_pos_dic = dict()
    for i, robot_detrob in enumerate(robots_det_rob):
        if robot_detrob:
            robots_detrob_dic[i] = len(robot_detrob)
            robots_pos_dic[i] = robots_pos[i]
    print('robots_position dic', robots_pos_dic)
    print('robots_det_rob dic', robots_pos_dic)
    return robots_pos_dic, robots_detrob_dic


# -------------------------6-6-2021----------------------#
# Robot aggregation with object avoidance working fine with
# e_puck_aggregation_ultrasonic V-rep file


vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

current_time = time.time()
num_robots = 10
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots
v_r = [float(0.0)] * num_robots
rot_t = [float(0.0)] * num_robots
robots_position = list()
robots_det_rob = list()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
while current_time - loop_start_time < 200:
    rob = list(range(num_robots))
    ex2 = time.time()
    for i in range(num_robots):
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()
        # print('Detection------States1', det_state1)
        robots[i].sensor_chemo(0.15, 0.2)
        theta1 = robots[i].object_avoidance(0.2)
        robots[i].chemotaxis_gradient()
        v_l[i], v_r[i], rot_t[i] = robots[i].change_direction()
    ex1 = time.time()
    print('Calculation time is', (ex2-ex1))
    for j in range(num_robots):
        robots[j].movement(v_l[j], v_r[j])
        robots[j].update_position()
        start_time[j] = time.time()
    while rob:
        current_time = time.time()
        for k in rob:
            diff = (current_time - start_time[k])
            # print('difference of time', diff)
            if diff >= rot_t[k]:
                print('For robot {} Target time is {} and Passed time is {}'.format(k, rot_t[k], diff))
                rob.remove(k)
                robots[k].wait(0.0)
for l in range(num_robots): # Evaluate all positions of robots at the end
    print('iteration of l', l)
    robots_position.append([robots[l].robot_position[0], robots[l].robot_position[1]])
    robots_det_rob.append(robots[l].det_rob)
robot_pos_dic, robot_obj_dic = cluster(robots_position, robots_det_rob)  #
#vrep.simxPauseCommunication(clientID, False)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))