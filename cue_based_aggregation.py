import sim as vrep  # V-rep library
import sys
import pickle
import time  # used to keep track of time
import psutil
from regulation_mechanism import Modc
from robot_epuck import Robot
import numpy as np  # array library
import matplotlib.pyplot as plt
import math
import random
from utlis import *
import pickle


# ----------------My Functions----------------#
# from robots_aggregation_avoiding_objects import spat_fitness


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
#robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num))
          #for num in range(num_robots)]
with open('robot_file', 'rb') as f:  # to generate your own file  (rb for wrting in bytes)
    robots = pickle.load(f)  # storing variable into file
num_robots = len(robots)
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots
v_r = [float(0.0)] * num_robots
rot_t = [float(0.0)] * num_robots
rob_status = np.zeros([num_robots])
reg_mech = [Modc() for i in range(num_robots)]
robots_position = [list()] * num_robots
robots_det_rob = 0
infla = list()
spat_fitness = list()
danger_st_con = [False] * num_robots
semi_str = [False] * num_robots  # Condition to complete semi-matured cell responce
rob_iter = np.zeros([num_robots], dtype='int')  # iteration of each robot
t = 0  # Current Iteration
grad_ir = [0] * num_robots
pre_ir = [0] * num_robots
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
while current_time - loop_start_time < 900:
    rob = list(range(num_robots))
    for m in range(num_robots):
        print('Robot # ', m)
        rot_theta = 0.0
        sensor_raw1, det_state1 = robots[m].ultrasonic_values()
        # print('Detection- -----States1', det_state1)
        robots[m].sensor_mod_values(0.125, 0.175)
        robots[m].sensor_avoidance_values(0.2, 0.25)
        ir_value, pre_red = robots[m].vision_sensor()
        ir_raw, front_ir_value = robots[m].front_vision_sensor()
        print('front_ir_value', front_ir_value)
        #grad_ir[m] = ir_value - front_ir_value
        grad_ir[m] = ir_value - pre_ir[m]
        pre_ir[m] = ir_value
        print('Vision Sensor Value of Robot {} is {}'.format(m, ir_value))
        if ir_value == 1 and front_ir_value == 0:
            v_l[m], v_r[m], rot_t[m] = robots[m].random_comp_straight(1, 1)
            print("Robot {} will move into black area".format(m))
            continue
        if ir_value == 1 and len(robots[i].det_rob) < 4:  # and front_ir_value == 1:  # To avoid every object and robots outsids black area
            max_grad = 0.0
            all_grad = np.zeros([4])
            obj_theta = robots[m].object_avoidance(robots[m].det_obj_handles, 0.225)
            if not robots[m].obj_avoid:
                levy_theta = robots[m].random_rotation_angle()
                if levy_theta == 0.0:
                    v_l[m], v_r[m], rot_t[m] = robots[m].levy_flight(1.5, 1)
                    print('Robot {} is straight movement in levy search and ir value is {}'.format(m, ir_value))
                    continue
                else:
                    print('Robot {} is rotating in levy search and ir value is {}'.format(m, ir_value))
                    v_l[m], v_r[m], rot_t[m] = robots[m].control_param(levy_theta, 1.5)
                    continue
        else:  # To avoid every object inside black area
            if front_ir_value - ir_value == 1:  # Preventing from going out of target area
                v_l[m], v_r[m], rot_t[m] = robots[m].random_comp_straight(-2, 1.5)
                #print('Back in black area')
                continue
            print('Robot {} is in black area and ir value is {}'.format(m, ir_value))
            obj_theta = robots[m].object_avoidance(robots[m].static_object_handles, 0.2)
            all_grad, max_grad = robots[m].chemotaxis_gradient()  # gradient to move the robots
            rob_status[m] = reg_mech[m].no_neigh_rob(2, 0.08, robots[m].det_rob)  # measure maturity of D c
            if not robots[m].obj_avoid:  # Prefer obstacle avoidance
                if (rob_status[m] == 3 and robots[m].det_rob) or danger_st_con[m]:  # DC is matured so move robot straight or rptate randomly(in next step)
                    if not robots[m].detectionStates[6] and not danger_st_con[m]:  # Move backword
                        v_l[m], v_r[m], rot_t[m] = robots[m].random_straight(-1.5, 1.25)
                        danger_st_con[m] = True
                        # print('Robot {} is moving away because its status is {}'.format(i, rob_status[m]))
                        continue
                    elif not robots[m].detectionStates[2] and not danger_st_con[m]:  # Move forward
                        v_l[m], v_r[m], rot_t[m] = robots[m].random_straight(1.5, 1.25)
                        danger_st_con[m] = True
                        # print('{} Robot {} is moving away because its status is {}'.format(rob_iter[i], i, rob_status[m]))
                        continue
                    else:  # Will cause rotation after moving back
                        rob_iter[m] = 0  # make iterration for a robot zero
                        max_grad = 0.0  # make max gradient zero to avoid specific rotation in next command
                        all_grad = np.zeros([4], dtype='float')
                        danger_st_con[m] = False  # Condition to regulate matured movement
                        robots[m].r_rot = True  # To rotate robot randomly ratber than specifically
                        reg_mech[m].danger_nr = 0.0  # make danger zero
                    # ----Action in case of sem   oi mature dcand not robots[m].det_rob)
                elif rob_status[m] == 2 and robots[m].det_rob and not semi_str[m] and rob_iter[
                    m] >= 3:  # Dc is immatured and chech robot has a neighbor
                    v_l[m], v_r[m], rot_t[m] = robots[m].control_param(
                        (math.pi / 2) * (
                                (-1) ** random.randint(0, 1)))  # to rotate robot perpadicular to neighbor robot
                    semi_str[m] = True  # prepare for next step straight movment
                    print('{} Robot {} semi-mature rotation movement because its status is {}'.format(rob_iter[i], i, rob_status[m]))
                    continue
                elif semi_str[m]:  # straight movement after rotation
                    rob_iter[m] = 0  # make iterration for a robot zero to reduce semi-mature action
                    semi_str[m] = False  # Complete current step
                    if not robots[m].detectionStates[6]:  # Move backword
                        v_l[m], v_r[m], rot_t[m] = robots[m].random_straight(-1, 1)
                        # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[m]))
                        continue
                    elif not robots[m].detectionStates[2]:  # move forward
                        v_l[m], v_r[m], rot_t[m] = robots[m].random_straight(1, 1)
                        # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[m]))
                        continue
                #elif rob_status[m] == 1:
        #print('Current Gradient', robots[m].currrent_gradient)
        rot_theta = robots[m].rotational_angle(all_grad, max_grad, obj_theta)
        v_l[m], v_r[m], rot_t[m] = robots[m].control_param(rot_theta, 1.5)
    for j in range(num_robots):
        robots[j].movement(v_l[j], v_r[j])
        start_time[j] = time.time()
    while rob:
        current_time = time.time()
        for k in rob:
            diff = (current_time - start_time[k])
            # print('difference of time', diff)
            if diff >= rot_t[k]:
                # print('For robot {} Target time is {} and Passed time is {}'.format(k, rot_t[k], diff))
                rob.remove(k)
                robots[k].wait(0.0)
    robots_det_rob = 0
    for l in range(num_robots):  # Evaluate all positions of robots at the end
        robots_det_rob = robots_det_rob + (len(robots[l].det_rob))
    infla.append(1 - robots_det_rob / 45)
    for i in range(num_robots):
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(-1)
    spat_fitness.append(fitness_aggregation(robots_position))
    t = t + 1

vrep.simxPauseCommunication(clientID, False)
#vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
plt.title('Inflamation Plot for 15 Robots in Case of Configuration C ')
plt.xlabel('Iteration #')
plt.ylabel('Inflamation')
plt.plot(list(range(t)), infla)
plt.show()
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))
plt.title('Spacious Fitness for 15 Robots')
plt.xlabel('Iteration #')
plt.ylabel('Spacious Fitness')
plt.plot(list(range(t)), spat_fitness)
plt.show()