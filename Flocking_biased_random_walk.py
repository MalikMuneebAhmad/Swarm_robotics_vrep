import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
from numpy import interp
import psutil  # to find computation of CPU
import numpy as np  # array library
import matplotlib.pyplot as plt  # Plotting library
import math  # math library
import random  # random number generation
from regulation_mechanism import Modc  # Regulation mechanism base on DC maturity
from robot_epuck import Robot  # Robot library
from utlis import *

#-------------------------9-6-2021----------------------#
# Robot flocking (NOT TESTED YET) with Flocking_towards_target
# All robots will move towards target keeping safe distance with neighboring robots
# File include feature of robot moving towards gradient, avoiding obstacle

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    #vrep.simxSynchronous(clientID, True)
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit()

current_time = time.time()
#num_robots = 1
#robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
with open('robot_file', 'rb') as f:  # to generate your own file  (rb for wrting in bytes)
    robots = pickle.load(f)  # storing variable into file
num_robots = len(robots)
errorCode, point_cloud = vrep.simxGetObjectHandle(clientID, 'Point_cloud',vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles
returnCode, target_pos = vrep.simxGetObjectPosition(clientID, point_cloud, -1,
                                                                  vrep.simx_opmode_oneshot_wait)
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
robot_handles = [robots[i].robot_handle for i in range(num_robots)]
robots_position = [list()] * num_robots  # position of robots
rob_dist = [list()] * num_robots  # distance of robots
rob_orien = [list()] * num_robots  # orientation of robots
prev_cal_angle = [[] for _ in range(num_robots)]  # Calculated angle in previous trial
prev_cal_mag = [[] for _ in range(num_robots)]  # Calculated magnitude in previous trial
exp_pre_orien = [0.0] * num_robots  # expected robot position in previous trial
str_mov = [True] * num_robots
spat_fitness = []  # spatial fitness during
com_flock_pos = []  # Center of mass position for complete trial
#robots_position = [list()] * num_robots  # Position of robot
t = 0  # Current Iteration
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)  # Automatically start simulation from V-rep
print('Simulation started')
loop_start_time = time.time()  # S  tart measuring time
for i in range(num_robots):  # initially make velocity of each robot equal to zero
    robots[i].wait(0.0)
#------------------Main Loop-----------------#
while current_time - loop_start_time < 500:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    t1 = time.time()
    #----------------Calculate inflammation in each iteration------------#
    #for l in range(num_robots):
        #robots_position[l], disp, dir_rob_wax = robots[l].get_position(-1)
    comb_clusters, size_clusters, main_cluster, max_cluster_size = flock_members(robots_position, 0.35)
    target_flock_perf = 0.3  # Center of mass of flock must be 0.3m away from target
    target_spat = interp(max_cluster_size, [1, 15], [0.3, 0.5])
    cluster_rob_pos = [ele for i, ele in enumerate(robots_position) if i in main_cluster]
    cluster_rob_handles = [robot_handles[rob] for rob in comb_clusters]  # handles of robot part of cluster
    #print('len of cluster_rob_pos', len(cluster_rob_pos))
    comp_flock, com_flock = fitness_aggregation(cluster_rob_pos)
    spat_fitness.append(comp_flock)
    com_flock_pos.append(com_flock)
    # ----------Loop to measure control parameter for each robot----------#
    for i in range(num_robots):  # measure control parameter for each robot
        print('robot number # ', i)
        rot_theta = 0.0
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        robots[i].sensor_mod_values(0.10, 0.15)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(0.15, 0.25)  # modified sensor values for object avoidance
        # -----------------Random levy search in black area-------------#
        wondering_rob = True if set(robots[i].det_rob).intersection(set(cluster_rob_handles)) else False   # condition for wondering robot
        if wondering_rob :  # a robot outside is unknown to target point and in search of flock
            obj_theta = robots[i].object_avoidance(robots[i].det_obj_handles, 0.20)
            if not robots[i].obj_avoid:
                levy_theta = robots[i].random_rotation_angle()
                if levy_theta == 0.0:
                    v_l[i], v_r[i], rot_t[i] = robots[i].levy_flight(1.5, 1)
                    print('Robot levy movement in black area')
                    continue
                else:
                    #print('Robot {} is rotating in levy search and ir value is {}'.format(m, ir_value))
                    v_l[i], v_r[i], rot_t[i] = robots[i].control_param(levy_theta, 1.5)
                    continue
            else:  # set parameters for object avoidance in black area
                v_l[i], v_r[i], rot_t[i] = robots[i].control_param(obj_theta, 1.5)
                continue
        obj_theta = robots[i].object_avoidance(robots[i].static_object_handles, 0.20)
        rot_theta = obj_theta
        # To calculate center of mass # New COM will be calculated for previos rotation and straight movemt has been perfomed
        if not robots[i].obj_avoid and not (prev_cal_mag[i] and prev_cal_angle[i]):
            returnCode, target_pos = vrep.simxGetObjectPosition(clientID, point_cloud, robots[i].robot_handle,
                                                                vrep.simx_opmode_streaming)
            print('target_point', target_pos)
            #grad_all, max_grad = robots[i].chemotaxis_gradient()
            target_mag, target_angle = angles_calcu(target_pos[2], target_pos[1], 0, 0)  # get pos of target point vector w.r.t world frame of ref
            if current_time - loop_start_time < 15:  # In s tart of simulatiom all robots will directed to gradient
                weight_forces = 0.3
                weight_target = 1
            else:  # other robots in gradient arena
                if spat_fitness[-1] > target_spat:
                    weight_forces = 1.5  # -1.2
                    weight_target = 1/8  # 0.5
                else:
                    weight_forces = 1
                    weight_target = 1/6
            com_x, com_y = robots[i].flock_com(target_mag, target_angle, weight_forces, weight_target)  # New COM calculated in terms of co-ordinates
            curr_com_mag, curr_com_angle = angles_calcu(com_x, com_y, 0, 0)  # New COM calculated in terms vector
            print('Calculated Angle in that trial is', curr_com_angle * 180/math.pi)
            prev_cal_mag[i].append(max(0.15, curr_com_mag))  #  update variable of previous com magnitude
            prev_cal_angle[i].append(- curr_com_angle)  # update variable of previous com angle
            rot_theta = - curr_com_angle
            #---------------orientation of robot and rotation regulation factor-----------#
            exp_pre_orien[i] = rob_orien[i] if exp_pre_orien[i] == 0.0 else exp_pre_orien[i]
            diff_angle = math.pi - abs(abs(exp_pre_orien[i] - rob_orien[i]) - math.pi)
            #print('Robot orientation is', rob_orien * 180/math.pi)
            #print('Expected orientation is', exp_pre_orien[i] * 180/math.pi)
            reg_fac = 1 + abs(diff_angle/exp_pre_orien[i])
            print('regulation_fac', reg_fac)
            exp_pre_orien[i] = rob_orien[i] + rot_theta  # update previous orientation for next iteration
        t2 = time.time()  # Unnecessary
        if str_mov[i] or robots[i].obj_avoid:  # robot will rotate for flocking or for obj avoid
            print('Robot is rotating')
            if prev_cal_angle[i]:  # to rotate towards center of mass
                rot_command = prev_cal_angle[i].pop()  # assign variable to rotate for com
                #print('Rotation command', rot_command * 180/math.pi)
                #print('Angle has been assigned')
            else:  # for obj avoidance rotation
                rot_command = obj_theta
            v_l[i], v_r[i], rot_t[i] = robots[i].rotation_robot(rot_command, reg_fac)  # either 1 or regulation factor can be given as input
            print('left velocity', v_l[i])
            print('right velocity', v_r[i])
            str_mov[i] = False  # to switch between two states (straight and rotate)
        else:
            print('Robot is moving straight')
            prev_cal_angle[i].clear()  # clear angle list to make straight movement is a last step
            disp_command = prev_cal_mag[i].pop()
            print('Displacement command is ', disp_command)
            v_l[i], v_r[i], rot_t[i] = robots[i].displacement_robot(disp_command)
            #v_l[i], v_r[i], rot_t[i] = robots[i].straight_gradient_movement()
            print('left velocity', v_l[i])
            print('right velocity', v_r[i])
            str_mov[i] = True
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
    print('Implemented')
    for i in range(num_robots):
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(-1)  # current orienation
    t += 1
vrep.simxPauseCommunication(clientID, False)