import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
import numpy as np  # array library
import math
from robot_epuck import Robot
from utlis import *

#----------------6-7-2021--------------#
# Flocking behavior in gradient environment has been implemented
# Vrep recommended file is "flocking_gradient"
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

with open('robot_file', 'rb') as f:  # to generate your own file  (rb for wrting in bytes)
    robots = pickle.load(f)  # storing variable into file
num_robots = len(robots)
#num_robots = 5
#robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
rot_rob_pre = [0.0] * num_robots
disp_rob_pre = [0.0] * num_robots
prev_cal_angle = []  # Calculated angle in previous trial
prev_cal_mag = []  # Calculated magnitude in previous trial
exp_pre_orien = [0.0] * num_robots  # expected robot position in previous trial
t = 0
str_mov = [True] * num_robots
while current_time - loop_start_time < 200:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    for i in range(num_robots):  # measure control parameter for each robot
        print('Robot number = ', i)
        rot_theta = 0.0
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        robots[i].sensor_mod_values(0.10, 0.15)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(0.15, 0.25)  # modified sensor values for object avoidance
        obj_theta = robots[i].object_avoidance(robots[i].static_object_handles, 0.20)
        rot_theta = obj_theta
        if not robots[i].obj_avoid and not (prev_cal_mag and prev_cal_angle):  # To calculate center of mass
            vision_values = robots[i].read_vision_sensor()
            print('Vision sensor values are', vision_values)
            vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
            print('moving displacement', vision_max_grad)
            com_x, com_y = robots[i].flock_com(vision_max_grad, - vision_grad_theta, 3.5, 1)  # New COM calculated in terms of co-ordinates
            curr_com_mag, curr_com_angle = angles_calcu(com_x, com_y, 0, 0)  # New COM calculated in terms vector
            prev_cal_mag.append(curr_com_mag)  # update variable of previous com magnitude
            prev_cal_angle.append(curr_com_angle)  # update variable of previous com angle
            rot_theta = - curr_com_angle
            # ---------------orientation of robot and rotation regulation factor-----------#
            rob_pos, rob_dist, rob_orien = robots[i].get_position()  # current orienation
            exp_pre_orien[i] = rob_orien if exp_pre_orien[i] == 0.0 else exp_pre_orien[i]
            diff_angle = math.pi - abs(abs(exp_pre_orien[i] - rob_orien) - math.pi)
            print('Robot orientation is', rob_orien * 180 / math.pi)
            print('Expected orientation is', exp_pre_orien[i] * 180 / math.pi)
            reg_fac = 1 + (diff_angle / exp_pre_orien[i])
            print('regulation_fac', reg_fac)
            exp_pre_orien[i] = rob_orien + rot_theta  # update previous orientation for next iteration
        if str_mov[i] or robots[i].obj_avoid:  # robot will rotate for flocking or for obj avoid
            print('Robot is rotating')
            if prev_cal_angle:  # to rotate towards center of mass
                rot_command = prev_cal_angle.pop()  # assign variable to rotate for com
                # print('Angle has been assigned')
            else:  # for obj avoidance rotation
                rot_command = obj_theta
            v_l[i], v_r[i], rot_t[i] = robots[i].rotation_robot(rot_command, - reg_fac)  # -ve means towards and +ve means away
            print('left velocity', v_l[i])
            print('right velocity', v_r[i])
            str_mov[i] = False  # to switch between two states (straight and rotate)
        else:
            print('Robot is moving straight')
            prev_cal_angle.clear()  # clear angle list to make straight movement is a last step
            disp_command = prev_cal_mag.pop() * 2.5
            print('disp_command',disp_command)
            v_l[i], v_r[i], rot_t[i] = robots[i].displacement_robot(disp_command)
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
    # for i in range(num_robots):
    # robots_position[i] = robots[i].get_position()
    t += 1
vrep.simxPauseCommunication(clientID, False)