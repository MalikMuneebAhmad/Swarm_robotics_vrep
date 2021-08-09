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
robot_handles = [robots[i].robot_handle for i in range(num_robots)]
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
rot_rob_pre = [0.0] * num_robots
disp_rob_pre = [0.0] * num_robots
all_rob_allocation = [False] * num_robots
prev_cal_angle = [[] for _ in range(num_robots)]  # Calculated angle in previous trial
prev_cal_mag = [[] for _ in range(num_robots)]  # Calculated magnitude in previous trial
exp_pre_orien = [0.0] * num_robots  # expected robot position in previous trial
all_semi = []
t = 0
str_mov = [True] * num_robots
while current_time - loop_start_time < 100:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    for i in range(num_robots):  # measure control parameter for each robot
        print('Robot number = ', i)
        rot_theta = 0.0
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        vision_values = robots[i].read_vision_sensor()  # reading values from vision sensors
        avg_ir_value = np.sum(vision_values) / 8
        front_ir_raw, front_ir_value = robots[i].front_vision_sensor()
        robots[i].sensor_mod_values(0.10, 0.15)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(0.15, 0.25)  # modified sensor values for object avoidance
        robots[i].presence_in_gradient = avg_ir_value > 0.1  # true when robot in black area
        # ----------------Robot is present on black and gradient area ---------------#
        all_rob_allocation = [robots[i].presence_in_gradient for i in
                              range(num_robots)]  # list of state (black and gradient area) of all robots
        # ----------------Semi mature condition evaluation-----------------#
        # ------------Check a robot from black area is a neighbor of gradient robot---------#
        robots[i].semi_mature_flock, leader_robot = gradient_rob_nei(robot_handles, robots[i].det_obj_handles,
                                                            all_rob_allocation, robots[i].presence_in_gradient)
        #print('for robot {} semi mature flocking is {}'.format(i, robots[i].semi_mature_flock))
        #print('Position of leader is ', leader_robot_position)
        all_semi.append(robots[i].semi_mature_flock)
        #-----------------regulate a robot at boundry of black and gradient---------#
        front_ir_raw = 0 if robots[i].sensors_detecting[2] == 0 else front_ir_raw  # for object in front of robot
        grad_front_ir = front_ir_raw - vision_values[6]
        if grad_front_ir >= 0 and (front_ir_raw < 0.05 or vision_values[6] < 0.05):
            factor = 1 if grad_front_ir >= 0 else -1
            v_l[i], v_r[i], rot_t[i] = robots[i].random_comp_straight(0.9 * factor, 1)
            print("Robot will regulate at boundry of black area")
            continue
        # -----------------Random levy search in black area-------------#
        if avg_ir_value < 0.1 and not robots[i].semi_mature_flock:  # avg intensity of 8 sensors is low in black area
            obj_theta = robots[i].object_avoidance(robots[i].det_obj_handles, 0.225)
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
        #----------------Robot in gradient area-------------------#
        obj_theta = robots[i].object_avoidance(robots[i].static_object_handles, 0.20)
        rot_theta = obj_theta
        #print('prev_cal_mag', prev_cal_mag)
        #print('prev_cal_angle', prev_cal_angle)
        if not robots[i].obj_avoid and not (prev_cal_mag[i] and prev_cal_angle[i]):  # To calculate center of mass
            #print('Vision sensor values are', vision_values)
            vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
            if robots[i].semi_mature_flock:
                returnCode, leader_robot_position = vrep.simxGetObjectPosition(clientID, robot_handles[leader_robot],
                                                                robot_handles[i], vrep.simx_opmode_streaming)  # robot pos w.r.t world frame of reference
                vision_max_grad, vision_grad_theta = angles_calcu(leader_robot_position[2] , leader_robot_position[1], 0, 0)
                print('for robot {} semi mature flocking is {}'.format(i, robots[i].semi_mature_flock))
            if current_time - loop_start_time < 8:  # In start of simulatiom all robots will directed to gradient
                weight_forces = 0.3
                weight_target = 1
            elif robots[i].semi_mature_flock:  # if robot at edge and detecting a gradient area robot
                weight_forces = -0.0
                weight_target = - 1.0
            else:  # other robots in gradient arena
                weight_forces = -0.7
                weight_target = 1.0
                print('robot {} in gradient'.format(i))
            com_x, com_y = robots[i].flock_com(vision_max_grad, vision_grad_theta, weight_forces, weight_target)  # New COM calculated in terms of co-ordinates
            curr_com_mag, curr_com_angle = angles_calcu(com_x, com_y, 0, 0)  # New COM calculated in terms vector
            prev_cal_mag[i].append(clamp(curr_com_mag, 0.05, 0.1))  # update variable of previous com magnitude
            prev_cal_angle[i].append(curr_com_angle)  # update variable of previous com angle
            rot_theta = - curr_com_angle
            # ---------------orientation of robot and rotation regulation factor-----------#
            rob_pos, rob_dist, rob_orien = robots[i].get_position()  # current orienation
            exp_pre_orien[i] = rob_orien if exp_pre_orien[i] == 0.0 else exp_pre_orien[i]
            diff_angle = math.pi - abs(abs(exp_pre_orien[i] - rob_orien) - math.pi)
            #print('Robot orientation is', rob_orien * 180 / math.pi)
            #print('Expected orientation is', exp_pre_orien[i] * 180 / math.pi)
            reg_fac = 1 + abs(diff_angle / exp_pre_orien[i])
            #print('regulation_fac', reg_fac)
            exp_pre_orien[i] = rob_orien + rot_theta  # update previous orientation for next iteration
        if str_mov[i] or robots[i].obj_avoid:  # robot will rotate for flocking or for obj avoid
            print('Robot is rotating')
            if prev_cal_angle[i]:  # to rotate towards center of mass
                rot_command = prev_cal_angle[i].pop()  # assign variable to rotate for com
                # print('Angle has been assigned')
            else:  # for obj avoidance rotation
                rot_command = obj_theta
            v_l[i], v_r[i], rot_t[i] = robots[i].rotation_robot(rot_command, 1)  # -ve means towards and +ve means away
            #print('left velocity', v_l[i])
            #print('right velocity', v_r[i])
            str_mov[i] = False  # to switch between two states (straight and rotate)
        else:
            print('Robot is moving straight')
            prev_cal_angle[i].clear()  # clear angle list to make straight movement is a last step
            disp_command = prev_cal_mag[i].pop()
            #print('Magnitude list of a robot after pop', prev_cal_mag[i])
            #print('disp_command',disp_command)
            #print('robots[i].front_sensor_ok', robots[i].front_sensor_ok)
            #print('robots[i].front_ok', robots[i].front_ok)
            #print('robot sum of values ', np.sum(robots[i].sensor_raw_values[5:]))
            if robots[i].front_sensor_ok and robots[i].front_ok and (np.sum(robots[i].sensor_raw_values[5:])/max(1, np.count_nonzero(robots[i].sensors_detecting[5:] == 1))) > 0.15:
                v_l[i], v_r[i], rot_t[i] = robots[i].backward_leader_move()
            else:
                v_l[i], v_r[i], rot_t[i] = robots[i].displacement_robot(disp_command)
            prev_cal_mag[i].clear()
            #print('left velocity', v_l[i])
            #print('right velocity', v_r[i])
            str_mov[i] = True
    print('Semi mature condition of all robots', all_semi)
    all_semi.clear()
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