import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
import numpy as np  # array library
import math
from numpy import interp
import matplotlib.pyplot as plt
from utlis import *

#----------------9-11-2021--------------#
# Foraging behavior using chemotaxis gradient in obstacle environment has been implemented
# Vrep recommended file is "Swarm_robotics_foraging_behavior"
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
loop_start_time = time.time()
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
foods_pos = []  # food position w.r.t ref frame
rot_rob_pre = [0.0] * num_robots
disp_rob_pre = [0.0] * num_robots
str_mov = [True] * num_robots  # Condition for straight movement
rob_status = [False] * num_robots  # either is searching or moving to nest
prev_cal_angle = [[] for _ in range(num_robots)]  # Calculated angle in previous trial
prev_cal_mag = [[] for _ in range(num_robots)]  # Calculated magnitude in previous trial
exp_pre_orien = [0.0] * num_robots  # expected robot position in previous trial
robots_position = [list()] * num_robots  # position of robots w.r.t frame of reference
rob_dist = [list()] * num_robots  # dist of robots w.r.t frame of reference
rob_orien = [list()] * num_robots  # orientation of robots w.r.t frame of reference
all_rob_pick_food = np.array([False] * num_robots)  # Status of all robots having food or not
all_nest_info = [[] for _ in range(num_robots)]  # Robots containing nest information
food_handles = []  # Handles of all food items
food_items = [str(i) for i in range(20)]
for i in food_items:  # Loop to acquire Handles of all objects
    errorCode, food_handle = vrep.simxGetObjectHandle(clientID, 'Food' + i, vrep.simx_opmode_blocking)
    food_handles.append(food_handle)
food_handles = [i for i in food_handles if i != 0]
for food_handle in food_handles:  # loop to acquire position of food
    returnCode, food_pos = vrep.simxGetObjectPosition(clientID, food_handle, robots[0].ref_frame_handle,
                                                      vrep.simx_opmode_blocking)
    foods_pos.append(food_pos[:2])
avoid_food_handles = food_handles
arena_x, arena_y, food_pos_im = cartesian_im_trans(3.1, 3.1, 0.025, foods_pos)
chem_im = np.zeros((arena_x, arena_y))
mono_im = np.zeros((arena_x, arena_y))
time_consume = []  # time consumed in each iteration (implementation of control parameters)
inflammation = []  # sum of ir values of all robots for all iterations
t = 0
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
while current_time - loop_start_time < 600:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    #----------------Calculate inflammation in each iteration------------#
    for i in range(num_robots):
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(robots[0].ref_frame_handle)  # Current orienation and position w.r.t frame of reference
        all_rob_pick_food[i] = robots[i].food_picked
        all_nest_info[i] = robots[i].nest_info  # Update robots nest info
    arena_x, arena_y, rob_pos_im = cartesian_im_trans(3.1, 3.1, 0.025, robots_position)  # Convert robots pos in pixels
    rob_pos_im_mod = rob_pos_im[all_rob_pick_food == False]  # Only free robots will induce monokines
    #print('Robot Orientations are', np.array(rob_orien) * (180/math.pi))
    mono_im = chemotaxis_gradient_forging(mono_im, rob_pos_im_mod, 'von_neumann', 0.15, 0.025, 0.35, 100)  # Updating mono-taxis mask
    print('mono im max', np.max(mono_im))
    chem_im = chemotaxis_gradient_forging(chem_im, food_pos_im, 'von_neumann', 0.125, 0.021, 0.35, 100)  # Updating chemo-taxis mask
    #----------Loop to measure control parameter for each robot----------#
    for i in range(num_robots):
        print('Robot number = ', i)
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        vision_values = robots[i].read_vision_sensor()  # reading values from vision sensors
        front_ir_value, front_ir_status = robots[i].front_vision_sensor()
        robots[i].sensor_mod_values(0.1, 0.15)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(0.15, 0.25)  # modified sensor values for object avoidance
        chem_avg_value = average_filter(chem_im, rob_pos_im[i])  # Average chemo value around robot
        avg_ir_value = np.sum(vision_values) / 8
        grad_ir_nest = front_ir_value - vision_values[6]  # calculating gradient of ir values for robot to drop food
        if not robots[i].nest_info:  # Update nest info of an unknown robot
            detected_robot = set(robot_handles).intersection(set(robots[i].det_rob))
            detected_rob_num = [robot_handles.index(m) for m in detected_robot]
            for n in detected_rob_num:
                robots[i].nest_info = robots[n].nest_info
        nest_search = robots[i].food_picked and avg_ir_value < 0.05 and not bool(robots[i].nest_info) # Condition for searching nest levy flight
        move_nest = robots[i].food_picked and avg_ir_value > 0.05  # -ve gradient of ir sensors
        leave_nest = not robots[i].food_picked and avg_ir_value > 0.05  # +ve gradient of ir sensors
        informed_robot = bool(robots[i].nest_info) and robots[i].food_picked and avg_ir_value < 0.05  # knows about nest
        print('condition for nest search', nest_search)
        print('condition for moving into nest search', move_nest)
        print('condition for moving out nest search', leave_nest)
        print('condition for moving Target', informed_robot)
        print('chem_avg_value', chem_avg_value)
        chem_avg_value = 1 if nest_search or move_nest or leave_nest else chem_avg_value  # To end influence of chemo arrah
        avoidable_obj = robots[i].static_object_handles + robot_handles + (avoid_food_handles if robots[i].food_picked else [])
        # -----------------Random levy search in black area-------------#
        if (chem_avg_value < 0.0001 and not bool(robots[i].nest_info)) or nest_search:  # threshold to check robot is in gradient of food or looking for nest
            obj_theta = robots[i].object_avoidance(avoidable_obj, 0.20)  # obj avoidance will be for robots and objects
            if not robots[i].obj_avoid:
                levy_theta = robots[i].random_rotation_angle()
                if levy_theta == 0.0:
                    v_l[i], v_r[i], rot_t[i] = robots[i].levy_flight(2.5, 1)
                    print('Robot levy movement')
                    continue
                else:
                    # print('Robot {} is rotating in levy search '.format(m))
                    v_l[i], v_r[i], rot_t[i] = robots[i].control_param(levy_theta, 1.5)
                    continue
            else:  # set parameters for object avoidance during levy searching
                v_l[i], v_r[i], rot_t[i] = robots[i].control_param(obj_theta, 1.5)
                continue
        #vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
        #vision_max_grad = clamp(vision_max_grad, 0.1, 0.25)
        #----------------Robot in gradient of food-------------------#
        obj_theta = robots[i].object_avoidance(avoidable_obj, 0.20)
        rot_theta = obj_theta
        picked_food = list(set(robots[i].det_obj_handles[1:4]).intersection(set(food_handles)))
        #pick_cond = robots[i].det_obj_handles[2] in food_handles
        if picked_food and not robots[
            i].food_picked:  # Pick food from Arena, when robot does not have food
            print('Food is found by robot')
            robots[i].pick_food_object(picked_food[0])  # other method robots[i].det_obj_handles[2]
            index_cap_food = food_handles.index(picked_food[0])  # find index of captured food handle
            food_handles.remove(picked_food[0])  # Remove food element from food list
            cap_food_pos = food_pos_im[index_cap_food]  # Acquire pos of captured food and remove it
            chem_im[cap_food_pos[0]][cap_food_pos[1]] = 0.0  # Update chemo array
            food_pos_im = np.delete(food_pos_im, index_cap_food, axis=0)  # Remove the position of capture food
            continue
        if not robots[i].obj_avoid and not (prev_cal_mag[i] and prev_cal_angle[i]):  # To calculate center of mass
            if informed_robot:  # Use its knowledge to reach target location
                print('Following its knowledge')
                vision_max_grad, vision_grad_theta = angles_calcu(robots[i].nest_info[0], robots[i].nest_info[1], robots_position[i][0], robots_position[i][1])  # Angle of nest w.r.t robot
                vision_grad_theta = ((vision_grad_theta - rob_orien[i]) + math.pi) % (2 * math.pi) - math.pi  # Finding exact required angle of movement including robot orientation
                prev_cal_angle[i].append(vision_grad_theta)  # update variable of previous angle
                prev_cal_mag[i].append(clamp(vision_max_grad, 0.05, 0.1))  # update variable of previous  magnitude
                continue
            elif move_nest:  # move into nest, to drop food
                print('Moving into nest')
                print('AVERAGE ir VALUE IS', avg_ir_value)
                vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
                vision_max_grad = clamp(vision_max_grad, 0.1, 0.2)
                prev_cal_angle[i].append(vision_grad_theta)  # update variable of previous angle
                prev_cal_mag[i].append(vision_max_grad)  # update variable of previous  magnitude
                if avg_ir_value > 0.65 and grad_ir_nest > 0:
                    print('Food is dropped--------------------------------------')
                    robots[i].drop_food_object()
                    robots[i].nest_info = robots_position[i]  # update nest info of a robot who reached at nest
                    prev_cal_angle[i].clear()
                    prev_cal_angle[i].append(0.75 * math.pi)
                    prev_cal_mag[i].clear()
                    #  Move robot 180 degree to move away
                continue
            elif leave_nest:  # move away  nest, to search new food
                print('Moving out from nest')
                vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(-vision_values/2)
                vision_max_grad = clamp(- vision_max_grad/2, 0.1, 0.25)
                prev_cal_angle[i].append(vision_grad_theta)  # update variable of previous angle
                prev_cal_mag[i].append(vision_max_grad)  # update variable of previous  magnitude
                continue
            all_grad, max_grad = chemotaxis_gradient_foraging(chem_im, mono_im, rob_pos_im[i], 0.001)
            chemo_theta = robots[i].rotational_angle(all_grad, max_grad, obj_theta)  #  Require diection of Robot calculated by chemotaxis
            rot_theta = ((chemo_theta - rob_orien[i]) + math.pi) % (2 * math.pi) - math.pi
            disp_value = 0.0375 if chem_avg_value > 0.045 else 0.075  # displacement value inversely proportional to presence of chemical
            prev_cal_angle[i].append(rot_theta)  # update variable of previous angle
            prev_cal_mag[i].append(disp_value)   # update variable of previous  magnitude
        if str_mov[i] or robots[i].obj_avoid:  # robot will rotate for flocking or for obj avoid
            print('Robot is rotating')
            if prev_cal_angle[i]:  # to rotate towards food
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
            v_l[i], v_r[i], rot_t[i] = robots[i].displacement_robot(disp_command)
            prev_cal_mag[i].clear()
            #print('left velocity', v_l[i])
            #print('right velocity', v_r[i])
            str_mov[i] = True
    #-------------------Implementation of Control Parameters---------------#
    for j in range(num_robots):  # Starting movement of all robots Loop
        robots[j].movement(v_l[j], v_r[j])
        start_time[j] = time.time()  # Calculation of start time
    init_time = time.time()  # time for the calculation of instantaneous velocity
    while rob:  # Loop will end when 'rob' list will be empty
        current_time = time.time()
        for k in rob:
            diff = (current_time - start_time[k])
            # print('difference of time', diff)
            if diff >= rot_t[k]:
                # print('For robot {} Target time is {} and Passed time is {}'.format(k, rot_t[k], diff))
                rob.remove(k)
                robots[k].wait(0.0)  # Stop of robot
    final_time = time.time()  # time for the calculation of instantaneous velocity
    time_consume.append(final_time - init_time)  # total time consumed for each implementation of control parameters
    t += 1
    #plt.title('Monokine Plot')
    #plt.xlabel('Y-axis')
    #plt.ylabel('X-axis')
    #plt.imshow(mono_im, origin='lower')
    #plt.show()
vrep.simxPauseCommunication(clientID, False)
#time.sleep(5)
#vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
plt.title('Food Location Plot')
plt.xlabel('Y-axis')
plt.ylabel('X-axis')
plt.imshow(np.transpose(chem_im), origin='lower')
plt.show()

