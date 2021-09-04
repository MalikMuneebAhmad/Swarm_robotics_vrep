import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
import numpy as np  # array library
import math
from numpy import interp
import matplotlib.pyplot as plt
from utlis import *

#----------------6-7-2021--------------#
# Flocking behavior in diagonale gradient obstacle  environment has been implemented
# Vrep recommended file is "flocking_towards_light_simplified_epuck"
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
all_rob_allocation = [False] * num_robots  # either in gradient or in black area
avg_ir_value = np.zeros(num_robots)  # average ir values for all robots
perf_ir_value = np.zeros(num_robots)  # performance ir values for all robots
prev_cal_angle = [[] for _ in range(num_robots)]  # Calculated angle in previous trial
prev_cal_mag = [[] for _ in range(num_robots)]  # Calculated magnitude in previous trial
exp_pre_orien = [0.0] * num_robots  # expected robot position in previous trial
all_semi = []  # semi mature status of robots
robots_position = [list()] * num_robots  # position of robots
spat_fitness = []  # spatial fitness during
com_flock_pos = []  # Center of mass position for complete trial
time_consume = []  # time consumed in each iteration (implementation of control parameters)
inst_vel = []  # Instantaneous velocity of Center of Mass
inflammation = []  # sum of ir values of all robots for all iterations
t = 0
str_mov = [True] * num_robots
flock_perf = 1
target_flock_perf = 0.9
#while current_time - loop_start_time < 180:  # Main loop
while flock_perf > target_flock_perf:
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    #----------------Calculate inflammation in each iteration------------#
    for l in range(num_robots):
        robots_position[l], disp, dir_rob_wax = robots[l].get_position()
    comb_clusters, size_clusters, main_cluster, max_cluster_size = flock_members(robots_position, 0.35)
    target_flock_perf = interp(max_cluster_size, [1, 15], [0.4, 0.6])  # For diagnol movement limits are (0.15 - 0.55) otherwisw (0.05 - 0.45)
    target_spat = interp(max_cluster_size, [1, 15], [0.3, 0.5])
    cluster_rob_pos = [ele for i, ele in enumerate(robots_position) if i in main_cluster]
    #print('len of cluster_rob_pos', len(cluster_rob_pos))
    comp_flock, com_flock = fitness_aggregation(cluster_rob_pos)
    spat_fitness.append(comp_flock)
    com_flock_pos.append(com_flock)
    #----------Loop to measure control parameter for each robot----------#
    for i in range(num_robots):
        print('Robot number = ', i)
        rot_theta = 0.0
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        vision_values = robots[i].read_vision_sensor()  # reading values from vision sensors
        avg_ir_value[i] = np.sum(vision_values) / 8  # avg ir value of vision sensor for a robot
        avg_ir_value[i] = 0.0 if avg_ir_value[i] == 1 else avg_ir_value[i]  # Ir vale zero when ir sensor is not responding at start
        print('avg_ir_value', avg_ir_value[i])
        #perf_ir_value[i] = (avg_ir_value[i] * robots[i].no_nei)/4 if i not in main_cluster else avg_ir_value[i]  # effect of performance due to abandoned robot will be less
        perf_ir_value[i] = avg_ir_value[i] if i in main_cluster else 0  # effect of performance due to abandoned robot will be less
        front_ir_raw, front_ir_value = robots[i].front_vision_sensor()
        robots[i].sensor_mod_values(0.11, 0.14)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(0.15, 0.25)  # modified sensor values for object avoidance
        robots[i].presence_in_gradient = avg_ir_value[i] > 0.1  # true when robot in black area
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
        if grad_front_ir > 0 and (front_ir_raw < 0.05 or vision_values[6] < 0.05):
            factor = 1 if grad_front_ir >= 0 else -1
            v_l[i], v_r[i], rot_t[i] = robots[i].random_comp_straight(0.9 * factor, 1)
            print("Robot will regulate at boundry of black area")
            continue
        # -----------------Random levy search in black area-------------#
        if avg_ir_value[i] < 0.1 and not robots[i].semi_mature_flock:  # avg intensity of 8 sensors is low in black area
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
        #----------------Robot in gradient area-------------------#
        obj_theta = robots[i].object_avoidance(robots[i].static_object_handles, 0.20)
        rot_theta = obj_theta
        #print('prev_cal_mag', prev_cal_mag)
        #print('prev_cal_angle', prev_cal_angle)
        if not robots[i].obj_avoid and not (prev_cal_mag[i] and prev_cal_angle[i]):  # To calculate center of mass
            #print('Vision sensor values are', vision_values)
            vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
            vision_max_grad = clamp(vision_max_grad, 0.1, 0.25)
            if not robots[i].det_rob:
                vision_max_grad = 3
            print('vision_max_grad', vision_max_grad)
            #print('Robot {} vision sensor gradient is {}'.format(i, vision_max_grad))
            if robots[i].semi_mature_flock:
                returnCode, leader_robot_position = vrep.simxGetObjectPosition(clientID, robot_handles[leader_robot],
                                                                robot_handles[i], vrep.simx_opmode_streaming)  # robot pos w.r.t world frame of reference
                vision_max_grad, vision_grad_theta = angles_calcu(leader_robot_position[2], leader_robot_position[1], 0, 0)
                vision_grad_theta = -vision_grad_theta
                perf_ir_value[i] = 0.5  # Robot, part of flock but present in black area, have 0.5 IR value
                print('for robot {} semi mature flocking is {}'.format(i, robots[i].semi_mature_flock))
            if current_time - loop_start_time < 15:  # In s tart of simulatiom all robots will directed to gradient
                weight_forces = 0.3
                weight_target = 1
            elif robots[i].semi_mature_flock:  # if robot at edge and detecting a gradient area robot
                weight_forces = -0.0
                weight_target = 1.0
            else:  # other robots in gradient arena
                if spat_fitness[-1] > target_spat:
                    weight_forces = -1.0  # -1.2
                    weight_target = 0.8  # 0.5
                else:
                    weight_forces = -0.5
                    weight_target = 1.0
                print('robot {} in gradient'.format(i))
            com_x, com_y = robots[i].flock_com(vision_max_grad, vision_grad_theta, weight_forces, weight_target)  # New COM calculated in terms of co-ordinates
            curr_com_mag, curr_com_angle = angles_calcu(com_x, com_y, 0, 0)  # New COM calculated in terms vector
            prev_cal_mag[i].append(clamp(curr_com_mag, 0.05, 0.1))  # update variable of previous com magnitude
            prev_cal_angle[i].append(curr_com_angle)  #  update variable of previous com angle
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
            #if robots[i].front_sensor_ok and robots[i].front_ok and (np.sum(robots[i].sensor_raw_values[5:])/max(1, np.count_nonzero(robots[i].sensors_detecting[5:] == 1))) > 0.15:
            if avg_ir_value[i] > (1 - target_flock_perf):  # staying and keeping distance
                v_l[i], v_r[i], rot_t[i] = robots[i].straight_gradient_movement(60, True)
            elif robots[i].front_sensor_ok and robots[i].front_ok and (1 in robots[i].sensors_detecting[5:]) and abs(np.sum(robots[i].sensor_values[5:])/max(1, np.count_nonzero(robots[i].sensors_detecting[5:] == 1))) > 0.04:
                v_l[i], v_r[i], rot_t[i] = robots[i].backward_leader_move(10)
            else:
                print('combine effect ')
                #v_l[i], v_r[i], rot_t[i] = robots[i].displacement_robot(disp_command)
                vlg, vrg, rottg = robots[i].displacement_robot(disp_command)
                v_l[i], v_r[i], rot_t[i] = robots[i].straight_gradient_movement(30, False)
                disp_fact = 0.4 if spat_fitness[-1] > target_spat else 0.7
                #disp_fact = 0.5 + ((target_spat - spat_fitness[-1]) * 1.5)
                v_l[i] += disp_fact * vlg
                v_r[i] += disp_fact * vrg
                rot_t[i] += disp_fact * rottg
            prev_cal_mag[i].clear()
            #print('left velocity', v_l[i])
            #print('right velocity', v_r[i])
            str_mov[i] = True
    #print('Semi mature condition of all robots', all_semi)
    all_semi.clear()
    print('Ir Value of all robots', perf_ir_value)
    print('Performance factor is ', flock_perf)
    print('Target Ir value is ', (1 - target_flock_perf))
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
    flock_perf = sum(1 - perf_ir_value) / max_cluster_size  # group compactness component (GCC)
    inflammation.append(flock_perf)
    t += 1
vrep.simxPauseCommunication(clientID, False)
#-------------Calculation of Instantaneous and average velocity--------------#
for i in range(2, t):
    inst_vel.append(dist_btw_points(com_flock_pos[i], com_flock_pos[i - 1]))  # /time_consume[i - 1])
gsc = (1 + (np.array(inst_vel) * 1000))/2
gcc = np.array(spat_fitness[2:])
w = 0.4
f = w * gcc + (w - 1) * gsc
flock_dist = dist_btw_points(com_flock_pos[-1], com_flock_pos[1])
total_time_con = sum(time_consume[1:])
avg_vel = flock_dist/total_time_con
avg_compactness = sum(spat_fitness[1:]) / (t - 1)
#-----------Plotting Instantaneous velocity and spatiousness--------------#
plt.title('Instantaneous Velocity of COM')
plt.xlabel('Iteration #')
plt.ylabel('Velocity of flock')
plt.plot(list(range(t - 2)), inst_vel)
plt.show()
plt.title('Compactness of Flock')
plt.xlabel('Iteration #')
plt.ylabel('Compactness of Robots')
plt.plot(list(range(t)), spat_fitness, color='b', label='Compactness')
plt.plot([1, t], [avg_compactness, avg_compactness], color='g', label='Average Compactness')
#plt.plot([1, t], [0.4, 0.4], color='r', label='Require Compactness')
plt.legend()
plt.show()
plt.title('Inflammation of ir')
plt.xlabel('Iteration #')
plt.ylabel('Combine Ir value of ron=bots')
plt.plot(list(range(t)), inflammation)
plt.show()
plt.title('Combine inflammation')
plt.xlabel('Iteration #')
plt.ylabel('Gcc +Gsc')
plt.plot(list(range(len(f))), f)
plt.show()
print('Number of robots remain in flock', max_cluster_size)
print('Distance traveled by flock is {} m'.format(flock_dist))
print('Time consumed to reach goal is {}sec '.format(total_time_con))
print('Average velocity flock is {}m/s.'.format(avg_vel))
print('Average Compactness of flock is {}'.format(avg_compactness))
print('Final Inflmmation is', inflammation[-1])
