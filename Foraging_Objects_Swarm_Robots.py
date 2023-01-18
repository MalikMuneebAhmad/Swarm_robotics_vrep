import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import pickle
import math
import matplotlib.pyplot as plt
import matplotlib
from scipy.signal import lfilter, savgol_filter
from utlis import *
matplotlib.use('Qt5Agg')   # use Qt5 as backend, comment this line for default backend

#----------------9-11-2021--------------#
# Foraging behavior using chemotaxis gradient in obstacle environment has been implemented
# Vrep recommended file is "Foraging_behavior_e_puck_Swarm.ttt"
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
total_num_food = int(input("Defaut Number of food is 20, Enter total number of food ")) # number of food placed in srena
target_num_food = int(input("What percentsge of food do you want to collect =  "))  # (for one metric) after collecting that number loop will be terminated
target_num_food = int((target_num_food/100) * total_num_food)
exec_time = int(input("Enter Execution time = "))  # Execution Time
total_food_coll = 0  # Food collected by swarm at start is zero
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
min_safe_dist = 0.1  # Minimum safe distance for sensor
max_safe_dist = 0.15  # Maximum safe distance for sensor
sensor_range = 0.25  # Maximum Sensor range
max_avoid_dist = 0.2  # Distance from object at which object avoidance activate (front three sensors)
max_avoid_val = 0.15  # Value according to which sensors values are modefied to avoid obstacles
spat_fitness = [] # Compactness of Items
arb_food_pos = []
food_coll_time = []  # Collected food with respect time
real_time = []  # Real time duration
rob_dist = [list()] * num_robots  # dist of robots w.r.t frame of reference
rob_orien = [list()] * num_robots  # orientation of robots w.r.t frame of reference
all_rob_pick_food = np.array([False] * num_robots)  # Status of all robots having food or not
all_nest_info = [[] for _ in range(num_robots)]  # Robots containing nest information
food_handles = []  # Handles of all food items
food_items = [str(i) for i in range(total_num_food)]
all_nest_food = []  # handles of all food in nest
# -------- Von-Neumann neighborhood function parameters for robot monokines ------------#
mono_rob_val = 100
mono_rob_diff = 0.35
mono_rob_c1 = 0.15
mono_rob_c2 = 0.025
# -------- Von-Neumann neighborhood function parameters for food chemokines ------------#
chem_food_val = 100
chem_food_diff = 0.35
chem_food_c1 = 0.125
chem_food_c2 = 0.021
for i in food_items:  # Loop to acquire Handles of all objects
    errorCode, food_handle = vrep.simxGetObjectHandle(clientID, 'Food' + i, vrep.simx_opmode_blocking)
    food_handles.append(food_handle)
food_handles_plot = food_handles
final_foods_pos = []
food_handles = [i for i in food_handles if i != 0]
for food_handle in food_handles:  # loop to acquire position of food
    returnCode, food_pos = vrep.simxGetObjectPosition(clientID, food_handle, robots[0].ref_frame_handle,
                                                      vrep.simx_opmode_blocking)
    foods_pos.append(food_pos[:2])
initial_foods_pos = np.array(foods_pos)  # Initial food pos
avoid_food_handles = food_handles
arena_x, arena_y, food_pos_im = cartesian_im_trans(3.1, 3.1, 0.025, foods_pos)
chem_im = np.zeros((arena_x, arena_y))
mono_im = np.zeros((arena_x, arena_y))
time_consume = []  # time consumed in each iteration (implementation of control parameters)
inflammation = []  # sum of ir values of all robots for all iterations
energy_robot = [500 for _ in range(num_robots)]
total_energy_swarm = []
energy_unit_consum = 1  # Consumption of energy at each step
energy_addition = 200  # Amount of Energy added when picking objects or food
mySimulationTime = float()
t = 0
current_time = time.time()
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
time_error, vrep_t = vrep.simxGetFloatSignal(clientID, "mySimulationTime", vrep.simx_opmode_streaming)
#while total_food_coll < target_num_food:
    #current_time - loop_start_time
while vrep_t < exec_time:  # Main loop
    print('waiting.............................................................')
    time_error, vrep_t = vrep.simxGetFloatSignal(clientID, "mySimulationTime", vrep.simx_opmode_buffer)
    print('Simulation time is ', vrep_t)
    print('Current Loop time ', current_time - loop_start_time)
    print('Iteration Number is ', t)
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    #----------------Calculate inflammation in each iteration------------#
    for i in range(num_robots):
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(robots[0].ref_frame_handle)  # Current orienation and position w.r.t frame of reference
        all_rob_pick_food[i] = robots[i].food_picked
        all_nest_info[i] = robots[i].nest_info  # Update robots nest info
    arena_x, arena_y, rob_pos_im = cartesian_im_trans(3.1, 3.1, 0.025, robots_position)  # Convert robots pos in pixels
    rob_pos_im_mod = rob_pos_im[all_rob_pick_food == False]  # Only free robots will induce monokines
    #print('Robot Orientations are', np.array(rob_orien) * (180/math.pi))
    mono_im = chemotaxis_gradient_forging(mono_im, rob_pos_im_mod, 'von_neumann', mono_rob_c1, mono_rob_c2, mono_rob_diff, mono_rob_val)  # Updating mono-taxis mask
    print('mono im max', np.max(mono_im))
    chem_im = chemotaxis_gradient_forging(chem_im, food_pos_im, 'von_neumann', chem_food_c1, chem_food_c2, mono_rob_diff, chem_food_val)  # Updating chemo-taxis mask
    #----------Loop to measure control parameter for each robot----------#
    for i in range(num_robots):
        print('Robot number = ', i)
        energy_robot[i] -= energy_unit_consum  # Reduce energy at each step
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # Extract raw values from sensors
        # print('Detection- -----States1', det_state1)
        vision_values = robots[i].read_vision_sensor()  # reading values from vision sensors
        front_ir_value, front_ir_status = robots[i].front_vision_sensor()
        robots[i].sensor_mod_values(min_safe_dist, max_safe_dist)  # modified sensor values for virtual forces
        robots[i].sensor_avoidance_values(max_avoid_val, sensor_range)  # modified sensor values for object avoidance
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
        avoid_obj_nest = robots[i].static_object_handles + robot_handles + all_nest_food
        # -----------------Random levy search in black area-------------#
        if (chem_avg_value < 0.0001 and not bool(robots[i].nest_info)) or nest_search:  # threshold to check robot is in gradient of food or looking for nest
            obj_theta = robots[i].object_avoidance(avoidable_obj, max_avoid_dist)  # obj avoidance will be for robots and objects
            if not robots[i].obj_avoid:
                levy_theta = robots[i].random_rotation_angle()
                if levy_theta == 0.0:
                    v_l[i], v_r[i], rot_t[i] = robots[i].levy_flight(2.5, 1)
                    print('Robot levy movement')
                    continue
                else:
                    # print('Robot {} is rotating in levy search '.format(m))
                    v_l[i], v_r[i], rot_t[i] = robots [i].control_param(levy_theta, 1.5)
                    continue
            else:  # set parameters for object avoidance during levy searching
                v_l[i], v_r[i], rot_t[i] = robots[i].control_param(obj_theta, 1.5)
                continue
        #vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(vision_values)
        #vision_max_grad = clamp(vision_max_grad, 0.1, 0.25)
        #----------------Robot in gradient of food-------------------#
        obj_theta = robots[i].object_avoidance(avoidable_obj, max_avoid_dist)
        rot_theta = obj_theta
        picked_food = list(set(robots[i].det_obj_handles[1:4]).intersection(set(food_handles)))
        #nest_food_found = list(set(robots[i].det_obj_handles[1:4]).intersection(set(all_nest_food)))
        nest_food_found = True if robots[i].det_obj_handles[2] in all_nest_food else False
        #pick_cond = robots[i].det_obj_handles[2] in food_handles
        if picked_food and not robots[
            i].food_picked:  # Pick food from Arena, when robot does not have food
            print('Food is found by robot')
            robots[i].pick_food_object(picked_food[0])  # other method robots[i].det_obj_handles[2]
            energy_robot[i] += energy_addition  # Increase in energy of robots when find food
            index_cap_food = food_handles.index(picked_food[0])  # find index of captured food handle
            #all_nest_food.append(picked_food[0])  # Update nest food handles
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
                if (avg_ir_value > 0.65 and grad_ir_nest > 0) or nest_food_found:
                    print('Food is dropped--------------------------------------')
                    all_nest_food.append(robots[i].picked_food_handle)
                    robots[i].drop_food_object([0, 0.1, 0.15], 1)  # 1 define foraging, 0 will be for clustering
                    robots[i].nest_info = robots_position[i]  # update nest info of a robot who reached at nest
                    prev_cal_angle[i].clear()
                    prev_cal_angle[i].append(0.75 * math.pi)
                    prev_cal_mag[i].clear()
                    #  Move robot 180 degree to move away
                continue
            elif leave_nest:  # move away  nest, to search new food
                print('Moving out from nest')
                vision_max_grad, vision_grad_theta = robots[i].vision_sensor_gradient(-vision_values / 2)
                vision_max_grad = clamp(- vision_max_grad / 2, 0.1, 0.25)
                if robots[i].det_obj_handles[2] in all_nest_food:
                    obj_theta = robots[i].object_avoidance(avoid_obj_nest, 0.15)
                    vision_max_theta = obj_theta
                prev_cal_angle[i].append(vision_grad_theta)  # update variable of previous angle
                prev_cal_mag[i].append(vision_max_grad)  # update variable of previous  magnitude
                continue
            all_grad, max_grad = chemotaxis_gradient_foraging(chem_im, mono_im, rob_pos_im[i], 1, 0.001)
            chemo_theta = robots[i].rotational_angle(all_grad, max_grad, obj_theta)  #  Require diection of Robot calculated by chemotaxis
            rot_theta = ((chemo_theta - rob_orien[i]) + math.pi) % (2 * math.pi) - math.pi
            #rot_theta = obj_theta if robots[i].obj_avoid else rot_theta  #  Avoid object of nest
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
    arb_food_pos = []
    '''for food_handle in food_handles_plot:  # loop to acquire position of food
        returnCode, food_pos = vrep.simxGetObjectPosition(clientID, food_handle, robots[0].ref_frame_handle,
                                                          vrep.simx_opmode_blocking)
        arb_food_pos.append(food_pos[:2])
    comp_food, com_food = fitness_aggregation(arb_food_pos)
    spat_fitness.append(comp_food)'''
    time_consume.append(final_time - init_time)  # total time consumed for each implementation of control parameters
    total_energy_swarm.append(sum(energy_robot))  # Sum energy of whole swarm at any time
    total_food_coll = len(all_nest_food)  # no of food collected uptill now
    food_coll_time.append(total_food_coll)
    real_time.append(vrep_t)
    foraging_time = current_time - loop_start_time
    t += 1
    if t == 1:
        plot1 = plt.figure(1)
        plt.title('Initial Food Chemokine Image'.format(num_robots))
        plt.xlabel('Y-axis')
        plt.ylabel('X-axis')
        plt.imshow(np.transpose(chem_im), origin='lower')
        plt.show()
for food_handle in food_handles_plot:  # loop to acquire position of food
    returnCode, food_pos = vrep.simxGetObjectPosition(clientID, food_handle, robots[0].ref_frame_handle,
                                                      vrep.simx_opmode_blocking)
    arb_food_pos.append(food_pos[:2])
comp_food, com_food = fitness_aggregation(arb_food_pos)
spat_fitness.append(comp_food)
vrep.simxPauseCommunication(clientID, False)
#time.sleep(5)
#vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
print('Final Compactness of Robots {} after foraging.'.format(comp_food))
print('Swarm Consumed {} sec to foraged {} items in number of iterations {}.'.format(vrep_t, total_food_coll, t))
print('Total energy of Swarm is {} and Compactness of Food is {}'.format(total_energy_swarm[-1], comp_food))
if target_num_food in food_coll_time:
    print('80% of food is collected in time {} and Total number of iterations are {}'.format(real_time[food_coll_time.index(target_num_food)], t))
else:
    print('Target is not Achieved')
final_foods_pos = np.array(arb_food_pos)
plot2 = plt.figure(2)
plt.title('Food Location Plot for {} robots'.format(num_robots))
plt.xlabel('Y-axis')
plt.ylabel('X-axis')
plt.imshow(np.transpose(chem_im), origin='lower')
plt.show()
#Plot of energy at each time of swarm
plot3 = plt.figure(3)
plt.title('Energy of Swarm of {} Robots'.format(num_robots))
plt.xlabel('Iteration')
plt.ylabel('Energy of Swarm')
plt.plot(list(range(t)), total_energy_swarm, color='b')
plt.show()
#Plot of placement of food
plot4 = plt.figure(4)
plt.title('Initial and final Location of Items '.format(num_robots))
plt.xlabel('X-axis')
plt.ylabel('y-axis')
plt.scatter(initial_foods_pos[:, 0], initial_foods_pos[:, 1], color='r')
plt.scatter(final_foods_pos[:, 0], final_foods_pos[:, 1],  color='b')
plt.legend(["Initial Position", "Final Position"], loc ="upper right")
plt.show()
# Compactness of Food Itemes
plot5 = plt.figure(5)
plt.title('Compactness of Items'.format(num_robots))
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.plot(list(range(len(spat_fitness[1:]))), spat_fitness[1:])
plt.show()
# Collected food with reso=pest to real time plot4 = plt.figure(4)
#mod_food_coll_time = savgol_filter(food_coll_time, 5, 2)
food_coll_inter = coll_items(total_num_food, total_food_coll,food_coll_time, real_time)
n = 15  # the larger n is, the smoother curve will be ratio_rob_agg
b = [1.0 / n] * n
a = 1
food_coll_time = lfilter(b, a, food_coll_time)
plot6 = plt.figure(6)
plt.title('Collected Number of Items for N = {} and $V_f$ = {} with respect to time '.format(num_robots, total_num_food))
plt.xlabel('Real Time (sec)')
plt.ylabel('Collected Number of Food')
plt.plot(real_time, food_coll_time)
plt.show()
