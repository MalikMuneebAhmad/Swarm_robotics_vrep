import sim as vrep  # V-rep library
import sys
import pickle
import time  # used to keep track of time
import psutil  # to find computation of CPU
import matplotlib.pyplot as plt  # Plotting library
import math  # math library
import random  # random number generation
from regulation_mechanism import Modc  # Regulation mechanism base on DC maturity
from scipy.signal import lfilter
from utlis import *

# e_puck_aggregation_ultrasonic V-rep file
# File include feature of robot aggregation avoiding obstacle with DC regulation mechanism

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

current_time = time.time()
#num_robots = 16  # to load robots in that script
#robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
with open('robot_file', 'rb') as f:  # to generate your own file  (rb for wrting in bytes)
    robots = pickle.load(f)  # storing variable into file
num_robots = len(robots)
start_time = [float(0.0)] * num_robots
execu_time = int(input('Enter Execution Time = '))  # Total time for Experiment in sec
min_safe_dist = 0.1  # Minimum safe distance for sensor
max_safe_dist = 0.2  # Maximum safe distance for sensor
sensor_range = 0.25  # Maximum Sensor range
max_avoid_dist = 0.2  # Distance from object at which object avoidance activate (front three sensors)
min_req_nei = 2  # minimum number of required neighboring robots
diff_rate = float(input('Enter Diffusion Rate = '))  # Total time for Experiment in sec  # Diffusion rate of danger
expe_total_inflam = num_robots * 3  # Each robot must have 3 neighboring robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
robot_handles = [robots[i].robot_handle for i in range(num_robots)]
rob_status = np.zeros([num_robots],
                      dtype='int')  # robot status like DC cell Status (Immature, semi mature, fully matured)
reg_mech = [Modc() for i in range(num_robots)]  # Creating object of regulation mechanisim of each robot
robots_position = [list()] * num_robots  # Position of robot
rob_dist = [list()] * num_robots  # Distance from axis of robot
rob_orien = [list()] * num_robots  # Orientation of robot
robots_det_rob = 0  # To calculate inflamation (no of robots detected by a robot)
infla = [1]  # inflamation at each iteration
comp_fitness = list()  # Spatial fitness calculation
danger_st_con = [False] * num_robots  # Condition to complete matured cell responce
semi_str = [False] * num_robots  # Condition to complete semi-matured cell responce
t = 0  # Current Iteration
rob_iter = np.zeros([num_robots], dtype='int')  # iteration of each robot
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)  # Automatically start simulation from V-rep
loop_start_time = time.time()  # Start measuring time
first_run = True
for i in range(num_robots):  # initially make velocity of each robot equal to zero
    robots[i].wait(0.0)
    robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(-1)
initial_robot_posx = [item[0] for item in robots_position]
initial_robot_posy = [item[1] for item in robots_position]
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
time_error, vrep_t = vrep.simxGetFloatSignal(clientID, "mySimulationTime", vrep.simx_opmode_streaming)
#------------------Main Loop-----------------#
while vrep_t < execu_time:  # Main loop
    print('waiting.............................................................')
    time_error, vrep_t = vrep.simxGetFloatSignal(clientID, "mySimulationTime", vrep.simx_opmode_buffer)
    print('Simulation time is ', vrep_t)
    print('Current Loop time ', current_time - loop_start_time)
    print('Iteration Number is ', t)
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    for i in range(num_robots):  # initially make velocity of each robot equal to zero
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(-1)
    for i in range(num_robots):  # measure control parameter for each robot
        rob_iter[i] += 1
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # sensor readings
        robots[i].sensor_mod_values(min_safe_dist, max_safe_dist)  # modify sensor readings
        robots[i].sensor_avoidance_values(max_avoid_dist, sensor_range)
        obj_theta = robots[i].object_avoidance(robots[i].static_object_handles, max_avoid_dist)  # check for object avoidance and calculation of angle
        all_grad, max_grad = robots[i].chemotaxis_gradient()  # gradient to move the robots
        print('Robot {} Detected Number of Robots is {}'.format(i, robots[i].det_rob))
        rob_status[i] = reg_mech[i].no_neigh_rob(min_req_nei, diff_rate, robots[i].det_rob)  # measure maturity of DC
        if not robots[i].obj_avoid:  # Prefer obstacle avoidance
            if (rob_status[i] == 3 and robots[i].det_rob) or danger_st_con[i]:  # DC is matured so move robot straight or rptate randomly(in next step)
                if not robots[i].detectionStates[6] and not danger_st_con[i]:  # Move backword
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(-1.8, 1.25)   # I changed here
                    danger_st_con[i] = True
                    # print('Robot {} is moving away because its status is {}'.format(i, rob_status[i]))
                    continue
                elif not robots[i].detectionStates[2] and not danger_st_con[i]:  # Move forward
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(1.8, 1.25)
                    danger_st_con[i] = True
                    # print('{} Robot {} is moving away because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    continue
                else:  # Will cause rotation after moving back
                    # print('{} Robot {} is rotating because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    rob_iter[i] = 0  # make iterration for a robot zero
                    max_grad = 0.0  # make max gradient zero to avoid specific rotation in next command
                    all_grad = np.zeros([4], dtype='float')
                    danger_st_con[i] = False  # Condition to regulate matured movement
                    robots[i].r_rot = True  # To rotate robot randomly rather than specifically
                    reg_mech[i].danger_nr = 0.0  # make danger zero
                # ----Action in case of semoi mature dcand not robots[i].det_rob)
            elif rob_status[i] == 2 and robots[i].det_rob and not semi_str[i] and rob_iter[i] >= 3:  # Dc is immatured and chech robot has a neighbor
                v_l[i], v_r[i], rot_t[i] = robots[i].control_param(
                    (math.pi / 4) * ((-1) ** random.randint(0, 1)), 1)  # to rotate robot perpadicular to neighbor robot
                semi_str[i] = True  # prepare for next step straight movment
                print('{} Robot {} semi-mature rotation movement because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                continue
            elif semi_str[i]:  # straight movement after rotation6
                rob_iter[i] = 0  # make iterration for a robot zero to reduce semi-mature action
                semi_str[i] = False  # Complete current step
                if not robots[i].detectionStates[6]:  # Move backword
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(-1.8, 1)
                    # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    continue
                elif not robots[i].detectionStates[2]:  # move forward
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(1.8, 1)
                    # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    continue
        rot_theta = robots[i].rotational_angle(all_grad, max_grad, obj_theta)
        # print('rotational angle is',rot_theta)
        v_l[i], v_r[i], rot_t[i] = robots[i].control_param(rot_theta, 1)
        # print('left velocity {} right velocity {} and time is {}'.format(v_l[i],v_r[i], rot_t[i]))
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
    robots_det_rob = 0
    for l in range(num_robots):  # Evaluate all neighboring robots at the end
        robots_det_rob = robots_det_rob + (len(robots[l].det_rob))
    infla.append(1 - robots_det_rob/expe_total_inflam)
    for i in range(num_robots):
        robots_position[i], rob_dist[i], rob_orien[i] = robots[i].get_position(-1)
        if first_run:
            initial_robot_posx = [item[0] for item in robots_position]
            initial_robot_posy = [item[1] for item in robots_position]
            first_run = False
    compactness_value, coord_com = fitness_aggregation(robots_position)
    comp_fitness.append(compactness_value)
    t += 1
vrep.simxPauseCommunication(clientID, False)
final_robot_posx = [item[0] for item in robots_position]
final_robot_posy = [item[1] for item in robots_position]
#vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
print('Final Compactness is = ' + str(compactness_value))
n = 15  # the larger n is, the smoother curve will be
b = [1.0 / n] * n
a = 1
mod_comp_fitness = lfilter(b, a, comp_fitness)
mod_infla_fitness = lfilter(b, a, infla)
plt.title('Inflamation Plot for {} Robots in '.format(num_robots))
plt.xlabel('Iteration #')
plt.ylabel('Inflamation')
plt.plot(list(range(t+1)), mod_infla_fitness)
plt.show()
#Plot of placement of food
plt.title('Initial and final Position of Robots'.format(num_robots))
plt.xlabel('Robot Position X-Coordinate')
plt.ylabel('Robot Position Y-Coordinate')
plt.scatter(initial_robot_posx, initial_robot_posy, color='r')
plt.scatter(final_robot_posx, final_robot_posy,  color='b')
plt.legend(["Initial Robot Position", "Final Robot Position"], loc ="upper right")
plt.show()
plt.title('Compactness Fitness for {} Robots'.format(num_robots))
plt.xlabel('Iteration #')
plt.ylabel('Compactness Fitness')
plt.plot(list(range(t)), mod_comp_fitness)
plt.show()
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))
