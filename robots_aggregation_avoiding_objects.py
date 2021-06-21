import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import psutil  # to find computation of CPU
import numpy as np  # array library
import matplotlib.pyplot as plt  # Plotting library
import math  # math library
import random  # random number generation
from regulation_mechanism import Modc  # Regulation mechanism base on DC maturity
from robot_epuck import Robot  # Robot library


# ----------------My Functions----------------#

def center_mass_dist(rob_position):  # function for calculating distances of mass from center of mass
    x_sum = np.sum(rob_position[:, 0]) / len(rob_position)
    y_sum = np.sum(rob_position[:, 1]) / len(rob_position)
    cen_mass = np.array([x_sum, y_sum])
    diff = rob_position - cen_mass
    summa = (diff * diff)
    dist_robots = np.sqrt(summa.sum(axis=1))
    return dist_robots


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


# spatial inter-robot relationships
def spatial_fitness(dist_rob):  # distance of robots from center of mass
    dist_rob = dist_rob/np.max(dist_rob)
    dist_rob = (1 - dist_rob)/len(dist_rob)
    fitness = np.sum(dist_rob)
    return fitness

# -------------------------6-6-2021----------------------#
# Robot aggregation with Aggregation_back_surface, Aggregation_robots_0.3ultrasonic working fine with
# e_puck_aggregation_ultrasonic V-rep file
# File include feature of robot aggregation avoiding obstacle with DC regulation mechanism


vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

current_time = time.time()
num_robots = 5
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots  # left wheel angular velocity
v_r = [float(0.0)] * num_robots  # right wheel angular velocity
rot_t = [float(0.0)] * num_robots  # Time to sleep
robot_handles = [robots[i].robot_handle for i in range(num_robots)]
rob_status = np.zeros([num_robots],
                      dtype='int')  # robot status like DC cell Status (Immature, semi mature, fully matured)
reg_mech = [Modc() for i in range(num_robots)]  # Creating object of regulation mechanisim of each robot
robots_position = np.zeros((num_robots, 2))  # Position of robot
robots_det_rob = 0  # To calculate inflamation (no of robots detected by a robot)
infla = list()  # inflamation at each iteration
danger_st_con = [False] * num_robots  # Condition to complete matured cell responce
semi_str = [False] * num_robots  # Condition to complete semi-matured cell responce
t = 0  # Current Iteration
rob_iter = np.zeros([num_robots], dtype='int')  # iteration of each robot
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)  # Automatically start simulation from V-rep
loop_start_time = time.time()  # Start measuring time
for i in range(num_robots):  # initially make velocity of each robot equal to zero
    # print('Object type of robot', vrep.simxGetObjectGroupData()bject
    robots[i].wait(0.0)
# robots[0].set_position([0,0,0])
# returnCode, robot_handles,intData,floatData, stringData = vrep.simxGetObjectGroupData(clientID, vrep.sim_object_visionsensor_type, 4,  vrep.simx_opmode_blocking)
while current_time - loop_start_time < 10:  # Main loop
    rob = list(range(num_robots))  # To run the execution in third loop (while)
    for i in range(num_robots):  # measure control parameter for each robot
        rob_iter[i] += 1
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()  # sensor readings
        robots[i].sensor_mod_values(0.1, 0.2)  # modify sensor readings
        obj_theta = robots[i].object_avoidance(0.2)  # check for object avoidance and calculation of angle
        all_grad, max_grad = robots[i].chemotaxis_gradient()  # gradient to move the robots
        rob_status[i] = reg_mech[i].no_neigh_rob(2, 0.08, robots[i].det_rob)  # measure maturity of DC
        if not robots[i].obj_avoid:  # Prefer obstacle avoidance
            if (rob_status[i] == 3 and robots[i].det_rob) or danger_st_con[
                i]:  # DC is matured so move robot straight or rptate randomly(in next step)
                if not robots[i].detectionStates[6] and not danger_st_con[i]:  # Move backword
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(-1, 2)
                    danger_st_con[i] = True
                    # print('Robot {} is moving away because its status is {}'.format(i, rob_status[i]))
                    break
                elif not robots[i].detectionStates[2] and not danger_st_con[i]:  # Move forward
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(1, 2)
                    danger_st_con[i] = True
                    # print('{} Robot {} is moving away because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    break
                else:  # Will cause rotation after moving back
                    # print('{} Robot {} is rotating because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    rob_iter[i] = 0  # make iterration for a robot zero
                    max_grad = 0.0  # make max gradient zero to avoid specific rotation in next command
                    all_grad = np.zeros([4], dtype='float')
                    danger_st_con[i] = False  # Condition to regulate matured movement
                    robots[i].r_rot = True  # To rotate robot randomly ratber than specifically
                    reg_mech[i].danger_nr = 0.0  # make danger zero
                # ----Action in case of semoi mature dcand not robots[i].det_rob)
            elif rob_status[i] == 2 and robots[i].det_rob and not semi_str[i] and rob_iter[
                i] >= 3:  # Dc is immatured and chech robot has a neighbor
                v_l[i], v_r[i], rot_t[i] = robots[i].control_param(
                    (math.pi / 2) * ((-1) ** random.randint(0, 1)))  # to rotate robot perpadicular to neighbor robot
                semi_str[i] = True  # prepare for next step straight movment
                print('{} Robot {} semi-mature rotation movement because its status is {}'.format(rob_iter[i], i,
                                                                                                  rob_status[i], ))
                break
            elif semi_str[i]:  # straight movement after rotation
                rob_iter[i] = 0  # make iterration for a robot zero to reduce semi-mature action
                semi_str[i] = False  # Complete current step
                if not robots[i].detectionStates[6]:  # Move backword
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(-1, 1)
                    # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    break
                elif not robots[i].detectionStates[2]:  # move forward
                    v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(1, 1)
                    # print('{} Robot {} semi-mature straight movement because its status is {}'.format(rob_iter[i], i, rob_status[i]))
                    break
        rot_theta = robots[i].rotational_angle(all_grad, max_grad, obj_theta)
        # print('rotational angle is',rot_theta)
        v_l[i], v_r[i], rot_t[i] = robots[i].control_param(rot_theta)
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
    infla.append(1 - robots_det_rob / 45)
    t = t + 1
vrep.simxPauseCommunication(clientID, False)
# --------------Get Robot positions--------------#
# returnCode, robot_handles,intData,floatData, stringData = vrep.simxGetObjectGroupData(clientID, vrep.sim_object_visionsensor_type, 4,  vrep.simx_opmode_blocking)
# print('returnCode', returnCode)
# print('position of robots', floatData)
for i in range(num_robots):
    robots_position[i] = robots[i].get_position()
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
avg_dis = center_mass_dist(robots_position)  # average distance of robots from the center of mass of the swarm
plt.title('Inflamation Plot for 15 Robots')
plt.xlabel('Iteration #')
plt.ylabel('Inflamation')
plt.plot(list(range(t)), infla)
plt.show()
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))
