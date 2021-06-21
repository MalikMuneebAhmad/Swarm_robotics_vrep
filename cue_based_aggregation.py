import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import psutil
from regulation_mechanism import Modc
from robot_epuck import Robot
import numpy as np  # array library
import matplotlib.pyplot as plt


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
num_robots = 15
robots = [Robot(clientID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(num_robots)]
start_time = [float(0.0)] * num_robots
v_l = [float(0.0)] * num_robots
v_r = [float(0.0)] * num_robots
rot_t = [float(0.0)] * num_robots
rob_status = np.zeros([num_robots])
reg_mech = [Modc() for i in range(num_robots)]
robots_position = list()
robots_det_rob = 0
infla = list()
danger_st_con = [False] * num_robots
t = 0
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
loop_start_time = time.time()
while current_time - loop_start_time < 300:
    rob = list(range(num_robots))
    for i in range(num_robots):
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()
        # print('Detection------States1', det_state1)
        robots[i].sensor_mod_values(0.125, 0.175)
        theta1 = robots[i].object_avoidance(0.2)
        all_grad, max_grad = robots[i].chemotaxis_gradient()
        rob_status[i] = reg_mech[i].no_neigh_rob(2, 0.08, robots[i].det_rob)
        sensor_raw = [False if robots[i].sensor_raw_values[j] < 0.05 else True for j in range(8)]
        ir_value = robots[i].Vision_sensor
        print('IR value', ir_value)
        #------------movement in case of danger will be produced here
        #if danger_st_con[i] and (not robots[i].obj_avoid) and (sensor_raw[2] or sensor_raw[6]) : # and not (0.03 in robots[i].sensor_raw_values):
            #danger_st_con[i] = not danger_st_con[i]
            #v_l[i], v_r[i], rot_t[i] = robots[i].random_straight()
            #break
        if (rob_status[i] == 3 and robots[i].theta == 0.0) or danger_st_con[i] and not robots[i].obj_avoid:
            if robots[i].detectionStates[2]:  # MOve backword
                v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(-1)
                danger_st_con[i] = True
                break
            elif robots[i].detectionStates[6]: # move forward
                v_l[i], v_r[i], rot_t[i] = robots[i].random_straight(1)
                danger_st_con[i] = True
                break
            else:
                max_grad = 0.0
                all_grad = np.zeros([4], dtype='float')
                danger_st_con[i] = False
                reg_mech[i].danger_nr = 0.0
                robots[i].r_rot = True
        robots[i].rotational_angle(all_grad, max_grad)
        v_l[i], v_r[i], rot_t[i] = robots[i].control_param()
    for j in range(num_robots):
        robots[j].movement(v_l[j], v_r[j])
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
    robots_det_rob = 0
    for l in range(num_robots):  # Evaluate all positions of robots at the end
        robots_det_rob = robots_det_rob + (len(robots[l].det_rob))
    infla.append(1 - robots_det_rob/45)
    t = t + 1

#vrep.simxPauseCommunication(clientID, False)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
plt.title('Inflamation Plot for 15 Robots')
plt.xlabel('Iteration #')
plt.ylabel('Inflamation')
plt.plot(list(range(t)), infla)
plt.show()
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))