from robot_epuck import Robot
import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import psutil
import datetime
import random



vrep.simxFinish(-1)  # just in case, close all opened connections
ID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
type(ID)
if ID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(ID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')


#vrep.simxStopSimulation(ID, vrep.simx_opmode_oneshot)
#robot0 = Robot(ID, 'ePuck#0', 'ePuck_proxSensor#0')
#robot1 = Robot(ID, 'ePuck#1', 'ePuck_proxSensor#1')
#robot2 = Robot(ID, 'ePuck#1', 'ePuck_proxSensor#1')'''

#returnCode, baseHandle = vrep.simxLoadModel(ID, 'D:\Python_Vrep\e_puck_aggregation_ultrasonic.ttm', bool(0),  vrep.simx_opmode_blocking)
#print('return code', returnCode)
robots = [Robot(ID, 'ePuck#' + str(num), 'ePuck_proxSensor#' + str(num)) for num in range(10)]
x = datetime.datetime(2021, 5, 31, 17, 47, 0, 0) - datetime.datetime(2021, 5, 31, 17, 46, 0, 0)
first_time = datetime.datetime.now()
t = 0
num_robots = 10

start_time = [datetime.datetime.now()] * num_robots
sleep_time = [datetime.datetime.now()] * num_robots
v_l1 = [0.0] * num_robots
v_r1 = [0.0] * num_robots
rot_t1 = [0.0] * num_robots
current_time = datetime.datetime.now()
#vrep.simxSynchronous(ID, True)
print('Started Execution time', datetime.datetime.now()-first_time)
vrep.simxStartSimulation(ID, vrep.simx_opmode_blocking)
sim_start_time = datetime.datetime.now()
while current_time - sim_start_time < x:
    for i in range(num_robots):
        sensor_raw1, det_state1 = robots[i].ultrasonic_values()
        # print('sensor_raw0', sensor_raw1)
        robots[i].sensor_chemo(0.15, 0.20)
        theta1, det1 = robots[i].chemotaxis_gradient()
        v_l1[i], v_r1[i], rot_t1[i] = robots[i].change_direction(theta1)
    for i in range(num_robots):
        current_time = datetime.datetime.now()
        diff = (current_time - start_time[i]).total_seconds()
        if diff >= rot_t1[i]:
            print('For robot {} Target time is {} and Passed time is {}'.format(i, rot_t1[i],
                                                                                diff))
            #print('Execution of Code', robots[i].robot_name)
            robots[i].wait(0.0)
            robots[i].movement(v_l1[i], v_r1[i])
            start_time[i] = datetime.datetime.now()
            #sleep_time[i] = start_time[i] + datetime.timedelta(days=0, seconds=0, microseconds=0, milliseconds=rot_t1, minutes=0, hours=0, weeks=0)
            #sleep_time[i] =
    t += 1
print('Ending Time', datetime.datetime.now() - first_time)
vrep.simxStopSimulation(ID, vrep.simx_opmode_blocking)
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))
