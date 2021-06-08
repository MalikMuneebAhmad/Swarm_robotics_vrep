import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt
import random
import psutil


def angles_calcu(sensor_x, sensor_y, robot_x, robot_y):  # function for calculating gradient or angles
    grad_x = (sensor_x - robot_x)
    grad_y = (sensor_y - robot_y)
    angles = np.arctan2(grad_y, grad_x)
    return angles


class Robot:

    def __init__(self, clientID, robot_name, sensor_name, motor_name):
        self.clientID = clientID
        self.robot_name = robot_name
        self.sensor_name = sensor_name
        self.motor_name = motor_name
        self.num_sensors = int(16)
        self.sensor_handles = np.array([], dtype='i')
        self.detectionStates = [False] * self.num_sensors
        self.sensors_position = np.zeros([self.num_sensors, 3])
        self.sensor_values = np.array([0.0] * self.num_sensors)
        self.sensor_raw_values = np.array([0.0] * self.num_sensors)
        self.sensor_angles = np.array([])

        # self.detectedPoint = np.zeros([16, 3])

        # -------------------get the handles of Ultrasonic sensors and motors-----------------#
        # Robot Handle and its position initialization function
        self.errorCode, self.robot_handle = vrep.simxGetObjectHandle(clientID, self.robot_name,
                                                                     vrep.simx_opmode_oneshot_wait)
        self.returnCode, self.robot_position = vrep.simxGetObjectPosition(clientID, self.robot_handle, -1,
                                                                          vrep.simx_opmode_streaming)
        # Get robot position
        self.returnCode, self.robot_position = vrep.simxGetObjectPosition(clientID, self.robot_handle, -1,
                                                                          vrep.simx_opmode_oneshot_wait)

        print('Execution1')
        for x in range(1, self.num_sensors + 1):  # Sensor Handles Access
            print('Execution1a')
            self.errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, self.sensor_name + str(x),
                                                                     vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles
            self.sensor_handles = np.append(self.sensor_handles, sensor_handle)
            self.errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                self.clientID, sensor_handle, vrep.simx_opmode_streaming)  # First call for Read ultrasonic Sensor

            self.returnCode, sensor_position = vrep.simxGetObjectPosition(clientID, self.sensor_handles[x - 1],
                                                                          vrep.sim_handle_parent,
                                                                          vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor position w.r.t Robot
            self.sensors_position[x - 1] = sensor_position
        self.sensor_angles = angles_calcu(self.sensors_position[:, 0], self.sensors_position[:, 1], 0.0, 0.0)
        print('sensor_angles', self.sensor_angles)
        self.errorCode, self.left_motor_handle = vrep.simxGetObjectHandle(clientID,
                                                                          'Pioneer_p3dx_left' + self.motor_name,
                                                                          vrep.simx_opmode_oneshot_wait)
        # Find motors handles
        self.errorCode, self.right_motor_handle = vrep.simxGetObjectHandle(clientID,
                                                                           'Pioneer_p3dx_right' + self.motor_name,
                                                                           vrep.simx_opmode_oneshot_wait)
        print('Executed')

    def ultrasonic_values(self, maxDetectionDist):  # Calculate distances for ultrasonic Sensors
        for s in range(1, self.num_sensors + 1):
            self.errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                self.clientID, self.sensor_handles[s - 1],
                vrep.simx_opmode_oneshot_wait)  # Measure distance using Ultrasonic Sensor
            self.detectionStates[s - 1] = detectionState
            if not detectionState:  # To overcome out of bound distance problem
                detectedPoint = [0.0] * 3
            dist = np.linalg.norm(detectedPoint)  # Calculation of distance
            if dist > maxDetectionDist:
                self.sensor_values[s - 1] = maxDetectionDist
            elif dist <= maxDetectionDist:
                self.sensor_values[s - 1] = dist
            # print('detectedPoint', detectedPoint)
            self.sensor_raw_values[s - 1] = dist
        return np.round(self.sensor_values, 2), np.round(self.sensor_raw_values, 2), self.detectionStates

    def object_avoidance(self, maxDetectionDist, front_maxdist, vs):  # Caculate motor parameters
        t_t = 0.0  # and time so that required angle of rotation will be achieved
        force_co_efficient = -(maxDetectionDist - self.sensor_values)
        fx = force_co_efficient * np.cos(self.sensor_angles)
        fy = force_co_efficient * np.sin(self.sensor_angles)
        fres_x = np.sum(fx)
        fres_y = np.sum(fy)
        if fres_x == 0.0 or fres_y == 0.0:
            theta = 0.1
        else:
            theta = math.atan2(fres_y, fres_x)
        # print('Theta', theta)
        l = math.sqrt(fres_y ** 2 + fres_x ** 2)
        if l == 0:
            k1 = 1
        else:
            k1 = max((0.1, abs(theta)))
        w = (0.8 * abs(theta))  # w = r*theta
        v = k1 * vs
        if theta != 0.0 and (
                front_maxdist >= self.sensor_raw_values[3] > 0.01 or front_maxdist >= self.sensor_raw_values[4] > 0.01):
            print('Robot is taking turn')
            v_l = -v if theta < 0 else v
            v_r = -v_l
            if v == 0:
                t_t = 0.1
            elif t_t < 0.05:  # To avoid very small turn time
                t_t = random.random() / 2
            else:
                t_t = w / (2 * v)
        else:
            print('Robot is moving straight')
            v_r = 0.5
            v_l = 0.5
            t_t = 1
        return v_r, v_l, t_t

    def movement(self, v_left, v_right, t_turn):
        self.errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor_handle, v_left,
                                                         vrep.simx_opmode_streaming)
        self.errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor_handle, v_right,
                                                         vrep.simx_opmode_streaming)
        time.sleep(t_turn)


t = 0

vrep.simxFinish(-1)  # just in case, close all opened connections
ID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
type(ID)
if ID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(ID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

robot1 = Robot(ID, 'Pioneer_p3dx0', 'Pioneer_p3dx_ultrasonicSensor', 'Motor#0')
robot2 = Robot(ID, 'Pioneer_p3dx1', 'Pioneer_p3dx_ultrasonicSensor1#', 'Motor#1')
robot3 = Robot(ID, 'Pioneer_p3dx2', 'Pioneer_p3dx_ultrasonicSensor2#', 'Motor#2')


while t < 30:
    robot1.ultrasonic_values(0.7)
    vr, vl, tt = robot1.object_avoidance(0.7, 0.8, 1)
    robot1.movement(vl, vr, tt)
    robot2.ultrasonic_values(0.7)
    vr1, vl1, tt1 = robot2.object_avoidance(0.7, 0.8, 1)
    robot2.movement(vl1, vr1, tt1)
    robot3.ultrasonic_values(0.7)
    vr2, vl2, tt2 = robot2.object_avoidance(0.7, 0.8, 1)
    robot2.movement(vl2, vr2, tt2)
    print('t', t)
    t += 1
print('System CPU load is {} %'.format(psutil.cpu_percent(interval=0.5)))
