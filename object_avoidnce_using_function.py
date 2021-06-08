import sim as vrep  # V-rep library
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt
import random
# Object Avoidance of pioneer_3dx using function
# Only for single robot
# Little difficult to extend for multiple robot1

# --------Initialization of Variables-------#

maxDetectionDist = 0.6
front_maxdist = 0.7
theta = 0.0
t_t = 0.0
t = 0
vs = 1
sensor_handles = np.array([], dtype='i')
sensor_values = np.array([0.0] * 16)
exact_dist = np.array([0.0] * 16)
sensor_states = [False] * 16
sensors_position = np.zeros([16, 3])
sensors_position_vector = np.zeros([16, 2])
detectedpoint_vector = np.zeros([16, 3])
plt.xlim([-10, 10])
plt.ylim([-10, 10])
# --------------------Initial Code To Connect with V-REP--------------------#

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')
    returnCode = vrep.simxAddStatusbarMessage(clientID, 'Connected to remote API python', vrep.simx_opmode_oneshot)
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# -------------------get the handles of Ultrasonic sensors and motors-----------------#

errorCode, object_handle = vrep.simxGetObjectHandle(clientID, 'Cuboid4', vrep.simx_opmode_blocking)
# Robot Handle and its position initialization function
errorCode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx' + str(),
                                                   vrep.simx_opmode_oneshot_wait)
# print('Robot Handler', robot_handle)
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
# robot_position.pop()
returnCode, robot_position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_oneshot_wait)
print('Robot position', robot_position)
# robot_position.pop()


for x in range(1, 16 + 1):  # Sensor Handles Access
    errorCode, sensor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x),
                                                        vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor handles

    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)  # First call for Read ultrasonic Sensor

    returnCode, sensor_position = vrep.simxGetObjectPosition(clientID, sensor_handle, vrep.sim_handle_parent,
                                                             vrep.simx_opmode_oneshot_wait)  # Retrieving Sensor position w.r.t Robot
    # print("sensor {} position is {} ".format(x, sensor_position))
    sensors_position[x - 1] = sensor_position
    sensor_handles = np.append(sensor_handles, sensor_handle)
    # robot_angle = math.atan2(sensor_position[1], sensor_position[0])
    # robot_pos_length = math.sqrt(sensor_position[1]**2 + sensor_position[0]**2)

# sensors_position_vector = sensors_position + robot_position  # Sensor position w.r.t world frame of reference

# print("sensors_position_vector", sensors_position_vector)
# Find motors handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor#0',
                                                        vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor#0',
                                                         vrep.simx_opmode_oneshot_wait)

# -----------------------Functions------------------------#


def angles_calcu(sensor_x, sensor_y, robot_x, robot_y): # function for calculating gradient or angles
    grad_x = (sensor_x - robot_x)
    grad_y = (sensor_y - robot_y)
    angles = np.arctan2(grad_y, grad_x)
    return angles


def distance_calc(s, detectionState, detectedPoint, maxDetectionDist): # Calculate distances for ultrasonic Sensors
    sensor_states[s - 1] = detectionState
    if not sensor_states[s - 1]:  # To overcome out of bound distance problem
        detectedPoint = [0.0] * 3
    dist = np.linalg.norm(detectedPoint)  # Calculation of distance
    if dist > maxDetectionDist:
        sensor_values[s - 1] = maxDetectionDist
    elif dist <= maxDetectionDist:
        sensor_values[s - 1] = dist
    detectedpoint_vector[s - 1] = detectedPoint
    # print('detectedPoint', detectedPoint)
    exact_dist[s - 1] = dist
    return np.round(sensor_values, 2), np.round(exact_dist, 2), sensor_states


def object_avoidance(sensors_position, maxDetectionDist, sensor_values, sensor_exact_dist, front_maxdist, vs):  # Caculate motor parameters
    t_t = 0.0  # and time so that required angle of rotation will be achieved
    sensor_angles = angles_calcu(sensors_position[:, 0], sensors_position[:, 1], 0.0, 0.0)
    print('Sensor Angles', sensor_angles * 180 / math.pi)
    force_co_efficient = -(maxDetectionDist - sensor_values)
    fx = force_co_efficient * np.cos(sensor_angles)
    fy = force_co_efficient * np.sin(sensor_angles)
    fres_x = np.sum(fx)
    fres_y = np.sum(fy)
    if fres_x == 0.0 or fres_y == 0.0:
        theta = 0.1
    else:
        theta = math.atan2(fres_y, fres_x)
    print('Theta', theta)
    l = math.sqrt(fres_y ** 2 + fres_x ** 2)
    if l == 0:
        k1 = 1
    else:
        k1 = max((0.1, abs(theta)))
    w = (0.8 * abs(theta))  # w = r*theta
    v = k1 * vs

    if theta != 0.0 and (front_maxdist >= sensor_exact_dist[3] > 0.01 or front_maxdist >= sensor_exact_dist[4] > 0.01):
        print('Robot is taking turn')
        v_l = -v if theta < 0 else v
        v_r = -v_l
        if v == 0:
            t_t = 0.1
        elif t_t < 0.05:  # To avoid very small turn time
            t_t = random.random()/2
        else:
            t_t = w / (2 * v)
    else:
        print('Robot is moving straight')
        v_r = 0.5
        v_l = 0.5
        t_t = 1
    return v_r, v_l, t_t


# ---------------------Main Code for object Avoidance---------------------#


while t < 120:
    for s in range(1, 16 + 1):
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handles[s - 1], vrep.simx_opmode_oneshot_wait)  # Measure distance using Ultrasonic Sensor

        sensor_values, exact_dist, sensor_states = distance_calc(s, detectionState, detectedPoint, 0.8)
        print('exact distance', exact_dist)
    v_right, v_left, t_turn = object_avoidance(sensors_position, maxDetectionDist, sensor_values, exact_dist,
                                               front_maxdist, vs)
    print('v left', v_left)
    print('t_turn', t_turn)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, v_left, vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, v_right, vrep.simx_opmode_streaming)
    time.sleep(t_turn)
    t += 1